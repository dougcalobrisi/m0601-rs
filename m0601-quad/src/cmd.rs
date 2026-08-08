//! Subcommand implementations. Ordered like the bring-up sequence:
//! `check` (no port) → `check --probe` (read-only) → `monitor` (no
//! motion) → `jog`/`calibrate` (one wheel, bounded) → `drive` (the TUI).

use std::error::Error;
use std::io::Write as _;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::sync_channel;
use std::time::{Duration, Instant};

use crate::clock::RealClock;
use crate::config::{Config, End, Side, WheelCfg};
use crate::io::SimWheel;
use crate::logger;
use crate::pilot::{self, Pilot};
use crate::rover::{self, StopGuard};
use crate::state::{Shared, lock};
use crate::ui::{self, WheelInfo};

type CmdResult = Result<(), Box<dyn Error>>;

/// `check`: validate, print the resolved table; `--probe` additionally
/// opens the port and asks each wheel to answer inside the configured
/// reply window.
pub fn check(cfg: &Config, probe: bool, shared: &Arc<Shared>) -> CmdResult {
    println!("config OK: 4 wheels on {}", cfg.bus.port);
    println!(
        "cycle {:.1} ms ({:.1} Hz per wheel), min gap {:.1} ms, reply wait {:.1} ms",
        cfg.bus.cycle_ms,
        1000.0 / cfg.bus.cycle_ms,
        cfg.bus.min_gap_ms,
        cfg.bus.reply_wait_ms,
    );
    println!();
    println!("  corner       id    name           mirrored  invert  effective");
    for w in cfg.wheels_in_grid_order() {
        println!(
            "  {:<12} 0x{:02X}  {:<14} {:<9} {:<7} {}",
            w.corner(),
            w.id,
            w.name,
            if w.mirrored { "yes" } else { "no" },
            if w.invert { "yes" } else { "no" },
            if w.reversed() { "REVERSED" } else { "forward" },
        );
    }

    if !probe {
        return Ok(());
    }

    println!();
    let rover = rover::open(cfg)?;
    println!(
        "port open; kernel low-latency: {}",
        if rover.low_latency {
            "yes"
        } else {
            "NO — see the udev rule in USAGE.md"
        }
    );
    if let Some(timer) = read_latency_timer(&cfg.bus.port) {
        println!(
            "adapter latency_timer: {timer} ms{}",
            if timer > 2 { "  [!] want 1" } else { "" }
        );
    }
    let mut wheels = rover.wheels;
    for (i, w) in wheels.iter_mut().enumerate() {
        if !shared.running.load(Ordering::Relaxed) {
            break;
        }
        let cfg_wheel = cfg.wheels_in_grid_order()[i].corner();
        let frame = m0601::protocol::frame_feedback(w.id());
        let start = Instant::now();
        match w.transact(&frame, cfg.reply_wait())? {
            Some(fb) => println!(
                "  {:<12} 0x{:02X}  replied in the {:.1} ms window ({:.1} ms total)  {:+} RPM  {}",
                cfg_wheel,
                fb.id,
                cfg.bus.reply_wait_ms,
                start.elapsed().as_secs_f64() * 1000.0,
                fb.speed_rpm,
                fb.faults,
            ),
            None => match w.query()? {
                // The long-window retry separates "wrong timing budget"
                // from "wheel absent" — different fixes.
                Some(_) => println!(
                    "  {:<12} 0x{:02X}  [!] silent in {:.1} ms but answered a 150 ms window — \
                     raise reply_wait_ms or fix adapter latency",
                    cfg_wheel,
                    w.id(),
                    cfg.bus.reply_wait_ms,
                ),
                None => println!(
                    "  {:<12} 0x{:02X}  SILENT — check power, wiring, id",
                    cfg_wheel,
                    w.id()
                ),
            },
        }
    }
    Ok(())
}

fn read_latency_timer(port: &str) -> Option<u32> {
    let tty = port.rsplit('/').next()?;
    let path = format!("/sys/bus/usb-serial/devices/{tty}/latency_timer");
    std::fs::read_to_string(path).ok()?.trim().parse().ok()
}

/// `monitor`: headless round-robin telemetry + CSV. **Commands no
/// motion** — the first thing to run on a freshly wired rover.
pub fn monitor(cfg: &Config, shared: &Arc<Shared>) -> CmdResult {
    let rover = rover::open(cfg)?;
    let mut wheels = rover.wheels;
    let names: Vec<String> = cfg
        .wheels_in_grid_order()
        .iter()
        .map(|w| w.corner())
        .collect();

    let mut csv = match &cfg.log {
        Some(log) => {
            let file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log.path)?;
            let fresh = file.metadata().map(|m| m.len() == 0).unwrap_or(false);
            let mut out = std::io::BufWriter::new(file);
            if fresh {
                writeln!(out, "{}", logger::CSV_HEADER)?;
            }
            Some(out)
        }
        None => None,
    };

    println!(
        "monitoring {} wheels on {} — Ctrl-C to stop",
        wheels.len(),
        cfg.bus.port
    );
    let mut status: Vec<String> = names.iter().map(|n| format!("{n} --")).collect();
    while shared.running.load(Ordering::Relaxed) {
        for (i, w) in wheels.iter_mut().enumerate() {
            if !shared.running.load(Ordering::Relaxed) {
                break;
            }
            match w.poll_query(cfg.reply_wait()) {
                Ok(Some(fb)) => {
                    status[i] = format!(
                        "{} {:+4} RPM {:+.2} A {} C {}",
                        names[i],
                        fb.speed_rpm,
                        fb.current_a,
                        fb.temp_c.map_or("--".into(), |t: u8| t.to_string()),
                        fb.faults,
                    );
                    if let Some(out) = csv.as_mut() {
                        writeln!(
                            out,
                            "{},{:#04X},{},{},{:.3},{},{:.1},{},{},{},{},",
                            jiff::Timestamp::now(),
                            fb.id,
                            fb.mode_name(),
                            fb.speed_rpm,
                            fb.current_a,
                            fb.temp_c.map_or(String::new(), |t| t.to_string()),
                            fb.position_deg,
                            fb.faults.0,
                            fb.faults,
                            fb.raw_hex(),
                            names[i],
                        )?;
                    }
                }
                Ok(None) => status[i] = format!("{} SILENT", names[i]),
                Err(e) => status[i] = format!("{} err: {e}", names[i]),
            }
            std::thread::sleep(cfg.cycle());
        }
        print!("\r{}        ", status.join(" | "));
        std::io::stdout().flush()?;
        if let Some(out) = csv.as_mut() {
            out.flush()?;
        }
    }
    println!();
    Ok(())
}

/// Small extension so monitor/jog can query without importing protocol
/// details everywhere.
trait PollQuery {
    fn poll_query(&mut self, wait: Duration) -> m0601::Result<Option<m0601::Feedback>>;
}

impl PollQuery for m0601::M0601 {
    fn poll_query(&mut self, wait: Duration) -> m0601::Result<Option<m0601::Feedback>> {
        let frame = m0601::protocol::frame_feedback(self.id());
        self.transact(&frame, wait)
    }
}

/// Resolve `front-left` / `fl` / `front-driver` … to a configured wheel.
pub fn find_wheel<'c>(cfg: &'c Config, name: &str) -> Result<&'c WheelCfg, String> {
    let n = name.to_ascii_lowercase();
    let (end, side) = match n.as_str() {
        "front-left" | "fl" | "front-driver" => (End::Front, Side::Left),
        "front-right" | "fr" | "front-pass" => (End::Front, Side::Right),
        "rear-left" | "rl" | "rear-driver" | "back-left" => (End::Rear, Side::Left),
        "rear-right" | "rr" | "rear-pass" | "back-right" => (End::Rear, Side::Right),
        _ => {
            return Err(format!(
                "unknown wheel \"{name}\" (use front-left/front-right/rear-left/rear-right)"
            ));
        }
    };
    cfg.wheels
        .iter()
        .find(|w| w.end == end && w.side == side)
        .ok_or_else(|| format!("no wheel configured at {end}-{side}"))
}

/// `jog`: one wheel, bounded time, then a full-vehicle stop.
pub fn jog(cfg: &Config, shared: &Arc<Shared>, wheel: &str, rpm: i16, secs: f64) -> CmdResult {
    let target = find_wheel(cfg, wheel)?;
    let rpm = rpm.clamp(-cfg.limits.max_rpm, cfg.limits.max_rpm);
    let duration = Duration::try_from_secs_f64(secs).map_err(|e| format!("--secs: {e}"))?;

    let rover = rover::open(cfg)?;
    // Armed before the first frame: every exit path stops the vehicle.
    let guard = StopGuard {
        bus: rover.bus.clone(),
        ids: rover.ids.clone(),
    };
    // Fail explicitly: silently jogging a different wheel than asked
    // would be a safety bug, not a fallback.
    let idx = rover
        .ids
        .iter()
        .position(|&id| id == target.id)
        .ok_or_else(|| format!("wheel 0x{:02X} not among the opened motors", target.id))?;
    let mut wheels = rover.wheels;

    println!(
        "jog {} (0x{:02X}) at {rpm:+} RPM for {:.1} s — Ctrl-C stops",
        target.corner(),
        target.id,
        duration.as_secs_f64(),
    );
    rover
        .bus
        .set_mode_all(&[target.id], m0601::Mode::Velocity)?;
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline && shared.running.load(Ordering::Relaxed) {
        wheels[idx].drive_velocity_accel(rpm, cfg.limits.accel)?;
        std::thread::sleep(cfg.cycle());
    }
    drop(guard); // the explicit, blocking stop
    println!("stopped.");
    Ok(())
}

/// `calibrate`: walk all four wheels, watch each, and print the corrected
/// `invert` lines. **Never writes the config** — round-tripping TOML with
/// comments invites clobbering; a copy-pasteable block is safer.
pub fn calibrate(cfg: &Config, shared: &Arc<Shared>) -> CmdResult {
    let rover = rover::open(cfg)?;
    let guard = StopGuard {
        bus: rover.bus.clone(),
        ids: rover.ids.clone(),
    };
    let mut wheels = rover.wheels;
    let stdin = std::io::stdin();
    let mut results: Vec<(u8, String, bool, bool)> = Vec::new(); // id, name, old, new

    println!("Chassis on blocks, all four wheels clear? Each wheel spins FORWARD");
    println!("(rover-forward) at 40 RPM for 2 s; answer whether it really did.");
    for (i, w) in cfg.wheels_in_grid_order().iter().enumerate() {
        if !shared.running.load(Ordering::Relaxed) {
            break;
        }
        println!();
        print!(
            "ENTER to spin {} (0x{:02X}), or q+ENTER to abort: ",
            w.corner(),
            w.id
        );
        std::io::stdout().flush()?;
        let mut line = String::new();
        stdin.read_line(&mut line)?;
        if line.trim().eq_ignore_ascii_case("q") {
            break;
        }

        rover.bus.set_mode_all(&[w.id], m0601::Mode::Velocity)?;
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline && shared.running.load(Ordering::Relaxed) {
            wheels[i].drive_velocity_accel(40, cfg.limits.accel)?;
            std::thread::sleep(cfg.cycle());
        }
        rover.bus.safe_stop_all(&[w.id]);

        print!("did {} roll rover-FORWARD? [y/n] ", w.corner());
        std::io::stdout().flush()?;
        let mut answer = String::new();
        stdin.read_line(&mut answer)?;
        let forward = answer.trim().eq_ignore_ascii_case("y");
        // The wheel obeyed the current invert setting; if it went the
        // wrong way, flipping `invert` (and only invert — calibrate
        // never advises touching `mirrored`) fixes it.
        let new_invert = if forward { w.invert } else { !w.invert };
        results.push((w.id, w.name.clone(), w.invert, new_invert));
    }
    drop(guard);

    // "All tested wheels were fine" is vacuously true after an abort —
    // an empty (or partial) run must never be reported as a verified
    // config.
    let untested = cfg.wheels.len() - results.len();
    println!();
    if results.iter().any(|(_, _, old, new)| old != new) {
        println!("paste these into wheels.toml (matching each [[wheel]] by id):");
        for (id, name, old, new) in &results {
            if old != new {
                println!("  # wheel 0x{id:02X} \"{name}\":");
                println!("  invert = {new}   # was {old}");
            }
        }
    } else if untested == 0 {
        println!("all wheels already roll forward — wheels.toml is correct as-is.");
    } else if results.is_empty() {
        println!("aborted before any wheel was tested — nothing verified.");
    } else {
        println!("the {} wheel(s) tested roll forward.", results.len());
    }
    if untested > 0 && !results.is_empty() {
        println!(
            "[!] aborted early: {untested} wheel(s) never tested — wheels.toml is not fully verified."
        );
    }
    Ok(())
}

/// `stop`: one-shot vehicle-wide stop.
pub fn stop(cfg: &Config) -> CmdResult {
    let rover = rover::open(cfg)?;
    println!("stopping {} wheels…", rover.ids.len());
    rover.bus.safe_stop_all(&rover.ids);
    println!("done (braked).");
    Ok(())
}

pub struct DriveFlags {
    pub dry_run: bool,
    pub ignore_silent: bool,
    pub ignore_motion: bool,
    pub ignore_faults: bool,
}

/// `drive`: the 2×2 TUI. Fail-closed startup, then pilot + UI + logger.
pub fn drive(cfg: Config, shared: Arc<Shared>, flags: DriveFlags, warned: bool) -> CmdResult {
    let infos: [WheelInfo; 4] = {
        let ws = cfg.wheels_in_grid_order();
        [0, 1, 2, 3].map(|i| WheelInfo {
            label: ws[i].corner(),
            id: ws[i].id,
            reversed: ws[i].reversed(),
        })
    };
    let labels: [String; 4] = [0, 1, 2, 3].map(|i| infos[i].label.clone());
    let sides: [Side; 4] = {
        let ws = cfg.wheels_in_grid_order();
        [0, 1, 2, 3].map(|i| ws[i].side)
    };

    // Logger (optional).
    let (log_tx, log_thread) = match &cfg.log {
        Some(log) => {
            let (tx, rx) = sync_channel::<pilot::LogRow>(64);
            let path = log.path.clone();
            let names = labels.clone();
            (
                Some(tx),
                Some(std::thread::spawn(move || logger::run(rx, &path, &names))),
            )
        }
        None => (None, None),
    };

    if flags.dry_run {
        // No port is opened at all: SimWheels behind the same seam.
        let ws = cfg.wheels_in_grid_order();
        let wheels = [0, 1, 2, 3].map(|i| SimWheel::new(ws[i].id, ws[i].reversed()));
        let mut p = Pilot::new(
            wheels,
            sides,
            labels,
            cfg.clone(),
            RealClock,
            Arc::clone(&shared),
            || {},
            log_tx,
            Some(pilot::UI_WATCHDOG),
        );
        *lock(&shared.ui_tick) = Instant::now();
        let pilot_thread = std::thread::spawn(move || pilot::run_guarded(&mut p));
        let ui_result = run_ui_stopping_pilot(&shared, pilot_thread, || {
            ui::run(&shared, &infos, &cfg, "DRY RUN (no port)")
        });
        finish_logger(log_thread, &shared);
        ui_result?;
        return Ok(());
    }

    let mut rover = rover::open(&cfg)?;
    if !rover.low_latency {
        eprintln!("[!] kernel low-latency not set; poll timing may miss (see USAGE.md)");
    }

    // Fail-closed startup: ask "is it CONFIRMED safe?", refuse on any
    // no — the same polarity as the CLI's position_entry_allowed.
    startup_gate(&cfg, &mut rover.wheels, &flags)?;

    // Armed before the first drive frame.
    let guard = StopGuard {
        bus: rover.bus.clone(),
        ids: rover.ids.clone(),
    };
    rover.bus.set_mode_all(&rover.ids, m0601::Mode::Velocity)?;

    if warned || !rover.low_latency {
        // Reusing the CLI's pattern: let warnings be read before the
        // alternate screen swallows the scrollback.
        std::thread::sleep(Duration::from_millis(1500));
    }

    let wheels: [m0601::M0601; 4] = match rover.wheels.try_into() {
        Ok(w) => w,
        Err(_) => return Err("expected 4 wheels".into()),
    };
    let stop_bus = rover.bus.clone();
    let stop_ids = rover.ids.clone();
    let mut p = Pilot::new(
        wheels,
        sides,
        labels,
        cfg.clone(),
        RealClock,
        Arc::clone(&shared),
        move || stop_bus.safe_stop_all(&stop_ids),
        log_tx,
        Some(pilot::UI_WATCHDOG),
    );
    // Stamp the UI heartbeat NOW: `Shared::new` ran before the startup
    // gate and the warning pause, so the stale stamp would trip the
    // pilot's UI watchdog on its very first cycle and overwrite the
    // ready prompt with a false "UI stopped ticking" alarm.
    *lock(&shared.ui_tick) = Instant::now();
    let pilot_thread = std::thread::spawn(move || pilot::run_guarded(&mut p));

    let ui_result = run_ui_stopping_pilot(&shared, pilot_thread, || {
        ui::run(&shared, &infos, &cfg, &cfg.bus.port)
    });
    drop(guard); // belt-and-braces second stop
    finish_logger(log_thread, &shared);
    ui_result?;
    println!("all wheels stopped (braked).");
    Ok(())
}

/// Run the UI and, on ANY exit — return or panic — clear `running` and
/// join the pilot before handing control back. Unwinding straight past
/// the join would *detach* the pilot thread, which could then re-command
/// nonzero velocity after the caller's `StopGuard` stop and leave the
/// motors holding speed at process exit. The panic is re-raised only
/// after the pilot is provably gone.
fn run_ui_stopping_pilot<R>(
    shared: &Arc<Shared>,
    pilot: std::thread::JoinHandle<()>,
    ui: impl FnOnce() -> R,
) -> R {
    // AssertUnwindSafe: the closure only touches `Shared`, whose locks
    // are poison-tolerant by design.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(ui));
    shared.running.store(false, Ordering::Relaxed);
    let _ = pilot.join();
    match result {
        Ok(r) => r,
        Err(panic) => std::panic::resume_unwind(panic),
    }
}

fn finish_logger(
    thread: Option<std::thread::JoinHandle<std::io::Result<u64>>>,
    shared: &Arc<Shared>,
) {
    if let Some(t) = thread {
        match t.join() {
            Ok(Ok(rows)) => {
                let dropped = shared.dropped_log_rows.load(Ordering::Relaxed);
                if dropped > 0 {
                    eprintln!("[!] log: wrote {rows} rows, dropped {dropped}");
                }
            }
            Ok(Err(e)) => eprintln!("[!] log: {e}"),
            Err(_) => eprintln!("[!] log thread panicked"),
        }
    }
}

/// Probe every wheel once and refuse to start unless each is confirmed
/// present, still, and fault-free (each check has an explicit override).
fn startup_gate(
    cfg: &Config,
    wheels: &mut [m0601::M0601],
    flags: &DriveFlags,
) -> Result<(), Box<dyn Error>> {
    let mut refusals = Vec::new();
    for (i, w) in wheels.iter_mut().enumerate() {
        let corner = cfg.wheels_in_grid_order()[i].corner();
        match w.query()? {
            None => {
                if !flags.ignore_silent {
                    refusals.push(format!(
                        "{corner} (0x{:02X}) did not reply — --ignore-silent to override",
                        w.id()
                    ));
                }
            }
            Some(fb) => {
                if fb.speed_rpm.unsigned_abs() > 5 && !flags.ignore_motion {
                    refusals.push(format!(
                        "{corner} already turning at {:+} RPM — --ignore-motion to override",
                        fb.speed_rpm
                    ));
                }
                if !fb.faults.is_ok() && !flags.ignore_faults {
                    refusals.push(format!(
                        "{corner} reports {} — --ignore-faults to override",
                        fb.faults
                    ));
                }
            }
        }
    }
    if refusals.is_empty() {
        Ok(())
    } else {
        Err(format!("refusing to start:\n  {}", refusals.join("\n  ")).into())
    }
}
