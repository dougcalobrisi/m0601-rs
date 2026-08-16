//! Read-only and single-wheel bring-up commands: `check`, `monitor`, `jog`,
//! `calibrate`, `stop`. None of these run the full pilot; the drive TUI lives
//! in [`drive`](mod@super::drive).

use std::io::Write as _;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crate::config::Config;
use crate::logger;
use crate::rover;
use crate::state::Shared;

use super::{CmdResult, PollQuery, find_wheel};

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
            "yes".to_owned()
        } else {
            format!("NO — see the udev rule in {}", crate::LATENCY_DOC_URL)
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
        Some(log) => Some(logger::open_appending(&log.path)?),
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
                            "{}",
                            logger::csv_row(
                                jiff::Timestamp::now(),
                                &fb,
                                fb.temp_c,
                                fb.position_deg,
                                &names[i],
                                None,
                            )
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

/// `jog`: one wheel, bounded time, then a full-vehicle stop.
pub fn jog(cfg: &Config, shared: &Arc<Shared>, wheel: &str, rpm: i16, secs: f64) -> CmdResult {
    let target = find_wheel(cfg, wheel)?;
    let rpm = rpm.clamp(-cfg.limits.max_rpm, cfg.limits.max_rpm);
    let duration = Duration::try_from_secs_f64(secs).map_err(|e| format!("--secs: {e}"))?;

    let rover = rover::open(cfg)?;
    // Armed before the first frame: every exit path stops the vehicle.
    let guard = rover.stop_guard();
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
    let guard = rover.stop_guard();
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
