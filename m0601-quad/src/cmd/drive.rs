//! The `drive` subcommand: the 2×2 TUI and its thread orchestration
//! (fail-closed startup gate, pilot + UI + logger, and the stop-on-any-exit
//! join dance).

use std::error::Error;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::sync_channel;
use std::time::{Duration, Instant};

use crate::clock::RealClock;
use crate::config::{Config, Side};
use crate::io::SimWheel;
use crate::logger;
use crate::pilot::{self, Pilot};
use crate::rover;
use crate::state::{Shared, lock};
use crate::ui::{self, WheelInfo};

use super::CmdResult;

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
        eprintln!(
            "[!] kernel low-latency not set; poll timing may miss (see {})",
            crate::LATENCY_DOC_URL
        );
    }

    // Fail-closed startup: ask "is it CONFIRMED safe?", refuse on any
    // no — the same polarity as the CLI's position_entry_allowed.
    startup_gate(&cfg, &mut rover.wheels, &flags)?;

    // Armed before the first drive frame.
    let guard = rover.stop_guard();
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
