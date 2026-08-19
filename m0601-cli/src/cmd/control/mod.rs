//! `control` — full-screen keyboard dashboard.
//!
//! # Thread ownership and shutdown
//!
//! The poll thread **owns the serial port** and drives the motor at 50 Hz;
//! the UI thread owns the terminal and only edits [`state::Shared`]. Every
//! exit path funnels through the same sequence — `running = false` → join
//! poll thread → its epilogue runs [`M0601::safe_stop`], which forces
//! velocity mode before zeroing (a zero setpoint means "go to 0°" in
//! position mode) and then brakes:
//!
//! 1. **Q / Esc / Ctrl-C (as raw-mode key)** — UI clears `running`.
//! 2. **UI panic** — RAII guards unwind in reverse order: `TermGuard`
//!    restores the terminal first (so the panic message is readable), then
//!    `StopGuard` clears `running` and joins the poll thread. A panic hook
//!    also restores the terminal before printing. Note `StopGuard`'s join
//!    is unbounded: if a serial write wedges in the driver, the process
//!    waits on it (the motor is coasting by then, per protocol).
//! 3. **SIGINT before raw mode / SIGTERM / SIGHUP** (dropped SSH session) —
//!    the `ctrlc` handler just clears `running`; the UI tick notices within
//!    100 ms and falls through the normal path.
//! 4. **SIGKILL / power loss** — nothing can run, and that is fine *by
//!    protocol*: polling stops, so the motor coasts to a stop.

mod draw;
mod keys;
mod poll;
mod state;
mod ui;

use std::io;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::thread::JoinHandle;
use std::time::Duration;

use crossterm::cursor::{Hide, Show};
use crossterm::execute;
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use m0601::{M0601, Mode};

use state::Shared;

/// Default velocity ramp for interactive driving (`--accel`).
///
/// `0` — the motor's own default ramp. Named apart from the library's
/// [`m0601::DEFAULT_DRIVE_ACCEL`](m0601::DEFAULT_DRIVE_ACCEL) so the two never
/// read as the same number: `drive velocity` and `drive_velocity` take the
/// library default, `control` takes this one.
///
/// A keystroke here commands a large instantaneous step — a jump to the full
/// preset from standstill, or an F→B reversal — so a too-sharp ramp genuinely
/// risks the 3 A bus-overcurrent trip on a loaded wheel. This was `3`, picked
/// to be "gentler than `1`", but **no vendor source states which end of the
/// byte's range is gentle** (see [`m0601::protocol::frame_velocity`]), so that
/// was a guess that may have been backwards. Ask the motor for its own default
/// instead, and let the operator sweep `--accel` once they have measured it.
pub const CONTROL_DEFAULT_ACCEL: u8 = 0;

/// Restores the terminal on drop (alt screen, raw mode, cursor). Created
/// *after* raw mode is entered so an unwind always restores.
struct TermGuard;

impl Drop for TermGuard {
    fn drop(&mut self) {
        restore_terminal();
    }
}

fn restore_terminal() {
    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), LeaveAlternateScreen, Show);
}

/// Restores the process panic hook on drop. `set_hook` is a global side
/// effect; without this the terminal-restoring hook outlives `control::run`,
/// which matters if it is ever driven from a longer-lived process. `take_hook`
/// removes our hook and reinstates the default in its place.
struct HookGuard;

impl Drop for HookGuard {
    fn drop(&mut self) {
        let _ = std::panic::take_hook();
    }
}

/// Stops the motor on drop: clears `running` and joins the poll thread,
/// whose epilogue runs `safe_stop`. Declared before `TermGuard` so it drops
/// *after* it — terminal first, motor second.
struct StopGuard {
    shared: Arc<Shared>,
    handle: Option<JoinHandle<()>>,
}

impl Drop for StopGuard {
    fn drop(&mut self) {
        self.shared.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

pub fn run(port: &str, id: u8, rpm: i16, accel: u8) -> m0601::Result<ExitCode> {
    // Short port timeout: the 50 Hz loop must never block long on the OS.
    let mut motor = M0601::open(port, id, Duration::from_millis(50))?;
    // Start in velocity mode. A failure here is not fatal — the motor may
    // already be in velocity mode, and the switch is unacknowledged either
    // way — but the warning would be swallowed by the alternate screen a
    // few lines below, so pause long enough for it to be read.
    if let Err(e) = motor.set_mode(Mode::Velocity) {
        eprintln!("[!] initial mode switch failed ({e}); continuing.");
        eprintln!("    The dashboard shows the motor's reported mode; check it before driving.");
        std::thread::sleep(Duration::from_millis(1500));
    }

    let shared = Arc::new(Shared::new());

    // Signals (SIGINT pre-raw-mode, SIGTERM, SIGHUP) only clear the flag.
    {
        let shared = shared.clone();
        if let Err(e) = ctrlc::set_handler(move || shared.running.store(false, Ordering::Relaxed)) {
            // Not fatal — Q/Esc/Ctrl-C still work as raw-mode keys — but the
            // operator should know that a SIGTERM will now kill the process
            // outright, leaving the motor to coast rather than brake.
            eprintln!("[!] could not install signal handler ({e});");
            eprintln!("    SIGTERM/SIGHUP will not run the braked stop — use Q to quit.");
        }
    }

    // A panic anywhere must not leave the terminal in raw mode with the
    // message invisible.
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        restore_terminal();
        default_hook(info);
    }));
    // Restore the previous hook on every exit path (error return included).
    let _hook = HookGuard;

    let poll_handle = {
        let shared = shared.clone();
        std::thread::spawn(move || poll::run(motor, shared, accel))
    };

    // Drop order (reverse of declaration): _term first, then _stop.
    let _stop = StopGuard {
        shared: shared.clone(),
        handle: Some(poll_handle),
    };
    enable_raw_mode()?;
    // Constructed immediately, BEFORE the fallible alt-screen setup below:
    // a `?` between enabling raw mode and arming the guard would return
    // with the tty still raw and nothing left to restore it (the panic hook
    // does not fire on a plain error return).
    let _term = TermGuard;
    execute!(io::stdout(), EnterAlternateScreen, Hide)?;

    let ui_result = ui::run(&shared, port, id, rpm);

    drop(_term); // restore terminal
    drop(_stop); // stop + join poll thread -> safe_stop
    ui_result?;

    println!("Stopped and port closed.");
    Ok(ExitCode::SUCCESS)
}
