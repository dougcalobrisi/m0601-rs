//! Shared state between the UI thread and the 50 Hz poll thread.
//!
//! The invariant that keeps the 50 Hz loop honest: **no lock is ever held
//! across serial I/O**. Both threads lock only to copy small values in or
//! out, so lock hold times are nanoseconds against a 20 ms cycle budget.

use std::sync::atomic::AtomicBool;
use std::sync::{Mutex, MutexGuard, PoisonError};

use m0601::{Feedback, Mode};

/// A queued mode switch, serviced by the poll thread (`set_mode` sends five
/// frames and must not run on the UI thread — it doesn't own the port).
#[derive(Clone, Copy)]
pub struct ModeRequest {
    /// Mode to switch the motor into.
    pub mode: Mode,
    /// Setpoint to adopt once the switch lands.
    ///
    /// `Some` when the operator asked for a mode *and* a value in one
    /// keystroke (pressing `F` from current mode means "be in velocity mode,
    /// going forward"); the poll thread must apply it rather than
    /// substituting its own default and silently discarding the request.
    ///
    /// `None` means "pick something that keeps the wheel where it is" —
    /// which is 0 for velocity and current, but the *current angle* for
    /// position, where 0 would command a move to 0°.
    pub target: Option<i32>,
}

/// What the poll thread should be sending right now. `Copy`, so the poll
/// thread locks, copies it out, unlocks, and does I/O lock-free.
#[derive(Clone, Copy)]
pub struct CmdState {
    /// Mode we are driving as. Updated only once the poll thread has
    /// actually sent the switch — never optimistically, or the dashboard
    /// would claim a mode the motor is not in.
    pub mode: Mode,
    /// Velocity RPM / current raw / position raw, depending on mode.
    /// Kept in range by the key handler.
    pub target: i32,
    /// Electric brake engaged (velocity mode only).
    pub brake: bool,
    /// Queued mode switch, if any.
    pub mode_request: Option<ModeRequest>,
}

/// State shared between the UI and poll threads.
pub struct Shared {
    /// Cleared by: Q/Esc/Ctrl-C key, signal handler, or either thread
    /// failing. Both loops exit when false.
    pub running: AtomicBool,
    /// Drive command, owned logically by the UI, read by the poll thread.
    pub cmd: Mutex<CmdState>,
    /// Latest telemetry, written by the poll thread, read by the UI.
    pub fb: Mutex<Option<Feedback>>,
    /// One-line status message shown at the bottom of the dashboard.
    pub msg: Mutex<String>,
}

impl Shared {
    pub fn new() -> Self {
        Self {
            running: AtomicBool::new(true),
            cmd: Mutex::new(CmdState {
                mode: Mode::Velocity,
                target: 0,
                brake: false,
                mode_request: None,
            }),
            fb: Mutex::new(None),
            msg: Mutex::new("Ready. Ensure the wheel is clear before spinning.".to_owned()),
        }
    }

    pub fn set_msg(&self, msg: impl Into<String>) {
        *lock(&self.msg) = msg.into();
    }
}

/// Lock that shrugs off poisoning: all guarded data is plain values (no
/// invariants to corrupt), and the control loop must keep driving/stopping
/// the motor even if the other thread panicked.
pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}
