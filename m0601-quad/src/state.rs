//! State shared between the pilot, UI and logger threads.
//!
//! Carried over from the CLI's control loop, and load-bearing here too:
//! **no lock is ever held across serial I/O.** Threads lock only to copy
//! small values in or out. One mutex guards all four wheels — there is no
//! lock ordering to get wrong, and the UI always sees a coherent snapshot
//! of the vehicle, never a torn one.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU64};
use std::time::Instant;

use m0601::Telemetry;

/// What the operator wants right now. Owned logically by the UI; the
/// pilot copies it out once per cycle.
#[derive(Debug, Clone, Copy)]
pub struct Intent {
    /// `-1.0..=1.0`, latched (this is not hold-to-drive — a terminal has
    /// no key-release events to build a deadman from).
    pub throttle: f32,
    /// `-1.0..=1.0`, positive = right.
    pub turn: f32,
    /// Hold the electric brake on all wheels.
    pub brake: bool,
    /// One-shot: zero everything NOW, bypassing the ramp. Consumed by the
    /// pilot (set false once acted on).
    pub all_stop: bool,
    /// One-shot: clear a latched trip. Consumed by the pilot.
    pub rearm: bool,
}

impl Default for Intent {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            turn: 0.0,
            brake: false,
            all_stop: false,
            rearm: false,
        }
    }
}

/// Everything the pilot knows about one wheel. `cmd_rpm` is the ramped
/// setpoint actually on the wire this cycle — logging it is what makes a
/// rover log useful ("commanded 100, got 30" vs "commanded 30").
#[derive(Debug, Clone, Copy, Default)]
pub struct WheelState {
    pub cmd_rpm: i16,
    pub telemetry: Telemetry,
    /// When the last parseable reply from this wheel arrived.
    pub last_reply: Option<Instant>,
    /// Consecutive polls of this wheel that returned nothing.
    pub missed_polls: u32,
    /// When reported current first exceeded the trip threshold and has
    /// stayed above it since (the debounce window for the current trip).
    pub over_current_since: Option<Instant>,
}

pub struct Shared {
    /// Cleared by: quit key, signals, or either thread failing.
    pub running: AtomicBool,
    /// UI → pilot.
    pub intent: Mutex<Intent>,
    /// Pilot → UI/logger, one coherent vehicle snapshot.
    pub wheels: Mutex<[WheelState; 4]>,
    /// A latched trip reason; `None` = armed. Only the pilot writes it;
    /// clearing requires the operator's explicit re-arm (`R`).
    pub trip: Mutex<Option<String>>,
    /// One-line status message for the dashboard footer.
    pub msg: Mutex<String>,
    /// UI heartbeat: the pilot zeroes all setpoints if this goes stale
    /// (a UI wedged on a hung SSH pty must not leave four wheels driving).
    pub ui_tick: Mutex<Instant>,
    /// CSV rows dropped because the logger channel was full.
    pub dropped_log_rows: AtomicU64,
    /// Cycles the pilot overran its deadline, consecutively.
    pub overruns: AtomicU64,
}

impl Shared {
    pub fn new() -> Self {
        Self {
            running: AtomicBool::new(true),
            intent: Mutex::new(Intent::default()),
            wheels: Mutex::new([WheelState::default(); 4]),
            trip: Mutex::new(None),
            msg: Mutex::new("Ready. Wheels clear of the ground?".to_owned()),
            ui_tick: Mutex::new(Instant::now()),
            dropped_log_rows: AtomicU64::new(0),
            overruns: AtomicU64::new(0),
        }
    }

    pub fn set_msg(&self, msg: impl Into<String>) {
        *lock(&self.msg) = msg.into();
    }
}

impl Default for Shared {
    fn default() -> Self {
        Self::new()
    }
}

/// Poison-tolerant lock: guarded data is plain values, and the stop paths
/// must keep working even if another thread panicked.
pub fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}
