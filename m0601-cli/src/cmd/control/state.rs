//! Shared state between the UI thread and the 50 Hz poll thread.
//!
//! The invariant that keeps the 50 Hz loop honest: **no lock is ever held
//! across serial I/O**. Both threads lock only to copy small values in or
//! out, so lock hold times are nanoseconds against a 20 ms cycle budget.

use std::sync::atomic::AtomicBool;
use std::sync::{Mutex, MutexGuard, PoisonError};

use m0601::{Feedback, Mode, ReplyKind};

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

/// Latest telemetry, plus the readings that only one reply layout carries
/// and must be retained across the replies that don't.
///
/// The 50 Hz loop gets a drive reply every cycle (hi-res 16-bit position, no
/// temperature) and an extra 0x74 query reply only every 10th cycle
/// (temperature + a coarse 8-bit position). So the winding temperature and
/// the hi-res angle each come from a *different* reply layout; each is kept
/// apart from `fb` rather than flickering as `fb` alternates between the two.
#[derive(Clone, Copy, Default)]
pub struct Telemetry {
    /// Most recent reply of either kind — the source of mode, speed, current
    /// and faults, which decode identically in both layouts.
    pub fb: Option<Feedback>,
    /// Winding temperature from the most recent query (0x74) reply.
    pub temp_c: Option<u8>,
    /// Wheel angle from the most recent *drive* reply (hi-res 16-bit). Held
    /// apart from `fb` so the every-10th-cycle query reply's coarse 8-bit
    /// angle doesn't make the displayed position flicker between resolutions.
    pub position_deg: Option<f32>,
}

impl Telemetry {
    /// Store `fb` as latest, and separately retain the readings only one
    /// layout carries: temperature (query replies) and the hi-res angle
    /// (drive replies).
    pub fn absorb(&mut self, fb: Feedback) {
        if let Some(t) = fb.temp_c {
            self.temp_c = Some(t);
        }
        if fb.kind == ReplyKind::Drive {
            self.position_deg = Some(fb.position_deg);
        }
        self.fb = Some(fb);
    }
}

/// State shared between the UI and poll threads.
pub struct Shared {
    /// Cleared by: Q/Esc/Ctrl-C key, signal handler, or either thread
    /// failing. Both loops exit when false.
    pub running: AtomicBool,
    /// Drive command, owned logically by the UI, read by the poll thread.
    pub cmd: Mutex<CmdState>,
    /// Latest telemetry, written by the poll thread, read by the UI.
    pub telemetry: Mutex<Telemetry>,
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
            telemetry: Mutex::new(Telemetry::default()),
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

#[cfg(test)]
mod tests {
    use super::Telemetry;
    use m0601::Feedback;
    use m0601::protocol::{ReplyKind, parse_feedback};

    /// The same reply bytes decoded as either kind (temp 40 °C as a query).
    fn fb(kind: ReplyKind) -> Feedback {
        parse_feedback(&[0x01, 0x02, 0, 0, 0, 0x64, 0x28, 0x80, 0, 0], kind).expect("valid frame")
    }

    #[test]
    fn absorb_retains_temperature_across_drive_replies() {
        let mut t = Telemetry::default();
        t.absorb(fb(ReplyKind::Drive));
        assert_eq!(t.temp_c, None, "no query reply seen yet");
        t.absorb(fb(ReplyKind::Query));
        assert_eq!(t.temp_c, Some(40));
        t.absorb(fb(ReplyKind::Drive));
        assert_eq!(t.temp_c, Some(40), "a drive reply must not clear it");
        // fb always tracks the latest reply of either kind.
        assert_eq!(t.fb.map(|fb| fb.kind), Some(ReplyKind::Drive));
    }

    #[test]
    fn absorb_keeps_hi_res_drive_angle_across_a_query_reply() {
        let mut t = Telemetry::default();
        // A query reply before any drive reply: no hi-res angle retained yet
        // (the UI falls back to the reply's own coarse position meanwhile).
        t.absorb(fb(ReplyKind::Query));
        assert_eq!(t.position_deg, None, "no hi-res drive reply seen yet");
        // A drive reply establishes the hi-res angle...
        t.absorb(fb(ReplyKind::Drive));
        let hi_res = t.position_deg.expect("drive reply sets the hi-res angle");
        // ...and a later query reply must NOT overwrite it with its coarse
        // 8-bit angle — that flicker is exactly what this field prevents.
        t.absorb(fb(ReplyKind::Query));
        assert_eq!(
            t.position_deg,
            Some(hi_res),
            "a query reply must not downgrade the retained hi-res angle"
        );
    }
}
