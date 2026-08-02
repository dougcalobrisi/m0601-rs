//! The 50 Hz poll thread. It **owns the serial port** — no other thread
//! touches the bus — and it is the one place [`M0601::safe_stop`] runs, on
//! every exit path including a panic inside its own loop body.
//!
//! The stop is a step to zero followed by the electric brake, not a ramp;
//! `safe_stop` uses accel 1, the motor's fastest setting.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use m0601::protocol::{
    Frame, frame_brake, frame_current, frame_feedback, frame_position, frame_velocity,
};
use m0601::{M0601, Mode};

use super::state::{CmdState, Shared, lock};

/// 50 Hz — the protocol's minimum rate for sustained motion.
const CYCLE: Duration = Duration::from_millis(20);
/// Per-cycle reply wait. Well inside the cycle budget (10 bytes @ 115200
/// ≈ 0.9 ms each way); the CLI-level `--timeout` is never used here.
const REPLY_WAIT: Duration = Duration::from_millis(6);

/// Thread entry point. Runs the loop, then unconditionally stops the motor
/// — whether the loop ended by flag or by panicking.
pub fn run(mut motor: M0601, shared: Arc<Shared>) {
    let result = catch_unwind(AssertUnwindSafe(|| poll_loop(&mut motor, &shared)));
    if result.is_err() {
        // Clear `running` BEFORE the ~300 ms stop sequence, not after. The
        // panic hook has already restored the terminal by this point, so a
        // UI thread left looping here would spend that time painting
        // full-screen redraws over the user's real scrollback.
        shared.running.store(false, Ordering::Relaxed);
        shared.set_msg("poll thread panicked — stopping motor");
    }
    motor.safe_stop();
    if result.is_err() {
        shared.set_msg("poll thread panicked — motor stopped");
    }
}

fn active_frame(id: u8, cmd: &CmdState) -> Frame {
    match cmd.mode {
        Mode::Velocity if cmd.brake => frame_brake(id),
        Mode::Velocity => frame_velocity(id, cmd.target.clamp(-330, 330) as i16, 1),
        Mode::Current => frame_current(id, cmd.target.clamp(-32767, 32767) as i16),
        Mode::Position => frame_position(id, cmd.target.clamp(0, 32767) as u16),
    }
}

fn poll_loop(motor: &mut M0601, shared: &Shared) {
    // Absolute deadlines: sleep until `next`, not `CYCLE` after variable
    // work — otherwise the reply wait would drag the loop below 50 Hz.
    let mut next = Instant::now() + CYCLE;
    let mut cycle: u64 = 0;

    while shared.running.load(Ordering::Relaxed) {
        // Service a queued mode switch first (sends 5 frames, ~100 ms).
        let request = lock(&shared.cmd).mode_request.take();
        if let Some(req) = request {
            match motor.set_mode(req.mode) {
                Ok(()) => {
                    // Honour a setpoint the operator asked for in the same
                    // keystroke; otherwise pick one that means "stay put" —
                    // 0 for velocity and current, but the wheel's present
                    // angle for position, where 0 means "drive to 0 deg".
                    let target = req.target.unwrap_or_else(|| match req.mode {
                        Mode::Position => lock(&shared.fb)
                            .map_or(0, |fb| super::ui::deg_to_raw(fb.position_deg)),
                        Mode::Velocity | Mode::Current => 0,
                    });
                    let mut cmd = lock(&shared.cmd);
                    cmd.mode = req.mode;
                    cmd.target = target;
                    cmd.brake = false;
                }
                Err(e) => shared.set_msg(format!("mode switch failed: {e}")),
            }
            next = Instant::now() + CYCLE;
            continue;
        }

        // Copy the command out under the lock, do I/O without it.
        let cmd = *lock(&shared.cmd);

        // The drive frame goes out EVERY cycle. Substituting a feedback
        // query for it every 10th cycle would leave a 40 ms hole between
        // drive frames — 25 Hz instantaneous against a protocol floor of
        // 50 Hz — and the motor would coast a little every 200 ms.
        let drive = active_frame(motor.id(), &cmd);
        match motor.transact(&drive, REPLY_WAIT) {
            Ok(Some(fb)) => *lock(&shared.fb) = Some(fb),
            Ok(None) => {} // silent cycle — keep driving
            // Transient bus errors (USB hiccup) must not kill the loop; the
            // protocol coasts the motor if we truly go quiet.
            Err(e) => shared.set_msg(format!("bus error: {e} (still polling)")),
        }

        // Drive replies carry telemetry too, but only the 0x74 reply
        // refreshes temperature — so ask for one as an EXTRA frame. Two
        // transactions still fit the 20 ms budget (each is ~6 ms of wait
        // plus ~2 ms on the wire).
        if cycle.is_multiple_of(10) {
            let query = frame_feedback(motor.id());
            if let Ok(Some(fb)) = motor.transact(&query, REPLY_WAIT) {
                *lock(&shared.fb) = Some(fb);
            }
        }

        cycle += 1;
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        }
        next += CYCLE;
        if next < now {
            next = now + CYCLE; // fell behind — re-anchor instead of bursting
        }
    }
}
