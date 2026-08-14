//! The 50 Hz poll thread. It **owns the serial port** — no other thread
//! touches the bus — and it is the one place [`M0601::safe_stop`] runs, on
//! every exit path including a panic inside its own loop body.
//!
//! The stop is a step to zero followed by the electric brake, not a ramp;
//! `safe_stop` uses accel 1, the motor's fastest setting.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Instant;

use m0601::protocol::{
    CUR_MAX, CUR_MIN, Frame, POS_MAX, RPM_MAX, RPM_MIN, frame_brake, frame_current, frame_feedback,
    frame_position, frame_velocity,
};
use m0601::{M0601, Mode};

use super::state::{CmdState, Shared, lock};
use crate::cmd::{CYCLE, REPLY_WAIT, next_deadline};

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
        Mode::Velocity => frame_velocity(
            id,
            cmd.target.clamp(RPM_MIN.into(), RPM_MAX.into()) as i16,
            1,
        ),
        Mode::Current => frame_current(id, cmd.target.clamp(CUR_MIN.into(), CUR_MAX.into()) as i16),
        Mode::Position => frame_position(id, cmd.target.clamp(0, POS_MAX.into()) as u16),
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
                        Mode::Position => {
                            // Seed the hold from the SAME angle the dashboard
                            // shows — the hi-res angle retained from drive
                            // replies — not the coarse 8-bit angle of the
                            // latest 0x74 query reply, or "hold current
                            // position" would nudge the wheel up to ~1.4°.
                            let tele = lock(&shared.telemetry);
                            tele.position_deg
                                .or_else(|| tele.fb.map(|fb| fb.position_deg))
                                .map_or(0, super::keys::deg_to_raw)
                        }
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
            Ok(Some(fb)) => lock(&shared.telemetry).absorb(fb),
            Ok(None) => {} // silent cycle — keep driving
            // Transient bus errors (USB hiccup) must not kill the loop; the
            // protocol coasts the motor if we truly go quiet.
            Err(e) => shared.set_msg(format!("bus error: {e} (still polling)")),
        }

        // Drive replies carry telemetry too (with a hi-res 16-bit position),
        // but only the 0x74 reply carries the winding temperature — so ask
        // for one as an EXTRA frame; `absorb` retains its temperature across
        // the drive replies in between. Two transactions still fit the 20 ms
        // budget (each is ~6 ms of wait plus ~2 ms on the wire).
        if cycle.is_multiple_of(10) {
            let query = frame_feedback(motor.id());
            if let Ok(Some(fb)) = motor.transact(&query, REPLY_WAIT) {
                lock(&shared.telemetry).absorb(fb);
            }
        }

        cycle += 1;
        next = next_deadline(next);
    }
}

#[cfg(test)]
mod tests {
    use super::active_frame;
    use crate::cmd::CYCLE;
    use crate::cmd::control::state::CmdState;
    use m0601::Mode;
    use m0601::protocol::DRIVE_HZ_MIN;
    use std::time::Duration;

    fn cmd(mode: Mode, target: i32, brake: bool) -> CmdState {
        CmdState {
            mode,
            target,
            brake,
            mode_request: None,
        }
    }

    #[test]
    fn the_cycle_honours_the_protocol_drive_rate() {
        // CYCLE is a hardcoded 20 ms and DRIVE_HZ_MIN is the protocol floor
        // it exists to satisfy; nothing else ties the two together, so
        // raising the constant would silently under-drive the motor.
        let slowest_allowed = Duration::from_secs(1) / DRIVE_HZ_MIN;
        assert!(
            CYCLE <= slowest_allowed,
            "{CYCLE:?} per cycle is below the {DRIVE_HZ_MIN} Hz floor ({slowest_allowed:?})"
        );
    }

    #[test]
    fn each_mode_sends_its_own_kind_of_drive_frame() {
        // Literal wire bytes — recomputing these with the frame builders
        // would assert the builders against themselves.
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Velocity, 100, false)),
            [0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00, 0x00, 0xE4]
        );
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Current, 4096, false)),
            [0x01, 0x64, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB]
        );
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Position, 16_384, false)),
            [0x01, 0x64, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97]
        );
    }

    #[test]
    fn brake_is_only_honoured_in_velocity_mode() {
        // The brake byte does nothing outside velocity mode, so a brake flag
        // set there must not suppress the real setpoint — the wheel would
        // coast while the dashboard said BRAKING.
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Velocity, 100, true)),
            [0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1],
            "velocity + brake must send the brake frame"
        );
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Current, 4096, true)),
            [0x01, 0x64, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB],
            "a stale brake flag must not replace the current setpoint"
        );
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Position, 16_384, true)),
            [0x01, 0x64, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97],
            "a stale brake flag must not replace the position setpoint"
        );
    }

    #[test]
    fn out_of_range_targets_clamp_rather_than_wrap() {
        // `target` is an i32 narrowed with `as` per mode, so the clamp is
        // what stops the cast truncating. These two cases are the ones
        // where it is load-bearing — delete the clamp and they fail.
        //
        // A negative position: `-5i32 as u16` is 65531, which frame_position
        // then floors to POS_MAX, putting a full 360 deg on the wire in
        // place of 0.
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Position, -5, false)),
            [0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50],
            "a negative position must clamp to 0, not land on a full turn"
        );
        // A current beyond i16: `99_999i32 as i16` is -31073, so the cast
        // flips the sign and commands near-full-scale torque the *other*
        // way. Nothing downstream catches that — -31073 is in range.
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Current, 99_999, false)),
            [0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97],
            "an over-range current must saturate, not wrap to full reverse"
        );
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Current, -99_999, false)),
            [0x01, 0x64, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0]
        );
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Position, 99_999, false)),
            [0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97]
        );
        // Velocity is belt-and-braces: 5000 fits in i16, so the cast cannot
        // truncate, and frame_velocity clamps to RPM_MAX regardless. These
        // two pin the behaviour but would still pass with the clamp gone —
        // they document intent, they do not guard it.
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Velocity, 5_000, false)),
            [0x01, 0x64, 0x01, 0x4A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x7C]
        );
        assert_eq!(
            active_frame(0x01, &cmd(Mode::Velocity, -5_000, false)),
            [0x01, 0x64, 0xFE, 0xB6, 0x00, 0x00, 0x01, 0x00, 0x00, 0x75]
        );
    }

    #[test]
    fn the_frame_is_addressed_to_the_handles_own_motor() {
        assert_eq!(active_frame(0x2A, &cmd(Mode::Velocity, 0, false))[0], 0x2A);
        assert_eq!(active_frame(0x2A, &cmd(Mode::Velocity, 0, true))[0], 0x2A);
    }
}
