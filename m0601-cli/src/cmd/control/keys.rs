//! Key handling — the write side of the control TUI.
//!
//! Translates key presses into [`Shared`] command state; like the renderer in
//! [`draw`](super::draw), it never touches the serial port.

use std::sync::atomic::Ordering;

use crossterm::event::{KeyCode, KeyModifiers};
use m0601::Mode;

use super::state::{CmdState, ModeRequest, Shared, lock};
use crate::cmd::POSITION_ENTRY_RPM;

const RPM_MIN: i32 = m0601::protocol::RPM_MIN as i32;
const RPM_MAX: i32 = m0601::protocol::RPM_MAX as i32;

pub(super) fn handle_key(shared: &Shared, code: KeyCode, modifiers: KeyModifiers, preset_rpm: i16) {
    // Ctrl-C arrives as a key event in raw mode — treat as quit, before the
    // plain 'c' (current mode) branch can see it.
    if modifiers.contains(KeyModifiers::CONTROL) && matches!(code, KeyCode::Char('c' | 'C')) {
        quit(shared);
        return;
    }

    let mut cmd = lock(&shared.cmd);
    match code {
        KeyCode::Char('q' | 'Q') | KeyCode::Esc => {
            drop(cmd);
            quit(shared);
        }
        // Speed keys are velocity-mode commands. They must *request* the
        // mode switch (which sends the 0xA0 frames) rather than just
        // relabelling our own state — otherwise the motor stays in current
        // or position mode and reads the RPM figure as a torque or an
        // angle, while the dashboard cheerfully reports "VELOCITY".
        KeyCode::Char('f' | 'F') => {
            let pending = drive_at(&mut cmd, i32::from(preset_rpm));
            drop(cmd);
            shared.set_msg(annotate(format!("Forward {preset_rpm} RPM"), pending));
        }
        KeyCode::Char('b' | 'B') => {
            let pending = drive_at(&mut cmd, -i32::from(preset_rpm));
            drop(cmd);
            shared.set_msg(annotate(format!("Backward {preset_rpm} RPM"), pending));
        }
        KeyCode::Char(c @ '1'..='5') => {
            let rpm = i32::from(c as u8 - b'0') * 50;
            let pending = drive_at(&mut cmd, rpm);
            drop(cmd);
            shared.set_msg(annotate(format!("{rpm} RPM"), pending));
        }
        KeyCode::Left | KeyCode::Right => {
            if cmd.mode == Mode::Velocity {
                let delta = if code == KeyCode::Left { -10 } else { 10 };
                cmd.target = (cmd.target + delta).clamp(RPM_MIN, RPM_MAX);
                cmd.brake = false;
                let target = cmd.target;
                drop(cmd);
                shared.set_msg(format!("{target} RPM"));
            } else {
                let mode = cmd.mode;
                drop(cmd);
                shared.set_msg(format!("Nudge is velocity-mode only (in {mode}) — press V"));
            }
        }
        KeyCode::Char('s' | 'S') => {
            // "Target 0" only means "stop" in velocity mode. In position
            // mode it commands a move to 0 deg — up to half a revolution of
            // travel from a key labelled "stop" — so hold the current angle
            // instead, and in current mode say plainly that zero torque is
            // a coast, not a brake.
            match cmd.mode {
                Mode::Velocity => {
                    cmd.target = 0;
                    cmd.brake = false;
                    drop(cmd);
                    shared.set_msg("Stop (0 RPM)");
                }
                Mode::Current => {
                    cmd.target = 0;
                    cmd.brake = false;
                    drop(cmd);
                    shared.set_msg("Zero current — coasting (K cannot brake in current mode)");
                }
                Mode::Position => {
                    drop(cmd);
                    match hold_position(shared) {
                        Some(deg) => shared.set_msg(format!("Holding {deg:.1} deg")),
                        None => {
                            shared.set_msg("No telemetry — cannot hold position; press V to stop")
                        }
                    }
                }
            }
        }
        KeyCode::Char('k' | 'K') => {
            if cmd.mode == Mode::Velocity {
                cmd.brake = true;
                drop(cmd);
                shared.set_msg("Electric brake");
            } else {
                drop(cmd);
                shared.set_msg("Brake only in velocity mode");
            }
        }
        KeyCode::Char('v' | 'V') => {
            cmd.mode_request = Some(ModeRequest {
                mode: Mode::Velocity,
                target: None,
            });
            drop(cmd);
            shared.set_msg("Switching to VELOCITY (target 0)");
        }
        KeyCode::Char('c' | 'C') => {
            cmd.mode_request = Some(ModeRequest {
                mode: Mode::Current,
                target: None,
            });
            drop(cmd);
            shared.set_msg("Switching to CURRENT (target 0)");
        }
        KeyCode::Char('p' | 'P') => {
            drop(cmd);
            // Protocol guard: position mode requires <10 RPM, and it fails
            // closed on missing telemetry — see `position_entry_allowed`,
            // which the batch `drive position` path shares.
            let speed = lock(&shared.telemetry).fb.map(|fb| fb.speed_rpm);
            if !crate::cmd::position_entry_allowed(speed) {
                match speed {
                    None => shared.set_msg(format!(
                        "Refused: no telemetry — cannot confirm <{POSITION_ENTRY_RPM} RPM"
                    )),
                    Some(rpm) => shared.set_msg(format!(
                        "Refused: {rpm} RPM — must be under {POSITION_ENTRY_RPM} RPM for POSITION mode"
                    )),
                }
            } else {
                lock(&shared.cmd).mode_request = Some(ModeRequest {
                    mode: Mode::Position,
                    target: None, // poll thread seeds the present angle
                });
                // The poll thread seeds the target with the wheel's present
                // angle, so entering position mode holds still rather than
                // driving to 0 deg.
                shared.set_msg("Switching to POSITION (holding current angle)");
            }
        }
        _ => {}
    }
}

/// Drive at `rpm`, switching to velocity mode first if we are not already
/// there. Returns `true` when a mode switch is now pending, so the caller
/// can say so rather than implying the speed took effect immediately.
///
/// The setpoint rides along inside the request: the poll thread seeds a
/// fresh target after every switch, and would otherwise overwrite this one.
fn drive_at(cmd: &mut CmdState, rpm: i32) -> bool {
    let rpm = rpm.clamp(RPM_MIN, RPM_MAX);
    cmd.brake = false;
    cmd.target = rpm;
    if cmd.mode == Mode::Velocity && cmd.mode_request.is_none() {
        return false;
    }
    cmd.mode_request = Some(ModeRequest {
        mode: Mode::Velocity,
        target: Some(rpm),
    });
    true
}

/// Note that a setpoint is queued behind a mode switch, so the operator is
/// never shown a speed the motor is not actually acting on yet.
fn annotate(msg: String, pending: bool) -> String {
    if pending {
        format!("{msg} (switching to VELOCITY first)")
    } else {
        msg
    }
}

/// Set the position target to the wheel's current angle. `None` when there
/// is no telemetry to derive it from.
fn hold_position(shared: &Shared) -> Option<f32> {
    // Prefer the hi-res angle retained from drive replies (what the
    // dashboard displays); fall back to the latest reply's own angle. Seeding
    // from the coarse 8-bit query reply would hold up to ~1.4° off the shown
    // value and contradict the "Holding … deg" message.
    let deg = {
        let tele = lock(&shared.telemetry);
        tele.position_deg
            .or_else(|| tele.fb.map(|fb| fb.position_deg))?
    };
    let mut cmd = lock(&shared.cmd);
    cmd.target = deg_to_raw(deg);
    cmd.brake = false;
    Some(deg)
}

/// Degrees to a position setpoint, widened to the `i32` that [`CmdState`]
/// carries. Clamping, rounding and NaN handling all live in
/// [`m0601::protocol::deg_to_raw`]. Shared with the poll thread, which seeds
/// the present angle on entry to position mode.
pub(super) fn deg_to_raw(deg: f32) -> i32 {
    i32::from(m0601::protocol::deg_to_raw(deg))
}

fn quit(shared: &Shared) {
    {
        let mut cmd = lock(&shared.cmd);
        cmd.target = 0;
        cmd.brake = false;
    }
    shared.set_msg("Quitting...");
    shared.running.store(false, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::{RPM_MAX, deg_to_raw, handle_key};
    use crate::cmd::control::state::{Shared, lock};
    use crossterm::event::{KeyCode, KeyModifiers};
    use m0601::Mode;
    use m0601::protocol::ReplyKind;

    const NONE: KeyModifiers = KeyModifiers::NONE;

    /// Query-reply telemetry reporting `mode` and `rpm`, as the motor would
    /// send it in answer to a 0x74 frame (position byte 0x80 ≈ 180.7°).
    fn seed(shared: &Shared, mode: u8, rpm: i16) {
        let [hi, lo] = rpm.to_be_bytes();
        let frame = [0x01, mode, 0x00, 0x00, hi, lo, 0x28, 0x80, 0x00, 0x00];
        if let Some(fb) = m0601::protocol::parse_feedback(&frame, ReplyKind::Query) {
            lock(&shared.telemetry).absorb(fb);
        }
    }

    /// Drive-reply telemetry: hi-res 16-bit position, no temperature.
    fn seed_drive(shared: &Shared, mode: u8, rpm: i16, pos_raw: u16) {
        let [shi, slo] = rpm.to_be_bytes();
        let [phi, plo] = pos_raw.to_be_bytes();
        let frame = [0x01, mode, 0x00, 0x00, shi, slo, phi, plo, 0x00, 0x00];
        if let Some(fb) = m0601::protocol::parse_feedback(&frame, ReplyKind::Drive) {
            lock(&shared.telemetry).absorb(fb);
        }
    }

    fn press(shared: &Shared, c: char) {
        handle_key(shared, KeyCode::Char(c), NONE, 100);
    }

    #[test]
    fn speed_keys_request_a_real_mode_switch_not_just_a_relabel() {
        let shared = Shared::new();
        // Pretend the motor is in current mode.
        lock(&shared.cmd).mode = Mode::Current;

        press(&shared, 'f');

        let cmd = *lock(&shared.cmd);
        let req = cmd.mode_request.expect("F must queue a 0xA0 mode switch");
        assert_eq!(req.mode, Mode::Velocity);
        assert_eq!(
            req.target,
            Some(100),
            "setpoint rides along with the switch"
        );
        // Crucially, our own idea of the mode has NOT moved yet: the motor
        // is still in current mode until the poll thread sends the frames.
        assert_eq!(cmd.mode, Mode::Current);
    }

    #[test]
    fn speed_keys_in_velocity_mode_need_no_switch() {
        let shared = Shared::new(); // starts in Velocity
        press(&shared, '3');
        let cmd = *lock(&shared.cmd);
        assert!(cmd.mode_request.is_none());
        assert_eq!(cmd.target, 150);
    }

    #[test]
    fn stop_in_position_mode_does_not_command_a_move_to_zero() {
        let shared = Shared::new();
        lock(&shared.cmd).mode = Mode::Position;
        lock(&shared.cmd).target = 20_000;
        seed(&shared, 0x03, 0); // wheel resting at 0x80 = ~180.7 deg

        press(&shared, 's');

        let target = lock(&shared.cmd).target;
        assert_ne!(
            target, 0,
            "target 0 in position mode means 'drive to 0 deg'"
        );
        // It holds the reported angle instead.
        assert_eq!(target, deg_to_raw(128.0 * 360.0 / 255.0));
    }

    #[test]
    fn stop_in_velocity_mode_is_zero_rpm() {
        let shared = Shared::new();
        lock(&shared.cmd).target = 250;
        press(&shared, 's');
        assert_eq!(lock(&shared.cmd).target, 0);
    }

    #[test]
    fn position_mode_is_refused_without_telemetry() {
        let shared = Shared::new();
        assert!(lock(&shared.telemetry).fb.is_none());
        press(&shared, 'p');
        assert!(
            lock(&shared.cmd).mode_request.is_none(),
            "unknown speed must fail closed, not read as zero"
        );
    }

    #[test]
    fn position_mode_is_refused_above_ten_rpm() {
        let shared = Shared::new();
        seed(&shared, 0x02, 300);
        press(&shared, 'p');
        assert!(lock(&shared.cmd).mode_request.is_none());
    }

    #[test]
    fn position_mode_is_allowed_when_stopped() {
        let shared = Shared::new();
        seed(&shared, 0x02, 0);
        press(&shared, 'p');
        let req = lock(&shared.cmd).mode_request.expect("switch queued");
        assert_eq!(req.mode, Mode::Position);
        assert_eq!(req.target, None, "poll thread seeds the present angle");
    }

    #[test]
    fn brake_is_ignored_outside_velocity_mode() {
        let shared = Shared::new();
        lock(&shared.cmd).mode = Mode::Current;
        press(&shared, 'k');
        assert!(
            !lock(&shared.cmd).brake,
            "the brake byte does nothing in current mode; do not claim otherwise"
        );
    }

    #[test]
    fn quit_keys_clear_running() {
        for key in ['q', 'Q'] {
            let shared = Shared::new();
            press(&shared, key);
            assert!(!shared.running.load(std::sync::atomic::Ordering::Relaxed));
        }
        // Ctrl-C must quit, not be swallowed by the 'c' (current mode) arm.
        let shared = Shared::new();
        handle_key(&shared, KeyCode::Char('c'), KeyModifiers::CONTROL, 100);
        assert!(!shared.running.load(std::sync::atomic::Ordering::Relaxed));
        assert!(lock(&shared.cmd).mode_request.is_none());
    }

    #[test]
    fn nudge_stays_within_the_motor_range() {
        let shared = Shared::new();
        for _ in 0..50 {
            handle_key(&shared, KeyCode::Right, NONE, 100);
        }
        assert_eq!(lock(&shared.cmd).target, RPM_MAX);
    }

    #[test]
    fn deg_to_raw_covers_the_full_turn_without_wrapping() {
        assert_eq!(deg_to_raw(0.0), 0);
        assert_eq!(deg_to_raw(360.0), 32_767);
        // Out-of-band and non-finite inputs clamp rather than wrap or trap.
        assert_eq!(deg_to_raw(-90.0), 0);
        assert_eq!(deg_to_raw(1_000.0), 32_767);
        assert_eq!(deg_to_raw(f32::NAN), 0);
        // Pin the midpoint, don't just assert it lands somewhere in range:
        // `deg_to_raw` clamps before scaling, so a range check here holds for
        // every possible input and would survive any scaling bug.
        assert_eq!(deg_to_raw(180.0), 16_384);
    }

    #[test]
    fn deg_to_raw_round_trips_drive_reply_angles_exactly() {
        // A drive reply reports raw × 360/32767 degrees; "hold this angle"
        // must map that back to exactly raw, or entering position mode
        // commands a (tiny) move. Rounding makes the trip exact.
        for raw in [1u16, 3, 1000, 16_383, 20_000, 32_766, 32_767] {
            let deg = f32::from(raw) * 360.0 / 32_767.0;
            assert_eq!(deg_to_raw(deg), i32::from(raw), "raw {raw}");
        }
    }

    #[test]
    fn stop_in_position_mode_holds_hi_res_drive_angle_exactly() {
        let shared = Shared::new();
        lock(&shared.cmd).mode = Mode::Position;
        lock(&shared.cmd).target = 0;
        seed_drive(&shared, 0x03, 0, 20_000); // ≈ 219.7°, hi-res

        press(&shared, 's');

        assert_eq!(
            lock(&shared.cmd).target,
            20_000,
            "held angle must be the exact reported position step"
        );
    }

    #[test]
    fn stop_in_position_mode_seeds_the_displayed_hi_res_angle_not_the_coarse_reply() {
        // A drive reply set the hi-res angle the dashboard shows; a *later*
        // query reply then left `fb` coarse (~1.4° steps) without disturbing
        // that retained hi-res value. Holding position must seed from the
        // displayed hi-res angle, not the coarse `fb`, or "hold current
        // position" nudges the wheel and contradicts the on-screen readout.
        let shared = Shared::new();
        lock(&shared.cmd).mode = Mode::Position;
        seed_drive(&shared, 0x03, 0, 20_000); // hi-res ≈ 219.7°, retained
        seed(&shared, 0x03, 0); // coarse query reply, 0x80 ≈ 180.7°

        // Precondition: the coarse `fb` and the retained hi-res angle really
        // do disagree, so the test would catch a regression to the coarse seed.
        {
            let tele = lock(&shared.telemetry);
            let coarse = tele.fb.map(|fb| fb.position_deg).expect("fb seeded");
            let hires = tele.position_deg.expect("hi-res retained");
            assert!(
                (coarse - hires).abs() > 30.0,
                "coarse {coarse} and hi-res {hires} must differ for this test"
            );
        }

        press(&shared, 's');

        assert_eq!(
            lock(&shared.cmd).target,
            20_000,
            "held angle must be the displayed hi-res angle, not the coarse reply"
        );
    }
}
