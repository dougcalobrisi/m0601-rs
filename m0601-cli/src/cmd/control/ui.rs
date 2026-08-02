//! Dashboard drawing and key handling (crossterm raw mode + alt screen).
//!
//! This thread never touches the serial port; it only edits [`Shared`].

use std::io::{self, Write};
use std::sync::atomic::Ordering;
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::style::{Attribute, Color, Print, ResetColor, SetAttribute, SetForegroundColor};
use crossterm::terminal::{Clear, ClearType};
use crossterm::{cursor::MoveTo, queue};
use m0601::Mode;

use super::state::{CmdState, ModeRequest, Shared, lock};

const RPM_MIN: i32 = m0601::protocol::RPM_MIN as i32;
const RPM_MAX: i32 = m0601::protocol::RPM_MAX as i32;

/// Uppercase mode name for the status line.
fn label(mode: Mode) -> String {
    mode.to_string().to_uppercase()
}

pub fn run(shared: &Shared, port: &str, id: u8, preset_rpm: i16) -> io::Result<()> {
    let mut out = io::stdout();
    while shared.running.load(Ordering::Relaxed) {
        draw(&mut out, shared, port, id)?;
        // ~10 Hz redraw; keys are handled as they arrive.
        if event::poll(Duration::from_millis(100))?
            && let Event::Key(key) = event::read()?
            && key.kind != KeyEventKind::Release
        {
            handle_key(shared, key.code, key.modifiers, preset_rpm);
        }
    }
    Ok(())
}

fn draw(out: &mut impl Write, shared: &Shared, port: &str, id: u8) -> io::Result<()> {
    let telemetry = *lock(&shared.telemetry);
    let fb = telemetry.fb;
    let (mode, target, braking) = {
        let cmd = lock(&shared.cmd);
        (cmd.mode, cmd.target, cmd.brake)
    };
    let msg = lock(&shared.msg).clone();

    queue!(out, Clear(ClearType::All))?;
    queue!(
        out,
        MoveTo(2, 0),
        SetForegroundColor(Color::Cyan),
        SetAttribute(Attribute::Bold),
        Print("M0601 Hub Motor — Live Control"),
        SetAttribute(Attribute::Reset),
        ResetColor
    )?;
    // Show what the MOTOR reports, not just what we intend. If the two ever
    // disagree, the operator has to be able to see it: a dashboard that
    // shows the requested mode is exactly how a "brake" key ends up
    // freewheeling a wheel while the screen says BRAKING.
    let reported = fb.and_then(|fb| fb.mode);
    let mode_line = match reported {
        Some(actual) if actual == mode => format!("Mode: {}", label(actual)),
        Some(actual) => format!("Mode: {} (motor: {})", label(mode), label(actual)),
        None => format!("Mode: {} (motor: ?)", label(mode)),
    };
    queue!(
        out,
        MoveTo(2, 1),
        Print(format!("Port {port}   ID 0x{id:02X}   ")),
    )?;
    let desynced = reported.is_some_and(|actual| actual != mode);
    if desynced {
        queue!(
            out,
            SetForegroundColor(Color::Red),
            SetAttribute(Attribute::Bold),
            Print(mode_line),
            SetAttribute(Attribute::Reset),
            ResetColor
        )?;
    } else {
        queue!(out, Print(mode_line))?;
    }

    if let Some(fb) = fb {
        let speed = fb.speed_rpm;
        // The brake byte is honoured only in velocity mode, and only the
        // motor's own reported mode proves we are in it.
        let really_braking = braking && reported == Some(Mode::Velocity);
        let (status, color) = if really_braking {
            ("BRAKING".to_owned(), Color::Red)
        } else if speed.abs() < 3 {
            ("STATIONARY".to_owned(), Color::Yellow)
        } else if speed < 0 {
            ("SPINNING CCW <<".to_owned(), Color::Green)
        } else {
            ("SPINNING >> CW".to_owned(), Color::Green)
        };

        queue!(
            out,
            MoveTo(4, 3),
            SetAttribute(Attribute::Bold),
            Print(format!("Speed    : {speed:+5} RPM")),
            SetAttribute(Attribute::Reset)
        )?;
        queue!(
            out,
            MoveTo(4, 4),
            Print(format!("Current  : {:+7.3} A", fb.current_a))
        )?;
        queue!(
            out,
            MoveTo(4, 5),
            Print(format!("Position : {:6.1} deg", fb.position_deg))
        )?;
        // Temperature arrives only in the every-10th-cycle 0x74 reply;
        // `telemetry.temp_c` holds the last one seen. "--" until the first.
        let temp = telemetry
            .temp_c
            .map_or_else(|| " --".to_owned(), |t| format!("{t:3}"));
        queue!(out, MoveTo(4, 6), Print(format!("Temp     : {temp} C")))?;
        queue!(out, MoveTo(34, 3), Print("Status: "))?;
        queue!(
            out,
            MoveTo(42, 3),
            SetForegroundColor(color),
            SetAttribute(Attribute::Bold),
            Print(status),
            SetAttribute(Attribute::Reset),
            ResetColor
        )?;
        if fb.faults.is_ok() {
            queue!(
                out,
                MoveTo(34, 5),
                SetForegroundColor(Color::Green),
                Print("Error : OK"),
                ResetColor
            )?;
        } else {
            queue!(
                out,
                MoveTo(34, 5),
                SetForegroundColor(Color::Red),
                SetAttribute(Attribute::Bold),
                Print(format!("Error : {}", fb.faults)),
                SetAttribute(Attribute::Reset),
                ResetColor
            )?;
        }
    } else {
        queue!(
            out,
            MoveTo(4, 4),
            SetForegroundColor(Color::Yellow),
            Print("Waiting for telemetry..."),
            ResetColor
        )?;
    }

    let unit = match mode {
        Mode::Velocity => "RPM",
        Mode::Current | Mode::Position => "raw",
    };
    queue!(
        out,
        MoveTo(4, 8),
        SetAttribute(Attribute::Bold),
        Print(format!(
            "Target ({}): {target} {unit}",
            mode.to_string().to_lowercase()
        )),
        SetAttribute(Attribute::Reset)
    )?;

    let keys = [
        "F/B  forward / backward      1-5  50..250 RPM",
        "<-/->  nudge +/-10 RPM       S  stop    K  brake (velocity only)",
        "V/C/P  mode velocity/current/position",
        "Q  quit (velocity mode, zero, then brake)",
    ];
    for (i, line) in keys.iter().enumerate() {
        queue!(out, MoveTo(4, 10 + i as u16), Print(*line))?;
    }
    queue!(
        out,
        MoveTo(2, 15),
        SetForegroundColor(Color::Cyan),
        Print(format!(">> {msg}")),
        ResetColor
    )?;
    out.flush()
}

fn handle_key(shared: &Shared, code: KeyCode, modifiers: KeyModifiers, preset_rpm: i16) {
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
                        None => shared
                            .set_msg("No telemetry — cannot hold position; press V to stop"),
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
            // Protocol guard: position mode requires <10 RPM. Fail CLOSED —
            // no telemetry means the speed is unknown, not that it is zero,
            // and `is_some_and` on None would have read as "not too fast"
            // and let the switch through on a bus whose RX path is dead.
            let speed = lock(&shared.telemetry).fb.map(|fb| fb.speed_rpm);
            match speed {
                None => shared.set_msg("Refused: no telemetry — cannot confirm <10 RPM"),
                Some(rpm) if rpm.abs() >= 10 => shared.set_msg(format!(
                    "Refused: {rpm} RPM — must be under 10 RPM for POSITION mode"
                )),
                Some(_) => {
                    lock(&shared.cmd).mode_request = Some(ModeRequest {
                        mode: Mode::Position,
                        target: None, // poll thread seeds the present angle
                    });
                    // The poll thread seeds the target with the wheel's
                    // present angle, so entering position mode holds still
                    // rather than driving to 0 deg.
                    shared.set_msg("Switching to POSITION (holding current angle)");
                }
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
    let deg = lock(&shared.telemetry).fb.map(|fb| fb.position_deg)?;
    let mut cmd = lock(&shared.cmd);
    cmd.target = deg_to_raw(deg);
    cmd.brake = false;
    Some(deg)
}

/// Degrees to a position setpoint in `0..=POS_MAX`.
///
/// Clamps rather than wrapping: an angle slightly past 360° should hold at
/// the top of the range, not snap round to 0° and drive a full revolution.
///
/// Rounds to nearest so that a hi-res drive-reply angle (`raw` × 360/32767)
/// round-trips back to exactly `raw` — truncating instead can land one step
/// low and make "hold this angle" command a (sub-perceptible) move.
pub fn deg_to_raw(deg: f32) -> i32 {
    // `f32::clamp` propagates NaN rather than clamping it, so rule NaN out
    // explicitly instead of leaning on the `as` cast's NaN-to-zero rule.
    if !deg.is_finite() {
        return 0;
    }
    let frac = (deg / 360.0).clamp(0.0, 1.0);
    // Saturating cast of an already-clamped value: cannot wrap or trap.
    (frac * f32::from(m0601::protocol::POS_MAX)).round() as i32
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
    use m0601::protocol::ReplyKind;
    use m0601::Mode;

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
        assert_eq!(req.target, Some(100), "setpoint rides along with the switch");
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
        assert_ne!(target, 0, "target 0 in position mode means 'drive to 0 deg'");
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
        assert!((0..=32_767).contains(&deg_to_raw(180.0)));
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
}
