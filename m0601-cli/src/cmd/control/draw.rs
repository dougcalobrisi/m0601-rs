//! Live dashboard rendering — the read side of the control TUI.
//!
//! This never touches the serial port; it only reads [`Shared`]. Key handling
//! (the write side) lives in [`keys`](super::keys).

use std::io::{self, Write};

use crossterm::style::{Attribute, Color, Print, ResetColor, SetAttribute, SetForegroundColor};
use crossterm::terminal::{Clear, ClearType};
use crossterm::{cursor::MoveTo, queue};
use m0601::Mode;

use super::state::{Shared, lock};

/// Uppercase mode name for the status line.
fn label(mode: Mode) -> String {
    mode.to_string().to_uppercase()
}

pub(super) fn draw(out: &mut impl Write, shared: &Shared, port: &str, id: u8) -> io::Result<()> {
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
        // Prefer the hi-res angle retained from drive replies; without it the
        // coarse 8-bit position in the every-10th-cycle query reply would make
        // this reading flicker between resolutions. Falls back to this reply's
        // own position until the first drive reply lands.
        let position = telemetry.position_deg.unwrap_or(fb.position_deg);
        queue!(
            out,
            MoveTo(4, 5),
            Print(format!("Position : {position:6.1} deg"))
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

#[cfg(test)]
mod tests {
    use super::draw;
    use crate::cmd::control::state::{Shared, lock};
    use m0601::protocol::ReplyKind;

    /// Query-reply telemetry reporting `mode` and `rpm`, as the motor would
    /// send it in answer to a 0x74 frame (position byte 0x80 ≈ 180.7°).
    fn seed(shared: &Shared, mode: u8, rpm: i16) {
        let [hi, lo] = rpm.to_be_bytes();
        let frame = [0x01, mode, 0x00, 0x00, hi, lo, 0x28, 0x80, 0x00, 0x00];
        if let Some(fb) = m0601::protocol::parse_feedback(&frame, ReplyKind::Query) {
            lock(&shared.telemetry).absorb(fb);
        }
    }

    /// Render the dashboard into a buffer and return it as text.
    fn render(shared: &Shared) -> String {
        let mut out = Vec::new();
        draw(&mut out, shared, "/dev/ttyUSB0", 0x01).expect("render into a Vec cannot fail");
        String::from_utf8_lossy(&out).into_owned()
    }

    #[test]
    fn braking_is_shown_only_when_the_motor_reports_velocity_mode() {
        // The brake byte is honoured only in velocity mode. Showing BRAKING
        // on the strength of our own request is how a "brake" key ends up
        // freewheeling a wheel while the screen insists it is braking.
        let shared = Shared::new();
        lock(&shared.cmd).brake = true;
        seed(&shared, 0x01, 0); // motor reports CURRENT
        assert!(
            !render(&shared).contains("BRAKING"),
            "claimed BRAKING while the motor was in current mode"
        );

        // Same flag, but now the motor confirms velocity mode.
        let shared = Shared::new();
        lock(&shared.cmd).brake = true;
        seed(&shared, 0x02, 0); // motor reports VELOCITY
        assert!(render(&shared).contains("BRAKING"));
    }

    #[test]
    fn a_mode_disagreement_is_surfaced_not_hidden() {
        // We think velocity; the motor says position. The operator has to be
        // able to see that, because a setpoint means something different in
        // each mode.
        let shared = Shared::new();
        seed(&shared, 0x03, 0);
        let out = render(&shared);
        assert!(out.contains("VELOCITY"), "requested mode missing");
        assert!(out.contains("POSITION"), "motor's actual mode missing");

        // When they agree, only the one mode is named.
        let shared = Shared::new();
        seed(&shared, 0x02, 0);
        let out = render(&shared);
        assert!(out.contains("VELOCITY"));
        assert!(!out.contains("motor:"), "no disagreement to report");
    }

    #[test]
    fn telemetry_is_awaited_rather_than_invented() {
        // Nothing has replied yet: the dashboard must say so instead of
        // rendering a default-looking 0 RPM / 0 A readout.
        let shared = Shared::new();
        let out = render(&shared);
        assert!(out.contains("Waiting for telemetry"));
        assert!(!out.contains("STATIONARY"));
    }
}
