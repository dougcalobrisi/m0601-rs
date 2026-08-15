//! `raw` — send an arbitrary frame (9 bytes = CRC auto-added, or 10).

use std::process::ExitCode;
use std::time::Duration;

use m0601::M0601;
use m0601::protocol::{Frame, ReplyKind, frame_from_bytes, parse_feedback};

/// Tokenize `"01 74,00 ..."` (spaces and/or commas, optional `0x` prefixes)
/// into a frame; the library appends the CRC when 9 bytes are given and
/// enforces the length rule.
fn parse_hex_frame(input: &str) -> Result<Frame, String> {
    let mut bytes = Vec::new();
    for tok in input.replace(',', " ").split_whitespace() {
        let tok = tok
            .strip_prefix("0x")
            .or_else(|| tok.strip_prefix("0X"))
            .unwrap_or(tok);
        let b = u8::from_str_radix(tok, 16).map_err(|e| format!("bad hex byte {tok:?}: {e}"))?;
        bytes.push(b);
    }
    frame_from_bytes(&bytes).map_err(|e| format!("{e}. Provide 9 bytes (CRC auto-added) or 10."))
}

fn hex_upper(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// The command byte (frame byte 1) of a frame that can put the motor in
/// motion: `0x64` drives, `0xA0` switches mode. Feedback (`0x74`) and the
/// special unaddressed frames cannot, so they need no `--yes`.
fn is_motion_command(frame: &Frame) -> bool {
    matches!(frame[1], 0x64 | 0xA0)
}

pub fn run(port: &str, id: u8, timeout: Duration, hex: &str, yes: bool) -> m0601::Result<ExitCode> {
    let frame = match parse_hex_frame(hex) {
        Ok(f) => f,
        Err(msg) => {
            eprintln!("[x] {msg}");
            return Ok(ExitCode::FAILURE);
        }
    };

    // `raw` has none of `drive`'s rails (StopGuard, position-entry check), so
    // gate the two command bytes that can move the wheel behind --yes. Note
    // --id does not alter the literal bytes: byte 0 is whatever was typed.
    let motion = is_motion_command(&frame);
    if motion && !yes {
        eprintln!(
            "[x] {} is a motion command (byte 1 = 0x{:02X}); pass --yes to send it.",
            hex_upper(&frame),
            frame[1]
        );
        eprintln!("    It can move the motor and `raw` has no stop guard.");
        return Ok(ExitCode::FAILURE);
    }

    let mut motor = M0601::open(port, id, timeout)?;
    println!("TX: {}", hex_upper(&frame));
    let resp = motor.send_raw(&frame, timeout.max(Duration::from_millis(200)))?;
    if resp.is_empty() {
        println!("RX: (no response)");
    } else {
        println!("RX: {}", hex_upper(&resp));
        // The reply layout depends on the command that was sent: only decode
        // when the TX frame is one that elicits telemetry, using its layout.
        if let Some(fb) = ReplyKind::from_tx(&frame).and_then(|kind| parse_feedback(&resp, kind)) {
            let temp = fb
                .temp_c
                .map_or_else(|| "--".to_owned(), |t| format!("{t}C"));
            println!(
                "    decoded -> mode {}, {} RPM, {:.3} A, {:.1} deg, temp {temp}, err {}",
                fb.mode_name(),
                fb.speed_rpm,
                fb.current_a,
                fb.position_deg,
                fb.faults
            );
        }
    }

    // A single drive frame coasts by protocol, but a current-mode frame is a
    // torque impulse; a mode switch leaves the motor armed. Brake on the way
    // out so `raw` never exits having nudged the wheel into a live state.
    if motion {
        motor.safe_stop();
        println!("(motion frame — braked on exit)");
    }
    Ok(ExitCode::SUCCESS)
}

#[cfg(test)]
mod tests {
    use super::parse_hex_frame;

    #[test]
    fn nine_bytes_get_crc_appended() {
        let f = parse_hex_frame("01 74 00 00 00 00 00 00 00").unwrap();
        assert_eq!(f, [0x01, 0x74, 0, 0, 0, 0, 0, 0, 0, 0x04]);
    }

    #[test]
    fn ten_bytes_pass_through() {
        let f = parse_hex_frame("01 74 00 00 00 00 00 00 00 FF").unwrap();
        assert_eq!(f[9], 0xFF); // not recomputed
    }

    #[test]
    fn commas_and_prefixes_accepted() {
        let f = parse_hex_frame("0x01,0x74, 00 00 00 00 00 00 00").unwrap();
        assert_eq!(f.len(), 10);
        assert_eq!(f[0], 0x01);
    }

    #[test]
    fn wrong_length_rejected() {
        assert!(parse_hex_frame("01 74").is_err());
        assert!(parse_hex_frame("").is_err());
        assert!(parse_hex_frame("01 02 03 04 05 06 07 08 09 0A 0B").is_err());
    }

    #[test]
    fn bad_hex_rejected() {
        assert!(parse_hex_frame("01 74 00 00 00 00 00 00 GG").is_err());
    }

    #[test]
    fn motion_commands_are_recognised_by_byte_1() {
        use super::is_motion_command;
        // 0x64 drive, 0xA0 mode switch → gated behind --yes.
        assert!(is_motion_command(&parse_hex_frame("01 64 00 64 00 00 01 00 00").unwrap()));
        assert!(is_motion_command(&parse_hex_frame("01 A0 00 00 00 00 00 00 01").unwrap()));
        // 0x74 feedback query → read-only, no gate.
        assert!(!is_motion_command(&parse_hex_frame("01 74 00 00 00 00 00 00 00").unwrap()));
    }
}
