//! `raw` — send an arbitrary frame (9 bytes = CRC auto-added, or 10).

use std::process::ExitCode;
use std::time::Duration;

use m0601::protocol::{Frame, ReplyKind, frame_from_bytes, parse_feedback, validate_id};
use m0601::{Bus, Transport};

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
/// motion: `0x64` drives, `0xA0` switches mode.
///
/// This keys on byte 1 alone, so it is deliberately broader than "frames that
/// will actually move something": the broadcast ID query (`C8 64 00…DE`) is
/// gated too. That is intended — `raw` sends the bytes the user typed, and
/// `C8 64` with a non-zero value is a drive command to *every* motor on the
/// bus, so the harmless broadcast query cannot be ungated without ungating
/// that as well. `scan` is the safe way to issue a broadcast query.
///
/// Feedback (`0x74`) and set-ID (byte 1 = `0x55`) need no `--yes`.
fn is_motion_command(frame: &Frame) -> bool {
    matches!(frame[1], 0x64 | 0xA0)
}

/// The brake target for a motion frame: byte 0 of the typed frame when it is
/// a valid unicast address, so the brake goes to the motor the frame actually
/// commanded. `0xC8` — the broadcast destination — and the reserved addresses
/// fall back to `--id`: a broadcast drive commands every motor, and a unicast
/// brake can only ever cover one of them.
fn brake_target(frame: &Frame, id: u8) -> u8 {
    if frame[0] != 0xC8 && validate_id(frame[0]).is_ok() {
        frame[0]
    } else {
        id
    }
}

/// The line printed after the exit brake, describing *which* motor it reached.
///
/// Keyed on whether [`brake_target`] took byte 0 or fell back to `--id`, not on
/// byte 0 alone: `0x00` and `0xFF` are reserved, not broadcast, yet they fall
/// back exactly as `0xC8` does, and claiming the brake reached the addressed
/// motor would be wrong for all three.
fn exit_brake_note(frame: &Frame, brake_id: u8) -> String {
    if brake_id == frame[0] {
        format!("(motion frame — braked motor 0x{brake_id:02X} on exit)")
    } else if frame[0] == 0xC8 {
        format!("(broadcast motion frame — braked only motor 0x{brake_id:02X}; other motors coast)")
    } else {
        format!(
            "(frame addressed 0x{:02X}, not a unicast ID — braked the --id motor 0x{brake_id:02X} on exit)",
            frame[0]
        )
    }
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
        eprintln!("    It can move the motor and `raw` has none of `drive`'s rails.");
        return Ok(ExitCode::FAILURE);
    }

    let bus = Bus::open(port, timeout)?;
    run_on_bus(&bus, id, &frame, motion, timeout)
}

/// Everything `run` does after the frame is parsed, gated, and the port is
/// open — generic over the transport so the exit-brake behaviour is testable
/// against a [`MockTransport`](m0601::MockTransport) (the bus mints motor
/// handles on the already-open port).
fn run_on_bus<T: Transport>(
    bus: &Bus<T>,
    id: u8,
    frame: &Frame,
    motion: bool,
    timeout: Duration,
) -> m0601::Result<ExitCode> {
    let mut motor = bus.motor(id)?;
    let brake_id = brake_target(frame, id);

    println!("TX: {}", hex_upper(frame));
    let resp = match motor.send_raw(frame, timeout.max(Duration::from_millis(200))) {
        Ok(resp) => resp,
        // The frame may already be on the wire when the reply read fails, so
        // a motion frame still gets its brake before the error propagates.
        Err(e) => {
            if motion && let Ok(mut target) = bus.motor(brake_id) {
                target.safe_stop();
                eprintln!(
                    "(motion frame — attempted a best-effort brake to motor 0x{brake_id:02X} despite the error)"
                );
            }
            return Err(e);
        }
    };
    if resp.is_empty() {
        println!("RX: (no response)");
    } else {
        println!("RX: {}", hex_upper(&resp));
        // The reply layout depends on the command that was sent: only decode
        // when the TX frame is one that elicits telemetry, using its layout.
        if let Some(fb) = ReplyKind::from_tx(frame).and_then(|kind| parse_feedback(&resp, kind)) {
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
    // out rather than exit having nudged the wheel into a live state.
    if motion {
        if let Ok(mut target) = bus.motor(brake_id) {
            target.safe_stop();
        }
        println!("{}", exit_brake_note(frame, brake_id));
    }
    Ok(ExitCode::SUCCESS)
}

#[cfg(test)]
mod tests {
    use std::process::ExitCode;
    use std::time::Duration;

    use m0601::protocol::frame_brake;
    use m0601::{Bus, MockTransport};

    use super::{brake_target, exit_brake_note, parse_hex_frame, run_on_bus};

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
        assert!(is_motion_command(
            &parse_hex_frame("01 64 00 64 00 00 01 00 00").unwrap()
        ));
        assert!(is_motion_command(
            &parse_hex_frame("01 A0 00 00 00 00 00 00 01").unwrap()
        ));
        // 0x74 feedback query → read-only, no gate.
        assert!(!is_motion_command(
            &parse_hex_frame("01 74 00 00 00 00 00 00 00").unwrap()
        ));
    }

    #[test]
    fn brake_target_prefers_the_addressed_motor() {
        let unicast = parse_hex_frame("02 64 00 64 00 00 01 00 00").unwrap();
        assert_eq!(brake_target(&unicast, 0x01), 0x02);
        // Broadcast destination and the reserved addresses fall back to --id.
        let broadcast = parse_hex_frame("C8 64 00 64 00 00 01 00 00").unwrap();
        assert_eq!(brake_target(&broadcast, 0x01), 0x01);
        let reserved_zero = parse_hex_frame("00 64 00 64 00 00 01 00 00").unwrap();
        assert_eq!(brake_target(&reserved_zero, 0x03), 0x03);
        let reserved_ff = parse_hex_frame("FF 64 00 64 00 00 01 00 00").unwrap();
        assert_eq!(brake_target(&reserved_ff, 0x03), 0x03);
    }

    #[test]
    fn exit_brake_note_never_claims_a_fallback_brake_hit_the_addressed_motor() {
        let unicast = parse_hex_frame("02 64 00 64 00 00 01 00 00").unwrap();
        assert_eq!(
            exit_brake_note(&unicast, brake_target(&unicast, 0x01)),
            "(motion frame — braked motor 0x02 on exit)"
        );
        let broadcast = parse_hex_frame("C8 64 00 64 00 00 01 00 00").unwrap();
        assert_eq!(
            exit_brake_note(&broadcast, brake_target(&broadcast, 0x01)),
            "(broadcast motion frame — braked only motor 0x01; other motors coast)"
        );
        // The reserved addresses fall back too, and must say so rather than
        // reporting a brake on the address that was typed.
        for hex in ["00 64 00 64 00 00 01 00 00", "FF 64 00 64 00 00 01 00 00"] {
            let reserved = parse_hex_frame(hex).unwrap();
            let note = exit_brake_note(&reserved, brake_target(&reserved, 0x03));
            assert!(
                note.contains("not a unicast ID") && note.contains("0x03"),
                "reserved address note was {note:?}"
            );
        }
    }

    const TIMEOUT: Duration = Duration::from_millis(50);

    /// `safe_stop` is fifteen frames: five velocity-mode switches, five
    /// velocity-0 drives, five brakes — see `M0601::safe_stop`.
    const SAFE_STOP_FRAME_COUNT: usize = 15;

    /// Index of the first of `safe_stop`'s five closing brake frames in
    /// `MockTransport::sent`, after the one raw frame the test itself sends.
    const BRAKE_FRAMES_START: usize = 1 + SAFE_STOP_FRAME_COUNT - 5;

    /// Run `run_on_bus` over a mock and hand back the mock for inspection.
    fn run_on_mock(
        mock: MockTransport,
        hex: &str,
        id: u8,
        motion: bool,
    ) -> (m0601::Result<ExitCode>, MockTransport) {
        let frame = parse_hex_frame(hex).unwrap();
        let bus = Bus::with_transport(mock, TIMEOUT);
        let result = run_on_bus(&bus, id, &frame, motion, TIMEOUT);
        let mock = bus
            .into_transport()
            .expect("no motor handles outlive run_on_bus");
        (result, mock)
    }

    #[test]
    fn unicast_motion_frame_brakes_the_addressed_motor_not_dash_id() {
        // Byte 0 addresses motor 0x02 while --id is 0x01: the exit brake
        // must chase the motor the frame actually commanded.
        let (result, mock) = run_on_mock(
            MockTransport::default(),
            "02 64 00 64 00 00 01 00 00",
            0x01,
            true,
        );
        assert!(result.is_ok());
        assert_eq!(mock.sent.len(), 1 + SAFE_STOP_FRAME_COUNT);
        assert_eq!(mock.sent[0][0], 0x02); // the typed frame itself
        for (i, sent) in mock.sent[1..].iter().enumerate() {
            assert_eq!(
                sent[0], 0x02,
                "safe_stop frame {i} addressed to 0x{:02X}",
                sent[0]
            );
        }
        // The sequence ends in actual brake frames, addressed the same way.
        for sent in &mock.sent[BRAKE_FRAMES_START..] {
            assert_eq!(sent, &frame_brake(0x02).to_vec());
        }
    }

    #[test]
    fn broadcast_motion_frame_brakes_the_dash_id_motor() {
        // 0xC8 commands every motor; the unicast brake falls back to --id.
        let (result, mock) = run_on_mock(
            MockTransport::default(),
            "C8 64 00 64 00 00 01 00 00",
            0x05,
            true,
        );
        assert!(result.is_ok());
        assert_eq!(mock.sent.len(), 1 + SAFE_STOP_FRAME_COUNT);
        for sent in &mock.sent[1..] {
            assert_eq!(sent[0], 0x05);
        }
        for sent in &mock.sent[BRAKE_FRAMES_START..] {
            assert_eq!(sent, &frame_brake(0x05).to_vec());
        }
    }

    #[test]
    fn non_motion_frame_sends_no_brake() {
        let (result, mock) = run_on_mock(
            MockTransport::default(),
            "01 74 00 00 00 00 00 00 00",
            0x01,
            false,
        );
        assert!(result.is_ok());
        assert_eq!(mock.sent.len(), 1); // just the query, no stop sequence
    }

    #[test]
    fn failed_exchange_still_brakes_before_the_error_propagates() {
        // fail_io makes every operation error while still recording the
        // frame first, so this exercises the error branch of run_on_bus:
        // the brake sequence must be attempted before the error surfaces.
        let mock = MockTransport {
            fail_io: true,
            ..MockTransport::default()
        };
        let (result, mock) = run_on_mock(mock, "02 64 00 64 00 00 01 00 00", 0x01, true);
        assert!(result.is_err());
        assert_eq!(mock.sent.len(), 1 + SAFE_STOP_FRAME_COUNT);
        for sent in &mock.sent[1..] {
            assert_eq!(sent[0], 0x02); // still aimed at the addressed motor
        }
    }
}
