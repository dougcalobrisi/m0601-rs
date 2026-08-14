//! Golden-vector tests for the pure protocol layer.
//!
//! Every expected value here is a literal, derived from the DFRobot frame
//! layout and the CRC-8/MAXIM specification rather than produced by the
//! code under test. Byte-for-byte wire parity is the contract, so an
//! assertion that recomputes its expectation with the function it is
//! testing would be worthless — see [`crc8_maxim_matches_the_published_spec`].

// Test helpers may assert; the workspace no-panic lints target library code.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use m0601::protocol::{
    CUR_MAX, CUR_MIN, POS_MAX, ReplyKind, crc8_maxim, frame_brake, frame_current, frame_feedback,
    frame_from_bytes, frame_id_query, frame_mode, frame_position, frame_set_id, frame_velocity,
    parse_feedback, validate_id,
};
use m0601::{Error, Faults, Mode};

#[test]
fn crc8_maxim_matches_the_published_spec() {
    // CRC-8/MAXIM (Dallas/1-Wire): poly 0x31 reflected (0x8C), init 0x00,
    // refin/refout true, xorout 0x00. Its published check value — the CRC of
    // the ASCII string "123456789" — is 0xA1. This one constant is what
    // anchors every other vector in this file to an external definition.
    assert_eq!(crc8_maxim(b"123456789"), 0xA1);

    assert_eq!(crc8_maxim(&[]), 0x00);
    assert_eq!(crc8_maxim(&[0, 1, 2, 3, 4, 5, 6, 7, 8]), 0x83);
    // The fixed broadcast ID-query frame's trailing byte is its CRC.
    assert_eq!(crc8_maxim(&[0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0]), 0xDE);
}

#[test]
fn feedback_query_frame() {
    assert_eq!(
        frame_feedback(0x01),
        [0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
    );
}

#[test]
fn velocity_frames() {
    assert_eq!(
        frame_velocity(0x01, 100, 1),
        [0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00, 0x00, 0xE4]
    );
    assert_eq!(
        frame_velocity(0x01, -330, 1),
        [0x01, 0x64, 0xFE, 0xB6, 0x00, 0x00, 0x01, 0x00, 0x00, 0x75]
    );
}

#[test]
fn velocity_clamps_out_of_range() {
    // 500 RPM clamps to +330 (0x014A) — clamping is API contract.
    assert_eq!(
        frame_velocity(0x01, 500, 1),
        [0x01, 0x64, 0x01, 0x4A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x7C]
    );
    // -500 clamps to -330 (0xFEB6). Spelled out rather than compared to
    // frame_velocity(-330), which would hold even if the clamp were wrong.
    assert_eq!(
        frame_velocity(0x01, -500, 1),
        [0x01, 0x64, 0xFE, 0xB6, 0x00, 0x00, 0x01, 0x00, 0x00, 0x75]
    );
    // Saturating at the type boundary, not wrapping to full forward.
    assert_eq!(
        frame_velocity(0x01, i16::MIN, 1),
        [0x01, 0x64, 0xFE, 0xB6, 0x00, 0x00, 0x01, 0x00, 0x00, 0x75]
    );
}

#[test]
fn accel_byte_is_carried_verbatim() {
    // Byte 6 is the acceleration; 0 means "motor default".
    assert_eq!(
        frame_velocity(0x01, 100, 20),
        [0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x14, 0x00, 0x00, 0x9B]
    );
    assert_eq!(
        frame_velocity(0x01, 100, 0),
        [0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F]
    );
}

/// Known-answer frames from two independent implementations: the
/// `Il1yasviel/navigation_robot` C driver's unit tests and the MotorLink
/// README's captured command reference (all with accel = 0). Agreement here
/// is agreement with code that has driven real hardware.
#[test]
fn community_known_answer_velocity_frames() {
    for (rpm, expected) in [
        (30, [0x01, 0x64, 0x00, 0x1E, 0, 0, 0, 0, 0, 0x18]),
        (50, [0x01, 0x64, 0x00, 0x32, 0, 0, 0, 0, 0, 0xD3]),
        (100, [0x01, 0x64, 0x00, 0x64, 0, 0, 0, 0, 0, 0x4F]),
        (150, [0x01, 0x64, 0x00, 0x96, 0, 0, 0, 0, 0, 0x53]),
        (0, [0x01, 0x64, 0x00, 0x00, 0, 0, 0, 0, 0, 0x50]),
        (-50, [0x01, 0x64, 0xFF, 0xCE, 0, 0, 0, 0, 0, 0xDA]),
        (-100, [0x01, 0x64, 0xFF, 0x9C, 0, 0, 0, 0, 0, 0x9A]),
        (-150, [0x01, 0x64, 0xFF, 0x6A, 0, 0, 0, 0, 0, 0x5A]),
    ] {
        assert_eq!(frame_velocity(0x01, rpm, 0), expected, "{rpm} RPM");
    }
}

/// See [`community_known_answer_velocity_frames`] for provenance.
#[test]
fn community_known_answer_current_frames() {
    for (value, expected) in [
        (-10000, [0x01, 0x64, 0xD8, 0xF0, 0, 0, 0, 0, 0, 0x78]),
        (-5000, [0x01, 0x64, 0xEC, 0x78, 0, 0, 0, 0, 0, 0xD3]),
        (2000, [0x01, 0x64, 0x07, 0xD0, 0, 0, 0, 0, 0, 0x27]),
        (5000, [0x01, 0x64, 0x13, 0x88, 0, 0, 0, 0, 0, 0xA7]),
        (10000, [0x01, 0x64, 0x27, 0x10, 0, 0, 0, 0, 0, 0x57]),
    ] {
        assert_eq!(frame_current(0x01, value), expected, "current {value}");
    }
}

/// See [`community_known_answer_velocity_frames`] for provenance.
/// 8192/32767 ≈ 90°, 10000 ≈ 109.9°, 20000 ≈ 219.7°, 30000 ≈ 329.6°.
#[test]
fn community_known_answer_position_frames() {
    for (raw, expected) in [
        (8192, [0x01, 0x64, 0x20, 0x00, 0, 0, 0, 0, 0, 0xBF]),
        (10000, [0x01, 0x64, 0x27, 0x10, 0, 0, 0, 0, 0, 0x57]),
        (20000, [0x01, 0x64, 0x4E, 0x20, 0, 0, 0, 0, 0, 0x5E]),
        (30000, [0x01, 0x64, 0x75, 0x30, 0, 0, 0, 0, 0, 0xA7]),
    ] {
        assert_eq!(frame_position(0x01, raw), expected, "position {raw}");
    }
}

#[test]
fn brake_frame() {
    assert_eq!(
        frame_brake(0x01),
        [0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1]
    );
}

#[test]
fn current_frame() {
    assert_eq!(
        frame_current(0x01, -1234),
        [0x01, 0x64, 0xFB, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07]
    );
}

#[test]
fn current_clamps_at_its_symmetric_limits() {
    assert_eq!(
        frame_current(0x01, CUR_MAX),
        [0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97]
    );
    assert_eq!(
        frame_current(0x01, CUR_MIN),
        [0x01, 0x64, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0]
    );
    // i16::MIN (-32768) is outside the symmetric range and must clamp to
    // CUR_MIN rather than reaching the wire as 0x8000.
    assert_eq!(
        frame_current(0x01, i16::MIN),
        [0x01, 0x64, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0]
    );
}

#[test]
fn position_frames() {
    assert_eq!(
        frame_position(0x01, 32767),
        [0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97]
    );
    assert_eq!(
        frame_position(0x01, 0),
        [0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50]
    );
    // Values above POS_MAX clamp down to it (0x7FFF), spelled out so the
    // assertion cannot be satisfied by two equally-wrong results.
    assert_eq!(
        frame_position(0x01, u16::MAX),
        [0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97]
    );
}

#[test]
fn frame_from_bytes_appends_crc_or_passes_through() {
    // 9 bytes: CRC computed and appended.
    assert_eq!(
        frame_from_bytes(&[0x01, 0x74, 0, 0, 0, 0, 0, 0, 0]).unwrap(),
        [0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
    );
    // 10 bytes: byte 9 preserved verbatim, so deliberately corrupt frames
    // can be sent for protocol probing.
    assert_eq!(
        frame_from_bytes(&[0x01, 0x74, 0, 0, 0, 0, 0, 0, 0, 0xEE]).unwrap()[9],
        0xEE
    );
    assert!(matches!(
        frame_from_bytes(&[0x01, 0x74]),
        Err(Error::InvalidFrameLen(2))
    ));
    assert!(matches!(
        frame_from_bytes(&[]),
        Err(Error::InvalidFrameLen(0))
    ));
    assert!(matches!(
        frame_from_bytes(&[0u8; 11]),
        Err(Error::InvalidFrameLen(11))
    ));
}

#[test]
fn mode_frame_has_mode_byte_not_crc() {
    assert_eq!(
        frame_mode(0x01, Mode::Velocity),
        [0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]
    );
    assert_eq!(frame_mode(0x01, Mode::Current)[9], 0x01);
    assert_eq!(frame_mode(0x01, Mode::Position)[9], 0x03);
}

#[test]
fn id_query_frame_is_fixed() {
    assert_eq!(
        frame_id_query(),
        [0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE]
    );
}

#[test]
fn set_id_frame_layout_and_validation() {
    assert_eq!(
        frame_set_id(0x05).unwrap(),
        [0xAA, 0x55, 0x53, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    );
    assert!(matches!(frame_set_id(0x00), Err(Error::InvalidId(0x00))));
    assert!(matches!(frame_set_id(0xFF), Err(Error::InvalidId(0xFF))));
    assert!(validate_id(0x01).is_ok());
    assert!(validate_id(0xFE).is_ok());
}

#[test]
fn parse_feedback_query_golden_vector() {
    let fb = parse_feedback(
        &[0x01, 0x02, 0xF8, 0x30, 0x00, 0x64, 0x28, 0x80, 0x03, 0x00],
        ReplyKind::Query,
    )
    .expect("valid frame parses");
    assert_eq!(fb.id, 1);
    assert_eq!(fb.kind, ReplyKind::Query);
    assert_eq!(fb.mode, Some(Mode::Velocity));
    assert_eq!(fb.mode_raw, 0x02);
    // 0xF830 = -1998; -1998 * 8 / 32767 = -0.4879... A. Compared at the
    // 3 decimals the CLI displays.
    assert_eq!((fb.current_a * 1000.0).round() / 1000.0, -0.488);
    assert_eq!(fb.speed_rpm, 100);
    assert_eq!(fb.temp_c, Some(40));
    // 0x80 = 128; 128 * 360 / 255 = 180.70... deg.
    assert_eq!((fb.position_deg * 10.0).round() / 10.0, 180.7);
    assert_eq!(fb.faults, Faults(0x03));
    assert_eq!(fb.faults.to_string(), "SensorErr | Overcurrent");
    assert!(!fb.crc_ok);
    assert_eq!(fb.raw_hex(), "01 02 F8 30 00 64 28 80 03 00");
    assert_eq!(fb.mode_name(), "Velocity");
}

/// The same bytes as [`parse_feedback_query_golden_vector`] decoded as a
/// drive reply — this pair of tests *is* the dual-layout contract: bytes
/// 6–7 (`0x28 0x80`) are one 16-bit position, and no temperature exists.
#[test]
fn parse_feedback_drive_golden_vector() {
    let fb = parse_feedback(
        &[0x01, 0x02, 0xF8, 0x30, 0x00, 0x64, 0x28, 0x80, 0x03, 0x00],
        ReplyKind::Drive,
    )
    .expect("valid frame parses");
    assert_eq!(fb.kind, ReplyKind::Drive);
    assert_eq!(fb.temp_c, None, "drive replies carry no temperature");
    // 0x2880 = 10368; 10368 * 360 / 32767 = 113.90... deg.
    assert_eq!((fb.position_deg * 10.0).round() / 10.0, 113.9);
    // Common fields decode identically to the query layout.
    assert_eq!(fb.speed_rpm, 100);
    assert_eq!((fb.current_a * 1000.0).round() / 1000.0, -0.488);
    assert_eq!(fb.faults, Faults(0x03));
}

#[test]
fn parse_feedback_drive_position_endpoints() {
    let mut frame = [0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x00, 0x00];
    let fb = parse_feedback(&frame, ReplyKind::Drive).unwrap();
    assert_eq!(fb.position_deg, 360.0, "0x7FFF is a full turn");
    frame[6] = 0x00;
    frame[7] = 0x00;
    let fb = parse_feedback(&frame, ReplyKind::Drive).unwrap();
    assert_eq!(fb.position_deg, 0.0);
}

#[test]
fn parse_feedback_drive_position_clamps_out_of_range() {
    // The drive-reply position is documented as 0..=32767. A corrupt frame
    // with bit 15 set would decode, unclamped, to up to ~720°; the decoder
    // clamps to POS_MAX so telemetry never leaves the 0..=360° range — even
    // in the default advisory-CRC mode, which passes bad frames through.
    for raw in [0x8000u16, 0xC000, 0xFFFF] {
        let [hi, lo] = raw.to_be_bytes();
        let frame = [0x01, 0x02, 0x00, 0x00, 0x00, 0x00, hi, lo, 0x00, 0x00];
        let fb = parse_feedback(&frame, ReplyKind::Drive).unwrap();
        assert!(
            (0.0..=360.0).contains(&fb.position_deg),
            "0x{raw:04X} decoded to {} deg, outside 0..=360",
            fb.position_deg
        );
        assert_eq!(fb.position_deg, 360.0, "clamps to POS_MAX (a full turn)");
    }
}

/// The reply layout is knowable only from the frame that elicited it; this
/// pins the classification for every TX frame the crate can produce.
#[test]
fn reply_kind_classification() {
    assert_eq!(
        ReplyKind::from_tx(&frame_feedback(1)),
        Some(ReplyKind::Query)
    );
    assert_eq!(
        ReplyKind::from_tx(&frame_velocity(1, 100, 1)),
        Some(ReplyKind::Drive)
    );
    assert_eq!(
        ReplyKind::from_tx(&frame_current(1, 100)),
        Some(ReplyKind::Drive)
    );
    assert_eq!(
        ReplyKind::from_tx(&frame_position(1, 100)),
        Some(ReplyKind::Drive)
    );
    assert_eq!(ReplyKind::from_tx(&frame_brake(1)), Some(ReplyKind::Drive));
    // The broadcast ID query's command byte is 0x64 — drive layout.
    assert_eq!(
        ReplyKind::from_tx(&frame_id_query()),
        Some(ReplyKind::Drive)
    );
    // Frames that elicit no telemetry classify as None.
    assert_eq!(ReplyKind::from_tx(&frame_mode(1, Mode::Velocity)), None);
    assert_eq!(ReplyKind::from_tx(&frame_set_id(0x05).unwrap()), None);
    assert_eq!(ReplyKind::from_tx(&[0x01]), None);
    assert_eq!(ReplyKind::from_tx(&[]), None);
}

#[test]
fn parse_feedback_length_handling() {
    // Too short: no telemetry.
    assert!(parse_feedback(&[0x01; 9], ReplyKind::Query).is_none());
    assert!(parse_feedback(&[], ReplyKind::Query).is_none());
    // Longer input parses its first 10 bytes.
    let mut long = vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00];
    long.extend_from_slice(&[0xAA, 0xBB]);
    let fb = parse_feedback(&long, ReplyKind::Query).expect("first 10 bytes parse");
    assert_eq!(fb.speed_rpm, 100);
}

#[test]
fn parse_feedback_never_rejects_on_crc() {
    // Byte 9 below is 0x0D — the literal CRC-8/MAXIM of the preceding nine
    // bytes, written out rather than computed here. Calling crc8_maxim() to
    // build the expectation would make this test pass for *any*
    // implementation of it, correct or not.
    let good = [0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x0D];
    assert_eq!(crc8_maxim(&good[..9]), 0x0D, "vector is self-consistent");
    assert!(parse_feedback(&good, ReplyKind::Query).unwrap().crc_ok);
    // crc_ok is computed identically for both layouts.
    assert!(parse_feedback(&good, ReplyKind::Drive).unwrap().crc_ok);

    // A frame whose byte 9 is *not* a valid CRC-8/MAXIM still parses, with
    // crc_ok = false — the reply CRC is advisory, never grounds to reject.
    let mut bad = good;
    bad[9] ^= 0xFF;
    for kind in [ReplyKind::Query, ReplyKind::Drive] {
        let fb = parse_feedback(&bad, kind).unwrap();
        assert!(!fb.crc_ok);
        assert_eq!(fb.speed_rpm, 100, "telemetry is never rejected on CRC");
    }
}

#[test]
fn faults_display_parity() {
    assert_eq!(Faults(0x00).to_string(), "OK");
    assert_eq!(Faults(0x01).to_string(), "SensorErr");
    assert_eq!(
        Faults(0x1F).to_string(),
        "SensorErr | Overcurrent | PhaseOvercurrent | Stall | Overheat"
    );
    // Unknown bits only: hex fallback.
    assert_eq!(Faults(0x20).to_string(), "0x20");
    // Known and unknown together: neither is dropped. A motor reporting a
    // bit this crate doesn't recognise must not have it silently hidden
    // behind whichever known bit happens to share the byte.
    assert_eq!(Faults(0x21).to_string(), "SensorErr | 0x20");
    assert_eq!(Faults(0xE0).to_string(), "0xE0");
    assert_eq!(
        Faults(0xFF).to_string(),
        "SensorErr | Overcurrent | PhaseOvercurrent | Stall | Overheat | 0xE0"
    );
    assert!(Faults(0x01).sensor_err());
    assert!(Faults(0x02).overcurrent());
    assert!(Faults(0x04).phase_overcurrent());
    assert!(Faults(0x08).stall());
    assert!(Faults(0x10).overheat());
    assert!(Faults(0).is_ok());
    assert!(!Faults(0x20).is_ok());
}

#[test]
fn mode_conversions() {
    assert_eq!(Mode::from_byte(0x02), Some(Mode::Velocity));
    assert_eq!(Mode::from_byte(0x07), None);
    assert_eq!(Mode::Velocity.as_byte(), 0x02);
    assert_eq!("position".parse::<Mode>().unwrap(), Mode::Position);
    assert_eq!("VELOCITY".parse::<Mode>().unwrap(), Mode::Velocity);
    assert!("sideways".parse::<Mode>().is_err());
}

// ── Unit conversions ────────────────────────────────────────────────────────

#[test]
fn deg_to_raw_round_trips_every_position_step_exactly() {
    use m0601::protocol::{deg_to_raw, raw_to_deg};
    // "Hold this angle" reads an angle out of a drive reply and converts it
    // straight back to a setpoint. If that round trip is ever off by one
    // step, entering position mode commands a small move instead of holding
    // still — so check the whole range, not a handful of samples.
    for raw in 0..=POS_MAX {
        assert_eq!(deg_to_raw(raw_to_deg(raw)), raw, "raw {raw}");
    }
}

#[test]
fn conversions_clamp_at_the_reachable_limits() {
    use m0601::protocol::{amps_to_raw, deg_to_raw, raw_to_amps, raw_to_deg, raw8_to_deg};

    // Endpoints, spelled out rather than recomputed from the functions.
    assert_eq!(deg_to_raw(0.0), 0);
    assert_eq!(deg_to_raw(360.0), 32_767);
    assert_eq!(deg_to_raw(180.0), 16_384);
    assert_eq!(raw_to_deg(0), 0.0);
    assert_eq!(raw_to_deg(32_767), 360.0);
    // The 8-bit query position divides by 255, so 0xFF is a full turn.
    assert_eq!(raw8_to_deg(0), 0.0);
    assert_eq!(raw8_to_deg(255), 360.0);

    assert_eq!(amps_to_raw(0.0), 0);
    assert_eq!(amps_to_raw(8.0), CUR_MAX);
    assert_eq!(amps_to_raw(-8.0), CUR_MIN);
    assert_eq!(amps_to_raw(1.0), 4096);
    assert_eq!(amps_to_raw(-1.0), -4096);
    assert_eq!(raw_to_amps(0), 0.0);

    // Out of band saturates; it must never wrap to the opposite extreme.
    assert_eq!(deg_to_raw(-90.0), 0);
    assert_eq!(deg_to_raw(1_000.0), 32_767);
    assert_eq!(amps_to_raw(100.0), CUR_MAX);
    assert_eq!(amps_to_raw(-100.0), CUR_MIN);

    // Non-finite input must not reach an `as` cast unclamped.
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert_eq!(deg_to_raw(bad), 0);
        assert_eq!(amps_to_raw(bad), 0);
    }
}
