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
    CUR_MAX, CUR_MIN, crc8_maxim, frame_brake, frame_current, frame_feedback, frame_from_bytes,
    frame_id_query, frame_mode, frame_position, frame_set_id, frame_velocity, parse_feedback,
    validate_id,
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
fn parse_feedback_golden_vector() {
    let fb = parse_feedback(&[0x01, 0x02, 0xF8, 0x30, 0x00, 0x64, 0x28, 0x80, 0x03, 0x00])
        .expect("valid frame parses");
    assert_eq!(fb.id, 1);
    assert_eq!(fb.mode, Some(Mode::Velocity));
    assert_eq!(fb.mode_raw, 0x02);
    // 0xF830 = -1998; -1998 * 8 / 32767 = -0.4879... A. Compared at the
    // 3 decimals the CLI displays.
    assert_eq!((fb.current_a * 1000.0).round() / 1000.0, -0.488);
    assert_eq!(fb.speed_rpm, 100);
    assert_eq!(fb.temp_c, 40);
    // 0x80 = 128; 128 * 360 / 255 = 180.70... deg.
    assert_eq!((fb.position_deg * 10.0).round() / 10.0, 180.7);
    assert_eq!(fb.faults, Faults(0x03));
    assert_eq!(fb.faults.to_string(), "SensorErr | Overcurrent");
    assert!(!fb.crc_ok);
    assert_eq!(fb.raw_hex(), "01 02 F8 30 00 64 28 80 03 00");
    assert_eq!(fb.mode_name(), "Velocity");
}

#[test]
fn parse_feedback_length_handling() {
    // Too short: no telemetry.
    assert!(parse_feedback(&[0x01; 9]).is_none());
    assert!(parse_feedback(&[]).is_none());
    // Longer input parses its first 10 bytes.
    let mut long = vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00];
    long.extend_from_slice(&[0xAA, 0xBB]);
    let fb = parse_feedback(&long).expect("first 10 bytes parse");
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
    assert!(parse_feedback(&good).unwrap().crc_ok);

    // A frame whose byte 9 is *not* a CRC-8/MAXIM — the normal case for
    // real motor replies — still parses, with crc_ok = false.
    let mut bad = good;
    bad[9] ^= 0xFF;
    let fb = parse_feedback(&bad).unwrap();
    assert!(!fb.crc_ok);
    assert_eq!(fb.speed_rpm, 100, "telemetry is never rejected on CRC");
}

#[test]
fn faults_display_parity() {
    assert_eq!(Faults(0x00).to_string(), "OK");
    assert_eq!(Faults(0x01).to_string(), "SensorErr");
    assert_eq!(
        Faults(0x1F).to_string(),
        "SensorErr | Overcurrent | PhaseOvercurrent | Stall | Troubleshoot"
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
        "SensorErr | Overcurrent | PhaseOvercurrent | Stall | Troubleshoot | 0xE0"
    );
    assert!(Faults(0x01).sensor_err());
    assert!(Faults(0x02).overcurrent());
    assert!(Faults(0x04).phase_overcurrent());
    assert!(Faults(0x08).stall());
    assert!(Faults(0x10).troubleshoot());
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
