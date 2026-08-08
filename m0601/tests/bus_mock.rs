//! Driver-behavior tests against [`MockTransport`] — no hardware, no timing.

// Test helpers may assert; the workspace no-panic lints target library code.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::time::Duration;

use m0601::protocol::{ReplyKind, frame_feedback, frame_id_query, frame_mode, frame_velocity};
use m0601::{Bus, Error, M0601, MockTransport, Mode};

const TIMEOUT: Duration = Duration::from_millis(150);

fn motor(mock: MockTransport) -> M0601<MockTransport> {
    M0601::with_transport(mock, 0x01, TIMEOUT).expect("valid id")
}

/// A plausible telemetry reply from motor `id`: 100 RPM, 40 °C, no faults.
fn telemetry(id: u8) -> Vec<u8> {
    vec![id, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00]
}

#[test]
fn invalid_ids_rejected_at_construction() {
    assert!(matches!(
        M0601::with_transport(MockTransport::default(), 0x00, TIMEOUT),
        Err(Error::InvalidId(0x00))
    ));
    assert!(matches!(
        Bus::with_transport(MockTransport::default(), TIMEOUT).motor(0xFF),
        Err(Error::InvalidId(0xFF))
    ));
}

#[test]
fn query_happy_path() {
    let mut m = motor(MockTransport::with_replies([telemetry(0x01)]));
    let fb = m.query().unwrap().expect("telemetry parsed");
    assert_eq!(fb.kind, ReplyKind::Query);
    assert_eq!(fb.speed_rpm, 100);
    assert_eq!(fb.temp_c, Some(40));
    // Exactly one frame went out: the feedback query.
    let mock = m.into_transport().expect("sole handle");
    assert_eq!(mock.sent, vec![frame_feedback(0x01).to_vec()]);
}

#[test]
fn transact_selects_reply_layout_from_the_tx_frame() {
    // Identical reply bytes; what they mean depends on what was asked.
    let reply = vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x80, 0x00, 0x00];

    // Drive frame → drive layout: bytes 6-7 are one 16-bit position
    // (0x2880 = 10368 → ~113.9°), and there is no temperature.
    let mut m = motor(MockTransport::with_replies([reply.clone()]));
    let fb = m
        .transact(&frame_velocity(0x01, 100, 1), Duration::ZERO)
        .unwrap()
        .expect("telemetry");
    assert_eq!(fb.kind, ReplyKind::Drive);
    assert_eq!(fb.temp_c, None);
    assert_eq!((fb.position_deg * 10.0).round() / 10.0, 113.9);

    // Feedback query → query layout: 40 °C and a coarse position
    // (0x80 = 128 → ~180.7°).
    let mut m = motor(MockTransport::with_replies([reply]));
    let fb = m
        .transact(&frame_feedback(0x01), Duration::ZERO)
        .unwrap()
        .expect("telemetry");
    assert_eq!(fb.kind, ReplyKind::Query);
    assert_eq!(fb.temp_c, Some(40));
    assert_eq!((fb.position_deg * 10.0).round() / 10.0, 180.7);
}

#[test]
fn transact_mode_frame_yields_no_telemetry_but_still_sends() {
    // A mode-switch frame elicits no reply; even if stale bytes are sitting
    // in the buffer they must not be decoded against it.
    let mut m = motor(MockTransport::with_replies([telemetry(0x01)]));
    let frame = frame_mode(0x01, Mode::Velocity);
    assert!(m.transact(&frame, Duration::ZERO).unwrap().is_none());
    let mock = m.into_transport().expect("sole handle");
    assert_eq!(mock.sent, vec![frame.to_vec()]);
}

#[test]
fn query_silence_is_ok_none_not_error() {
    let mut m = motor(MockTransport::default());
    assert!(m.query().unwrap().is_none());
}

#[test]
fn query_strips_half_duplex_echo() {
    // Reply arrives as <tx echo><telemetry>.
    let mut m = motor(MockTransport {
        echo_tx: true,
        ..MockTransport::with_replies([telemetry(0x01)])
    });
    let fb = m.query().unwrap().expect("echo stripped, telemetry parsed");
    assert_eq!(fb.speed_rpm, 100);
    assert_eq!(fb.id, 0x01);
}

#[test]
fn query_pure_echo_is_none() {
    // Adapter echoes the TX but no motor answers. The echo is a full 10
    // bytes and starts with the motor's own ID, so a naive parser would
    // report it as telemetry; the echo prefix is stripped and the empty
    // remainder yields None.
    let mut m = motor(MockTransport {
        echo_tx: true,
        ..MockTransport::default()
    });
    assert!(m.query().unwrap().is_none());
}

#[test]
fn query_rejects_reply_from_a_different_motor() {
    // On a shared bus, another motor's frame — stale in the adapter buffer,
    // or a late answer landing in this transaction's window — must never be
    // handed back as this motor's telemetry.
    let mut m = motor(MockTransport::with_replies([telemetry(0x02)]));
    assert!(
        m.query().unwrap().is_none(),
        "motor 0x01 must not accept 0x02's telemetry"
    );
}

#[test]
fn query_accepts_only_its_own_id_on_a_shared_bus() {
    let bus = Bus::with_transport(
        MockTransport::with_replies([telemetry(0x02), telemetry(0x01)]),
        TIMEOUT,
    );
    let mut one = bus.motor(0x01).unwrap();
    // First reply belongs to 0x02 and is dropped; the second is ours.
    assert!(one.query().unwrap().is_none());
    assert_eq!(one.query().unwrap().expect("own telemetry").id, 0x01);
}

#[test]
fn query_ignores_partial_echo_rather_than_parsing_it() {
    // A half-buffered echo can't be matched by strip_prefix, so what is left
    // is not frame-aligned and must be rejected on that alone. The ID check
    // is no help here: a truncated echo starts with byte 0 of our own TX
    // frame, which is the addressed motor's ID — the very value the ID check
    // is looking for.
    let mut m = motor(MockTransport {
        echo_tx: true,
        echo_truncate: Some(4),
        ..MockTransport::default()
    });
    assert!(m.query().unwrap().is_none());
}

#[test]
fn query_short_reply_is_none() {
    // Truncated frame (motor cut off mid-transmission): too short to parse.
    let mut m = motor(MockTransport::with_replies([vec![0x01, 0x02, 0x00]]));
    assert!(m.query().unwrap().is_none());
}

#[test]
fn set_mode_sends_exactly_five_frames() {
    let mut m = motor(MockTransport::default());
    m.set_mode(Mode::Position).unwrap();
    let mock = m.into_transport().expect("sole handle");
    assert_eq!(mock.sent.len(), 5);
    for frame in &mock.sent {
        assert_eq!(frame, &frame_mode(0x01, Mode::Position).to_vec());
    }
}

#[test]
fn scan_broadcast_finds_id() {
    // Bus answers the broadcast with a frame starting with the motor's ID.
    let bus = Bus::with_transport(MockTransport::with_replies([telemetry(0x2A)]), TIMEOUT);
    let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
    assert_eq!(report.ids, vec![0x2A]);
    assert!(!report.garbled);
}

#[test]
fn scan_rejects_own_tx_echo() {
    // Only the echo of the query comes back — no motor. The query frame
    // contains 0xC8, which is in the valid ID range; without echo
    // stripping the scan would "find" motor 0xC8.
    let bus = Bus::with_transport(
        MockTransport {
            echo_tx: true,
            ..MockTransport::default()
        },
        TIMEOUT,
    );
    let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
    assert_eq!(report.ids, Vec::<u8>::new());
    // A fully-recognised echo is accounted for — nothing garbled about it.
    assert!(!report.garbled);
}

#[test]
fn scan_ignores_stray_bytes_that_are_not_a_whole_frame() {
    // A runt burst of line noise in the valid ID range must not be reported
    // as a motor: discovery walks whole 10-byte frames, not loose bytes.
    let bus = Bus::with_transport(
        MockTransport::with_replies([vec![0x2A, 0x2B, 0x2C]]),
        TIMEOUT,
    );
    let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
    assert_eq!(report.ids, Vec::<u8>::new());
    // ...but those bytes came from somewhere, and the caller must hear it.
    assert!(report.garbled);
}

#[test]
fn scan_flags_multi_motor_collision_as_garbled_not_silent() {
    // Observed on a real four-motor bus: the unarbitrated broadcast replies
    // collide into a 15-byte burst that is no whole number of frames. The
    // old scan reported "no motors" for this; garbled is what lets a caller
    // tell a colliding bus from a dead one and escalate to a full scan.
    let collided = vec![
        0x03, 0x00, 0x00, 0x14, 0x00, 0x05, 0x00, 0x00, 0x0F, 0x13, 0x00, 0x09, 0xC7, 0x00, 0x64,
    ];
    let bus = Bus::with_transport(MockTransport::with_replies([collided]), TIMEOUT);
    let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
    assert_eq!(report.ids, Vec::<u8>::new());
    assert!(report.garbled);
}

#[test]
fn scan_finds_several_motors_answering_back_to_back() {
    let mut reply = telemetry(0x2A);
    reply.extend(telemetry(0x05));
    let bus = Bus::with_transport(MockTransport::with_replies([reply]), TIMEOUT);
    let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
    assert_eq!(report.ids, vec![0x05, 0x2A]);
    assert!(!report.garbled);
}

#[test]
fn scan_ignores_collision_garbage_but_keeps_clean_frames() {
    // Two motors answering a broadcast at once collide into bytes belonging
    // to neither. A 10-byte chunk whose ID byte is out of range must not
    // register; a clean frame after it still does — and the junk chunk is
    // reported as garbling, since it hints at further motors colliding.
    let mut resp = vec![0x00, 0xFF, 0x13, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    resp.extend(telemetry(0x2A));
    let bus = Bus::with_transport(MockTransport::with_replies([resp]), TIMEOUT);
    let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
    assert_eq!(report.ids, vec![0x2A]);
    assert!(report.garbled);
}

#[test]
fn scan_partial_echo_reports_nothing_rather_than_a_phantom() {
    // Same misalignment hazard as parse_reply, with a nastier payload: the
    // ID-query frame's own destination byte is 0xC8, which is a valid motor
    // ID. A truncated echo shifts every chunk boundary, so chunk 0 begins
    // with that 0xC8 — and the scan used to report a motor at 0xC8 while
    // completely missing the real one at 0x2A.
    for keep in 1..10 {
        let bus = Bus::with_transport(
            MockTransport {
                echo_tx: true,
                echo_truncate: Some(keep),
                ..MockTransport::with_replies([telemetry(0x2A)])
            },
            TIMEOUT,
        );
        let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
        assert_eq!(
            report.ids,
            Vec::<u8>::new(),
            "echo truncated to {keep} produced a phantom ID"
        );
        // The unstrippable partial echo plus the reply is unattributable
        // data — flagged, so the real motor at 0x2A is not written off.
        assert!(report.garbled, "echo truncated to {keep} not flagged");
    }
}

#[test]
fn full_scan_partial_echo_does_not_answer_every_probe() {
    // Stage 2 matches a reply by its first byte equalling the probed ID —
    // but the probe frame's own first byte IS the probed ID, so a partial
    // echo would satisfy that test at all 254 addresses.
    let mut replies: Vec<Vec<u8>> = vec![Vec::new()]; // broadcast: silence
    for _ in 0x01..=0xFEu8 {
        replies.push(telemetry(0x10));
    }
    let bus = Bus::with_transport(
        MockTransport {
            echo_tx: true,
            echo_truncate: Some(3),
            ..MockTransport::with_replies(replies)
        },
        TIMEOUT,
    );
    assert_eq!(bus.scan(0x01..=0xFE, |_| {}).unwrap().ids, Vec::<u8>::new());
}

#[test]
fn scan_echo_plus_reply_finds_real_id() {
    let bus = Bus::with_transport(
        MockTransport {
            echo_tx: true,
            ..MockTransport::with_replies([telemetry(0x2A)])
        },
        TIMEOUT,
    );
    let report = bus.scan(std::iter::empty(), |_| {}).unwrap();
    assert_eq!(report.ids, vec![0x2A]);
    assert!(!report.garbled);
}

#[test]
fn full_scan_probes_all_ids_and_matches_by_reply_id() {
    // Silent bus except ID 0x10, which answers its own probe.
    let mut replies: Vec<Vec<u8>> = vec![Vec::new()]; // broadcast: silence
    for id in 0x01..=0xFEu8 {
        replies.push(if id == 0x10 {
            telemetry(0x10)
        } else {
            Vec::new()
        });
    }
    let bus = Bus::with_transport(MockTransport::with_replies(replies), TIMEOUT);
    let mut progress = Vec::new();
    let found = bus.scan(0x01..=0xFE, |id| progress.push(id)).unwrap().ids;
    assert_eq!(found, vec![0x10]);
    // All 254 IDs probed, in order, each with its own feedback frame.
    assert_eq!(progress, (0x01..=0xFEu8).collect::<Vec<_>>());
    let mock = bus.into_transport().expect("no motors minted");
    assert_eq!(mock.sent.len(), 1 + 254); // broadcast + probes
    assert_eq!(mock.sent[1], frame_feedback(0x01).to_vec());
    assert_eq!(mock.sent[254], frame_feedback(0xFE).to_vec());
}

#[test]
fn full_scan_ignores_reply_with_wrong_id() {
    // A reply whose first byte isn't the probed ID (noise/cross-talk) must
    // not register as a find.
    let mut replies: Vec<Vec<u8>> = vec![Vec::new()];
    for id in 0x01..=0xFEu8 {
        replies.push(if id == 0x10 {
            telemetry(0x99)
        } else {
            Vec::new()
        });
    }
    let bus = Bus::with_transport(MockTransport::with_replies(replies), TIMEOUT);
    assert_eq!(bus.scan(0x01..=0xFE, |_| {}).unwrap().ids, Vec::<u8>::new());
}

#[test]
fn scan_polls_only_the_requested_ids_and_skips_invalid_ones() {
    // Broadcast silence; motor 0x05 answers its probe. 0x00 and 0xFF are
    // not assignable IDs and must not be probed at all.
    let mut replies: Vec<Vec<u8>> = vec![Vec::new()]; // broadcast: silence
    for id in 0x01..=0x0Fu8 {
        replies.push(if id == 0x05 {
            telemetry(0x05)
        } else {
            Vec::new()
        });
    }
    let bus = Bus::with_transport(MockTransport::with_replies(replies), TIMEOUT);
    let report = bus.scan((0x00..=0x0F).chain([0xFF]), |_| {}).unwrap();
    assert_eq!(report.ids, vec![0x05]);
    let mock = bus.into_transport().expect("no motors minted");
    // 1 broadcast + 15 probes: 0x00 and 0xFF were skipped.
    assert_eq!(mock.sent.len(), 16);
    assert_eq!(mock.sent[1], frame_feedback(0x01).to_vec());
    assert_eq!(mock.sent[15], frame_feedback(0x0F).to_vec());
}

#[test]
fn set_id_repeats_five_times_then_requeries() {
    // After the change, the broadcast re-query reports the new ID.
    let bus = Bus::with_transport(MockTransport::with_replies([telemetry(0x05)]), TIMEOUT);
    assert_eq!(bus.set_id(0x05).unwrap(), Some(0x05));
    let mock = bus.into_transport().expect("no motors minted");
    assert_eq!(mock.sent.len(), 6); // 5× set-ID + 1 broadcast query
    let set_frame = [0xAA, 0x55, 0x53, 0x05, 0, 0, 0, 0, 0, 0].to_vec();
    for frame in &mock.sent[..5] {
        assert_eq!(frame, &set_frame);
    }
    assert_eq!(mock.sent[5], frame_id_query().to_vec());
}

#[test]
fn set_id_silence_returns_none() {
    let bus = Bus::with_transport(MockTransport::default(), TIMEOUT);
    assert_eq!(bus.set_id(0x05).unwrap(), None);
}

#[test]
fn set_id_rejects_invalid_target() {
    let bus = Bus::with_transport(MockTransport::default(), TIMEOUT);
    assert!(matches!(bus.set_id(0x00), Err(Error::InvalidId(0x00))));
    // Nothing was sent for the invalid request.
    assert!(bus.into_transport().expect("no motors").sent.is_empty());
}

#[test]
fn safe_stop_forces_velocity_mode_before_zeroing() {
    // A zero-valued drive frame only means "stop" in velocity mode: in
    // position mode the identical bytes command a move to 0 deg. So the
    // stop sequence must establish the mode it is about to assume, and the
    // five mode frames must come FIRST.
    let mut m = motor(MockTransport::default());
    m.safe_stop();
    let mock = m.into_transport().expect("sole handle");
    assert_eq!(mock.sent.len(), 15, "5 mode + 5 zero + 5 brake");

    // Expected bytes written out, not recomputed from the code under test.
    let mode = vec![0x01, 0xA0, 0, 0, 0, 0, 0, 0, 0, 0x02];
    let zero = vec![0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xFB];
    let brake = vec![0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1];
    assert!(mock.sent[..5].iter().all(|f| *f == mode));
    assert!(mock.sent[5..10].iter().all(|f| *f == zero));
    assert!(mock.sent[10..].iter().all(|f| *f == brake));
}

#[test]
fn safe_stop_survives_io_failure() {
    // The shutdown path must not give up on errors: every frame is still
    // attempted and safe_stop returns normally.
    let mut m = motor(MockTransport {
        fail_io: true,
        ..MockTransport::default()
    });
    m.safe_stop(); // must not panic or bail early
    assert_eq!(m.into_transport().expect("sole handle").sent.len(), 15);
}

#[test]
fn drive_frames_are_fire_and_forget() {
    let mut m = motor(MockTransport::default());
    m.drive_velocity(250).unwrap();
    m.brake().unwrap();
    let mock = m.into_transport().expect("sole handle");
    assert_eq!(mock.sent.len(), 2);
    // Literal wire bytes: comparing against frame_velocity() here would
    // assert the builder against itself and pass even if it were wrong.
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x00, 0xFA, 0x00, 0x00, 0x01, 0x00, 0x00, 0x46]
    );
    assert_eq!(
        mock.sent[1],
        vec![0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1]
    );
}

#[test]
fn drive_velocity_clamps_at_the_driver_boundary() {
    let mut m = motor(MockTransport::default());
    m.drive_velocity(999).unwrap();
    m.drive_velocity(-999).unwrap();
    let mock = m.into_transport().expect("sole handle");
    // 330 and -330 as they appear on the wire, spelled out.
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x01, 0x4A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x7C]
    );
    assert_eq!(
        mock.sent[1],
        vec![0x01, 0x64, 0xFE, 0xB6, 0x00, 0x00, 0x01, 0x00, 0x00, 0x75]
    );
}

#[test]
fn drive_velocity_accel_reaches_the_wire() {
    let mut m = motor(MockTransport::default());
    m.drive_velocity_accel(100, 20).unwrap();
    let mock = m.into_transport().expect("sole handle");
    // Byte 6 carries the acceleration; default drive_velocity would send 1.
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x14, 0x00, 0x00, 0x9B]
    );
}

#[test]
fn drive_current_and_position_clamp_at_the_driver_boundary() {
    let mut m = motor(MockTransport::default());
    m.drive_current(i16::MIN).unwrap(); // outside the symmetric range
    m.drive_position(40000).unwrap(); // above POS_MAX
    let mock = m.into_transport().expect("sole handle");
    // CUR_MIN (0x8001) and POS_MAX (0x7FFF) as literal wire bytes.
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0]
    );
    assert_eq!(
        mock.sent[1],
        vec![0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97]
    );
}

#[test]
fn io_errors_propagate_from_every_path_except_safe_stop() {
    let mut m = motor(MockTransport {
        fail_io: true,
        ..MockTransport::default()
    });
    assert!(m.drive_velocity(100).is_err());
    assert!(m.drive_current(100).is_err());
    assert!(m.drive_position(100).is_err());
    assert!(m.brake().is_err());
    assert!(m.set_mode(Mode::Velocity).is_err());
    assert!(m.query().is_err());
    assert!(m.transact(&frame_feedback(0x01), Duration::ZERO).is_err());
    assert!(m.send_raw(&frame_feedback(0x01), Duration::ZERO).is_err());
}

#[test]
fn send_raw_returns_unparsed_reply_bytes() {
    // send_raw is the escape hatch: no echo stripping, no ID filtering,
    // no telemetry decode — bytes in, bytes out.
    let bus = Bus::with_transport(
        MockTransport::with_replies([vec![0xDE, 0xAD, 0xBE, 0xEF]]),
        TIMEOUT,
    );
    let resp = bus.send_raw(&frame_feedback(0x01), Duration::ZERO).unwrap();
    assert_eq!(resp, vec![0xDE, 0xAD, 0xBE, 0xEF]);

    let mut m = bus.motor(0x01).unwrap();
    // Silence comes back as an empty Vec, not an error.
    let resp = m.send_raw(&frame_feedback(0x01), Duration::ZERO).unwrap();
    assert!(resp.is_empty());
}

#[test]
fn into_transport_requires_the_last_handle() {
    let bus = Bus::with_transport(MockTransport::default(), TIMEOUT);
    let m = bus.motor(0x01).unwrap();
    assert!(
        bus.into_transport().is_none(),
        "a motor still holds the bus"
    );
    let clone = m.clone();
    assert!(m.into_transport().is_none(), "a clone still holds the bus");
    assert!(clone.into_transport().is_some(), "last handle recovers it");
}

#[test]
fn partial_echoes_of_every_length_never_parse_as_telemetry() {
    // A truncated TX echo cannot be matched by strip_prefix; whatever
    // remains must be rejected, not mistaken for telemetry. Sweep every
    // possible echo length.
    //
    // The silent-bus case below is the easy half — the leftover is under a
    // full frame, so it fails on length no matter what else is wrong. The
    // dangerous half is `partial_echo_followed_by_a_reply_is_not_telemetry`.
    for keep in 0..=10 {
        let mut m = motor(MockTransport {
            echo_tx: true,
            echo_truncate: Some(keep),
            ..MockTransport::default()
        });
        assert!(m.query().unwrap().is_none(), "echo truncated to {keep}");
    }
}

#[test]
fn partial_echo_followed_by_a_reply_is_not_telemetry() {
    // The case a silent bus cannot exercise: a truncated echo *plus* a
    // genuine reply is 10 or more bytes, so nothing fails on length, and the
    // straddling frame it decodes to is not obviously wrong. It starts at
    // byte 0 of the unstripped echo, which is byte 0 of our own TX frame —
    // this motor's ID — so the ID check waves it through, and the values
    // that come out look entirely plausible.
    //
    // Before frame alignment was enforced, this wheel spinning at 300 RPM
    // reported 0 RPM for a 6-byte echo and 1 RPM for a 5-byte one. Callers
    // refuse to enter position mode above 10 RPM on the strength of that
    // number, so "300 reads as 0" is the failure that matters.
    let spinning = vec![0x01, 0x02, 0x00, 0x00, 0x01, 0x2C, 0x28, 0x80, 0x00, 0x00];
    for keep in 1..10 {
        let mut m = motor(MockTransport {
            echo_tx: true,
            echo_truncate: Some(keep),
            ..MockTransport::with_replies([spinning.clone()])
        });
        assert!(
            m.query().unwrap().is_none(),
            "echo truncated to {keep} straddled a real reply and parsed as telemetry"
        );
    }

    // The intact-echo case still works — this must reject misalignment, not
    // every reply that arrives behind an echo.
    let mut m = motor(MockTransport {
        echo_tx: true,
        ..MockTransport::with_replies([spinning])
    });
    assert_eq!(m.query().unwrap().expect("intact echo").speed_rpm, 300);
}

#[test]
fn a_neighbours_reply_landing_first_does_not_hide_our_own() {
    // Multi-drop: another motor's late answer can arrive inside our
    // transaction window, ahead of ours. Both are whole frames, so the
    // buffer is well-formed — pick out the one addressed to us rather than
    // writing off the whole read.
    let mut both = telemetry(0x02);
    both.extend(telemetry(0x01));
    let mut m = motor(MockTransport::with_replies([both]));
    let fb = m.query().unwrap().expect("our own frame is in there");
    assert_eq!(fb.id, 0x01);
    assert_eq!(fb.speed_rpm, 100);
}

#[test]
fn error_display_and_permission_helper() {
    let e = Error::InvalidId(0x00);
    assert_eq!(e.to_string(), "invalid motor ID 0x00 (must be 0x01..=0xFE)");
    assert!(!e.is_permission_denied());
    assert_eq!(
        Error::InvalidFrameLen(3).to_string(),
        "invalid frame length 3 (need 9 or 10 bytes)"
    );
    let denied = Error::Io(std::io::Error::from(std::io::ErrorKind::PermissionDenied));
    assert!(denied.is_permission_denied());
    let broken = Error::Io(std::io::Error::from(std::io::ErrorKind::BrokenPipe));
    assert!(!broken.is_permission_denied());
}

// ── Multi-motor bus sharing ─────────────────────────────────────────────────

#[test]
fn two_motors_share_one_bus() {
    let bus = Bus::with_transport(MockTransport::default(), TIMEOUT);
    let mut left = bus.motor(0x01).unwrap();
    let mut right = bus.motor(0x02).unwrap();

    left.drive_velocity(100).unwrap();
    right.drive_velocity(100).unwrap();
    left.brake().unwrap();

    // All frames went out on the one shared transport, correctly addressed.
    drop(left);
    drop(right);
    let mock = bus.into_transport().expect("all handles dropped");
    assert_eq!(mock.sent.len(), 3);
    // Literal bytes: the point of this test is that each handle addresses
    // its own motor, and byte 0 is the whole claim.
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00, 0x00, 0xE4]
    );
    assert_eq!(
        mock.sent[1],
        vec![0x02, 0x64, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00, 0x00, 0x11]
    );
    assert_eq!(
        mock.sent[2],
        vec![0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1]
    );
}

#[test]
fn motor_handles_are_cloneable() {
    let m = motor(MockTransport::default());
    let mut a = m.clone();
    let mut b = m;
    a.drive_velocity(50).unwrap();
    b.drive_velocity(-50).unwrap();
    drop(a);
    let mock = b.into_transport().expect("last handle");
    assert_eq!(mock.sent.len(), 2);
    // Both clones reach the same transport, each carrying its own setpoint —
    // a count alone would pass if a clone sent the wrong ID or value, or if
    // one frame were sent twice.
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x00, 0x32, 0x00, 0x00, 0x01, 0x00, 0x00, 0x78]
    );
    assert_eq!(
        mock.sent[1],
        vec![0x01, 0x64, 0xFF, 0xCE, 0x00, 0x00, 0x01, 0x00, 0x00, 0x71]
    );
}

// ── Mirrored (left/right) wheels ────────────────────────────────────────────

#[test]
fn mirrored_negates_velocity_and_current_setpoints() {
    let mut m = motor(MockTransport::default()).mirrored(true);
    assert!(m.is_mirrored());
    m.drive_velocity(100).unwrap();
    m.drive_current(-1234).unwrap();
    let mock = m.into_transport().expect("sole handle");
    // Literal wire bytes. Building the expectation with frame_velocity()
    // would assert the builder against itself — and this is the mirror-sign
    // test, so a byte-swapped or sign-flipped builder is exactly what it
    // has to be able to catch.
    //
    // "+100 forward" goes out as -100: 0xFF9C big-endian, accel 1.
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0xFF, 0x9C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x31]
    );
    // -1234 goes out as +1234 = 0x04D2.
    assert_eq!(
        mock.sent[1],
        vec![0x01, 0x64, 0x04, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E]
    );
}

#[test]
fn mirrored_flips_telemetry_signs() {
    // Wire reports +100 RPM and -0.488 A; a mirrored handle presents them
    // as -100 RPM and +0.488 A so that "positive = robot forward" holds on
    // both sides.
    let reply = vec![0x01, 0x02, 0xF8, 0x30, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00];
    let mut m = motor(MockTransport::with_replies([reply])).mirrored(true);
    let fb = m.query().unwrap().expect("telemetry");
    assert_eq!(fb.speed_rpm, -100);
    // 0xF830 = -1998 → -0.488 A on the wire, sign-flipped to +0.488.
    assert_eq!((fb.current_a * 1000.0).round() / 1000.0, 0.488);
    // The raw wire frame is untouched.
    assert_eq!(fb.raw[2..6], [0xF8, 0x30, 0x00, 0x64]);
}

#[test]
fn mirrored_drive_reply_flips_speed_but_not_position() {
    let reply = vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x80, 0x00, 0x00];
    let mut m = motor(MockTransport::with_replies([reply])).mirrored(true);
    let fb = m
        .transact(&frame_velocity(0x01, 100, 1), Duration::ZERO)
        .unwrap()
        .expect("telemetry");
    assert_eq!(fb.speed_rpm, -100);
    // The 16-bit drive-reply position is an absolute angle: not mirrored.
    assert_eq!((fb.position_deg * 10.0).round() / 10.0, 113.9);
    assert_eq!(fb.temp_c, None);
}

#[test]
fn mirrored_i16_min_saturates_then_clamps() {
    // -32768 has no i16 negation; saturating_neg gives 32767, which then
    // clamps to the mode's maximum instead of wrapping to full reverse.
    let mut m = motor(MockTransport::default()).mirrored(true);
    m.drive_velocity(i16::MIN).unwrap(); // → +330
    m.drive_current(i16::MIN).unwrap(); // → CUR_MAX
    let mock = m.into_transport().expect("sole handle");
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x01, 0x4A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x7C]
    );
    assert_eq!(
        mock.sent[1],
        vec![0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97]
    );
}

#[test]
fn mirrored_leaves_position_and_brake_untouched() {
    let mut m = motor(MockTransport::default()).mirrored(true);
    m.drive_position(1000).unwrap();
    m.brake().unwrap();
    m.safe_stop(); // velocity 0 negates to 0 — sequence identical
    let mock = m.into_transport().expect("sole handle");
    assert_eq!(
        mock.sent[0],
        vec![0x01, 0x64, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9F]
    );
    assert_eq!(
        mock.sent[1],
        vec![0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1]
    );
    // safe_stop leads with the mode switch, then the (unnegated) zeros.
    assert_eq!(mock.sent[2], vec![0x01, 0xA0, 0, 0, 0, 0, 0, 0, 0, 0x02]);
    assert_eq!(
        mock.sent[7],
        vec![0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xFB]
    );
}

#[test]
fn unmirrored_is_default_and_identity() {
    let mut m = motor(MockTransport::with_replies([telemetry(0x01)]));
    assert!(!m.is_mirrored());
    let fb = m.query().unwrap().expect("telemetry");
    assert_eq!(fb.speed_rpm, 100);
}
