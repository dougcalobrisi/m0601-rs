//! Timing tests for the enforced inter-frame gap ([`Bus::with_min_gap`]).
//!
//! These are the one thing `MockTransport` cannot see: its `pace` returns
//! zero, so the gap machinery is invisible to every mock test by design.
//! Here a transport with the *default* (real) `pace` records the instant of
//! each transmission, and the tests assert on the spacing between them.
//! Gaps are tens of milliseconds so the assertions hold on a loaded
//! machine; total added test time is well under a second.

// Test helpers may assert; the workspace no-panic lints target library code.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::time::{Duration, Instant};

use m0601::error::Result;
use m0601::{Bus, DEFAULT_MIN_GAP, Transport};

const TIMEOUT: Duration = Duration::from_millis(150);

/// Records when each frame hit the wire. Keeps the default [`Transport::pace`]
/// (a real wait), unlike `MockTransport`.
#[derive(Default)]
struct TimedTransport {
    sent_at: Vec<Instant>,
}

impl Transport for TimedTransport {
    fn send(&mut self, _data: &[u8]) -> Result<()> {
        self.sent_at.push(Instant::now());
        Ok(())
    }

    fn send_recv(&mut self, _data: &[u8], _wait: Duration) -> Result<Vec<u8>> {
        self.sent_at.push(Instant::now());
        Ok(Vec::new())
    }
}

fn deltas(sent_at: &[Instant]) -> Vec<Duration> {
    sent_at.windows(2).map(|w| w[1] - w[0]).collect()
}

#[test]
fn consecutive_frames_are_never_closer_than_min_gap() {
    let gap = Duration::from_millis(20);
    let bus = Bus::with_transport(TimedTransport::default(), TIMEOUT).with_min_gap(gap);
    let mut motor = bus.motor(0x01).unwrap();
    drop(bus);
    for _ in 0..3 {
        motor.drive_velocity(100).unwrap();
    }
    let t = motor.into_transport().expect("sole handle");
    assert_eq!(t.sent_at.len(), 3);
    for (i, d) in deltas(&t.sent_at).into_iter().enumerate() {
        assert!(d >= gap, "frames {i}/{} only {d:?} apart", i + 1);
    }
}

#[test]
fn zero_min_gap_restores_back_to_back_sends() {
    let start = Instant::now();
    let bus = Bus::with_transport(TimedTransport::default(), TIMEOUT).with_min_gap(Duration::ZERO);
    let mut motor = bus.motor(0x01).unwrap();
    drop(bus);
    for _ in 0..20 {
        motor.drive_velocity(100).unwrap();
    }
    // Twenty no-op sends take microseconds. If opting out silently left the
    // default gap in force they would take at least 19 × 2.5 ms = 47.5 ms,
    // so the 40 ms bound distinguishes the two while staying far above any
    // plausible scheduler hiccup.
    assert!(
        start.elapsed() < Duration::from_millis(40),
        "opting out must not leave any enforced spacing behind (took {:?})",
        start.elapsed()
    );
}

#[test]
fn the_gap_holds_across_threads_and_cloned_handles() {
    // Two threads each driving their own wheel through cloned handles —
    // the docs/content/docs/library/multi-motor.md scenario ("Frame
    // spacing across handles"). The gap must be a property of the shared
    // port, not of any one handle, or the two threads' frames can land
    // back-to-back and corrupt on the half-duplex wire.
    let gap = Duration::from_millis(10);
    let bus = Bus::with_transport(TimedTransport::default(), TIMEOUT).with_min_gap(gap);
    let mut left = bus.motor(0x01).unwrap();
    let mut right = bus.motor(0x02).unwrap();
    drop(bus);

    std::thread::scope(|s| {
        s.spawn(|| {
            for _ in 0..4 {
                left.drive_velocity(100).unwrap();
            }
        });
        s.spawn(|| {
            for _ in 0..4 {
                right.drive_velocity(100).unwrap();
            }
        });
    });

    drop(right);
    let t = left.into_transport().expect("last handle");
    assert_eq!(t.sent_at.len(), 8);
    for (i, d) in deltas(&t.sent_at).into_iter().enumerate() {
        assert!(d >= gap, "frames {i}/{} only {d:?} apart", i + 1);
    }
}

#[test]
fn a_four_motor_stop_round_fits_its_20_ms_budget() {
    // safe_stop_all paces its rounds on absolute 20 ms deadlines, so the
    // four frames of a round (each ~0.9 ms on the wire at 115200 baud,
    // each preceded by the enforced gap) must fit inside one round with
    // headroom. If DEFAULT_MIN_GAP grows past ~4 ms this stops being true
    // and the group stop silently stretches — fail here instead.
    let bits_per_frame = m0601::protocol::FRAME_LEN as u64 * 10; // 8N1: 10 bits/byte
    let frame_time =
        Duration::from_micros(bits_per_frame * 1_000_000 / u64::from(m0601::protocol::BAUD));
    let round = 4 * (frame_time + DEFAULT_MIN_GAP);
    assert!(
        round <= Duration::from_millis(20) * 3 / 4,
        "4 x (frame {frame_time:?} + gap {DEFAULT_MIN_GAP:?}) = {round:?} \
         leaves too little of the 20 ms round for jitter"
    );
}
