//! Hardware-in-the-loop tests. All `#[ignore]` — they need a real motor.
//!
//! Run with a motor connected:
//!
//! ```sh
//! M0601_PORT=/dev/ttyUSB0 cargo test -p m0601 --test hardware -- --ignored --test-threads=1
//! ```
//!
//! `--test-threads=1` is recommended, but these tests also serialise
//! themselves on an internal lock, so omitting it costs wall-clock time
//! rather than corrupting the bus. `M0601_ID` selects a motor other than
//! `0x01`.
//!
//! `scan_finds_motor` and `query_returns_telemetry` only query — the wheel
//! does not move. `spin_and_stop` DOES spin the wheel briefly and therefore
//! additionally requires `M0601_ALLOW_MOTION=1`; make sure the wheel is off
//! the ground first.

// Test helpers may assert; the workspace no-panic lints target library code.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use m0601::{M0601, Mode};

const TIMEOUT: Duration = Duration::from_millis(150);

/// The serial port is exclusive, so these tests must never overlap.
/// `--test-threads=1` is documented above, but documentation does not
/// enforce anything — this does, so forgetting the flag costs time rather
/// than producing a garbled bus and a mystifying failure.
static PORT: Mutex<()> = Mutex::new(());

fn port_guard() -> MutexGuard<'static, ()> {
    PORT.lock().unwrap_or_else(PoisonError::into_inner)
}

fn port() -> String {
    std::env::var("M0601_PORT").expect("set M0601_PORT (e.g. /dev/ttyUSB0) to run hardware tests")
}

/// Motor ID under test; override with `M0601_ID` (decimal or `0x` hex).
fn motor_id() -> u8 {
    match std::env::var("M0601_ID") {
        Ok(s) => {
            let t = s.trim();
            let parsed = t
                .strip_prefix("0x")
                .or_else(|| t.strip_prefix("0X"))
                .map_or_else(|| t.parse::<u8>().ok(), |h| u8::from_str_radix(h, 16).ok());
            parsed.expect("M0601_ID must be a byte, e.g. 0x01 or 1")
        }
        Err(_) => 0x01,
    }
}

fn open() -> M0601 {
    M0601::open(&port(), motor_id(), TIMEOUT).expect("open serial port")
}

#[test]
#[ignore = "needs hardware: set M0601_PORT"]
fn scan_finds_motor() {
    let _guard = port_guard();
    let bus = m0601::Bus::open(&port(), TIMEOUT).expect("open serial port");
    let ids = bus.scan(false, |_| {}).expect("scan I/O");
    assert!(!ids.is_empty(), "no motor answered the broadcast ID query");
}

#[test]
#[ignore = "needs hardware: set M0601_PORT"]
fn query_returns_telemetry() {
    let _guard = port_guard();
    let id = motor_id();
    let mut m = open();
    let fb = m.query().expect("query I/O").unwrap_or_else(|| {
        panic!("motor 0x{id:02X} did not reply — check power/wiring, or set M0601_ID")
    });
    assert_eq!(fb.id, id);
    // A 0x74 query reply always carries the winding temperature. Sanity,
    // not exactness: a resting winding is well below the 80 °C trip.
    let temp = fb.temp_c.expect("0x74 reply carries winding temperature");
    assert!(temp < 80, "implausible temperature {temp}");
    assert!(fb.mode.is_some(), "unknown mode byte 0x{:02X}", fb.mode_raw);
}

/// Capture raw replies to both a 0x74 query and a 0x64 drive frame and
/// report whether byte 9 is a CRC-8/MAXIM over bytes 0..9. Sources
/// disagree: the DFRobot wiki and the navigation_robot C driver say
/// replies carry that CRC; this crate's original hardware observation said
/// they don't. This test settles it for the connected unit — it asserts
/// nothing beyond getting replies, but prints the verdict.
#[test]
#[ignore = "needs hardware: set M0601_PORT"]
fn reply_checksum_capture() {
    let _guard = port_guard();
    let id = motor_id();
    let mut m = open();

    let report = |label: &str, tx: &[u8], rx: &[u8]| {
        let rx = rx.strip_prefix(tx).unwrap_or(rx);
        let Some(frame) = rx.get(..10) else {
            eprintln!("{label}: no reply captured ({} bytes)", rx.len());
            return;
        };
        let crc = m0601::protocol::crc8_maxim(&frame[..9]);
        let hex: Vec<String> = frame.iter().map(|b| format!("{b:02X}")).collect();
        eprintln!(
            "{label}: {} — byte 9 = 0x{:02X}, CRC-8/MAXIM(bytes 0..9) = 0x{crc:02X} → {}",
            hex.join(" "),
            frame[9],
            if frame[9] == crc { "MATCHES" } else { "DIFFERS" },
        );
    };

    let query = m0601::protocol::frame_feedback(id);
    let rx = m
        .send_raw(&query, Duration::from_millis(50))
        .expect("query I/O");
    report("0x74 query reply", &query, &rx);

    // A zero-velocity drive frame is safe: it commands no motion in the
    // power-up default velocity mode. Do not run this after switching the
    // motor to position mode.
    let drive = m0601::protocol::frame_velocity(id, 0, 1);
    let rx = m
        .send_raw(&drive, Duration::from_millis(50))
        .expect("drive I/O");
    report("0x64 drive reply", &drive, &rx);
}

/// Guard that stops the motor even if the test panics mid-spin.
struct StopOnDrop(Option<M0601>);

impl Drop for StopOnDrop {
    fn drop(&mut self) {
        if let Some(mut m) = self.0.take() {
            m.safe_stop();
        }
    }
}

#[test]
#[ignore = "needs hardware AND spins the wheel: set M0601_PORT and M0601_ALLOW_MOTION=1"]
fn spin_and_stop() {
    let _guard = port_guard();
    if std::env::var("M0601_ALLOW_MOTION").as_deref() != Ok("1") {
        eprintln!("skipping: M0601_ALLOW_MOTION=1 not set");
        return;
    }
    let mut guard = StopOnDrop(Some(open()));
    let m = guard.0.as_mut().expect("just constructed");

    m.set_mode(Mode::Velocity).expect("set velocity mode");

    // Drive at 60 RPM for 2 s, polling at 50 Hz as the protocol requires.
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut fastest: i16 = 0;
    while Instant::now() < deadline {
        let frame = m0601::protocol::frame_velocity(m.id(), 60, 1);
        if let Ok(Some(fb)) = m.transact(&frame, Duration::from_millis(6)) {
            fastest = fastest.max(fb.speed_rpm);
            // Drive replies use the drive layout: no temperature, and a
            // 16-bit position that must stay within one revolution.
            assert!(fb.temp_c.is_none(), "drive reply carried a temperature");
            assert!(
                (0.0..=360.0).contains(&fb.position_deg),
                "drive-reply position out of range: {}",
                fb.position_deg
            );
        }
        std::thread::sleep(Duration::from_millis(14));
    }
    m.safe_stop();

    assert!(fastest > 20, "wheel never spun up (peak {fastest} RPM)");

    // After the stop sequence the wheel should be (nearly) stationary.
    std::thread::sleep(Duration::from_millis(500));
    let fb = m.query().expect("query I/O").expect("telemetry after stop");
    assert!(
        fb.speed_rpm.abs() < 10,
        "still moving: {} RPM",
        fb.speed_rpm
    );
}
