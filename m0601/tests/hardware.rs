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
    let report = bus.scan(std::iter::empty(), |_| {}).expect("scan I/O");
    // On a multi-motor bus the broadcast replies collide, so garbled bytes
    // are proof of life just as a clean ID is; only silence is a failure.
    assert!(
        !report.ids.is_empty() || report.garbled,
        "no motor answered the broadcast ID query"
    );
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
            if frame[9] == crc {
                "MATCHES"
            } else {
                "DIFFERS"
            },
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
    // Fail rather than return: a silent `return` here reports `ok`, which is
    // indistinguishable from the wheel having actually spun up and stopped.
    // This test is already `#[ignore]`d, so reaching this line means someone
    // opted in with `--ignored` — tell them why nothing moved.
    assert_eq!(
        std::env::var("M0601_ALLOW_MOTION").as_deref(),
        Ok("1"),
        "spin_and_stop moves the wheel; set M0601_ALLOW_MOTION=1 to allow it \
         (make sure the wheel is off the ground), or deselect this test with \
         `--skip spin_and_stop`"
    );
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

/// Which end of the acceleration byte's range is the gentle one — the open
/// question in `docs/content/docs/protocol.md` (contradiction 6).
///
/// No vendor source states the direction. The upstream DDT manual gives only
/// a unit, `RPM/0.1ms`, a *rate*, under which a larger byte ramps *harder*;
/// the DFRobot wiki contradicts that unit within the same sentence. This
/// crate stopped guessing and set every default to `0`. This capture settles
/// it on hardware, the way `reply_checksum_capture` settled the reply CRC.
///
/// For each accel byte it commands a step from rest to `M0601_TEST_RPM`
/// (default 120) and reports **time to 90% of setpoint** and **peak current**
/// during the ramp. Both matter: the first gives the direction, the second is
/// the quantity the 3 A bus-overcurrent argument actually rests on.
///
/// ```sh
/// M0601_PORT=/dev/ttyUSB0 M0601_ALLOW_MOTION=1 \
///   cargo test -p m0601 --test hardware -- --ignored --nocapture accel_direction_capture
/// ```
///
/// **Spins the wheel, repeatedly. Get it off the ground first.**
#[test]
#[ignore = "needs hardware AND spins the wheel: set M0601_PORT and M0601_ALLOW_MOTION=1"]
fn accel_direction_capture() {
    let _guard = port_guard();
    assert_eq!(
        std::env::var("M0601_ALLOW_MOTION").as_deref(),
        Ok("1"),
        "accel_direction_capture spins the wheel; set M0601_ALLOW_MOTION=1 to \
         allow it (make sure the wheel is off the ground)"
    );

    /// Accel bytes to sweep. `0` is the motor's own default and is the row
    /// every other one is read against — if `0` lands on top of one end of
    /// the sweep, that also tells us what the default *is*.
    const SWEEP: [u8; 7] = [0, 1, 2, 5, 20, 100, 255];
    /// Give up on a trial that never reaches 90%; also the ramp window.
    const TRIAL: Duration = Duration::from_millis(3000);
    /// Sampling cadence. Well above the 50 Hz the protocol requires (so
    /// motion is sustained) and well under the 500 Hz ceiling, chosen for
    /// timing resolution: a fast ramp may complete inside one sample, and
    /// that saturation is itself a reportable result.
    const SAMPLE_GAP: Duration = Duration::from_millis(5);
    const REPLY_WAIT: Duration = Duration::from_millis(4);
    /// Keep driving past the 90% crossing so peak current covers the whole
    /// ramp, not just its first 90%.
    const HOLD_PAST_CROSSING: Duration = Duration::from_millis(300);

    let target: i16 = std::env::var("M0601_TEST_RPM")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(120);
    assert!(
        (20..=330).contains(&target),
        "M0601_TEST_RPM must be 20..=330, got {target}"
    );
    let reached = f32::from(target) * 0.9;

    let mut guard = StopOnDrop(Some(open()));
    let m = guard.0.as_mut().expect("just constructed");
    m.set_mode(Mode::Velocity).expect("set velocity mode");

    eprintln!(
        "\naccel direction capture: step 0 -> {target} RPM, 90% = {reached:.0} RPM, \
         sampling every {}ms\n",
        SAMPLE_GAP.as_millis()
    );
    eprintln!(
        "{:>5}  {:>12}  {:>11}  {:>10}  {:>8}",
        "accel", "t to 90%", "peak RPM", "peak A", "samples"
    );

    let mut results: Vec<(u8, Option<Duration>, i16, f32)> = Vec::new();

    for accel in SWEEP {
        // Come to a complete rest between trials, so every step starts from
        // the same place. safe_stop brakes; give the wheel time to settle.
        m.safe_stop();
        std::thread::sleep(Duration::from_millis(800));
        if let Ok(Some(fb)) = m.query() {
            assert!(
                fb.speed_rpm.abs() < 10,
                "wheel did not come to rest before the accel {accel} trial: {} RPM",
                fb.speed_rpm
            );
        }
        // safe_stop leaves the brake engaged; re-establish velocity mode and
        // release it with one zero-velocity, brake-off frame before the step,
        // or the ramp would be measured against a held wheel.
        m.set_mode(Mode::Velocity).expect("set velocity mode");
        let release = m0601::protocol::frame_velocity(m.id(), 0, accel);
        let _ = m.transact(&release, REPLY_WAIT);
        std::thread::sleep(Duration::from_millis(200));

        let frame = m0601::protocol::frame_velocity(m.id(), target, accel);
        let start = Instant::now();
        let mut time_to_90: Option<Duration> = None;
        let mut peak_rpm: i16 = 0;
        let mut peak_a: f32 = 0.0;
        let mut samples: u32 = 0;

        while start.elapsed() < TRIAL {
            if let Ok(Some(fb)) = m.transact(&frame, REPLY_WAIT) {
                samples += 1;
                peak_rpm = peak_rpm.max(fb.speed_rpm);
                peak_a = peak_a.max(fb.current_a.abs());
                if time_to_90.is_none() && f32::from(fb.speed_rpm) >= reached {
                    time_to_90 = Some(start.elapsed());
                }
                // A trip here is not noise, it is the headline: an
                // overcurrent means this accel value is the dangerous end on
                // this rig, and the trial's "never" below is its consequence.
                if !fb.faults.is_ok() {
                    eprintln!(
                        "  ! accel {accel}: motor reported faults [{}] at {:.0} ms",
                        fb.faults,
                        start.elapsed().as_secs_f64() * 1000.0
                    );
                }
            }
            // Stop early once the setpoint is reached and briefly held — no
            // reason to keep a spun-up wheel turning for the full window.
            if time_to_90.is_some_and(|t| start.elapsed() > t + HOLD_PAST_CROSSING) {
                break;
            }
            std::thread::sleep(SAMPLE_GAP);
        }

        eprintln!(
            "{accel:>5}  {:>12}  {peak_rpm:>11}  {peak_a:>10.2}  {samples:>8}",
            match time_to_90 {
                Some(t) => format!("{:.0} ms", t.as_secs_f64() * 1000.0),
                None => "never".to_string(),
            }
        );
        results.push((accel, time_to_90, peak_rpm, peak_a));
    }

    m.safe_stop();

    // Compare the two ends of the nonzero sweep. The absolute times matter
    // less than their ORDER: whichever end takes longer to reach setpoint is
    // the gentle end, and that is the whole question.
    let timed: Vec<(u8, Duration)> = results
        .iter()
        .filter(|(a, ..)| *a != 0)
        .filter_map(|(a, t, ..)| t.map(|t| (*a, t)))
        .collect();

    eprintln!();
    match (timed.first(), timed.last()) {
        (Some(&(lo_accel, lo_t)), Some(&(hi_accel, hi_t))) if lo_accel != hi_accel => {
            let verdict = if hi_t > lo_t {
                "LARGER IS GENTLER — the direction the docs used to claim, by luck. \
                 The upstream RPM/0.1ms rate unit is not literal."
            } else {
                "LARGER IS HARSHER — the upstream rate unit is literal, and the \
                 defaults this crate removed (stop_accel 5, control 3, quad 5) were \
                 backwards."
            };
            eprintln!(
                "VERDICT: accel {lo_accel} reached 90% in {:.0} ms, accel {hi_accel} \
                 in {:.0} ms.\n{verdict}",
                lo_t.as_secs_f64() * 1000.0,
                hi_t.as_secs_f64() * 1000.0,
            );
        }
        _ => eprintln!(
            "VERDICT: inconclusive — fewer than two trials reached 90% of setpoint. \
             Try a lower M0601_TEST_RPM, or a longer trial window."
        ),
    }
    if let Some((_, _, _, peak)) = results.iter().find(|(a, ..)| *a == 0) {
        eprintln!(
            "Motor default (accel 0) peaked at {peak:.2} A; compare against the rows \
             above to see which byte values are safe on this rig."
        );
    }
    eprintln!(
        "\nRecord the table and the verdict in docs/content/docs/protocol.md \
         (contradiction 6) and close issue #2.\n"
    );
}
