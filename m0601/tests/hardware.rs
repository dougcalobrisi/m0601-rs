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

/// Setpoint for the motion captures; override with `M0601_TEST_RPM`.
/// Bounded so a typo cannot command something wild.
fn test_rpm() -> i16 {
    let target: i16 = std::env::var("M0601_TEST_RPM")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(120);
    assert!(
        (20..=330).contains(&target),
        "M0601_TEST_RPM must be 20..=330, got {target}"
    );
    target
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
/// capture settles it on hardware, the way `reply_checksum_capture` settled
/// the reply CRC.
///
/// For each accel byte it commands a step from rest to `M0601_TEST_RPM`
/// (default 120) and reports **time to 90% of setpoint** and **peak current**
/// during the ramp. Both matter: the first gives the direction, the second is
/// the quantity the old 3 A bus-overcurrent worry rested on.
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

    let target = test_rpm();
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
        "accel", "t to 90%", "peak RPM", "peak |A|", "samples"
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
                "LARGER IS GENTLER — the direction this crate's docs originally \
                 claimed (correctly, though no published source states it). The \
                 upstream RPM/0.1ms rate unit is not literal."
            } else {
                "LARGER IS HARSHER — the upstream rate unit is literal, and the \
                 shipped defaults (stop_accel 5, control 3, quad 5) sit on the \
                 harsh end. Re-examine them before trusting any of them."
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

/// How a stop trial asks the wheel to slow down.
#[derive(Clone, Copy)]
enum StopStyle {
    /// Send no drive frames at all — the wheel coasts, as it does when a
    /// controller dies. The control every other row is read against: a
    /// deceleration no faster than this one is doing nothing.
    Coast,
    /// The electric brake byte, the final phase of `safe_stop`.
    Brake,
    /// Velocity-0 drive frames at this accel byte — the ramp phase of
    /// `safe_stop`, which uses `BusTiming::stop_accel` (default 5).
    Ramp(u8),
}

impl StopStyle {
    fn label(self) -> String {
        match self {
            Self::Coast => "coast (no frames)".to_string(),
            Self::Brake => "brake".to_string(),
            Self::Ramp(a) => format!("velocity-0 @ accel {a}"),
        }
    }
}

/// What `safe_stop`'s ramp phase actually accomplishes before the brake takes
/// over — the follow-up question from `accel_direction_capture`.
///
/// `safe_stop` sends five velocity-0 rounds 20 ms apart (100 ms) at
/// `BusTiming::stop_accel`, then five brake rounds. Since the accel byte
/// measured at ~3.6 ms per RPM per unit on spin-*up*, a stop at accel `5`
/// would shed only a few RPM in that 100 ms window — meaning the brake does
/// nearly all the work and the ramp's documented role is largely notional.
/// Spin-up and spin-down need not behave alike, though, so this measures the
/// deceleration directly.
///
/// Each trial spins the wheel up to `M0601_TEST_RPM` (default 120), then
/// applies one stop style and reports the speed still present at the 100 ms
/// handover point plus the total time to rest. `Coast` is the control.
///
/// ```sh
/// M0601_PORT=/dev/ttyUSB0 M0601_ALLOW_MOTION=1 \
///   cargo test -p m0601 --test hardware -- --ignored --nocapture stop_ramp_capture
/// ```
///
/// **Spins the wheel, repeatedly. Get it off the ground first.**
#[test]
#[ignore = "needs hardware AND spins the wheel: set M0601_PORT and M0601_ALLOW_MOTION=1"]
fn stop_ramp_capture() {
    let _guard = port_guard();
    assert_eq!(
        std::env::var("M0601_ALLOW_MOTION").as_deref(),
        Ok("1"),
        "stop_ramp_capture spins the wheel; set M0601_ALLOW_MOTION=1 to allow it \
         (make sure the wheel is off the ground)"
    );

    /// The stop styles to compare, control first. The ramp values span the
    /// whole byte deliberately: on spin-*up* accel 255 is ~250x gentler than
    /// accel 1, so if deceleration honours the byte at all, these two rows
    /// cannot possibly match.
    const TRIALS: [StopStyle; 8] = [
        StopStyle::Coast,
        StopStyle::Brake,
        StopStyle::Ramp(0),
        StopStyle::Ramp(1),
        StopStyle::Ramp(5),
        StopStyle::Ramp(20),
        StopStyle::Ramp(100),
        StopStyle::Ramp(255),
    ];
    /// `safe_stop` gives the ramp five rounds 20 ms apart before the brake
    /// rounds begin. Whatever speed is left at this instant is what the brake
    /// inherits — the number this whole capture exists to find.
    const HANDOVER: Duration = Duration::from_millis(100);
    /// Below this the wheel counts as stopped.
    const REST_RPM: i16 = 5;
    /// Give up on a stop that will not arrive.
    const STOP_CAP: Duration = Duration::from_millis(6000);
    const SAMPLE_GAP: Duration = Duration::from_millis(5);
    const REPLY_WAIT: Duration = Duration::from_millis(4);
    /// Accel for the spin-*up* before each trial, so every trial starts from
    /// the same place. The fastest ramp, to keep the spin-up short.
    const SPINUP_ACCEL: u8 = 1;

    let target = test_rpm();

    let mut guard = StopOnDrop(Some(open()));
    let m = guard.0.as_mut().expect("just constructed");
    m.set_mode(Mode::Velocity).expect("set velocity mode");

    eprintln!(
        "\nstop ramp capture: spin up to {target} RPM, then stop. Handover at \
         {} ms is where safe_stop switches from the ramp to the brake.\n",
        HANDOVER.as_millis()
    );
    eprintln!(
        "{:>22}  {:>14}  {:>10}  {:>13}  {:>8}",
        "stop style", "RPM @ handover", "shed", "time to rest", "peak |A|"
    );

    let mut results: Vec<(StopStyle, Option<i16>, Option<Duration>)> = Vec::new();

    for style in TRIALS {
        // ── spin up ──────────────────────────────────────────────────────
        m.set_mode(Mode::Velocity).expect("set velocity mode");
        let up = m0601::protocol::frame_velocity(m.id(), target, SPINUP_ACCEL);
        let want = f32::from(target) * 0.9;
        let spin_deadline = Instant::now() + Duration::from_millis(2500);
        let mut at_speed: Option<Instant> = None;
        while Instant::now() < spin_deadline {
            if let Ok(Some(fb)) = m.transact(&up, REPLY_WAIT)
                && at_speed.is_none()
                && f32::from(fb.speed_rpm) >= want
            {
                at_speed = Some(Instant::now());
            }
            // Hold briefly at speed so every trial decelerates from a settled
            // wheel rather than from one still accelerating.
            if at_speed.is_some_and(|t| t.elapsed() > Duration::from_millis(400)) {
                break;
            }
            std::thread::sleep(SAMPLE_GAP);
        }
        assert!(
            at_speed.is_some(),
            "wheel never reached {want:.0} RPM before the {} trial",
            style.label()
        );

        // ── stop ─────────────────────────────────────────────────────────
        let stop_frame = match style {
            StopStyle::Coast => None,
            StopStyle::Brake => Some(m0601::protocol::frame_brake(m.id())),
            StopStyle::Ramp(a) => Some(m0601::protocol::frame_velocity(m.id(), 0, a)),
        };
        let start = Instant::now();
        let mut at_handover: Option<i16> = None;
        let mut time_to_rest: Option<Duration> = None;
        // Peak current during the stop: the 3 A trip was the stated reason the
        // stop ramp needed softening, so measure it rather than reasoning
        // about it.
        let mut peak_a: f32 = 0.0;

        while start.elapsed() < STOP_CAP {
            // Coast sends a feedback query, which commands no motion — the
            // wheel is slowing because nothing is driving it.
            let sample = match &stop_frame {
                Some(f) => m.transact(f, REPLY_WAIT),
                None => m.query_with(REPLY_WAIT),
            };
            if let Ok(Some(fb)) = sample {
                let elapsed = start.elapsed();
                peak_a = peak_a.max(fb.current_a.abs());
                if at_handover.is_none() && elapsed >= HANDOVER {
                    at_handover = Some(fb.speed_rpm.abs());
                }
                if fb.speed_rpm.abs() < REST_RPM {
                    time_to_rest = Some(elapsed);
                    break;
                }
            }
            std::thread::sleep(SAMPLE_GAP);
        }

        eprintln!(
            "{:>22}  {:>14}  {:>10}  {:>13}  {peak_a:>8.2}",
            style.label(),
            at_handover.map_or_else(|| "-".to_string(), |r| format!("{r} RPM")),
            at_handover.map_or_else(
                || "-".to_string(),
                |r| format!("{}%", (i32::from(target - r) * 100) / i32::from(target))
            ),
            time_to_rest.map_or_else(
                || format!(">{:.1} s", STOP_CAP.as_secs_f64()),
                |t| format!("{:.0} ms", t.as_secs_f64() * 1000.0)
            ),
        );
        results.push((style, at_handover, time_to_rest));

        // Leave the wheel genuinely stopped between trials whatever the style
        // under test did, and let the brake release before the next spin-up.
        m.safe_stop();
        std::thread::sleep(Duration::from_millis(700));
    }

    m.safe_stop();

    // ── Verdict ──────────────────────────────────────────────────────────
    //
    // Two separate questions, and conflating them is easy:
    //   1. Do velocity-0 frames decelerate at all? (ramp vs COAST)
    //   2. Does `stop_accel` change that deceleration? (ramp vs ramp)
    // Only (2) is what the `stop_accel` knob claims to do.
    let handover_of = |want: &str| {
        results
            .iter()
            .find(|(s, ..)| s.label() == want)
            .and_then(|(_, h, _)| *h)
    };
    let ramps: Vec<(u8, i16)> = results
        .iter()
        .filter_map(|(s, h, _)| match (s, h) {
            (StopStyle::Ramp(a), Some(h)) => Some((*a, *h)),
            _ => None,
        })
        .collect();

    eprintln!();
    if let (Some(coast), Some(brake)) = (handover_of("coast (no frames)"), handover_of("brake")) {
        eprintln!(
            "Q1 — do velocity-0 frames decelerate at all? Coasting leaves {coast} RPM at \
             the handover and the brake leaves {brake} RPM."
        );
        match ramps.iter().map(|(_, h)| *h).min() {
            Some(best) if best < coast - 5 => eprintln!(
                "     YES: the ramp phase leaves {best} RPM, well under coasting. It is \
                 real deceleration, not just the wheel losing speed on its own."
            ),
            Some(best) => eprintln!(
                "     NO: the ramp phase leaves {best} RPM, no better than coasting \
                 ({coast}). The brake delivers the entire stop."
            ),
            None => eprintln!("     inconclusive: no ramp trial produced a handover sample."),
        }
    }

    if ramps.len() >= 2 {
        let lo = ramps.iter().map(|(_, h)| *h).min().unwrap_or(0);
        let hi = ramps.iter().map(|(_, h)| *h).max().unwrap_or(0);
        let spread = hi - lo;
        let listed: Vec<String> = ramps.iter().map(|(a, h)| format!("{a}->{h}")).collect();
        eprintln!(
            "\nQ2 — does stop_accel change the deceleration? accel->RPM at handover: {}",
            listed.join(", ")
        );
        // The byte spans a ~250x range on spin-up. If deceleration honoured it
        // even weakly, the ends could not land within a few RPM of each other.
        if spread <= 5 {
            eprintln!(
                "     NO — spread is {spread} RPM across the whole byte range. The accel \
                 byte has NO measurable effect on deceleration, even though on spin-up \
                 the same values differ by more than 250x. stop_accel is inert on this \
                 firmware, and any doc claiming a given value decelerates more gently \
                 than another is wrong."
            );
        } else {
            eprintln!(
                "     YES — spread is {spread} RPM across the byte range, so the value \
                 does matter and stop_accel's documented role stands."
            );
        }
    }

    eprintln!(
        "\nRecord this in docs/content/docs/concepts/stopping-safely.md and in the \
         SAFE_STOP_ACCEL docs. Note the caveat: an unloaded wheel. A loaded one may \
         draw enough current for the difference to matter.\n"
    );
}

/// Audit trail for [`stop_ramp_capture`]'s surprising result — that the accel
/// byte is inert on deceleration.
///
/// That claim rests on one number per trial, and a single sample can hide a
/// bug: a byte that never reached the wire, a wheel that was not really at
/// speed, a handover sampled at the wrong instant. This dumps the **whole**
/// deceleration curve at the two extremes of the byte, side by side, plus the
/// literal TX frame for each so the wire byte is visible rather than assumed.
///
/// If the two columns track each other sample for sample, the byte is inert.
/// If `stop_ramp_capture` were measuring something else, the curves would
/// diverge somewhere even when their 100 ms samples happened to agree.
///
/// ```sh
/// M0601_PORT=/dev/ttyUSB0 M0601_ALLOW_MOTION=1 \
///   cargo test -p m0601 --test hardware -- --ignored --nocapture stop_ramp_curve_capture
/// ```
///
/// **Spins the wheel. Get it off the ground first.**
#[test]
#[ignore = "needs hardware AND spins the wheel: set M0601_PORT and M0601_ALLOW_MOTION=1"]
fn stop_ramp_curve_capture() {
    let _guard = port_guard();
    assert_eq!(
        std::env::var("M0601_ALLOW_MOTION").as_deref(),
        Ok("1"),
        "stop_ramp_curve_capture spins the wheel; set M0601_ALLOW_MOTION=1 to allow it"
    );

    /// The extremes. On spin-up these differ by more than 250x.
    const PROBES: [u8; 2] = [1, 255];
    const SAMPLE_GAP: Duration = Duration::from_millis(5);
    const REPLY_WAIT: Duration = Duration::from_millis(4);
    const CURVE_LEN: Duration = Duration::from_millis(400);

    let target = test_rpm();

    let mut guard = StopOnDrop(Some(open()));
    let m = guard.0.as_mut().expect("just constructed");
    m.set_mode(Mode::Velocity).expect("set velocity mode");

    // (elapsed_ms, rpm, amps) per probe.
    let mut curves: Vec<Vec<(u128, i16, f32)>> = Vec::new();

    for accel in PROBES {
        m.set_mode(Mode::Velocity).expect("set velocity mode");
        let up = m0601::protocol::frame_velocity(m.id(), target, 1);
        let want = f32::from(target) * 0.9;
        let spin_deadline = Instant::now() + Duration::from_millis(2500);
        let mut at_speed: Option<Instant> = None;
        while Instant::now() < spin_deadline {
            if let Ok(Some(fb)) = m.transact(&up, REPLY_WAIT)
                && at_speed.is_none()
                && f32::from(fb.speed_rpm) >= want
            {
                at_speed = Some(Instant::now());
            }
            if at_speed.is_some_and(|t| t.elapsed() > Duration::from_millis(400)) {
                break;
            }
            std::thread::sleep(SAMPLE_GAP);
        }
        assert!(
            at_speed.is_some(),
            "wheel never reached speed for accel {accel}"
        );

        let stop = m0601::protocol::frame_velocity(m.id(), 0, accel);
        let hex: Vec<String> = stop.iter().map(|b| format!("{b:02X}")).collect();
        eprintln!(
            "accel {accel:>3}: TX {}  (byte 6 = 0x{:02X})",
            hex.join(" "),
            stop[6]
        );
        assert_eq!(stop[6], accel, "the accel byte must reach the wire");

        let start = Instant::now();
        let mut curve = Vec::new();
        while start.elapsed() < CURVE_LEN {
            if let Ok(Some(fb)) = m.transact(&stop, REPLY_WAIT) {
                curve.push((
                    start.elapsed().as_millis(),
                    fb.speed_rpm.abs(),
                    fb.current_a.abs(),
                ));
            }
            std::thread::sleep(SAMPLE_GAP);
        }
        curves.push(curve);

        m.safe_stop();
        std::thread::sleep(Duration::from_millis(700));
    }

    m.safe_stop();

    let (a, b) = (&curves[0], &curves[1]);
    // An empty or threadbare curve must never read as agreement: a `worst` of
    // zero over zero comparisons would print CONFIRMED from no data, and
    // dropped replies are a real failure mode on this link (a reply window
    // near 1 ms loses every sample).
    const MIN_SAMPLES: usize = 20;
    assert!(
        a.len() >= MIN_SAMPLES && b.len() >= MIN_SAMPLES,
        "too few samples to audit anything (accel {}: {}, accel {}: {}) — \
         were the replies dropped?",
        PROBES[0],
        a.len(),
        PROBES[1],
        b.len()
    );
    eprintln!(
        "\n{:>13}  {:>18}  {:>18}  {:>6}",
        "t (a / b)",
        format!("accel {} (rpm/A)", PROBES[0]),
        format!("accel {} (rpm/A)", PROBES[1]),
        "delta"
    );
    // Pair samples by elapsed time, not by index: one dropped reply would
    // shift every later index by a full sample period and silently compare
    // different instants. Each accel-`a` sample is read against the accel-`b`
    // sample nearest in time, and pairs further apart than half a sample
    // period are skipped rather than compared.
    let mut worst: i16 = 0;
    let mut paired: usize = 0;
    let mut j = 0usize;
    for &(t_a, rpm_a, amp_a) in a {
        while j + 1 < b.len() && b[j + 1].0.abs_diff(t_a) < b[j].0.abs_diff(t_a) {
            j += 1;
        }
        let (t_b, rpm_b, amp_b) = b[j];
        if t_b.abs_diff(t_a) > 5 {
            continue;
        }
        paired += 1;
        let d = (rpm_a - rpm_b).abs();
        worst = worst.max(d);
        eprintln!(
            "{t_a:>5}/{t_b:<5} ms  {rpm_a:>11} / {amp_a:>4.2}  {rpm_b:>11} / {amp_b:>4.2}  {d:>6}"
        );
    }
    assert!(
        paired >= MIN_SAMPLES,
        "only {paired} time-aligned sample pairs — the two runs' cadences \
         diverged too far to audit"
    );
    eprintln!(
        "\nWorst divergence between accel {} and accel {} across {paired} \
         time-aligned pairs: {worst} RPM.",
        PROBES[0], PROBES[1]
    );
    eprintln!(
        "{}",
        if worst <= 5 {
            "CONFIRMED: the curves are the same. The accel byte does not affect \
             deceleration — stop_ramp_capture's single-point result was not an artefact."
        } else {
            "CONTRADICTED: the curves diverge, so the byte DOES affect deceleration \
             and stop_ramp_capture's handover sample was misleading. Investigate."
        }
    );
}

/// Is the acceleration ramp a straight line, or a first-order lag? The caveat
/// [`accel_direction_capture`] leaves open: time-to-90% alone cannot
/// distinguish `rate ∝ 1/accel` (a linear ramp to setpoint) from `τ ∝ accel`
/// (an exponential approach) — the seven summary rows fit both models. The
/// published "~3.6 ms per RPM per unit" law assumes the straight line, so this
/// dumps the full spin-up curve at two mid-range accel values and inspects the
/// shape directly.
///
/// Two discriminators, both immune to the fixed ~60 ms start-up offset because
/// they use only *differences* between crossing times:
///
/// - **Quartile-span ratio** `(t75 − t50) / (t50 − t25)`, where `tN` is the
///   first crossing of N% of setpoint. A linear ramp gives 1.0; a first-order
///   lag gives `ln(2) / ln(4/3) ≈ 1.71`.
/// - **Chord bow**: how far the measured curve rises above the straight chord
///   drawn from the 25% crossing to the 90% crossing, as a percentage of
///   setpoint. A straight ramp bows ~0%; a first-order lag bows ~12%.
///
/// ```sh
/// M0601_PORT=/dev/ttyUSB0 M0601_ALLOW_MOTION=1 \
///   cargo test -p m0601 --test hardware -- --ignored --nocapture accel_curve_capture
/// ```
///
/// **Spins the wheel, slowly and for several seconds. Get it off the ground.**
#[test]
#[ignore = "needs hardware AND spins the wheel: set M0601_PORT and M0601_ALLOW_MOTION=1"]
fn accel_curve_capture() {
    let _guard = port_guard();
    assert_eq!(
        std::env::var("M0601_ALLOW_MOTION").as_deref(),
        Ok("1"),
        "accel_curve_capture spins the wheel; set M0601_ALLOW_MOTION=1 to allow it \
         (make sure the wheel is off the ground)"
    );

    /// Mid-range values, where the ramp is slow enough that its shape spans
    /// many samples but the trial still finishes in seconds. The extremes are
    /// no better: at `1` the whole ramp fits in ~45 samples, and at `255` a
    /// trial would run minutes.
    const PROBES: [u8; 2] = [5, 20];
    const SAMPLE_GAP: Duration = Duration::from_millis(5);
    const REPLY_WAIT: Duration = Duration::from_millis(4);
    /// Stop sampling once the wheel is essentially at setpoint.
    const DONE_FRAC: f64 = 0.98;

    let target = test_rpm();

    let mut guard = StopOnDrop(Some(open()));
    let m = guard.0.as_mut().expect("just constructed");
    m.set_mode(Mode::Velocity).expect("set velocity mode");

    eprintln!(
        "\naccel curve capture: step 0 -> {target} RPM at accel {:?}, full curve\n",
        PROBES
    );

    // (elapsed_ms, signed rpm, signed amps) per probe.
    let mut curves: Vec<Vec<(u128, i16, f32)>> = Vec::new();

    for accel in PROBES {
        // Start every trial from a genuine rest, with the brake released —
        // same preamble as accel_direction_capture, for the same reason.
        m.safe_stop();
        std::thread::sleep(Duration::from_millis(800));
        if let Ok(Some(fb)) = m.query() {
            assert!(
                fb.speed_rpm.abs() < 5,
                "wheel did not come to rest before the accel {accel} trial: {} RPM",
                fb.speed_rpm
            );
        }
        m.set_mode(Mode::Velocity).expect("set velocity mode");
        let release = m0601::protocol::frame_velocity(m.id(), 0, accel);
        let _ = m.transact(&release, REPLY_WAIT);
        std::thread::sleep(Duration::from_millis(200));

        let step = m0601::protocol::frame_velocity(m.id(), target, accel);
        let hex: Vec<String> = step.iter().map(|b| format!("{b:02X}")).collect();
        eprintln!(
            "accel {accel:>3}: TX {}  (byte 6 = 0x{:02X})",
            hex.join(" "),
            step[6]
        );
        assert_eq!(step[6], accel, "the accel byte must reach the wire");

        // Cap from the provisional law (t90 ≈ 62 ms + 3.56 ms × 0.9·target ×
        // accel), doubled, plus slack — generous enough that only a wheel far
        // slower than either model predicts ever hits it.
        let cap_ms = 500.0 + 2.0 * (62.0 + 3.56 * 0.9 * f64::from(target) * f64::from(accel));
        let cap = Duration::from_millis(cap_ms as u64);
        let done = f64::from(target) * DONE_FRAC;

        let start = Instant::now();
        let mut curve = Vec::new();
        while start.elapsed() < cap {
            if let Ok(Some(fb)) = m.transact(&step, REPLY_WAIT) {
                curve.push((start.elapsed().as_millis(), fb.speed_rpm, fb.current_a));
                if f64::from(fb.speed_rpm) >= done {
                    break;
                }
            }
            std::thread::sleep(SAMPLE_GAP);
        }
        curves.push(curve);

        m.safe_stop();
        std::thread::sleep(Duration::from_millis(700));
    }

    m.safe_stop();

    // First crossing of `frac`·target, linearly interpolated between the
    // samples that straddle it.
    let crossing = |curve: &[(u128, i16, f32)], frac: f64| -> Option<f64> {
        let want = f64::from(target) * frac;
        let mut prev: Option<(u128, i16)> = None;
        for &(t, rpm, _) in curve {
            let r = f64::from(rpm);
            if r >= want {
                return Some(match prev {
                    Some((pt, prpm)) if f64::from(prpm) < want && r > f64::from(prpm) => {
                        let pr = f64::from(prpm);
                        pt as f64 + (t - pt) as f64 * (want - pr) / (r - pr)
                    }
                    _ => t as f64,
                });
            }
            prev = Some((t, rpm));
        }
        None
    };

    let mut ratios: Vec<f64> = Vec::new();
    for (accel, curve) in PROBES.iter().zip(&curves) {
        // A threadbare curve, or one that never reached 90%, cannot support a
        // shape verdict — fail loudly rather than judging from thin data.
        assert!(
            curve.len() >= 30,
            "accel {accel}: only {} samples — were the replies dropped?",
            curve.len()
        );
        let (t25, t50, t75, t90) = (
            crossing(curve, 0.25),
            crossing(curve, 0.50),
            crossing(curve, 0.75),
            crossing(curve, 0.90),
        );
        let (Some(t25), Some(t50), Some(t75), Some(t90)) = (t25, t50, t75, t90) else {
            panic!(
                "accel {accel}: the wheel never crossed 90% of setpoint inside the \
                 window — no shape verdict possible. Longer cap, or lower M0601_TEST_RPM."
            );
        };

        // Print the curve, decimated to ~48 rows; the crossings above use
        // every sample.
        let step_by = (curve.len() / 48).max(1);
        eprintln!(
            "\naccel {accel} curve ({} samples, every {step_by}th shown):",
            curve.len()
        );
        for (t, rpm, a) in curve.iter().step_by(step_by) {
            eprintln!("  {t:>6} ms  {rpm:>4} rpm  {a:+.2} A");
        }

        let ratio = (t75 - t50) / (t50 - t25);
        ratios.push(ratio);

        // Chord bow: maximum rise of the curve above the straight line from
        // the 25% crossing to the 90% crossing, as % of setpoint.
        let slope = (0.90 - 0.25) * f64::from(target) / (t90 - t25);
        let mut bow: f64 = 0.0;
        for &(t, rpm, _) in curve {
            let t = t as f64;
            if t >= t25 && t <= t90 {
                let chord = 0.25 * f64::from(target) + slope * (t - t25);
                bow = bow.max(f64::from(rpm) - chord);
            }
        }
        let bow_pct = bow / f64::from(target) * 100.0;

        // ms per RPM per accel unit over the mid-ramp, for the published law.
        let ms_per_rpm_unit = (t75 - t25) / (0.5 * f64::from(target)) / f64::from(*accel);

        eprintln!(
            "accel {accel}: t25 {t25:.0} ms, t50 {t50:.0} ms, t75 {t75:.0} ms, t90 {t90:.0} ms\n\
             \x20         span ratio (t75-t50)/(t50-t25) = {ratio:.2} \
             (linear 1.00, first-order lag 1.71)\n\
             \x20         chord bow +{bow_pct:.1}% of setpoint (linear ~0%, lag ~12%)\n\
             \x20         mid-ramp slope: {ms_per_rpm_unit:.2} ms per RPM per accel unit"
        );
    }

    eprintln!();
    if ratios.iter().all(|r| *r < 1.25) {
        eprintln!(
            "VERDICT: LINEAR RAMP. The approach to setpoint is a straight line, so \
             the published time-per-RPM law (protocol.md) stands as stated."
        );
    } else if ratios.iter().all(|r| *r > 1.45) {
        eprintln!(
            "VERDICT: FIRST-ORDER LAG. The approach is asymptotic, so the published \
             law must be restated as a time constant (τ ∝ accel), not a linear ramp \
             rate. Update protocol.md and protocol.rs."
        );
    } else {
        eprintln!(
            "VERDICT: AMBIGUOUS — the two accel values disagree, or a ratio landed \
             between the models. Inspect the printed curves before touching the docs."
        );
    }
}

/// One telemetry sample from a phase capture: elapsed ms, signed RPM, signed
/// amps, raw fault byte.
type Sample = (u128, i16, f32, u8);

/// Spin the wheel up to `target` and hold there briefly, so a following
/// measurement starts from a settled wheel. Returns false if it never
/// reached 90% of setpoint.
fn spin_up_to(m: &mut M0601, target: i16, hold: Duration) -> bool {
    let up = m0601::protocol::frame_velocity(m.id(), target, 1);
    let want = f32::from(target) * 0.9;
    let deadline = Instant::now() + Duration::from_millis(2500);
    let mut at_speed: Option<Instant> = None;
    while Instant::now() < deadline {
        if let Ok(Some(fb)) = m.transact(&up, Duration::from_millis(4))
            && at_speed.is_none()
            && f32::from(fb.speed_rpm) >= want
        {
            at_speed = Some(Instant::now());
        }
        if at_speed.is_some_and(|t| t.elapsed() > hold) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    false
}

/// Winding temperature, which only a `0x74` query reply carries.
fn winding_temp(m: &mut M0601) -> Option<u8> {
    m.query().ok().flatten().and_then(|fb| fb.temp_c)
}

/// Where the braking energy goes — the loose end from [`stop_ramp_capture`].
///
/// That capture showed velocity-0 frames shedding speed far faster than
/// coasting while the reported current sat near **zero**, which is physically
/// odd: the kinetic energy has to go somewhere. It also recorded current as a
/// magnitude, discarding the sign, so a regenerative (negative) current would
/// have been invisible as such.
///
/// This answers the question that actually affects code — **is the reported
/// current field blind while the motor brakes?** — because `m0601-quad` trips
/// its vehicle-wide stop off that field (`limits.current_trip_a`), and a field
/// that reads ~0 during braking is a hole in that monitor. It does **not**
/// settle what crosses the bus: [`braking_current_fast_capture`] shows this
/// link aliases events shorter than a few milliseconds, so these samples speak
/// to what the field *reports*, never to what the hardware is doing.
///
/// Part A logs signed current and fault bits through steady running, a
/// velocity-0 stop, and a brake stop. Part B is a thermal probe: equal numbers
/// of spin-ups, differing only in how the wheel returns to rest (braked vs
/// coasting), to see whether braking heats the motor measurably. Part B is
/// expected to be inconclusive — the temperature field is 1 °C granular — so
/// treat a null result there as "too small to see", not as evidence of
/// absence.
///
/// ```sh
/// M0601_PORT=/dev/ttyUSB0 M0601_ALLOW_MOTION=1 \
///   cargo test -p m0601 --test hardware -- --ignored --nocapture braking_current_capture
/// ```
///
/// **Spins the wheel many times, for a few minutes. Get it off the ground.**
#[test]
#[ignore = "needs hardware AND spins the wheel: set M0601_PORT and M0601_ALLOW_MOTION=1"]
fn braking_current_capture() {
    let _guard = port_guard();
    assert_eq!(
        std::env::var("M0601_ALLOW_MOTION").as_deref(),
        Ok("1"),
        "braking_current_capture spins the wheel; set M0601_ALLOW_MOTION=1 to allow it"
    );

    const SAMPLE_GAP: Duration = Duration::from_millis(5);
    const REPLY_WAIT: Duration = Duration::from_millis(4);
    const HOLD: Duration = Duration::from_millis(400);
    /// Cycles per thermal trial.
    const CYCLES: usize = 12;
    /// A coast is over when the wheel is this slow, or when the cap expires.
    const COAST_CAP: Duration = Duration::from_millis(8000);
    const REST_RPM: i16 = 5;

    let target = test_rpm();

    let mut guard = StopOnDrop(Some(open()));
    let m = guard.0.as_mut().expect("just constructed");
    m.set_mode(Mode::Velocity).expect("set velocity mode");

    // ── Part A: signed current and faults through each phase ─────────────
    eprintln!("\n== Part A: signed current through each phase (steady = {target} RPM) ==\n");

    /// The three phases to sample. An enum rather than label strings, so the
    /// frame selection cannot silently diverge from what the row is called.
    #[derive(Clone, Copy, PartialEq)]
    enum Phase {
        Steady,
        Vel0,
        Brake,
    }
    // One entry per phase: its label and its telemetry samples.
    let mut phases: Vec<(&str, Vec<Sample>)> = Vec::new();

    for phase in [Phase::Steady, Phase::Vel0, Phase::Brake] {
        let label = match phase {
            Phase::Steady => "steady",
            Phase::Vel0 => "velocity-0",
            Phase::Brake => "brake",
        };
        assert!(
            spin_up_to(m, target, HOLD),
            "wheel never reached speed before the {label} phase"
        );
        let frame = match phase {
            Phase::Steady => m0601::protocol::frame_velocity(m.id(), target, 1),
            Phase::Vel0 => m0601::protocol::frame_velocity(m.id(), 0, 5),
            Phase::Brake => m0601::protocol::frame_brake(m.id()),
        };
        let window = if phase == Phase::Steady {
            Duration::from_millis(200)
        } else {
            Duration::from_millis(600)
        };
        let start = Instant::now();
        let mut samples = Vec::new();
        while start.elapsed() < window {
            if let Ok(Some(fb)) = m.transact(&frame, REPLY_WAIT) {
                samples.push((
                    start.elapsed().as_millis(),
                    fb.speed_rpm,
                    // SIGNED, deliberately: a regenerative current would show
                    // as negative, and every earlier capture threw that away.
                    fb.current_a,
                    fb.faults.0,
                ));
                if phase != Phase::Steady && fb.speed_rpm.abs() < REST_RPM {
                    break;
                }
            }
            std::thread::sleep(SAMPLE_GAP);
        }
        phases.push((label, samples));
        m.safe_stop();
        std::thread::sleep(Duration::from_millis(700));
    }

    eprintln!(
        "{:>12}  {:>8}  {:>10}  {:>10}  {:>10}  {:>14}",
        "phase", "samples", "min A", "max A", "mean |A|", "faults seen"
    );
    for (label, s) in &phases {
        if s.is_empty() {
            eprintln!("{label:>12}  {:>8}", 0);
            continue;
        }
        let min = s.iter().map(|x| x.2).fold(f32::INFINITY, f32::min);
        let max = s.iter().map(|x| x.2).fold(f32::NEG_INFINITY, f32::max);
        let mean = s.iter().map(|x| x.2.abs()).sum::<f32>() / s.len() as f32;
        let bits = s.iter().fold(0u8, |acc, x| acc | x.3);
        eprintln!(
            "{label:>12}  {:>8}  {min:>10.2}  {max:>10.2}  {mean:>10.2}  {:>14}",
            s.len(),
            format!("{}", m0601::Faults(bits))
        );
    }

    // The first few samples of a stop are where any spike lives.
    for (label, s) in &phases {
        if *label == "steady" {
            continue;
        }
        let head: Vec<String> = s
            .iter()
            .take(12)
            .map(|(ms, rpm, a, _)| format!("{ms}ms {rpm}rpm {a:+.2}A"))
            .collect();
        eprintln!("\n{label} first samples: {}", head.join(", "));
    }

    // ── Part B: thermal probe ────────────────────────────────────────────
    //
    // Both trials perform the SAME number of spin-ups, so spin-up heating
    // cancels. They differ only in how the wheel returns to rest. If braking
    // heats the motor, the braked trial should end hotter.
    eprintln!("\n== Part B: thermal probe ({CYCLES} cycles each) ==\n");

    let thermal = |m: &mut M0601, braked: bool| -> (Option<u8>, Option<u8>, f64, usize) {
        let before = winding_temp(m);
        let t0 = Instant::now();
        // Completed cycles are counted and reported: a failed spin-up shortens
        // a trial, and unequal cycle counts would break the "same number of
        // spin-ups" control the comparison rests on.
        let mut completed = 0usize;
        for _ in 0..CYCLES {
            m.set_mode(Mode::Velocity).ok();
            if !spin_up_to(m, target, Duration::from_millis(150)) {
                break;
            }
            completed += 1;
            if braked {
                let zero = m0601::protocol::frame_velocity(m.id(), 0, 5);
                let start = Instant::now();
                while start.elapsed() < Duration::from_millis(1200) {
                    if let Ok(Some(fb)) = m.transact(&zero, REPLY_WAIT)
                        && fb.speed_rpm.abs() < REST_RPM
                    {
                        break;
                    }
                    std::thread::sleep(SAMPLE_GAP);
                }
            } else {
                // Coast: send nothing that drives. Poll only, so the wheel
                // slows on its own — the control for "same spin-ups, no brake".
                let start = Instant::now();
                while start.elapsed() < COAST_CAP {
                    if let Ok(Some(fb)) = m.query_with(REPLY_WAIT)
                        && fb.speed_rpm.abs() < REST_RPM
                    {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(20));
                }
            }
        }
        let elapsed = t0.elapsed().as_secs_f64();
        std::thread::sleep(Duration::from_millis(500));
        (before, winding_temp(m), elapsed, completed)
    };

    let (b0, b1, b_secs, b_cycles) = thermal(m, true);
    eprintln!(
        "braked  : {} -> {} °C over {b_secs:.0} s ({b_cycles}/{CYCLES} cycles)",
        b0.map_or("?".into(), |t| t.to_string()),
        b1.map_or("?".into(), |t| t.to_string()),
    );

    // Let it settle back down so the second trial does not inherit the first
    // trial's heat. Not a full cooldown — just enough to stop the trend.
    eprintln!("cooling 90 s…");
    std::thread::sleep(Duration::from_secs(90));

    let (c0, c1, c_secs, c_cycles) = thermal(m, false);
    eprintln!(
        "coasting: {} -> {} °C over {c_secs:.0} s ({c_cycles}/{CYCLES} cycles)",
        c0.map_or("?".into(), |t| t.to_string()),
        c1.map_or("?".into(), |t| t.to_string()),
    );
    if b_cycles != c_cycles {
        eprintln!(
            "    ! trials completed UNEQUAL cycle counts ({b_cycles} vs {c_cycles}) — \
             the equal-spin-ups control does not hold for this run."
        );
    }

    m.safe_stop();

    // ── Verdict ──────────────────────────────────────────────────────────
    eprintln!();
    if let Some((_, s)) = phases.iter().find(|(l, _)| *l == "velocity-0") {
        let max_mag = s.iter().map(|x| x.2.abs()).fold(0.0_f32, f32::max);
        let most_negative = s.iter().map(|x| x.2).fold(f32::INFINITY, f32::min);
        eprintln!(
            "Q — is the current field blind while braking? velocity-0 peak magnitude \
             {max_mag:.2} A, most negative {most_negative:+.2} A."
        );
        if max_mag < 1.0 {
            eprintln!(
                "    YES: the wheel sheds most of its speed while the reported current \
                 stays under {max_mag:.2} A. A monitor watching this field cannot see a \
                 velocity-0 stop. What crosses the BUS is NOT settled by this — the \
                 link aliases short transients (see braking_current_fast_capture), so \
                 this is a claim about the telemetry, not about the hardware."
            );
        } else {
            eprintln!(
                "    NO: braking draws {max_mag:.2} A, visible to a current monitor. \
                 The earlier ~0 A readings were an artefact."
            );
        }
        if most_negative < -0.1 {
            eprintln!(
                "    Current DOES go negative ({most_negative:+.2} A) — regeneration is \
                 reported, just small."
            );
        } else {
            eprintln!(
                "    Current never goes meaningfully negative, so the field is not \
                 reporting regeneration at all (it is not a signed-energy readout)."
            );
        }
    }
    match (b0, b1, c0, c1) {
        (Some(b0), Some(b1), Some(c0), Some(c1)) => {
            let (db, dc) = (i16::from(b1) - i16::from(b0), i16::from(c1) - i16::from(c0));
            eprintln!("\nThermal: braked ΔT {db:+} °C, coasting ΔT {dc:+} °C.");
            if (db - dc).abs() <= 1 {
                eprintln!(
                    "    INCONCLUSIVE, as expected: the difference is within the 1 °C \
                     resolution. Too little energy in an unloaded rotor to resolve."
                );
                eprintln!(
                    "    The control is also weaker than it looks: the two trials take \
                     very different wall-clock ({b_secs:.0} s braked vs {c_secs:.0} s \
                     coasting, since a coast to rest takes seconds), so they differ in \
                     idle time and ambient drift as well as in braking. Reading any \
                     1 °C difference as signal would be wrong. Resolving this properly \
                     needs a loaded wheel or an external probe, not this test."
                );
            } else if db > dc {
                eprintln!(
                    "    Braking heats the winding {} °C more than coasting for the same \
                     number of spin-ups. The trials also differ in duration, so confirm \
                     that before drawing any conclusion from it.",
                    db - dc
                );
            } else {
                eprintln!("    Braking did NOT heat the winding more than coasting.");
            }
        }
        _ => eprintln!("\nThermal: inconclusive — temperature unavailable."),
    }
}

/// Is [`braking_current_capture`]'s near-zero braking current real, or an
/// artefact of sampling too slowly?
///
/// That capture sampled every ~9 ms and saw the whole braking transient in a
/// **single sample** (−0.63 A for velocity-0, −1.99 A for the brake). A single
/// sample is exactly what aliasing looks like: if the real waveform is spiky,
/// a 9 ms sampler will miss most of it and report a mean near zero whatever is
/// actually happening.
///
/// This re-runs the stop at the protocol's rate ceiling — the bus is opened
/// with a much smaller inter-frame gap and the loop never sleeps — and prints
/// the achieved cadence alongside the curve, so the reading can be judged
/// against the rate it was taken at.
///
/// It matters because an earlier draft of the docs claimed a velocity-0 stop
/// cannot trip the 3 A *bus* protection "since almost nothing crosses the
/// bus". That claim rested on the mean being real, and this capture is why it
/// was retracted. (The separate finding — that a current *monitor* cannot see
/// a velocity-0 stop — holds either way, and more strongly if the waveform is
/// spiky, since `m0601-quad` polls each wheel only every ~144 ms.)
///
/// ```sh
/// M0601_PORT=/dev/ttyUSB0 M0601_ALLOW_MOTION=1 \
///   cargo test -p m0601 --test hardware -- --ignored --nocapture braking_current_fast_capture
/// ```
///
/// **Spins the wheel. Get it off the ground first.**
#[test]
#[ignore = "needs hardware AND spins the wheel: set M0601_PORT and M0601_ALLOW_MOTION=1"]
fn braking_current_fast_capture() {
    let _guard = port_guard();
    assert_eq!(
        std::env::var("M0601_ALLOW_MOTION").as_deref(),
        Ok("1"),
        "braking_current_fast_capture spins the wheel; set M0601_ALLOW_MOTION=1"
    );

    /// Well under the default 2.5 ms. The default is sized so a reply can
    /// never still be on the wire when the next frame starts; `transact`
    /// already waits for the reply, so the gap is the throttle here, not the
    /// safety margin. If it is too small the replies stop parsing — which
    /// shows up as a collapsed sample count, printed below.
    const FAST_GAP: Duration = Duration::from_micros(300);
    /// 1.8 ms, and this is as fast as the link goes.
    ///
    /// The 300 µs bus gap above does **not** speed anything up: frames land on
    /// a ~4 ms quantum set by the USB-serial link, not by our pacing, and the
    /// FTDI `latency_timer` is already 1 ms. Tightening this window to 1.1 ms
    /// to try to fit one quantum instead loses essentially every reply (0 of
    /// ~30 samples), which also measures the motor's turnaround as **>1.1 ms**
    /// — consistent with, and tighter than, the "~1.0–1.5 ms" in
    /// `m0601-quad/wheels.toml`.
    ///
    /// So ~8–10 ms is the floor for telemetry sampling here, and events
    /// shorter than that cannot be resolved through this link at all.
    const REPLY_WAIT: Duration = Duration::from_micros(1800);
    const WINDOW: Duration = Duration::from_millis(250);

    let target = test_rpm();
    let id = motor_id();

    let bus = m0601::Bus::open(&port(), TIMEOUT)
        .expect("open serial port")
        .with_min_gap(FAST_GAP);
    let mut guard = StopOnDrop(Some(bus.motor(id).expect("valid id")));
    let m = guard.0.as_mut().expect("just constructed");
    m.set_mode(Mode::Velocity).expect("set velocity mode");

    eprintln!(
        "\nfast braking capture: min_gap {} µs, reply_wait {} µs\n",
        FAST_GAP.as_micros(),
        REPLY_WAIT.as_micros()
    );

    for (label, ramp_stop) in [("velocity-0", true), ("brake", false)] {
        assert!(
            spin_up_to(m, target, Duration::from_millis(400)),
            "wheel never reached speed before the {label} phase"
        );
        let frame = if ramp_stop {
            m0601::protocol::frame_velocity(m.id(), 0, 5)
        } else {
            m0601::protocol::frame_brake(m.id())
        };

        let start = Instant::now();
        let mut samples: Vec<(u128, i16, f32)> = Vec::new();
        // No sleep: the bus's own gap is the only pacing.
        while start.elapsed() < WINDOW {
            if let Ok(Some(fb)) = m.transact(&frame, REPLY_WAIT) {
                samples.push((start.elapsed().as_micros(), fb.speed_rpm, fb.current_a));
            }
        }

        let cadence = if samples.len() > 1 {
            let span = samples[samples.len() - 1].0 - samples[0].0;
            span as f64 / (samples.len() - 1) as f64 / 1000.0
        } else {
            f64::NAN
        };
        let peak = samples.iter().map(|s| s.2.abs()).fold(0.0_f32, f32::max);
        let mean = if samples.is_empty() {
            0.0
        } else {
            samples.iter().map(|s| s.2.abs()).sum::<f32>() / samples.len() as f32
        };
        // How much of the window was spent above a current that matters?
        let over_1a = samples.iter().filter(|s| s.2.abs() >= 1.0).count();

        eprintln!(
            "{label}: {} samples, {cadence:.2} ms apart, peak |{peak:.2}| A, \
             mean |{mean:.2}| A, {over_1a} samples >= 1 A",
            samples.len()
        );
        let head: Vec<String> = samples
            .iter()
            .take(24)
            .map(|(us, rpm, a)| format!("{:.1}ms {rpm}rpm {a:+.2}A", *us as f64 / 1000.0))
            .collect();
        eprintln!("  {}\n", head.join(", "));

        m.safe_stop();
        std::thread::sleep(Duration::from_millis(700));
    }

    m.safe_stop();
    eprintln!(
        "Compare against braking_current_capture at ~9 ms: velocity-0 peak 0.63 A, \
         mean 0.03 A; brake peak 1.99 A, mean 0.16 A.\n\
         \n\
         Expect velocity-0 to reproduce (it does: ~0.69 A peak, ~0.05 A mean) and the \
         BRAKE to come out higher (~2.28 A peak, ~0.30 A mean, several consecutive \
         samples over 1 A). That difference is the point: the brake transient IS \
         aliased at ~9 ms, which proves aliasing is real on this link. Velocity-0 \
         reproducing is therefore reassuring but NOT proof — no achievable rate here \
         can resolve a sub-4 ms event. Claims about this phase must be about what the \
         telemetry reports, not about what crosses the bus."
    );
}
