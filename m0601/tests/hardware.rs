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

    let target: i16 = std::env::var("M0601_TEST_RPM")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(120);
    assert!(
        (20..=330).contains(&target),
        "M0601_TEST_RPM must be 20..=330, got {target}"
    );

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
        "stop style", "RPM @ handover", "shed", "time to rest", "peak A"
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
        // Peak current during the stop: the 3 A trip is the whole reason the
        // stop ramp is said to need softening, so measure it rather than
        // reasoning about it.
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
