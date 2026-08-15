//! `drive` — non-interactive, scriptable motion in any of the three motor
//! modes (velocity / current / position).
//!
//! Each mode holds one fixed setpoint and resends it at 50 Hz — the
//! protocol's floor for sustained motion — until either `--secs` elapses or
//! the operator interrupts. Every exit path (normal, `?` error, Ctrl-C,
//! panic) runs [`M0601::safe_stop`]: it forces velocity mode, zeroes, and
//! brakes, so a zero-valued frame can never be misread as "go to 0°"
//! (position) or "zero torque / coast" (current).
//!
//! The interactive [`control`](super::control) command covers the same three
//! modes with live keyboard input; this is its batch/scriptable counterpart.

use std::io::{self, Write};
use std::process::ExitCode;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use m0601::protocol::{
    Frame, frame_current, frame_feedback, frame_position, frame_velocity, raw_to_amps, raw_to_deg,
};
use m0601::{Feedback, M0601, Mode};

use crate::cmd::{CYCLE, POSITION_ENTRY_RPM, REPLY_WAIT, next_deadline};

/// Refresh the winding temperature every Nth cycle with an extra 0x74 query
/// (drive replies carry no temperature).
const TEMP_EVERY: u64 = 10;
/// Redraw the status line every Nth cycle (~10 Hz), not every drive frame.
const DRAW_EVERY: u64 = 5;
/// Give up after this many *consecutive* transact errors (~1 s at 50 Hz). A
/// hard I/O error every cycle for a whole second means the adapter is gone,
/// not a transient RS485 hiccup (those surface as `Ok(None)`, not `Err`).
const MAX_CONSECUTIVE_ERRORS: u32 = 50;

/// A fully-resolved setpoint in the motor's own wire units. Friendly units
/// (amps, degrees) are converted at the CLI boundary by
/// [`m0601::protocol::amps_to_raw`] and [`m0601::protocol::deg_to_raw`].
pub enum Setpoint {
    /// Velocity loop: signed RPM plus an acceleration byte.
    Velocity {
        /// Target speed in RPM (clamped to ±330 on the wire).
        rpm: i16,
        /// Acceleration byte: `1` is the fastest ramp, `0` the motor default.
        accel: u8,
    },
    /// Current loop: signed raw setpoint (`±32767` ≈ ±8 A).
    Current {
        /// Raw current setpoint.
        raw: i16,
    },
    /// Position loop: unsigned raw angle (`0..=32767` = 0°..360°).
    Position {
        /// Raw position setpoint.
        raw: u16,
    },
}

/// What to drive, and for how long.
pub struct Plan {
    /// The mode and setpoint to hold.
    pub setpoint: Setpoint,
    /// Stop after this many seconds; `None` runs until Ctrl-C.
    pub secs: Option<f64>,
}

fn mode_of(sp: &Setpoint) -> Mode {
    match sp {
        Setpoint::Velocity { .. } => Mode::Velocity,
        Setpoint::Current { .. } => Mode::Current,
        Setpoint::Position { .. } => Mode::Position,
    }
}

fn drive_frame(id: u8, sp: &Setpoint) -> Frame {
    match *sp {
        Setpoint::Velocity { rpm, accel } => frame_velocity(id, rpm, accel),
        Setpoint::Current { raw } => frame_current(id, raw),
        Setpoint::Position { raw } => frame_position(id, raw),
    }
}

/// Stops the motor on drop — the single exit funnel for normal end, `?`
/// error, Ctrl-C, and panic. Holds a cheap clone of the handle (both share
/// the one bus) so [`Drop`] can stop the motor without entangling the loop's
/// borrow of the original.
///
/// The panic guarantee depends on unwinding: with `panic = "abort"` in the
/// release profile this `Drop` would not run on a panic (the interactive
/// `control` poll thread instead wraps its loop in `catch_unwind`). The
/// workspace release profile does not set `abort`; keep it that way, or move
/// this loop under `catch_unwind` too, so the promise above holds.
struct StopGuard {
    motor: M0601,
}

impl Drop for StopGuard {
    fn drop(&mut self) {
        self.motor.safe_stop();
    }
}

/// Absolute stop deadline for a timed run.
///
/// `None` (no `--secs`) means "run until Ctrl-C". A *present* `--secs` fails
/// **closed** — an immediate deadline of `start`, so the loop stops on its
/// first cycle — for anything that cannot become a real future instant:
/// a value `try_from_secs_f64` rejects (non-finite/negative), OR a huge
/// finite value that converts but overflows `Instant` when added (which
/// would otherwise *panic*). Never "run forever" on a bad duration.
/// `parse_seconds` bars such values today; this keeps the fail-direction
/// safe if that validation is ever loosened or another caller constructs
/// `Plan` directly.
fn run_deadline(start: Instant, secs: Option<f64>) -> Option<Instant> {
    secs.map(|s| {
        Duration::try_from_secs_f64(s)
            .ok()
            .and_then(|d| start.checked_add(d))
            .unwrap_or(start)
    })
}

pub fn run(port: &str, id: u8, timeout: Duration, plan: Plan) -> m0601::Result<ExitCode> {
    let mode = mode_of(&plan.setpoint);
    let mut motor = M0601::open(port, id, timeout)?;

    // Position mode requires the wheel under 10 RPM before the switch. Fail
    // closed: no telemetry means the speed is unknown, not that it is zero.
    if mode == Mode::Position {
        let speed = motor.query()?.map(|fb| fb.speed_rpm);
        if !crate::cmd::position_entry_allowed(speed) {
            match speed {
                None => {
                    eprintln!(
                        "[x] Refused: no telemetry — cannot confirm the wheel is under {POSITION_ENTRY_RPM} RPM."
                    );
                    eprintln!("    Check 18 V power, wiring (brown → GND), A/B polarity, and --id.");
                }
                Some(rpm) => eprintln!(
                    "[x] Refused: {rpm} RPM — must be under {POSITION_ENTRY_RPM} RPM to enter POSITION mode."
                ),
            }
            return Ok(ExitCode::FAILURE);
        }
    }

    // Arm the stop guard BEFORE the first frame goes out, so every path from
    // here on brakes the motor.
    let stop = StopGuard {
        motor: motor.clone(),
    };

    let (running, handler) = crate::cmd::install_stop_flag();
    if let Err(e) = handler {
        // Without the handler, no signal — Ctrl-C included — unwinds through
        // StopGuard, so any signal terminates the process outright and the
        // motor coasts to a stop instead of braking. A timed run (--secs) that
        // completes on its own still exits the loop normally and brakes.
        eprintln!(
            "[!] could not install signal handler ({e}); a signal (Ctrl-C, SIGTERM, SIGHUP) \
             will now coast the motor rather than brake it. --secs still brakes on completion."
        );
    }

    // Establish the mode (5× 0xA0), then hold the setpoint at 50 Hz.
    motor.set_mode(mode)?;
    let frame = drive_frame(id, &plan.setpoint);
    describe(&plan, mode);

    let start = Instant::now();
    let deadline = run_deadline(start, plan.secs);

    let mut last_fb: Option<Feedback> = None;
    let mut last_temp: Option<u8> = None;
    let mut cycle: u64 = 0;
    let mut errors = 0u32;
    let mut next = Instant::now() + CYCLE;

    while running.load(Ordering::Relaxed) {
        if let Some(d) = deadline
            && Instant::now() >= d
        {
            break;
        }

        // The drive frame goes out EVERY cycle to hold the 50 Hz floor.
        match motor.transact(&frame, REPLY_WAIT) {
            Ok(Some(fb)) => {
                last_fb = Some(fb);
                errors = 0;
            }
            Ok(None) => errors = 0,
            // A transient bus hiccup must not abort the run — the protocol
            // coasts the motor if we truly go silent — but a *persistent*
            // error (adapter unplugged mid-run) should give up rather than
            // spin forever claiming to drive. Dropping StopGuard on the way
            // out still brakes if the wheel is somehow reachable.
            Err(e) => {
                errors += 1;
                // Bus-error notices are diagnostics — to stderr, so a
                // `drive ... 2>/dev/null` keeps only the live status stream.
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    eprint!(
                        "\r[x] bus error {errors}x: {e} — frames are not reaching the motor; stopping.\n"
                    );
                    let _ = io::stderr().flush();
                    break;
                }
                eprint!("\r[!] bus error: {e} (retrying)                    ");
                let _ = io::stderr().flush();
            }
        }

        // Temperature only rides in the 0x74 reply — fetch one as an EXTRA
        // frame so the every-cycle drive cadence is undisturbed.
        if cycle.is_multiple_of(TEMP_EVERY)
            && let Ok(Some(fb)) = motor.transact(&frame_feedback(id), REPLY_WAIT)
            && let Some(t) = fb.temp_c
        {
            last_temp = Some(t);
        }

        if cycle.is_multiple_of(DRAW_EVERY)
            && let Some(fb) = last_fb
        {
            print_status(&fb, last_temp);
        }

        cycle += 1;
        next = next_deadline(next);
    }

    drop(stop); // brake now, before the closing line
    println!(
        "\nStopped and braked after {:.1} s.",
        start.elapsed().as_secs_f64()
    );
    Ok(ExitCode::SUCCESS)
}

/// One-line summary of what is being commanded, printed before the loop.
fn describe(plan: &Plan, mode: Mode) {
    let dur = match plan.secs {
        Some(s) => format!("for {s:.1} s"),
        None => "until Ctrl-C".to_owned(),
    };
    let what = match plan.setpoint {
        Setpoint::Velocity { rpm, accel } => format!("{rpm:+} RPM (accel {accel})"),
        Setpoint::Current { raw } => format!("{:+.3} A (raw {raw})", raw_to_amps(raw)),
        Setpoint::Position { raw } => format!("{:.1}° (raw {raw})", raw_to_deg(raw)),
    };
    println!("Driving {mode} {what} {dur}. Keep the wheel clear; Ctrl-C stops and brakes.");
}

fn print_status(fb: &Feedback, temp: Option<u8>) {
    let temp = temp.map_or_else(|| " --".to_owned(), |t| format!("{t:3}"));
    let fault = if fb.faults.is_ok() {
        "OK".to_owned()
    } else {
        format!("FAULT {}", fb.faults)
    };
    print!(
        "\r  {:<8} | {:+5} RPM | {:+7.3} A | {:6.1}° | {temp} C | {fault}    ",
        fb.mode_name(),
        fb.speed_rpm,
        fb.current_a,
        fb.position_deg,
    );
    let _ = io::stdout().flush();
}

#[cfg(test)]
mod tests {
    use super::{Setpoint, drive_frame, mode_of, run_deadline};
    use crate::cmd::CYCLE;
    use m0601::Mode;
    use m0601::protocol::DRIVE_HZ_MIN;
    use std::time::{Duration, Instant};

    #[test]
    fn a_bad_secs_fails_closed_to_an_immediate_stop() {
        let start = Instant::now();
        // No --secs: run until Ctrl-C.
        assert_eq!(run_deadline(start, None), None);
        // Present but unconvertible: stop immediately (deadline == start),
        // never "run forever". This is the safety-relevant fail direction.
        assert_eq!(run_deadline(start, Some(f64::INFINITY)), Some(start));
        assert_eq!(run_deadline(start, Some(f64::NAN)), Some(start));
        assert_eq!(run_deadline(start, Some(-1.0)), Some(start));
        // Too large to convert at all: fails closed to `start`.
        assert_eq!(run_deadline(start, Some(f64::MAX)), Some(start));
        // Large-but-convertible whose addition would overflow `Instant`:
        // must NOT panic. checked_add maps overflow to `start` (or yields a
        // real future instant where the platform can represent it) — either
        // way, never "run forever" and never a panic.
        assert!(run_deadline(start, Some(1e15)).is_some());
        // A normal value produces a real future deadline.
        assert!(run_deadline(start, Some(2.0)).is_some_and(|d| d > start));
    }

    #[test]
    fn the_cycle_honours_the_protocol_drive_rate() {
        // Same hardcoded 20 ms as the interactive loop, same reason to pin it.
        let slowest_allowed = Duration::from_secs(1) / DRIVE_HZ_MIN;
        assert!(
            CYCLE <= slowest_allowed,
            "{CYCLE:?} per cycle is below the {DRIVE_HZ_MIN} Hz floor ({slowest_allowed:?})"
        );
    }

    #[test]
    fn every_setpoint_declares_the_mode_its_frame_needs() {
        // The mode established by set_mode and the frame sent afterwards
        // must agree: a drive frame's 16-bit value is interpreted per the
        // motor's ACTIVE mode, so a mismatch here means the wheel reads a
        // position setpoint as an RPM figure, or vice versa.
        let cases = [
            (
                Setpoint::Velocity { rpm: 100, accel: 1 },
                Mode::Velocity,
                [0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00, 0x00, 0xE4],
            ),
            (
                Setpoint::Current { raw: 4096 },
                Mode::Current,
                [0x01, 0x64, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB],
            ),
            (
                Setpoint::Position { raw: 16_384 },
                Mode::Position,
                [0x01, 0x64, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97],
            ),
        ];
        for (setpoint, mode, wire) in cases {
            assert_eq!(mode_of(&setpoint), mode);
            assert_eq!(drive_frame(0x01, &setpoint), wire);
        }
    }

    #[test]
    fn the_accel_byte_reaches_the_wire() {
        // Byte 6. `--accel` is exposed only on the velocity subcommand, so
        // clap rejects it elsewhere rather than it being dropped here.
        let f = drive_frame(
            0x01,
            &Setpoint::Velocity {
                rpm: 100,
                accel: 20,
            },
        );
        assert_eq!(f[6], 20);
        assert_eq!(
            f,
            [0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x14, 0x00, 0x00, 0x9B]
        );
    }
}
