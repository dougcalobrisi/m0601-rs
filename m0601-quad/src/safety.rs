//! Fault policy as a pure function: [`assess`] turns one wheel's state
//! into a verdict, the pilot merely executes it. No I/O, no clock reads —
//! `now` comes in as an argument — so the whole table is unit-testable.
//!
//! | condition | reaction |
//! |---|---|
//! | silent < stale_ms | `Continue` — one dropped poll is expected |
//! | silent in stale..dead | `Warn` |
//! | silent ≥ dead_ms | `FailStopAll` — 3 driving + 1 unknown yaws the machine |
//! | `SENSOR_ERR` | `FailStopAll` — closed-loop velocity is meaningless without halls |
//! | `STALL` | `FailStopAll` — one jammed wheel + three driving pivots the chassis |
//! | `OVERHEAT` | `FailStopAll`, **latched by the pilot** — the motor auto-clears at 75 °C and an unattended machine must not lurch back into motion |
//! | motor `OVERCURRENT` / `PHASE_OVERCURRENT` bit | `FailStopAll` — a self-protecting motor tripped; brake the group like any other hard fault |
//! | host current over trip, debounced | `FailStopNoBrake` — zero and latch, but do **not** brake: braking a jammed wheel draws more current, so the stop is held by continuing velocity-0 frames instead |
//! | unknown fault bits | `Warn` — never silently drop what the motor reports |
//!
//! Both stop variants latch (the pilot requires an explicit re-arm — see
//! `pilot::trip` and `pilot::trip_no_brake`, which both `latch_and_zero`);
//! the distinction called out for overheat is that its *fault bit* clears
//! itself, which is exactly why the latch matters.
//!
//! `FailStopAll` outranks `FailStopNoBrake` when verdicts combine: if one
//! wheel demands a braking group stop and another only the no-brake host
//! trip, the vehicle brakes. The no-brake path exists for the isolated
//! host-side current trip, where braking a jammed wheel would draw more
//! current; it is held at zero by the continuing velocity-0 stream.

use std::time::Instant;

use crate::config::Config;
use crate::state::WheelState;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reaction {
    /// Keep driving.
    Continue,
    /// Keep driving, tell the operator.
    Warn(String),
    /// Latch and zero every setpoint, but send **no brake frames**: the
    /// stop is held by the continuing velocity-0 stream. For the host-side
    /// current trip, where braking a jammed wheel would draw more current.
    FailStopNoBrake(String),
    /// Latch, zero every setpoint, run the braking group stop, await
    /// re-arm.
    FailStopAll(String),
}

impl Reaction {
    /// Severity for combining verdicts across four wheels. Higher wins:
    /// a braking group stop (`FailStopAll`) outranks the no-brake host trip.
    fn rank(&self) -> u8 {
        match self {
            Reaction::Continue => 0,
            Reaction::Warn(_) => 1,
            Reaction::FailStopNoBrake(_) => 2,
            Reaction::FailStopAll(_) => 3,
        }
    }

    /// The worse of two reactions.
    pub fn max(self, other: Reaction) -> Reaction {
        if other.rank() > self.rank() {
            other
        } else {
            self
        }
    }
}

/// Judge one wheel. `label` names it in operator-facing reasons
/// ("FRONT-LEFT stall"), because "wheel 2" means nothing at 2 a.m.
pub fn assess(w: &WheelState, label: &str, cfg: &Config, now: Instant) -> Reaction {
    let mut verdict = Reaction::Continue;

    // Telemetry age. A wheel that has never replied has age "forever":
    // fail-closed — assess is only called once the pilot is cycling, and
    // startup separately refuses wheels that don't answer the probe.
    let age = w.last_reply.map(|t| now.duration_since(t));
    match age {
        Some(age) if age < cfg.stale() => {}
        Some(age) if age < cfg.dead() => {
            verdict = verdict.max(Reaction::Warn(format!(
                "{label} telemetry stale ({} ms)",
                age.as_millis()
            )));
        }
        _ => {
            verdict = verdict.max(Reaction::FailStopAll(format!(
                "{label} telemetry dead (three driving wheels + one unknown = yaw)"
            )));
        }
    }

    if let Some(fb) = w.telemetry.fb {
        let f = fb.faults;
        if f.sensor_err() {
            verdict = verdict.max(Reaction::FailStopAll(format!(
                "{label} sensor error — closed-loop velocity needs its halls"
            )));
        }
        if f.stall() {
            verdict = verdict.max(Reaction::FailStopAll(format!("{label} stall")));
        }
        if f.overheat() {
            verdict = verdict.max(Reaction::FailStopAll(format!(
                "{label} overheat (bit auto-clears at 75 C; latch holds)"
            )));
        }
        if f.overcurrent() || f.phase_overcurrent() {
            // A motor that set this bit has entered its own overcurrent
            // protection; stop the group with a braking stop like any other
            // hard fault.
            verdict = verdict.max(Reaction::FailStopAll(format!(
                "{label} motor overcurrent protection tripped"
            )));
        }
        // Any set bit the driver doesn't define — surfaced, never dropped.
        let unknown = f.unknown_bits();
        if unknown != 0 {
            verdict = verdict.max(Reaction::Warn(format!(
                "{label} unknown fault bits 0x{unknown:02X}"
            )));
        }
    }

    // Host-side current trip, debounced: start-up inrush is normal and
    // shorter than the window. The pilot maintains `over_current_since`.
    if let Some(since) = w.over_current_since
        && now.duration_since(since) >= cfg.current_trip()
    {
        verdict = verdict.max(Reaction::FailStopNoBrake(format!(
            "{label} current over {:.1} A for {} ms",
            cfg.limits.current_trip_a,
            cfg.current_trip().as_millis()
        )));
    }

    verdict
}

#[cfg(test)]
mod tests {
    use super::*;
    use m0601::protocol::{ReplyKind, crc8_maxim, parse_feedback};
    use std::time::Duration;

    fn cfg() -> Config {
        Config::parse(include_str!("../wheels.toml")).expect("shipped config parses")
    }

    /// A wheel with a reply `age` ago carrying `faults`.
    fn wheel(now: Instant, age: Duration, faults: u8) -> WheelState {
        let mut frame = [0u8; 10];
        frame[0] = 0x03;
        frame[1] = 0x02;
        frame[8] = faults;
        frame[9] = crc8_maxim(&frame[..9]);
        let fb = parse_feedback(&frame, ReplyKind::Query).expect("valid frame");
        let mut w = WheelState::default();
        w.telemetry.absorb(fb);
        w.last_reply = Some(now - age);
        w
    }

    #[test]
    fn fresh_healthy_telemetry_continues() {
        let now = Instant::now();
        let w = wheel(now, Duration::from_millis(50), 0);
        assert_eq!(assess(&w, "FL", &cfg(), now), Reaction::Continue);
    }

    #[test]
    fn one_missed_poll_is_not_a_fail_stop() {
        // Round-robin polls one wheel every 8th cycle, so ~144 ms between a
        // wheel's polls is normal and one miss doubles it — still well
        // under stale_ms. This is why stale warns instead of stopping.
        let now = Instant::now();
        let w = wheel(now, Duration::from_millis(600), 0);
        assert!(matches!(assess(&w, "FL", &cfg(), now), Reaction::Warn(_)));
    }

    #[test]
    fn dead_telemetry_stops_the_vehicle() {
        let now = Instant::now();
        let w = wheel(now, Duration::from_millis(1600), 0);
        assert!(matches!(
            assess(&w, "FL", &cfg(), now),
            Reaction::FailStopAll(_)
        ));
    }

    #[test]
    fn a_wheel_that_never_replied_fails_closed() {
        let now = Instant::now();
        let w = WheelState::default();
        assert!(matches!(
            assess(&w, "FL", &cfg(), now),
            Reaction::FailStopAll(_)
        ));
    }

    #[test]
    fn each_hard_fault_bit_stops_the_vehicle() {
        // Every defined fault bit — including the motor's own overcurrent
        // (0x02) and phase-overcurrent (0x04) — brakes the whole group.
        let now = Instant::now();
        for bits in [0x01u8, 0x02, 0x04, 0x08, 0x10] {
            let w = wheel(now, Duration::from_millis(10), bits);
            assert!(
                matches!(assess(&w, "FL", &cfg(), now), Reaction::FailStopAll(_)),
                "fault bits 0x{bits:02X} must stop all four wheels"
            );
        }
    }

    #[test]
    fn unknown_fault_bits_warn_but_do_not_stop() {
        let now = Instant::now();
        let w = wheel(now, Duration::from_millis(10), 0x20);
        match assess(&w, "FL", &cfg(), now) {
            Reaction::Warn(msg) => assert!(msg.contains("0x20"), "{msg}"),
            other => panic!("expected Warn, got {other:?}"),
        }
    }

    #[test]
    fn overcurrent_trips_only_after_the_debounce_window() {
        // Inrush shorter than the window must pass; sustained must trip.
        let now = Instant::now();
        let mut w = wheel(now, Duration::from_millis(10), 0);
        w.over_current_since = Some(now - Duration::from_millis(100));
        assert_eq!(assess(&w, "FL", &cfg(), now), Reaction::Continue);
        w.over_current_since = Some(now - Duration::from_millis(450));
        // No-brake: the trip is held by velocity-0 frames, never the
        // electric brake — braking a jammed wheel draws more current.
        assert!(matches!(
            assess(&w, "FL", &cfg(), now),
            Reaction::FailStopNoBrake(_)
        ));
    }

    #[test]
    fn the_worst_reaction_wins_when_combining() {
        let warn = Reaction::Warn("w".into());
        let soft = Reaction::FailStopNoBrake("c".into());
        let stop = Reaction::FailStopAll("s".into());
        assert_eq!(Reaction::Continue.max(warn.clone()), warn.clone());
        assert_eq!(warn.max(soft.clone()), soft.clone());
        // A braking group stop outranks the no-brake host trip: if one wheel
        // demands FailStopAll and another only the host-side no-brake trip,
        // the vehicle brakes. Order must not matter.
        assert_eq!(soft.clone().max(stop.clone()), stop.clone());
        assert_eq!(stop.clone().max(soft), stop.clone());
        assert_eq!(stop.clone().max(Reaction::Continue), stop);
    }
}
