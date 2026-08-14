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
//! | motor `OVERCURRENT` / `PHASE_OVERCURRENT` bit | `FailStopNoBrake` — the motor has usually already cut power, but one frame can't prove it, so never brake a wheel drawing overcurrent (the brake shorts the windings and pulls more); held by velocity-0 like the host trip |
//! | host current over trip, debounced | `FailStopNoBrake` — zero and latch, but do **not** brake: braking a still-energized jammed wheel draws more current, so the stop is held by continuing velocity-0 frames instead |
//! | unknown fault bits | `Warn` — never silently drop what the motor reports |
//!
//! Every stop variant latches (the pilot requires an explicit re-arm); the
//! distinction called out for overheat is that its *fault bit* clears
//! itself, which is exactly why the latch matters.
//!
//! `FailStopNoBrake` is a **veto that outranks `FailStopAll`**: if any wheel
//! is drawing overcurrent the whole vehicle stops *without* braking, even
//! when another wheel (dead, stalled, …) independently demands a braking
//! stop — because any overcurrent on the shared bus makes adding brake
//! current the more dangerous option. The trade is that a compound failure
//! loses the firm electric brake and is held by velocity-0 alone.

use std::time::Instant;

use crate::config::Config;
use crate::state::WheelState;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reaction {
    Continue,
    /// Keep driving, tell the operator.
    Warn(String),
    /// Latch, zero every setpoint, run the braking group stop, await
    /// re-arm.
    FailStopAll(String),
    /// Latch and zero every setpoint, but send **no brake frames**: the
    /// stop is held by the continuing velocity-0 stream. Raised by any
    /// overcurrent condition (host trip or a motor-reported bit), where
    /// braking a still-energized wheel would only draw more current.
    ///
    /// This is a **veto**: it outranks [`FailStopAll`](Reaction::FailStopAll)
    /// in aggregation, so if *any* wheel is drawing overcurrent the whole
    /// vehicle stops without braking, even when another wheel independently
    /// demands a braking stop. Both variants latch and hold at zero; the
    /// veto only drops the brake frames (see `pilot::trip_no_brake`).
    FailStopNoBrake(String),
}

impl Reaction {
    /// Precedence for combining verdicts across four wheels. Higher wins.
    ///
    /// `FailStopNoBrake` ranks **above** `FailStopAll`: it is a "must not
    /// brake" veto, not merely a milder stop, so once any wheel raises it no
    /// wheel is braked. Both still latch and hold the vehicle at zero.
    fn rank(&self) -> u8 {
        match self {
            Reaction::Continue => 0,
            Reaction::Warn(_) => 1,
            Reaction::FailStopAll(_) => 2,
            Reaction::FailStopNoBrake(_) => 3,
        }
    }

    /// The higher-precedence of two reactions (see [`rank`](Reaction::rank)).
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
            // GUARD: never brake a wheel that is drawing overcurrent. The
            // electric brake shorts the windings and would only pull more
            // current on a still-energized wheel. A motor that set this bit
            // has *usually* already entered protection and cut power — but a
            // single telemetry frame can't prove it has, so don't rely on
            // that: treat a motor-reported overcurrent exactly like the
            // host-side trip below. FailStopNoBrake is a veto that outranks
            // FailStopAll (see Reaction::rank), so this holds even when
            // another wheel independently demands a braking stop.
            verdict = verdict.max(Reaction::FailStopNoBrake(format!(
                "{label} motor overcurrent protection tripped"
            )));
        }
        let unknown = f.0 & !0x1F;
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
    fn sensor_stall_and_overheat_brake_the_vehicle() {
        let now = Instant::now();
        for bits in [0x01u8, 0x08, 0x10] {
            let w = wheel(now, Duration::from_millis(10), bits);
            assert!(
                matches!(assess(&w, "FL", &cfg(), now), Reaction::FailStopAll(_)),
                "fault bits 0x{bits:02X} must brake all four wheels"
            );
        }
    }

    #[test]
    fn motor_overcurrent_stops_without_braking() {
        // A wheel drawing overcurrent must never be braked — the electric
        // brake shorts the windings and pulls more current. Both the bus
        // (0x02) and phase (0x04) overcurrent bits take the no-brake path,
        // matching the host-side current trip.
        let now = Instant::now();
        for bits in [0x02u8, 0x04] {
            let w = wheel(now, Duration::from_millis(10), bits);
            assert!(
                matches!(assess(&w, "FL", &cfg(), now), Reaction::FailStopNoBrake(_)),
                "overcurrent bits 0x{bits:02X} must stop without braking"
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
    fn the_no_brake_veto_dominates_when_combining() {
        let warn = Reaction::Warn("w".into());
        let soft = Reaction::FailStopNoBrake("c".into());
        let stop = Reaction::FailStopAll("s".into());
        assert_eq!(Reaction::Continue.max(warn.clone()), warn.clone());
        assert_eq!(warn.clone().max(stop.clone()), stop.clone());
        // The no-brake veto outranks a braking stop: if one wheel is drawing
        // overcurrent and another is dead, the vehicle stops WITHOUT braking
        // — adding brake current on an already-overcurrent bus is the more
        // dangerous option. Order must not matter.
        assert_eq!(soft.clone().max(stop.clone()), soft.clone());
        assert_eq!(stop.clone().max(soft.clone()), soft.clone());
        assert_eq!(soft.clone().max(warn), soft.clone());
        assert_eq!(stop.clone().max(Reaction::Continue), stop);
    }

    #[test]
    fn overcurrent_on_one_wheel_vetoes_braking_a_dead_wheel() {
        // Compound failure across wheels: FL is drawing overcurrent while FR
        // has gone dead (no telemetry). The vehicle must stop, but the
        // overcurrent veto means it is held at zero, never braked.
        let now = Instant::now();
        let overcurrent = wheel(now, Duration::from_millis(10), 0x02);
        let dead = WheelState::default(); // never replied → FailStopAll

        let verdict = assess(&overcurrent, "FL", &cfg(), now).max(assess(&dead, "FR", &cfg(), now));
        assert!(
            matches!(verdict, Reaction::FailStopNoBrake(_)),
            "overcurrent anywhere must veto braking, even beside a dead wheel"
        );
    }
}
