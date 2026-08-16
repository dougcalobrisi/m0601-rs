//! Host-side setpoint slew limiting: [`SlewLimiter`].
//!
//! # Why this is in a motor driver
//!
//! The M0601 closes its own velocity, current and position loops in firmware
//! (see [`Mode`](crate::Mode)), and the drive frame's `accel` byte ramps the
//! motor *toward* whatever setpoint it was last given. Neither of those bounds
//! how fast the **host** may move the setpoint itself. A keystroke, a joystick
//! snap, or a mixer output that jumps from one cycle to the next is a step
//! change on the wire, and a large step on a loaded wheel can spike current
//! into the motor's 3 A bus-overcurrent protection — the same failure the
//! [`stop_accel`](crate::BusTiming) default of `5` exists to avoid on the way
//! down.
//!
//! That makes the rate of change of a setpoint a property of *this motor*, not
//! of any particular robot, so it lives here. What stays with the application
//! is everything above it: kinematics, the drive loop, and any closed loop over
//! a robot-level quantity. See the crate docs for that boundary.
//!
//! # No clock, by design
//!
//! [`SlewLimiter`] never reads the time. The caller passes the elapsed time to
//! [`step`](SlewLimiter::step), because the driver owns no scheduler and no
//! `Clock` seam — that is the application's, and keeping it there is what lets
//! a limiter be tested with plain arithmetic instead of sleeps.

use std::time::Duration;

use crate::error::{Error, Result};

/// A first-order slew-rate limiter for a setpoint.
///
/// Each [`step`](Self::step) moves the held setpoint toward a target by at most
/// `max_change_per_second * elapsed`, so a step change on the input becomes a
/// bounded ramp on the output.
///
/// The limiter is unit-agnostic: use it for RPM (its usual home) or for amps in
/// current mode. It is **not** for position mode, where the setpoint is an
/// absolute angle the motor interpolates to on its own — slewing an angle
/// commands a different move, not a gentler one.
///
/// It holds no clock, allocates nothing, and never panics.
///
/// ```
/// use std::time::Duration;
/// use m0601::SlewLimiter;
///
/// // 300 RPM/s, stepped at 50 Hz => at most 6 RPM per cycle.
/// let mut limiter = SlewLimiter::new(300.0)?;
/// let cycle = Duration::from_millis(20);
///
/// assert_eq!(limiter.step(250.0, cycle), 6.0);
/// assert_eq!(limiter.step(250.0, cycle), 12.0);
///
/// // A stop path must not ramp: jump straight to zero.
/// limiter.reset_to(0.0);
/// assert_eq!(limiter.current_setpoint(), 0.0);
/// # Ok::<(), m0601::Error>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlewLimiter {
    max_change_per_second: f32,
    current_setpoint: f32,
}

impl SlewLimiter {
    /// The gentlest useful limiter: one unit of setpoint per second.
    ///
    /// A last-resort fallback for callers that cannot propagate an error —
    /// `SlewLimiter::new(rate).unwrap_or(SlewLimiter::GENTLE)` — since [`new`]
    /// refuses a bad rate rather than silently disabling limiting. Slower is
    /// always the safe direction: this errs toward a machine that barely moves
    /// rather than one that steps its setpoint.
    ///
    /// [`new`]: Self::new
    pub const GENTLE: Self = Self {
        max_change_per_second: 1.0,
        current_setpoint: 0.0,
    };

    /// Create a limiter that allows at most `max_change_per_second` units of
    /// setpoint change per second, starting from a held setpoint of `0.0`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidSlewRate`] if the rate is not finite and
    /// strictly positive. A zero or negative rate would freeze the setpoint at
    /// its initial value, and a `NaN` rate would silently disable limiting —
    /// both are far more dangerous as a running default than as a startup
    /// error, so they are refused here rather than sanitized away.
    pub fn new(max_change_per_second: f32) -> Result<Self> {
        if !max_change_per_second.is_finite() || max_change_per_second <= 0.0 {
            return Err(Error::InvalidSlewRate(max_change_per_second));
        }
        Ok(Self {
            max_change_per_second,
            current_setpoint: 0.0,
        })
    }

    /// Advance one cycle toward `target_setpoint` and return the new held
    /// setpoint.
    ///
    /// The change is capped at `max_change_per_second * elapsed`, so passing
    /// the loop's cycle time each iteration produces the configured ramp. A
    /// zero `elapsed` holds the setpoint unchanged.
    ///
    /// A non-finite `target_setpoint` holds the current setpoint instead of
    /// poisoning it: in a 50 Hz drive loop a `NaN` that reached the wire would
    /// be latched into every later cycle.
    pub fn step(&mut self, target_setpoint: f32, elapsed: Duration) -> f32 {
        if !target_setpoint.is_finite() {
            return self.current_setpoint;
        }
        // Both bounds are finite and non-negative (the rate is validated in
        // `new`, and a `Duration` is never negative or NaN), so neither the
        // multiply nor the `clamp` can trip on a NaN bound.
        let max_change = self.max_change_per_second * elapsed.as_secs_f32();
        let change = (target_setpoint - self.current_setpoint).clamp(-max_change, max_change);
        self.current_setpoint += change;
        self.current_setpoint
    }

    /// Jump the held setpoint straight to `value`, bypassing the rate limit.
    ///
    /// This is the deliberate escape hatch, and there are two places a drive
    /// loop needs it:
    ///
    /// - **Stop paths.** A fail-safe that ramps is not a fail-safe. On an
    ///   all-stop, a latched fault, or a dead operator link, write
    ///   `reset_to(0.0)` and send zero immediately.
    /// - **Holding a brake.** While the wheels are being braked they are
    ///   physically stopping, so the limiter must not keep winding toward a
    ///   still-latched throttle. Left to wind, it would command the fully
    ///   ramped setpoint in a *single* step the moment the brake released —
    ///   producing the exact lurch the limiter exists to prevent. Hold it at
    ///   `0.0` while braking so release ramps up from zero like any other
    ///   start.
    ///
    /// A non-finite `value` is ignored, for the reason given on
    /// [`step`](Self::step).
    pub fn reset_to(&mut self, value: f32) {
        if value.is_finite() {
            self.current_setpoint = value;
        }
    }

    /// The setpoint currently held, as last returned by [`step`](Self::step).
    pub fn current_setpoint(&self) -> f32 {
        self.current_setpoint
    }

    /// The configured maximum change per second.
    pub fn max_change_per_second(&self) -> f32 {
        self.max_change_per_second
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CYCLE: Duration = Duration::from_millis(20);

    fn limiter(rate: f32) -> SlewLimiter {
        match SlewLimiter::new(rate) {
            Ok(l) => l,
            Err(e) => unreachable!("valid rate rejected: {e}"),
        }
    }

    #[test]
    fn new_rejects_rates_that_would_disable_limiting() {
        for bad in [0.0, -1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(
                SlewLimiter::new(bad).is_err(),
                "rate {bad} must be refused, not sanitized"
            );
        }
        assert!(SlewLimiter::new(f32::MIN_POSITIVE).is_ok());
    }

    #[test]
    fn steps_are_capped_at_rate_times_elapsed() {
        let mut l = limiter(300.0);
        // 300 RPM/s * 20 ms = 6 RPM per cycle.
        assert_eq!(l.step(250.0, CYCLE), 6.0);
        assert_eq!(l.step(250.0, CYCLE), 12.0);
        assert_eq!(l.current_setpoint(), 12.0);
    }

    #[test]
    fn converges_to_the_target_and_then_holds_it() {
        let mut l = limiter(300.0);
        for _ in 0..100 {
            l.step(60.0, CYCLE);
        }
        assert_eq!(
            l.step(60.0, CYCLE),
            60.0,
            "must settle exactly, not oscillate"
        );
        assert_eq!(l.step(60.0, CYCLE), 60.0);
    }

    #[test]
    fn never_overshoots_a_target_closer_than_one_step() {
        let mut l = limiter(300.0);
        assert_eq!(l.step(2.0, CYCLE), 2.0, "a 2 RPM target must not become 6");
    }

    #[test]
    fn limiting_is_symmetric_on_the_way_down_and_negative() {
        let mut l = limiter(300.0);
        l.reset_to(100.0);
        assert_eq!(l.step(0.0, CYCLE), 94.0);
        l.reset_to(0.0);
        assert_eq!(l.step(-250.0, CYCLE), -6.0, "reverse must ramp too");
    }

    #[test]
    fn a_full_reversal_ramps_through_zero_rather_than_stepping() {
        let mut l = limiter(300.0);
        l.reset_to(250.0);
        let after = l.step(-250.0, CYCLE);
        assert_eq!(after, 244.0);
        assert!(
            after > 0.0,
            "F->B must not cross to full reverse in one cycle"
        );
    }

    #[test]
    fn reset_to_bypasses_the_limit_for_stop_paths() {
        let mut l = limiter(300.0);
        l.reset_to(250.0);
        assert_eq!(l.current_setpoint(), 250.0);
        l.reset_to(0.0);
        assert_eq!(l.current_setpoint(), 0.0, "a fail-safe must not ramp");
    }

    #[test]
    fn zero_elapsed_holds_the_setpoint() {
        let mut l = limiter(300.0);
        l.reset_to(42.0);
        assert_eq!(l.step(250.0, Duration::ZERO), 42.0);
    }

    #[test]
    fn non_finite_target_holds_instead_of_poisoning_the_setpoint() {
        let mut l = limiter(300.0);
        l.reset_to(50.0);
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert_eq!(l.step(bad, CYCLE), 50.0);
            assert!(l.current_setpoint().is_finite(), "{bad} leaked into state");
        }
        // A good target after a bad one still works.
        assert_eq!(l.step(56.0, CYCLE), 56.0);
    }

    #[test]
    fn non_finite_reset_is_ignored() {
        let mut l = limiter(300.0);
        l.reset_to(7.0);
        l.reset_to(f32::NAN);
        assert_eq!(l.current_setpoint(), 7.0);
    }

    #[test]
    fn an_absurd_elapsed_does_not_overshoot_or_go_non_finite() {
        let mut l = limiter(f32::MAX);
        let out = l.step(250.0, Duration::from_secs(u32::MAX.into()));
        assert_eq!(out, 250.0, "a huge dt saturates at the target, not past it");
        assert!(out.is_finite());
    }
}
