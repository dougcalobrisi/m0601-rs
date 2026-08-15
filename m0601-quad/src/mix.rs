//! Skid-steer mixing: operator intent → per-side wheel RPM. Pure — no
//! I/O, no time, fully tested before anything renders or transmits it.

use crate::config::Side;

/// Operator intent, both axes in `-1.0..=1.0`.
///
/// Sign convention: **positive turn = left** (counter-clockwise from
/// above), matching ROS's REP-103 angular-z (CCW-positive) — and the
/// downstream consumer's physical `omega_radps`. The keypad keeps its intuitive
/// feel by mapping A/Left → +turn and D/Right → −turn (see `ui::handle_key`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveCmd {
    throttle: f32,
    turn: f32,
}

impl DriveCmd {
    /// Clamps both axes into `-1.0..=1.0`; non-finite values become 0.0
    /// (a NaN throttle must never reach the wire as a NaN-cast RPM).
    pub fn new(throttle: f32, turn: f32) -> Self {
        let sane = |v: f32| {
            if v.is_finite() {
                v.clamp(-1.0, 1.0)
            } else {
                0.0
            }
        };
        Self {
            throttle: sane(throttle),
            turn: sane(turn),
        }
    }

    /// Both sides' normalized outputs, saturation already resolved.
    fn sides(&self) -> (f32, f32) {
        // Positive turn = left (CCW) = right side speeds up.
        let l = self.throttle - self.turn;
        let r = self.throttle + self.turn;
        // Scale-to-fit, NOT clamping. At throttle 1.0, turn 0.5 a naive
        // clamp yields (1.0, 0.5) — ratio 2:1 where 3:1 was commanded, so
        // the turn goes shallow exactly at speed. Scaling both by the
        // overflow preserves the commanded ratio, which is what turn
        // radius geometrically *is*: it gives up speed instead of
        // steering. (WPILib calls this desaturation. PX4/ArduPilot go
        // further and cut throttle to preserve the yaw *rate* itself;
        // ratio-preserving is enough for a bench rover and keeps 100%
        // throttle meaning "as fast as the limit allows".)
        let m = l.abs().max(r.abs());
        if m > 1.0 { (l / m, r / m) } else { (l, r) }
    }
}

/// The RPM to command a wheel on `side`, given `max_rpm` (the config's
/// throttle ceiling, not the motor's 330 limit).
///
/// Front and rear never differ — enforced by this signature: it takes a
/// [`Side`] and nothing else, so a front/rear split is not expressible.
/// An Ackermann mixer would need the axle too; changing this signature is
/// the right place to notice that redesign.
pub fn wheel_rpm(cmd: DriveCmd, side: Side, max_rpm: i16) -> i16 {
    let (l, r) = cmd.sides();
    let v = match side {
        Side::Left => l,
        Side::Right => r,
    };
    // v is in -1.0..=1.0 by construction, so the product fits in i16.
    (v * f32::from(max_rpm)).round() as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Side::{Left, Right};

    fn rpms(t: f32, s: f32, max: i16) -> (i16, i16) {
        let cmd = DriveCmd::new(t, s);
        (wheel_rpm(cmd, Left, max), wheel_rpm(cmd, Right, max))
    }

    #[test]
    fn straight_ahead_drives_both_sides_equally() {
        assert_eq!(rpms(0.5, 0.0, 120), (60, 60));
        assert_eq!(rpms(-0.5, 0.0, 120), (-60, -60));
    }

    #[test]
    fn positive_turn_is_left_right_side_speeds_up() {
        let (l, r) = rpms(0.5, 0.2, 120);
        assert!(r > l, "turning left (CCW): right {r} must outrun left {l}");
    }

    #[test]
    fn spin_in_place_counter_rotates_the_sides() {
        // Positive turn spins CCW: left side backs up, right side drives.
        assert_eq!(rpms(0.0, 1.0, 120), (-120, 120));
        assert_eq!(rpms(0.0, -1.0, 120), (120, -120));
    }

    #[test]
    fn partial_turn_at_full_throttle_preserves_the_ratio() {
        // The test that FAILS under naive clamping: t=1.0, s=0.5 is
        // r=1.5, l=0.5 — a 3:1 ratio. Clamping to (0.5, 1.0) flattens it
        // to 1:2 and the rover understeers exactly at full speed.
        let (l, r) = rpms(1.0, 0.5, 120);
        assert_eq!((l, r), (40, 120), "3:1 preserved by scaling, not 2:1");
    }

    #[test]
    fn zero_command_is_zero_on_both_sides() {
        assert_eq!(rpms(0.0, 0.0, 120), (0, 0));
    }

    #[test]
    fn non_finite_axes_become_zero_not_nan_rpm() {
        assert_eq!(
            rpms(f32::NAN, 0.7, 120).0,
            wheel_rpm(DriveCmd::new(0.0, 0.7), Left, 120)
        );
        assert_eq!(rpms(f32::INFINITY, f32::NEG_INFINITY, 120), (0, 0));
    }

    #[test]
    fn axes_clamp_to_unit_range() {
        assert_eq!(rpms(5.0, 0.0, 120), (120, 120));
        assert_eq!(rpms(-5.0, 0.0, 120), (-120, -120));
    }

    #[test]
    fn no_valid_config_can_command_more_than_its_max_rpm() {
        // Ties the mixer to the config's ceiling: whatever the operator
        // does, no wheel exceeds max_rpm.
        for t in [-1.0f32, -0.7, 0.0, 0.3, 1.0] {
            for s in [-1.0f32, -0.5, 0.0, 0.5, 1.0] {
                let (l, r) = rpms(t, s, 120);
                assert!(l.abs() <= 120 && r.abs() <= 120, "t={t} s={s} -> ({l},{r})");
            }
        }
    }
}
