//! Subcommand implementations.

pub mod control;
pub mod drive;
pub mod info;
pub mod monitor;
pub mod raw;
pub mod scan;
pub mod set_id;

/// The protocol's speed ceiling for entering position mode, in RPM.
pub const POSITION_ENTRY_RPM: i16 = 10;

/// Whether the wheel is *confirmed* slow enough to switch into position mode.
///
/// Takes the last reported speed, or `None` when no telemetry has arrived.
///
/// Fails closed on `None`: an unknown speed is not a zero one, so a bus
/// whose RX path is dead never satisfies this guard.
///
/// Note the polarity, which is where the trap is. Asking "is it confirmed
/// slow?" refuses `None` naturally. The inverted form does not: a
/// "refuse if too fast" check written as
/// `speed.is_some_and(|rpm| rpm.unsigned_abs() >= LIMIT)` yields `false`
/// for `None`, reads as "not too fast", and lets the switch through on a
/// silent bus.
///
/// Shared by the batch (`drive position`) and interactive (`control`, `P`)
/// paths so the two guards cannot drift apart.
///
/// Uses `unsigned_abs` rather than `abs`: `i16::MIN.abs()` overflows and
/// panics in debug builds, and `speed_rpm` comes straight off the wire, so
/// `0x8000` in the speed bytes is reachable from a corrupt frame. A panic
/// here would be a panic on the safety path.
pub fn position_entry_allowed(speed_rpm: Option<i16>) -> bool {
    matches!(speed_rpm, Some(rpm) if rpm.unsigned_abs() < POSITION_ENTRY_RPM.unsigned_abs())
}

#[cfg(test)]
mod tests {
    use super::{POSITION_ENTRY_RPM, position_entry_allowed};

    #[test]
    fn unknown_speed_is_refused_not_treated_as_zero() {
        assert!(!position_entry_allowed(None));
    }

    #[test]
    fn the_threshold_is_exclusive_in_both_directions() {
        assert!(position_entry_allowed(Some(0)));
        assert!(position_entry_allowed(Some(9)));
        assert!(position_entry_allowed(Some(-9)));
        // Exactly at the limit is refused — "under 10 RPM", not "10 or less".
        assert!(!position_entry_allowed(Some(POSITION_ENTRY_RPM)));
        assert!(!position_entry_allowed(Some(-POSITION_ENTRY_RPM)));
        assert!(!position_entry_allowed(Some(330)));
        assert!(!position_entry_allowed(Some(-330)));
    }

    #[test]
    fn extremes_do_not_overflow_on_abs() {
        // i16::MIN.abs() would panic in debug; the guard must refuse it.
        assert!(!position_entry_allowed(Some(i16::MIN)));
        assert!(!position_entry_allowed(Some(i16::MAX)));
    }
}
