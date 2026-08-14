//! Subcommand implementations.

use std::time::{Duration, Instant};

pub mod control;
pub mod drive;
pub mod info;
pub mod monitor;
pub mod raw;
pub mod scan;
pub mod set_id;

/// The protocol's speed ceiling for entering position mode, in RPM.
pub const POSITION_ENTRY_RPM: i16 = 10;

/// 50 Hz — the protocol's floor for sustained motion. The batch `drive` loop
/// and the interactive [`control`] poll thread both hold this cadence.
pub const CYCLE: Duration = Duration::from_millis(20);

/// Per-cycle reply wait, well inside the [`CYCLE`] budget (a 10-byte frame is
/// ~0.9 ms each way at 115200); the CLI-level `--timeout` is never used in
/// either loop.
pub const REPLY_WAIT: Duration = Duration::from_millis(6);

/// Advance an absolute-deadline scheduler by one [`CYCLE`]: sleep until the
/// current `next` deadline, then return the following one — re-anchoring to
/// *now* if the loop already fell more than a cycle behind, so a slow cycle is
/// absorbed rather than repaid as a burst of back-to-back frames.
///
/// Both 50 Hz loops (`drive` and `control::poll`) share this so their cadence,
/// and its behaviour under overrun, stay identical. Absolute deadlines matter:
/// sleeping `CYCLE` *after* each cycle's variable work (a reply wait, an extra
/// query) would drag the loop below the 50 Hz floor.
pub fn next_deadline(next: Instant) -> Instant {
    let now = Instant::now();
    if next > now {
        std::thread::sleep(next - now);
    }
    let advanced = next + CYCLE;
    // Re-check the clock *after* the sleep: an oversleep (the sleep returning
    // late under OS scheduling) leaves the pre-sleep `now` stale, and comparing
    // against it would let `advanced` be handed back already in the past —
    // exactly the back-to-back burst this branch exists to prevent.
    let now = Instant::now();
    if advanced < now {
        now + CYCLE // fell behind — re-anchor instead of bursting
    } else {
        advanced
    }
}

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
