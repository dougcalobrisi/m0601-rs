//! Bus-wide timing configuration ([`BusTiming`]) and the wire-occupancy
//! budget ([`bus_period`]).
//!
//! Pure config and arithmetic: nothing here touches the transport. The
//! constants are the values the crate has always used; [`BusTiming`] gathers
//! the tunable ones so a whole bus can be reconfigured in one call.

use std::time::Duration;

/// How long to listen for answers to a broadcast ID query.
const BROADCAST_WAIT: Duration = Duration::from_millis(300);
/// Gap between the five repetitions of a mode-switch frame.
const MODE_REPEAT_GAP: Duration = Duration::from_millis(20);
/// Gap between the five repetitions of a set-ID frame.
const SET_ID_REPEAT_GAP: Duration = Duration::from_millis(50);
/// Settling time after the set-ID sequence before re-querying.
const SET_ID_SETTLE: Duration = Duration::from_millis(500);
/// Gap between the frames of a [`M0601::safe_stop`](crate::M0601::safe_stop)
/// sequence (50 Hz).
const SAFE_STOP_GAP: Duration = Duration::from_millis(20);
/// Acceleration byte for the velocity-0 rounds of a stop sequence.
///
/// **Not** the fastest ramp (`1`): a hard ramp-to-zero on a loaded wheel can
/// trip the motor's own 3 A bus-overcurrent protection *during* the stop, at
/// which point it stops responding to drive commands and the controlled
/// deceleration is defeated — the opposite of what a safe stop wants. A
/// moderate ramp decelerates firmly without provoking that trip, and the
/// brake rounds that follow still deliver the hard final hold. `5` matches
/// the value `m0601-quad` recommends for launch (see its `limits.accel`).
const SAFE_STOP_ACCEL: u8 = 5;

/// Default minimum idle gap enforced between frames on a bus — see
/// [`Bus::with_min_gap`](crate::Bus::with_min_gap).
///
/// Sized to cover one reply frame (~0.9 ms at 115200 baud) plus an
/// allowance for the motor's turnaround, so the reply a fire-and-forget
/// drive frame elicits cannot still be on the wire when the next frame
/// starts. The turnaround component is an estimate, not a measurement —
/// when tighter scheduling matters, measure it and pass the real number to
/// [`Bus::with_min_gap`](crate::Bus::with_min_gap).
pub const DEFAULT_MIN_GAP: Duration = Duration::from_micros(2500);

/// Default acceleration byte for
/// [`M0601::drive_velocity`](crate::M0601::drive_velocity) — the motor's
/// *fastest* ramp. Override the default per handle with
/// [`M0601::with_default_accel`](crate::M0601::with_default_accel), or per
/// call with
/// [`M0601::drive_velocity_accel`](crate::M0601::drive_velocity_accel).
pub const DEFAULT_DRIVE_ACCEL: u8 = 1;

/// Tunable bus-wide timing and stop behavior for one physical bus.
///
/// Every field defaults to the value the crate has always used
/// ([`BusTiming::default`]), so a bus left unconfigured behaves exactly as
/// before. Override what you need — wholesale with
/// [`Bus::with_timing`](crate::Bus::with_timing), or one field at a time with
/// the matching builder ([`Bus::with_stop_accel`](crate::Bus::with_stop_accel),
/// [`Bus::with_min_gap`](crate::Bus::with_min_gap)). Like the idle gap, this
/// lives on the **shared** bus, not per handle: set it once at open time and
/// every motor minted from the bus sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusTiming {
    /// Minimum idle time enforced between consecutive frames on the wire, so
    /// no frame overlaps the reply the previous one elicited.
    pub min_gap: Duration,
    /// Acceleration byte for the velocity-0 rounds of a controlled stop.
    /// **Not** the fastest ramp (`1`): a hard ramp-to-zero on a loaded wheel
    /// can trip the motor's 3 A protection mid-stop and defeat the stop.
    pub stop_accel: u8,
    /// Gap between the rounds of a [`M0601::safe_stop`](crate::M0601::safe_stop)
    /// sequence.
    pub stop_gap: Duration,
    /// Gap between the five repetitions of a mode-switch frame.
    pub mode_repeat_gap: Duration,
    /// Gap between the five repetitions of a set-ID frame.
    pub set_id_repeat_gap: Duration,
    /// Settling time after the set-ID sequence before re-querying.
    pub set_id_settle: Duration,
    /// How long to listen for answers to a broadcast ID query.
    pub broadcast_wait: Duration,
}

impl Default for BusTiming {
    fn default() -> Self {
        Self {
            min_gap: DEFAULT_MIN_GAP,
            stop_accel: SAFE_STOP_ACCEL,
            stop_gap: SAFE_STOP_GAP,
            mode_repeat_gap: MODE_REPEAT_GAP,
            set_id_repeat_gap: SET_ID_REPEAT_GAP,
            set_id_settle: SET_ID_SETTLE,
            broadcast_wait: BROADCAST_WAIT,
        }
    }
}

/// The minimum wall-clock a bus needs for one round of `n_drives`
/// fire-and-forget drive frames plus `n_polls` read exchanges, given the
/// enforced idle `min_gap` after every frame and the `reply_wait` each poll
/// blocks for.
///
/// This is the "budget the wire" arithmetic from the crate docs made
/// executable: a drive frame costs one
/// [`frame_time`](crate::protocol::frame_time) plus `min_gap`. A poll costs
/// *two* frame times plus `reply_wait` and `min_gap`: the transport sleeps
/// out its own wire time **and** the reply window
/// ([`Transport::send_recv`](crate::transport::Transport::send_recv) sleeps
/// `frame + reply_wait`), then the trailing idle gap re-budgets a full
/// `frame + min_gap` from the poll's return — the frame's wire time is
/// spaced once inside the transaction and once in the trailing gap. A
/// periodic multi-motor loop's cycle must exceed this, and stay at or under
/// [`drive_floor`](crate::protocol::drive_floor), or it cannot sustain its
/// own period. `m0601-quad` sizes its cycle against `bus_period(4, 1, …)`.
///
/// ```
/// use std::time::Duration;
/// use m0601::bus_period;
/// let gap = Duration::from_millis(2);
/// // Four drives + one poll with a 2 ms reply window ≈ 17.21 ms.
/// let p = bus_period(4, 1, gap, gap);
/// assert!((17_000..17_600).contains(&(p.as_micros() as u64)));
/// ```
pub fn bus_period(
    n_drives: u32,
    n_polls: u32,
    min_gap: Duration,
    reply_wait: Duration,
) -> Duration {
    let frame = crate::protocol::frame_time();
    (frame + min_gap) * n_drives + (frame * 2 + reply_wait + min_gap) * n_polls
}
