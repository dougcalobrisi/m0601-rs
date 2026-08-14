//! The pacing / round-scheduling engine.
//!
//! [`Port`] is the shared state behind one physical bus (the transport plus
//! the idle-gap bookkeeping); the free functions here are the primitives that
//! keep frames from overlapping on the half-duplex wire — [`with_gap`] paces a
//! single send, [`send_round`] interleaves one step across several motors, and
//! [`stop_all`]/[`mode_all`] build the grouped stop and mode-switch sequences
//! on top of them. All of this is independent of the public [`Bus`](super::Bus)
//! and [`M0601`](super::M0601) types, which merely drive it.

use std::sync::{Mutex, MutexGuard, PoisonError, TryLockError};
use std::time::{Duration, Instant};

use crate::error::Result;
use crate::protocol::{Frame, frame_brake, frame_mode, frame_velocity};
use crate::transport::Transport;
use crate::types::Mode;

use super::timing::BusTiming;

/// Poison-tolerant lock. The guarded transport holds no invariants a panic
/// could corrupt mid-update (each call is a complete frame exchange), and
/// motor I/O — above all the stop paths — must keep working even if another
/// thread panicked.
pub(super) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The shared state behind one physical bus: the transport plus the pacing
/// bookkeeping that keeps frames from overlapping on the half-duplex wire.
/// One lock guards both, so the idle-gap accounting cannot race the I/O it
/// meters — the gap holds across every handle, on every thread.
pub(super) struct Port<T> {
    pub(super) transport: T,
    /// When the last transmitting operation completed.
    pub(super) last_tx: Option<Instant>,
    /// Tunable timing shared across every handle on this bus (idle gap, stop
    /// ramp, mode/set-ID/broadcast waits).
    pub(super) timing: BusTiming,
}

impl<T: Transport> Port<T> {
    /// How much longer the bus must stay idle before the next TX may start.
    /// Routed through [`Transport::pace`] so mocks never wait.
    ///
    /// The budget is `frame_time() + min_gap`, not `min_gap` alone: a
    /// fire-and-forget send returns as soon as the frame is buffered (no
    /// `tcdrain` — see [`Transport::send`]), so `last_tx` marks the *start*
    /// of the frame on the wire and the frame's own wire time must be
    /// spaced out here. For an op that already outlived its TX (a poll
    /// slept out its reply window) the extra `frame_time()` is ~0.9 ms of
    /// deliberate over-spacing — never a collision risk, and
    /// [`bus_period`](super::bus_period) budgets for it so a loop sized
    /// against it does not overrun.
    fn gap_remaining(&self) -> Duration {
        match self.last_tx {
            Some(at) => self.transport.pace(
                (crate::protocol::frame_time() + self.timing.min_gap).saturating_sub(at.elapsed()),
            ),
            None => Duration::ZERO,
        }
    }
}

/// Run one transmitting operation against the bus, first waiting out its
/// minimum idle gap ([`Bus::with_min_gap`](super::Bus::with_min_gap)).
///
/// The wait happens *outside* the lock and the gap is then re-checked, so a
/// competing handle that transmitted during the sleep pushes this frame
/// further back rather than overlapping it. `last_tx` is stamped when the
/// operation returns: for a fire-and-forget send that is the moment the
/// frame started onto the wire (writes are not drained), which is why
/// [`Port::gap_remaining`] budgets the frame's wire time on top of the
/// idle gap that keeps the unread reply it elicits clear of the next
/// frame.
pub(super) fn with_gap<T: Transport, R>(
    port: &Mutex<Port<T>>,
    mut op: impl FnMut(&mut T) -> Result<R>,
) -> Result<R> {
    loop {
        let mut guard = lock(port);
        let wait = guard.gap_remaining();
        if wait.is_zero() {
            let result = op(&mut guard.transport);
            guard.last_tx = Some(Instant::now());
            return result;
        }
        drop(guard);
        std::thread::sleep(wait);
    }
}

/// One interleaved round of a grouped sequence: the same step's frame goes
/// to every motor in `ids`, then the rest of the round is slept out against
/// an absolute `deadline` — so the round period is what the caller chose,
/// independent of motor count (for as long as the frames fit inside it).
///
/// Every frame is attempted even after an error — this runs on shutdown
/// paths where "keep telling the other motors to stop" beats propagating
/// the first failure. The first error is returned for callers that do want
/// it.
fn send_round<T: Transport>(
    port: &Mutex<Port<T>>,
    ids: &[u8],
    frame_for: impl Fn(u8) -> Frame,
    deadline: Instant,
) -> Result<()> {
    let mut result = Ok(());
    for &id in ids {
        let frame = frame_for(id);
        if let Err(e) = with_gap(port, |t| t.send(&frame))
            && result.is_ok()
        {
            result = Err(e);
        }
    }
    let remaining = deadline.saturating_duration_since(Instant::now());
    let sleep = lock(port).transport.pace(remaining);
    if !sleep.is_zero() {
        std::thread::sleep(sleep);
    }
    result
}

/// The one safe-stop implementation: [`M0601::safe_stop`](super::M0601::safe_stop)
/// calls it with a single ID, [`Bus::safe_stop_all`](super::Bus::safe_stop_all)
/// with the whole vehicle. Round-major — five velocity-mode rounds, five
/// velocity-0 rounds, five brake rounds, 20 ms apart — so every motor is told
/// to stop in step and N motors take the same ~300 ms as one. Best-effort:
/// errors are swallowed, every frame is attempted.
pub(super) fn stop_all<T: Transport>(port: &Mutex<Port<T>>, ids: &[u8]) {
    if ids.is_empty() {
        return;
    }
    // Snapshot the configurable stop ramp/gap once (BusTiming is Copy).
    let BusTiming {
        stop_accel,
        stop_gap,
        ..
    } = lock(port).timing;
    let mut deadline = Instant::now();
    for step in 0..15u8 {
        deadline += stop_gap;
        let _ = send_round(
            port,
            ids,
            |id| match step {
                0..=4 => frame_mode(id, Mode::Velocity),
                5..=9 => frame_velocity(id, 0, stop_accel),
                _ => frame_brake(id),
            },
            deadline,
        );
    }
}

/// The one mode-switch implementation: five rounds of `0xA0` frames, 20 ms
/// apart, as the protocol requires. [`M0601::set_mode`](super::M0601::set_mode)
/// calls it with a single ID, [`Bus::set_mode_all`](super::Bus::set_mode_all)
/// with several.
pub(super) fn mode_all<T: Transport>(port: &Mutex<Port<T>>, ids: &[u8], mode: Mode) -> Result<()> {
    if ids.is_empty() {
        return Ok(());
    }
    let mode_repeat_gap = lock(port).timing.mode_repeat_gap;
    let mut deadline = Instant::now();
    for _ in 0..5 {
        deadline += mode_repeat_gap;
        send_round(port, ids, |id| frame_mode(id, mode), deadline)?;
    }
    Ok(())
}

/// Read the shared `min_gap` for a `Debug` impl without ever blocking. Debug
/// can run from a panic path that already holds the port lock (a stop guard
/// unwinding mid-send), so this uses `try_lock` and reports `None` on
/// contention rather than deadlocking the formatter. A poisoned lock still
/// yields the value — the crate is poison-tolerant everywhere else too.
pub(super) fn peek_min_gap<T: Transport>(port: &Mutex<Port<T>>) -> Option<Duration> {
    match port.try_lock() {
        Ok(p) => Some(p.timing.min_gap),
        Err(TryLockError::Poisoned(p)) => Some(p.into_inner().timing.min_gap),
        Err(TryLockError::WouldBlock) => None,
    }
}
