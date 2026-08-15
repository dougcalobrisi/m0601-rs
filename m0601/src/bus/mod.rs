//! The [`Bus`] (a shared RS485 port) and [`M0601`] (one motor on it) types.
//!
//! RS485 is multi-drop: several motors share one A/B pair, each with a
//! unique ID in `0x01..=0xFE`. [`Bus`] owns the transport; [`M0601`] handles
//! are cheap, cloneable, and internally serialize their frame exchanges
//! through the bus lock, so a two-wheel robot is:
//!
//! ```
//! use std::time::Duration;
//! use m0601::{Bus, MockTransport};
//!
//! # fn main() -> m0601::Result<()> {
//! # let transport = MockTransport::default();
//! let bus = Bus::with_transport(transport, Duration::from_millis(150));
//! let mut left = bus.motor(0x01)?.mirrored(true); // FIT1042 (left)
//! let mut right = bus.motor(0x02)?;               // FIT1038 (right)
//! left.drive_velocity(100)?;  // both wheels move the robot "forward"
//! right.drive_velocity(100)?;
//! # Ok(())
//! # }
//! ```
//!
//! The module is split by concern: `timing` holds the tunable [`BusTiming`]
//! and the [`bus_period`] budget, `pacing` the shared port and the
//! idle-gap/round primitives, and `motor` the per-motor [`M0601`] handle.
//! This file keeps the [`Bus`] itself — the port owner and the bus-wide
//! operations (scan, set-ID, group stop/mode).

mod motor;
mod pacing;
mod timing;

pub use motor::{M0601, PositionMirror};
pub use timing::{BusTiming, DEFAULT_DRIVE_ACCEL, DEFAULT_MIN_GAP, bus_period};

use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use crate::error::Result;
use crate::protocol::{self, frame_feedback, frame_id_query, frame_set_id, frames};
use crate::transport::{SerialTransport, Transport};
use crate::types::Mode;

use pacing::{Port, lock, mode_all, peek_min_gap, stop_all, with_gap};

/// What a [`Bus::scan`] found.
#[derive(Debug, Clone, PartialEq, Eq)]
// Returned by `scan`, never built by callers — `#[non_exhaustive]` leaves room
// for a future field (e.g. per-ID timing) without a breaking change.
#[non_exhaustive]
pub struct ScanReport {
    /// Motor IDs that answered, ascending.
    pub ids: Vec<u8>,
    /// The broadcast stage read bytes it could not attribute to any motor —
    /// a misaligned buffer, or an aligned frame whose ID byte is out of
    /// range.
    ///
    /// On a multi-drop bus this is the signature of several motors
    /// answering the unarbitrated broadcast at once: their replies collide
    /// into bytes belonging to neither. (A partly-captured TX echo looks
    /// the same; either way the bus is *not* silent.) So `ids` being empty
    /// alongside `garbled: true` does **not** mean no motors are present —
    /// it is a strong hint to re-scan polling every ID, which probes each
    /// ID in isolation. (A full scan can still collide where two motors
    /// share one ID — the aftermath of a misused
    /// [`set_id`](Bus::set_id) — since both answer the same probe.)
    pub garbled: bool,
}

/// A shared RS485 bus: owns the transport that one or more [`M0601`] motors
/// talk through.
///
/// Bus-wide operations live here — [`scan`](Self::scan) (the broadcast ID
/// query is unaddressed) and [`set_id`](Self::set_id) (the set-ID frame is
/// unaddressed too, which is exactly why it demands a single motor on the
/// bus). Per-motor operations live on the [`M0601`] handles minted by
/// [`motor`](Self::motor).
pub struct Bus<T: Transport = SerialTransport> {
    port: Arc<Mutex<Port<T>>>,
    timeout: Duration,
    /// Propagated to every [`M0601`] minted from this bus — see
    /// [`with_strict_crc`](Self::with_strict_crc).
    strict_crc: bool,
    /// Default velocity-drive accel propagated to every [`M0601`] minted from
    /// this bus — see [`with_default_accel`](Self::with_default_accel).
    default_accel: u8,
}

// Manual impl: buses are cheap to clone regardless of whether T is Clone —
// a clone is another handle on the same physical port (mirrors M0601).
// Needed by callers that keep one Bus in a control loop and another in a
// stop guard or signal handler.
impl<T: Transport> Clone for Bus<T> {
    fn clone(&self) -> Self {
        Self {
            port: Arc::clone(&self.port),
            timeout: self.timeout,
            strict_crc: self.strict_crc,
            default_accel: self.default_accel,
        }
    }
}

// Manual impl: T need not be Debug, and the port mutex must not be *blocked
// on* just to format — `peek_min_gap` degrades to `None` rather than
// deadlocking if Debug runs from a path already holding the lock.
impl<T: Transport> std::fmt::Debug for Bus<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Bus")
            .field("timeout", &self.timeout)
            .field("strict_crc", &self.strict_crc)
            .field("default_accel", &self.default_accel)
            .field("min_gap", &peek_min_gap(&self.port))
            .finish_non_exhaustive()
    }
}

impl Bus<SerialTransport> {
    /// Open `port` (e.g. `/dev/ttyUSB0`) at 115200 8N1.
    ///
    /// `timeout` becomes the default reply wait for motors minted from this
    /// bus, and the backstop timeout on OS reads.
    pub fn open(port: &str, timeout: Duration) -> Result<Self> {
        Ok(Self::with_transport(
            SerialTransport::open(port, timeout)?,
            timeout,
        ))
    }
}

impl<T: Transport> Bus<T> {
    /// Build a bus over any [`Transport`] (mocks included).
    pub fn with_transport(transport: T, timeout: Duration) -> Self {
        Self {
            port: Arc::new(Mutex::new(Port {
                transport,
                last_tx: None,
                timing: BusTiming::default(),
            })),
            timeout,
            strict_crc: false,
            default_accel: DEFAULT_DRIVE_ACCEL,
        }
    }

    /// Set the minimum idle time enforced between consecutive frames, so no
    /// two frames can overlap on the half-duplex bus. It applies to every
    /// send from every handle minted from this bus, so back-to-back calls
    /// and multi-threaded callers are both safe.
    ///
    /// The gap lives on the shared port, not on this handle: despite the
    /// by-value builder shape, calling it on one clone changes the gap for
    /// *every* clone and every motor handle on the same bus. There is one
    /// gap per physical bus, not one per handle — set it once at open time.
    ///
    /// This is a **floor, not an exact spacing**: USB adapters and OS
    /// scheduling can stretch any individual gap, and the only guarantee is
    /// that the bus stays idle for *at least* this long between frames.
    /// Defaults to [`DEFAULT_MIN_GAP`]; `Duration::ZERO` opts out entirely,
    /// for callers running their own scheduler. Set it from a turnaround
    /// you measured, not from a guess.
    ///
    /// # Why this exists
    ///
    /// Every drive (`0x64`) frame elicits a reply even when nothing reads
    /// it. Two drive frames sent back-to-back therefore put the second on
    /// the wire while the first frame's reply is still transmitting; both
    /// corrupt, and in a periodic loop the *same* frame corrupts every
    /// cycle — one motor simply never moves. Comparable protocols mandate
    /// exactly this idle floor (Modbus RTU's 3.5-character silence,
    /// CANopen's PDO inhibit time); the M0601 leaves it to the host, so
    /// the bus enforces it here.
    #[must_use]
    pub fn with_min_gap(self, gap: Duration) -> Self {
        lock(&self.port).timing.min_gap = gap;
        self
    }

    /// The enforced minimum idle gap between frames
    /// ([`with_min_gap`](Self::with_min_gap)).
    pub fn min_gap(&self) -> Duration {
        lock(&self.port).timing.min_gap
    }

    /// Replace the whole [`BusTiming`] for this bus (idle gap, stop ramp,
    /// mode/set-ID/broadcast waits) in one call — the from-config entry point.
    ///
    /// Like [`with_min_gap`](Self::with_min_gap), the timing lives on the
    /// shared port: it applies to every clone and every motor handle on the
    /// same bus, so set it once at open time. Individual builders
    /// ([`with_stop_accel`](Self::with_stop_accel),
    /// [`with_min_gap`](Self::with_min_gap)) tweak one field of it.
    #[must_use]
    pub fn with_timing(self, timing: BusTiming) -> Self {
        lock(&self.port).timing = timing;
        self
    }

    /// The acceleration byte used for the velocity-0 rounds of a controlled
    /// stop ([`M0601::safe_stop`] / [`safe_stop_all`](Self::safe_stop_all)).
    ///
    /// Shared-port builder, exactly like [`with_min_gap`](Self::with_min_gap).
    /// Defaults to a moderate ramp (see [`BusTiming`]); a hard ramp (`1`) on a
    /// loaded wheel can trip the motor's overcurrent protection mid-stop.
    #[must_use]
    pub fn with_stop_accel(self, accel: u8) -> Self {
        lock(&self.port).timing.stop_accel = accel;
        self
    }

    /// The current [`BusTiming`] for this bus.
    pub fn timing(&self) -> BusTiming {
        lock(&self.port).timing
    }

    /// Opt into **strict CRC** for every motor minted from this bus: a
    /// telemetry frame whose byte 9 fails the CRC-8/MAXIM check is dropped to
    /// `Ok(None)` instead of being returned with
    /// [`Feedback::crc_ok`](crate::Feedback::crc_ok)` == false`.
    ///
    /// The default is **advisory** (`false`), matching the rest of the crate:
    /// telemetry is returned regardless and the CRC verdict is left in
    /// `crc_ok` for the caller to weigh, because genuine replies from some
    /// firmware revisions have been seen to disagree on the checksum (see
    /// `PROTOCOL.md`). Turn this on where a corrupt frame is worse than a
    /// dropped one — above all before an odometry integrator, which a single
    /// bad position sample can throw off for good.
    ///
    /// Unlike [`with_min_gap`](Self::with_min_gap), this flag lives on the
    /// handle, not the shared port: it is copied into each [`M0601`] at
    /// [`motor`](Self::motor) time (exactly as [`timeout`](Self::timeout) is),
    /// so set it before minting the motors you want it to cover. A motor's own
    /// [`M0601::with_strict_crc`] can still override it per handle.
    #[must_use]
    pub fn with_strict_crc(mut self, strict: bool) -> Self {
        self.strict_crc = strict;
        self
    }

    /// Whether motors minted from this bus reject CRC-failing frames
    /// ([`with_strict_crc`](Self::with_strict_crc)).
    pub fn strict_crc(&self) -> bool {
        self.strict_crc
    }

    /// The default acceleration byte for [`M0601::drive_velocity`] on every
    /// motor minted from this bus.
    ///
    /// Like [`with_strict_crc`](Self::with_strict_crc) this is a per-handle
    /// setting, copied into each [`M0601`] at [`motor`](Self::motor) time; a
    /// motor's own [`M0601::with_default_accel`] overrides it. Defaults to
    /// [`DEFAULT_DRIVE_ACCEL`]. The per-call [`M0601::drive_velocity_accel`]
    /// always wins over both.
    #[must_use]
    pub fn with_default_accel(mut self, accel: u8) -> Self {
        self.default_accel = accel;
        self
    }

    /// The default velocity-drive accel for motors minted from this bus
    /// ([`with_default_accel`](Self::with_default_accel)).
    pub fn default_accel(&self) -> u8 {
        self.default_accel
    }

    /// A driver handle for the motor at `id` (validated: `0x01..=0xFE`).
    ///
    /// Handles keep the underlying transport alive; the `Bus` itself may be
    /// dropped once all motors are minted.
    pub fn motor(&self, id: u8) -> Result<M0601<T>> {
        protocol::validate_id(id)?;
        Ok(M0601 {
            port: Arc::clone(&self.port),
            id,
            timeout: self.timeout,
            mirrored: false,
            position_mirror: PositionMirror::default(),
            strict_crc: self.strict_crc,
            default_accel: self.default_accel,
        })
    }

    /// The default reply-wait / backstop timeout for this bus.
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Send an arbitrary frame and return the raw reply bytes (may be
    /// empty). Powers the CLI's `raw` subcommand.
    pub fn send_raw(&self, frame: &[u8], wait: Duration) -> Result<Vec<u8>> {
        with_gap(&self.port, |t| t.send_recv(frame, wait))
    }

    /// Discover motor IDs on the bus.
    ///
    /// Stage 1 broadcasts the fixed ID-query frame and listens for 300 ms.
    /// Stage 2 probes each ID in `poll` with a feedback frame, at a cost of
    /// one `timeout` apiece — pass `0x01..=0xFE` for an exhaustive scan
    /// (~254 × timeout), a narrower range to trade coverage for time, or
    /// `std::iter::empty()` for broadcast-only. IDs outside `0x01..=0xFE`
    /// are skipped. `progress` is called with each ID before it is probed.
    ///
    /// Half-duplex TX echoes are stripped before interpreting replies.
    ///
    /// # Stage 1 is best-effort, and absence of proof is not proof of
    /// absence
    ///
    /// Motors answer a broadcast without arbitration, so two that reply at
    /// once collide into bytes belonging to neither. **A stage-1 result of
    /// one ID is therefore not evidence that only one motor is present** —
    /// poll every ID when that distinction matters (as
    /// [`set_id`](Self::set_id) requires it to).
    ///
    /// When the collision garbles the buffer beyond parsing, stage 1 cannot
    /// name any ID at all — a four-motor bus can scan as *empty*. That case
    /// is reported via [`ScanReport::garbled`] so callers can escalate to a
    /// wider poll instead of concluding the bus is dead.
    ///
    /// # Blocking
    ///
    /// Every probe holds the bus lock across its reply wait, so a scan
    /// monopolises the bus for roughly one `timeout` per polled ID. Do not
    /// run a long one concurrently with a drive loop on the same bus: the
    /// loop's frames will not get out, and its motor will coast.
    pub fn scan(
        &self,
        poll: impl IntoIterator<Item = u8>,
        mut progress: impl FnMut(u8),
    ) -> Result<ScanReport> {
        let mut found = std::collections::BTreeSet::new();

        // Stage 1 — broadcast. Replies land back-to-back when several motors
        // answer without colliding, so walk whole frames and take each one's
        // ID byte. (Scanning for any in-range byte instead would happily
        // report a leftover echo or payload byte as a motor.)
        //
        // `frames` also rejects a misaligned buffer outright. It has to: a
        // partial echo shifts every chunk boundary, and chunk 0 then begins
        // with the *query's own* destination byte 0xC8 — which is in the
        // valid ID range. The scan would report a motor at 0xC8 that does
        // not exist and miss the one that does.
        //
        // Either rejection leaves bytes unaccounted for; `garbled` records
        // that, because "the bus answered with garbage" (motors colliding)
        // and "the bus is silent" (no motors) demand opposite responses
        // from the caller.
        let broadcast_wait = self.timing().broadcast_wait;
        let query = frame_id_query();
        let resp = with_gap(&self.port, |t| t.send_recv(&query, broadcast_wait))?;
        let payload = protocol::strip_echo(&query, &resp);
        let mut garbled = false;
        match frames(&query, &resp) {
            Some(chunks) => {
                for chunk in chunks {
                    if (0x01..=0xFE).contains(&chunk[0]) {
                        found.insert(chunk[0]);
                    } else {
                        garbled = true;
                    }
                }
            }
            None => garbled = !payload.is_empty(),
        }

        // Stage 2 — per-ID poll.
        for id in poll {
            if !(0x01..=0xFE).contains(&id) {
                continue;
            }
            progress(id);
            let probe = frame_feedback(id);
            let resp = with_gap(&self.port, |t| t.send_recv(&probe, self.timeout))?;
            // A valid reply is a whole frame carrying the probed ID. The
            // alignment check matters here too: the probe's own first
            // byte *is* `id`, so a partial echo would answer every probe
            // and report a motor at all 254 addresses.
            if frames(&probe, &resp)
                .into_iter()
                .flatten()
                .any(|frame| frame[0] == id)
            {
                found.insert(id);
            }
        }

        Ok(ScanReport {
            ids: found.into_iter().collect(),
            garbled,
        })
    }

    /// Change the persistent RS485 ID of the (single!) motor on the bus.
    ///
    /// Sends the frame five times, waits 500 ms, then re-queries via
    /// broadcast; returns the ID the bus now reports (`None` if nothing
    /// answered — power-cycle and re-scan to confirm).
    ///
    /// # Only one motor may be connected
    ///
    /// The set-ID frame is unaddressed: **every** motor that hears it takes
    /// the new ID, leaving a bus full of duplicates that can only be undone
    /// by disconnecting them one at a time. This method cannot detect that
    /// situation for you — verify with
    /// [`scan(0x01..=0xFE, …)`](Self::scan) beforehand, since a stage-1 scan
    /// cannot distinguish one motor from several answering at once.
    pub fn set_id(&self, new_id: u8) -> Result<Option<u8>> {
        let timing = self.timing();
        let frame = frame_set_id(new_id)?;
        for _ in 0..5 {
            self.send_paced(&frame, timing.set_id_repeat_gap)?;
        }
        self.pause(timing.set_id_settle);

        let query = frame_id_query();
        let resp = with_gap(&self.port, |t| t.send_recv(&query, timing.broadcast_wait))?;
        Ok(frames(&query, &resp)
            .into_iter()
            .flatten()
            .map(|frame| frame[0])
            .find(|b| (0x01..=0xFE).contains(b)))
    }

    /// Best-effort interleaved controlled stop for every motor in `ids`.
    ///
    /// Same sequence and guarantees as [`M0601::safe_stop`] — establish
    /// velocity mode, five velocity-0 frames, five brake frames — but
    /// round-major: each step's frame goes to every motor before the shared
    /// 20 ms gap, so N motors stop in the same ~300 ms as one. Stopping
    /// motors one at a time instead takes N × 300 ms, during which the
    /// not-yet-stopped wheels coast — on a skid-steer chassis, one braked
    /// wheel against three coasting ones is an uncommanded yaw. Rounds are
    /// paced on absolute deadlines, so the period does not stretch with
    /// motor count — as long as the round's frames fit inside it. With enough
    /// motors that `ids.len()` frames (each
    /// [`frame_time`](crate::protocol::frame_time) `+ min_gap`) exceed the
    /// 20 ms round, a round runs long and the next simply starts late (the
    /// stop still completes; it just takes more than ~300 ms).
    ///
    /// This is a shutdown path — it runs from quit handlers, panic unwinds
    /// and signal handlers, so it must not fail: it returns `()`, swallows
    /// I/O errors, attempts every frame regardless, and never panics.
    /// An empty `ids` is a no-op. IDs are used as given (no validation —
    /// refusing to stop is worse than addressing a motor that isn't there).
    pub fn safe_stop_all(&self, ids: &[u8]) {
        stop_all(&self.port, ids);
    }

    /// Switch every motor in `ids` into `mode`, round-major: each of the
    /// protocol's five `0xA0` repetitions goes to every motor before the
    /// shared 20 ms gap, ~100 ms total regardless of motor count. The
    /// motors send no acknowledgement.
    ///
    /// Mode frames elicit no reply, which is exactly what makes batching
    /// them this tightly safe; drive frames are all answered, so their
    /// spacing floor is [`with_min_gap`](Self::with_min_gap) instead.
    ///
    /// Every ID is validated before any frame is sent, so a bad ID fails the
    /// whole call untouched. An *I/O* error partway through is different: the
    /// round it occurs in is completed (every motor still gets that frame),
    /// but the remaining rounds are skipped and the error is returned — so on
    /// a write failure some motors may have received fewer than the five
    /// repetitions the protocol wants. The switch is not atomic across the
    /// wire; treat an `Err` as "the group mode may be inconsistent" and
    /// recover (re-issue, or [`safe_stop_all`](Self::safe_stop_all)).
    ///
    /// As with [`M0601::set_mode`], switching into [`Mode::Position`] requires
    /// the wheel to be turning slower than 10 RPM.
    pub fn set_mode_all(&self, ids: &[u8], mode: Mode) -> Result<()> {
        for &id in ids {
            protocol::validate_id(id)?;
        }
        mode_all(&self.port, ids, mode)
    }

    /// Recover the transport, if this `Bus` and all its motors are dropped
    /// (useful for inspecting a mock in tests).
    pub fn into_transport(self) -> Option<T> {
        Arc::into_inner(self.port).map(|m| {
            m.into_inner()
                .unwrap_or_else(PoisonError::into_inner)
                .transport
        })
    }

    /// Send under the lock, then observe the gap *outside* it.
    fn send_paced(&self, frame: &[u8], gap: Duration) -> Result<()> {
        with_gap(&self.port, |t| t.send(frame))?;
        self.pause(gap);
        Ok(())
    }

    /// Sleep for the transport's paced version of `d`, without the lock.
    fn pause(&self, d: Duration) {
        let d = lock(&self.port).transport.pace(d);
        if !d.is_zero() {
            std::thread::sleep(d);
        }
    }
}
