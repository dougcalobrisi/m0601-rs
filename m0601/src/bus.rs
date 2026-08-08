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

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use crate::error::Result;
use crate::protocol::{
    self, Frame, ReplyKind, frame_brake, frame_current, frame_feedback, frame_id_query, frame_mode,
    frame_position, frame_set_id, frame_velocity, parse_feedback,
};
use crate::transport::{SerialTransport, Transport};
use crate::types::{Feedback, Mode};

/// How long to listen for answers to a broadcast ID query.
const BROADCAST_WAIT: Duration = Duration::from_millis(300);
/// Gap between the five repetitions of a mode-switch frame.
const MODE_REPEAT_GAP: Duration = Duration::from_millis(20);
/// Gap between the five repetitions of a set-ID frame.
const SET_ID_REPEAT_GAP: Duration = Duration::from_millis(50);
/// Settling time after the set-ID sequence before re-querying.
const SET_ID_SETTLE: Duration = Duration::from_millis(500);
/// Gap between the frames of a [`M0601::safe_stop`] sequence (50 Hz).
const SAFE_STOP_GAP: Duration = Duration::from_millis(20);

/// Poison-tolerant lock. The guarded transport holds no invariants a panic
/// could corrupt mid-update (each call is a complete frame exchange), and
/// motor I/O — above all the stop paths — must keep working even if another
/// thread panicked.
fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Strip a leading half-duplex TX echo and split what remains into whole
/// frames. Returns `None` unless that is a non-empty exact multiple of
/// [`FRAME_LEN`](protocol::FRAME_LEN).
///
/// # Why the length must divide evenly
///
/// `strip_prefix` is all-or-nothing: if the echo is short by even one byte
/// it is not recognised, and offset 0 is then no longer a frame boundary.
/// Parsing from there anyway yields a frame *straddling* the tail of the
/// echo and the head of the real reply — and that garbage is not obviously
/// garbage. It looks like telemetry, it passes the per-motor ID check
/// (a truncated echo begins with the addressed motor's own ID, exactly as a
/// genuine reply does), and it decodes to plausible values. Measured across
/// every cut point, a wheel turning at 300 RPM read back as 0, 1, 258 or
/// 512 RPM — and for seven of the nine cuts that is under the `< 10 RPM`
/// guard callers rely on before entering position mode, which is the one
/// place a wrong speed reading is actively dangerous.
///
/// A well-formed transaction is always a whole number of frames — the reply
/// alone, or the echo plus the reply — so anything else means the stream is
/// misaligned and none of it can be trusted. Rejecting on that costs at most
/// one dropped reading, which every caller already tolerates.
fn frames<'a>(tx: &[u8], rx: &'a [u8]) -> Option<std::slice::ChunksExact<'a, u8>> {
    let rx = rx.strip_prefix(tx).unwrap_or(rx);
    if rx.is_empty() || !rx.len().is_multiple_of(protocol::FRAME_LEN) {
        return None;
    }
    Some(rx.chunks_exact(protocol::FRAME_LEN))
}

/// What a [`Bus::scan`] found.
#[derive(Debug, Clone, PartialEq, Eq)]
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
    /// it is a strong hint to re-scan with `full: true`, which probes each
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
    transport: Arc<Mutex<T>>,
    timeout: Duration,
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
            transport: Arc::new(Mutex::new(transport)),
            timeout,
        }
    }

    /// A driver handle for the motor at `id` (validated: `0x01..=0xFE`).
    ///
    /// Handles keep the underlying transport alive; the `Bus` itself may be
    /// dropped once all motors are minted.
    pub fn motor(&self, id: u8) -> Result<M0601<T>> {
        protocol::validate_id(id)?;
        Ok(M0601 {
            transport: Arc::clone(&self.transport),
            id,
            timeout: self.timeout,
            mirrored: false,
        })
    }

    /// The default reply-wait / backstop timeout for this bus.
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Send an arbitrary frame and return the raw reply bytes (may be
    /// empty). Powers the CLI's `raw` subcommand.
    pub fn send_raw(&self, frame: &[u8], wait: Duration) -> Result<Vec<u8>> {
        lock(&self.transport).send_recv(frame, wait)
    }

    /// Discover motor IDs on the bus.
    ///
    /// Stage 1 broadcasts the fixed ID-query frame and listens for 300 ms.
    /// With `full`, stage 2 probes every ID `0x01..=0xFE` with a feedback
    /// frame (slow: ~254 × timeout). `progress` is called with each ID
    /// before it is probed.
    ///
    /// Half-duplex TX echoes are stripped before interpreting replies.
    ///
    /// # Stage 1 is best-effort, and absence of proof is not proof of
    /// absence
    ///
    /// Motors answer a broadcast without arbitration, so two that reply at
    /// once collide into bytes belonging to neither. **A stage-1 result of
    /// one ID is therefore not evidence that only one motor is present** —
    /// pass `full: true` when that distinction matters (as
    /// [`set_id`](Self::set_id) requires it to).
    ///
    /// When the collision garbles the buffer beyond parsing, stage 1 cannot
    /// name any ID at all — a four-motor bus can scan as *empty*. That case
    /// is reported via [`ScanReport::garbled`] so callers can escalate to a
    /// full scan instead of concluding the bus is dead.
    ///
    /// # Blocking
    ///
    /// Every probe holds the bus lock across its reply wait, so a `full`
    /// scan monopolises the bus for ~254 × `timeout`. Do not run one
    /// concurrently with a drive loop on the same bus: the loop's frames
    /// will not get out, and its motor will coast.
    pub fn scan(&self, full: bool, mut progress: impl FnMut(u8)) -> Result<ScanReport> {
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
        let query = frame_id_query();
        let resp = lock(&self.transport).send_recv(&query, BROADCAST_WAIT)?;
        let payload = resp.strip_prefix(query.as_slice()).unwrap_or(&resp);
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

        // Stage 2 — exhaustive poll.
        if full {
            for id in 0x01..=0xFEu8 {
                progress(id);
                let probe = frame_feedback(id);
                let resp = lock(&self.transport).send_recv(&probe, self.timeout)?;
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
    /// [`scan(true, …)`](Self::scan) beforehand, since a stage-1 scan
    /// cannot distinguish one motor from several answering at once.
    pub fn set_id(&self, new_id: u8) -> Result<Option<u8>> {
        let frame = frame_set_id(new_id)?;
        for _ in 0..5 {
            self.send_paced(&frame, SET_ID_REPEAT_GAP)?;
        }
        self.pause(SET_ID_SETTLE);

        let query = frame_id_query();
        let resp = lock(&self.transport).send_recv(&query, BROADCAST_WAIT)?;
        Ok(frames(&query, &resp)
            .into_iter()
            .flatten()
            .map(|frame| frame[0])
            .find(|b| (0x01..=0xFE).contains(b)))
    }

    /// Recover the transport, if this `Bus` and all its motors are dropped
    /// (useful for inspecting a mock in tests).
    pub fn into_transport(self) -> Option<T> {
        Arc::into_inner(self.transport)
            .map(|m| m.into_inner().unwrap_or_else(PoisonError::into_inner))
    }

    /// Send under the lock, then observe the gap *outside* it.
    fn send_paced(&self, frame: &[u8], gap: Duration) -> Result<()> {
        lock(&self.transport).send(frame)?;
        self.pause(gap);
        Ok(())
    }

    /// Sleep for the transport's paced version of `d`, without the lock.
    fn pause(&self, d: Duration) {
        let d = lock(&self.transport).pace(d);
        if !d.is_zero() {
            std::thread::sleep(d);
        }
    }
}

/// Driver handle for one M0601 motor on an RS485 [`Bus`].
///
/// Minted by [`Bus::motor`] (or the [`M0601::open`] single-motor
/// convenience). Handles are `Clone` and `Send`; each frame exchange locks
/// the shared bus for exactly one transaction, so multiple threads can each
/// drive their own wheel.
///
/// # The polling contract
///
/// The drive methods ([`drive_velocity`](Self::drive_velocity),
/// [`drive_current`](Self::drive_current),
/// [`drive_position`](Self::drive_position), [`brake`](Self::brake)) each
/// send **one** frame. The motor sustains motion only while such frames keep
/// arriving at ≥[`DRIVE_HZ_MIN`](crate::protocol::DRIVE_HZ_MIN) Hz (max
/// [`CMD_HZ_MAX`](crate::protocol::CMD_HZ_MAX) Hz) — a single call will not
/// keep the wheel spinning, and if the host stops sending, the motor coasts
/// to a stop. That coast-on-silence behavior is the protocol's built-in
/// fail-safe; [`safe_stop`](Self::safe_stop) upgrades "coast" to "brake" for
/// orderly shutdowns.
///
/// # Mirrored (left/right) wheels
///
/// The FIT1042 (left) and FIT1038 (right) SKUs are mirror-image builds of
/// the same M0601 motor. Construct one side with
/// [`mirrored(true)`](Self::mirrored) and "positive = robot forward" holds
/// for both: velocity/current *setpoints* are negated on the way out, and
/// the *signs* of reported speed and current are flipped on the way in.
/// Position values pass through untouched — the right mirror transform for
/// an angle (e.g. `360° − x`) depends on your mechanical convention — and
/// [`Feedback::raw`] always keeps the unmodified wire frame.
pub struct M0601<T: Transport = SerialTransport> {
    transport: Arc<Mutex<T>>,
    id: u8,
    timeout: Duration,
    mirrored: bool,
}

// Manual impl: handles are cheap to clone regardless of whether T is Clone.
impl<T: Transport> Clone for M0601<T> {
    fn clone(&self) -> Self {
        Self {
            transport: Arc::clone(&self.transport),
            id: self.id,
            timeout: self.timeout,
            mirrored: self.mirrored,
        }
    }
}

impl M0601<SerialTransport> {
    /// Single-motor convenience: open `port` and address motor `id`.
    /// Equivalent to `Bus::open(port, timeout)?.motor(id)`.
    pub fn open(port: &str, id: u8, timeout: Duration) -> Result<Self> {
        Bus::open(port, timeout)?.motor(id)
    }
}

impl<T: Transport> M0601<T> {
    /// Build a single-motor driver over any [`Transport`] (mocks included).
    /// Equivalent to `Bus::with_transport(transport, timeout).motor(id)`.
    pub fn with_transport(transport: T, id: u8, timeout: Duration) -> Result<Self> {
        Bus::with_transport(transport, timeout).motor(id)
    }

    /// Mark this wheel as the mirrored (opposite-side) build — see the
    /// type-level docs. Builder-style: `bus.motor(0x01)?.mirrored(true)`.
    #[must_use]
    pub fn mirrored(mut self, mirrored: bool) -> Self {
        self.mirrored = mirrored;
        self
    }

    /// Whether this handle flips velocity/current signs.
    pub fn is_mirrored(&self) -> bool {
        self.mirrored
    }

    /// The motor ID this handle addresses.
    pub fn id(&self) -> u8 {
        self.id
    }

    /// The configured reply-wait / backstop timeout.
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Recover the transport, if this is the last handle on the bus
    /// (useful for inspecting a mock in tests).
    pub fn into_transport(self) -> Option<T> {
        Arc::into_inner(self.transport)
            .map(|m| m.into_inner().unwrap_or_else(PoisonError::into_inner))
    }

    /// Send an arbitrary frame and return the raw reply bytes (may be
    /// empty).
    pub fn send_raw(&mut self, frame: &[u8], wait: Duration) -> Result<Vec<u8>> {
        lock(&self.transport).send_recv(frame, wait)
    }

    /// Strip a leading half-duplex TX echo, parse, and reject frames from
    /// any motor other than this handle's.
    ///
    /// The reply *layout* is selected from the TX frame's command byte
    /// ([`ReplyKind::from_tx`]): a `0x74` query reply carries temperature +
    /// an 8-bit position, a `0x64` drive reply a 16-bit position and no
    /// temperature. Frames that elicit no telemetry (mode switch, set-ID)
    /// yield `None`.
    ///
    /// Some RS485 adapters loop their own transmission back, so a reply can
    /// arrive as `<tx frame><telemetry>` — or as a bare `<tx frame>` when no
    /// motor answers. A genuine reply can never byte-equal the TX frame (its
    /// byte 1 is a mode value, not the command), so an exact TX prefix is
    /// always an echo and is stripped unconditionally. A *partial* echo
    /// cannot be stripped, and is rejected instead — see [`frames`] for why
    /// that case is more dangerous than it looks.
    ///
    /// The ID check matters on the multi-drop bus this crate exists to
    /// support: without it, a stale frame still in the adapter's buffer, or
    /// one motor's late answer landing inside another's transaction window,
    /// would be handed back as *this* motor's telemetry — a wheel reporting
    /// its neighbour's speed. When several whole frames arrive together, the
    /// one addressed to this handle is picked out rather than the buffer
    /// being written off because a neighbour happened to answer first.
    fn parse_reply(&self, tx: &[u8], rx: &[u8]) -> Option<Feedback> {
        let kind = ReplyKind::from_tx(tx)?;
        let fb = frames(tx, rx)?
            .find_map(|frame| parse_feedback(frame, kind).filter(|fb| fb.id == self.id))?;
        Some(self.adjust(fb))
    }

    /// Apply the mirror convention to incoming telemetry.
    fn adjust(&self, mut fb: Feedback) -> Feedback {
        if self.mirrored {
            fb.speed_rpm = fb.speed_rpm.saturating_neg();
            // `0.0 - x` rather than `-x` so a zero current mirrors to +0.0,
            // not the -0.0 that would print as "-0.000 A".
            fb.current_a = 0.0 - fb.current_a;
        }
        fb
    }

    /// Send `frame` and parse the reply as telemetry, decoding it per the
    /// layout the sent command elicits ([`ReplyKind::from_tx`]) — so a
    /// drive frame's reply yields a hi-res 16-bit position and
    /// `temp_c: None`, while a `0x74` query's reply yields a temperature
    /// and an 8-bit position.
    ///
    /// `Ok(None)` means the bus stayed silent, the reply was too short, the
    /// reply came from a different motor ID, or `frame` is one that elicits
    /// no telemetry at all (mode switch, set-ID — the frame is still sent) —
    /// all expected outcomes, not errors. Used by the CLI's 50 Hz control
    /// loop with a short `wait` (~6 ms). Mirrored handles flip the signs of
    /// the parsed speed/current (the raw frame is untouched).
    pub fn transact(&mut self, frame: &Frame, wait: Duration) -> Result<Option<Feedback>> {
        let rx = lock(&self.transport).send_recv(frame, wait)?;
        Ok(self.parse_reply(frame, &rx))
    }

    /// Query telemetry with a feedback (`0x74`) frame, waiting the
    /// configured timeout for the reply.
    ///
    /// This is the only exchange whose reply carries the winding
    /// temperature (`temp_c` is `Some`); its position reading is the
    /// coarse 8-bit one (~1.4°). Drive replies via
    /// [`transact`](Self::transact) have it the other way around.
    pub fn query(&mut self) -> Result<Option<Feedback>> {
        let frame = frame_feedback(self.id);
        self.transact(&frame, self.timeout)
    }

    /// Switch control mode, sending the `0xA0` frame five times as the
    /// protocol requires. The motor sends no acknowledgement.
    ///
    /// Before switching to [`Mode::Position`], the motor must be turning
    /// slower than 10 RPM.
    pub fn set_mode(&mut self, mode: Mode) -> Result<()> {
        let frame = frame_mode(self.id, mode);
        for _ in 0..5 {
            self.send_paced(&frame, MODE_REPEAT_GAP)?;
        }
        Ok(())
    }

    /// Send one velocity drive frame (clamped to ±330 RPM, accel 1); a
    /// mirrored handle negates `rpm` first.
    /// Must be resent at ≥50 Hz to sustain motion — see the type-level docs.
    ///
    /// Use [`drive_velocity_accel`](Self::drive_velocity_accel) to soften
    /// the default accel of 1, which is the motor's *fastest* ramp.
    pub fn drive_velocity(&mut self, rpm: i16) -> Result<()> {
        self.drive_velocity_accel(rpm, 1)
    }

    /// Send one velocity drive frame with an explicit acceleration.
    ///
    /// `0` selects the motor's default; `1` — what
    /// [`drive_velocity`](Self::drive_velocity) uses — is the *fastest*
    /// ramp, not a gentle one, and larger values ramp more gently. A large
    /// step at `accel: 1` draws a current spike that can trip the motor's
    /// 3 A bus-overcurrent protection on a loaded wheel; raise the value to
    /// soften it.
    ///
    /// The vendor sources also give this byte a unit ("1 RPM per 0.1 ms")
    /// that reads as a rate, which contradicts the ramp direction above.
    /// That contradiction is unresolved, so only the direction is documented
    /// — see [`frame_velocity`].
    pub fn drive_velocity_accel(&mut self, rpm: i16, accel: u8) -> Result<()> {
        let rpm = if self.mirrored {
            rpm.saturating_neg()
        } else {
            rpm
        };
        lock(&self.transport).send(&frame_velocity(self.id, rpm, accel))
    }

    /// Send one current drive frame (`i16` ≈ −8 A..+8 A); a mirrored handle
    /// negates `value` first.
    /// Must be resent at ≥50 Hz to sustain motion — see the type-level docs.
    pub fn drive_current(&mut self, value: i16) -> Result<()> {
        let value = if self.mirrored {
            value.saturating_neg()
        } else {
            value
        };
        lock(&self.transport).send(&frame_current(self.id, value))
    }

    /// Send one position drive frame (clamped to `0..=32767` = 0°..360°).
    /// NOT mirror-adjusted — see the type-level docs.
    /// Must be resent at ≥50 Hz to hold — see the type-level docs.
    pub fn drive_position(&mut self, raw: u16) -> Result<()> {
        lock(&self.transport).send(&frame_position(self.id, raw))
    }

    /// Send one electric-brake frame (velocity mode only).
    /// Must be resent at ≥50 Hz to keep braking — see the type-level docs.
    pub fn brake(&mut self) -> Result<()> {
        lock(&self.transport).send(&frame_brake(self.id))
    }

    /// Best-effort controlled stop: force velocity mode, then five
    /// velocity-0 frames, then five brake frames, 20 ms apart (~300 ms
    /// total).
    ///
    /// # Why it switches mode first
    ///
    /// A drive (`0x64`) frame's 16-bit value is interpreted per the motor's
    /// *active* mode, so a zero-valued frame only means "stop" in velocity
    /// mode. In [`Mode::Position`] the very same bytes mean **"rotate to
    /// 0°"** — a stop sequence that could spin the wheel up to a full
    /// half-turn — and in [`Mode::Current`] they mean zero torque (a coast)
    /// while the brake byte is ignored entirely. Since callers reach this
    /// from panic and signal paths where the active mode is not knowable,
    /// it establishes velocity mode itself rather than assuming one.
    ///
    /// This is the shutdown path — it runs from quit handlers, panic
    /// unwinds, and signal handlers, so it must not fail: I/O errors are
    /// swallowed and the sequence presses on. (Even in the worst case the
    /// protocol fail-safe applies: no frames means the motor coasts to a
    /// stop.)
    pub fn safe_stop(&mut self) {
        // Unacknowledged by the motor, so send the protocol's five copies
        // and press on regardless — same best-effort contract as the rest.
        let mode = frame_mode(self.id, Mode::Velocity);
        for _ in 0..5 {
            let _ = self.send_paced(&mode, MODE_REPEAT_GAP);
        }

        let zero = frame_velocity(self.id, 0, 1);
        let brake = frame_brake(self.id);
        for frame in [
            &zero, &zero, &zero, &zero, &zero, &brake, &brake, &brake, &brake, &brake,
        ] {
            let _ = self.send_paced(frame, SAFE_STOP_GAP);
        }
    }

    /// Send under the lock, then observe the gap *outside* it — never hold
    /// the shared bus while sleeping.
    fn send_paced(&self, frame: &[u8], gap: Duration) -> Result<()> {
        lock(&self.transport).send(frame)?;
        let gap = lock(&self.transport).pace(gap);
        if !gap.is_zero() {
            std::thread::sleep(gap);
        }
        Ok(())
    }
}
