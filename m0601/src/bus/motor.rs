//! The per-motor [`M0601`] handle: one motor's view of the shared [`Bus`].

use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use crate::error::Result;
use crate::protocol::{
    Frame, ReplyKind, frame_brake, frame_current, frame_feedback, frame_position, frame_velocity,
    frames, parse_feedback,
};
use crate::transport::{SerialTransport, Transport};
use crate::types::{Feedback, Mode};

use super::Bus;
use super::pacing::{Port, mode_all, peek_min_gap, stop_all, with_gap};

/// How a handle maps the *position* an opposite-side (mirrored) wheel reports.
///
/// [`mirrored(true)`](M0601::mirrored) flips the signs of reported speed and
/// current so "positive = robot forward" holds on both sides, but it leaves
/// the reported **angle** alone — because the right transform for an angle
/// depends on your mechanical convention, and the driver won't guess. This
/// selects that convention when you do want it applied.
///
/// It is an independent knob from [`mirrored`](M0601::mirrored): set both on a
/// mirrored wheel (`.mirrored(true).position_mirror(PositionMirror::Reflect)`)
/// to have angle read in robot-forward terms too. [`Feedback::raw`] always
/// keeps the untouched wire bytes regardless.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
#[non_exhaustive]
pub enum PositionMirror {
    /// Report the wire angle unchanged. The default, and the crate's
    /// historical behavior for every handle.
    #[default]
    PassThrough,
    /// Reflect the angle about 0°: a reported `θ` becomes `(360° − θ) mod
    /// 360`, so a mirror-image wheel's angle counts up the same way its
    /// sign-flipped speed does.
    Reflect,
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
/// Reported position passes through untouched by default — the right mirror
/// transform for an angle depends on your mechanical convention, so the
/// driver won't guess — but opt into one with
/// [`position_mirror`](Self::position_mirror) ([`PositionMirror::Reflect`])
/// when you want the angle in robot-forward terms too. [`Feedback::raw`]
/// always keeps the unmodified wire frame.
pub struct M0601<T: Transport = SerialTransport> {
    pub(super) port: Arc<Mutex<Port<T>>>,
    pub(super) id: u8,
    pub(super) timeout: Duration,
    pub(super) mirrored: bool,
    /// How reported position is mapped for a mirrored wheel — see
    /// [`position_mirror`](Self::position_mirror).
    pub(super) position_mirror: PositionMirror,
    /// Reject CRC-failing telemetry rather than returning it advisory — see
    /// [`with_strict_crc`](Self::with_strict_crc).
    pub(super) strict_crc: bool,
    /// Default accel byte for [`drive_velocity`](Self::drive_velocity) — see
    /// [`with_default_accel`](Self::with_default_accel).
    pub(super) default_accel: u8,
}

// Manual impl: handles are cheap to clone regardless of whether T is Clone.
impl<T: Transport> Clone for M0601<T> {
    fn clone(&self) -> Self {
        Self {
            port: Arc::clone(&self.port),
            id: self.id,
            timeout: self.timeout,
            mirrored: self.mirrored,
            position_mirror: self.position_mirror,
            strict_crc: self.strict_crc,
            default_accel: self.default_accel,
        }
    }
}

// Manual impl: T need not be Debug, and the port mutex must not be *blocked
// on* just to format — `peek_min_gap` degrades to `None` rather than
// deadlocking if Debug runs from a path already holding the lock.
impl<T: Transport> std::fmt::Debug for M0601<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("M0601")
            .field("id", &self.id)
            .field("timeout", &self.timeout)
            .field("mirrored", &self.mirrored)
            .field("position_mirror", &self.position_mirror)
            .field("strict_crc", &self.strict_crc)
            .field("default_accel", &self.default_accel)
            .field("min_gap", &peek_min_gap(&self.port))
            .finish_non_exhaustive()
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

    /// Choose how reported **position** is mapped for a mirrored wheel — see
    /// [`PositionMirror`]. Builder-style:
    /// `bus.motor(0x01)?.mirrored(true).position_mirror(PositionMirror::Reflect)`.
    ///
    /// Defaults to [`PositionMirror::PassThrough`] (the wire angle unchanged),
    /// which is what every handle did before this existed. This is independent
    /// of [`mirrored`](Self::mirrored): it only rewrites the *position* field
    /// of parsed telemetry, and never the raw frame ([`Feedback::raw`]).
    #[must_use]
    pub fn position_mirror(mut self, mode: PositionMirror) -> Self {
        self.position_mirror = mode;
        self
    }

    /// This handle's position-mirror convention
    /// ([`position_mirror`](Self::position_mirror)).
    pub fn position_mirror_mode(&self) -> PositionMirror {
        self.position_mirror
    }

    /// Reject CRC-failing telemetry on this handle: any decoded [`Feedback`]
    /// whose byte 9 fails the CRC-8/MAXIM check is turned into `Ok(None)` by
    /// [`transact`](Self::transact), [`query`](Self::query) and
    /// [`query_with`](Self::query_with), instead of being returned with
    /// [`Feedback::crc_ok`]` == false`.
    ///
    /// Builder-style, for the single-motor [`open`](Self::open) path:
    /// `M0601::open(port, id, t)?.with_strict_crc(true)`. Motors minted from a
    /// [`Bus`] inherit the bus-wide setting
    /// ([`Bus::with_strict_crc`]); this overrides it for one handle. The
    /// default is **advisory** (`false`) — see [`Bus::with_strict_crc`] for
    /// why, and prefer strict only where a corrupt frame would poison an
    /// odometry integrator.
    #[must_use]
    pub fn with_strict_crc(mut self, strict: bool) -> Self {
        self.strict_crc = strict;
        self
    }

    /// Enable or disable strict CRC on an existing handle in place — the
    /// setter form of [`with_strict_crc`](Self::with_strict_crc).
    pub fn set_strict_crc(&mut self, strict: bool) {
        self.strict_crc = strict;
    }

    /// Whether this handle rejects CRC-failing telemetry
    /// ([`with_strict_crc`](Self::with_strict_crc)).
    pub fn strict_crc(&self) -> bool {
        self.strict_crc
    }

    /// Set the default acceleration byte used by
    /// [`drive_velocity`](Self::drive_velocity) on this handle.
    ///
    /// Builder-style. Motors minted from a [`Bus`] inherit the bus-wide
    /// default ([`Bus::with_default_accel`]); this overrides it for one
    /// handle. The per-call [`drive_velocity_accel`](Self::drive_velocity_accel)
    /// always takes precedence. Defaults to
    /// [`DEFAULT_DRIVE_ACCEL`](crate::DEFAULT_DRIVE_ACCEL) (the motor's
    /// fastest ramp).
    #[must_use]
    pub fn with_default_accel(mut self, accel: u8) -> Self {
        self.default_accel = accel;
        self
    }

    /// Set the default velocity-drive accel on an existing handle in place —
    /// the setter form of [`with_default_accel`](Self::with_default_accel).
    pub fn set_default_accel(&mut self, accel: u8) {
        self.default_accel = accel;
    }

    /// This handle's default velocity-drive accel
    /// ([`with_default_accel`](Self::with_default_accel)).
    pub fn default_accel(&self) -> u8 {
        self.default_accel
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
        Arc::into_inner(self.port).map(|m| {
            m.into_inner()
                .unwrap_or_else(PoisonError::into_inner)
                .transport
        })
    }

    /// Send an arbitrary frame and return the raw reply bytes (may be
    /// empty).
    pub fn send_raw(&mut self, frame: &[u8], wait: Duration) -> Result<Vec<u8>> {
        with_gap(&self.port, |t| t.send_recv(frame, wait))
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
        // Pick the frame addressed to this handle. In strict-CRC mode a frame
        // whose CRC does not check is not a candidate at all — so a stale,
        // corrupt frame sitting ahead of the real reply in the same read is
        // skipped rather than sinking the whole reply, and a later valid frame
        // for this id can still be selected. Advisory mode keeps taking the
        // first id-matching frame regardless of CRC.
        let fb = frames(tx, rx)?.find_map(|frame| {
            parse_feedback(frame, kind)
                .filter(|fb| fb.id == self.id && (!self.strict_crc || fb.crc_ok))
        })?;
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
        if self.position_mirror == PositionMirror::Reflect {
            // Reflect about 0°: 360 − θ, folded back into [0, 360) so that a
            // reported 0° stays 0° rather than becoming 360°. `raw` is left
            // untouched — it is always the ground-truth wire frame.
            fb.position_deg = (360.0 - fb.position_deg).rem_euclid(360.0);
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
    /// all expected outcomes, not errors. Suited to a realtime control loop
    /// with a short `wait` (~6 ms). Mirrored handles flip the signs of
    /// the parsed speed/current (the raw frame is untouched).
    ///
    /// When [`strict CRC`](Self::with_strict_crc) is enabled on this handle, a
    /// decoded frame whose CRC does not check drops to `Ok(None)` here rather
    /// than being returned with `crc_ok == false`.
    pub fn transact(&mut self, frame: &Frame, wait: Duration) -> Result<Option<Feedback>> {
        let rx = with_gap(&self.port, |t| t.send_recv(frame, wait))?;
        // Strict handles reject CRC-failing frames during selection (see
        // `parse_reply`), so bad telemetry never leaves the driver and a valid
        // frame later in the same read is still picked up; the default stays
        // advisory and returns the first id-matching frame with `crc_ok` set.
        Ok(self.parse_reply(frame, &rx))
    }

    /// Query telemetry with a feedback (`0x74`) frame, waiting the configured
    /// timeout for the reply. Equivalent to
    /// [`query_with(self.timeout())`](Self::query_with).
    ///
    /// The full-timeout wait suits one-shot and interactive use, where the backstop
    /// timeout is an acceptable ceiling on a single call. It is **not** for a
    /// realtime loop on a shared bus: the wait is held under the bus lock, so
    /// a slow or silent motor stalls every other wheel for the whole timeout —
    /// use [`query_with`](Self::query_with) with a short wait there.
    ///
    /// This is the only exchange whose reply carries the winding
    /// temperature (`temp_c` is `Some`); its position reading is the
    /// coarse 8-bit one (~1.4°). Drive replies via
    /// [`transact`](Self::transact) have it the other way around.
    pub fn query(&mut self) -> Result<Option<Feedback>> {
        self.query_with(self.timeout)
    }

    /// Query telemetry with a feedback (`0x74`) frame, waiting only `wait`
    /// for the reply instead of the configured backstop timeout.
    ///
    /// Same reply as [`query`](Self::query) — the winding temperature and the
    /// coarse 8-bit position — but with a caller-chosen wait, so a realtime
    /// control loop can bound how long the shared bus lock is held. That wait
    /// is the whole cost of the transaction on a half-duplex bus (another
    /// motor's frame cannot be interleaved into it), so a loop polling several
    /// motors in turn should pass a short wait — roughly **3–6 ms**, enough
    /// to cover the ~0.9 ms reply frame plus the motor's turnaround — rather
    /// than the tens-to-hundreds of ms a backstop [`timeout`](Self::timeout)
    /// typically is. A reply that has not arrived within `wait` reads as a
    /// silent bus (`Ok(None)`), which the loop simply retries next cycle.
    ///
    /// Honours [`strict CRC`](Self::with_strict_crc) exactly as
    /// [`transact`](Self::transact) does.
    pub fn query_with(&mut self, wait: Duration) -> Result<Option<Feedback>> {
        let frame = frame_feedback(self.id);
        self.transact(&frame, wait)
    }

    /// Switch control mode, sending the `0xA0` frame five times as the
    /// protocol requires. The motor sends no acknowledgement.
    ///
    /// Before switching to [`Mode::Position`], the motor must be turning
    /// slower than 10 RPM.
    pub fn set_mode(&mut self, mode: Mode) -> Result<()> {
        mode_all(&self.port, &[self.id], mode)
    }

    /// Send one velocity drive frame (clamped to ±330 RPM); a mirrored handle
    /// negates `rpm` first.
    /// Must be resent at ≥50 Hz to sustain motion — see the type-level docs.
    ///
    /// Uses this handle's default accel —
    /// [`DEFAULT_DRIVE_ACCEL`](crate::DEFAULT_DRIVE_ACCEL) (the motor's
    /// *fastest* ramp) unless changed with
    /// [`with_default_accel`](Self::with_default_accel) /
    /// [`Bus::with_default_accel`]. Pass an explicit ramp per call with
    /// [`drive_velocity_accel`](Self::drive_velocity_accel).
    pub fn drive_velocity(&mut self, rpm: i16) -> Result<()> {
        self.drive_velocity_accel(rpm, self.default_accel)
    }

    /// Send one velocity drive frame with an explicit acceleration.
    ///
    /// **Larger is gentler.** `1` — what
    /// [`drive_velocity`](Self::drive_velocity) uses — is the *fastest* ramp,
    /// and `0` selects the motor's default, which measures identical to `1`
    /// rather than being a neutral middle. A large step at that ramp draws a
    /// current spike that can trip the motor's 3 A bus-overcurrent protection
    /// on a loaded wheel; raise the value to soften it.
    ///
    /// Raise it modestly: the ramp slows fast. Measured on an unloaded wheel,
    /// a step to 120 RPM takes ~0.45 s at `1`, ~2 s at `5`, and had not
    /// arrived after 3 s at `20`. `3`–`5` is a useful softening; `40` is
    /// nearly a standstill. See
    /// [`frame_velocity`](crate::protocol::frame_velocity) for the full table.
    ///
    /// The accel byte bounds how fast the motor chases the setpoint, not how
    /// fast *you* move the setpoint — for that, and for a mitigation that does
    /// not depend on the motor's ramp at all, see
    /// [`SlewLimiter`](crate::SlewLimiter).
    pub fn drive_velocity_accel(&mut self, rpm: i16, accel: u8) -> Result<()> {
        let rpm = if self.mirrored {
            rpm.saturating_neg()
        } else {
            rpm
        };
        let frame = frame_velocity(self.id, rpm, accel);
        with_gap(&self.port, |t| t.send(&frame))
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
        let frame = frame_current(self.id, value);
        with_gap(&self.port, |t| t.send(&frame))
    }

    /// Send one position drive frame (clamped to `0..=32767` = 0°..360°).
    /// NOT mirror-adjusted — see the type-level docs.
    /// Must be resent at ≥50 Hz to hold — see the type-level docs.
    pub fn drive_position(&mut self, raw: u16) -> Result<()> {
        let frame = frame_position(self.id, raw);
        with_gap(&self.port, |t| t.send(&frame))
    }

    /// Send one electric-brake frame (velocity mode only).
    /// Must be resent at ≥50 Hz to keep braking — see the type-level docs.
    pub fn brake(&mut self) -> Result<()> {
        let frame = frame_brake(self.id);
        with_gap(&self.port, |t| t.send(&frame))
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
        stop_all(&self.port, &[self.id]);
    }
}
