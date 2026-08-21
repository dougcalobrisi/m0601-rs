//! Pure protocol layer: frame construction and parsing. No I/O.
//!
//! Everything here is a deterministic function of its inputs, which is what
//! makes the wire format unit-testable byte-for-byte (see `tests/vectors.rs`,
//! whose expected bytes are hand-derived from the DFRobot protocol
//! documentation and an independent CRC-8/MAXIM implementation).
//!
//! Out-of-range drive values are **clamped**, not rejected — a setpoint that
//! is merely too large should saturate the wheel, never wrap it around to
//! full reverse. This is part of the crate's API contract.

use std::time::Duration;

use crate::error::{Error, Result};
use crate::types::{Faults, Feedback, Mode};

/// RS485 baud rate. The format is fixed: 115200 8N1, half-duplex.
pub const BAUD: u32 = 115_200;

/// Every frame on the bus, in both directions, is exactly this long.
pub const FRAME_LEN: usize = 10;

/// Drive command: the 16-bit value is interpreted per the active [`Mode`].
pub const CMD_DRIVE: u8 = 0x64;
/// Feedback query command: the motor replies with a telemetry frame.
pub const CMD_QUERY: u8 = 0x74;
/// Mode-switch command. **Its last byte is the mode, not a CRC.**
pub const CMD_MODE: u8 = 0xA0;

/// Minimum velocity command, RPM.
pub const RPM_MIN: i16 = -330;
/// Maximum velocity command, RPM.
pub const RPM_MAX: i16 = 330;
/// Minimum current command; maps to roughly −8 A.
///
/// Note this is `-32767`, not [`i16::MIN`] — the range is symmetric, and
/// `-32768` is clamped away rather than sent.
pub const CUR_MIN: i16 = -32_767;
/// Maximum current command; maps to roughly +8 A.
pub const CUR_MAX: i16 = 32_767;
/// Maximum position command; `0..=32767` maps to 0°..360°.
pub const POS_MAX: u16 = 32_767;
/// Brake byte value: in velocity mode, `0xFF` in byte 7 engages the
/// electric brake.
pub const BRAKE_BYTE: u8 = 0xFF;

/// Minimum drive-frame repetition rate (Hz) that sustains motion.
///
/// The M0601 is a *polling* device: it keeps moving only while drive frames
/// keep arriving. Below ~50 Hz it coasts to a stop.
pub const DRIVE_HZ_MIN: u32 = 50;
/// Maximum command rate the motor accepts (Hz).
pub const CMD_HZ_MAX: u32 = 500;

/// A complete 10-byte bus frame.
pub type Frame = [u8; FRAME_LEN];

/// Time on the wire for one [`FRAME_LEN`]-byte frame at [`BAUD`] — 8N1 sends
/// 10 bits per byte (1 start + 8 data + 1 stop). This is the unit every
/// bus-occupancy budget is built from (see [`crate::bus::bus_period`]); the
/// same wire time that sizes [`DEFAULT_MIN_GAP`](crate::DEFAULT_MIN_GAP).
///
/// ```
/// use m0601::protocol::frame_time;
/// // 10 bytes × 10 bits ÷ 115200 baud ≈ 868 µs.
/// assert_eq!(frame_time().as_micros(), 868);
/// ```
pub fn frame_time() -> Duration {
    let bits = FRAME_LEN as u64 * 10;
    Duration::from_micros(bits * 1_000_000 / u64::from(BAUD))
}

/// The longest a wheel may go between drive frames before it coasts: the
/// period of the [`DRIVE_HZ_MIN`] floor. A periodic control loop's cycle
/// must not exceed this, or every cycle the motor slips below the floor and
/// coasts a little.
///
/// ```
/// use m0601::protocol::drive_floor;
/// assert_eq!(drive_floor().as_millis(), 20); // 1 s / 50 Hz
/// ```
pub fn drive_floor() -> Duration {
    Duration::from_secs(1) / DRIVE_HZ_MIN
}

/// CRC-8/MAXIM (Dallas 1-Wire): polynomial x⁸+x⁵+x⁴+1, reflected (0x8C),
/// init 0.
///
/// Host→motor frames carry this over bytes 0–8 in byte 9 — except the
/// mode-switch ([`frame_mode`]) and set-ID ([`frame_set_id`]) frames, which
/// carry no CRC at all.
///
/// Motor replies carry it too, over the same bytes: a hardware capture
/// settled that question, and [`Feedback::crc_ok`] reports the result. It
/// stays informational by default — telemetry is not *rejected* on it —
/// because the reference implementations disagree and firmware revisions may
/// differ. Callers who need the opposite trade-off can opt in with
/// [`parse_feedback_strict`] or [`Bus::with_strict_crc`](crate::Bus::with_strict_crc).
/// See `PROTOCOL.md` in the repository.
///
/// ```
/// use m0601::protocol::crc8_maxim;
/// assert_eq!(crc8_maxim(&[]), 0x00);
/// assert_eq!(crc8_maxim(&[0, 1, 2, 3, 4, 5, 6, 7, 8]), 0x83);
/// ```
pub fn crc8_maxim(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8C
            } else {
                crc >> 1
            };
        }
    }
    crc
}

/// Build a standard frame: `[id, cmd, data..., crc]`.
fn frame(id: u8, cmd: u8, data: [u8; 7]) -> Frame {
    let mut f: Frame = [0; FRAME_LEN];
    f[0] = id;
    f[1] = cmd;
    f[2..9].copy_from_slice(&data);
    f[9] = crc8_maxim(&f[..9]);
    f
}

/// Velocity drive frame. `rpm` is clamped to [`RPM_MIN`]`..=`[`RPM_MAX`].
///
/// `accel` sets how hard the motor ramps toward the setpoint: **larger is
/// gentler**, and `0` selects the motor's own default — which is the *fastest*
/// ramp, not a middle one.
///
/// No vendor source states that direction; it was measured here
/// (`accel_direction_capture` in `m0601/tests/hardware.rs`). Stepping an
/// unloaded wheel from rest to 120 RPM, time to 90% of setpoint was:
///
/// | accel | 0 | 1 | 2 | 5 | 20 | 100 | 255 |
/// |---|---|---|---|---|---|---|---|
/// | t to 90% | 446 ms | 446 ms | 837 ms | 1.99 s | >3 s | >3 s | >3 s |
///
/// Time-to-setpoint is linear in the byte at roughly **3.6 ms per RPM per
/// unit**, so the byte behaves as a *time per rpm* — the orientation of the
/// wiki's worked example, not of the `RPM/0.1ms` **rate** its own unit line
/// and the upstream manual give. Read literally that rate would make larger
/// values harsher; it does the opposite. `0` and `1` are indistinguishable,
/// confirming the wiki's "the default value as 1". The ramp's *shape* is
/// verified too (`accel_curve_capture`): the full spin-up curve is a straight
/// line to setpoint, not an exponential approach, so "linear" is measured
/// rather than inferred from summary times.
///
/// The magnitude is one unloaded motor on one rig; the ordering is the durable
/// part. Large values are gentler than they look useful: at `20` the same step
/// had only reached 41 RPM after three seconds. See `PROTOCOL.md`.
///
/// Only sustains motion while resent at ≥[`DRIVE_HZ_MIN`] Hz.
///
/// ```
/// use m0601::protocol::frame_velocity;
/// assert_eq!(
///     frame_velocity(0x01, 100, 1),
///     [0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00, 0x00, 0xE4],
/// );
/// // Out-of-range values clamp: 500 RPM becomes 330.
/// assert_eq!(
///     frame_velocity(0x01, 500, 1),
///     [0x01, 0x64, 0x01, 0x4A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x7C],
/// );
/// ```
pub fn frame_velocity(id: u8, rpm: i16, accel: u8) -> Frame {
    let v = rpm.clamp(RPM_MIN, RPM_MAX).to_be_bytes();
    frame(id, CMD_DRIVE, [v[0], v[1], 0, 0, accel, 0, 0])
}

/// Current drive frame. `value` is clamped to [`CUR_MIN`]`..=`[`CUR_MAX`]
/// (`±32767`, roughly −8 A..+8 A).
///
/// The range is symmetric, so [`i16::MIN`] (`-32768`) is *not* a valid
/// setpoint and clamps up to `-32767`.
///
/// ```
/// use m0601::protocol::frame_current;
/// assert_eq!(
///     frame_current(0x01, -1234),
///     [0x01, 0x64, 0xFB, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07],
/// );
/// // i16::MIN clamps to -32767 rather than going out on the wire as 0x8000.
/// assert_eq!(
///     frame_current(0x01, i16::MIN),
///     [0x01, 0x64, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0],
/// );
/// ```
pub fn frame_current(id: u8, value: i16) -> Frame {
    let v = value.clamp(CUR_MIN, CUR_MAX).to_be_bytes();
    frame(id, CMD_DRIVE, [v[0], v[1], 0, 0, 0, 0, 0])
}

/// Position drive frame. `raw` is clamped to `0..=`[`POS_MAX`]
/// (0°..360°).
///
/// ```
/// use m0601::protocol::frame_position;
/// assert_eq!(
///     frame_position(0x01, 32767),
///     [0x01, 0x64, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97],
/// );
/// ```
pub fn frame_position(id: u8, raw: u16) -> Frame {
    let v = raw.min(POS_MAX).to_be_bytes();
    frame(id, CMD_DRIVE, [v[0], v[1], 0, 0, 0, 0, 0])
}

/// Electric-brake frame (velocity mode only): value 0 with [`BRAKE_BYTE`]
/// in the brake position.
///
/// ```
/// use m0601::protocol::frame_brake;
/// assert_eq!(
///     frame_brake(0x01),
///     [0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1],
/// );
/// ```
pub fn frame_brake(id: u8) -> Frame {
    frame(id, CMD_DRIVE, [0, 0, 0, 0, 0, BRAKE_BYTE, 0])
}

/// Mode-switch frame. **The last byte is the mode value, not a CRC** — this
/// is the protocol's one deliberate deviation from the standard frame shape.
/// Must be sent five times ([`M0601::set_mode`](crate::M0601::set_mode)
/// does so).
///
/// ```
/// use m0601::{protocol::frame_mode, Mode};
/// assert_eq!(
///     frame_mode(0x01, Mode::Velocity),
///     [0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02],
/// );
/// ```
pub fn frame_mode(id: u8, mode: Mode) -> Frame {
    let mut f: Frame = [0; FRAME_LEN];
    f[0] = id;
    f[1] = CMD_MODE;
    f[9] = mode.as_byte();
    f
}

/// Feedback query frame: the addressed motor replies with telemetry.
///
/// ```
/// use m0601::protocol::frame_feedback;
/// assert_eq!(
///     frame_feedback(0x01),
///     [0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04],
/// );
/// ```
pub fn frame_feedback(id: u8) -> Frame {
    frame(id, CMD_QUERY, [0; 7])
}

/// Broadcast ID-query frame (fixed bytes `C8 64 00×7 DE`). Any motor on the
/// bus answers with a frame starting with its own ID.
pub fn frame_id_query() -> Frame {
    [0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE]
}

/// Set-ID frame (`AA 55 53 <new_id> 00×6`, **no CRC**). Persistent; must be
/// sent five times with only one motor on the bus
/// ([`Bus::set_id`](crate::Bus::set_id) handles both).
///
/// Returns [`Error::InvalidId`] outside `0x01..=0xFE`.
pub fn frame_set_id(new_id: u8) -> Result<Frame> {
    validate_id(new_id)?;
    Ok([0xAA, 0x55, 0x53, new_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
}

/// Check that `id` is an assignable motor ID (`0x01..=0xFE`).
///
/// `0xC8` is accepted but best avoided: it is the destination byte of the
/// broadcast ID query ([`frame_id_query`]), so a motor assigned that ID
/// cannot be told apart from the query itself when a half-duplex adapter
/// echoes the transmission back.
pub fn validate_id(id: u8) -> Result<()> {
    if (0x01..=0xFE).contains(&id) {
        Ok(())
    } else {
        Err(Error::InvalidId(id))
    }
}

/// Build a [`Frame`] from raw bytes: 9 bytes get a CRC-8/MAXIM appended,
/// 10 bytes pass through untouched (byte 9 is *not* recomputed, so this can
/// send deliberately corrupt frames).
///
/// Returns [`Error::InvalidFrameLen`] for any other length.
///
/// ```
/// use m0601::protocol::frame_from_bytes;
/// // 9 bytes: CRC appended.
/// assert_eq!(
///     frame_from_bytes(&[0x01, 0x74, 0, 0, 0, 0, 0, 0, 0])?[9],
///     0x04,
/// );
/// // 10 bytes: byte 9 kept verbatim, even when wrong.
/// assert_eq!(
///     frame_from_bytes(&[0x01, 0x74, 0, 0, 0, 0, 0, 0, 0, 0xFF])?[9],
///     0xFF,
/// );
/// assert!(frame_from_bytes(&[0x01, 0x74]).is_err());
/// # Ok::<(), m0601::Error>(())
/// ```
pub fn frame_from_bytes(bytes: &[u8]) -> Result<Frame> {
    match bytes.len() {
        9 => {
            let mut f: Frame = [0; FRAME_LEN];
            f[..9].copy_from_slice(bytes);
            f[9] = crc8_maxim(&f[..9]);
            Ok(f)
        }
        FRAME_LEN => bytes
            .try_into()
            .map_err(|_| Error::InvalidFrameLen(bytes.len())),
        n => Err(Error::InvalidFrameLen(n)),
    }
}

/// Full-scale torque current in amps, at [`CUR_MAX`] (and −[`CUR_MAX`]).
pub const CUR_FULL_SCALE_A: f32 = 8.0;

/// Raw current value → amps (`raw × 8 / 32767`).
///
/// ```
/// use m0601::protocol::raw_to_amps;
/// assert_eq!(raw_to_amps(0), 0.0);
/// assert!((raw_to_amps(32767) - 8.0).abs() < 1e-6);
/// assert!((raw_to_amps(-4096) + 1.0).abs() < 1e-3);
/// ```
pub fn raw_to_amps(raw: i16) -> f32 {
    f32::from(raw) * CUR_FULL_SCALE_A / f32::from(CUR_MAX)
}

/// Amps → raw current setpoint, clamped to [`CUR_MIN`]`..=`[`CUR_MAX`].
///
/// Inverts [`raw_to_amps`], rounding to nearest. Non-finite input maps to
/// `0` — `f32::clamp` propagates NaN rather than clamping it, so it is ruled
/// out here instead of relying on the `as` cast's NaN-to-zero rule.
///
/// ```
/// use m0601::protocol::{amps_to_raw, CUR_MAX, CUR_MIN};
/// assert_eq!(amps_to_raw(0.0), 0);
/// assert_eq!(amps_to_raw(8.0), CUR_MAX);
/// assert_eq!(amps_to_raw(1.0), 4096);
/// // Beyond the reachable range it saturates rather than wrapping.
/// assert_eq!(amps_to_raw(100.0), CUR_MAX);
/// assert_eq!(amps_to_raw(-100.0), CUR_MIN);
/// assert_eq!(amps_to_raw(f32::NAN), 0);
/// ```
pub fn amps_to_raw(amps: f32) -> i16 {
    if !amps.is_finite() {
        return 0;
    }
    let raw = (amps * f32::from(CUR_MAX) / CUR_FULL_SCALE_A).round();
    // Clamped before the cast, so the conversion is total.
    raw.clamp(f32::from(CUR_MIN), f32::from(CUR_MAX)) as i16
}

/// 16-bit position → degrees (`raw × 360 / 32767`), as carried by a
/// [`ReplyKind::Drive`] reply.
///
/// ```
/// use m0601::protocol::raw_to_deg;
/// assert_eq!(raw_to_deg(0), 0.0);
/// assert_eq!(raw_to_deg(32767), 360.0);
/// ```
pub fn raw_to_deg(raw: u16) -> f32 {
    f32::from(raw) * 360.0 / f32::from(POS_MAX)
}

/// 8-bit position → degrees (`raw × 360 / 255`), as carried by a
/// [`ReplyKind::Query`] reply.
///
/// The divisor is **255**, not 256, so `0xFF` reads as a full 360° (i.e. 0°,
/// wrapped); every known implementation divides by 255.
///
/// ```
/// use m0601::protocol::raw8_to_deg;
/// assert_eq!(raw8_to_deg(0), 0.0);
/// assert_eq!(raw8_to_deg(255), 360.0);
/// ```
pub fn raw8_to_deg(raw: u8) -> f32 {
    f32::from(raw) * 360.0 / 255.0
}

/// Degrees → raw position setpoint, clamped to `0..=`[`POS_MAX`].
///
/// Clamps rather than wrapping: an angle slightly past 360° should hold at
/// the top of the range, not snap round to 0° and drive a full revolution.
/// Non-finite input maps to `0`.
///
/// Rounds to nearest, so an angle read back from a drive reply round-trips
/// to exactly the value it came from — truncating instead can land one step
/// low and turn "hold this angle" into a command to move.
///
/// ```
/// use m0601::protocol::{deg_to_raw, raw_to_deg, POS_MAX};
/// assert_eq!(deg_to_raw(0.0), 0);
/// assert_eq!(deg_to_raw(180.0), 16_384);
/// assert_eq!(deg_to_raw(360.0), POS_MAX);
/// // Out-of-band and non-finite inputs clamp rather than wrap or trap.
/// assert_eq!(deg_to_raw(-90.0), 0);
/// assert_eq!(deg_to_raw(720.0), POS_MAX);
/// assert_eq!(deg_to_raw(f32::NAN), 0);
/// // Round-trips exactly.
/// assert_eq!(deg_to_raw(raw_to_deg(20_000)), 20_000);
/// ```
pub fn deg_to_raw(deg: f32) -> u16 {
    if !deg.is_finite() {
        return 0;
    }
    let frac = (deg / 360.0).clamp(0.0, 1.0);
    // Saturating cast of an already-clamped value: cannot wrap or trap.
    (frac * f32::from(POS_MAX)).round() as u16
}

/// Degrees → raw 8-bit position, clamped to `0..=255`, as carried in byte 7
/// of a [`ReplyKind::Query`] reply.
///
/// The inverse of [`raw8_to_deg`] (divisor **255**, so 360° maps to `0xFF`).
/// Like [`deg_to_raw`] it clamps rather than wrapping, rounds to nearest, and
/// maps non-finite input to `0`. Mainly of use to a simulator or test
/// synthesizing a query reply with [`frame_query_reply`].
///
/// ```
/// use m0601::protocol::{deg_to_raw8, raw8_to_deg};
/// assert_eq!(deg_to_raw8(0.0), 0);
/// assert_eq!(deg_to_raw8(360.0), 255);
/// assert_eq!(deg_to_raw8(-1.0), 0);      // clamps, not wraps
/// assert_eq!(deg_to_raw8(720.0), 255);
/// assert_eq!(deg_to_raw8(f32::NAN), 0);
/// // Round-trips a value that came from raw8_to_deg exactly.
/// assert_eq!(deg_to_raw8(raw8_to_deg(200)), 200);
/// ```
pub fn deg_to_raw8(deg: f32) -> u8 {
    if !deg.is_finite() {
        return 0;
    }
    let frac = (deg / 360.0).clamp(0.0, 1.0);
    // Saturating cast of an already-clamped value: cannot wrap or trap.
    (frac * 255.0).round() as u8
}

/// Which command elicited a telemetry reply — and therefore how its
/// bytes 6–7 must be decoded.
///
/// The motor answers with **two different reply layouts** (verified against
/// the DDT vendor sample and the DFRobot wiki — see `PROTOCOL.md`). Bytes
/// 0–5, 8 and 9 are identical in both; only bytes 6–7 differ:
///
/// | Kind    | Elicited by             | Byte 6            | Byte 7        |
/// |---------|-------------------------|-------------------|---------------|
/// | `Query` | `0x74` feedback query   | winding temp (°C) | position u8   |
/// | `Drive` | `0x64` drive, broadcast | position u16 BE (high) | (low)    |
///
/// A `Drive` reply carries **no temperature**, but its 16-bit position is
/// ~128× finer than the `Query` reply's single byte (~0.011° vs ~1.4°).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReplyKind {
    /// Reply to a feedback query ([`CMD_QUERY`], `0x74`): byte 6 is the
    /// winding temperature in °C, byte 7 an 8-bit position (×360/255°).
    Query,
    /// Reply to a drive frame ([`CMD_DRIVE`], `0x64`) or the broadcast ID
    /// query: bytes 6–7 are a 16-bit big-endian position (×360/32767°);
    /// the frame carries no temperature.
    Drive,
}

impl ReplyKind {
    /// Classify a reply by the TX frame that elicited it (byte 1, the
    /// command byte). Returns `None` for frames that elicit no telemetry —
    /// the mode switch (`0xA0`) and set-ID frames — and for short slices.
    ///
    /// The broadcast ID query classifies as [`Drive`](Self::Drive): its
    /// byte 1 is [`CMD_DRIVE`], and motors answer it in the drive layout.
    ///
    /// ```
    /// use m0601::protocol::{self, ReplyKind};
    /// assert_eq!(ReplyKind::from_tx(&protocol::frame_feedback(1)), Some(ReplyKind::Query));
    /// assert_eq!(ReplyKind::from_tx(&protocol::frame_brake(1)), Some(ReplyKind::Drive));
    /// assert_eq!(ReplyKind::from_tx(&protocol::frame_id_query()), Some(ReplyKind::Drive));
    /// assert_eq!(ReplyKind::from_tx(&protocol::frame_mode(1, m0601::Mode::Velocity)), None);
    /// assert_eq!(ReplyKind::from_tx(&[]), None);
    /// ```
    pub fn from_tx(tx: &[u8]) -> Option<Self> {
        match *tx.get(1)? {
            CMD_QUERY => Some(Self::Query),
            CMD_DRIVE => Some(Self::Drive),
            _ => None,
        }
    }
}

/// Parse a telemetry frame from raw reply bytes, decoding bytes 6–7
/// according to `kind` — see [`ReplyKind`] for why the caller must know
/// which command the reply answers.
///
/// Returns `None` when fewer than [`FRAME_LEN`] bytes are supplied; longer
/// input parses its first 10 bytes. Frames are validated by *length only*:
/// [`Feedback::crc_ok`] reports whether byte 9 matches a CRC-8/MAXIM
/// (genuine replies do carry one — verified on hardware), but this
/// function never rejects on it — see `PROTOCOL.md`.
///
/// Common layout: `[id, mode, current_i16_be, speed_i16_be, .., faults,
/// chk]` with current scaled ×8/32767 A. `Query` replies put temperature
/// (°C) in byte 6 and an 8-bit position (×360/255°) in byte 7; `Drive`
/// replies put a 16-bit position (×360/32767°) in bytes 6–7 and no
/// temperature.
///
/// The `Query` position uses ×360/**255**, so byte 7 = `0xFF` reads as a
/// full 360° (i.e. 0°, wrapped); every known implementation divides by 255,
/// not 256.
///
/// ```
/// use m0601::protocol::{parse_feedback, ReplyKind};
/// let raw = [0x01, 0x02, 0xF8, 0x30, 0x00, 0x64, 0x28, 0x80, 0x03, 0x00];
/// let q = parse_feedback(&raw, ReplyKind::Query).unwrap();
/// assert_eq!(q.speed_rpm, 100);
/// assert_eq!(q.temp_c, Some(40));
/// assert_eq!(q.faults.to_string(), "SensorErr | Overcurrent");
/// assert!(!q.crc_ok);
/// // The very same bytes decode differently as a drive reply: bytes 6–7
/// // are one 16-bit position (0x2880 = 10368 → ~113.9°), no temperature.
/// let d = parse_feedback(&raw, ReplyKind::Drive).unwrap();
/// assert_eq!(d.temp_c, None);
/// assert!((d.position_deg - 113.91).abs() < 0.01);
/// ```
pub fn parse_feedback(data: &[u8], kind: ReplyKind) -> Option<Feedback> {
    let raw: Frame = data.get(..FRAME_LEN)?.try_into().ok()?;
    let current_raw = i16::from_be_bytes([raw[2], raw[3]]);
    // The vendor sample and the navigation_robot C driver both decode the
    // drive-reply position as an unsigned 16-bit value (range 0..=32767).
    // Clamp to `POS_MAX` so a corrupt reply with bit 15 set decodes to a
    // valid 0..=360° angle rather than the up-to-~720° an unclamped u16
    // would yield — telemetry stays inside its documented range even in the
    // default advisory-CRC mode, where a bad frame is passed through.
    let (temp_c, position_deg) = match kind {
        ReplyKind::Query => (Some(raw[6]), raw8_to_deg(raw[7])),
        ReplyKind::Drive => (
            None,
            raw_to_deg(u16::from_be_bytes([raw[6], raw[7]]).min(POS_MAX)),
        ),
    };
    Some(Feedback {
        id: raw[0],
        kind,
        mode: Mode::from_byte(raw[1]),
        mode_raw: raw[1],
        current_a: raw_to_amps(current_raw),
        speed_rpm: i16::from_be_bytes([raw[4], raw[5]]),
        temp_c,
        position_deg,
        faults: Faults(raw[8]),
        crc_ok: crc8_maxim(&raw[..9]) == raw[9],
        raw,
    })
}

/// Like [`parse_feedback`], but **rejects** a frame whose byte 9 does not
/// match its CRC-8/MAXIM: a decoded [`Feedback`] with `crc_ok == false`
/// becomes `None`.
///
/// This is the pure-function form of the bus's opt-in strict-CRC mode
/// ([`Bus::with_strict_crc`](crate::Bus::with_strict_crc) /
/// [`M0601::with_strict_crc`](crate::M0601::with_strict_crc)). The default
/// [`parse_feedback`] stays advisory — it returns the telemetry and leaves
/// the CRC verdict in [`Feedback::crc_ok`] for the caller to weigh — because
/// genuine replies from some firmware revisions have been seen to disagree on
/// the checksum. Reach for the strict form only where a corrupt frame is
/// worse than a dropped one, e.g. before feeding an odometry integrator.
///
/// ```
/// use m0601::protocol::{parse_feedback, parse_feedback_strict, ReplyKind};
/// // Byte 9 is deliberately wrong (a good CRC here is 0x00).
/// let bad = [0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0xFF];
/// assert!(parse_feedback(&bad, ReplyKind::Query).is_some_and(|fb| !fb.crc_ok));
/// assert!(parse_feedback_strict(&bad, ReplyKind::Query).is_none());
/// ```
pub fn parse_feedback_strict(data: &[u8], kind: ReplyKind) -> Option<Feedback> {
    parse_feedback(data, kind).filter(|fb| fb.crc_ok)
}

/// Encode a synthetic **query-layout** reply frame (the layout elicited by a
/// [`CMD_QUERY`] / `0x74` request): the encode-side inverse of
/// [`parse_feedback`] for [`ReplyKind::Query`].
///
/// The `0x74` names the *TX command* that selects this layout, not a byte in
/// the frame: byte 1 of the reply is the [`Mode`], never the command byte.
///
/// This is what a simulator or a test needs to stand in for a real motor —
/// building the ten telemetry bytes (with a correct CRC-8/MAXIM in byte 9) by
/// hand is error-prone, and [`Feedback`] is deliberately `#[non_exhaustive]`
/// so it cannot be constructed by struct literal. Build a reply frame here,
/// then feed it to [`parse_feedback`] or a
/// [`MockTransport`](crate::MockTransport) exactly as a real reply would flow.
///
/// `current_a` is quantised through [`amps_to_raw`] and `position_deg` through
/// [`deg_to_raw8`] (the coarse ~1.4° byte-7 resolution of a query reply), so a
/// round-trip back through [`parse_feedback`] returns those two within one
/// quantisation step; `id`, `mode`, `speed_rpm`, `temp_c` and `faults`
/// round-trip exactly. The resulting frame's [`Feedback::crc_ok`] is `true`.
///
/// ```
/// use m0601::protocol::{frame_query_reply, parse_feedback, ReplyKind};
/// use m0601::{Faults, Mode};
/// let frame = frame_query_reply(0x01, Mode::Velocity, 1.0, 100, 40, 180.0, Faults(0));
/// let fb = parse_feedback(&frame, ReplyKind::Query).unwrap();
/// assert_eq!(fb.id, 0x01);
/// assert_eq!(fb.mode, Some(Mode::Velocity));
/// assert_eq!(fb.speed_rpm, 100);
/// assert_eq!(fb.temp_c, Some(40));
/// assert!(fb.crc_ok);
/// assert!((fb.current_a - 1.0).abs() < 0.01);
/// assert!((fb.position_deg - 180.0).abs() < 1.5); // coarse 8-bit position
/// ```
pub fn frame_query_reply(
    id: u8,
    mode: Mode,
    current_a: f32,
    speed_rpm: i16,
    temp_c: u8,
    position_deg: f32,
    faults: Faults,
) -> Frame {
    let current = amps_to_raw(current_a).to_be_bytes();
    let speed = speed_rpm.to_be_bytes();
    reply_frame(
        id,
        mode,
        current,
        speed,
        [temp_c, deg_to_raw8(position_deg)],
        faults,
    )
}

/// Encode a synthetic **drive-layout** reply frame (the layout elicited by a
/// [`CMD_DRIVE`] / `0x64` frame or the broadcast ID query): the encode-side
/// inverse of [`parse_feedback`] for [`ReplyKind::Drive`].
///
/// As with [`frame_query_reply`], the `0x64` is the *TX command* that selects
/// the layout, not a byte in the reply — byte 1 is the [`Mode`].
///
/// The counterpart to [`frame_query_reply`] for the drive-reply layout —
/// bytes 6–7 hold a hi-res 16-bit position (~0.011°) and there is no
/// temperature. See [`frame_query_reply`] for why this exists and how it round-
/// trips; here `position_deg` goes through the finer [`deg_to_raw`], so it
/// returns from [`parse_feedback`] within ~0.011°.
///
/// ```
/// use m0601::protocol::{frame_drive_reply, parse_feedback, ReplyKind};
/// use m0601::{Faults, Mode};
/// let frame = frame_drive_reply(0x02, Mode::Velocity, -2.0, -50, 113.9, Faults(Faults::STALL));
/// let fb = parse_feedback(&frame, ReplyKind::Drive).unwrap();
/// assert_eq!(fb.id, 0x02);
/// assert_eq!(fb.speed_rpm, -50);
/// assert_eq!(fb.temp_c, None);          // drive replies carry no temperature
/// assert!(fb.faults.stall());
/// assert!(fb.crc_ok);
/// assert!((fb.current_a + 2.0).abs() < 0.01);
/// assert!((fb.position_deg - 113.9).abs() < 0.02);
/// ```
pub fn frame_drive_reply(
    id: u8,
    mode: Mode,
    current_a: f32,
    speed_rpm: i16,
    position_deg: f32,
    faults: Faults,
) -> Frame {
    let current = amps_to_raw(current_a).to_be_bytes();
    let speed = speed_rpm.to_be_bytes();
    reply_frame(
        id,
        mode,
        current,
        speed,
        deg_to_raw(position_deg).to_be_bytes(),
        faults,
    )
}

/// Assemble a reply frame from its already-encoded parts and seal it with a
/// correct CRC-8/MAXIM. Bytes 6–7 (`bytes_6_7`) are the one part that differs
/// between the two reply layouts.
fn reply_frame(
    id: u8,
    mode: Mode,
    current: [u8; 2],
    speed: [u8; 2],
    bytes_6_7: [u8; 2],
    faults: Faults,
) -> Frame {
    let mut frame: Frame = [0; FRAME_LEN];
    frame[0] = id;
    frame[1] = mode.as_byte();
    frame[2] = current[0];
    frame[3] = current[1];
    frame[4] = speed[0];
    frame[5] = speed[1];
    frame[6] = bytes_6_7[0];
    frame[7] = bytes_6_7[1];
    frame[8] = faults.0;
    frame[9] = crc8_maxim(&frame[..9]);
    frame
}

/// Strip a leading half-duplex TX echo from a raw reply.
///
/// Some RS485 adapters loop the host's own transmission back, so a reply can
/// arrive as `<tx frame><telemetry>`. An exact `tx` prefix is always an echo
/// (a genuine reply can never byte-equal the frame that elicited it — its
/// byte 1 is a mode value, not the command), so it is removed unconditionally.
/// A *partial* echo cannot be matched and passes through untouched — that
/// misaligned case is what [`frames`] rejects; see its docs for why that
/// matters more than it looks.
pub(crate) fn strip_echo<'a>(tx: &[u8], rx: &'a [u8]) -> &'a [u8] {
    rx.strip_prefix(tx).unwrap_or(rx)
}

/// Strip a leading half-duplex TX echo ([`strip_echo`]) and split what remains
/// into whole frames. Returns `None` unless that is a non-empty exact multiple
/// of [`FRAME_LEN`].
///
/// # Why the length must divide evenly
///
/// [`strip_echo`] is all-or-nothing: if the echo is short by even one byte it
/// is not recognised, and offset 0 is then no longer a frame boundary. Parsing
/// from there anyway yields a frame *straddling* the tail of the echo and the
/// head of the real reply — and that garbage is not obviously garbage. It looks
/// like telemetry, it passes the per-motor ID check (a truncated echo begins
/// with the addressed motor's own ID, exactly as a genuine reply does), and it
/// decodes to plausible values. Measured across every cut point, a wheel
/// turning at 300 RPM read back as 0, 1, 258 or 512 RPM — and for seven of the
/// nine cuts that is under the `< 10 RPM` guard callers rely on before entering
/// position mode, which is the one place a wrong speed reading is actively
/// dangerous.
///
/// A well-formed transaction is always a whole number of frames — the reply
/// alone, or the echo plus the reply — so anything else means the stream is
/// misaligned and none of it can be trusted. Rejecting on that costs at most
/// one dropped reading, which every caller already tolerates.
pub(crate) fn frames<'a>(tx: &[u8], rx: &'a [u8]) -> Option<std::slice::Iter<'a, Frame>> {
    let rx = strip_echo(tx, rx);
    let (whole, remainder) = rx.as_chunks::<FRAME_LEN>();
    if whole.is_empty() || !remainder.is_empty() {
        return None;
    }
    Some(whole.iter())
}
