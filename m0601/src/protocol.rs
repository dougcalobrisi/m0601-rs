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

/// CRC-8/MAXIM (Dallas 1-Wire): polynomial x⁸+x⁵+x⁴+1, reflected (0x8C),
/// init 0.
///
/// Host→motor frames carry this over bytes 0–8 in byte 9 — except the
/// mode-switch ([`frame_mode`]) and set-ID ([`frame_set_id`]) frames, which
/// carry no CRC at all.
///
/// Motor replies carry it too, over the same bytes: a hardware capture
/// settled that question, and [`Feedback::crc_ok`] reports the result. It
/// stays informational — telemetry is never *rejected* on it — because the
/// reference implementations disagree and firmware revisions may differ.
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
/// `accel` sets how steeply the motor ramps toward the setpoint: `1` is the
/// *fastest* ramp, larger values ramp more gently, and `0` selects the
/// motor's own default. Only that direction is documented here — the vendor
/// sources state a unit for this byte ("1 RPM per 0.1 ms") whose sense
/// contradicts the ramp direction every source agrees on, and it has not
/// been resolved against hardware. See `PROTOCOL.md`.
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
    let (temp_c, position_deg) = match kind {
        ReplyKind::Query => (Some(raw[6]), raw8_to_deg(raw[7])),
        ReplyKind::Drive => (None, raw_to_deg(u16::from_be_bytes([raw[6], raw[7]]))),
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
