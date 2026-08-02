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
/// Host→motor frames carry this over bytes 0..9 in byte 9 — except the
/// mode-switch ([`frame_mode`]) and set-ID ([`frame_set_id`]) frames, which
/// carry no CRC at all. Motor replies do **not** use this CRC either.
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
/// `accel` is the acceleration time in units of 1 RPM per 0.1 ms; `0` selects
/// the motor's default (minimum effective value is 1).
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

/// Parse a feedback frame from raw reply bytes.
///
/// Returns `None` when fewer than [`FRAME_LEN`] bytes are supplied; longer
/// input parses its first 10 bytes. Frames are validated by *length only* —
/// the motor's replies do not carry a CRC-8/MAXIM in byte 9 (it is some other
/// checksum), so [`Feedback::crc_ok`] is informational and this function
/// never rejects on it.
///
/// Reply layout: `[id, mode, current_i16_be, speed_i16_be, temp_u8, pos_u8,
/// faults, chk]` with current scaled ×8/32767 A and position ×360/255°.
///
/// Position uses ×360/**255**, so byte 7 = `0xFF` reads as a full 360°
/// (i.e. 0°, wrapped). If your unit turns out to encode a revolution as 256
/// steps rather than 255, every reading here is stretched by ~0.4 % —
/// worth confirming against a physically indexed wheel before relying on
/// [`Feedback::position_deg`] for anything precise.
///
/// ```
/// use m0601::protocol::parse_feedback;
/// let fb = parse_feedback(&[0x01, 0x02, 0xF8, 0x30, 0x00, 0x64, 0x28, 0x80, 0x03, 0x00])
///     .unwrap();
/// assert_eq!(fb.speed_rpm, 100);
/// assert_eq!(fb.temp_c, 40);
/// assert_eq!(fb.faults.to_string(), "SensorErr | Overcurrent");
/// assert!(!fb.crc_ok);
/// ```
pub fn parse_feedback(data: &[u8]) -> Option<Feedback> {
    let raw: Frame = data.get(..FRAME_LEN)?.try_into().ok()?;
    let current_raw = i16::from_be_bytes([raw[2], raw[3]]);
    Some(Feedback {
        id: raw[0],
        mode: Mode::from_byte(raw[1]),
        mode_raw: raw[1],
        current_a: f32::from(current_raw) * 8.0 / 32767.0,
        speed_rpm: i16::from_be_bytes([raw[4], raw[5]]),
        temp_c: raw[6],
        position_deg: f32::from(raw[7]) * 360.0 / 255.0,
        faults: Faults(raw[8]),
        crc_ok: crc8_maxim(&raw[..9]) == raw[9],
        raw,
    })
}
