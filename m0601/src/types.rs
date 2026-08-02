//! Data types: control [`Mode`], fault flags, and parsed [`Feedback`] telemetry.

use std::fmt;
use std::str::FromStr;

use crate::protocol::Frame;

/// The three closed-loop control modes of the M0601.
///
/// The active mode determines how the 16-bit value in a drive (`0x64`) frame
/// is interpreted. Switch modes with
/// [`M0601::set_mode`](crate::M0601::set_mode); the mode-switch frame must be
/// (and is) sent five times.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Mode {
    /// Current loop: `-32767..=32767` maps to roughly −8 A..+8 A.
    Current = 0x01,
    /// Velocity loop: `-330..=330` RPM.
    Velocity = 0x02,
    /// Position loop: `0..=32767` maps to 0°..360°.
    ///
    /// The motor must be turning slower than 10 RPM before switching into
    /// this mode.
    Position = 0x03,
}

impl Mode {
    /// Decode a mode byte from a feedback frame. Returns `None` for unknown
    /// values.
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x01 => Some(Mode::Current),
            0x02 => Some(Mode::Velocity),
            0x03 => Some(Mode::Position),
            _ => None,
        }
    }

    /// The wire value of this mode (`0x01`/`0x02`/`0x03`).
    pub fn as_byte(self) -> u8 {
        self as u8
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Mode::Current => "Current",
            Mode::Velocity => "Velocity",
            Mode::Position => "Position",
        })
    }
}

impl FromStr for Mode {
    type Err = String;

    /// Parses `"current"`, `"velocity"` or `"position"` (case-insensitive).
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.to_ascii_lowercase().as_str() {
            "current" => Ok(Mode::Current),
            "velocity" => Ok(Mode::Velocity),
            "position" => Ok(Mode::Position),
            _ => Err(format!("unknown mode {s:?} (current|velocity|position)")),
        }
    }
}

/// Fault bitmask from byte 8 of a feedback frame.
///
/// The motor protects itself in hardware — bus overcurrent at 3 A, phase
/// overcurrent at 4.6 A, winding over-temperature at 80 °C (released at
/// 75 °C), stall after >5 s — and auto-resets each protection after ~5 s.
/// These bits report which protection tripped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct Faults(pub u8);

impl Faults {
    /// Hall/encoder sensor error (`0x01`).
    pub const SENSOR_ERR: u8 = 0x01;
    /// Bus overcurrent, 3 A threshold (`0x02`).
    pub const OVERCURRENT: u8 = 0x02;
    /// Phase overcurrent, 4.6 A threshold (`0x04`).
    pub const PHASE_OVERCURRENT: u8 = 0x04;
    /// Stall protection, >5 s locked (`0x08`).
    pub const STALL: u8 = 0x08;
    /// General troubleshoot flag (`0x10`).
    pub const TROUBLESHOOT: u8 = 0x10;

    const NAMES: [(u8, &'static str); 5] = [
        (Self::SENSOR_ERR, "SensorErr"),
        (Self::OVERCURRENT, "Overcurrent"),
        (Self::PHASE_OVERCURRENT, "PhaseOvercurrent"),
        (Self::STALL, "Stall"),
        (Self::TROUBLESHOOT, "Troubleshoot"),
    ];

    /// `true` when no fault bit is set.
    pub fn is_ok(self) -> bool {
        self.0 == 0
    }

    /// Hall/encoder sensor error.
    pub fn sensor_err(self) -> bool {
        self.0 & Self::SENSOR_ERR != 0
    }

    /// Bus overcurrent (3 A threshold).
    pub fn overcurrent(self) -> bool {
        self.0 & Self::OVERCURRENT != 0
    }

    /// Phase overcurrent (4.6 A threshold).
    pub fn phase_overcurrent(self) -> bool {
        self.0 & Self::PHASE_OVERCURRENT != 0
    }

    /// Stall protection tripped (locked >5 s).
    pub fn stall(self) -> bool {
        self.0 & Self::STALL != 0
    }

    /// General troubleshoot flag.
    pub fn troubleshoot(self) -> bool {
        self.0 & Self::TROUBLESHOOT != 0
    }
}

/// `"OK"`, or the known fault names joined with `" | "`. Any bits outside
/// the documented set are appended as hex so nothing the motor reports is
/// ever silently dropped.
///
/// ```
/// use m0601::Faults;
/// assert_eq!(Faults(0x00).to_string(), "OK");
/// assert_eq!(Faults(0x03).to_string(), "SensorErr | Overcurrent");
/// assert_eq!(Faults(0x20).to_string(), "0x20");
/// // A known bit alongside an unknown one reports both.
/// assert_eq!(Faults(0x21).to_string(), "SensorErr | 0x20");
/// ```
impl fmt::Display for Faults {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_ok() {
            return f.write_str("OK");
        }
        let mut any = false;
        for (bit, name) in Self::NAMES {
            if self.0 & bit != 0 {
                if any {
                    f.write_str(" | ")?;
                }
                f.write_str(name)?;
                any = true;
            }
        }
        // Report leftover bits rather than hiding them behind a known name.
        let known = Self::NAMES.iter().fold(0u8, |acc, (bit, _)| acc | bit);
        let unknown = self.0 & !known;
        if unknown != 0 {
            if any {
                f.write_str(" | ")?;
            }
            write!(f, "0x{unknown:02X}")?;
        }
        Ok(())
    }
}

/// Parsed telemetry from a 10-byte feedback frame.
///
/// Produced by [`protocol::parse_feedback`](crate::protocol::parse_feedback)
/// and [`M0601::query`](crate::M0601::query). Values are stored at full
/// precision; round only when displaying (the CLI uses `{:+.3}` A and
/// `{:.1}`°).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feedback {
    /// Responding motor's RS485 ID.
    pub id: u8,
    /// Active control mode, if byte 1 held a known mode value.
    pub mode: Option<Mode>,
    /// Raw mode byte, for display when `mode` is `None`.
    pub mode_raw: u8,
    /// Torque current in amps (`i16` × 8 / 32767).
    pub current_a: f32,
    /// Signed wheel speed in RPM.
    pub speed_rpm: i16,
    /// Winding temperature in °C.
    pub temp_c: u8,
    /// Wheel position in degrees (`u8` × 360 / 255).
    pub position_deg: f32,
    /// Fault bitmask (byte 8).
    pub faults: Faults,
    /// Whether byte 9 matches a CRC-8/MAXIM over bytes 0..9.
    ///
    /// **Informational only.** The motor's replies do *not* carry a
    /// CRC-8/MAXIM there (byte 9 is some other checksum), so this is
    /// normally `false` for genuine frames. Never reject telemetry on it.
    pub crc_ok: bool,
    /// The raw 10-byte frame the telemetry was parsed from.
    pub raw: Frame,
}

impl Feedback {
    /// The raw frame as uppercase space-separated hex,
    /// e.g. `"01 02 F8 30 00 64 28 80 03 00"`.
    pub fn raw_hex(&self) -> String {
        let mut s = String::with_capacity(self.raw.len() * 3 - 1);
        for (i, b) in self.raw.iter().enumerate() {
            if i > 0 {
                s.push(' ');
            }
            let _ = fmt::Write::write_fmt(&mut s, format_args!("{b:02X}"));
        }
        s
    }

    /// The mode name (`"Velocity"`), or the raw byte as hex (`"0x07"`) when
    /// the mode byte is unknown.
    pub fn mode_name(&self) -> String {
        match self.mode {
            Some(m) => m.to_string(),
            None => format!("0x{:02X}", self.mode_raw),
        }
    }
}
