//! Data types: control [`Mode`], fault flags, and parsed [`Feedback`] telemetry.

use std::fmt;
use std::str::FromStr;

use crate::protocol::{Frame, ReplyKind};

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
/// 75 °C), stall after >5 s — and auto-resets each protection after ~5 s
/// (over-temperature releases on cooling instead). These bits report which
/// protection tripped.
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
    /// Overheat fault: winding over-temperature, 80 °C trip, released on
    /// cooling to 75 °C (`0x10`).
    ///
    /// The DFRobot wiki names this bit "Overheat fault"; MotorLink's label
    /// "Troubleshoot" is wrong — see `PROTOCOL.md`.
    pub const OVERHEAT: u8 = 0x10;

    const NAMES: [(u8, &'static str); 5] = [
        (Self::SENSOR_ERR, "SensorErr"),
        (Self::OVERCURRENT, "Overcurrent"),
        (Self::PHASE_OVERCURRENT, "PhaseOvercurrent"),
        (Self::STALL, "Stall"),
        (Self::OVERHEAT, "Overheat"),
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

    /// Overheat fault (80 °C trip / 75 °C release).
    pub fn overheat(self) -> bool {
        self.0 & Self::OVERHEAT != 0
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
    /// Which reply layout this frame was decoded with — determined by the
    /// command that elicited it, not by anything in the reply itself.
    pub kind: ReplyKind,
    /// Active control mode, if byte 1 held a known mode value.
    pub mode: Option<Mode>,
    /// Raw mode byte, for display when `mode` is `None`.
    pub mode_raw: u8,
    /// Torque current in amps (`i16` × 8 / 32767).
    pub current_a: f32,
    /// Signed wheel speed in RPM.
    pub speed_rpm: i16,
    /// Winding temperature in °C — `Some` only for
    /// [`ReplyKind::Query`] replies. Drive-frame and broadcast replies
    /// carry position in that byte instead, never a temperature.
    pub temp_c: Option<u8>,
    /// Wheel position in degrees. Resolution depends on [`kind`](Self::kind):
    /// ~1.4° for a `Query` reply (`u8` × 360 / 255), ~0.011° for a `Drive`
    /// reply (`u16` × 360 / 32767).
    pub position_deg: f32,
    /// Fault bitmask (byte 8).
    pub faults: Faults,
    /// Whether byte 9 matches a CRC-8/MAXIM over bytes 0-8.
    ///
    /// Genuine replies carry that CRC (verified against real hardware —
    /// see `PROTOCOL.md`), so this is normally `true`. It is still
    /// **informational only**: telemetry is never rejected on it, since
    /// not all reference implementations agree and firmware revisions may
    /// differ.
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

/// Latest telemetry, plus the readings that only one reply layout carries
/// and must be retained across the replies that don't.
///
/// This is protocol knowledge, not presentation: the winding temperature
/// arrives only in query (`0x74`) replies, and the hi-res 16-bit angle only
/// in drive replies ([`ReplyKind`] selects the layout). Any loop that mixes
/// the two — the canonical shape is a drive frame every cycle and a query
/// every Nth for temperature — would otherwise watch each reading flicker
/// to `None` (or to the coarse 8-bit angle) as `fb` alternates between
/// layouts. Feed every parsed reply to [`absorb`](Self::absorb) and read
/// the retained fields instead.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Telemetry {
    /// Most recent reply of either kind — the source of mode, speed,
    /// current and faults, which decode identically in both layouts.
    pub fb: Option<Feedback>,
    /// Winding temperature from the most recent query (`0x74`) reply.
    pub temp_c: Option<u8>,
    /// Wheel angle from the most recent *drive* reply (hi-res 16-bit).
    /// Held apart from `fb` so a query reply's coarse 8-bit angle doesn't
    /// make a displayed position flicker between resolutions.
    pub position_deg: Option<f32>,
}

impl Telemetry {
    /// Store `fb` as latest, and separately retain the readings only one
    /// layout carries: temperature (query replies) and the hi-res angle
    /// (drive replies).
    pub fn absorb(&mut self, fb: Feedback) {
        if let Some(t) = fb.temp_c {
            self.temp_c = Some(t);
        }
        if fb.kind == ReplyKind::Drive {
            self.position_deg = Some(fb.position_deg);
        }
        self.fb = Some(fb);
    }
}

#[cfg(test)]
mod tests {
    use super::Telemetry;
    use crate::Feedback;
    use crate::protocol::{ReplyKind, parse_feedback};

    /// The same reply bytes decoded as either kind (temp 40 °C as a query).
    fn fb(kind: ReplyKind) -> Feedback {
        parse_feedback(&[0x01, 0x02, 0, 0, 0, 0x64, 0x28, 0x80, 0, 0], kind).expect("valid frame")
    }

    #[test]
    fn absorb_retains_temperature_across_drive_replies() {
        let mut t = Telemetry::default();
        t.absorb(fb(ReplyKind::Drive));
        assert_eq!(t.temp_c, None, "no query reply seen yet");
        t.absorb(fb(ReplyKind::Query));
        assert_eq!(t.temp_c, Some(40));
        t.absorb(fb(ReplyKind::Drive));
        assert_eq!(t.temp_c, Some(40), "a drive reply must not clear it");
        // fb always tracks the latest reply of either kind.
        assert_eq!(t.fb.map(|fb| fb.kind), Some(ReplyKind::Drive));
    }

    #[test]
    fn absorb_keeps_hi_res_drive_angle_across_a_query_reply() {
        let mut t = Telemetry::default();
        // A query reply before any drive reply: no hi-res angle retained yet
        // (a UI falls back to the reply's own coarse position meanwhile).
        t.absorb(fb(ReplyKind::Query));
        assert_eq!(t.position_deg, None, "no hi-res drive reply seen yet");
        // A drive reply establishes the hi-res angle...
        t.absorb(fb(ReplyKind::Drive));
        let hi_res = t.position_deg.expect("drive reply sets the hi-res angle");
        // ...and a later query reply must NOT overwrite it with its coarse
        // 8-bit angle — that flicker is exactly what this field prevents.
        t.absorb(fb(ReplyKind::Query));
        assert_eq!(
            t.position_deg,
            Some(hi_res),
            "a query reply must not downgrade the retained hi-res angle"
        );
    }
}
