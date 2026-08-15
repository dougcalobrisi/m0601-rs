//! Data types: control [`Mode`], fault flags, and parsed [`Feedback`] telemetry.

use std::fmt;
use std::str::FromStr;

use crate::protocol::{Frame, ReplyKind, raw_to_deg};

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

    /// Union of every fault bit this driver defines (`0x1F`): the five
    /// protections above. The remaining bits of byte 8 are unassigned by the
    /// protocol as the driver understands it.
    ///
    /// The single source of truth for "which bits are known" — used by
    /// [`unknown_bits`](Self::unknown_bits) and [`Display`](fmt::Display).
    /// A consumer that classifies faults should mask against this rather than
    /// hardcode `0x1F`, so a future bit added here updates every caller.
    pub const KNOWN_MASK: u8 = Self::SENSOR_ERR
        | Self::OVERCURRENT
        | Self::PHASE_OVERCURRENT
        | Self::STALL
        | Self::OVERHEAT;

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

    /// Any set bits outside [`KNOWN_MASK`](Self::KNOWN_MASK) — a fault the
    /// motor reported that this driver has no name for. `0` when every set bit
    /// is a documented protection.
    ///
    /// Lets a consumer surface an unrecognised fault (e.g. from a newer
    /// firmware) without keeping its own copy of the known-bit set.
    pub fn unknown_bits(self) -> u8 {
        self.0 & !Self::KNOWN_MASK
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
        let unknown = self.unknown_bits();
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
/// precision; round only when displaying (e.g. `{:+.3}` A and `{:.1}`°).
#[derive(Debug, Clone, Copy, PartialEq)]
// Returned by the driver, never constructed by callers: `#[non_exhaustive]`
// keeps room to surface a new wire field (the protocol has unused frame bytes)
// without it being a breaking change. `raw` is the escape hatch until then.
#[non_exhaustive]
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
// Built by `absorb`ing replies, not by callers; `#[non_exhaustive]` leaves room
// to retain another field later. `Default` still constructs it in-crate.
#[non_exhaustive]
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

/// Unwraps the M0601's single-turn angle into a continuous multi-turn one.
///
/// The motor reports position as an absolute angle within one revolution —
/// `0..=32767` raw = 0°..360° in a [`ReplyKind::Drive`] reply, or the coarse
/// 8-bit angle in a query reply — and that reading is *clamped, never
/// wrapped*: at the top of a turn it snaps back to 0° for the next. An
/// odometry integrator needs the opposite: an angle that keeps counting up
/// (or down) across revolution boundaries, so that "the wheel turned 3.5
/// times forward" reads as `+1260°`, not as a 90° that is indistinguishable
/// from a wheel that barely moved.
///
/// Feed each periodic sample to [`update`](Self::update) (degrees) or
/// [`update_raw`](Self::update_raw) (a 16-bit reading, converted with the
/// same scale as [`raw_to_deg`]) and read back
/// the running total. The unwrap is done on the **shortest arc**: each new
/// sample is compared against the previous one and the difference is folded
/// into `(-180°, +180°]` before being added to the accumulated angle, so a
/// `359° → 1°` step counts as `+2°` (the wheel rolled forward across the
/// seam) and a `1° → 359°` step as `−2°` (it rolled back).
///
/// # Validity bound — samples must be closer than 180° apart
///
/// The shortest-arc rule cannot tell a small forward step from a large
/// backward one: a *true* motion of more than 180° between two samples
/// aliases to the short way round and is integrated with the wrong sign and
/// magnitude. Poll often enough that the wheel cannot turn half a revolution
/// between samples: at the M0601's 330 RPM ceiling (1980°/s) that means a
/// per-wheel poll no slower than ~11 Hz; a wheel that never exceeds a lower
/// speed tolerates a proportionally slower poll. Given a poll interval,
/// [`max_unaliased_rpm`](Self::max_unaliased_rpm) returns the exact speed
/// ceiling the shortest-arc unwrap stays valid up to.
///
/// The type is pure (no I/O) and cheap to copy, so a control loop can keep
/// one per wheel.
///
/// ```
/// use m0601::PositionAccumulator;
/// let mut acc = PositionAccumulator::new();
/// assert_eq!(acc.update(350.0), 0.0);   // first sample: the reference, 0°
/// assert_eq!(acc.update(10.0), 20.0);   // +20° across the 360°/0° seam
/// assert_eq!(acc.update(350.0), 0.0);   // −20° back the short way
/// ```
#[derive(Debug, Clone, Default)]
pub struct PositionAccumulator {
    /// The previous absolute sample in degrees; `None` until the first
    /// [`update`](Self::update) establishes the reference.
    last: Option<f32>,
    /// Continuous accumulated angle in degrees. Kept in `f64` so long runs
    /// don't quantize: at ~100k° an `f32` has only ~0.008° of resolution and
    /// small deltas start rounding visibly, while `f64` stays exact to well
    /// past any realistic mission length.
    cumulative: f64,
}

impl PositionAccumulator {
    /// A fresh accumulator with no reference sample yet. The first sample fed
    /// to it becomes the zero reference and yields `0.0` cumulative.
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one absolute single-turn sample (degrees) and return the updated
    /// continuous angle.
    ///
    /// The **first** sample establishes the reference and returns `0.0` — the
    /// accumulator measures motion *relative to where it started*, not an
    /// absolute heading. Every later sample adds the shortest-arc delta from
    /// the previous sample (folded into `(-180°, +180°]`) to the running
    /// total; see the type docs for the 180° validity bound.
    ///
    /// A non-finite sample (`NaN`/`±∞`) is ignored: the reference and the
    /// accumulated angle are left untouched and the current total is
    /// returned, so one bad reading cannot corrupt the integration or panic.
    pub fn update(&mut self, sample_deg: f32) -> f64 {
        if !sample_deg.is_finite() {
            return self.cumulative;
        }
        match self.last {
            None => {
                self.last = Some(sample_deg);
            }
            Some(prev) => {
                // Fold the raw difference into (-180, 180]: `rem_euclid`
                // lands it in [0, 360), then anything past 180 is the same
                // motion taken the short way round the other direction.
                let mut delta = (f64::from(sample_deg) - f64::from(prev)).rem_euclid(360.0);
                if delta > 180.0 {
                    delta -= 360.0;
                }
                self.cumulative += delta;
                self.last = Some(sample_deg);
            }
        }
        self.cumulative
    }

    /// Feed one raw 16-bit position reading (`0..=32767` = 0°..360°, the
    /// [`ReplyKind::Drive`] layout) and return the updated continuous angle.
    ///
    /// Convenience over [`update`](Self::update) using the same scale as
    /// [`raw_to_deg`].
    pub fn update_raw(&mut self, raw: u16) -> f64 {
        self.update(raw_to_deg(raw))
    }

    /// The continuous accumulated angle in degrees (`0.0` before the first
    /// sample). Positive is the direction the first motion went.
    pub fn cumulative_deg(&self) -> f64 {
        self.cumulative
    }

    /// The accumulated angle expressed in whole and fractional revolutions
    /// (`cumulative_deg() / 360`).
    pub fn revolutions(&self) -> f64 {
        self.cumulative / 360.0
    }

    /// Forget the reference and zero the accumulated angle, as if newly
    /// [`new`](Self::new)-constructed. The next sample re-establishes the
    /// reference and returns `0.0`.
    pub fn reset(&mut self) {
        self.last = None;
        self.cumulative = 0.0;
    }

    /// The fastest wheel speed (RPM) whose motion the shortest-arc unwrap can
    /// still resolve when samples are `gap` apart — i.e. the speed at which the
    /// wheel travels exactly 180° per `gap`. Above it, a sample steps more than
    /// half a turn and [`update`](Self::update) aliases (see the validity
    /// bound in the type docs).
    ///
    /// This is the derivation of that bound made executable: 180° per `gap` is
    /// half a revolution, so the ceiling is `30 / gap_secs` RPM. A control loop
    /// polling at a fixed cadence can compare this against its own speed limit
    /// to know whether its odometry can alias, or re-baseline the accumulator
    /// when a poll gap grows past the corresponding interval. Returns `+∞` for
    /// a zero `gap`.
    ///
    /// ```
    /// use std::time::Duration;
    /// use m0601::PositionAccumulator;
    /// // A 20 ms poll resolves up to 1500 RPM — far above the motor's ceiling.
    /// let ceiling = PositionAccumulator::max_unaliased_rpm(Duration::from_millis(20));
    /// assert!((ceiling - 1500.0).abs() < 1e-9);
    /// ```
    pub fn max_unaliased_rpm(gap: std::time::Duration) -> f64 {
        let secs = gap.as_secs_f64();
        if secs <= 0.0 {
            f64::INFINITY
        } else {
            30.0 / secs
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{PositionAccumulator, Telemetry};
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

    #[test]
    fn first_sample_is_the_zero_reference() {
        let mut acc = PositionAccumulator::new();
        assert_eq!(acc.update(123.5), 0.0, "first sample yields 0 cumulative");
        assert_eq!(acc.cumulative_deg(), 0.0);
        assert_eq!(acc.revolutions(), 0.0);
    }

    #[test]
    fn monotonic_forward_accumulates_past_a_full_turn() {
        let mut acc = PositionAccumulator::new();
        acc.update(0.0);
        // Walk forward in 90° steps through more than one revolution.
        for deg in [90.0, 180.0, 270.0, 0.0, 90.0] {
            acc.update(deg);
        }
        // Five +90° steps = 450°, i.e. one and a quarter turns forward.
        assert!((acc.cumulative_deg() - 450.0).abs() < 1e-3);
        assert!(acc.cumulative_deg() > 360.0);
        assert!((acc.revolutions() - 1.25).abs() < 1e-4);
    }

    #[test]
    fn reverse_past_zero_goes_negative() {
        let mut acc = PositionAccumulator::new();
        acc.update(0.0);
        // Walk backward across the 0°/360° seam.
        for deg in [270.0, 180.0, 90.0, 0.0, 270.0] {
            acc.update(deg);
        }
        // Five −90° steps = −450°.
        assert!((acc.cumulative_deg() + 450.0).abs() < 1e-3);
        assert!(acc.cumulative_deg() < 0.0);
        assert!((acc.revolutions() + 1.25).abs() < 1e-4);
    }

    #[test]
    fn seam_crossing_takes_the_short_arc() {
        // 359° → 1° is +2° forward, not −358°.
        let mut acc = PositionAccumulator::new();
        acc.update(359.0);
        assert!((acc.update(1.0) - 2.0).abs() < 1e-3);

        // 1° → 359° is −2° back, not +358°.
        let mut acc = PositionAccumulator::new();
        acc.update(1.0);
        assert!((acc.update(359.0) + 2.0).abs() < 1e-3);
    }

    #[test]
    fn reset_clears_reference_and_total() {
        let mut acc = PositionAccumulator::new();
        acc.update(10.0);
        acc.update(100.0);
        assert!(acc.cumulative_deg() > 0.0);
        acc.reset();
        assert_eq!(acc.cumulative_deg(), 0.0);
        // Next sample re-establishes the reference and yields 0 again.
        assert_eq!(acc.update(200.0), 0.0);
        assert_eq!(acc.update(210.0), 10.0);
    }

    #[test]
    fn non_finite_sample_is_ignored() {
        let mut acc = PositionAccumulator::new();
        acc.update(10.0);
        acc.update(40.0); // +30
        let before = acc.cumulative_deg();
        // NaN and infinities leave the state untouched and never panic.
        assert_eq!(acc.update(f32::NAN), before);
        assert_eq!(acc.update(f32::INFINITY), before);
        assert_eq!(acc.update(f32::NEG_INFINITY), before);
        assert_eq!(acc.cumulative_deg(), before);
        // A good sample after the bad ones still integrates from the last
        // valid reference (40°), not from the discarded garbage.
        assert!((acc.update(70.0) - (before + 30.0)).abs() < 1e-3);
    }

    #[test]
    fn update_raw_matches_the_drive_reply_scale() {
        // Raw 0 → 8192 is a quarter turn (~90°), safely inside the 180° fold
        // bound; update_raw and the raw_to_deg-then-update path must agree.
        let mut acc = PositionAccumulator::new();
        assert_eq!(acc.update_raw(0), 0.0);
        let via_raw = acc.update_raw(8_192);

        let mut acc2 = PositionAccumulator::new();
        acc2.update(crate::protocol::raw_to_deg(0));
        let via_deg = acc2.update(crate::protocol::raw_to_deg(8_192));
        assert_eq!(via_raw, via_deg);
        assert!((via_raw - 90.0).abs() < 0.01);
    }

    #[test]
    fn known_mask_is_the_union_of_the_named_bits() {
        use super::Faults;
        let named = Faults::NAMES.iter().fold(0u8, |acc, (bit, _)| acc | bit);
        assert_eq!(
            Faults::KNOWN_MASK,
            named,
            "KNOWN_MASK must cover exactly the named bits"
        );
        assert_eq!(Faults::KNOWN_MASK, 0x1F);
    }

    #[test]
    fn unknown_bits_reports_only_undefined_bits() {
        use super::Faults;
        assert_eq!(Faults(0x00).unknown_bits(), 0x00);
        assert_eq!(
            Faults(0x1F).unknown_bits(),
            0x00,
            "every named bit is known"
        );
        assert_eq!(Faults(0x20).unknown_bits(), 0x20);
        assert_eq!(
            Faults(0x21).unknown_bits(),
            0x20,
            "a known bit alongside an unknown one"
        );
    }

    #[test]
    fn max_unaliased_rpm_is_the_180_deg_per_gap_ceiling() {
        use std::time::Duration;
        // 180° per gap = half a rev; at 100 ms that is 5 rev/s = 300 RPM.
        let ceiling = PositionAccumulator::max_unaliased_rpm(Duration::from_millis(100));
        assert!((ceiling - 300.0).abs() < 1e-9);
        // A zero gap can never alias, so the ceiling is unbounded.
        assert_eq!(
            PositionAccumulator::max_unaliased_rpm(Duration::ZERO),
            f64::INFINITY
        );
    }
}
