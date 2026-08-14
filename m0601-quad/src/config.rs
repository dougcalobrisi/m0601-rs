//! `wheels.toml`: the durable, *verified* record of which motor sits at
//! which corner, plus bus timing and safety limits.
//!
//! Parsing is strict (`deny_unknown_fields`: a typo'd `invret = true`
//! would otherwise leave a wheel spinning backwards on a live rover) and
//! validation reports **every** error at once as a numbered list — fixing
//! a four-wheel config one error per run encourages guessing.
//!
//! None of these structs may use `#[serde(flatten)]`: serde documents it
//! as incompatible with `deny_unknown_fields`, which is the more valuable
//! of the two here.

use std::fmt;
use std::time::Duration;

use serde::Deserialize;

/// Chassis side. `driver`/`pass` are accepted as aliases (driver's side =
/// left).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    /// Driver's side.
    #[serde(alias = "driver")]
    Left,
    /// Passenger's side.
    #[serde(alias = "pass", alias = "passenger")]
    Right,
}

/// Chassis end.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum End {
    Front,
    #[serde(alias = "back")]
    Rear,
}

impl fmt::Display for Side {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Side::Left => "LEFT",
            Side::Right => "RIGHT",
        })
    }
}

impl fmt::Display for End {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            End::Front => "FRONT",
            End::Rear => "REAR",
        })
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusCfg {
    pub port: String,
    pub cycle_ms: f64,
    pub min_gap_ms: f64,
    pub reply_wait_ms: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Limits {
    pub max_rpm: i16,
    pub accel: u8,
    pub ramp_rpm_per_s: f64,
    pub current_trip_a: f64,
    pub current_trip_ms: f64,
    pub stale_ms: f64,
    pub dead_ms: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LogCfg {
    pub path: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WheelCfg {
    pub id: u8,
    pub name: String,
    pub side: Side,
    pub end: End,
    /// What you *observed* (found with `calibrate`): this wheel runs
    /// backwards from what its `mirrored` setting predicts.
    #[serde(default)]
    pub invert: bool,
    /// The SKU's mechanical build: FIT1042 (left) / FIT1038 (right) are
    /// mirror-image motors.
    #[serde(default)]
    pub mirrored: bool,
}

impl WheelCfg {
    /// Whether "+RPM = rover forward" needs a sign flip on this wheel.
    /// Fed straight to `M0601::mirrored`, which negates the setpoint
    /// outbound and flips reported speed/current signs inbound, so every
    /// consumer reads in the rover's frame. The two inputs are different
    /// kinds of fact — `mirrored` is the SKU, `invert` is what you
    /// observed on the bench — but for a velocity-mode app they are the
    /// same transform, so only their XOR matters.
    pub fn reversed(&self) -> bool {
        self.mirrored ^ self.invert
    }

    /// `"FRONT-LEFT"` — derived from `end`/`side`, never from `name`, so a
    /// label can never disagree with the corner it describes.
    pub fn corner(&self) -> String {
        format!("{}-{}", self.end, self.side)
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub bus: BusCfg,
    pub limits: Limits,
    pub log: Option<LogCfg>,
    #[serde(rename = "wheel")]
    pub wheels: Vec<WheelCfg>,
}

/// Everything wrong (fatal) and everything suspicious (non-fatal) about a
/// parsed config, all at once.
#[derive(Debug, Default)]
pub struct Report {
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

/// Milliseconds → `Duration`, rejecting the non-finite and negative values
/// TOML happily expresses (`nan`, `inf`, `-1`). `Duration::from_secs_f64`
/// would panic on exactly those — the hazard the workspace manifest
/// documents — so this is the only conversion path the app uses.
fn ms(value: f64) -> Option<Duration> {
    Duration::try_from_secs_f64(value / 1000.0).ok()
}

impl Config {
    /// Parse TOML text. A parse failure is unrecoverable (there is nothing
    /// to validate), so it returns immediately with toml's own message,
    /// which names unknown keys and bad types with line numbers.
    pub fn parse(text: &str) -> Result<Config, String> {
        toml::from_str(text).map_err(|e| e.to_string())
    }

    /// Load and fully validate; `Err` is the complete numbered error list.
    pub fn load(path: &str) -> Result<(Config, Vec<String>), Vec<String>> {
        let text =
            std::fs::read_to_string(path).map_err(|e| vec![format!("cannot read {path}: {e}")])?;
        let cfg = Config::parse(&text).map_err(|e| vec![format!("{path}: {e}")])?;
        let report = cfg.validate();
        if report.errors.is_empty() {
            Ok((cfg, report.warnings))
        } else {
            Err(report.errors)
        }
    }

    // Timing accessors. The `unwrap_or` fallbacks are **only** reachable for
    // non-finite/negative values, which `validate()` (below) rejects outright
    // — so on the sole shipped call path (`load` → `validate` → pilot) they
    // never fire, and a merely-wrong-but-finite value is used as written
    // rather than masked. Callers that `parse()` without `validate()` (some
    // unit tests) are the only ones that can observe a fallback; treat these
    // literals as a last-resort library default, not the operator's tuning.
    pub fn cycle(&self) -> Duration {
        ms(self.bus.cycle_ms).unwrap_or(Duration::from_millis(18))
    }
    pub fn min_gap(&self) -> Duration {
        ms(self.bus.min_gap_ms).unwrap_or(m0601::DEFAULT_MIN_GAP)
    }
    pub fn reply_wait(&self) -> Duration {
        ms(self.bus.reply_wait_ms).unwrap_or(Duration::from_millis(3))
    }
    pub fn current_trip(&self) -> Duration {
        ms(self.limits.current_trip_ms).unwrap_or(Duration::from_millis(400))
    }
    pub fn stale(&self) -> Duration {
        ms(self.limits.stale_ms).unwrap_or(Duration::from_millis(500))
    }
    pub fn dead(&self) -> Duration {
        ms(self.limits.dead_ms).unwrap_or(Duration::from_millis(1500))
    }

    /// Wheels in dashboard order: front-left, front-right, rear-left,
    /// rear-right. Valid only after `validate` passed (each corner once).
    pub fn wheels_in_grid_order(&self) -> Vec<&WheelCfg> {
        let mut ws: Vec<&WheelCfg> = self.wheels.iter().collect();
        ws.sort_by_key(|w| (matches!(w.end, End::Rear), matches!(w.side, Side::Right)));
        ws
    }

    /// Every check, every finding, one pass. Errors are fatal; warnings
    /// print with the `[!]` prefix and a pause before anything moves.
    pub fn validate(&self) -> Report {
        let mut r = Report::default();
        let e = &mut r.errors;

        // -- bus timing ---------------------------------------------------
        for (label, v) in [
            ("bus.cycle_ms", self.bus.cycle_ms),
            ("bus.min_gap_ms", self.bus.min_gap_ms),
            ("bus.reply_wait_ms", self.bus.reply_wait_ms),
            ("limits.ramp_rpm_per_s", self.limits.ramp_rpm_per_s),
            ("limits.current_trip_a", self.limits.current_trip_a),
            ("limits.current_trip_ms", self.limits.current_trip_ms),
            ("limits.stale_ms", self.limits.stale_ms),
            ("limits.dead_ms", self.limits.dead_ms),
        ] {
            // TOML 1.0 permits `nan` and `inf` literals; both would panic
            // a naive Duration conversion. Negatives parse but would
            // otherwise silently fall back to defaults in the accessors —
            // reject them here so a mistyped config cannot masquerade as
            // a tuned one.
            if !v.is_finite() || v < 0.0 {
                e.push(format!(
                    "{label} must be a finite, non-negative number (got {v})"
                ));
            }
        }

        if let Some(cycle) = ms(self.bus.cycle_ms) {
            let floor = m0601::drive_floor();
            if cycle > floor {
                e.push(format!(
                    "bus.cycle_ms = {} puts each wheel's drive interval over \
                     the 50 Hz floor ({floor:?}); the motors will coast every cycle",
                    self.bus.cycle_ms
                ));
            } else if cycle == floor {
                r.warnings.push(
                    "bus.cycle_ms = 20 has exactly zero timing margin; 18 is the tested value"
                        .into(),
                );
            }
            // The cycle must fit its bus occupancy: four drive frames plus
            // one poll, each frame trailed by the enforced idle gap and the
            // poll additionally holding the bus for its reply window. The
            // library owns that arithmetic (`m0601::bus_period`); `busy` is
            // the bus-imposed minimum period. No slack at all is an error;
            // thin slack a warning.
            if let (Some(gap), Some(wait)) = (ms(self.bus.min_gap_ms), ms(self.bus.reply_wait_ms)) {
                let busy = m0601::bus_period(4, 1, gap, wait);
                if busy >= cycle {
                    e.push(format!(
                        "bus timing does not fit: 4 drives + 1 poll occupy ~{busy:?} \
                         of the {cycle:?} cycle; shrink min_gap_ms/reply_wait_ms \
                         or grow cycle_ms"
                    ));
                } else if busy * 10 > cycle * 9 {
                    r.warnings.push(format!(
                        "bus timing is tight: ~{busy:?} of the {cycle:?} cycle is \
                         occupied (<10% slack for OS jitter)"
                    ));
                }
            }
        } else if self.bus.cycle_ms.is_finite() {
            e.push(format!(
                "bus.cycle_ms = {} is not a duration",
                self.bus.cycle_ms
            ));
        }
        if self.bus.cycle_ms == 0.0 {
            e.push("bus.cycle_ms must be positive".into());
        }

        // -- limits -------------------------------------------------------
        if !(1..=m0601::protocol::RPM_MAX).contains(&self.limits.max_rpm) {
            e.push(format!(
                "limits.max_rpm = {} outside 1..={}",
                self.limits.max_rpm,
                m0601::protocol::RPM_MAX
            ));
        }
        if self.limits.accel == 1 {
            r.warnings.push(
                "limits.accel = 1 is the motor's FASTEST ramp; a loaded launch can trip \
                 the 3 A bus-overcurrent protection. Use 5 unless measured otherwise"
                    .into(),
            );
        }
        if self.limits.ramp_rpm_per_s <= 0.0 {
            e.push(format!(
                "limits.ramp_rpm_per_s = {} must be positive",
                self.limits.ramp_rpm_per_s
            ));
        }
        if self.limits.current_trip_a <= 0.0 && self.limits.current_trip_a.is_finite() {
            e.push(format!(
                "limits.current_trip_a = {} must be positive",
                self.limits.current_trip_a
            ));
        }
        if let (Some(stale), Some(dead)) = (ms(self.limits.stale_ms), ms(self.limits.dead_ms))
            && stale >= dead
        {
            e.push(format!(
                "limits.stale_ms ({}) must be below limits.dead_ms ({})",
                self.limits.stale_ms, self.limits.dead_ms
            ));
        }

        // -- wheels -------------------------------------------------------
        if self.wheels.len() != 4 {
            e.push(format!(
                "expected exactly 4 [[wheel]] blocks, found {}",
                self.wheels.len()
            ));
        }
        for w in &self.wheels {
            if let Err(err) = m0601::protocol::validate_id(w.id) {
                e.push(format!("wheel \"{}\": {err}", w.name));
            }
        }
        for (i, a) in self.wheels.iter().enumerate() {
            for b in &self.wheels[i + 1..] {
                if a.id == b.id {
                    e.push(format!(
                        "wheels \"{}\" and \"{}\" share id 0x{:02X}",
                        a.name, b.name, a.id
                    ));
                }
                if a.side == b.side && a.end == b.end {
                    e.push(format!(
                        "wheels \"{}\" and \"{}\" both claim the {} corner",
                        a.name,
                        b.name,
                        a.corner()
                    ));
                }
            }
        }

        r
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The config this crate ships must always be loadable — it is the
    /// reference config people copy.
    const SHIPPED: &str = include_str!("../wheels.toml");

    fn parsed(text: &str) -> Config {
        Config::parse(text).expect("parses")
    }

    #[test]
    fn the_shipped_wheels_toml_is_valid() {
        let cfg = parsed(SHIPPED);
        let report = cfg.validate();
        assert!(report.errors.is_empty(), "{:#?}", report.errors);
        // The honest bus model (including the enforced gap after the poll
        // exchange) leaves the shipped 18 ms cycle only ~0.7 ms of true
        // slack, so the tight-timing warning is EXPECTED — and pinned
        // here so it cannot silently multiply or vanish. Removing it
        // means either measuring a smaller min_gap on hardware or giving
        // up coast margin against the 20 ms floor; see wheels.toml.
        assert_eq!(report.warnings.len(), 1, "{:#?}", report.warnings);
        assert!(
            report.warnings[0].contains("bus timing is tight"),
            "{:#?}",
            report.warnings
        );
        // And it records the physical map from the wheels.toml grid.
        let order: Vec<u8> = cfg.wheels_in_grid_order().iter().map(|w| w.id).collect();
        assert_eq!(order, [0x03, 0x04, 0x01, 0x02], "FL, FR, RL, RR");
    }

    #[test]
    fn unknown_keys_are_fatal_at_parse_time() {
        // The classic: `invret` instead of `invert` must not silently
        // leave a wheel spinning backwards.
        let text = SHIPPED.replace("invert = false", "invret = false");
        let err = Config::parse(&text).expect_err("must reject");
        assert!(err.contains("invret"), "error should name the key: {err}");
    }

    #[test]
    fn driver_and_pass_are_side_aliases() {
        let cfg = parsed(SHIPPED);
        assert_eq!(cfg.wheels[0].side, Side::Left, "driver = left");
        assert_eq!(cfg.wheels[1].side, Side::Right, "pass = right");
    }

    #[test]
    fn every_error_is_reported_in_one_pass() {
        // Break several things at once; each must appear.
        let text = SHIPPED
            .replace("max_rpm = 120", "max_rpm = 999")
            .replace("id = 0x04", "id = 0x03") // duplicate id
            .replace("cycle_ms = 18.0", "cycle_ms = 25.0");
        let report = parsed(&text).validate();
        assert!(report.errors.len() >= 3, "{:#?}", report.errors);
        assert!(report.errors.iter().any(|e| e.contains("max_rpm")));
        assert!(report.errors.iter().any(|e| e.contains("share id")));
        assert!(report.errors.iter().any(|e| e.contains("50 Hz")));
    }

    #[test]
    fn negative_timing_values_are_errors_not_silent_defaults() {
        // ms() maps a negative to None and the accessors fall back to
        // defaults — validation must catch it first, or a mistyped
        // config silently runs on numbers the operator never chose.
        let text = SHIPPED.replace("stale_ms = 500.0", "stale_ms = -1.0");
        let report = parsed(&text).validate();
        assert!(
            report
                .errors
                .iter()
                .any(|e| e.contains("stale_ms") && e.contains("non-negative")),
            "{:#?}",
            report.errors
        );
    }

    #[test]
    fn non_finite_floats_are_rejected_not_panicked_on() {
        let text = SHIPPED.replace("stale_ms = 500.0", "stale_ms = inf");
        let report = parsed(&text).validate();
        assert!(
            report.errors.iter().any(|e| e.contains("stale_ms")),
            "{:#?}",
            report.errors
        );
    }

    #[test]
    fn duplicate_corner_is_an_error_with_both_names() {
        let text = SHIPPED.replace(
            "name = \"rear driver\"\nside = \"driver\"\nend = \"rear\"",
            "name = \"rear driver\"\nside = \"driver\"\nend = \"front\"",
        );
        let report = parsed(&text).validate();
        assert!(
            report
                .errors
                .iter()
                .any(|e| e.contains("FRONT-LEFT") && e.contains("rear driver")),
            "{:#?}",
            report.errors
        );
    }

    #[test]
    fn a_cycle_below_the_bus_minimum_period_is_an_error_not_a_warning() {
        // With min_gap 2.0 / reply_wait 3.0 the bus needs ~17.4 ms per
        // cycle; a 16 ms cycle can never sustain its period and must be
        // refused, not waved through with a tightness warning.
        let text = SHIPPED.replace("cycle_ms = 18.0", "cycle_ms = 16.0");
        let report = parsed(&text).validate();
        assert!(
            report.errors.iter().any(|e| e.contains("does not fit")),
            "{:#?}",
            report.errors
        );
    }

    #[test]
    fn accel_1_warns_but_does_not_refuse() {
        let text = SHIPPED.replace("accel = 5", "accel = 1");
        let report = parsed(&text).validate();
        assert!(report.errors.is_empty());
        assert!(report.warnings.iter().any(|w| w.contains("accel")));
    }

    #[test]
    fn reversed_is_the_xor_of_mirrored_and_invert() {
        let mut w = WheelCfg {
            id: 1,
            name: "t".into(),
            side: Side::Left,
            end: End::Front,
            invert: false,
            mirrored: false,
        };
        assert!(!w.reversed());
        w.invert = true;
        assert!(w.reversed());
        w.mirrored = true;
        assert!(
            !w.reversed(),
            "both flipped cancels — the documented hazard"
        );
    }
}
