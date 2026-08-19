//! `m0601` — control tool for the DFRobot M0601 (FIT1042) hub motor over
//! RS485.

mod cmd;

use std::process::ExitCode;
use std::time::Duration;

use clap::{Parser, Subcommand};
use m0601::protocol::{RPM_MAX, RPM_MIN};

/// Fallback if the validated timeout somehow will not convert (it will).
const DEFAULT_TIMEOUT: Duration = Duration::from_millis(150);

#[derive(Parser)]
#[command(
    name = "m0601",
    version,
    about = "DFRobot M0601 hub motor control tool (RS485)."
)]
struct Cli {
    /// Serial port
    #[arg(long, global = true, default_value = "/dev/ttyUSB0")]
    port: String,

    /// Motor ID 0x01..0xFE, e.g. 0x01 (ignored by `scan`, which probes all)
    #[arg(long, global = true, default_value = "0x01", value_parser = parse_id)]
    id: u8,

    /// Serial read timeout in seconds (reply window for one-shot reads;
    /// the 50 Hz drive/control loops use their own fixed 6 ms wait)
    #[arg(long, global = true, default_value_t = 0.15, value_parser = parse_timeout)]
    timeout: f64,

    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Discover motor IDs on the bus
    Scan {
        /// Poll every ID 0x01..0xFE (~40s) instead of the default 0x01..0x0F
        #[arg(long)]
        full: bool,
    },
    /// View config + one-shot live readout
    Info,
    /// Headless live dashboard, optional CSV logging
    Monitor {
        /// Poll rate
        #[arg(long, default_value_t = 5.0, value_parser = parse_hz)]
        hz: f64,
        /// Also log readings to CSV
        #[arg(long, value_name = "FILE")]
        csv: Option<String>,
    },
    /// Full-screen dashboard with keyboard control
    Control {
        /// Preset speed for F/B keys, -330..=330
        #[arg(long, default_value_t = 100, value_parser = parse_rpm, allow_hyphen_values = true)]
        rpm: i16,
        /// Velocity ramp byte; 0 = the motor's own default (the default here).
        /// Which end of the range softens the ramp is undocumented and
        /// unmeasured, so sweep it on your own wheel before relying on it.
        #[arg(long, default_value_t = cmd::control::CONTROL_DEFAULT_ACCEL)]
        accel: u8,
    },
    /// Drive one mode at a fixed setpoint (scriptable; Ctrl-C or --secs stops)
    Drive {
        #[command(subcommand)]
        mode: DriveMode,
    },
    /// Change a motor's RS485 ID (persistent, one motor only)
    SetId {
        /// New ID 0x01..0xFE
        #[arg(long, value_parser = parse_id)]
        new: u8,
        /// Skip confirmation prompt
        #[arg(long)]
        yes: bool,
    },
    /// Send an arbitrary frame (9 bytes = CRC auto-added, or 10)
    Raw {
        /// Hex bytes, e.g. "01 74 00 00 00 00 00 00 00"
        hex: String,
        /// Required to send a drive (0x64) or mode-switch (0xA0) frame — the
        /// two commands that can move the motor.
        #[arg(long)]
        yes: bool,
    },
}

/// The three motor modes, each with its own natural units. `--secs` bounds
/// the run; omit it to drive until Ctrl-C.
#[derive(Subcommand)]
enum DriveMode {
    /// Velocity loop: hold an RPM setpoint
    Velocity {
        /// Target speed, -330..=330 RPM
        #[arg(long, value_parser = parse_rpm, allow_hyphen_values = true)]
        rpm: i16,
        /// Acceleration byte; 0 = motor default. Ramp direction undocumented
        #[arg(long, default_value_t = 1)]
        accel: u8,
        /// Stop after this many seconds (default: until Ctrl-C)
        #[arg(long, value_parser = parse_seconds)]
        secs: Option<f64>,
    },
    /// Current loop: hold a torque-current setpoint
    Current {
        /// Target current, about -8.0..=8.0 A
        #[arg(long, value_parser = parse_amps, allow_hyphen_values = true)]
        amps: f32,
        /// Stop after this many seconds (default: until Ctrl-C)
        #[arg(long, value_parser = parse_seconds)]
        secs: Option<f64>,
    },
    /// Position loop: rotate to an absolute angle and hold it (needs <10 RPM)
    Position {
        /// Target angle, 0.0..=360.0 degrees
        #[arg(long, value_parser = parse_deg)]
        deg: f32,
        /// Hold for this many seconds (default: until Ctrl-C)
        #[arg(long, value_parser = parse_seconds)]
        secs: Option<f64>,
    },
}

/// A duration in seconds, as a finite non-negative float.
///
/// `Duration::from_secs_f64` **panics** on `inf` and `NaN`, and clamping
/// with `.max(0.0)` does not catch either — so reject them here, at the
/// argument boundary, rather than letting them reach the conversion.
fn parse_seconds(s: &str) -> Result<f64, String> {
    let v: f64 = s
        .trim()
        .parse()
        .map_err(|_| format!("invalid number {s:?}"))?;
    if !v.is_finite() {
        return Err(format!("{s:?} is not a finite number"));
    }
    if !(0.0..=3600.0).contains(&v) {
        return Err(format!("{v} is out of range (0..=3600 seconds)"));
    }
    Ok(v)
}

/// The smallest serial reply window that still gives a healthy motor time to
/// answer. `--timeout` doubles as this window, and `0` would leave only the
/// ~0.9 ms wire time — turning every read into a false "no response" (a false
/// "No motors found" from `scan`, a false "no motor detected" gating `set-id`).
const MIN_TIMEOUT_SECS: f64 = 0.005;

/// The global `--timeout`: a finite reply window with a small positive floor.
///
/// Distinct from [`parse_seconds`] (which allows `0`, meaningful for a
/// `--secs` run duration) precisely because here `0` is never useful and is
/// actively misleading — see [`MIN_TIMEOUT_SECS`].
fn parse_timeout(s: &str) -> Result<f64, String> {
    let v = parse_seconds(s)?;
    if v < MIN_TIMEOUT_SECS {
        return Err(format!(
            "{v} is too small; --timeout must be at least {MIN_TIMEOUT_SECS} s"
        ));
    }
    Ok(v)
}

/// A poll rate in Hz. Rejects non-finite and non-positive values, which
/// would otherwise produce an infinite or panicking interval.
fn parse_hz(s: &str) -> Result<f64, String> {
    let v: f64 = s
        .trim()
        .parse()
        .map_err(|_| format!("invalid number {s:?}"))?;
    if !v.is_finite() {
        return Err(format!("{s:?} is not a finite number"));
    }
    if !(0.001..=1000.0).contains(&v) {
        return Err(format!("{v} is out of range (0.001..=1000 Hz)"));
    }
    Ok(v)
}

/// A velocity preset, rejected up front rather than silently clamped on the
/// wire — a dashboard that displays 5000 RPM while commanding 330 is lying.
fn parse_rpm(s: &str) -> Result<i16, String> {
    let v: i16 = s
        .trim()
        .parse()
        .map_err(|_| format!("invalid number {s:?}"))?;
    if !(RPM_MIN..=RPM_MAX).contains(&v) {
        return Err(format!("{v} is out of range ({RPM_MIN}..={RPM_MAX} RPM)"));
    }
    Ok(v)
}

/// A torque-current setpoint in amps. The current loop maps ±32767 to about
/// ±8 A, so anything beyond ±8 A is unreachable and rejected up front.
fn parse_amps(s: &str) -> Result<f32, String> {
    let v: f32 = s
        .trim()
        .parse()
        .map_err(|_| format!("invalid number {s:?}"))?;
    if !v.is_finite() {
        return Err(format!("{s:?} is not a finite number"));
    }
    if !(-8.0..=8.0).contains(&v) {
        return Err(format!("{v} is out of range (-8.0..=8.0 A)"));
    }
    Ok(v)
}

/// An absolute angle in degrees for position mode (single-turn, 0..=360).
fn parse_deg(s: &str) -> Result<f32, String> {
    let v: f32 = s
        .trim()
        .parse()
        .map_err(|_| format!("invalid number {s:?}"))?;
    if !v.is_finite() {
        return Err(format!("{s:?} is not a finite number"));
    }
    if !(0.0..=360.0).contains(&v) {
        return Err(format!("{v} is out of range (0.0..=360.0 deg)"));
    }
    Ok(v)
}

/// A motor RS485 ID: an [`parse_int_auto`] value in the assignable range
/// `0x01..=0xFE`. Validating at the clap boundary turns `--id 0x00`/`0xFF`
/// into a clean usage error instead of a driver error at port-open time.
fn parse_id(s: &str) -> Result<u8, String> {
    let v = parse_int_auto(s)?;
    m0601::protocol::validate_id(v)
        .map_err(|_| format!("id 0x{v:02X} is out of range (0x01..=0xFE)"))?;
    Ok(v)
}

/// `int(s, 0)` equivalent: `0x`/`0o`/`0b` prefixes or plain decimal.
fn parse_int_auto(s: &str) -> Result<u8, String> {
    let s = s.trim();
    let (digits, radix) = match s.get(..2) {
        Some("0x") | Some("0X") => (&s[2..], 16),
        Some("0o") | Some("0O") => (&s[2..], 8),
        Some("0b") | Some("0B") => (&s[2..], 2),
        _ => (s, 10),
    };
    u8::from_str_radix(digits, radix).map_err(|e| format!("invalid number {s:?}: {e}"))
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    // parse_seconds already guarantees this converts; try_from keeps the
    // no-panic property even if that validation is ever loosened.
    let timeout = Duration::try_from_secs_f64(cli.timeout).unwrap_or(DEFAULT_TIMEOUT);

    let result = match cli.cmd {
        Cmd::Scan { full } => cmd::scan::run(&cli.port, timeout, full),
        Cmd::Info => cmd::info::run(&cli.port, cli.id, timeout),
        Cmd::Monitor { hz, csv } => cmd::monitor::run(&cli.port, cli.id, timeout, hz, csv),
        Cmd::Control { rpm, accel } => cmd::control::run(&cli.port, cli.id, rpm, accel),
        Cmd::Drive { mode } => {
            let plan = match mode {
                DriveMode::Velocity { rpm, accel, secs } => cmd::drive::Plan {
                    setpoint: cmd::drive::Setpoint::Velocity { rpm, accel },
                    secs,
                },
                DriveMode::Current { amps, secs } => cmd::drive::Plan {
                    setpoint: cmd::drive::Setpoint::Current {
                        raw: m0601::protocol::amps_to_raw(amps),
                    },
                    secs,
                },
                DriveMode::Position { deg, secs } => cmd::drive::Plan {
                    setpoint: cmd::drive::Setpoint::Position {
                        raw: m0601::protocol::deg_to_raw(deg),
                    },
                    secs,
                },
            };
            cmd::drive::run(&cli.port, cli.id, timeout, plan)
        }
        Cmd::SetId { new, yes } => cmd::set_id::run(&cli.port, timeout, new, yes),
        Cmd::Raw { hex, yes } => cmd::raw::run(&cli.port, cli.id, timeout, &hex, yes),
    };

    match result {
        Ok(code) => code,
        Err(e) => {
            eprintln!("[x] {e}");
            if e.is_permission_denied() {
                eprintln!(
                    "    Hint: add yourself to the dialout group:  sudo usermod -aG dialout $USER"
                );
                eprintln!("    (then log out and back in)");
            }
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_hz, parse_id, parse_int_auto, parse_rpm, parse_seconds, parse_timeout};
    use std::time::Duration;

    #[test]
    fn id_rejects_the_unassignable_ends_of_the_range() {
        // 0x00 and 0xFF are not assignable motor IDs; reject at the boundary.
        assert!(parse_id("0x00").is_err());
        assert!(parse_id("0").is_err());
        assert!(parse_id("0xFF").is_err());
        assert!(parse_id("255").is_err());
        assert_eq!(parse_id("0x01"), Ok(0x01));
        assert_eq!(parse_id("0xFE"), Ok(0xFE));
        assert_eq!(parse_id("42"), Ok(42));
    }

    #[test]
    fn timeout_rejects_zero_and_too_small_but_keeps_secs_permissive() {
        // The global --timeout is the serial reply window: 0 (or a sub-ms
        // value) turns every read into a false "no response".
        assert!(parse_timeout("0").is_err());
        assert!(parse_timeout("0.0").is_err());
        assert!(parse_timeout("0.001").is_err());
        assert!(parse_timeout("inf").is_err());
        assert_eq!(parse_timeout("0.15"), Ok(0.15));
        assert_eq!(parse_timeout("0.005"), Ok(0.005));
        // --secs keeps using parse_seconds, where 0 ("stop immediately") is
        // legitimate — the floor must not have leaked into it.
        assert_eq!(parse_seconds("0"), Ok(0.0));
    }

    #[test]
    fn rejects_non_finite_seconds_that_would_panic_duration() {
        // Duration::from_secs_f64 panics on each of these.
        for bad in ["inf", "-inf", "NaN", "1e30"] {
            assert!(parse_seconds(bad).is_err(), "{bad} must be rejected");
        }
        assert!(parse_seconds("-1").is_err());
        assert_eq!(parse_seconds("0.15"), Ok(0.15));
        assert_eq!(parse_seconds("0"), Ok(0.0));
    }

    #[test]
    fn accepted_seconds_always_convert_to_a_duration() {
        for good in ["0", "0.15", "1", "3600"] {
            let v = parse_seconds(good).expect("accepted");
            assert!(
                Duration::try_from_secs_f64(v).is_ok(),
                "{good} must convert"
            );
        }
    }

    #[test]
    fn rejects_degenerate_poll_rates() {
        for bad in ["inf", "NaN", "0", "-5", "1e-30", "100000"] {
            assert!(parse_hz(bad).is_err(), "{bad} must be rejected");
        }
        assert_eq!(parse_hz("5"), Ok(5.0));
    }

    #[test]
    fn rejects_rpm_outside_the_motor_range() {
        assert!(parse_rpm("5000").is_err());
        assert!(parse_rpm("-5000").is_err());
        assert_eq!(parse_rpm("330"), Ok(330));
        assert_eq!(parse_rpm("-330"), Ok(-330));
        assert_eq!(parse_rpm("0"), Ok(0));
    }

    #[test]
    fn rejects_amps_outside_the_current_range() {
        for bad in ["inf", "NaN", "8.1", "-8.1", "100"] {
            assert!(super::parse_amps(bad).is_err(), "{bad} must be rejected");
        }
        assert_eq!(super::parse_amps("1.5"), Ok(1.5));
        assert_eq!(super::parse_amps("-8"), Ok(-8.0));
        assert_eq!(super::parse_amps("0"), Ok(0.0));
    }

    #[test]
    fn rejects_degrees_outside_a_single_turn() {
        for bad in ["inf", "NaN", "-0.1", "360.1", "720"] {
            assert!(super::parse_deg(bad).is_err(), "{bad} must be rejected");
        }
        assert_eq!(super::parse_deg("0"), Ok(0.0));
        assert_eq!(super::parse_deg("180.5"), Ok(180.5));
        assert_eq!(super::parse_deg("360"), Ok(360.0));
    }

    #[test]
    fn parses_hex_and_decimal() {
        assert_eq!(parse_int_auto("0x2A"), Ok(0x2A));
        assert_eq!(parse_int_auto("0X0f"), Ok(0x0F));
        assert_eq!(parse_int_auto("42"), Ok(42));
        assert_eq!(parse_int_auto("0b101"), Ok(5));
        assert_eq!(parse_int_auto("0o17"), Ok(15));
    }

    #[test]
    fn rejects_junk() {
        assert!(parse_int_auto("zap").is_err());
        assert!(parse_int_auto("0xGG").is_err());
        assert!(parse_int_auto("300").is_err()); // overflows u8
        assert!(parse_int_auto("").is_err());
    }
}
