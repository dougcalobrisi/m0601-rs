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

    /// Motor ID, e.g. 0x01
    #[arg(long, global = true, default_value = "0x01", value_parser = parse_int_auto)]
    id: u8,

    /// Serial read timeout in seconds
    #[arg(long, global = true, default_value_t = 0.15, value_parser = parse_seconds)]
    timeout: f64,

    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Discover motor IDs on the bus
    Scan {
        /// Poll every ID 0x01..0xFE (~40s)
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
        #[arg(long, default_value_t = 100, value_parser = parse_rpm)]
        rpm: i16,
    },
    /// Change a motor's RS485 ID (persistent, one motor only)
    SetId {
        /// New ID 0x01..0xFE
        #[arg(long, value_parser = parse_int_auto)]
        new: u8,
        /// Skip confirmation prompt
        #[arg(long)]
        yes: bool,
    },
    /// Send an arbitrary frame (9 bytes = CRC auto-added, or 10)
    Raw {
        /// Hex bytes, e.g. "01 74 00 00 00 00 00 00 00"
        hex: String,
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
        Cmd::Control { rpm } => cmd::control::run(&cli.port, cli.id, rpm),
        Cmd::SetId { new, yes } => cmd::set_id::run(&cli.port, timeout, new, yes),
        Cmd::Raw { hex } => cmd::raw::run(&cli.port, cli.id, timeout, &hex),
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
    use super::{parse_hz, parse_int_auto, parse_rpm, parse_seconds};
    use std::time::Duration;

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
