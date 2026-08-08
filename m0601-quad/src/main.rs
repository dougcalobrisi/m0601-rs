//! `m0601-quad` — drive four M0601 hub motors as one skid-steer rover.
//!
//! This crate is also the **reference implementation** for multi-motor
//! use of the `m0601` library: the pilot shows the one-thread-owns-the-
//! bus scheduling the library's docs prescribe, `rover.rs` shows the
//! sign-convention wiring, and every exit path funnels to
//! `Bus::safe_stop_all`.
//!
//! Exit paths that stop all four wheels: quit key, `?`-propagation (the
//! `StopGuard` is armed before the first frame), pilot panic
//! (`catch_unwind`), UI panic (`TermGuard` then `StopGuard`), and
//! SIGINT/SIGTERM/SIGHUP via `ctrlc`. On SIGKILL all four coast —
//! documented, because a coasting rover rolls.

mod clock;
mod cmd;
mod config;
mod io;
mod logger;
mod mix;
mod pilot;
mod rover;
mod safety;
mod state;
mod ui;

use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::Ordering;

use clap::{Parser, Subcommand};

use crate::config::Config;
use crate::state::Shared;

#[derive(Parser)]
#[command(
    name = "m0601-quad",
    about = "Four-wheel skid-steer rover on one RS485 bus"
)]
struct Cli {
    /// Path to the wheel-map config.
    #[arg(long, default_value = "wheels.toml")]
    config: String,
    /// Override the serial port from the config.
    #[arg(long)]
    port: Option<String>,
    #[command(subcommand)]
    cmd: Option<Cmd>,
}

#[derive(Subcommand)]
enum Cmd {
    /// The 2x2 driving dashboard (default).
    Drive {
        /// Simulate four wheels; opens no serial port at all.
        #[arg(long)]
        dry_run: bool,
        /// Start even if a wheel does not answer the startup probe.
        #[arg(long)]
        ignore_silent: bool,
        /// Start even if a wheel is already turning (> 5 RPM).
        #[arg(long)]
        ignore_motion: bool,
        /// Start even if a wheel reports fault bits.
        #[arg(long)]
        ignore_faults: bool,
    },
    /// Validate the config and print the resolved wheel table.
    Check {
        /// Also open the port and probe each wheel once (read-only).
        #[arg(long)]
        probe: bool,
    },
    /// Headless telemetry + CSV. Commands no motion.
    Monitor,
    /// Spin one wheel at a bounded RPM for a bounded time, then stop.
    Jog {
        /// Which wheel: front-left/fl, front-right/fr, rear-left/rl,
        /// rear-right/rr (driver/pass accepted).
        #[arg(long)]
        wheel: String,
        #[arg(long, default_value_t = 60)]
        rpm: i16,
        #[arg(long, default_value_t = 2.0)]
        secs: f64,
    },
    /// Walk all four wheels and print corrected `invert` lines.
    Calibrate,
    /// One-shot vehicle-wide safe stop.
    Stop,
}

fn main() -> ExitCode {
    let cli = Cli::parse();

    let (mut cfg, warnings) = match Config::load(&cli.config) {
        Ok(x) => x,
        Err(errors) => {
            eprintln!("{} is not usable:", cli.config);
            for (i, e) in errors.iter().enumerate() {
                eprintln!("  {}. {e}", i + 1);
            }
            return ExitCode::from(2);
        }
    };
    if let Some(port) = cli.port {
        cfg.bus.port = port;
    }
    for w in &warnings {
        eprintln!("[!] {w}");
    }

    // One flag, one handler, every command: signals mean "stop".
    let shared = Arc::new(Shared::new());
    let sig = Arc::clone(&shared);
    if let Err(e) = ctrlc::set_handler(move || sig.running.store(false, Ordering::Relaxed)) {
        eprintln!("[!] cannot install signal handler: {e}");
    }

    let result = match cli.cmd.unwrap_or(Cmd::Drive {
        dry_run: false,
        ignore_silent: false,
        ignore_motion: false,
        ignore_faults: false,
    }) {
        Cmd::Check { probe } => cmd::check(&cfg, probe, &shared),
        Cmd::Monitor => cmd::monitor(&cfg, &shared),
        Cmd::Jog { wheel, rpm, secs } => cmd::jog(&cfg, &shared, &wheel, rpm, secs),
        Cmd::Calibrate => cmd::calibrate(&cfg, &shared),
        Cmd::Stop => cmd::stop(&cfg),
        Cmd::Drive {
            dry_run,
            ignore_silent,
            ignore_motion,
            ignore_faults,
        } => cmd::drive(
            cfg,
            shared,
            cmd::DriveFlags {
                dry_run,
                ignore_silent,
                ignore_motion,
                ignore_faults,
            },
            !warnings.is_empty(),
        ),
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
