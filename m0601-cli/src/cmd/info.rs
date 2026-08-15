//! `info` — configuration block plus a one-shot live readout.

use std::process::ExitCode;
use std::time::Duration;

use m0601::M0601;
use m0601::protocol::{BAUD, CUR_MAX, CUR_MIN, POS_MAX, RPM_MAX, RPM_MIN};

pub fn run(port: &str, id: u8, timeout: Duration) -> m0601::Result<ExitCode> {
    let mut motor = M0601::open(port, id, timeout)?;

    let bar = "=".repeat(48);
    println!("{bar}");
    println!("  M0601 Configuration");
    println!("{bar}");
    println!("  Port          : {port}");
    println!("  Baud / format : {BAUD} 8N1 (RS485 half-duplex)");
    println!("  Motor ID      : 0x{id:02X} ({id})");
    println!("  Velocity range: {RPM_MIN}..{RPM_MAX} RPM");
    println!("  Current range : {CUR_MIN}..{CUR_MAX} (~-8..+8 A)");
    println!("  Position range: 0..{POS_MAX} (0..360 deg)");
    println!("{}", "-".repeat(48));

    let Some(fb) = motor.query()? else {
        // Diagnostics go to stderr so `m0601 info > log.txt` captures only the
        // real readout, not this failure notice.
        eprintln!("  Live readout  : no valid response.");
        eprintln!("  Check 18V power, wiring (brown->GND), A/B polarity, and --id.");
        return Ok(ExitCode::FAILURE);
    };

    println!("  Mode          : {}", fb.mode_name());
    println!("  Speed         : {:+} RPM", fb.speed_rpm);
    println!("  Current       : {:+.3} A", fb.current_a);
    println!("  Position      : {:.1} deg", fb.position_deg);
    // query() replies always carry the temperature; "--" guards a future
    // caller change without a panicking unwrap.
    match fb.temp_c {
        Some(t) => println!("  Winding temp  : {t} C"),
        None => println!("  Winding temp  : --"),
    }
    let status = if fb.faults.is_ok() {
        "OK".to_owned()
    } else {
        format!("FAULT ({})", fb.faults)
    };
    println!("  Error         : 0x{:02X}  {status}", fb.faults.0);
    println!("  Raw frame     : {}", fb.raw_hex());
    println!("{bar}");
    Ok(ExitCode::SUCCESS)
}
