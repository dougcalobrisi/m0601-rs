//! `scan` — discover motor IDs on the bus.

use std::io::Write;
use std::process::ExitCode;
use std::time::Duration;

use m0601::Bus;
use m0601::protocol::BAUD;

pub fn run(port: &str, timeout: Duration, full: bool) -> m0601::Result<ExitCode> {
    let bus = Bus::open(port, timeout)?;
    println!("Scanning {port} @ {BAUD} 8N1 ...");

    let ids = if full {
        println!("Full poll 0x01..0xFE (~40s):");
        let found = bus.scan(true, |mid| {
            let filled = (30 * mid as usize) / 254;
            print!(
                "\r  [{}{}] 0x{mid:02X}",
                "#".repeat(filled),
                "-".repeat(30 - filled)
            );
            let _ = std::io::stdout().flush();
        })?;
        print!("\r{}\r", " ".repeat(50));
        found
    } else {
        bus.scan(false, |_| {})?
    };

    if ids.is_empty() {
        println!("\nNo motors found.");
        println!(
            "  Checklist: 18V power on? brown wire -> GND? try swapping A/B (orange<->white)."
        );
        return Ok(ExitCode::FAILURE);
    }

    println!("\nFound {} motor(s):", ids.len());
    for mid in &ids {
        println!("  - ID 0x{mid:02X} (decimal {mid})");
    }
    if let [only] = ids.as_slice() {
        println!("\nUse:  --id 0x{only:02X}");
    }
    Ok(ExitCode::SUCCESS)
}
