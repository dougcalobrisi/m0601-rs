//! `scan` — discover motor IDs on the bus.

use std::io::Write;
use std::process::ExitCode;
use std::time::Duration;

use m0601::protocol::BAUD;
use m0601::{Bus, ScanReport};

pub fn run(port: &str, timeout: Duration, full: bool) -> m0601::Result<ExitCode> {
    let bus = Bus::open(port, timeout)?;
    println!("Scanning {port} @ {BAUD} 8N1 ...");

    // `exhaustive` tracks whether every ID was individually probed — only
    // then is an empty result evidence of an empty bus, and only then is
    // stage-1 garbling not worth a warning.
    let (report, exhaustive) = if full {
        (full_scan(&bus)?, true)
    } else {
        let quick = bus.scan(false, |_| {})?;
        if quick.ids.is_empty() && quick.garbled {
            // The broadcast came back as garbage no motor sent — several
            // motors answering at once collide into exactly this. An empty
            // quick scan therefore proves nothing; probe each ID alone.
            // (Not collision-proof either — duplicate IDs still answer the
            // same probe together — but those need set-id surgery anyway.)
            println!("Broadcast reply was garbled (motors answering together collide).");
            (full_scan(&bus)?, true)
        } else {
            (quick, false)
        }
    };

    if report.ids.is_empty() {
        println!("\nNo motors found.");
        println!(
            "  Checklist: 18V power on? brown wire -> GND? try swapping A/B (orange<->white)."
        );
        return Ok(ExitCode::FAILURE);
    }

    println!("\nFound {} motor(s):", report.ids.len());
    for mid in &report.ids {
        println!("  - ID 0x{mid:02X} (decimal {mid})");
    }
    if !exhaustive && report.garbled {
        println!("\nPart of the broadcast reply was garbled — colliding motors may be hidden.");
        println!("Run `scan --full` for a definitive list.");
    }
    if let [only] = report.ids.as_slice() {
        println!("\nUse:  --id 0x{only:02X}");
    }
    Ok(ExitCode::SUCCESS)
}

/// Poll every ID individually, with a progress bar.
fn full_scan(bus: &Bus) -> m0601::Result<ScanReport> {
    println!("Full poll 0x01..0xFE (~40s):");
    let report = bus.scan(true, |mid| {
        let filled = (30 * mid as usize) / 254;
        print!(
            "\r  [{}{}] 0x{mid:02X}",
            "#".repeat(filled),
            "-".repeat(30 - filled)
        );
        let _ = std::io::stdout().flush();
    })?;
    print!("\r{}\r", " ".repeat(50));
    Ok(report)
}
