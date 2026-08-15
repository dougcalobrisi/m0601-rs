//! `scan` — discover motor IDs on the bus.

use std::io::Write;
use std::ops::RangeInclusive;
use std::process::ExitCode;
use std::time::Duration;

use m0601::protocol::BAUD;
use m0601::{Bus, ScanReport};

/// What the default scan polls after the broadcast. Motors ship at 0x01 and
/// small fleets stay in single digits, so this catches the common case in a
/// couple of seconds instead of `--full`'s ~40.
const QUICK_POLL: RangeInclusive<u8> = 0x01..=0x0F;
/// What `--full` polls: every assignable ID.
const FULL_POLL: RangeInclusive<u8> = 0x01..=0xFE;

pub fn run(port: &str, timeout: Duration, full: bool) -> m0601::Result<ExitCode> {
    let bus = Bus::open(port, timeout)?;
    println!("Scanning {port} @ {BAUD} 8N1 ...");

    // `exhaustive` tracks whether every ID was individually probed — only
    // then is an empty result evidence of an empty bus, and only then are
    // the range warning and the garbled warning not worth printing.
    let (report, exhaustive) = if full {
        (poll_scan(&bus, FULL_POLL)?, true)
    } else {
        let quick = poll_scan(&bus, QUICK_POLL)?;
        if quick.ids.is_empty() && quick.garbled {
            // The broadcast came back as garbage no motor sent — motors
            // are answering (colliding), just none inside the quick range.
            // An empty quick scan therefore proves nothing; probe them all.
            // (Not collision-proof either — duplicate IDs still answer the
            // same probe together — but those need set-id surgery anyway.)
            println!(
                "Broadcast reply was garbled (motors answering together collide), yet no \
                 motor answered 0x01..0x0F — polling every ID."
            );
            (poll_scan(&bus, FULL_POLL)?, true)
        } else {
            (quick, false)
        }
    };

    if report.ids.is_empty() {
        // Failure diagnostics to stderr; only found-motor lines are stdout data.
        eprintln!("\nNo motors found.");
        eprintln!(
            "  Checklist: 18V power on? brown wire -> GND? try swapping A/B (orange<->white)."
        );
        if exhaustive {
            eprintln!("  (All 254 IDs 0x01..0xFE were polled.)");
        } else {
            eprintln!("  Only IDs 0x01..0x0F were polled — `scan --full` tries all 254.");
        }
        return Ok(ExitCode::FAILURE);
    }

    println!("\nFound {} motor(s):", report.ids.len());
    for mid in &report.ids {
        println!("  - ID 0x{mid:02X} (decimal {mid})");
    }
    if !exhaustive {
        if report.garbled {
            println!(
                "\nOnly IDs 0x01..0x0F were polled, and part of the broadcast reply was \
                 garbled — colliding or higher-ID motors may be hidden."
            );
            println!("Run `scan --full` for a definitive list.");
        } else {
            println!("\nOnly IDs 0x01..0x0F were polled; motors above that need `scan --full`.");
        }
    }
    if let [only] = report.ids.as_slice() {
        println!("\nUse:  --id 0x{only:02X}");
    }
    Ok(ExitCode::SUCCESS)
}

/// Broadcast, then poll `range`, with a progress bar.
fn poll_scan(bus: &Bus, range: RangeInclusive<u8>) -> m0601::Result<ScanReport> {
    let (start, end) = (*range.start(), *range.end());
    let count = usize::from(end - start) + 1;
    let secs = (count as f64 * bus.timeout().as_secs_f64()).ceil();
    println!("Polling 0x{start:02X}..0x{end:02X} ({count} IDs, ~{secs:.0}s):");
    let report = bus.scan(range.clone(), progress_bar(&range))?;
    clear_progress();
    Ok(report)
}

/// A `Bus::scan` progress callback drawing a 30-cell bar across `range`.
/// Shared with `set-id`, whose exhaustive pre-write scan is otherwise a
/// ~40 s silent stall. `mid` is the ID *about* to be probed, so the bar
/// shows 1/count at the first ID and reaches full at the last.
pub(crate) fn progress_bar(range: &RangeInclusive<u8>) -> impl FnMut(u8) {
    let start = *range.start();
    let count = usize::from(*range.end() - start) + 1;
    move |mid| {
        let filled = 30 * (usize::from(mid - start) + 1) / count;
        print!(
            "\r  [{}{}] 0x{mid:02X}",
            "#".repeat(filled),
            "-".repeat(30 - filled)
        );
        let _ = std::io::stdout().flush();
    }
}

/// Erase the progress-bar line left by [`progress_bar`].
pub(crate) fn clear_progress() {
    print!("\r{}\r", " ".repeat(50));
    let _ = std::io::stdout().flush();
}
