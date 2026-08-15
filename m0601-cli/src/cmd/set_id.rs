//! `set-id` — change a motor's persistent RS485 ID.
//!
//! Guard rails: exactly one motor may be on the bus — verified with an
//! exhaustive scan, since the set-ID frame is unaddressed and would rename
//! every motor that hears it — and the change is confirmed interactively
//! unless `--yes` is given.

use std::io::Write;
use std::process::ExitCode;
use std::time::Duration;

use m0601::Bus;
use m0601::protocol::validate_id;

pub fn run(port: &str, timeout: Duration, new_id: u8, yes: bool) -> m0601::Result<ExitCode> {
    validate_id(new_id)?;

    let bar = "=".repeat(52);
    println!("{bar}");
    println!("  M0601 ID Changer");
    println!("  Port: {port}  ->  New ID: 0x{new_id:02X} ({new_id})");
    println!("{bar}");
    println!("  WARNING: only ONE motor may be on the bus. ID is persistent.");
    println!("  Do not connect another motor between this check and the write.");

    // The set-ID frame is unaddressed: every motor that hears it takes the
    // new ID. A broadcast (stage-1) scan cannot tell one motor from several
    // answering at once — their replies collide — so the guard rail is only
    // real if we poll every ID individually. Slow, but this writes
    // persistent state that is tedious to undo.
    let bus = Bus::open(port, timeout)?;
    // ~40 s to poll all 254 IDs — show the same progress bar as `scan` rather
    // than freezing silently (an unfed progress line invites a mid-scan kill).
    let range = 0x01..=0xFE;
    println!("  Checking the bus is not shared (polling all 254 IDs):");
    let ids = bus.scan(range.clone(), super::scan::progress_bar(&range))?.ids;
    super::scan::clear_progress();
    let current = match ids.as_slice() {
        [] => {
            eprintln!("[x] No motor detected. Check power/wiring.");
            return Ok(ExitCode::FAILURE);
        }
        [one] => *one,
        many => {
            let listed: Vec<String> = many.iter().map(|i| format!("0x{i:02X}")).collect();
            eprintln!(
                "[x] Multiple motors detected [{}]. Disconnect all but one.",
                listed.join(", ")
            );
            return Ok(ExitCode::FAILURE);
        }
    };

    println!("[ok] Current ID: 0x{current:02X}");
    if current == new_id {
        println!("[!] Already at that ID. Nothing to do.");
        return Ok(ExitCode::SUCCESS);
    }

    if !yes && !confirm(current, new_id)? {
        println!("Cancelled.");
        return Ok(ExitCode::SUCCESS);
    }

    match bus.set_id(new_id)? {
        Some(reported) if reported == new_id => {
            println!("[ok] SUCCESS — motor ID is now 0x{new_id:02X}. Use --id 0x{new_id:02X}.");
            Ok(ExitCode::SUCCESS)
        }
        Some(reported) => {
            eprintln!(
                "[x] Motor reports 0x{reported:02X} — change may have failed. Try power-cycling."
            );
            Ok(ExitCode::FAILURE)
        }
        None => {
            eprintln!("[?] No response after change. Power-cycle and run 'scan' to confirm.");
            Ok(ExitCode::FAILURE)
        }
    }
}

/// Ask the user to type "yes". Anything else cancels.
fn confirm(current: u8, new_id: u8) -> std::io::Result<bool> {
    print!("Change 0x{current:02X} -> 0x{new_id:02X}? type 'yes': ");
    std::io::stdout().flush()?;
    let mut line = String::new();
    std::io::stdin().read_line(&mut line)?;
    Ok(line.trim().eq_ignore_ascii_case("yes"))
}
