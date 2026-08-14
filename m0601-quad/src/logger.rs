//! CSV logging on its own thread. **CSV writes must never run on the
//! pilot thread**: one stalled SD-card write would coast all four wheels.
//! The pilot `try_send`s snapshots into a bounded channel and drops rows
//! (counted, shown in the UI) when the disk can't keep up.

use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::sync::mpsc::Receiver;

use crate::pilot::LogRow;

/// The CLI monitor's column layout, byte-identical (its comment says
/// "Stable — downstream logs depend on it"), with two columns APPENDED so
/// positional parsers of the old format keep working. `cmd_rpm` is what
/// makes a rover log useful: without it you cannot tell "commanded 100,
/// got 30" from "commanded 30".
pub const CSV_HEADER: &str = "timestamp,motor_id,mode,speed_rpm,current_a,temp_c,position_deg,\
                              error_code,error_str,raw_hex,wheel_name,cmd_rpm";

/// Format one CSV telemetry row (no trailing newline) in the [`CSV_HEADER`]
/// column order. Shared by the headless `monitor` command and this logger
/// thread so a column change is made in exactly one place.
///
/// `temp_c` and `position_deg` are passed in rather than read from `fb`
/// because the logger prefers the hi-res angle and last-seen temperature its
/// [`Telemetry`](m0601::Telemetry) accumulator retains, while `monitor` has
/// only the bare reply. `cmd_rpm` is `None` for `monitor` (which commands no
/// motion) and leaves that trailing column empty.
pub fn csv_row(
    stamp: jiff::Timestamp,
    fb: &m0601::Feedback,
    temp_c: Option<u8>,
    position_deg: f32,
    name: &str,
    cmd_rpm: Option<i16>,
) -> String {
    format!(
        "{stamp},{:#04X},{},{},{:.3},{},{:.1},{},{},{},{},{}",
        fb.id,
        fb.mode_name(),
        fb.speed_rpm,
        fb.current_a,
        temp_c.map_or_else(String::new, |t| t.to_string()),
        position_deg,
        fb.faults.0,
        fb.faults,
        fb.raw_hex(),
        name,
        cmd_rpm.map_or_else(String::new, |c| c.to_string()),
    )
}

/// Open `path` for appending (creating it if absent) and write the
/// [`CSV_HEADER`] first when the file is newly created. Shared with the
/// `monitor` command so the header/append handling lives once.
pub fn open_appending(path: &str) -> std::io::Result<BufWriter<std::fs::File>> {
    let file = OpenOptions::new().create(true).append(true).open(path)?;
    let new_file = file.metadata().map(|m| m.len() == 0).unwrap_or(false);
    let mut out = BufWriter::new(file);
    if new_file {
        writeln!(out, "{CSV_HEADER}")?;
    }
    Ok(out)
}

/// Runs until the channel closes (the pilot dropping its sender ends us).
/// Returns the rows written, for the caller's exit summary.
pub fn run(rx: Receiver<LogRow>, path: &str, names: &[String; 4]) -> std::io::Result<u64> {
    let mut out = open_appending(path)?;

    let mut rows: u64 = 0;
    let mut unflushed: u64 = 0;
    while let Ok(snapshot) = rx.recv() {
        let stamp = jiff::Timestamp::now();
        for (i, w) in snapshot.wheels.iter().enumerate() {
            let Some(fb) = w.telemetry.fb else {
                continue; // nothing received from this wheel yet
            };
            writeln!(
                out,
                "{}",
                csv_row(
                    stamp,
                    &fb,
                    w.telemetry.temp_c,
                    w.telemetry.position_deg.unwrap_or(fb.position_deg),
                    &names[i],
                    Some(w.cmd_rpm),
                )
            )?;
            rows += 1;
            unflushed += 1;
        }
        // Bounded staleness on power loss without a flush-per-row storm.
        // Counted since the last flush: `rows` grows by 0–4 per snapshot
        // (wheels without telemetry are skipped), so an exact
        // multiple-of-64 test could step over its trigger indefinitely.
        if unflushed >= 64 {
            out.flush()?;
            unflushed = 0;
        }
    }
    out.flush()?;
    Ok(rows)
}
