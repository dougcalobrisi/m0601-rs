//! `monitor` — headless live line-dashboard with optional CSV logging.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use m0601::{Feedback, M0601};

/// CSV column layout. Stable — downstream logs depend on it.
const CSV_HEADER: &str = "timestamp,motor_id,mode,speed_rpm,current_a,temp_c,\
                          position_deg,error_code,error_str,raw_hex";

/// Hand-rolled CSV is safe here: every field is program-controlled (numbers,
/// mode names, fault names joined with " | ", spaced hex) — none can ever
/// contain a comma, quote, or newline.
fn csv_row(fb: &Feedback) -> String {
    format!(
        "{},{},{},{},{:.3},{},{:.1},{},{},{}",
        jiff::Zoned::now().strftime("%Y-%m-%d %H:%M:%S"),
        fb.id,
        fb.mode_name(),
        fb.speed_rpm,
        fb.current_a,
        // Always present here — monitor only sends 0x74 queries, whose
        // replies carry the temperature — but never panic on a frame.
        fb.temp_c.map_or_else(String::new, |t| t.to_string()),
        fb.position_deg,
        fb.faults.0,
        fb.faults,
        fb.raw_hex(),
    )
}

pub fn run(
    port: &str,
    id: u8,
    timeout: Duration,
    hz: f64,
    csv: Option<String>,
) -> m0601::Result<ExitCode> {
    // `hz` is validated at the argument boundary (finite, 0.001..=1000), so
    // 1/hz is finite too — but try_from keeps this free of a panicking
    // conversion regardless of what a future caller passes.
    let interval = Duration::try_from_secs_f64(1.0 / hz).unwrap_or(Duration::from_millis(200));

    let mut log = match &csv {
        Some(path) => {
            // `File::create` truncates. Say so before it happens rather than
            // letting a re-run silently destroy the previous session's log.
            if std::path::Path::new(path).exists() {
                println!("[!] {path} already exists — overwriting it.");
            }
            let mut w = BufWriter::new(File::create(path)?);
            writeln!(w, "{CSV_HEADER}")?;
            Some(w)
        }
        None => None,
    };

    let mut motor = M0601::open(port, id, timeout)?;
    println!("Monitoring 0x{id:02X} on {port} at {hz} Hz. Ctrl+C to stop.");
    if let Some(path) = &csv {
        println!("Logging to {path}");
    }

    // Ctrl-C / SIGTERM / SIGHUP just clear the flag; the loop then exits and
    // the CSV file is flushed and closed normally.
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = running.clone();
        let _ = ctrlc::set_handler(move || running.store(false, Ordering::Relaxed));
    }

    let mut count: u64 = 0;
    let mut no_resp = 0u32;
    while running.load(Ordering::Relaxed) {
        let t0 = Instant::now();
        // A transient bus error (USB hiccup) must not kill a long-running
        // monitor — warn and keep polling, same policy as the control loop.
        let reading = match motor.query() {
            Ok(reading) => reading,
            Err(e) => {
                print!("\r[!] bus error: {e} — still polling          ");
                let _ = std::io::stdout().flush();
                std::thread::sleep(interval);
                continue;
            }
        };
        match reading {
            None => {
                // Debounce: a single dropped poll is common on RS485, so
                // leave the last good reading on screen and only warn after 5
                // consecutive misses (~1 s at 5 Hz) rather than flapping the
                // line on every transient gap.
                no_resp += 1;
                if no_resp >= 5 {
                    print!("\r[!] no response — check motor power/wiring     ");
                    let _ = std::io::stdout().flush();
                }
            }
            Some(fb) => {
                no_resp = 0;
                count += 1;
                let fault = if fb.faults.is_ok() { "OK  " } else { "FAULT" };
                let trailer = if fb.faults.is_ok() {
                    "  ".to_owned()
                } else {
                    format!(" {}", fb.faults)
                };
                let temp = fb
                    .temp_c
                    .map_or_else(|| " --".to_owned(), |t| format!("{t:3}"));
                print!(
                    "\r[{}] #{count:5} | {:<8} | Speed {:+4} RPM | Cur {:+6.3} A | \
                     Pos {:5.1} | Temp {temp}C | {fault}{trailer}",
                    jiff::Zoned::now().strftime("%H:%M:%S"),
                    fb.mode_name(),
                    fb.speed_rpm,
                    fb.current_a,
                    fb.position_deg,
                );
                let _ = std::io::stdout().flush();
                if let Some(w) = &mut log {
                    writeln!(w, "{}", csv_row(&fb))?;
                    w.flush()?; // survive abrupt termination row-by-row
                }
            }
        }
        let elapsed = t0.elapsed();
        if elapsed < interval {
            std::thread::sleep(interval - elapsed);
        }
    }

    println!("\nStopped.");
    if let (Some(w), Some(path)) = (&mut log, &csv) {
        w.flush()?;
        println!("Saved {count} rows to {path}");
    }
    Ok(ExitCode::SUCCESS)
}
