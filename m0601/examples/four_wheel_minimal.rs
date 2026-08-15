//! The whole driver in one screen: open a bus, mint one handle per wheel,
//! arm a stop guard, and run a short drive → poll → stop cycle.
//!
//! This is the distilled essence of the `m0601-quad` sample — no TUI, no
//! logger, no safety state machine — so the driver API is the only thing on
//! screen. `m0601-quad` is the same wiring grown into a real application.
//!
//! Run against hardware (four motors at IDs 1–4 on one RS485 adapter):
//!
//! ```text
//! cargo run --example four_wheel_minimal -- /dev/ttyUSB0
//! ```
//!
//! With no port argument it prints usage and exits cleanly, so
//! `cargo build --examples` needs no hardware.

use std::time::{Duration, Instant};

use m0601::{Bus, M0601, Mode, SerialTransport};

/// Four wheels: RS485 id and whether this corner is a mirror-image build
/// (the right side of the chassis). `mirrored(true)` makes `+rpm` mean
/// "forward" on every wheel — the driver negates the setpoint and flips the
/// reported speed underneath, so the app never touches a sign again.
const WHEELS: [(u8, bool); 4] = [
    (1, false), // front-left
    (2, true),  // front-right  (mirrored)
    (3, false), // rear-left
    (4, true),  // rear-right   (mirrored)
];

/// Stops every wheel when dropped — armed BEFORE the first drive frame, so a
/// `?` early-return or a panic still lands on a bus-wide stop.
struct StopGuard {
    bus: Bus<SerialTransport>,
    ids: Vec<u8>,
}

impl Drop for StopGuard {
    fn drop(&mut self) {
        self.bus.safe_stop_all(&self.ids);
    }
}

fn main() -> m0601::Result<()> {
    let Some(port) = std::env::args().nth(1) else {
        eprintln!("usage: four_wheel_minimal <serial-port>   e.g. /dev/ttyUSB0");
        return Ok(());
    };

    // One shared bus. The generous open timeout is the backstop; the drive
    // loop below passes its own short reply wait per poll.
    let bus = Bus::open(&port, Duration::from_millis(150))?;
    let ids: Vec<u8> = WHEELS.iter().map(|&(id, _)| id).collect();

    // One handle per wheel, sign convention applied once at construction.
    let mut wheels: Vec<M0601<SerialTransport>> = WHEELS
        .iter()
        .map(|&(id, mirror)| bus.motor(id).map(|m| m.mirrored(mirror)))
        .collect::<m0601::Result<_>>()?;

    // Arm the stop guard before anything moves, then switch the group into
    // velocity mode.
    let _stop = StopGuard {
        bus: bus.clone(),
        ids: ids.clone(),
    };
    bus.set_mode_all(&ids, Mode::Velocity)?;

    // Drive all four at 60 RPM for one second, resending every ~20 ms (the
    // 50 Hz floor — a wheel coasts if it goes longer than that between
    // frames), and polling one wheel per cycle round-robin with a short wait.
    let cycle = Duration::from_millis(20);
    let reply_wait = Duration::from_millis(5);
    let start = Instant::now();
    let mut k = 0usize;
    while start.elapsed() < Duration::from_secs(1) {
        let tick = Instant::now();
        for wheel in &mut wheels {
            wheel.drive_velocity(60)?;
        }
        if let Some(fb) = wheels[k].query_with(reply_wait)? {
            println!(
                "wheel {} -> {:+} RPM, {} faults",
                fb.id, fb.speed_rpm, fb.faults
            );
        }
        k = (k + 1) % wheels.len();
        if let Some(rem) = cycle.checked_sub(tick.elapsed()) {
            std::thread::sleep(rem);
        }
    }

    // `_stop` drops here and stops the group; this is also the path a `?`
    // above would have taken.
    Ok(())
}
