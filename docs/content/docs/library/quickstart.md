---
title: Quickstart
weight: 1
---

# Query without moving anything

```rust
use std::time::Duration;
use m0601::M0601;

fn main() -> m0601::Result<()> {
    let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;
    match motor.query()? {
        Some(fb) => println!(
            "{:+} RPM, {:.1}°, {:?} °C, faults: {}",
            fb.speed_rpm, fb.position_deg, fb.temp_c, fb.faults
        ),
        None => println!("no reply — check power, wiring, --id"),
    }
    Ok(())
}
```

`query()` sends a `0x74` feedback frame and parses the reply. It's the only reply
that carries winding temperature.

## `Ok(None)` is not an error

`Ok(None)` means the bus stayed silent — a wrong ID or an unpowered motor, **not
a failure**. `Err` always means the port or OS failed. This distinction runs
through the whole API; handle it explicitly rather than treating "no reply" as an
exception.

```rust
match M0601::open(port, 0x01, timeout) {
    Err(e) if e.is_permission_denied() => {
        eprintln!("add yourself to dialout: sudo usermod -aG dialout $USER");
    }
    Err(e) => eprintln!("open failed: {e}"),
    Ok(motor) => { /* ... */ }
}
```

Telemetry is never rejected on its checksum (`Feedback::crc_ok` is informational;
genuine replies normally have it `true`), and a reply from the wrong motor ID is
dropped, surfacing as `Ok(None)`.
