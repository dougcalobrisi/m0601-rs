---
title: Quickstart
weight: 1
---

# Quickstart

The smallest useful program: open the port, ask the motor how it's doing, print it.

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
        None => println!("no reply — check 18 V power, wiring (brown → GND), A/B, --id"),
    }
    Ok(())
}
```

`query()` sends a `0x74` feedback frame and parses the reply. It's the only reply
that carries winding temperature, which is why `fb.temp_c` is populated here and
comes back `None` from a reply to a drive frame ([Telemetry]({{< relref "telemetry"
>}}) has the details).

## `Ok(None)` versus `Err`

The two arms above aren't symmetric, and the distinction runs through the whole
crate. `Ok(None)` means the bus stayed silent — a wrong address, an unpowered motor,
a probe to an empty slot. That's an ordinary outcome on RS485, not a failure, and
the driver never manufactures an error for it. `Err`, by contrast, always means the
port or the OS failed: the device vanished, permission was denied, a write returned
an I/O error.

So the idiomatic shape is: `?`-propagate the real failures, and pattern-match the
`Option` for presence.

```rust
match M0601::open(port, 0x01, timeout) {
    Err(e) if e.is_permission_denied() => {
        eprintln!("add yourself to dialout: sudo usermod -aG dialout $USER");
    }
    Err(e) => eprintln!("open failed: {e}"),
    Ok(motor) => { /* ... */ }
}
```

`is_permission_denied()` is there precisely so you can give the `dialout` hint the
CLI gives, instead of surfacing a bare OS error to your users.

## Telemetry trust and strict CRC

**By default, telemetry is never rejected on its checksum.** `Feedback::crc_ok` is
informational — genuine replies normally have it `true` — but the driver hands you the
reading either way and lets you decide. What it *does* silently drop is a reply
carrying the wrong motor's ID, which surfaces as `Ok(None)` rather than as one motor
reporting its neighbour's speed. That guard matters more than the CRC on a shared bus;
see [Telemetry and echo]({{< relref "../concepts/telemetry-and-echo" >}}).

The reason the default is advisory rather than strict is caution, not observed
failures: the hardware capture in the [protocol reference]({{< relref "../protocol"
>}}) shows genuine replies *do* carry a valid CRC-8/MAXIM (earlier reports to the
contrary traced to echo contamination), but other firmware revisions may differ, and
dropping a real reading is its own hazard. But where a *corrupt* frame is worse than a *missing*
one — above all feeding an odometry integrator, which one bad position sample can
throw off permanently — flip it:

```rust
// Set it on the bus BEFORE minting handles: the flag is copied into each
// M0601 at `motor()` time, exactly as the timeout is.
let bus = Bus::open("/dev/ttyUSB0", timeout)?.with_strict_crc(true);
let mut motor = bus.motor(0x01)?;

let mut other = bus.motor(0x02)?.with_strict_crc(false); // override one handle
other.set_strict_crc(true);                              // or flip it later
```

With strict CRC on, a reply whose checksum doesn't verify becomes `Ok(None)` — the
same "no reply" outcome as silence, so none of your error handling changes shape. The
pure parser behind it is `protocol::parse_feedback_strict`, alongside the lenient
`protocol::parse_feedback`, if you decode frames yourself.
