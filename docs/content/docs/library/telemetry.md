---
title: Telemetry
weight: 4
---

# Telemetry — the two reply layouts

Every drive frame's reply carries telemetry too. Use `transact` to drive and read
in one exchange, as a 50 Hz loop should:

```rust
use m0601::protocol::frame_velocity;

let frame = frame_velocity(motor.id(), 100, 1);
if let Some(fb) = motor.transact(&frame, std::time::Duration::from_millis(6))? {
    // Drive replies: hi-res position (~0.011°), NO temperature.
    assert!(fb.temp_c.is_none());
    println!("{:+} RPM at {:.2}°", fb.speed_rpm, fb.position_deg);
}
```

The two layouts (see the [protocol reference]({{< relref "../protocol" >}}) for
the exact bytes):

| Reply to | `temp_c` | `position_deg` resolution |
|---|---|---|
| `query()` / `0x74` | `Some(°C)` | ~1.4° (8-bit) |
| drive frame / broadcast | `None` | ~0.011° (16-bit) |

## Pattern for long-running loops

`transact` the drive frame every cycle, and `query()` every ~10th cycle to
refresh temperature — cache the last value. That's exactly what the CLI's
`control` does.

## `Feedback` vs `Telemetry`

- **`Feedback`** is one parsed reply: `id`, `mode`, `current_a`, `speed_rpm`,
  `temp_c` (`Option`), `position_deg`, `faults`, `crc_ok`, and the raw `Frame`.
- **`Telemetry`** is an accumulator. Its `absorb(fb)` merges the two layouts so
  temperature (from queries) and hi-res angle (from drive replies) both persist
  across alternating replies — you always have the freshest of each.

## Faults

`Feedback::faults` is a `Faults` bitmask with named bits (`OVERCURRENT`,
`PHASE_OVERCURRENT`, `STALL`, `OVERHEAT`, `SENSOR_ERR`) and a `Display` impl. See
the [fault byte table]({{< relref "../protocol" >}}) for trip and release
thresholds.
