---
title: Drive loops
weight: 2
---

# Drive: your loop provides the 50 Hz cadence

The `drive_*` methods send **one frame each**. Motion is sustained only while you
resend at ~50 Hz (≤ every 20 ms). Stop, and the wheel coasts.

```rust
use std::time::{Duration, Instant};
use m0601::M0601;

fn main() -> m0601::Result<()> {
    let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;

    // Spin at 100 RPM for 3 seconds: one frame every 20 ms.
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        motor.drive_velocity(100)?;
        std::thread::sleep(Duration::from_millis(20));
    }

    motor.safe_stop(); // force velocity mode, zero, brake. Never errors.
    Ok(())
}
```

## Always `safe_stop` on the way out

Call `safe_stop()` on **every** exit path of a control loop — including panic and
signal handlers. It swallows I/O errors precisely so it can run there. If your
process dies anyway, the motor coasts to a stop on its own once frames stop
arriving.

`safe_stop` forces velocity mode first, then zeroes, then brakes — because a zero
setpoint only means "stop" in velocity mode.

## Acceleration

`drive_velocity` uses acceleration `1` — the motor's **fastest** ramp. A big step
at accel 1 on a loaded wheel can spike current into the 3 A protection. Use
`drive_velocity_accel(rpm, accel)` with a larger value (larger = gentler; `0` =
motor default) to ramp gently.

## Reading while you drive

Every drive frame's reply carries telemetry too. Use `transact` to drive and read
in one exchange — see [Telemetry]({{< relref "telemetry" >}}).
