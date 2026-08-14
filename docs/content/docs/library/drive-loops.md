---
title: Drive loops
weight: 2
---

# Drive loops: you own the cadence

`drive_velocity`, `drive_current`, and `drive_position` each send **one frame** and
return. That's the whole method. Motion is sustained only if you keep calling them at
50 Hz or faster; stop, and the wheel coasts within a couple of cycles. This isn't a
limitation to paper over — it's the protocol's fail-safe, and the API is honest about
it rather than hiding a background thread you can't see.

```rust
use std::time::{Duration, Instant};
use m0601::M0601;

fn main() -> m0601::Result<()> {
    let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;

    // Spin at 100 RPM for three seconds: one frame every 20 ms.
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        motor.drive_velocity(100)?;
        std::thread::sleep(Duration::from_millis(20));
    }

    motor.safe_stop(); // force velocity, zero, brake. Never errors.
    Ok(())
}
```

20 ms per cycle is 50 Hz, the floor. Faster is fine (up to 500 Hz); slower and the
motor starts coasting between frames, which reads as a wheel that stutters or won't
hold speed.

## Always `safe_stop` on the way out

`safe_stop()` is the counterpart to the loop, and you should call it on **every**
exit path — the normal end, an error, a panic, a signal handler. It's built to run
from those places: it returns nothing and swallows I/O errors, so it can't itself
panic or fail partway and leave the wheel driven.

What it actually does is worth knowing, because it explains a design choice you'll
see echoed in the CLI. It forces velocity mode *first*, then sends zero, then brakes.
The mode switch is not incidental: a zero setpoint only means "stop" in velocity
mode. In position mode those same zero bytes mean "rotate to 0°" — a stop command
that could spin the wheel up to half a turn — and in current mode they mean zero
torque, a coast. Since `safe_stop` runs from panic and signal paths where the active
mode isn't knowable, it establishes velocity mode itself rather than assuming.
[Stopping safely]({{< relref "../concepts/stopping-safely" >}}) covers this in full.

And if your process dies before `safe_stop` can run? The wheel coasts, because frames
stopped arriving. Worst case, the fail-safe still catches it.

## Acceleration, and the current spike

`drive_velocity` uses acceleration `1` by default — which is the motor's *fastest*
ramp, not a gentle one. On a loaded wheel a large velocity step at accel 1 can draw a
current spike big enough to trip the 3 A bus-overcurrent protection, which drops the
wheel until it auto-resets ~5 s later. If you see that, ramp softer. Per call:

```rust
motor.drive_velocity_accel(200, 40)?;   // larger accel byte = gentler ramp; 0 = motor default
```

Or change the default `drive_velocity` uses, once, so ordinary calls ramp gently — on
the whole bus or one handle:

```rust
let bus = Bus::open("/dev/ttyUSB0", timeout)?.with_default_accel(10); // every motor
let mut motor = bus.motor(0x01)?.with_default_accel(20);              // just this one
motor.drive_velocity(200)?;   // now uses accel 20; drive_velocity_accel still overrides
```

(The vendor docs give this byte a unit that reads like a rate, which contradicts the
"1 is fastest" direction everyone agrees on. That contradiction is unresolved, so the
crate documents only the direction — see [the protocol notes]({{< relref
"../protocol" >}}).)

## Reading while you drive

You don't have to choose between driving and reading — every drive frame's reply
carries telemetry. `transact` sends a frame and returns the parsed reply in one
exchange, which is the right shape for a loop that both commands and monitors. That's
the subject of the [Telemetry]({{< relref "telemetry" >}}) page.
