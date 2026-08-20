---
title: Drive loops
weight: 2
---

# Drive loops

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

## Stopping on exit

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

## Acceleration

`drive_velocity` uses acceleration `1` by default — which is the motor's *fastest*
ramp, not a gentle one, and so is `0`, which selects the motor default and
[measures the same]({{< relref "../protocol" >}}#known-contradictions-between-sources).
On a loaded wheel a large velocity step at that ramp can draw a current spike big
enough to trip the 3 A bus-overcurrent protection, which drops the wheel until it
auto-resets ~5 s later. If you see that, ramp softer — larger is gentler, but keep it
small: 120 RPM takes ~0.45 s at `1`, ~2 s at `5`, and over 3 s at `20`. Per call:

```rust
motor.drive_velocity_accel(200, 5)?;    // larger = gentler; 0 and 1 are the fastest
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

## Setpoint ramping: `SlewLimiter`

`accel` ramps the motor toward whatever setpoint it was last given; it does **not**
bound how fast *you* move that setpoint. A keystroke, a joystick snap, or a mixer
output that jumps between cycles is still a step change on the wire.

`SlewLimiter` bounds the setpoint's rate of change. It holds no clock — you pass the
elapsed time per `step`, so the scheduler stays yours. The worked example, the
constructor's error contract, the two safety rules (stop paths must bypass it; a held
brake must not let it wind up), why it's for RPM/amps but not position, and why this
is the driver's job at all, all live in one place:
[Setpoint shaping]({{< relref "../concepts/setpoint-shaping" >}}).

## Telemetry inside the loop

You don't have to choose between driving and reading — every drive frame's reply
carries telemetry. `transact` sends a frame and returns the parsed reply in one
exchange, which is the right shape for a loop that both commands and monitors. That's
the subject of the [Telemetry]({{< relref "telemetry" >}}) page.
