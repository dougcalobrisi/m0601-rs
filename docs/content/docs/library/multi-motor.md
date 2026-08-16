---
title: Multi-motor bus
weight: 6
---

# Multi-motor robots

RS485 is multi-drop: every wheel shares one A/B pair, each answering to a unique
address. A `Bus` owns that shared port and mints per-motor handles that are cheap to
clone and safe to move between threads — they all funnel through one physical port,
with the coordination handled for you.

```rust
use std::time::Duration;
use m0601::Bus;

fn main() -> m0601::Result<()> {
    let bus = Bus::open("/dev/ttyUSB0", Duration::from_millis(150))?;
    let mut left  = bus.motor(0x01)?.mirrored(true); // FIT1042, left side
    let mut right = bus.motor(0x02)?;                // FIT1038, right side

    // Mirrored, "+100" drives the robot forward on both sides.
    left.drive_velocity(100)?;
    right.drive_velocity(100)?;   // ...and resend both at ≥50 Hz
    Ok(())
}
```

## Mirroring

On a two-sided robot the left and right wheels face opposite ways, so "forward" is
+RPM on one and −RPM on the other. `mirrored(true)` hides that: it negates
velocity and current *setpoints* on the way out and flips the *signs* of reported
speed and current on the way in, so your code speaks in robot-forward terms and both
sides agree.

Position values pass through untouched **by default** — the right mirror transform for
an angle depends on your mechanical convention (is mirrored-90° equal to 270°, or
still 90°?), so the driver won't guess. It will do it on request, though:

```rust
use m0601::PositionMirror;

let mut right = bus.motor(0x02)?
    .mirrored(true)
    .position_mirror(PositionMirror::Reflect);   // default is PassThrough
```

`PositionMirror::Reflect` reflects the reported angle about 0° — a reported `θ`
becomes `(360 − θ) mod 360` — so a mirror-image wheel's angle counts up in the same
direction its sign-flipped speed does. `PassThrough` (the default) leaves the wire
angle alone. Pick the one that matches how your wheel is actually mounted;
`position_mirror_mode()` reads the setting back. Note this affects *reported* angle
only: `drive_position` setpoints are never mirrored either way.

`Feedback::raw` always holds the untouched wire bytes regardless, if you need the
ground truth. (A small tell that someone sweated the details: a mirrored zero current
comes back as `+0.0`, not the `-0.0` that would print as `-0.000 A`.)

## Frame spacing across handles

The two `drive_velocity` calls above go out one after another with no explicit delay,
and that's fine — but only because the bus inserts a gap for you. It has to. Every
drive frame elicits a reply even when nothing reads it, so two drive frames sent with
no gap would put the second on the wire while the first's reply is still transmitting.
Both corrupt. In a periodic loop the *same* frame corrupts every cycle, and the
symptom is maddening: one motor simply never moves while everything else works.

The bus enforces a minimum idle gap (2.5 ms by default) between frames to keep the
reply one frame elicits clear of the next. It's a property of the shared port, so it
holds across every cloned handle and every thread. `Bus::with_min_gap` tunes it —
set it from a turnaround you measured, not a guess, and set it once at open time
since there's one gap per physical bus, not one per handle. [The bus]({{< relref
"../concepts/the-bus" >}}) covers the reasoning and the multi-motor timing budget.

## Group stops

When you stop a vehicle, stopping the wheels *one at a time* is a bug. On a skid-steer
chassis, one braked wheel against three still-coasting ones is an uncommanded yaw —
the robot turns as it stops. `Bus::safe_stop_all` avoids that by going round-major:
it sends each step of the stop sequence to *every* motor, then the next step to every
motor, so all the wheels stop in step. N motors take the same ~300 ms as one.

```rust
bus.safe_stop_all(&[0x01, 0x02, 0x03, 0x04]);   // best-effort, errors swallowed
bus.set_mode_all(&[0x01, 0x02, 0x03, 0x04], Mode::Velocity)?;
```

`safe_stop_all` is best-effort by design — it runs on shutdown paths, where "keep
telling the other motors to stop" beats bailing on the first error. `Bus` is `Clone`,
so a signal handler or stop guard can hold its own handle to the same port and stop
everything from there.

## Budgeting the bus

Each motor needs *its* drive frame at ≥50 Hz, so N motors put at least N×50 frames a
second through one bus, plus the replies, plus the gaps. Four wheels at the crate
defaults is **~13.5 ms** of bus occupancy per 20 ms cycle before you read any
telemetry. Don't re-derive that by hand; [`bus_period`]({{< relref "budgeting" >}})
computes it, and the budgeting page holds the worked derivation.
That's workable, but it means you should keep reply waits short (6 ms, like the CLI),
read telemetry round-robin — one motor per cycle, not all four — and never *replace* a
drive frame with a query, or that motor coasts through the hole. Don't run a full
`scan` alongside a drive loop on the same bus either; the scan holds the wire for
~254 timeouts and starves the loop.

## See also

- [Concepts → The bus]({{< relref "../concepts/the-bus" >}}) — the half-duplex
  reasoning behind all of this.
- [Concepts → Stopping safely]({{< relref "../concepts/stopping-safely" >}}) — the
  round-major stop in detail.
