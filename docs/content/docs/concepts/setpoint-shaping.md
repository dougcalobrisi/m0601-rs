---
title: Setpoint shaping
weight: 65
---

# Setpoint shaping

There are four different things in this system that can be called "ramping," and
they act at different places. Getting them straight is the difference between a
machine that starts smoothly and one that trips its own overcurrent protection.

## Firmware-side control loops

The M0601 closes velocity, current, and position **in its own firmware**. Selecting a
[`Mode`]({{< relref "../library/modes" >}}) chooses *which of the motor's PIDs* your
drive frame feeds. It does not hand you an open-loop actuator to close a loop around.

This matters because the obvious-sounding advice — "the driver sends the setpoint,
closed-loop control is yours" — quietly invites a second velocity loop stacked on top
of the firmware's, closed over a 50 Hz half-duplex link with no timestamps. That
oscillates, and the failure is hard to diagnose precisely because the architecture
sounds like ordinary separation of concerns. Don't re-close what the motor already
closes. See [Where the driver ends]({{< relref "driver-boundary" >}}) for the full
three-level split.

## The three motor-side ramps

All three act on the **motor** side — they bound how fast the motor chases the
setpoint you last gave it:

| Mechanism | Scope | What it does |
|---|---|---|
| `drive_velocity_accel(rpm, accel)` | one call | the frame's `ACCEL` byte; `0` is the motor default, and [which end of the range is gentle is undocumented]({{< relref "../protocol" >}}#known-contradictions-between-sources) |
| `Bus::with_default_accel(n)` | whole bus | the default every `drive_velocity` uses |
| `BusTiming::stop_accel` | stops | defaults to `0` — the motor's own ramp, rather than a guess at a byte whose direction no source states |

## Host-side ramping: `SlewLimiter`

None of those bound how fast **you** move the setpoint. A keystroke, a joystick snap,
or a mixer output that jumps between cycles is still a step change on the wire, and
the current spike that follows is measured against the motor's 3 A bus-overcurrent
trip.

`SlewLimiter` bounds the setpoint's rate of change on the host side. It holds no
clock — you pass the elapsed time, so the scheduler stays yours and the limiter is
testable with arithmetic instead of sleeps:

```rust
use std::time::Duration;
use m0601::SlewLimiter;

let cycle = Duration::from_millis(20);
let mut ramp = SlewLimiter::new(300.0)?;   // 300 RPM/s => 6 RPM per cycle
let target = 250.0;

// ... once per cycle, in the drive loop:
let rpm = ramp.step(target, cycle).round() as i16;
// motor.drive_velocity(rpm)?;
```

`SlewLimiter::new` **returns a `Result` rather than sanitizing its input**: a zero or
negative rate would freeze the setpoint and a `NaN` rate would silently disable
limiting, and both are far worse discovered at 50 Hz than at startup. Where you can't
propagate an error, `SlewLimiter::GENTLE` is the infallible fallback — it errs toward
a machine that barely moves rather than one that steps. Non-finite *inputs* hold the
current setpoint instead of poisoning it, since a `NaN` reaching the state would latch
into every later cycle.

## The two safety rules

> [!WARNING]
> **Stop paths must bypass it.** On an all-stop, a latched fault, or a dead operator
> link, call `ramp.reset_to(0.0)` and send zero *now*. A fail-safe that ramps is not a
> fail-safe. → [Stopping safely]({{< relref "stopping-safely" >}})

> [!WARNING]
> **A held brake must not let it wind up.** While braking, pin it at `reset_to(0.0)`
> rather than letting it step toward a still-latched throttle. Otherwise releasing the
> brake commands the fully ramped setpoint in a single step — exactly the lurch the
> limiter exists to prevent.

Both are lessons [`m0601-quad`]({{< relref "../samples/quad" >}}) learned the hard
way; its pilot uses `SlewLimiter` and keeps three regression tests pinned on this
behavior.

## Not for position mode

Use it for RPM or amps, not for a position setpoint. A position setpoint is an
absolute angle the motor interpolates to on its own, so slewing it commands a
*different move*, not a gentler one.

## Design rationale

The rule of thumb on the [boundary page]({{< relref "driver-boundary" >}}) is *if it's
about this motor or this shared wire, it's the driver's job*. The binding constraint on
setpoint rate-of-change is the motor's 3 A bus-overcurrent trip — a property of this
motor, not of any particular chassis. Kinematics and outer-loop PID stay out, because
those are properties of a robot.

It also cleared the crate's usual bar for hoisting motor-domain math: two independent
consumers had already written it. That's the same path `deg_to_raw`,
`Telemetry::absorb`, `frame_time`/`drive_floor`/`bus_period`, `PositionAccumulator`,
and `Faults::KNOWN_MASK` all took.
