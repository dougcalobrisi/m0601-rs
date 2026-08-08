---
title: Telemetry
weight: 4
---

# Telemetry: two layouts, one accumulator

Every frame the motor replies to carries telemetry, including drive frames — so a
50 Hz loop can read while it commands. `transact` is the method for that: send a
frame, get the parsed reply back in one exchange.

```rust
use m0601::protocol::frame_velocity;

let frame = frame_velocity(motor.id(), 100, 1);
if let Some(fb) = motor.transact(&frame, std::time::Duration::from_millis(6))? {
    // A drive reply: fine 16-bit angle, but NO temperature.
    assert!(fb.temp_c.is_none());
    println!("{:+} RPM at {:.2}°", fb.speed_rpm, fb.position_deg);
}
```

## The two layouts

The catch is that the motor answers in **two different frame layouts**, and which one
you get depends on the command that asked:

| Reply to | `temp_c` | position resolution |
|---|---|---|
| `query()` / a `0x74` frame | `Some(°C)` | ~1.4°, from a single byte |
| a drive frame / the broadcast | `None` | ~0.011°, from 16 bits |

So the query reply is where temperature lives, and the drive reply is where the
precise angle lives — about 128× finer. Neither carries both. The driver decodes each
reply according to the command that elicited it, so `Feedback` always means the right
thing; you never have to guess which layout you're holding.

## The flicker problem, and `Telemetry`

Now put those together in a real loop: you `transact` a drive frame every cycle for
control, and slip in a `query()` every tenth cycle to refresh temperature. If you
render straight off `Feedback`, temperature *flickers* — present on the tenth-cycle
query reply, `None` on the nine drive replies in between. The angle flickers too,
between coarse and fine.

`Telemetry` fixes this. It's a small accumulator you feed each reply with `absorb`,
and it retains the temperature and the hi-res angle across layouts:

```rust
use m0601::Telemetry;

let mut tel = Telemetry::default();
// in the loop:
if let Some(fb) = motor.transact(&frame, wait)? {
    tel.absorb(fb);
}
// tel.temp_c holds the last real temperature even on cycles whose reply had none;
// tel.position_deg keeps the fine angle rather than downgrading to the 8-bit one.
```

This is exactly what the CLI's `control` and `drive` do to keep their dashboards from
strobing. If you're building any loop that mixes drive frames and periodic queries,
reach for `Telemetry` rather than reconciling the layouts yourself.

## Faults

`Feedback::faults` is a `Faults` newtype over the fault byte, with named bits and a
`Display` that spells them out (`SensorErr | Overcurrent`). Any bit outside the
documented set is shown as hex rather than dropped, so the motor can never report a
fault you don't see. The trip and release thresholds are in the [protocol
reference]({{< relref "../protocol" >}}); the short version is that each protection
auto-resets about five seconds after the condition clears.
