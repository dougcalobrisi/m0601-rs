---
title: Odometry
weight: 5
---

# Odometry

The motor's position reading is **single-turn absolute**: it tells you where in a
revolution the wheel is, and it wraps from 359.99° back to 0° every turn. That's
exactly wrong for distance travelled — naïvely differencing samples gives you a −360°
jump each revolution.

`PositionAccumulator` unwraps it into a continuous angle:

```rust
use m0601::PositionAccumulator;

let mut odo = PositionAccumulator::new();
// in the poll loop:
if let Some(fb) = motor.query_with(reply_wait)? {
    let travelled_deg = odo.update(fb.position_deg);
    println!("{travelled_deg:.1}° ({:.2} rev)", odo.revolutions());
}
```

| Method | What it does |
|---|---|
| `update(deg)` | feed one absolute sample; returns the updated continuous angle |
| `update_raw(u16)` | same, from a raw drive-reply reading (`0..=32767`) |
| `cumulative_deg()` | the running total without feeding a sample |
| `revolutions()` | the same value as whole and fractional turns |
| `reset()` | forget the reference and zero the total |

Three properties worth knowing:

- **It measures motion, not heading.** The first sample establishes the reference and
  returns `0.0`. The accumulator tells you how far the wheel has turned *since it
  started watching*, not where the wheel is.
- **It accumulates in `f64`.** At ~100 000° an `f32` has only ~0.008° of resolution
  and small deltas start rounding away visibly; `f64` stays exact well past any
  realistic mission.
- **A non-finite sample is ignored**, not integrated. One `NaN` can't corrupt the
  total or panic the loop.

## The aliasing bound

The unwrap works by taking the **shortest arc** between consecutive samples, folding
the difference into `(-180°, +180°]`. That is a guess, and it's only right while the
wheel actually turns **less than 180° between samples**. Turn further and the
shortest arc points the wrong way: the accumulator confidently reports slow motion
backwards.

Don't derive that threshold by hand — the crate does it:

```rust
use std::time::Duration;
use m0601::PositionAccumulator;

// A 20 ms poll resolves up to 1500 RPM — far above the motor's 330 RPM ceiling.
let ceiling = PositionAccumulator::max_unaliased_rpm(Duration::from_millis(20));
```

`max_unaliased_rpm(gap)` returns the speed at which the wheel travels exactly 180°
per `gap` — i.e. `30 / gap_secs` RPM. Compare it against your loop's own speed limit,
and you know whether your odometry *can* alias at all.

At a 20 ms cycle the answer is comfortable: 1500 RPM against a motor that tops out at
330. The bound stops being theoretical when the **gap grows** — round-robin polling
across four wheels, a thinned poll cadence, a scheduler hiccup, or a reconnect after a
silent stretch. A wheel polled every 8th cycle at 18 ms is sampled every ~144 ms, and
`max_unaliased_rpm(144 ms)` ≈ 208 RPM, which a 330 RPM wheel can exceed.

So: compare against `max_unaliased_rpm` for your *actual* per-wheel sample interval,
not your cycle time — and `reset()` the accumulator after any gap long enough to have
aliased, rather than integrating a delta you can't trust.

> [!TIP]
> This is also the strongest argument for the
> [strict-CRC opt-in]({{< relref "quickstart" >}}): a single corrupt position sample
> is permanent in an integrator, where a *dropped* one costs nothing.

## See also

- [Telemetry]({{< relref "telemetry" >}}) — which reply layout carries the fine
  16-bit angle, and the `Telemetry` accumulator that retains it.
- [Budgeting the wire]({{< relref "budgeting" >}}) — what sets your real sample
  interval.
