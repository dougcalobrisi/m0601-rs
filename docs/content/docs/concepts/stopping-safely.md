---
title: Stopping safely
weight: 4
---

# Stopping safely

Stopping a motor sounds like it should be the easy part — send zero. It isn't,
because on this motor zero doesn't mean what you'd assume, and because a stop often
has to run from the worst possible moment: a panic, a signal, a dropped connection.

## The mode switch before the stop

A zero setpoint means "stop" only in velocity mode. The identical zero-valued drive
frame means "rotate to 0°" in position mode — a stop command that could spin the wheel
up to half a turn on its way to "stopping" — and in current mode it means zero torque,
a coast, with the brake byte ignored entirely.

Now consider where a stop gets called from. A panic handler. A `SIGTERM`. A stop guard
unwinding mid-operation. In none of those places does the code reliably know what mode
the motor is currently in. So `safe_stop` doesn't assume — it *establishes* velocity
mode first, then sends zero, then brakes. Force the one mode where zero means stop,
and the sequence is correct no matter what the motor was doing when things went wrong.

The full sequence is five velocity-mode switch frames, then five zero-velocity frames,
then five brake frames, 20 ms apart — about 300 ms.

Both phases were measured (`stop_ramp_capture`, `m0601/tests/hardware.rs`), stopping an
unloaded wheel from 120 RPM:

| after 100 ms of… | speed left | peak current |
|---|---|---|
| nothing (coasting) | 119 RPM | 0.39 A |
| velocity-0 rounds | ~64 RPM | ~0.6 A |
| brake rounds | 13 RPM | **2.0 A** |

Two things follow, and both correct what this page used to say.

**The velocity-0 rounds do real work, but the `stop_accel` byte does not.** Coasting
sheds essentially nothing in 100 ms while the velocity-0 rounds shed nearly half the
speed — so the ramp phase earns its place. But sweeping `stop_accel` across its entire
range, `0` to `255`, moves that result by about 1 RPM. The full deceleration curves at
`1` and `255` are identical sample for sample. The accel byte shapes *acceleration*
only; on this firmware it is inert on the way down. Any advice to pick a gentler stop
ramp — including advice this page used to give — has no effect.

**The current comes from the brake, not the ramp.** The velocity-0 rounds draw well
under an amp and fall to essentially zero once the wheel is slowing; the brake rounds
draw ~2 A unloaded, two-thirds of the way to the 3 A trip. If a stop ever trips
overcurrent mid-sequence, the brake is the phase to suspect, and `stop_accel` is not the
lever on it.

The brake rounds still deliver the firm final hold, and they are what actually brings the
wheel to rest: velocity-0 alone takes ~370 ms, the brake ~250 ms, and coasting more than
six seconds. One unloaded motor on one firmware — under load the numbers will move, but
an inert byte is unlikely to become live.

And it's best-effort: it swallows every I/O error and keeps sending, because even total
failure is safe. If not one frame gets through, the wheel still coasts to a stop,
because the frames stopped arriving. The fail-safe is the floor under everything.

### Tuning the stop ramp

The stop ramp, the 20 ms round gap, and the mode/set-ID/broadcast waits are all fields
of `BusTiming`, set once on the bus (they default to the values above, so an
unconfigured bus behaves exactly as described). `stop_accel` is kept and still sent
despite measuring inert, because one motor on one firmware is not every motor — but do
not expect changing it to do anything:

```rust
use m0601::{Bus, BusTiming};

// One field at a time…
let bus = Bus::open("/dev/ttyUSB0", timeout)?.with_stop_accel(3);

// …or the whole struct, e.g. straight from your own config.
let bus = Bus::open("/dev/ttyUSB0", timeout)?
    .with_timing(BusTiming { stop_accel: 3, ..BusTiming::default() });
```

Like the idle gap, the timing lives on the shared bus: set it at open time and every
motor handle you mint from the bus uses it.

## Vehicle-wide stops

Stop four wheels one at a time and you've built a bug. Braking wheel 1 while wheels
2–4 still coast means, on a skid-steer chassis, one side biting while the other rolls
— an uncommanded yaw. The robot turns as it "stops."

`safe_stop_all` refuses to do that. It goes **round-major**: it sends step one of the
sequence to every motor, then step two to every motor, and so on — five velocity-mode
rounds, five zero rounds, five brake rounds. Every wheel gets the same command at
nearly the same moment, so they spin down together and the whole vehicle takes the same
~300 ms a single wheel would. (With enough motors that a round can't fit in its 20 ms
window, a round just runs long and the next starts late — the stop still completes, it
only takes a bit more than 300 ms.)

Like the single stop, it's best-effort and swallows errors: it runs on shutdown paths
where "keep telling the *other* motors to stop" beats bailing out on the first one that
didn't answer. And because `Bus` is `Clone`, a signal handler can hold its own handle
and stop the entire vehicle from outside the normal control flow.

## The limits of software stops

Every graceful and not-so-graceful exit brakes: normal completion, `?` errors, panics,
`Ctrl-C`, `SIGTERM`, `SIGHUP`. The exceptions are `SIGKILL` and losing power, where no
code runs at all. There the motor coasts, by protocol — which is safe, but worth
saying plainly: **`kill -9` is not an emergency stop.** It ends the process that was
holding the wheel at speed and lets it spin down on its own, rather than braking it. If
you need a hard stop, cut motor power; the software can only ever be as fast as its
~300 ms braked sequence, and only while it's alive to run it.
