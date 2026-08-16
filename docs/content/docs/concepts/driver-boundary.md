---
title: Where the driver ends
weight: 6
---

# Where the driver ends and your robot begins

This crate is a **motor driver**, not a robot framework. Knowing exactly where it
stops saves you from two opposite mistakes: reaching around it to re-encode things it
already owns, and waiting for it to grow features that are yours to write. This page
draws the line and explains why it sits where it does.

## The line

The driver owns three layers, and nothing above them:

1. **The wire** — the fixed 10-byte frame format, its CRC, the two reply layouts, the
   clamping and scaling of setpoints. All of it lives in the pure, I/O-free
   [`protocol`](https://docs.rs/m0601/latest/m0601/protocol/) module.
2. **The bus** — one half-duplex RS485 port shared by several individually-addressed
   motors, the inter-frame idle gap that keeps their frames from colliding, and the
   round-major group operations that switch or stop every wheel in step. See
   [The bus]({{< relref "the-bus" >}}) and [Stopping safely]({{< relref
   "stopping-safely" >}}).
3. **One motor** — the [`M0601`](https://docs.rs/m0601/latest/m0601/struct.M0601.html)
   handle: send one drive/query/mode frame, decode one reply, flip left/right signs
   for a mirror-image wheel.

Above the single motor and the shared wire, the driver does not go. It has no concept
of your chassis, your control loop, or your mission.

## What you get at the bus level — use it, don't rebuild it

Because the bus is genuinely a protocol concern, the driver already solves the
multi-motor problems that *look* like they'd be yours to solve. Reach for these before
writing your own:

- **Shared-port coordination.** A [`Bus`](https://docs.rs/m0601/latest/m0601/struct.Bus.html)
  mints cheap, cloneable handles that serialize their exchanges through one lock —
  and are `Send` whenever the transport is, as the default `SerialTransport` is, so
  each wheel can live on its own thread. You do not manage the port yourself.
- **The inter-frame gap.** The bus enforces a minimum idle time between frames so a
  drive frame's reply can't collide with the next frame — the single most common
  cause of "one motor mysteriously never moves." Tune it with
  `Bus::with_min_gap`, don't reimplement it.
- **Group stop / group mode.** `safe_stop_all` and `set_mode_all` go round-major, so
  a whole vehicle stops in the same ~300 ms as one motor and doesn't yaw on the way
  down. [Stopping safely]({{< relref "stopping-safely" >}}) explains why.
- **Wire-occupancy budgeting.** `bus_period`, `frame_time`, and `drive_floor` are the
  arithmetic for "will N wheels plus their polls fit in my cycle?" — provided as
  functions so you size the loop, not guess it.
- **Reply-layout decoding, mirroring, and multi-turn position.** The driver decodes
  each reply by the command that elicited it, flips speed/current signs for a mirrored
  wheel, and unwraps single-turn angle into a continuous one. If you find yourself
  hand-building a query frame, re-deriving frame time from the baud rate, or unwrapping
  angle yourself, check the API first — it's probably already there.
- **The odometry aliasing bound.** `PositionAccumulator`'s shortest-arc unwrap is only
  valid while a wheel turns under 180° between samples.
  `PositionAccumulator::max_unaliased_rpm(gap)` turns your poll interval into the exact
  speed ceiling, so you compare against it (or re-baseline on a long gap) instead of
  re-deriving the `30 / gap` relationship yourself.
- **Setpoint slew limiting.** `SlewLimiter` bounds how fast a setpoint may change, so a
  keystroke or a joystick snap becomes a ramp instead of a step that spikes current into
  the 3 A trip. It carries the two details that are easy to get wrong: stop paths must
  *bypass* the limit (`reset_to(0.0)`), and a held brake must not let it wind up toward
  a latched throttle, or release becomes the very lurch it was meant to prevent.
- **The set of defined fault bits.** `Faults::KNOWN_MASK` and `Faults::unknown_bits()`
  are the single source of truth for which fault bits this driver understands. Classify
  against them rather than hardcoding a mask like `0x1F`, so a fault bit added to a
  future firmware (and this driver) updates your code for free.

## What the driver leaves to you — on purpose

These are **not** missing features. Each is left out because baking in one answer
would be wrong for some robot:

- **Kinematics / wheel mixing.** Turning a body command (throttle+turn, or a
  velocity/yaw twist) into per-wheel setpoints is a *vehicle-class* decision —
  skid-steer, Ackermann, mecanum, and holonomic all mix differently. The driver speaks
  in per-wheel RPM/current/position and lets your vehicle layer own the mix.
- **Outer-loop control.** Not closed-loop control in general — read the split below
  before you write a PID. What is yours is the loop over a *robot* quantity: heading
  hold, odometry-driven distance, path following, station keeping. The driver sends
  exactly the setpoint you give it, and knows nothing about where your machine is.
- **The drive loop and its threading model.** The driver sends *one* frame per call;
  sustaining motion at ≥50 Hz — and how you thread and pace that loop — is the
  application's, so the driver drags in no runtime or scheduler. See [Polling and the
  fail-safe]({{< relref "polling-and-failsafe" >}}).
- **Config parsing.** Which motor ID sits at which corner, and where your limits come
  from, is your schema. The driver takes plain values (and gives you `BusTiming` to
  fill in), but reads no files.
- **Safety *policy*.** The driver gives you the raw fault bits and an active braked
  stop mechanism. Deciding what a stale reading or a stall bit *means* for your
  machine — warn, coast, hard-stop, re-arm — is policy that belongs with the robot.
- **The position-mirror convention.** Mirroring flips speed/current signs, but whether
  a mirror-image wheel's reported *angle* should be reflected (and how) depends on your
  mechanical build, so it's opt-in rather than assumed.

## Where control actually splits

"Closed-loop control is yours" is too blunt to be useful, and taken literally it leads
people into a real mistake. Control over an M0601 sits at three levels, and only the
outermost is yours:

1. **The inner loop — the motor's, already closed.** Velocity, current and position are
   the motor's own firmware loops (that is what [`Mode`](https://docs.rs/m0601/latest/m0601/enum.Mode.html)
   selects). When you send `100` in velocity mode you are handing a setpoint *to a
   running PID*, not driving a duty cycle. **Do not wrap a second host-side loop around
   the same variable** — a velocity PID on top of the motor's velocity PID, closed over
   a 50 Hz half-duplex link with no timestamps, fights the loop underneath it and tunes
   into oscillation. If a wheel is not holding its commanded RPM, that is a load,
   supply, or fault question, not a missing gain.
2. **Setpoint shaping — shared.** How fast the setpoint may *move* is a property of this
   motor, because the binding constraint is its 3 A bus-overcurrent trip, so the driver
   owns it:
   - the drive frame's `accel` byte, per call via `drive_velocity_accel` or as a default
     via `Bus::with_default_accel` — the motor's own ramp toward the setpoint;
   - `BusTiming::stop_accel` — the same ramp on the way down, defaulted to a moderate
     value so a hard stop can't trip the protection mid-stop;
   - [`SlewLimiter`](https://docs.rs/m0601/latest/m0601/struct.SlewLimiter.html) — a
     bound on how fast *you* move the setpoint, for the step changes a keystroke, a
     joystick snap, or a mixer output produces. It holds no clock; you pass the elapsed
     time.
3. **The outer loop — yours.** Anything closed over a robot-level quantity: heading,
   pose, distance travelled, a path, a mission. That needs odometry, a frame convention
   and a clock, all of which are the application's. The driver gives you the raw
   ingredients (`PositionAccumulator`, `Feedback`) and stops there.

The short version: **don't re-close the motor's loops, do use the driver's shaping, and
own everything above the wheel.**

## The seams

When you do need to reach past the defaults, use the extension points rather than
forking:

- **`Transport`** — the trait under `Bus`/`M0601`. Swap in a mock, a simulator, or your
  own scheduler; `MockTransport` drives every test in this crate with no hardware.
- **`BusTiming`** — one struct for every pacing/stop tunable, filled from your own
  config.
- **The pure `protocol` module** — every frame builder, parser, and scaler is public
  and I/O-free, so you can bypass the handles entirely and drive your own transport
  with just `protocol` and the data types.

The rule of thumb: if it's about *this motor* or *this shared wire*, it's the driver's
job — call it. If it's about *this robot*, it's yours — and the driver is built to get
out of your way.

## See also

- [The bus]({{< relref "the-bus" >}}) — the half-duplex reasoning behind the shared
  port.
- [Stopping safely]({{< relref "stopping-safely" >}}) — the round-major group stop.
- The library's [Multi-motor bus]({{< relref "../library/multi-motor" >}}) page — the
  same boundary from the API side.
