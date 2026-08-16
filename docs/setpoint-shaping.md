# Setpoint shaping and the control boundary

August 2026. Ahead of the first crates.io release, we revisited the question of
whether closed-loop control belongs in the driver. The conclusion was **no PID —
but the boundary doc was wrong about why**, and one primitive was genuinely
missing.

## What prompted it

`docs/content/docs/concepts/driver-boundary.md` told consumers:

> **Closed-loop control.** Velocity/position PID, ramping, and setpoint smoothing
> are yours. The driver sends exactly the setpoint you give it.

That sentence was inaccurate in two ways, and one of them was actively harmful.

**1. It invited a nested loop.** The M0601 closes velocity, current and position
in its own firmware — `Mode` selects *which of the motor's PIDs* the drive frame
feeds. A reader who took "velocity PID is yours" at face value would build a
second velocity loop on top of the first, closed over a 50 Hz half-duplex link
with no timestamps, and tune it into oscillation. The failure mode is
non-obvious precisely because the advice sounded like ordinary separation of
concerns.

**2. It contradicted the code.** The driver already shipped three ramping
mechanisms: the per-call `accel` byte, `Bus::with_default_accel`, and
`BusTiming::stop_accel` — the last defaulted to `5` specifically so a hard
ramp-to-zero could not trip the motor's 3 A protection mid-stop.

## What changed

- **Rewrote the boundary page's control section** into an explicit three-level
  split: the motor's inner loops (already closed — do not re-close them),
  setpoint shaping (shared, and mostly the driver's), and the outer loop over a
  robot-level quantity (yours). The old bullet became "Outer-loop control".
- **Added `m0601::SlewLimiter`** — a pure, allocation-free, panic-free
  first-order slew-rate limiter for setpoints.
- **Adopted it in `m0601-quad`**, replacing the hand-rolled `ramped: [f32; 4]`
  in the pilot.
- Documented it in `USAGE.md`, compile-checked via
  `m0601/examples/usage_doc_check.rs`.

## Why a slew limiter is the driver's job

Two independent consumers had already written the same thing: `m0601-quad`'s
pilot and a downstream consumer. That is the crate's established criterion
for hoisting motor-domain math — the same path `deg_to_raw`, `Telemetry::absorb`,
`frame_time`/`drive_floor`/`bus_period`, `PositionAccumulator` and
`Faults::KNOWN_MASK` all took.

It also satisfies the boundary page's own rule of thumb: *if it's about this
motor or this shared wire, it's the driver's job.* The binding constraint on how
fast a setpoint may move is the motor's 3 A bus-overcurrent trip — a property of
this motor, not of any particular chassis. Kinematics and outer-loop PID remain
out, because those are properties of a robot.

Doing it before the first publish also made it free: the workspace was still
`0.1.0` with no tags, so this was not a semver event.

## Design notes

- **No clock.** `step` takes the elapsed time from the caller. The driver owns no
  scheduler and no `Clock` seam — that carve-out was set when `frame_time` /
  `drive_floor` / `bus_period` were hoisted, and it is what lets the limiter be
  tested with arithmetic instead of sleeps.
- **`new` returns `Result`, it does not sanitize.** A zero or negative rate
  freezes the setpoint and a `NaN` rate silently disables limiting. Both are much
  worse discovered at 50 Hz than at startup, so they are refused.
  `SlewLimiter::GENTLE` exists for callers that cannot propagate an error; it errs
  toward a machine that barely moves, never toward an unramped step.
- **Non-finite inputs hold rather than poison.** A `NaN` target returns the
  current setpoint, mirroring the guard in `m0601-quad/src/mix.rs`. In a drive
  loop a `NaN` that reached the state would latch into every later cycle.
- **`reset_to` carries the earned knowledge.** Its doc comment records the two
  things `m0601-quad` learned the hard way: stop paths must *bypass* the limit,
  and a held brake must be pinned at zero so release does not command the fully
  ramped setpoint in one step — the exact lurch the limiter exists to prevent.
- **Not for position mode.** A position setpoint is an absolute angle the motor
  interpolates to itself; slewing it commands a different move, not a gentler one.

## Behavioral equivalence in `m0601-quad`

The pilot's ramp semantics are unchanged, and its three regression tests pass
untouched: `setpoints_ramp_instead_of_stepping`, `all_stop_is_not_rate_limited`,
and `brake_release_ramps_from_zero_not_from_the_latched_throttle`. The ramp still
advances by the *nominal* cycle rather than measured elapsed time — a scheduling
overrun must not license a larger setpoint step, and repeated overruns are
already handled as a fault (`MAX_OVERRUNS`). The latched-trip path resets the
limiters rather than decaying through them.
