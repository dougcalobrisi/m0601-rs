---
title: control
weight: 4
---

# `control` — drive from the keyboard

```sh
m0601 control --rpm 100     # full-screen dashboard; F/B drive at ±100
```

`control` is the interactive cockpit: a full-screen dashboard that runs the 50 Hz
drive loop for you and takes live keystrokes. It's what you want when you're bringing
up a motor by hand and reacting to what you see. Its non-interactive twin is
[`drive`]({{< relref "drive" >}}).

> [!CAUTION]
> This starts a live control session with no confirmation. Presets reach 250 RPM on a
> gearless wheel. Clear it first. → [Safety]({{< relref "../safety" >}})

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--rpm` | `100` | the speed the `F` and `B` keys drive at (−330..330) |
| `--accel` | `3` | velocity ramp: `1` = fastest, larger = gentler, `0` = motor default |

`--accel` defaults to `3` rather than the motor's fastest `1` on purpose. A keystroke
here commands a *large instantaneous step* — `F` jumps straight to the full preset,
and `F` → `B` is a complete reversal — and the sharpest ramp can spike current past
the 3 A bus-overcurrent trip on a loaded wheel. Pass `--accel 1` if you want the
snappy response and know the wheel is unloaded.

This is the ramp for *active driving* only. The **stop** ramp is separate: `safe_stop`
uses the library's own moderate `SAFE_STOP_ACCEL`, and `control` always uses that
default ([Stopping safely]({{< relref "../concepts/stopping-safely" >}})).

`control` is the one subcommand that ignores `--timeout` completely — it opens the
port with a fixed 50 ms timeout and its loop uses a fixed 6 ms reply wait, so nothing
you pass globally can stretch a 20 ms cycle.

> [!WARNING]
> **It latches.** Releasing a key does not stop the wheel. `F`, `B`, and `1`–`5` set a
> *sustained* setpoint that holds until you press `S`, `K`, or `Q`, or a signal
> arrives. Do not walk away from a spinning wheel expecting it to stop on its own —
> it stops on those keys, or when the host stops polling entirely (crash, unplug,
> power loss), which coasts it rather than braking it.

## The keymap

```
F/B    forward / backward at the --rpm preset
1-5    50 / 100 / 150 / 200 / 250 RPM
←/→    nudge ±10 RPM (velocity mode only)
S      stop  (see the per-mode note below)
K      electric brake (velocity mode only)
V/C/P  switch to velocity / current / position mode
Q/Esc  quit — forces velocity mode, zeroes, then brakes
```

`F`, `B`, and `1`–`5` don't just change the number they command — if you're not
already in velocity mode, they request a real mode switch first (the dashboard shows
`(switching to VELOCITY first)`). That matters: without it, pressing `F` in current
mode would feed your RPM figure to the motor as a *torque*, while the screen claimed
VELOCITY. The switch keeps the label honest.

### `S` and `K` are mode-aware

This is the part people trip over, and it's a direct consequence of "zero is not
universally stop":

- **In velocity mode**, `S` commands 0 RPM — an actual stop — and `K` engages the
  electric brake.
- **In current mode**, `S` commands zero torque, which is a *coast*, not a brake,
  and it says so: `Zero current — coasting (K cannot brake in current mode)`. `K`
  does nothing here.
- **In position mode**, `S` does *not* send zero — that would mean "rotate to 0°,"
  a potential half-turn. Instead it holds the wheel's current angle (`Holding 187.4
  deg`). If no telemetry has arrived yet it can't know the angle, so it tells you to
  press `V` to stop instead.

`K` only ever brakes in velocity mode; in the other modes the brake byte is ignored
by the motor, so the dashboard refuses rather than pretending.

### Entering position mode is gated

`P` is refused if the wheel is turning at 10 RPM or faster (a protocol constraint),
and also if no telemetry has arrived at all — an unknown speed is not a zero speed,
so `control` fails closed. When it does switch, it seeds the target with the wheel's
*present* angle, so entering position mode never itself commands a move.

## The reported-mode display

The dashboard shows the mode the **motor reports**, and if that ever disagrees with
what you asked for, the mode line turns **red** and shows both: `Mode: VELOCITY
(motor: CURRENT)`. This is a deliberate design choice, not a debug aid. A dashboard
that shows only what you *requested* is exactly how a "brake" keypress ends up
freewheeling a wheel while the screen says BRAKING. Believe the red line.

Similarly, the status word (STATIONARY, SPINNING, BRAKING) comes from the reported
speed, and BRAKING only shows when the motor actually confirms it's in velocity mode
braking. Until the first reply lands you'll see `Waiting for telemetry...`.

Position and temperature are shown from the best data available: the hi-res 16-bit
angle retained from drive replies (rather than flickering to the coarse 8-bit angle
that arrives with the periodic temperature query), and temperature from that query,
`--` until the first one lands.

## Exit braking

`Q`, `Esc`, and `Ctrl-C` all quit by forcing velocity mode, zeroing, and braking —
about 300 ms of frames. So do a panic and a `SIGTERM`/`SIGHUP` (a dropped SSH session
counts). The one thing that can't brake is `SIGKILL` or losing power: nothing runs,
so the motor coasts. That's the protocol fail-safe doing its job, but it means
`kill -9` is not an emergency stop.

Under the hood, the port is owned by a dedicated 50 Hz poll thread while the UI
thread only ever edits shared state — see [Internals]({{< relref "../internals" >}})
if you're curious how the guaranteed-stop-on-exit is wired.

## See also

- [`drive`]({{< relref "drive" >}}) — the same motion, scripted.
- [Concepts → Stopping safely]({{< relref "../concepts/stopping-safely" >}}) — why
  a stop starts by switching modes.
