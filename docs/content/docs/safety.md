---
title: Safety
weight: 30
---

# Safety

A direct-drive hub motor is not a hobby servo. There is **no gearbox** between the
winding and the tyre, 2 N·m of stall torque, and the CLI's presets reach 250 RPM. It
will drive a chassis off a bench, and it will not notice your hand.

This page is the one place all of that is stated plainly. The warnings scattered
through the rest of the docs point here.

## Pre-spin checklist

- **Get the wheel off the ground**, or clear of fingers, cables, and the edge of the
  bench.
- **Secure the chassis** so it can't drive itself away.
- **`control` and `drive` start driving the instant they open.** There is no
  confirmation prompt. The commands that never command motion are `scan`, `info`,
  `monitor`, and `set-id` (it only ever sends address frames). `raw` refuses the
  two command bytes that can move the wheel (`0x64` drive, `0xA0` mode switch)
  unless you pass `--yes` — and with `--yes` it will send whatever you typed. Every
  *other* frame — feedback queries, set-ID address frames, deliberately malformed
  bytes — `raw` sends with no flag and no prompt.

## Braked stops vs. coasts

The distinction that matters most: a **braked** stop is active and fast; a **coast**
is the motor freewheeling to a halt on its own. Both end at zero, but they take very
different distances, and a coasting rover rolls.

| Situation | Result |
|---|---|
| `Q` / `Esc` / `Ctrl-C` in `control` | **braked** |
| `drive` reaching its `--secs` deadline | **braked** |
| A `?` error mid-loop, or a panic | **braked** (guards unwind) |
| `SIGTERM` / `SIGHUP` — e.g. a dropped SSH session | **braked**, *if* the signal handler installed |
| Signal when the handler failed to install | **coast** — the tool warns you at startup |
| `SIGKILL` (`kill -9`) | **coast** |
| Power loss, unplugged adapter, host crash | **coast** |

> [!CAUTION]
> **`kill -9` is not an emergency stop.** It runs no code, so nothing brakes — the
> wheel spins down on its own. If you need a guaranteed hard stop, **cut motor
> power.** Software can only ever be as fast as its ~300 ms braked sequence, and only
> while it is alive to run it.

The coast case is not a defect. It is the protocol's
[fail-safe]({{< relref "concepts/polling-and-failsafe" >}}): the motor moves only
while drive frames keep arriving, so a dead host always resolves to a stopping wheel
rather than a running one. `safe_stop` is the *upgrade* from that coast to an active
braked stop, for the exits where code still runs.

## Four common surprises

**1. A zero setpoint does not mean "stop."** It means stop only in velocity mode. The
identical zero-valued frame commands a move to 0° in position mode — up to half a
turn — and zero torque, a coast, in current mode. This is why `safe_stop` forces
velocity mode *before* it sends anything.
→ [Stopping safely]({{< relref "concepts/stopping-safely" >}})

**2. `control` latches.** Releasing a key does not stop the wheel. `F`, `B`, and
`1`–`5` set a *sustained* setpoint that holds until you press `S`, `K`, or `Q`, or a
signal arrives. Do not walk away from a spinning wheel expecting it to stop itself.

**3. The acceleration byte's direction is unknown — do not lean on it.** A large
velocity step on a loaded wheel can spike current past the 3 A bus-overcurrent
protection and drop the wheel until it auto-resets ~5 s later, and this byte is
supposed to be the lever over that ramp. But **no vendor source states which end of
its range is gentle**, and the upstream manual's only statement about it is a rate
unit under which a *larger* value ramps *harder*
([details]({{< relref "protocol" >}}#known-contradictions-between-sources)). Every
default in this crate is now `0`, the motor's own ramp. On a vehicle, where several
wheels launch off one supply, bound the *step* with `SlewLimiter` — that works
regardless of direction — and treat a nonzero accel byte as something you measured,
not something you assumed.

**4. Believe the reported mode, not the requested one.** `control` shows the mode the
*motor* reports and turns the line red when the two disagree — because a dashboard
showing only your intent is exactly how a "brake" keypress ends up freewheeling a
wheel while the screen says BRAKING.

## `raw` and `set-id`

- **[`raw`]({{< relref "cli/raw" >}})** refuses the two command bytes that can move
  the wheel (`0x64` drive, `0xA0` mode switch) unless you pass `--yes`, and brakes
  the motor the frame addressed on exit when it sends one (a broadcast frame's brake
  covers only `--id`). It still has no loop and no position-mode pre-flight check,
  so it is for inspection and protocol work, not for motion you care about.
- **[`set-id`]({{< relref "cli/set-id" >}})** writes persistent state with an
  *unaddressed* frame: every motor that hears it takes the new ID. It polls all 254
  addresses first and refuses if it sees more than one motor — a guard that runs
  before any prompt, so `--yes` does not bypass it. Renumber one motor at a time,
  physically.

## Working without hardware

You can exercise a surprising amount of this with nothing connected:

- `MockTransport` runs driver logic against a scripted in-memory bus
  ([Testing]({{< relref "library/testing" >}})).
- `m0601-quad drive --dry-run` opens no serial port at all
  ([m0601-quad]({{< relref "samples/quad" >}})).
- The hardware-in-the-loop tests are `#[ignore]`d, and the one test that spins a wheel
  needs `M0601_ALLOW_MOTION=1` on top of that — a separate gate on purpose.

## See also

- [Stopping safely]({{< relref "concepts/stopping-safely" >}}) — why a stop starts by
  switching modes, and how a whole vehicle stops without yawing.
- [Polling and the fail-safe]({{< relref "concepts/polling-and-failsafe" >}}) — why
  the coast exists and why it's the feature.
- [Troubleshooting]({{< relref "troubleshooting" >}}) — when something is already
  wrong.
