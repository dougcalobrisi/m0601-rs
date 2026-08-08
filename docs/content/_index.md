---
title: m0601
type: docs
---

# m0601

A Rust driver and command-line tool for the DFRobot **M0601** direct-drive hub
motor, spoken over half-duplex RS485. One crate (`m0601`) you build robots on top
of, one binary (`m0601`) you point at a motor to make it move.

The M0601 is a rebadged Direct Drive Tech M0601C-111. DFRobot sells it as two
mirror-image SKUs — **FIT1042** (left) and **FIT1038** (right) — that are
electrically identical and speak the same protocol, so one driver covers both.

## Start here

If the motor is on your bench right now, go straight to the
[first-spin tutorial]({{< relref "docs/tutorial" >}}) — it takes you from bare
wires to a spinning wheel and back to a safe stop.

Otherwise pick a track:

### Drive it from the command line

`scan` the bus, watch telemetry with `monitor`, hold a setpoint with `drive`, or
take the wheel yourself in the full-screen `control` dashboard. No code required.
→ [CLI guide]({{< relref "docs/cli" >}})

### Build on the library

`M0601::open`, `query`, `drive_velocity`, done — a single motor is a dozen lines.
A shared `Bus` fans out to cheap per-motor handles for multi-wheel robots, with
group stops and left/right mirroring built in. → [Library guide]({{< relref "docs/library" >}})

### Understand why it behaves the way it does

The [Concepts]({{< relref "docs/concepts" >}}) section is the interesting part:
the polling fail-safe, why frames need spacing on a half-duplex wire, how one reply
can decode two ways, and why a stop starts by switching modes.

## The one thing to internalize

This is a **polling** protocol, and it is not Modbus. A drive command does not
latch. The motor moves only while drive frames keep arriving at **≥50 Hz**; stop
sending and the wheel coasts. That is not a bug to work around — it is the
protocol's fail-safe. If your program crashes, the adapter falls out, or the power
drops, the wheel spins down instead of running away.

The corollary bites people, so here it is up front: **a zero setpoint does not mean
"stop."** It means stop only in velocity mode. The identical zero-valued frame
commands a move to 0° in position mode and zero torque — a coast — in current mode.
Everything the driver does around stopping (`safe_stop` forcing velocity mode first,
`control`'s `S` key behaving differently per mode) follows from this one fact.

> [!WARNING]
> **Before you spin anything:** `control` and `drive` start driving the instant they
> open, with no confirmation prompt. A direct-drive hub motor has no gearbox and the
> presets reach 250 RPM. Clear the wheel and secure the chassis so it can't drive
> itself off the bench.
