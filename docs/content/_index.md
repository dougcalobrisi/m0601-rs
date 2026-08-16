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

### Read working code

`m0601/examples/four_wheel_minimal.rs` is the whole driver on one screen — a bus,
four mirrored handles, a stop guard, a 50 Hz loop. `m0601-quad` is that same wiring
grown into a real skid-steer rover, and its `--dry-run` mode runs with no hardware.
→ [Sample code]({{< relref "docs/samples" >}})

### Decide whether to build on it

[Where the driver ends]({{< relref "docs/concepts/driver-boundary" >}}) draws the line
between what this crate owns — the wire, the shared bus, one motor — and what it
deliberately leaves to your robot: kinematics, closed-loop control, config, safety
policy. Read it before you plan around a feature that isn't coming.

### Understand why it behaves the way it does

The [Concepts]({{< relref "docs/concepts" >}}) section is the interesting part:
the polling fail-safe, why frames need spacing on a half-duplex wire, how one reply
can decode two ways, and why a stop starts by switching modes.

## Three facts that explain almost everything

Nearly every design choice in this driver falls out of three properties of the motor.
Read these once and the rest of the docs stop being surprising.

**1. It's a polling protocol, and it is not Modbus.** A drive command does not latch.
The motor moves only while drive frames keep arriving at **≥50 Hz** (up to 500 Hz);
stop sending and the wheel coasts. That is not a bug to work around — it is the
protocol's fail-safe. If your program crashes, the adapter falls out, or the power
drops, the wheel spins down instead of running away. The CLI's `control` and `drive`
run that loop for you; the library's `drive_*` methods send exactly one frame each and
leave the cadence to you.

**2. Zero is not universally "stop."** The corollary bites people, so here it is up
front: a zero setpoint means stop only in velocity mode. The identical zero-valued
frame commands a move to 0° in position mode and zero torque — a coast — in current
mode. Everything the driver does around stopping (`safe_stop` forcing velocity mode
first, `control`'s `S` key behaving differently per mode) follows from this one fact.

**3. Telemetry comes in two layouts.** A reply to a `0x74` query carries winding
temperature and a coarse 8-bit angle. A reply to a drive frame carries a fine 16-bit
angle and no temperature. The same bytes decode differently depending on which command
asked for them — see [Telemetry and echo]({{< relref
"docs/concepts/telemetry-and-echo" >}}).

> [!WARNING]
> **Before you spin anything:** `control` and `drive` start driving the instant they
> open, with no confirmation prompt. A direct-drive hub motor has no gearbox and the
> presets reach 250 RPM. Clear the wheel and secure the chassis so it can't drive
> itself off the bench. → [Safety]({{< relref "docs/safety" >}})

## What you'll need

- **Rust 1.88+** (edition 2024, plus let-chains).
- **Linux** is the tested platform. The serial layer is portable, but the
  `/dev/ttyUSB0` paths and the `dialout` group are Linux-specific.
- **18 V DC** motor power and a USB–RS485 adapter (DFRobot ships the RainbowLink
  TEL0185; any decent FTDI-based dongle works).

[Getting started]({{< relref "docs/getting-started" >}}) covers build, wiring, and
serial-port permissions. Beyond the tracks above, the site also carries the
[protocol reference]({{< relref "docs/protocol" >}}) (the wire format byte by byte),
the [FAQ]({{< relref "docs/faq" >}}) and
[Troubleshooting]({{< relref "docs/troubleshooting" >}}), and
[Internals]({{< relref "docs/internals" >}}) for contributors.

The full type-level API reference comes from rustdoc:

```sh
cargo doc --open -p m0601
```
