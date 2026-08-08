---
title: Overview
weight: 1
bookFlatSection: false
---

# Documentation

Everything needed to drive the DFRobot M0601 hub motor with this repo, whether
you're calling the library from Rust or running the CLI against a motor on the
bench.

## Where to go

- **[Getting started]({{< relref "getting-started" >}})** — build, install, wire,
  and get serial-port permissions right.
- **[First-spin tutorial]({{< relref "tutorial" >}})** — bare wires to a spinning
  wheel, narrated step by step.
- **[CLI guide]({{< relref "cli" >}})** — a detailed page per subcommand.
- **[Library guide]({{< relref "library" >}})** — calling `m0601` from Rust.
- **[Concepts]({{< relref "concepts" >}})** — the design notes: why the driver
  behaves the way it does.
- **[Protocol reference]({{< relref "protocol" >}})** — the wire format, byte by
  byte, with sourcing.
- **[FAQ]({{< relref "faq" >}})** and **[Troubleshooting]({{< relref "troubleshooting" >}})** — when something's off.
- **[Internals]({{< relref "internals" >}})** — how the crate is built, for
  contributors.

## Three facts that explain almost everything

Nearly every design choice in this driver falls out of three properties of the
motor. Read these once and the rest of the docs stop being surprising.

**1. It's a polling protocol.** A drive command doesn't latch — the motor moves
only while drive frames arrive at ≥50 Hz (up to 500 Hz). Below that floor it
coasts. The CLI's `control` and `drive` run that loop for you; the library's
`drive_*` methods send exactly one frame each and leave the cadence to you. The
upside is a free watchdog: if the host dies, the wheel stops on its own.

**2. Zero is not universally "stop."** A zero setpoint stops the wheel in velocity
mode only. In position mode it commands a move to 0°; in current mode it commands
zero torque, which is a coast, not a brake. This is why `safe_stop` forces velocity
mode before it sends anything, and why `control`'s `S` key does something different
in each mode.

**3. Telemetry comes in two layouts.** A reply to a `0x74` query carries winding
temperature and a coarse 8-bit angle. A reply to a drive frame carries a fine
16-bit angle and no temperature. The same bytes decode differently depending on
which command asked for them — see [Telemetry and echo]({{< relref
"concepts/telemetry-and-echo" >}}).

## Requirements

- **Rust 1.88+** (edition 2024, plus let-chains).
- **Linux** is the tested platform. The serial layer is portable, but the
  `/dev/ttyUSB0` paths and the `dialout` group are Linux-specific.
- **18 V DC** motor power and a USB–RS485 adapter (DFRobot ships the RainbowLink
  TEL0185; any decent FTDI-based dongle works).

## API reference

The full type-level docs come from rustdoc:

```sh
cargo doc --open -p m0601
```

Publishing them alongside this site is planned for when GitHub Pages is switched
on; for now they're a one-command build locally.
