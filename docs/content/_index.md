---
title: m0601
type: docs
---

# m0601 — DFRobot M0601 hub motor, in Rust

A reusable **driver crate** (`m0601`) and a **CLI** (`m0601`) for the DFRobot
**M0601** direct-drive hub motor over half-duplex RS485.

The M0601 is a rebadged Direct Drive Tech **M0601C-111**; **FIT1042** (left) and
**FIT1038** (right) are DFRobot's SKUs for its mirror-image builds. They are
electrically identical and speak the same protocol, so this one library covers
both — see [mirroring]({{< relref "docs/library/multi-motor" >}}).

### Use it as a library

Add one dependency and talk to a motor in a dozen lines. A cheap, cloneable `Bus`
fans out to per-motor handles; drive loops, telemetry, and multi-wheel group stops
are first-class. → [Library guide]({{< relref "docs/library" >}})

### Run the CLI

`scan` the bus, `monitor` live telemetry, `drive` a setpoint, or take the wheel
with an interactive `control` dashboard — no code required.
→ [CLI guide]({{< relref "docs/cli" >}})

## The one rule

The M0601 is **not Modbus**: fixed 10-byte frames at 115200 8N1, and a *polling*
protocol. The motor keeps moving only while drive frames keep arriving at
**~50 Hz**. Stop sending and the wheel coasts — that is the protocol's built-in
fail-safe, and it shapes both the CLI and the library API.

And the corollary worth carrying into any code you write: **a zero setpoint does
not mean "stop".** It only does in velocity mode — the same zero-valued frame
commands a move to 0° in position mode and zero torque (a coast) in current mode.

## Quick start

```sh
git clone https://github.com/dougcalobrisi/m0601-rs-test.git
cd m0601-rs-test
cargo install --path m0601-cli     # installs the `m0601` binary
m0601 scan                         # find the motor on /dev/ttyUSB0
```

New here? Start with [Getting started]({{< relref "docs/getting-started" >}}) for
hardware wiring and permissions, then pick the [CLI]({{< relref "docs/cli" >}}) or
[library]({{< relref "docs/library" >}}) track.

> ⚠️ **Before you spin it:** `control` and `drive` start driving the motor
> immediately, with no confirmation prompt. A direct-drive hub motor has no
> gearbox, and the presets reach 250 RPM. Clear the wheel and secure the chassis.
