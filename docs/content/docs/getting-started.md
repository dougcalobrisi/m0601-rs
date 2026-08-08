---
title: Getting started
weight: 2
---

# Getting started

## Build & install

Needs Rust **1.88** or newer (edition 2024 plus let-chains). Linux is the tested
platform; the serial layer is portable, but the `/dev/ttyUSB0` paths and the
`dialout` group below are Linux-specific.

```sh
git clone https://github.com/dougcalobrisi/m0601-rs-test.git
cd m0601-rs-test
cargo build --release              # binary at target/release/m0601
cargo install --path m0601-cli     # or install `m0601` into ~/.cargo/bin
```

## Hardware setup

1. **Power**: 18 V DC on the 2-pin cable (red = +, black = GND). The motor is
   silent on the bus until powered.
2. **RS485**: white = A(+), orange = B(−) to your USB-RS485 adapter. The motor's
   A/B labels are inverted relative to many adapters — **if nothing answers, swap
   orange ↔ white** before debugging anything else.
3. **Brown wire → GND.** It is not optional; floating it causes intermittent
   comms errors.
4. Cable runs over ~1 m: add a 120 Ω termination resistor across A/B.
5. **Permissions** (Linux): if opening the port fails with a permission error,
   `sudo usermod -aG dialout $USER`, then log out and back in.

## Sanity check

Verify the whole chain in one command:

```sh
m0601 scan          # should print the motor's ID within a few seconds
```

Nothing found? Work through the [wiring checklist]({{< relref
"troubleshooting" >}}).

## Connection defaults

The link is fixed at **115200 8N1, RS485 half-duplex** — there is no baud flag.
Only three global options control the connection, valid before *or* after the
subcommand:

| Flag | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | serial port device path |
| `--id` | `0x01` | motor RS485 ID (hex `0x01` or decimal `1`) |
| `--timeout` | `0.15` | reply wait in seconds (0–3600) |

Motors ship at ID `0x01`. Assign new IDs one at a time with
[`set-id`]({{< relref "cli/set-id" >}}).

## Before you spin it

`control` and `drive` start driving the motor **immediately**, with no
confirmation prompt. A direct-drive hub motor has no gearbox to slow it down, and
the `1`–`5` presets reach 250 RPM.

- Clear the wheel, and secure the chassis so it cannot drive itself off the bench.
- Remember that **a zero setpoint does not mean "stop"** outside velocity mode.
- `Ctrl-C` brakes. So does every other exit path — but only while the process is
  alive to do it. On SIGKILL or power loss the motor coasts, per protocol.
