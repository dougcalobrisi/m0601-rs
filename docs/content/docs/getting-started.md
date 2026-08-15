---
title: Getting started
weight: 2
---

# Getting started

This page gets you built, wired, and permitted. If you'd rather learn by doing,
the [first-spin tutorial]({{< relref "tutorial" >}}) covers the same ground with a
motor actually turning.

## Build & install

You need Rust **1.88** or newer — the crate uses edition 2024 and let-chains, so
older toolchains won't compile it. Linux is the tested platform; the serial layer
is portable, but the device paths and the `dialout` group below are Linux-isms.

```sh
git clone https://github.com/dougcalobrisi/m0601-rs.git
cd m0601-rs
cargo build --release              # binary at target/release/m0601
cargo install --path m0601-cli     # or drop `m0601` into ~/.cargo/bin
```

The install builds the `m0601-cli` crate, whose binary is named `m0601`. The
library crate (also `m0601`) is a separate workspace member you depend on from your
own project — see the [library guide]({{< relref "library" >}}).

## Wiring, and why each wire matters

The motor has two cables: a 2-pin power cable and a 4-pin signal cable. Getting any
of the following wrong produces "nothing on the bus," so it's worth doing
deliberately.

**Power — 18 V DC** on the 2-pin cable, red to +, black to ground. The RS485
transceiver is powered from this same supply, so a motor with no power isn't a
motor answering with errors — it's silent. If a scan comes up empty, confirm 18 V
before you suspect anything subtle.

**RS485 — white is A(+), orange is B(−)** to your adapter. Here's the one that
wastes the most time: the M0601's A/B labelling is inverted relative to a lot of
USB-RS485 dongles. If nothing answers, **swap orange and white before you debug
anything else.** It's the single most common cause of a dead bus, and it's a
two-second test.

**Brown wire to ground.** Brown is a reserved/shield line, and it is not optional —
leave it floating and you get intermittent comms errors that look like flaky
hardware, especially on longer cable runs.

**Termination.** For cable runs over about a metre, put a 120 Ω resistor across A/B.
Short bench setups usually work without it; long runs without it drop frames.

## Serial-port permissions (Linux)

Opening `/dev/ttyUSB0` as a normal user usually fails the first time with a
permission error. That means your user isn't in the `dialout` group:

```sh
sudo usermod -aG dialout $USER
# then log out and back in — group membership is applied at login
```

The CLI detects this specific failure and prints the same hint, so you don't have
to remember it. If a command dies with `[x] ... Permission denied`, this is why.

## Connection defaults

The link is fixed at **115200 baud, 8N1, half-duplex** — there is no baud flag,
because the motor only speaks one rate. Three global options control the
connection, and they're valid before *or* after the subcommand:

| Flag | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | serial device path |
| `--id` | `0x01` | motor address (hex `0x01` or decimal `1`) |
| `--timeout` | `0.15` | reply wait, in seconds (0–3600) |

Motors ship at ID `0x01`, so the defaults Just Work for a single fresh motor. Once
you have more than one on the bus, give each a unique address with
[`set-id`]({{< relref "cli/set-id" >}}) — one at a time, for reasons that page
explains.

## Sanity check

One command proves the whole chain — power, wiring, adapter, permissions, address:

```sh
m0601 scan
```

A motor should show up within a couple of seconds. If it doesn't, work back through
the wiring above (start with the A/B swap), then see
[Troubleshooting]({{< relref "troubleshooting" >}}).
