---
title: CLI
weight: 10
bookCollapseSection: true
---

# CLI reference

The `m0601` binary drives one motor (or, for `scan` and `set-id`, surveys the whole
bus) without writing any code. Install it with `cargo install --path m0601-cli`.

Each subcommand has its own page below with the full behavior — output samples, exit
codes, error handling, and the footguns worth knowing. This page covers what's
common to all of them.

## Global flags

Three options apply to every subcommand, and clap accepts them **before or after**
the subcommand name, whichever reads better to you:

| Flag | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | serial device path |
| `--id` | `0x01` | motor address; accepts hex (`0x01`), octal, binary, or decimal |
| `--timeout` | `0.15` | reply wait in seconds, validated to a finite `0..=3600` |

```sh
m0601 --port /dev/ttyUSB1 --id 0x02 info
m0601 info --port /dev/ttyUSB1 --id 0x02     # identical
```

`--timeout` is the per-reply wait for the commands that do one exchange at a time —
`scan`, `info`, `monitor`, `set-id`. It deliberately does **not** govern the 50 Hz
loops in `control` and `drive`: those use a fixed 6 ms reply wait so a slow reply
can't stretch a 20 ms cycle. The one exception is `drive position`'s pre-flight
speed check, which waits the full `--timeout` because it happens before the loop
starts. `raw` raises whatever you pass to a 200 ms floor so a leisurely reply isn't
missed.

## Exit codes

Every command returns `0` on success and non-zero on failure, so they compose in
scripts. Two kinds of failure map to a non-zero exit:

- **An error** — the port or OS failed. Printed as `[x] {error}` on stderr. A
  permission error additionally prints the `dialout` hint.
- **No motor** — several commands (`scan`, `info`, `set-id`, and a refused `drive
  position`) exit non-zero when the bus stays silent, even though nothing technically
  errored. That lets `m0601 info` double as a presence check in a script.

## The commands

| Command | What it's for |
|---|---|
| [`scan`]({{< relref "scan" >}}) | find motor addresses on the bus |
| [`info`]({{< relref "info" >}}) | config block plus one live readout |
| [`monitor`]({{< relref "monitor" >}}) | continuous telemetry, optional CSV log |
| [`control`]({{< relref "control" >}}) | full-screen keyboard dashboard |
| [`drive`]({{< relref "drive" >}}) | hold one setpoint, scriptably |
| [`set-id`]({{< relref "set-id" >}}) | change a motor's persistent address |
| [`raw`]({{< relref "raw" >}}) | send an arbitrary frame |

`m0601 --help` and `m0601 <command> --help` print the built-in reference at any
time.
