---
title: CLI
weight: 40
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
| `--id` | `0x01` | motor address `0x01..=0xFE`; accepts hex (`0x01`), octal, binary, or decimal |
| `--timeout` | `0.15` | reply wait in seconds, validated to a finite `0.005..=3600` |

```sh
m0601 --port /dev/ttyUSB1 --id 0x02 info
m0601 info --port /dev/ttyUSB1 --id 0x02     # identical
```

Both numeric flags are validated at the argument boundary, so a bad value is a clean
usage error before any port is opened:

- **`--id`** is rejected outside `0x01..=0xFE`. `0x00` and `0xFF` are reserved by the
  protocol, so `--id 0x00` fails immediately rather than surfacing later as a driver
  error. (`scan` and `set-id` ignore `--id` entirely — both discover addresses by
  probing rather than being told one.)
- **`--timeout`** has a **0.005 s floor**. The timeout doubles as the serial reply
  window, and a frame itself takes ~0.9 ms on the wire, so a near-zero value left no
  room for a reply and turned every read into a false "no response". `--secs` on
  `drive` is a different parser and still accepts `0`, where it legitimately means
  "stop immediately".

There is no `--baud`: the link is fixed at 115200 8N1 because the motor speaks only
that one rate.

`--timeout` is the per-reply wait for the commands that do one exchange at a time —
`scan`, `info`, `monitor`, `set-id`. It deliberately does **not** govern the 50 Hz
loops in `control` and `drive`: those use a fixed 6 ms reply wait so a slow reply
can't stretch a 20 ms cycle. Around that:

- `drive` uses `--timeout` for the port open and for `drive position`'s pre-flight
  speed check, which happens before the loop starts; the loop itself does not.
- `monitor` bounds its own reply wait to 6 ms or less, so `--hz` stays honest on a
  slow bus — `--timeout` only caps it.
- `raw` raises whatever you pass to a 200 ms floor so a leisurely reply isn't missed.
- `control` ignores it altogether (fixed 50 ms open, fixed 6 ms loop wait).

## Exit codes and streams

Every command returns `0` on success and non-zero on failure, so they compose in
scripts. Two kinds of failure map to a non-zero exit:

- **An error** — the port or OS failed. Printed as `[x] {error}` on stderr. A
  permission error additionally prints the `dialout` hint.
- **No motor** — several commands (`scan`, `info`, `set-id`, and a refused `drive
  position`) exit non-zero when the bus stays silent, even though nothing technically
  errored. That lets `m0601 info` double as a presence check in a script.

A bad argument is clap's own usage error and exits `2`, before any port is opened.

**Data goes to stdout; diagnostics go to stderr.** Failure and refusal lines across
`info`, `scan`, `set-id`, `raw`, and `drive` are written to stderr, so redirecting
stdout captures only real readout data:

```sh
m0601 info > readout.txt              # readout only; the failure line still shows
m0601 drive velocity --rpm 100 2>/dev/null   # live status only, no bus-error notices
```

There is no `--json`. The one machine-readable format is
[`monitor --csv`]({{< relref "monitor" >}}), whose column set is a stable contract.

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
