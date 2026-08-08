---
title: CLI
weight: 10
bookCollapseSection: true
---

# CLI reference

The `m0601` binary is a control tool for a single motor (or, for `scan`/`set-id`,
the whole bus). Install it with `cargo install --path m0601-cli`.

## Global flags

Valid **before or after** the subcommand:

| Flag | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | serial port |
| `--id` | `0x01` | motor RS485 ID (hex `0x01` or decimal `1`) |
| `--timeout` | `0.15` | reply wait in seconds (0–3600) |

`--timeout` governs `scan`, `info`, `monitor` and `set-id`. It does **not** apply
to `control` or `drive`, whose 50 Hz loops use a fixed 6 ms reply wait (only
`drive`'s pre-flight speed check, before entering position mode, waits the full
`--timeout`); `raw` raises it to a 200 ms floor.

```sh
m0601 --port /dev/ttyUSB1 --id 0x02 info    # globals before the subcommand
m0601 info --port /dev/ttyUSB1 --id 0x02    # ...or after
```

## Subcommands

| Command | Purpose |
|---|---|
| [`scan`]({{< relref "scan" >}}) | discover motor IDs on the bus |
| [`info`]({{< relref "info" >}}) | config + one-shot live readout |
| [`monitor`]({{< relref "monitor" >}}) | headless live dashboard, optional CSV logging |
| [`control`]({{< relref "control" >}}) | full-screen dashboard with keyboard control |
| [`drive`]({{< relref "drive" >}}) | drive one mode at a fixed setpoint (scriptable) |
| [`set-id`]({{< relref "set-id" >}}) | change a motor's persistent RS485 ID |
| [`raw`]({{< relref "raw" >}}) | send an arbitrary frame |

Run `m0601 --help` or `m0601 <command> --help` for the built-in reference.
