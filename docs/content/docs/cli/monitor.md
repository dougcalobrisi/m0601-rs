---
title: monitor
weight: 3
---

# `monitor` — continuous telemetry

```sh
m0601 monitor --hz 5                 # live one-line dashboard, Ctrl+C to stop
m0601 monitor --hz 20 --csv log.csv  # ...and log every reading to a CSV (see the truncation note)
```

`monitor` is `info` on repeat: it polls at a rate you choose and keeps a single line
updated in place, optionally logging every reading to CSV. It only ever *queries* —
it never sends a drive frame — so the wheel keeps doing whatever it was doing (idle,
or driven by something else on the bus). Use it to watch temperature climb under
load, capture a run for later analysis, or just confirm a controller elsewhere is
doing what you expect.

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--hz` | `5.0` | poll rate, validated to `0.001..=1000` Hz |
| `--csv <FILE>` | — | also log each reading to a CSV file |

The rate is held by measuring how long each poll took and sleeping the remainder, so
`--hz 20` really means one reading every 50 ms, not "as fast as possible."

`--hz` is honest even on a slow or silent motor. Each poll waits a short, bounded
reply window — 6 ms, shortened further if the poll interval or `--timeout` is smaller
— rather than the full `--timeout`. Without that bound a silent motor would collapse
the effective rate to roughly `1/--timeout` regardless of what you asked for, so
`--hz 100` against a dead bus would quietly run at about 6 Hz. The error path paces
off the same elapsed clock as the success path, so a failing cycle doesn't over-sleep
either.

## Reading the line

```
[14:32:07] #  142 | Velocity | Speed +100 RPM | Cur +0.312 A | Pos 187.4 | Temp  41C | OK
```

Timestamp, a running count, the reported mode, and the decoded telemetry. `OK`
becomes `FAULT <names>` (names joined with ` | `) if a protection bit is set.

## Behavior on a rough bus

RS485 drops the occasional frame, and a long-running monitor shouldn't flap or die
because of it. Two behaviors handle that:

- **A single missed poll is ignored.** The last good reading stays on screen, and
  `monitor` only warns after **five consecutive** misses (about a second at 5 Hz)
  with `[!] no response — check motor power/wiring`. That keeps a healthy-but-lossy
  bus from strobing warnings at you.
- **A transient bus error doesn't kill it.** A USB hiccup prints `[!] bus error: ...
  — still polling` and the loop continues, same policy as the control loop.

## The CSV format

With `--csv`, every reading is appended as a row under this header:

```
timestamp,motor_id,mode,speed_rpm,current_a,temp_c,position_deg,error_code,error_str,raw_hex
```

`timestamp` is local wall-clock time formatted `%Y-%m-%d %H:%M:%S`. Two things to
rely on:

- **The schema is a stable contract.** Downstream logs and scripts depend on these
  columns and their order, so they don't change casually.
- **Rows are flushed as they're written.** If the session is killed mid-run,
  everything logged up to that point is already on disk — you don't lose the run to
  a buffer.

One sharp edge: opening the file **truncates** it. `monitor` warns before it does
(`[!] log.csv already exists — overwriting it.`), but a re-run with the same
filename replaces the previous log rather than appending. Name your files per run if
you want to keep them.

## Stopping

`Ctrl-C`, `SIGTERM`, or `SIGHUP` all stop the loop cleanly, flush and close the CSV,
and print `Saved N rows to log.csv`. Since `monitor` never drives the motor, there's
nothing to brake — stopping it just stops the watching.

If the signal handler can't be installed, `monitor` says so up front:

```
[!] could not install signal handler (...); Ctrl-C will terminate abruptly
    (CSV rows are already flushed per line, so none are lost).
```

The consequence is cosmetic here, unlike in [`drive`]({{< relref "drive" >}}): you
lose the closing `Stopped.` / `Saved N rows` summary, but every row written is
already on disk.

## See also

- [`info`]({{< relref "info" >}}) for a one-shot version.
- [`drive`]({{< relref "drive" >}}), which shows a live readout *while* driving.
