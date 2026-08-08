---
title: drive
weight: 5
---

# `drive` — hold a setpoint, scriptably

```sh
m0601 drive velocity --rpm 100 --secs 3
m0601 drive current  --amps 1.5 --secs 2
m0601 drive position --deg 180
```

`drive` is the batch counterpart to [`control`]({{< relref "control" >}}): it holds
a single setpoint in one mode, resending it at 50 Hz until a timer elapses or you
interrupt, then it brakes. It's what you use in scripts and test rigs, where you
want one command that does one thing and stops itself cleanly.

## The three modes

`drive` takes a mode as a required subcommand, each with its own natural units:

| Mode | Required flag | Range | Also accepts |
|---|---|---|---|
| `velocity` | `--rpm <i16>` | −330..330 RPM | `--accel <u8>` (default 1), `--secs` |
| `current` | `--amps <f32>` | −8.0..8.0 A | `--secs` |
| `position` | `--deg <f32>` | 0.0..360.0° | `--secs` |

```sh
m0601 drive velocity --rpm -80 --secs 3     # reverse at 80 RPM for 3 s
m0601 drive velocity --rpm 200 --accel 40   # softer ramp
m0601 drive current  --amps 1.5 --secs 2    # hold ~1.5 A of torque
m0601 drive position --deg 180              # rotate to 180° and hold
```

**Out-of-range values are rejected, not clamped.** Ask for `--rpm 5000` and the
argument parser refuses it up front. This is on purpose: a tool that silently drove
330 while printing 5000 would be lying to you, and the current and angle limits are
physical, not arbitrary — beyond ±8 A is simply unreachable, so it's an error, not a
rounding.

**`--secs` bounds the run** to `0..=3600` seconds; omit it to drive until `Ctrl-C`.
Either way the motor brakes on exit, so back-to-back scripted runs have no coasting
gap between them.

**`--accel` (velocity only) is a footgun worth respecting.** `1` is the motor's
*fastest* ramp and the default; larger is gentler; `0` is the motor's own default.
A big velocity step at accel 1 on a loaded wheel can spike current into the 3 A
protection and trip it. If a run keeps faulting out the instant it starts, soften
the ramp.

## Position mode checks before it commits

Switching into position mode is only legal below 10 RPM, so `drive position` does a
single pre-flight query first — the one place it waits your full `--timeout` rather
than the loop's 6 ms. It fails closed:

```
[x] Refused: 45 RPM — must be under 10 RPM to enter POSITION mode.
```

or, if the bus is silent:

```
[x] Refused: no telemetry — cannot confirm the wheel is under 10 RPM.
```

A silent bus means an unknown speed, and an unknown speed is not a slow one, so it
refuses. Both cases exit non-zero.

## While it runs

You get a one-line readout — mode, speed, current, position, temperature, faults —
refreshed about ten times a second. Winding temperature isn't in a drive reply, so
`drive` slips in an extra query every tenth cycle to keep it current. A transient
bus error prints `[!] bus error: ... (still driving)` and the loop keeps going
rather than dropping the wheel. It ends with:

```
Stopped and braked after 3.0 s.
```

## The braking guarantee, and its one gap

The moment `drive` sends its first frame, it arms a stop guard that brakes on
*every* subsequent exit path — normal completion, a `?` error mid-loop, `Ctrl-C`, a
panic. It forces velocity mode, zeroes, and brakes (~300 ms).

The gap is `SIGTERM`/`SIGHUP` when the signal handler couldn't be installed. `drive`
tries to install one and, if it fails, warns you outright:

```
[!] could not install signal handler (...); a SIGTERM/SIGHUP will coast the motor
    rather than brake it. Ctrl-C from the terminal still stops it.
```

And as always, `SIGKILL` and power loss coast the wheel — nothing runs to brake it.

## See also

- [`control`]({{< relref "control" >}}) — the interactive version.
- [Library → Drive loops]({{< relref "../library/drive-loops" >}}) — the same 50 Hz
  loop, in Rust, if you're building this into your own program.
