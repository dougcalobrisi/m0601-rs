---
title: drive
weight: 5
---

# `drive` — scriptable motion in one mode

`control` is interactive; `drive` is its batch counterpart. It holds a single
setpoint in one mode, resending at 50 Hz, until `--secs` elapses or you press
Ctrl-C — then it brakes. Each mode takes its own natural units:

```sh
m0601 drive velocity --rpm 100            # spin at 100 RPM until Ctrl-C
m0601 drive velocity --rpm -80 --secs 3   # reverse at 80 RPM for 3 s, then stop
m0601 drive velocity --rpm 200 --accel 40 # gentler ramp (accel 1 is the fastest)
m0601 drive current --amps 1.5 --secs 2   # hold ~1.5 A of torque for 2 s
m0601 drive position --deg 180            # rotate to 180° and hold
```

## Modes and options

`drive` takes a required mode subcommand:

| Mode | Required | Optional | Range |
|---|---|---|---|
| `velocity` | `--rpm <I16>` | `--accel <U8>` (default 1), `--secs <F64>` | −330..330 RPM |
| `current` | `--amps <F32>` | `--secs <F64>` | −8.0..8.0 A |
| `position` | `--deg <F32>` | `--secs <F64>` | 0.0..360.0° |

- **`--secs`** bounds the run (0–3600 s); omit it to drive until Ctrl-C. Either
  way the motor is braked on exit.
- **Units convert to the wire ranges**: `--rpm` clamps to ±330, `--amps` maps
  through ±32767 ≈ ±8 A, `--deg` maps 0..360 onto 0..32767. Out-of-range values
  are rejected up front by the argument parser, not silently clamped on the wire.
- **`--accel`** (velocity only) is the ramp byte: `1` is the motor's *fastest*
  ramp and the default; a large step at accel 1 on a loaded wheel can spike
  current into the 3 A protection. Raise it (larger = gentler; `0` = motor
  default) to ramp gently.
- **Position mode is refused at 10 RPM or above** (protocol constraint) and when
  no telemetry has arrived — an unknown speed is not a zero one. The pre-flight
  speed check is the only part of `drive` that waits the full `--timeout`; the
  50 Hz loop uses a fixed 6 ms reply wait like `control`.
- **`safe_stop` on every exit path** — clean end, `--secs` timeout, Ctrl-C,
  SIGTERM/SIGHUP, or a panic — forces velocity mode, zeroes, then brakes. On
  SIGKILL or power loss the polling simply stops and the motor coasts.

A live one-line readout (mode, speed, current, position, temp, faults) updates
~10 times a second while it runs. Winding temperature comes from an extra `0x74`
query interleaved every 10th cycle, since drive replies don't carry it.

If you script `drive` runs back to back, each one brakes at the end, so there is
no coasting gap between them.
