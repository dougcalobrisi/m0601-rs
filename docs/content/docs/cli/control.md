---
title: control
weight: 4
---

# `control` — interactive drive

```sh
m0601 control --rpm 100     # full-screen keyboard control
```

A full-screen dashboard that runs the 50 Hz drive loop for you and takes live
keyboard input. `control` is interactive; its scriptable counterpart is
[`drive`]({{< relref "drive" >}}).

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--rpm` | `100` | Preset speed for the `F`/`B` keys (−330..330). |

## Keys

| Key | Action |
|-----|--------|
| `F` / `B` | forward / backward at the `--rpm` preset (switches to velocity mode) |
| `1`–`5`   | 50–250 RPM (switches to velocity mode) |
| `←` / `→` | nudge ±10 RPM (velocity mode only) |
| `S`       | 0 RPM in velocity mode; hold the current angle in position mode; **zero torque — a coast, not a stop — in current mode** |
| `K`       | electric brake (velocity mode only; ignored in current and position mode) |
| `V`/`C`/`P` | switch mode: velocity / current / position |
| `Q` / `Esc` / `Ctrl-C` | quit — forces velocity mode, zeroes, then brakes |

## Behavior you'll actually notice

- `P` (position mode) is refused at 10 RPM or above (protocol constraint) and
  when no telemetry has arrived — an unknown speed is not zero. Entering position
  mode holds the wheel's *current* angle; it never jumps to 0°.
- The dashboard shows the mode the **motor reports**; if it ever differs from the
  requested one it turns red.
- Temperature updates every ~200 ms (it only arrives in the periodic telemetry
  query — drive replies don't carry it); shows `--` until the first.
- Every exit path stops the wheel — quit keys, panics, SIGINT/SIGTERM/SIGHUP (a
  dropped SSH session included). The stop is a fast step to zero plus brake, not a
  gentle ramp. On SIGKILL or power loss the polling stops and the motor coasts,
  per protocol.
