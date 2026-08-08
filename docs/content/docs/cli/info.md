---
title: info
weight: 2
---

# `info` — config plus a live snapshot

```sh
m0601 info
m0601 --id 0x02 info
```

`info` answers "is this motor there, and what does it say about itself right now?"
It prints the connection configuration and the fixed ranges for each control mode,
sends a single query, and shows one decoded reply. Nothing moves. It's the natural
follow-up to `scan` and a handy scriptable presence check, because it exits non-zero
when the motor doesn't answer.

## Output

```
================================================
  M0601 Configuration
================================================
  Port          : /dev/ttyUSB0
  Baud / format : 115200 8N1 (RS485 half-duplex)
  Motor ID      : 0x01 (1)
  Velocity range: -330..330 RPM
  Current range : -32767..32767 (~-8..+8 A)
  Position range: 0..32767 (0..360 deg)
------------------------------------------------
  Mode          : Velocity
  Speed         : +0 RPM
  Current       : -0.002 A
  Position      : 264.0 deg
  Winding temp  : 30 C
  Error         : 0x00  OK
  Raw frame     : 01 02 FF F9 00 00 1E BB 00 5F
================================================
```

The top block is static — it's the fixed protocol limits, useful as a reminder of
what each mode will accept. The bottom block is the live reading. A few things worth
knowing about it:

- **Speed** is signed (`+0 RPM`); a negative number means the wheel is turning the
  other way.
- **Current** shows three decimals and rarely reads exactly zero even at rest —
  that small offset is normal.
- **Winding temp** comes straight from the query reply. If a future change ever left
  it absent it renders as `--` rather than crashing, but in practice `info` always
  has it.
- **Error** reads `0x00  OK` when clean, or `0x{bits}  FAULT (names)` with the fault
  bits spelled out when something has tripped.
- **Raw frame** is the exact 10 bytes that came back, in case you want to check the
  decode by hand against the [protocol reference]({{< relref "../protocol" >}}).

## When the motor doesn't answer

```
  Live readout  : no valid response.
  Check 18V power, wiring (brown->GND), A/B polarity, and --id.
```

This exits non-zero. It means the port opened fine but no clean reply came back —
usually a wrong `--id`, or the RX half of the wiring. If `scan` found the motor but
`info` can't read it, suspect the address first.

## See also

- [Telemetry and echo]({{< relref "../concepts/telemetry-and-echo" >}}) — how that
  raw frame becomes the decoded fields.
- [`monitor`]({{< relref "monitor" >}}) — the same reading, continuously.
