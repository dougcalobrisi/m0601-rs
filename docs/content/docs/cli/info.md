---
title: info
weight: 2
---

# `info` — configuration + one-shot readout

```sh
$ m0601 info
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

Prints the connection configuration plus one live telemetry readout. Exits
nonzero when the motor doesn't reply — usable in scripts as a presence check.

Uses the global `--port`, `--id`, and `--timeout` flags; it has no options of its
own.
