---
title: raw
weight: 7
---

# `raw` — protocol probing

```sh
m0601 raw "01 74 00 00 00 00 00 00 00"       # 9 bytes: CRC appended
m0601 raw "01 74 00 00 00 00 00 00 00 FF"    # 10 bytes: sent verbatim
```

Sends an arbitrary frame and prints TX and RX in hex. When the sent command
elicits telemetry, it decodes the reply using the correct layout for that
command.

- **9 bytes** → the CRC-8/MAXIM is computed and appended for you.
- **10 bytes** → sent verbatim (use this to send a deliberately wrong CRC, or a
  frame whose byte 9 is not a checksum, like mode-switch or set-ID).

Accepts spaces or commas between bytes and optional `0x` prefixes. `raw` raises
the reply timeout to a 200 ms floor so a slow reply is not missed.

See the [protocol reference]({{< relref "../protocol" >}}) for the frame layouts.
