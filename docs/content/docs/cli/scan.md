---
title: scan
weight: 1
---

# `scan` — who's on the bus?

```sh
m0601 scan            # broadcast + poll IDs 0x01..0x0F, ~3 s
m0601 scan --full     # poll every ID 0x01..0xFE, ~40 s
```

The default scan broadcasts one query, then polls IDs `0x01..0x0F` individually —
motors ship at `0x01` and small fleets stay low, so that covers the common case
in seconds. The output always says which range was polled; motors assigned a
higher ID need `--full`.

The broadcast is unarbitrated, so **two motors can collide and look like one or
none**. When the collision garbles the reply so badly that no ID can be read
anywhere, `scan` says so and automatically escalates to the full poll; if it read
some IDs but also garbage, it lists what it found and suggests `--full`. When you
need certainty (e.g. before `set-id`), use `--full` directly.

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--full` | off | Poll every ID `0x01..0xFE` (~40 s) instead of the default `0x01..0x0F`. |
