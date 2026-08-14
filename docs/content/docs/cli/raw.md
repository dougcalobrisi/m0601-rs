---
title: raw
weight: 7
---

# `raw` — send an arbitrary frame

```sh
m0601 raw "01 74 00 00 00 00 00 00 00"       # 9 bytes: CRC appended for you
m0601 raw "01 74 00 00 00 00 00 00 00 FF"    # 10 bytes: sent exactly as typed
```

`raw` is the protocol workbench. It sends bytes you specify and prints what comes
back, decoding the reply when the command you sent is one that elicits telemetry.
Use it to poke at the protocol, reproduce a frame from the [reference]({{< relref
"../protocol" >}}), or test how the motor reacts to something deliberately malformed.

## The 9-vs-10-byte rule

The byte count you pass changes what happens to the checksum:

- **9 bytes** → the CRC-8/MAXIM is computed and appended, giving a valid 10-byte
  frame. This is the normal case.
- **10 bytes** → sent verbatim, byte 9 included. The CRC is **not** recomputed.

That second case is the point of `raw`: it lets you send a frame with a deliberately
wrong CRC, or a frame whose last byte isn't a checksum at all (the mode-switch and
set-ID frames put other data there). Anything other than 9 or 10 bytes is an error:
`... Provide 9 bytes (CRC auto-added) or 10.`

Separators are flexible — spaces or commas, with or without `0x` prefixes on
individual bytes.

## Output

```
TX: 01 74 00 00 00 00 00 00 00 04
RX: 01 02 FF F9 00 00 1E BB 00 5F
    decoded -> mode Velocity, 0 RPM, -0.002 A, 264.0 deg, temp 30C, err OK
```

`raw` only decodes when the frame you sent was a telemetry-eliciting command, and it
decodes using the correct layout for *that* command — a query reply and a drive reply
are laid out differently ([Telemetry and echo]({{< relref
"../concepts/telemetry-and-echo" >}})). A frame that draws no reply prints `RX: (no
response)` and exits successfully; that's a normal outcome for a bus with nothing at
the address.

The reply wait is raised to a 200 ms floor here, so a slow or unusual reply isn't
missed.

## What `raw` deliberately doesn't do

`raw` is a sharp tool with the guards removed. It sends your bytes once and does not
loop, so it won't sustain motion — a drive frame sent via `raw` moves the wheel for
one cycle and then it coasts. More importantly, **it bypasses the safety funnel**:
there's no position-mode pre-flight check, and no `safe_stop` on exit. If you hand-
craft a drive frame with `raw`, nothing brakes afterward. Reach for [`drive`]({{<
relref "drive" >}}) or [`control`]({{< relref "control" >}}) for anything you
actually want to move safely; keep `raw` for inspection and protocol work.

## See also

- [Protocol reference]({{< relref "../protocol" >}}) — every frame, byte by byte.
- [Internals]({{< relref "../internals" >}}) — the frame builders `raw` sits on top
  of.
