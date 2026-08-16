---
title: raw
weight: 7
---

# `raw` — send an arbitrary frame

```sh
m0601 raw "01 74 00 00 00 00 00 00 00"       # 9 bytes: CRC appended for you
m0601 raw "01 74 00 00 00 00 00 00 00 FF"    # 10 bytes: sent exactly as typed
m0601 raw --yes "01 64 00 64 00 00 03 00 00" # a drive frame — needs --yes
```

`raw` is the protocol workbench. It sends bytes you specify and prints what comes
back, decoding the reply when the command you sent is one that elicits telemetry.
Use it to poke at the protocol, reproduce a frame from the [reference]({{< relref
"../protocol" >}}), or test how the motor reacts to something deliberately malformed.

## Options

| Flag | Default | Meaning |
|---|---|---|
| `<HEX>` | *(required)* | the frame bytes, 9 or 10 of them |
| `--yes` | off | required to send a frame that can move the motor (see below) |

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

## The `--yes` motion gate

Two command bytes can put the wheel in motion, and `raw` will not send either one
unless you say so explicitly:

- **`0x64`** (drive) — commands a setpoint in whatever mode the motor is in.
- **`0xA0`** (mode switch) — leaves the motor armed in a different control loop.

Without `--yes` the frame is refused before the port is even opened, and the command
exits non-zero:

```
[x] 01 64 00 64 00 00 03 00 00 AB is a motion command (byte 1 = 0x64); pass --yes to send it.
    It can move the motor and `raw` has none of `drive`'s rails.
```

The gate keys on **byte 1 alone**, not on the whole frame, so it is deliberately
broader than "frames that will actually move something." In particular the
**broadcast ID query** (`C8 64 00 00 00 00 00 00 00 DE`) is gated too — its command
byte is `0x64`:

```
[x] C8 64 00 00 00 00 00 00 00 DE is a motion command (byte 1 = 0x64); pass --yes to send it.
```

That is not an over-reach. `raw` sends the bytes *you* typed, and `C8 64` with a
non-zero value in bytes 2–3 is a drive command addressed to **every motor on the
bus**. There is no way to gate the harmless broadcast query without also ungating
that, so the whole command byte is gated. Use [`scan`]({{< relref "scan" >}}) when you
just want the broadcast query — that is what it does, without the sharp edge.

Everything else needs no flag: the `0x74` feedback query, the set-ID frame (its byte 1
is `0x55`), and any malformed frame you invent.

When you do pass `--yes`, `raw` brakes on the way out. After the reply is printed it
runs `safe_stop` (force velocity, zero, brake) **against byte 0 of the frame you
typed** — the motor the frame actually commanded — and says so:

```
(motion frame — braked motor 0x05 on exit)
```

The brake is sent even when the exchange itself fails mid-flight: once the frame may
have reached the wire, an I/O error doesn't skip the stop. That matters most for the
two cases a coast wouldn't cover: a current-mode frame is a torque impulse, and a
mode switch leaves the motor armed even after the frame itself has been forgotten.

> [!WARNING]
> **A broadcast drive frame (`C8 64 …`) commands *every* motor, and a unicast brake
> can only cover one.** The brake falls back to `--id` whenever byte 0 is not a valid
> unicast address — the broadcast destination `0xC8` or a reserved `0x00`/`0xFF` — and
> the output never claims otherwise. A broadcast frame prints `(broadcast motion frame
> — braked only motor 0x01; other motors coast)`; a reserved address prints `(frame
> addressed 0x00, not a unicast ID — braked the --id motor 0x01 on exit)`. Keep a hand
> on the power for broadcast frames.

## Deliberate omissions

`--yes` buys you a brake on exit, not `drive`'s full set of rails. `raw` sends your
bytes **once** and does not loop, so it can't sustain motion — a drive frame moves
the wheel for one cycle and then it coasts. There is **no position-mode pre-flight
check**, so nothing stops you from switching a spinning wheel into position mode the
way [`drive position`]({{< relref "drive" >}}) would. And `--id` does **not** rewrite
the bytes you typed: byte 0 of the frame is whatever you put there, whatever the
global flag says.

Reach for [`drive`]({{< relref "drive" >}}) or [`control`]({{< relref "control" >}})
for anything you actually want to move safely; keep `raw` for inspection and protocol
work.

## See also

- [Protocol reference]({{< relref "../protocol" >}}) — every frame, byte by byte.
- [Internals]({{< relref "../internals" >}}) — the frame builders `raw` sits on top
  of.
