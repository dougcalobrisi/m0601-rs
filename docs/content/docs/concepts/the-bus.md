---
title: The bus
weight: 2
---

# The bus

RS485 is a two-wire, half-duplex, multi-drop bus. One differential pair (A/B) carries
traffic in both directions but only one direction at a time, and any number of motors
can hang off it, each with a unique address from `0x01` to `0xFE`. The host talks, a
motor answers, and everyone shares the same two wires.

"One direction at a time" is the whole story on this page. It's why the driver is
careful about *when* frames go out, not just what's in them.

## The back-to-back collision

Here's the trap. Every drive (`0x64`) frame elicits a reply — even a fire-and-forget
drive command you never read gets answered. Now send two drive frames back to back
with no gap between them. The second frame starts going out while the *reply to the
first* is still on the wire. On a half-duplex pair, those overlap. Both are corrupt.

In a one-shot script you might never notice. In a periodic 50 Hz loop it's a disaster
that looks like a hardware fault: the *same* frame collides every single cycle, so one
motor never moves at all while the rest of your robot works fine. You'd swear the
motor or its wiring was bad.

The fix is an idle gap. The bus enforces a minimum quiet time between frames —
`DEFAULT_MIN_GAP`, 2.5 ms — sized to cover one reply frame (~0.9 ms at 115200 baud)
plus an allowance for the motor's turnaround, so the reply a frame elicits has
finished before the next frame starts. Comparable protocols mandate exactly this kind
of idle floor; the M0601 leaves it to the host, so the [`Bus`]({{< relref
"../library/multi-motor" >}}) supplies it.

A few properties of that gap worth internalizing:

- **It's a floor, not exact spacing.** USB adapters and OS scheduling can stretch any
  individual gap longer; the bus guarantees frames don't overlap, not that they leave
  on a metronome. If you need tight cycle timing, one thread should own all the sends.
- **It lives on the port, not the handle.** There's one gap per physical bus. Cloning
  a motor handle or driving from two threads shares the same gap. Set it once, at open
  time, with `Bus::with_min_gap` — and set it from a turnaround you *measured*, not a
  number you liked.
- **Tuning it down is a hardware claim.** A smaller gap says "this adapter and this
  motor turn around faster than 2.5 ms," which you should verify before you rely on.

The idle gap is one field of `BusTiming`, the bus's tunable timing — alongside the stop
ramp and the mode/set-ID/broadcast waits. Every field defaults to the value the crate
has always used, and all of them live on the shared port the same way the gap does. Set
them individually (`with_min_gap`, `with_stop_accel`) or all at once from your own
config with `Bus::with_timing(BusTiming { .. })`.

## Addressing and collisions

Motors ship at `0x01`, and you assign the rest one at a time with [`set-id`]({{<
relref "../cli/set-id" >}}). Two protocol addresses are effectively off-limits:
`0x00` and `0xFF` are reserved, and `0xC8` is the destination of the broadcast ID
query — a motor sitting at `0xC8` can't be told apart from the query itself on an
adapter that echoes.

The broadcast query is why `scan` talks about "garbled" replies. It's unarbitrated:
every motor answers at once, and if more than one does, their frames land on top of
each other and decode to bytes no single motor sent. That's not silence — it's the
*signature* of multiple motors — which is exactly why an empty quick `scan` proves
nothing and `set-id` insists on polling all 254 addresses individually before it
trusts that only one motor is present.

## Bus budgeting

Because each motor needs its own drive frame at ≥50 Hz, the bus load scales with motor
count: N motors is at least N×50 frames per second, plus their replies, plus the gaps
between everything. Four wheels at the crate defaults is **~13.5 ms** of the 20 ms
cycle gone before you read a byte of telemetry (the worked derivation is in
[Budgeting the wire]({{< relref "../library/budgeting" >}})). It works, but it's why
the multi-motor advice is
specific: short reply waits, telemetry read round-robin rather than all at once, and
never a scan running against a live drive loop on the same wire.

You don't have to do that arithmetic by hand — `bus_period(n_drives, n_polls,
min_gap, reply_wait)` is it, in code. See [Budgeting the wire]({{< relref
"../library/budgeting" >}}).
