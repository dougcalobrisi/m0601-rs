---
title: scan
weight: 1
---

# `scan` — find motors on the bus

```sh
m0601 scan            # broadcast, then poll 0x01..0x0F  (~3 s)
m0601 scan --full     # poll every address 0x01..0xFE    (~40 s)
```

Reach for `scan` first whenever the bus misbehaves: it's how you confirm a motor is
present, discover an address you've forgotten, or check that you don't have two
motors fighting over one ID.

## The two-stage scan

A default scan does two things. It sends one broadcast query that every motor
answers at once, then it polls addresses `0x01` through `0x0F` individually. That
range is a deliberate bet: motors ship at `0x01` and small fleets stay in single
digits, so polling sixteen addresses catches the common case in a couple of seconds
instead of the ~40 it takes to walk all 254. If your motors live at higher
addresses, you need `--full`.

The broadcast is the fast path, but it's unarbitrated — when several motors reply
simultaneously their frames collide into bytes that belong to none of them. `scan`
notices this. If the broadcast comes back garbled *and* nobody answered in the quick
range, it concludes motors are out there colliding and escalates to a full poll on
its own, printing:

```
Broadcast reply was garbled (motors answering together collide),
yet no motor answered 0x01..0x0F — polling every ID.
```

The important consequence: **an empty quick scan proves nothing.** A four-motor bus
can broadcast-collide into silence and, if all four sit above `0x0F`, scan as empty.
Only a full poll — where every address is probed individually — turns "found
nothing" into real evidence of an empty bus. When you need certainty, for example
right before a `set-id`, run `--full`.

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--full` | off | Poll every address `0x01..0xFE` (~40 s) instead of `0x01..0x0F`. |

`scan` **ignores `--id`** — probing every address is the entire job, so there's
nothing for a single address to select. ([`set-id`]({{< relref "set-id" >}}) ignores it
too: it finds the motor's current address by scanning, precisely because you usually
don't know it.) `--port` and
`--timeout` still apply, and `--timeout` is what sets the per-probe wait (and so the
scan's total runtime).

## Output

While a poll runs, a progress bar tracks it with an ETA derived from your timeout
(`~ceil(count × timeout)` seconds):

```
Polling 0x01..0xFE (254 IDs, ~39s):
[############------------------]
```

Found motors are listed with both hex and decimal, and a single hit prints a
ready-to-paste address:

```
Found 1 motor(s):
  - ID 0x01 (decimal 1)
Use:  --id 0x01
```

If the quick scan found motors but wasn't exhaustive, it reminds you that higher-ID
or colliding motors could still be hidden and suggests `--full`.

Nothing on the bus returns a non-zero exit and the checklist that solves it most
often:

```
No motors found.
  Checklist: 18V power on? brown wire -> GND? try swapping A/B (orange<->white).
```

followed by whether it polled all 254 or only the quick range — so you know whether
"nothing" is definitive or just "nothing in the first sixteen."

## See also

- [Concepts → Telemetry and echo]({{< relref "../concepts/telemetry-and-echo" >}})
  for why colliding replies look garbled rather than silent.
- [`set-id`]({{< relref "set-id" >}}), which runs its own exhaustive scan before it
  writes.
