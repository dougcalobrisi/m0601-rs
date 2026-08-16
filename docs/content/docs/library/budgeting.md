---
title: Budgeting the wire
weight: 7
---

# Budgeting the wire

Every multi-motor loop eventually asks the same question: *will N wheels plus their
polls fit in my cycle?* The crate answers it with arithmetic rather than advice, so
you size a loop instead of guessing at one and discovering the answer as a wheel that
stutters.

Three functions, all pure and I/O-free:

```rust
use std::time::Duration;
use m0601::{bus_period, drive_floor, frame_time};

let frame  = frame_time();          // ~868 µs — one 10-byte frame at 115200 8N1
let floor  = drive_floor();         // 20 ms — the longest gap before a wheel coasts
let gap    = Duration::from_millis(2);
let period = bus_period(4, 1, gap, gap);   // four drives + one poll ≈ 17.2 ms
```

## The two cycle bounds

A periodic control loop has to satisfy both of these at once, and they squeeze from
opposite sides:

- **`drive_floor()` = 20 ms** is the ceiling on your cycle. Every wheel needs *its*
  drive frame at ≥50 Hz, so a cycle longer than this means every wheel coasts a
  little, every cycle.
- **`bus_period(...)`** is the floor. It's how much wire time the cycle's frames
  actually consume. A cycle shorter than this can't sustain its own period — the loop
  is asking for more bus than exists.

If `bus_period(...) > drive_floor()`, the design doesn't fit and no amount of tuning
the loop will save it; you need fewer polls per cycle, a shorter reply wait, or a
smaller measured gap.

## What each term costs

`bus_period(n_drives, n_polls, min_gap, reply_wait)` is:

- **A drive frame** costs one `frame_time()` plus `min_gap`. It's fire-and-forget —
  you don't wait for the reply, but the gap after it is what keeps that reply clear of
  the next frame ([The bus]({{< relref "../concepts/the-bus" >}})).
- **A poll** costs *two* frame times plus `reply_wait` plus `min_gap`. The transport
  sleeps out its own wire time **and** the reply window, then the trailing idle gap
  re-budgets a full frame plus gap from the poll's return — so the frame's wire time
  is spaced once inside the transaction and once in the trailing gap.

Worked, at the crate's defaults: four wheels driving with no polls is
4 × (0.868 ms + 2.5 ms) ≈ **13.5 ms** of a 20 ms cycle gone before a byte of
telemetry is read. That's the number quoted throughout the multi-motor pages, and
it's why the advice is what it is: short reply waits, telemetry read **round-robin**
(one motor per cycle, not all four), and never a [`scan`]({{< relref "../cli/scan" >}})
running against a live drive loop.

## Usage

The natural place is startup — fail at launch, in the light, rather than mid-drive. In
a throwaway script an assertion is enough:

```rust
let cycle = Duration::from_millis(18);
let need  = bus_period(4, 1, bus.min_gap(), reply_wait);
assert!(need < cycle && cycle <= drive_floor(),
        "cycle {cycle:?} cannot carry {need:?} of bus traffic under the {:?} floor",
        drive_floor());
```

In an application, check the same thing but return the error instead of panicking —
a robot that aborts mid-process is worse than one that refuses to start.
[`m0601-quad`]({{< relref "../samples/quad" >}}) does it that way: its config
validation calls `bus_period(4, 1, gap, wait)` and refuses to load the file at all
when the traffic does not fit the cycle, downgrading to a warning when the slack is
under 10%. The warning prints on *every* subcommand, not just `check`, and it quotes
the occupancy — `~13.5ms of the 18ms cycle is occupied` — rather than the remaining
slack. That figure is how its `cycle_ms = 18.0` and its decision to poll only every
*second* cycle were both arrived at rather than assumed.

## See also

- [Concepts → The bus]({{< relref "../concepts/the-bus" >}}) — why the gap exists.
- [Multi-motor bus]({{< relref "multi-motor" >}}) — the API these numbers govern.
- [Concepts → Latency]({{< relref "../concepts/latency" >}}) — why `reply_wait` is
  short and why the adapter's own timer can wreck all of this.
