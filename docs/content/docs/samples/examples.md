---
title: Crate examples
weight: 1
---

# Crate examples

Two files, with very different jobs. One is meant to be read and run; the other
exists only to be compiled.

## `four_wheel_minimal.rs` — the driver on one screen

```sh
cargo run -p m0601 --example four_wheel_minimal -- /dev/ttyUSB0
```

Four motors at IDs 1–4 on one RS485 adapter. About 50 lines of actual code, and it
touches every part of the multi-motor API in the order you'd meet them:

1. **One shared bus.** `Bus::open(&port, Duration::from_millis(150))?.with_min_gap(…)`
   — the generous timeout is a backstop for the open; the loop passes its own short
   reply wait per poll. The idle gap is tightened to 2 ms so the cycle in step 5
   fits.
2. **One handle per wheel, sign convention applied once.** A `const WHEELS: [(u8,
   bool); 4]` table pairs each RS485 id with whether that corner is a mirror-image
   build, and `bus.motor(id)?.mirrored(mirror)` bakes it in at construction. After
   that line the application never touches a sign again — `+60` means forward on all
   four. ([Mirroring]({{< relref "../library/multi-motor" >}}).)
3. **A stop guard armed *before* the first frame.** A tiny `struct StopGuard { bus,
   ids }` whose `Drop` calls `bus.safe_stop_all(&self.ids)`. Because `Bus` is `Clone`
   it holds its own handle to the same port, and because it's armed before anything
   moves, a `?` early-return or a panic anywhere below still lands on a bus-wide
   stop.
4. **A group mode switch.** `bus.set_mode_all(&ids, Mode::Velocity)?`.
5. **The loop, budgeted before it moves.** A startup `assert!` on
   `bus_period(4, 1, bus.min_gap(), REPLY_WAIT)` — ~17.2 ms at the 2 ms gap and 2 ms
   reply wait — checks the traffic fits inside the 20 ms cycle and that the cycle
   stays at or under `drive_floor()`. Then: drive all four at 60 RPM, resend every
   ~20 ms (the [50 Hz floor]({{< relref "../concepts/polling-and-failsafe" >}})), and
   poll **one** wheel per cycle round-robin — never substituting a query for a drive
   frame, and never polling all four in one cycle. That is the
   [bus budget]({{< relref "../library/budgeting" >}}) advice as working code, right
   down to the assertion that page recommends.

This is the distilled essence of [`m0601-quad`]({{< relref "quad" >}}) with the TUI,
the logger, and the safety state machine removed, so the driver API is the only thing
on screen.

**It is safe to build without hardware.** With no port argument it prints

```
usage: four_wheel_minimal <serial-port>   e.g. /dev/ttyUSB0
```

and returns `Ok(())`, so `cargo build --examples` — and CI's
`cargo build --workspace --all-targets` — never needs a motor.

> [!CAUTION]
> With a port argument it *does* spin four wheels at 60 RPM for one second. Clear
> them first.

## `usage_doc_check.rs` — the compile check behind these docs

Not meant to be run: its `main` is empty and every function is `#[allow(dead_code)]`.
Each function mirrors one snippet from the library documentation — `query_example`,
`drive_example`, `transact_example`, `modes_example`, `bus_example`,
`multi_motor_example`, `timing_example`, `strict_crc_example`,
`position_mirror_example`, `odometry_example`, `budgeting_example`,
`slew_example`, `low_latency_example`, `mock_example`.

The point is that CI builds it (`cargo build --workspace --all-targets`), so a
documented signature that drifts from the real API breaks the build rather than
quietly misleading a reader. If you change a public signature, this is the file that
will tell you which docs to update.

Keep it in sync when you edit the [Library guide]({{< relref "../library" >}}) —
that's the contract described in [Internals]({{< relref "../internals" >}}).
