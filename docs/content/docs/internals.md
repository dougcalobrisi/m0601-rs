---
title: Internals
weight: 45
---

# Internals

For contributors and the curious: how the crate is put together, and why it's shaped
that way. None of this is needed to *use* the driver — it's the map you'd want before
changing it.

## The transport seam

All I/O goes through one small trait, `Transport`, with two implementations:
`SerialTransport` for real hardware and `MockTransport` for tests. `M0601` and `Bus`
are generic over it, so the exact code that ships runs unchanged against a scripted
in-memory bus — there are no `cfg(test)` branches threading through the driver logic.

The trait's `pace` method is the neat part. A real transport reports how long a delay
would take; the mock reports zero. The driver takes that value under the bus lock but
performs the actual sleep *outside* the lock, so one motor's mode-switch sequence
doesn't stall another motor's 50 Hz loop — and under the mock, "sleeps" are free, so
loop tests run instantly instead of in real milliseconds.

`send_recv` is where the [latency]({{< relref "concepts/latency" >}}) work lives: it
writes, waits, then reads exactly the bytes the OS says are buffered rather than
issuing a blocking read, treating `TimedOut` as "no more data." That's what keeps a
silent motor from adding the full port timeout to a real-time cycle.

## `Bus` and `M0601`

`Bus<T>` owns the port behind an `Arc<Mutex>` and hands out `M0601<T>` handles that
are cheap to clone and safe to send across threads — they all share the one physical
port. The bus is what enforces the inter-frame gap and runs the round-major group
operations; a single `M0601::open` is really just `Bus::open(...).motor(id)` under the
hood.

The lock is **poison-tolerant** on purpose. The guarded transport holds no invariant a
panic could corrupt mid-update — each call is one complete frame exchange — and the
stop paths above all must keep working even if another thread panicked while holding
the port. A stop that deadlocked because a different thread died is the opposite of
safe. For the same reason, the `Debug` impl uses `try_lock` and reports rather than
blocks, so it can run from a panic path that already holds the lock without deadlocking
the formatter.

## The `control` TUI: two threads, careful teardown

`control` splits cleanly. A **poll thread owns the serial port** and runs the 50 Hz
loop; a **UI thread owns the terminal** and only ever edits a small shared state
struct. No lock is ever held across serial I/O — the hold times are nanoseconds
against a 20 ms budget — so the two never contend meaningfully.

The teardown is the interesting engineering. Every exit path funnels through the same
sequence: clear the `running` flag, join the poll thread, and let *its* epilogue run
`safe_stop`. RAII guards make that hold even through a panic. A `TermGuard` restores
the terminal (raw mode off, leave the alt-screen, cursor back) and a `StopGuard` clears
`running` and joins the poll thread — declared in an order such that on unwind the
terminal is restored *first* (so a panic message is readable) and the motor stops
*second*. The `TermGuard` is armed the instant raw mode is enabled, because a `?`
between enabling raw mode and arming the guard would return with the tty still raw and
nothing left to fix it. The poll loop even wraps itself in `catch_unwind` so that a
panic in the loop body still runs the ~300 ms braked stop before propagating.

The upshot is the guarantee the CLI docs promise: short of `SIGKILL` or power loss,
there is no way to leave `control` with the wheel driven.

## One `unsafe` block, well-fenced

The crate is `#![deny(unsafe_code)]` — `deny`, not `forbid`, because there's exactly
one exception and `forbid` couldn't be locally overridden. That exception is the pair
of Linux `TIOCGSERIAL`/`TIOCSSERIAL` ioctls in `low_latency`, which set the FTDI
latency flag on a file descriptor the crate owns, operating on a locally defined UAPI
struct. It's kept behind a scoped `allow` with per-call SAFETY comments, so the
crate-wide `deny` still covers everything else. Everything outside those two ioctls is
unsafe-free.

## How the docs stay honest

The Rust snippets in this site and the doc comments aren't decoration — they're
compiled. `m0601/examples/usage_doc_check.rs` is a real example the CI builds, and the
frame-level claims in `protocol.rs` are backed by doctests with exact expected bytes
(and the `parse_feedback` double-decode is a deliberate regression guard against the
two-layouts bug). If a documented signature drifts from the code, the build breaks.
That's the intended contract: the docs can be wrong about prose, but not about the
API.

## Workspace layout

```
m0601/        the library crate (what you depend on)
  src/        lib, bus, protocol, types, transport, error, low_latency
  examples/   usage_doc_check.rs — the compiled doc snippets
  tests/      golden vectors, mock-bus behavior, spacing, hardware-in-loop
m0601-cli/    the CLI crate (binary `m0601`)
  src/        main.rs + cmd/{scan,info,monitor,drive,set_id,raw,control/*}
docs/         this site (Hugo + hugo-book)
```

The root `README.md`, `USAGE.md`, and `PROTOCOL.md` remain the in-repo quick
references; this site is the expanded, browsable version of the same material.
