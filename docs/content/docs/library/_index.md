---
title: Library
weight: 20
bookCollapseSection: true
---

# Library guide

The `m0601` crate is the driver the CLI is built on. It talks to one motor
through an `M0601` handle, or fans out to many through a shared `Bus`.

## Add the dependency

By git:

```toml
[dependencies]
m0601 = { git = "https://github.com/dougcalobrisi/m0601-rs-test.git" }
```

Pin to a known-good state with `rev = "<sha>"`, `tag = "..."`, or `branch =
"main"` (default). `cargo update -p m0601` pulls the latest commit of the pinned
branch.

Or from a local checkout:

```toml
[dependencies]
m0601 = { path = "../m0601-rs-test/m0601" }
```

## The shape of the API

- **`M0601<T>`** — a handle to one motor. `open`, `query`, `drive_velocity`,
  `drive_current`, `drive_position`, `set_mode`, `transact`, `safe_stop`.
- **`Bus<T>`** — owns the shared RS485 port and mints cheap, cloneable,
  thread-safe per-motor handles (`bus.motor(id)`). Enforces frame spacing and
  offers group operations (`safe_stop_all`, `set_mode_all`).
- **`Feedback` / `Telemetry`** — parsed replies. See [Telemetry]({{< relref
  "telemetry" >}}).
- **`Mode`, `Faults`** — control mode and the fault bitmask.
- **`Transport`** — the serial abstraction; `SerialTransport` for hardware,
  `MockTransport` for tests.

Errors are a `thiserror` enum with `Result<T> = Result<T, m0601::Error>`. A
**silent bus is not an error** — a non-replying motor surfaces as `Ok(None)`,
never `Err`.

## Pages

| Page | What it covers |
|---|---|
| [Quickstart]({{< relref "quickstart" >}}) | query a motor without moving it |
| [Drive loops]({{< relref "drive-loops" >}}) | the 50 Hz cadence and `safe_stop` |
| [Modes]({{< relref "modes" >}}) | velocity / current / position |
| [Telemetry]({{< relref "telemetry" >}}) | the two reply layouts |
| [Multi-motor bus]({{< relref "multi-motor" >}}) | shared bus, mirroring, spacing, groups |
| [Testing without hardware]({{< relref "testing" >}}) | `MockTransport` |

> The runnable snippets here mirror `m0601/examples/usage_doc_check.rs`, which is
> compiled in CI — so the examples on this site are known to build.
