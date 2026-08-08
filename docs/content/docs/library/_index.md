---
title: Library
weight: 20
bookCollapseSection: true
---

# Library guide

The `m0601` crate is the driver the CLI is built on. Everything the command-line
tool does, you can do from Rust — and the crate is written for exactly that, with a
mock transport so you can test motor logic without hardware and a `#![deny(unsafe_code)]`
budget it keeps to a single well-defended block.

## Add the dependency

By git:

```toml
[dependencies]
m0601 = { git = "https://github.com/dougcalobrisi/m0601-rs-test.git" }
```

Pin to something reproducible with `rev`, `tag`, or `branch` (defaults to `main`);
`cargo update -p m0601` moves a branch pin forward. Or work from a local checkout:

```toml
[dependencies]
m0601 = { path = "../m0601-rs-test/m0601" }
```

## The shape of the API

Two types carry most of the work:

- **`M0601<T>`** is a handle to one motor: `open`, `query`, `drive_velocity`,
  `drive_current`, `drive_position`, `set_mode`, `transact`, `safe_stop`. Each
  `drive_*` call sends exactly one frame — the [50 Hz cadence]({{< relref
  "drive-loops" >}}) is yours to provide.
- **`Bus<T>`** owns the shared serial port and hands out cheap, cloneable per-motor
  handles via `bus.motor(id)`. It enforces [frame spacing]({{< relref "multi-motor"
  >}}), runs group operations like `safe_stop_all`, and is what you reach for with
  more than one motor.

Supporting cast: `Feedback` and `Telemetry` for parsed replies, `Mode` and `Faults`
for control state, and the `Transport` trait — `SerialTransport` on hardware,
`MockTransport` in [tests]({{< relref "testing" >}}).

## The contract that surprises people

**A silent bus is not an error.** A motor that doesn't reply — wrong ID, unpowered,
a scan probe to an empty address — comes back as `Ok(None)`, never `Err`. An `Err`
always means the port or the OS failed. Handle the two distinctly; treating "no
reply" as an exception will fight the whole API.

```rust
match motor.query()? {
    Some(fb) => { /* real telemetry */ }
    None     => { /* silence — expected, not a failure */ }
}
```

## Pages

- **[Quickstart]({{< relref "quickstart" >}})** — read a motor without moving it.
- **[Drive loops]({{< relref "drive-loops" >}})** — the 50 Hz cadence and
  `safe_stop`.
- **[Modes]({{< relref "modes" >}})** — velocity, current, position.
- **[Telemetry]({{< relref "telemetry" >}})** — the two reply layouts and the
  accumulator that reconciles them.
- **[Multi-motor bus]({{< relref "multi-motor" >}})** — sharing a bus, mirroring,
  spacing, group stops.
- **[Testing without hardware]({{< relref "testing" >}})** — `MockTransport`.

> [!NOTE]
> The snippets across these pages mirror `m0601/examples/usage_doc_check.rs`, which
> the CI compiles — so what's shown here is known to build against the real API.
