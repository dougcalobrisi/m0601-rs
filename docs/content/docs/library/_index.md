---
title: Library
weight: 50
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
m0601 = { git = "https://github.com/dougcalobrisi/m0601-rs.git" }
```

Pin to something reproducible with `rev`, `tag`, or `branch` (defaults to `main`);
`cargo update -p m0601` moves a branch pin forward. Or work from a local checkout:

```toml
[dependencies]
m0601 = { path = "../m0601-rs/m0601" }
```

## API shape

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
for control state, `PositionAccumulator` for [odometry]({{< relref "odometry" >}}),
`SlewLimiter` for [setpoint shaping]({{< relref "../concepts/setpoint-shaping" >}}),
and the `Transport` trait — `SerialTransport` on hardware, `MockTransport` in
[tests]({{< relref "testing" >}}).

`Bus` also carries the whole-bus operations the CLI is built from, so you rarely need
to hand-build a frame: `bus.scan(range, progress)` returns a `ScanReport { ids,
garbled }`, `bus.set_id(new_id)` performs the unaddressed rename, and `send_raw` (on
either type) puts arbitrary bytes on the wire. Errors are one `Error` enum —
`Serial`, `Io`, `InvalidId`, `InvalidFrameLen`, `InvalidSlewRate` — with
`is_permission_denied()` for the `dialout` case.

The exhaustive surface is rustdoc's job:

```sh
cargo doc --open -p m0601
```

## The `Ok(None)` contract

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

They run in reading order, in four groups.

**One motor** — everything you need for a single wheel:

- **[Quickstart]({{< relref "quickstart" >}})** — read a motor without moving it.
- **[Drive loops]({{< relref "drive-loops" >}})** — the 50 Hz cadence and
  `safe_stop`.
- **[Modes]({{< relref "modes" >}})** — velocity, current, position.

**Reading the motor back:**

- **[Telemetry]({{< relref "telemetry" >}})** — the two reply layouts and the
  accumulator that reconciles them.
- **[Odometry]({{< relref "odometry" >}})** — `PositionAccumulator` and the aliasing
  bound on unwrapping a single-turn angle.

**More than one motor:**

- **[Multi-motor bus]({{< relref "multi-motor" >}})** — sharing a bus, mirroring,
  spacing, group stops.
- **[Budgeting the wire]({{< relref "budgeting" >}})** — `bus_period`, `frame_time`,
  `drive_floor`: will N wheels fit in your cycle?

**And once it's yours to maintain:**

- **[Testing without hardware]({{< relref "testing" >}})** — `MockTransport`, and the
  hardware-in-the-loop tests when you do have a motor.

For working code rather than snippets, see
[Sample code]({{< relref "../samples" >}}) — `four_wheel_minimal.rs` is this
whole guide in about fifty lines.

> [!NOTE]
> The snippets across these pages mirror
> [`m0601/examples/usage_doc_check.rs`]({{< relref "../samples/examples" >}}), which
> the CI compiles — so what's shown here is known to build against the real API.
