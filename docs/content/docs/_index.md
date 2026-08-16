---
title: Overview
weight: 1
bookFlatSection: false
---

# Documentation

Everything needed to drive the DFRobot M0601 hub motor with this repo, whether you're
calling the library from Rust or running the CLI against a motor on the bench.

If you've landed here from a search engine, the [front page]({{< relref "/" >}}) is
the orientation — it has the three facts that explain almost everything about this
motor, and it points at the right track for what you're doing.

## The sections

- **[Getting started]({{< relref "getting-started" >}})** — build, install, wire, and
  get serial-port permissions right.
- **[First-spin tutorial]({{< relref "tutorial" >}})** — bare wires to a spinning
  wheel, narrated step by step.
- **[Safety]({{< relref "safety" >}})** — what brakes, what coasts, and what will hurt
  you. Short; read it before the wheel is on the ground.
- **[CLI guide]({{< relref "cli" >}})** — a detailed page per subcommand.
- **[Library guide]({{< relref "library" >}})** — calling `m0601` from Rust.
- **[Sample code]({{< relref "samples" >}})** — the runnable code in this repo:
  `four_wheel_minimal.rs` and the `m0601-quad` rover app.
- **[Concepts]({{< relref "concepts" >}})** — the design notes: why the driver behaves
  the way it does.
- **[Protocol reference]({{< relref "protocol" >}})** — the wire format, byte by byte,
  with sourcing.
- **[FAQ]({{< relref "faq" >}})** and
  **[Troubleshooting]({{< relref "troubleshooting" >}})** — when something's off.
- **[Internals]({{< relref "internals" >}})** — how the crate is built, for
  contributors.
