---
title: Sample code
weight: 60
bookCollapseSection: true
---

# Sample code

Runnable code that ships in this repository, in increasing order of size. Each one
exists to answer a different question.

- **[Examples]({{< relref "examples" >}})** — the two files in `m0601/examples/`.
  `four_wheel_minimal.rs` is the whole driver on one screen: a bus, four handles, a
  stop guard, a drive loop. `usage_doc_check.rs` is the compile check that keeps this
  site's Rust snippets honest.
- **[`m0601-quad`]({{< relref "quad" >}})** — the same wiring grown into a real
  application: a four-wheel skid-steer rover with a TOML wheel map, a dedicated pilot
  thread, latched fault handling, a terminal dashboard, and CSV logging. It is the
  reference implementation for multi-motor use of the library, and its `--dry-run`
  mode opens no serial port, so you can read *and run* it with no hardware.

The progression is deliberate. The minimal example shows you the API; `m0601-quad`
shows you what an application built on it has to add on top — and, by contrast, marks
[where the driver ends]({{< relref "../concepts/driver-boundary" >}}) and your robot
begins.
