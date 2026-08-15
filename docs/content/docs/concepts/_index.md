---
title: Concepts
weight: 25
bookCollapseSection: true
---

# Concepts & design notes

The CLI and library pages tell you *what* to do. This section is the *why* — the
handful of hardware and protocol facts that shaped every interesting decision in the
driver. None of it is required reading to spin a wheel, but it's what turns "the docs
said to do this" into "of course it works this way."

Each page stands alone, but they build on each other roughly in this order:

- **[Polling and the fail-safe]({{< relref "polling-and-failsafe" >}})** — why the
  motor coasts when you stop talking to it, and why that's the feature, not the bug.
- **[The bus]({{< relref "the-bus" >}})** — half-duplex RS485, addressing, and the
  frame spacing that keeps a periodic drive loop from corrupting itself.
- **[Telemetry and echo]({{< relref "telemetry-and-echo" >}})** — the two reply
  layouts, adapter echoes, and a genuinely dangerous frame-alignment bug the driver
  is built to avoid.
- **[Stopping safely]({{< relref "stopping-safely" >}})** — why a stop starts by
  switching modes, and how a whole vehicle stops without yawing.
- **[Latency]({{< relref "latency" >}})** — the FTDI 16 ms timer that quietly breaks
  short reply windows, and the two ways the driver defeats it.
- **[Where the driver ends]({{< relref "driver-boundary" >}})** — what this crate owns
  (the wire, the bus, one motor) versus what it deliberately leaves to your robot, and
  the seams for reaching past the defaults.

If you only read one, read the bus and echo pages — together they explain most of
what looks like superstition in the multi-motor code.
