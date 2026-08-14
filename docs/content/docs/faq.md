---
title: FAQ & gotchas
weight: 35
---

# FAQ & gotchas

The questions that come up once you're past "does it turn," most of them the kind you
only ask after something surprising happened. [Troubleshooting]({{< relref
"troubleshooting" >}}) is the symptom-to-fix table; this is the why-does-it-do-that.

## My wheel spins for a second and then stops on its own

Your control loop is running below 50 Hz. A drive command doesn't latch — the motor
holds a setpoint only while frames keep arriving at least every 20 ms, and coasts
otherwise. Either your loop's sleep is too long, or you replaced a drive frame with a
telemetry query on some cycles (that leaves a hole in the drive cadence). Read
telemetry from the drive reply instead, and keep the loop at 20 ms.
See [Polling and the fail-safe]({{< relref "concepts/polling-and-failsafe" >}}).

## I sent zero to stop it and it moved / didn't brake

Zero only means "stop" in velocity mode. In position mode a zero setpoint means
"rotate to 0°," which can be most of a turn; in current mode it means zero torque — a
coast, not a brake. Use `safe_stop` (library) or the mode-aware `S` key (`control`),
both of which force velocity mode first. See [Stopping safely]({{< relref
"concepts/stopping-safely" >}}).

## Position mode is refused even though the wheel is stopped

Switching into position mode is only legal below 10 RPM, and the check **fails
closed**: if no telemetry has arrived, the speed is *unknown*, and unknown is treated
as "not confirmed slow," so the switch is refused. A wheel that's genuinely stopped but
whose RX path is broken will hit this. Fix the reply direction of the wiring (it's
often the same A/B swap that causes a silent bus) and the reading will come back.

## `set-id` renamed all my motors at once

The set-ID frame is unaddressed — every motor on the bus takes the new ID. If you run
it with more than one motor connected, you get a bus full of duplicates that can only
be untangled by disconnecting them one at a time. The CLI guards against this by
polling all 254 addresses first and refusing if it sees more than one — and that
guard runs unconditionally, *before* any prompt, so `--yes` does not bypass it
(`--yes` only skips the interactive "type yes" confirmation). The one way to still
get a mass rename is duplicate IDs colliding so the scan detects just a single
motor. Renumber one motor at a time, physically. See
[`set-id`]({{< relref "cli/set-id" >}}).

## An empty `scan` — does that mean the bus is empty?

Only if it was a `--full` scan. A default scan polls `0x01..0x0F` after a broadcast,
and the broadcast can collide into garbage when several motors answer at once. So a
four-motor bus with everything above `0x0F` can scan as empty. When you need a
definitive answer — before a `set-id`, say — run `scan --full`, which probes every
address individually.

## `kill -9` didn't stop the motor

Correct, and intended. `SIGKILL` and power loss run no code, so nothing brakes — the
motor coasts, per the protocol fail-safe. Every *survivable* exit brakes (`Ctrl-C`,
`SIGTERM`, `SIGHUP`, panics), but `kill -9` is not an emergency stop. If you need a
guaranteed hard stop, cut motor power.

## `raw` sent my frame but the CRC was wrong / it didn't brake after

Both are by design. `raw` recomputes the CRC only when you give it 9 bytes; pass a
full 10 and it sends them verbatim, wrong checksum and all — that's how you test
malformed frames. And `raw` has no safety funnel: it sends once and does not brake on
exit, so a hand-crafted drive frame moves the wheel for one cycle with nothing to stop
it afterward. Use `drive` or `control` for motion you want handled safely.

## My driver keeps tripping a fault the instant it starts

Almost always the 3 A bus-overcurrent protection, tripped by too aggressive a ramp.
`drive_velocity` (and the CLI default) uses acceleration `1`, the motor's *fastest*
ramp; a big step at accel 1 on a loaded wheel spikes current past 3 A. Use
`drive_velocity_accel` with a larger accel byte (gentler), or `drive --accel 40`. The
protection auto-resets about five seconds after the condition clears.

## Out-of-range values — clamped or rejected?

At the CLI boundary, **rejected**: `--rpm 5000` is refused up front, because a tool
that drove 330 while printing 5000 would be lying. Inside the library's frame builders
and the `control` target, values **clamp** to the valid range (and clamp symmetrically,
so nothing wraps to the wrong sign). Different layers, different policy, on purpose.

## Everything's intermittent — dropouts, garbage, flaky reads

Two usual causes. The brown wire is floating (it must be tied to ground — it's not
optional), or you're missing 120 Ω termination on a cable run over about a metre. Both
produce exactly the "works sometimes" symptom that sends people chasing software bugs.
