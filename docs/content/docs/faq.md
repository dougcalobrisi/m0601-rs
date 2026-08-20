---
title: FAQ & gotchas
weight: 90
---

# FAQ & gotchas

The questions that come up once you're past "does it turn," most of them the kind you
only ask after something surprising happened. [Troubleshooting]({{< relref
"troubleshooting" >}}) is the symptom-to-fix table; this is the why-does-it-do-that.

## Wheel spins briefly, then coasts {#spins-then-stops}

Your control loop is running below 50 Hz. A drive command doesn't latch — the motor
holds a setpoint only while frames keep arriving at least every 20 ms, and coasts
otherwise. Either your loop's sleep is too long, or you replaced a drive frame with a
telemetry query on some cycles (that leaves a hole in the drive cadence). Read
telemetry from the drive reply instead, and keep the loop at 20 ms.
See [Polling and the fail-safe]({{< relref "concepts/polling-and-failsafe" >}}).

## Zero setpoint moved or failed to brake {#zero-didnt-stop}

Zero only means "stop" in velocity mode. In position mode a zero setpoint means
"rotate to 0°," which can be most of a turn; in current mode it means zero torque — a
coast, not a brake. Use `safe_stop` (library) or the mode-aware `S` key (`control`),
both of which force velocity mode first. See [Stopping safely]({{< relref
"concepts/stopping-safely" >}}).

## Position mode refused at standstill {#position-refused}

Switching into position mode is only legal below 10 RPM, and the check **fails
closed**: if no telemetry has arrived, the speed is *unknown*, and unknown is treated
as "not confirmed slow," so the switch is refused. A wheel that's genuinely stopped but
whose RX path is broken will hit this. Fix the reply direction of the wiring (it's
often the same A/B swap that causes a silent bus) and the reading will come back.

## `set-id` renamed every motor {#set-id-renamed-all}

The set-ID frame is unaddressed — every motor on the bus takes the new ID. If you run
it with more than one motor connected, you get a bus full of duplicates that can only
be untangled by disconnecting them one at a time. The CLI guards against this by
polling all 254 addresses first and refusing if it sees more than one — and that
guard runs unconditionally, *before* any prompt, so `--yes` does not bypass it
(`--yes` only skips the interactive "type yes" confirmation). The one way to still
get a mass rename is duplicate IDs colliding so the scan detects just a single
motor. Renumber one motor at a time, physically. See
[`set-id`]({{< relref "cli/set-id" >}}).

## Empty `scan` results {#empty-scan}

Only if it was a `--full` scan. A default scan polls `0x01..0x0F` after a broadcast,
so a bus with every motor above `0x0F` can scan as empty.

The CLI catches most of this itself: if the quick range comes back empty *and* the
broadcast reply was garbled — motors answering together and colliding — it concludes
motors are out there and escalates to a full `0x01..=0xFE` poll on its own, saying so.
The empty result you're looking at is therefore the case that escaped: the collision
read as *silence* rather than garbage, so there was nothing to escalate on.

When you need a definitive answer — before a `set-id`, say — run `scan --full`, which
probes every address individually. See [`scan`]({{< relref "cli/scan" >}}).

## `kill -9` and coasting {#kill-9-coasts}

Correct, and intended. `SIGKILL` and power loss run no code, so nothing brakes — the
motor coasts, per the protocol fail-safe. Every *survivable* exit brakes (`Ctrl-C`,
`SIGTERM`, `SIGHUP`, panics), but `kill -9` is not an emergency stop. If you need a
guaranteed hard stop, cut motor power.

## `raw` CRC handling and refusals {#raw-by-design}

Both are by design.

`raw` recomputes the CRC only when you give it **9** bytes; pass a full **10** and it
sends them verbatim, wrong checksum and all — that's precisely how you test malformed
frames.

And it refuses the two command bytes that can move the wheel — `0x64` (drive) and
`0xA0` (mode switch) — unless you pass `--yes`. With `--yes` it sends the frame and
then brakes on exit (`safe_stop`), aimed at byte 0 of the frame you typed — the
motor the frame actually commanded. The one gap is a broadcast `C8` drive frame: it
commands every motor, a unicast brake covers only one (`--id`), and the output says
so.

What it still doesn't give you is the rest of `drive`'s rails: it sends **once** and
does not loop, so motion isn't sustained, and there's no position-mode pre-flight
check. Use [`drive`]({{< relref "cli/drive" >}}) or
[`control`]({{< relref "cli/control" >}}) for motion you want handled safely.

## Faults at drive start {#fault-on-start}

Almost always the 3 A bus-overcurrent protection, tripped by too aggressive a ramp.
`drive_velocity` uses acceleration `1`, the motor's *fastest* ramp, and so does the
CLI's `drive velocity`; a big step at accel 1 on a loaded wheel spikes current past
3 A. (`control` is the exception — it defaults to `3`, deliberately gentler, because a
single keystroke there commands a large instantaneous step.)

Soften it with `drive_velocity_accel` or `drive velocity --accel n` (`--accel` lives on
the `velocity` subcommand, not on `drive` itself). Larger is gentler — but **`0` is
not**: it selects the motor's default, which
[measures identical to `1`]({{< relref "protocol" >}}#known-contradictions-between-sources),
so it is the harshest setting rather than a safe middle. Keep the number small either
way: a step to 120 RPM takes ~0.45 s at `1`, ~2 s at `5`, and over 3 s at `20`, so
`3`–`5` is the useful range and `40` is nearly a standstill.

The other lever, which works no matter what the motor's ramp is doing, is to make the
*step* smaller: ramp the setpoint host-side with
[`SlewLimiter`]({{< relref "concepts/setpoint-shaping" >}}) rather than commanding a
jump. The protection auto-resets about five seconds after the *trip*, so a wheel that
is still loaded simply trips again.

## Out-of-range values: clamp vs. reject {#out-of-range}

At the CLI boundary, **rejected**: `--rpm 5000` is refused up front, because a tool
that drove 330 while printing 5000 would be lying. Inside the library's frame builders
and the `control` target, values **clamp** to the valid range (and clamp symmetrically,
so nothing wraps to the wrong sign). Different layers, different policy, on purpose.

## Intermittent dropouts and garbage {#intermittent}

Two usual causes. The brown wire is floating (it must be tied to ground — it's not
optional), or you're missing 120 Ω termination on a cable run over about a metre. Both
produce exactly the "works sometimes" symptom that sends people chasing software bugs.
