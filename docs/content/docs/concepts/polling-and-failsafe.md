---
title: Polling and the fail-safe
weight: 1
---

# Polling and the fail-safe

A drive command to an M0601 does not latch. Send "100 RPM" once and the wheel gives
you a brief twitch and then coasts. To make it *hold* 100 RPM you have to keep saying
so — resend the drive frame at least every 20 ms, which is the 50 Hz floor for
sustained motion. The motor accepts commands up to 500 Hz; below ~50 Hz it decides
you've gone away and spins down.

This feels like extra work until you see what it buys you. The requirement to keep
talking *is* the safety system. If your program panics, the USB adapter falls out, or
the power supply browns out, the frames stop, and the motor coasts to a stop on its
own. There is no runaway state to design around, because "no one is commanding me"
resolves to "stop" in hardware. It's a command watchdog that's always on and can't be
forgotten.

## Implications for your code

Everything else about driving the motor follows from this one property:

- **The library's `drive_*` methods send exactly one frame.** They don't spawn a
  thread or hide a timer — the cadence is yours, explicitly, so you can see it and
  control it. See [Drive loops]({{< relref "../library/drive-loops" >}}).
- **The CLI runs the loop for you.** `control` and `drive` are the 50 Hz loop wrapped
  in a UI and a stop guard. That's most of what they are.
- **Substituting a query for a drive frame drops motion.** If a loop replaces one
  drive frame with a telemetry query every tenth cycle, that's a 40 ms hole — 25 Hz
  instantaneous, under the floor — and the wheel coasts a little every 200 ms. Read
  telemetry from the drive frame's own reply instead, or send the query *in addition*
  to the drive frame, never instead of it.

## Comparison with other fieldbuses

Nothing here is exotic; you've likely met each piece under a different name.

| M0601 behavior | Where you've seen it |
|---|---|
| enforced idle gap between frames | Modbus RTU's 3.5-character silence; CANopen PDO inhibit time |
| coast when frames stop (the 50 Hz floor) | a command watchdog / failsafe timeout, permanently on |
| `set_mode_all` / `safe_stop_all` | Dynamixel's broadcast Sync Write |
| automatic low-latency request on open | pyserial's `set_low_latency_mode(True)` |

The M0601 just leaves more of it to the host than a heavier protocol would, which is
why the driver has to be deliberate about spacing, stopping, and adapter latency —
the next few pages.
