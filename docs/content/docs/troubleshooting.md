---
title: Troubleshooting
weight: 40
---

# Troubleshooting

Symptom on the left, the thing to check on the right. For *why* these behave the way
they do, the [FAQ]({{< relref "faq" >}}) and [Concepts]({{< relref "concepts" >}})
sections go deeper.

| Symptom | Check |
|---|---|
| `scan` finds nothing | 18 V power on? A/B swapped (try orange ↔ white)? Brown wire → GND? A silent bus is almost always wiring. |
| `Permission denied` on the port | `sudo usermod -aG dialout $USER`, then log out and back in. |
| Found the motor, but `info` won't read it | Wrong `--id`, or the RX half of the wiring. `scan` shows the real address. |
| Wheel spins briefly, then coasts | Your loop is under 50 Hz, or a query is replacing a drive frame on some cycles. Keep the drive cadence at 20 ms. |
| Sent zero, but it moved or didn't brake | You weren't in velocity mode. Use `safe_stop` / the `S` key, which force velocity first. |
| `control`'s `P` (or `drive position`) refused | Wheel at ≥10 RPM, or no telemetry yet (it fails closed on unknown speed). |
| Faults the instant a drive starts | Ramp too steep — accel `1` spikes current past the 3 A protection. Soften with a larger accel byte. Auto-resets in ~5 s. |
| Intermittent garbage / dropouts | Brown wire floating, or missing 120 Ω termination on a cable over ~1 m. |
| Chaos after a `set-id` | The unaddressed frame renamed every motor that heard it. Reconnect one at a time and renumber. |
| Short reply waits read nothing | FTDI 16 ms latency timer. See [Latency]({{< relref "concepts/latency" >}}) — the udev rule fixes it. |

## Still stuck? Confirm the chain end to end

Work outward from the physical layer, because that's where the failures cluster:

1. **Power.** 18 V actually present at the motor. An unpowered motor is silent, not
   erroring.
2. **A/B polarity.** The single most common dead-bus cause. Swap orange ↔ white and
   re-`scan` before anything else.
3. **Brown → ground.** Not optional; a floating brown line gives you the flaky,
   intermittent symptoms that look like software.
4. **Address.** `m0601 scan --full` tells you the real ID definitively (the quick
   scan can miss colliding or higher-ID motors).
5. **Latency.** If replies clearly arrive but reads come up empty on short waits,
   check `latency_timer` is `1`, not `16`.

## The safety reminder, once more

The wheel has real torque (2 N·m stall) and `control` stops it fast, not gently. Keep
it off the ground or clear of fingers and cables before commanding motion — and
remember that `kill -9` coasts rather than brakes, so it isn't an emergency stop. Cut
power for that.
