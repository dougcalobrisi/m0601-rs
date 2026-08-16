---
title: Troubleshooting
weight: 100
---

# Troubleshooting

Symptom on the left, the thing to check on the right. This page is deliberately just
the table — where a symptom has a **Why** link, that's the [FAQ]({{< relref "faq" >}})
entry which owns the full explanation, so nothing is explained twice in two places.

| Symptom | Check | Why |
|---|---|---|
| `scan` finds nothing | 18 V power on? A/B swapped (try orange ↔ white)? Brown wire → GND? A silent bus is almost always wiring. | [why]({{< relref "faq" >}}#empty-scan) |
| `Permission denied` on the port | `sudo usermod -aG dialout $USER`, then log out and back in. | |
| Found the motor, but `info` won't read it | Wrong `--id`, or the RX half of the wiring. `scan` shows the real address. | |
| Wheel spins briefly, then coasts | Your loop is under 50 Hz, or a query is replacing a drive frame on some cycles. Keep the drive cadence at 20 ms. | [why]({{< relref "faq" >}}#spins-then-stops) |
| Sent zero, but it moved or didn't brake | You weren't in velocity mode. Use `safe_stop` / the `S` key, which force velocity first. | [why]({{< relref "faq" >}}#zero-didnt-stop) |
| `control`'s `P` (or `drive position`) refused | Wheel at ≥10 RPM, or no telemetry yet (it fails closed on unknown speed). | [why]({{< relref "faq" >}}#position-refused) |
| Faults the instant a drive starts | Ramp too steep — accel `1` spikes current past the 3 A protection. Soften with a larger accel byte. Auto-resets in ~5 s. | [why]({{< relref "faq" >}}#fault-on-start) |
| Motor ignores drive frames, a fault bit is set | A protection is active (3 A bus / 4.6 A phase / 80 °C / stall). The motor stops responding to drive commands until it clears — ~5 s, or on cooling to 75 °C for overheat. | |
| `raw` refuses to send my frame | It's a motion command (byte 1 = `0x64` or `0xA0`). Pass `--yes`. | [why]({{< relref "faq" >}}#raw-by-design) |
| Intermittent garbage / dropouts | Brown wire floating, or missing 120 Ω termination on a cable over ~1 m. | [why]({{< relref "faq" >}}#intermittent) |
| Chaos after a `set-id` | The unaddressed frame renamed every motor that heard it. Reconnect one at a time and renumber. | [why]({{< relref "faq" >}}#set-id-renamed-all) |
| A rejected `--rpm` / `--amps` / `--deg` | Out of range. The CLI rejects rather than clamps; the library clamps. | [why]({{< relref "faq" >}}#out-of-range) |
| `kill -9` didn't stop the motor | Correct — nothing runs, so it coasts. Not an emergency stop; cut power. | [why]({{< relref "faq" >}}#kill-9-coasts) |
| Short reply waits read nothing | FTDI 16 ms latency timer. See [Latency]({{< relref "concepts/latency" >}}) — the udev rule fixes it. | |

## End-to-end checklist

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

## Safety reminder

The wheel has real torque (2 N·m stall) and `control` stops it fast, not gently. Keep
it off the ground or clear of fingers and cables before commanding motion — and
remember that `kill -9` coasts rather than brakes, so it isn't an emergency stop. Cut
power for that. The full picture is on the [Safety]({{< relref "safety" >}}) page.
