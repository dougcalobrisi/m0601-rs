---
title: Troubleshooting
weight: 40
---

# Troubleshooting

| Symptom | Check |
|---|---|
| `scan` finds nothing | 18 V power on? A/B swapped (orange ↔ white)? Brown → GND? |
| Permission denied on the port | `sudo usermod -aG dialout $USER`, re-login |
| Motor found but wrong `--id` | `m0601 scan` shows the real ID |
| Moves briefly, then stops | your loop is below ~50 Hz — the motor coasts between frames |
| Intermittent garbage / dropouts | brown wire floating; missing 120 Ω termination on long cable |
| `P` refused in `control` | wheel at 10 RPM or above, or no telemetry yet |
| Motor ignores drive frames, fault bit set | a protection tripped (3 A bus / 4.6 A phase / 80 °C / stall) — auto-clears in ~5 s (overheat: on cooling to 75 °C) |
| Two motors, chaos after `set-id` | the set-ID frame renamed both — reconnect one at a time and re-assign |

## Wiring checklist (no motors found?)

- 18 V power on?
- Brown wire → GND?
- A/B swapped? (try orange ↔ white)
- Right `--id`? Run `m0601 scan`.

## USB adapter latency

FTDI adapters can hold received bytes for up to 16 ms. The driver requests
low-latency delivery automatically; if a `udev` rule is needed, see [Multi-motor
bus → USB adapter latency]({{< relref "library/multi-motor" >}}).

## A safety reminder

The wheel is strong (2 N·m stall torque) and `control` stops it fast, not gently.
Keep it off the ground or clear of fingers and cables before commanding motion.
