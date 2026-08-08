---
title: Modes
weight: 3
---

# Modes

```rust
use m0601::Mode;

motor.set_mode(Mode::Current)?;      // sends the switch 5×, ~100 ms
motor.drive_current(4096)?;          // ≈ 1 A of torque; resend at 50 Hz

motor.set_mode(Mode::Position)?;     // only below 10 RPM!
motor.drive_position(16384)?;        // ≈ 180°; resend at 50 Hz to hold
```

| Mode | Setpoint range | Physical meaning |
|---|---|---|
| `Current` | −32767 … +32767 (i16) | ≈ −8 … +8 A (`A = raw × 8/32767`) |
| `Velocity` (default) | −330 … +330 (i16) | RPM |
| `Position` | 0 … 32767 (u16) | 0° … 360° (`deg = raw × 360/32767`) |

- Out-of-range setpoints **clamp**, never wrap.
- The motor never acknowledges a mode switch; read back `Feedback::mode` to
  confirm it took.
- Switching **into position mode requires the wheel to be turning slower than
  10 RPM**.
- Remember: zero means stop / coast / "go to 0°" depending on the mode — which is
  why [`safe_stop`]({{< relref "drive-loops" >}}) forces velocity mode first.
