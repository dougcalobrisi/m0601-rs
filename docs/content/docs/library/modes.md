---
title: Modes
weight: 3
---

# Modes

The motor runs one control loop at a time, and the active mode decides how it reads
the 16-bit value in a drive frame. Switch with `set_mode`, then drive:

```rust
use m0601::Mode;

motor.set_mode(Mode::Current)?;      // sends the 0xA0 switch frame 5×, ~100 ms
motor.drive_current(4096)?;          // ≈ 1 A of torque; resend at 50 Hz to hold

motor.set_mode(Mode::Position)?;     // only legal below 10 RPM
motor.drive_position(16384)?;        // ≈ 180°; resend at 50 Hz to hold
```

| Mode | Setpoint | Physical meaning |
|---|---|---|
| `Velocity` (default) | −330..330 (i16) | RPM |
| `Current` | −32767..32767 (i16) | ≈ −8..+8 A (`A = raw × 8 / 32767`) |
| `Position` | 0..32767 (u16) | 0°..360° (`deg = raw × 360 / 32767`) |

A few things the table doesn't tell you:

**Out-of-range setpoints clamp, they don't wrap.** Ask for more than ±330 RPM and you
get ±330, not an integer that rolled over into a reverse command. The clamp is
symmetric on purpose — even the current range is `±32767`, not `i16::MIN`, so nothing
you pass can wrap to the wrong sign.

**The motor never acknowledges a mode switch.** `set_mode` sends the switch frame
five times (it's idempotent, so repeating it just makes it stick) and returns, but
the motor sends nothing back to confirm. To know the switch took, read
`Feedback::mode` from the next reply. This is why the CLI's `control` dashboard shows
the *reported* mode and flags a mismatch in red — the requested mode is a hope until
telemetry confirms it.

**Position mode has an entry condition.** You can only switch into it while the wheel
is turning slower than 10 RPM. Above that the switch is refused by the motor. In your
own code, gate it on a telemetry reading first, and fail closed if you don't have
one — an unknown speed is not a slow speed.

**Zero means three different things.** In velocity mode a zero setpoint stops the
wheel; in position mode it commands a move to 0°; in current mode it commands zero
torque, which is a coast. This is the single most important fact about the mode
system, and it's why [`safe_stop`]({{< relref "drive-loops" >}}) forces velocity mode
before sending zero.
