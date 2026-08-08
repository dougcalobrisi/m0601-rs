---
title: Multi-motor bus
weight: 5
---

# Multi-motor robot: shared bus, mirroring, frame spacing

RS485 is multi-drop — all wheels share one A/B pair, each with its own ID (assign
them one at a time with [`m0601 set-id`]({{< relref "../cli/set-id" >}})). A `Bus`
owns the port and mints cheap, cloneable, thread-safe per-motor handles.

```rust
use std::time::Duration;
use m0601::Bus;

fn main() -> m0601::Result<()> {
    let bus = Bus::open("/dev/ttyUSB0", Duration::from_millis(150))?;
    let mut left = bus.motor(0x01)?.mirrored(true); // FIT1042 (left)
    let mut right = bus.motor(0x02)?;               // FIT1038 (right)

    // With the left wheel mirrored, "+100" moves the robot forward on both
    // sides: setpoints are negated on the way out, and reported speed/current
    // signs are flipped on the way in.
    left.drive_velocity(100)?;
    right.drive_velocity(100)?;   // ...resend both at >=50 Hz
    Ok(())
}
```

Position values are *not* mirror-adjusted (the correct transform depends on your
mechanical convention), and `Feedback::raw` always holds the untouched wire bytes.

## Why those back-to-back calls are safe

Every drive (`0x64`) frame elicits a reply, even when nothing reads it. Sent with
no gap, the second frame would go out while the first frame's reply is still on
the half-duplex pair — both corrupt, and in a periodic loop the *same* frame
corrupts every cycle, so one motor simply never moves.

The bus prevents this by enforcing a minimum idle gap between frames (default
2.5 ms; `Bus::with_min_gap` tunes it, `Duration::ZERO` opts out). The gap is a
property of the shared port, so it holds across cloned handles and threads: two
threads *can* each drive their own wheel — but prefer one scheduler thread that
owns all sends when cycle timing matters, because the gap only guarantees frames
don't collide, not that they leave on schedule.

## Budgeting more than two motors

Each motor must see *its* drive frame at ≥50 Hz, so N motors need ≥N×50 frames/s
through one bus, plus their replies, plus gaps. Four wheels at the default gap is
~10 ms of bus occupancy per 20 ms cycle before any telemetry is read. Keep
per-transaction reply waits short (the CLI's loops use 6 ms), read telemetry
round-robin — one motor per cycle, not all four — and never *substitute* a query
for a drive frame: the motor coasts through the hole.

## Group operations

`Bus::set_mode_all` switches every wheel in ~100 ms and `Bus::safe_stop_all`
stops the whole vehicle in the same ~300 ms as one wheel. Both go round-major
(each step's frame to every motor, then the shared gap) — a vehicle whose wheels
stop one at a time yaws while it does. `Bus` is `Clone`, so a stop guard or signal
handler can hold its own handle to the same port.

## USB adapter latency

FTDI adapters hold received bytes for up to their 16 ms latency timer — longer
than a whole reply window. `SerialTransport::open` asks the kernel for
low-latency delivery automatically (`ASYNC_LOW_LATENCY`, needs no privileges on
kernel ≥ 4.12); `SerialTransport::low_latency()` reports whether it stuck. If it
didn't, set the timer with a udev rule instead:

```text
# /etc/udev/rules.d/99-m0601.rules
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

and verify with `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.

> Don't run `scan(0x01..=0xFE, ...)` concurrently with a drive loop on the same
> bus — the scan holds the bus for ~254 × timeout and the driven motor will coast.
