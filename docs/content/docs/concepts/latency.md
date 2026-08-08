---
title: Latency
weight: 5
---

# Latency: the 16 ms that breaks everything

Here's a failure that looks like a protocol problem but is really a USB one. You send
a query, wait a reasonable 6 ms for the reply, read the port — and get nothing. Do it
again, same result. The motor is fine, your framing is fine, and the reply *did*
arrive. It's just still sitting inside the adapter.

FTDI-based adapters — the usual RS485 dongle — don't hand each byte to the host as it
lands. They batch received bytes until a USB packet fills or their **latency timer**
fires, and from the factory that timer is **16 ms**. A 10-byte reply that physically
arrived in under a millisecond can therefore sit in the adapter for up to 16 ms before
your program sees it. That's longer than this protocol's entire reply window, and it's
far longer than the 20 ms cycle a 50 Hz loop lives in. Short reply waits read nothing,
every time, and the bus looks dead.

The driver attacks this from two directions.

## Ask the kernel to shrink the timer

On Linux, `SerialTransport::open` automatically requests low-latency delivery — it
sets `ASYNC_LOW_LATENCY` on the tty, which tells the `ftdi_sio` driver to program the
timer down to 1 ms. This is the same thing `setserial /dev/ttyUSB0 low_latency` and
pyserial's `set_low_latency_mode(True)` do, and since kernel 4.12 it needs no special
privileges beyond being able to open the port. It's best-effort: if it doesn't stick
(a non-FTDI chip, an old kernel), that's not an error, and `SerialTransport::low_latency()`
reports whether it took.

If it didn't, set the timer yourself with a udev rule:

```text
# /etc/udev/rules.d/99-m0601.rules
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

and confirm it:

```sh
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # want 1, not 16
```

## Never block on data that hasn't come

The timer is only half of it. The read strategy matters just as much, and it's a
subtle one. After sending, the driver waits the reply window, then asks the OS *how
many bytes are buffered* and reads exactly that many — it never issues a blocking read
for data that might not exist.

Why that's essential: a blocking read waits up to the port's own timeout for bytes to
show up. Inside a 50 Hz loop, a reply that's a little late — or a motor that didn't
answer at all — would stall the read for the full port timeout and blow the 20 ms
cycle budget, dropping the wheel into a coast. By reading only what's already buffered
and treating the OS's `TimedOut` as "no more data" rather than as an error, a
transaction takes as long as the reply took and no longer. Silence costs 6 ms, not
150.

It mirrors pyserial's `write(); sleep(wait); read_all()` pattern, and it's the reason
`--timeout` deliberately doesn't reach into the `control` and `drive` loops: those use
a fixed short wait, precisely so one slow reply can't stretch a real-time cycle.
