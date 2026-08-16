---
title: Testing without hardware
weight: 8
---

# Testing without hardware

`M0601` and `Bus` are generic over a `Transport`, and one of the transports is a
mock. That means the same code you ship on hardware runs unchanged against a
scripted in-memory bus — no `cfg` flags, no seams to maintain. It's how the crate's
own driver tests work, and it's public API you can use for yours.

```rust
use std::time::Duration;
use m0601::{M0601, MockTransport};

let mock = MockTransport::with_replies([
    // one query reply: 100 RPM, 40 °C, position byte 0x00
    vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00],
]);
let mut motor = M0601::with_transport(mock, 0x01, Duration::from_millis(150))?;

let fb = motor.query()?.unwrap();
assert_eq!(fb.speed_rpm, 100);

// Then inspect exactly what your code put on the wire:
let mock = motor.into_transport().unwrap();
assert_eq!(mock.sent.len(), 1);
```

Two halves make it useful: you script what the "motor" says with `with_replies`, and
afterward you recover the transport with `into_transport` to assert on `sent` — every
frame your code transmitted, in order. So you can test both directions: did the motor
report what I expected me to do with it, and did I send the right bytes?

## Simulating the cases a loopback can't

The value of a mock over a real motor on a loopback is that it can produce the
failures you need to test but can't easily reproduce on demand:

- **Silence** — leave a reply out and the query returns `Ok(None)`, so you can test
  your no-reply handling.
- **A TX echo** (`echo_tx`) — many USB adapters echo the transmitted frame back
  ahead of the real reply; the driver strips it, and this lets you prove that.
- **A truncated echo** (`echo_truncate`) — the nasty one: a partial echo the driver's
  all-or-nothing stripping must *reject* rather than mistake for telemetry (see
  [Telemetry and echo]({{< relref "../concepts/telemetry-and-echo" >}}) for why this
  is dangerous).
- **I/O failure** (`fail_io`) — frames are still recorded before the failure, so you
  can assert what a best-effort path like `safe_stop` attempted even when the writes
  "failed."

Because the mock's `pace` returns zero instead of really sleeping, these tests run
instantly — a 50 Hz loop under test doesn't wait real milliseconds between frames.

That free speed has one consequence worth knowing: **you cannot test timing with
`MockTransport`.** If what you're asserting is the inter-frame gap itself, the mock
will report zero elapsed no matter what the bus did. The crate's own `tests/spacing.rs`
handles this with a small local transport that records `Instant`s and keeps
`Transport`'s *default* `pace` (which really sleeps). Do the same if you need to prove
spacing rather than sequence.

## Running against real hardware

The crate's hardware-in-the-loop tests live in `m0601/tests/hardware.rs` and are all
`#[ignore]`d, so `cargo test` skips them and CI never needs a motor. To run them you
opt in explicitly:

```sh
M0601_PORT=/dev/ttyUSB0 cargo test -p m0601 --test hardware -- --ignored --test-threads=1
```

| Variable | Meaning |
|---|---|
| `M0601_PORT` | **required** — the serial device to open |
| `M0601_ID` | motor address, decimal or `0x` hex (default `1`) |
| `M0601_ALLOW_MOTION` | set to `1` to enable `spin_and_stop`, the one test that turns the wheel |

`--test-threads=1` is not optional: the serial port is exclusive, so parallel tests
would fight over it. And `M0601_ALLOW_MOTION` is a separate gate from `--ignored` on
purpose — running the read-only hardware tests should never be a decision that spins
a wheel you forgot to clear.
