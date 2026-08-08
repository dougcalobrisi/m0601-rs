---
title: Testing without hardware
weight: 6
---

# Testing your code without hardware

`MockTransport` scripts the bus in memory — this is how the crate's own driver
tests work, and it's public API:

```rust
use std::time::Duration;
use m0601::{M0601, MockTransport};

let mock = MockTransport::with_replies([
    // A query reply: 100 RPM, 40 °C, position byte 0x00.
    vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00],
]);
let mut motor = M0601::with_transport(mock, 0x01, Duration::from_millis(150))?;
let fb = motor.query()?.unwrap();
assert_eq!(fb.speed_rpm, 100);

// Afterwards, inspect every frame your code sent:
let mock = motor.into_transport().unwrap();
assert_eq!(mock.sent.len(), 1);
```

It can also simulate:

- a half-duplex TX echo (`echo_tx`),
- a truncated echo (`echo_truncate`),
- silence (empty or missing replies → `Ok(None)`),
- I/O failure (`fail_io`).

Because `M0601` and `Bus` are generic over `Transport`, the same code you ship on
hardware runs unchanged against the mock — no cfg flags, no seams.
