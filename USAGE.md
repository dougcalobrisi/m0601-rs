# Using m0601-rs — CLI and library guide

Practical guide to driving the DFRobot M0601 hub motor with this repo.
Companion documents:

- [README.md](README.md) — overview and quick reference
- [PROTOCOL.md](PROTOCOL.md) — full wire protocol and hardware spec
- `cargo doc --open -p m0601` — the library API contract, in depth

## Contents

- [Hardware setup](#hardware-setup)
- [The one rule: it's a polling protocol](#the-one-rule-its-a-polling-protocol)
- [CLI](#cli)
- [Library](#library)
- [Troubleshooting](#troubleshooting)

## Hardware setup

1. **Power**: 18 V DC on the 2-pin cable (red = +, black = GND). The motor
   is silent on the bus until powered.
2. **RS485**: white = A(+), orange = B(−) to your USB-RS485 adapter
   (DFRobot recommends the RainbowLink TEL0185). The motor's A/B labels are
   inverted relative to many adapters — **if nothing answers, swap
   orange ↔ white** before debugging anything else.
3. **Brown wire → GND.** It is not optional; floating it causes
   intermittent comms errors.
4. Cable runs over ~1 m: add a 120 Ω termination resistor across A/B.
5. **Permissions** (Linux): if opening the port fails with a permission
   error, `sudo usermod -aG dialout $USER`, then log out and back in.

Sanity check the whole chain in one command:

```sh
m0601 scan          # should print the motor's ID within a second
```

## The one rule: it's a polling protocol

A drive command does not latch. The motor moves **only while drive frames
keep arriving** (~50 Hz; the motor accepts up to 500 Hz). Stop sending and
it coasts to a stop — that is the built-in fail-safe, and it shapes both
the CLI and the library API:

- CLI `control` runs a 50 Hz loop for you.
- Library `drive_*` methods send **one frame each**; your loop provides
  the cadence.

And the corollary worth repeating from the README: **a zero setpoint only
means "stop" in velocity mode.** The same zero-valued frame commands a move
to 0° in position mode and zero torque (a coast) in current mode. Use
`safe_stop` for shutdowns — it forces velocity mode first.

## CLI

Build and install:

```sh
cargo build --release              # binary at target/release/m0601
cargo install --path m0601-cli     # or install `m0601` into ~/.cargo/bin
```

Global flags work before or after the subcommand:

| Flag | Default | Meaning |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | serial port |
| `--id` | `0x01` | motor RS485 ID (hex `0x01` or decimal `1`) |
| `--timeout` | `0.15` | reply wait in seconds (`scan`/`info`/`monitor`/`set-id`) |

### `scan` — who's on the bus?

```sh
m0601 scan            # broadcast query, ~1 s
m0601 scan --full     # poll every ID 0x01..0xFE, ~40 s
```

The fast scan broadcasts one query; motors answer without arbitration, so
**two motors can collide and look like one or none**. When you need
certainty (e.g. before `set-id`), use `--full`.

### `info` — configuration + one-shot readout

```sh
$ m0601 info
================================================
  M0601 Configuration
================================================
  Port          : /dev/ttyUSB0
  Baud / format : 115200 8N1 (RS485 half-duplex)
  Motor ID      : 0x01 (1)
  Velocity range: -330..330 RPM
  Current range : -32767..32767 (~-8..+8 A)
  Position range: 0..32767 (0..360 deg)
------------------------------------------------
  Mode          : Velocity
  Speed         : +0 RPM
  Current       : -0.002 A
  Position      : 264.0 deg
  Winding temp  : 30 C
  Error         : 0x00  OK
  Raw frame     : 01 02 FF F9 00 00 1E BB 00 5F
================================================
```

Exits nonzero when the motor doesn't reply — usable in scripts.

### `monitor` — live readout, optional CSV

```sh
m0601 monitor --hz 5                 # one-line live dashboard, Ctrl+C stops
m0601 monitor --hz 20 --csv log.csv  # also append rows to log.csv
```

CSV columns: `timestamp,motor_id,mode,speed_rpm,current_a,temp_c,
position_deg,error_code,error_str,raw_hex`. Rows are flushed as written,
so a killed session keeps everything logged so far. Monitoring only
*queries* — it never drives the motor, so the wheel stays put (or keeps
doing whatever another controller tells it). A transient bus error is
reported and polling continues.

### `control` — interactive drive

```sh
m0601 control --rpm 100     # full-screen keyboard control
```

| Key | Action |
|-----|--------|
| `F` / `B` | forward / backward at the `--rpm` preset (switches to velocity mode) |
| `1`–`5`   | 50–250 RPM (switches to velocity mode) |
| `←` / `→` | nudge ±10 RPM (velocity mode only) |
| `S`       | stop — 0 RPM in velocity mode, hold current angle in position mode |
| `K`       | electric brake (velocity mode only) |
| `V`/`C`/`P` | switch mode: velocity / current / position |
| `Q` / `Esc` / `Ctrl-C` | quit — forces velocity mode, zeroes, then brakes |

Notes on behavior you'll actually notice:

- `P` (position mode) is refused above 10 RPM (protocol constraint) and
  when no telemetry has arrived — an unknown speed is not zero. Entering
  position mode holds the wheel's *current* angle; it never jumps to 0°.
- The dashboard shows the mode the **motor reports**; if it ever differs
  from the requested one it turns red.
- Temperature updates every ~200 ms (it only arrives in the periodic
  telemetry query — drive replies don't carry it); `--` until the first.
- Every exit path stops the wheel — quit keys, panics, SIGINT/SIGTERM/
  SIGHUP (a dropped SSH session included). The stop is a fast step to
  zero plus brake, not a gentle ramp. On SIGKILL or power loss the
  polling stops and the motor coasts, per protocol.

### `set-id` — assign a bus address

```sh
m0601 set-id --new 0x02        # prompts for confirmation
m0601 set-id --new 0x02 --yes  # skip the prompt
```

The set-ID frame is **unaddressed**: every motor that hears it takes the
new ID. The CLI therefore polls all 254 IDs first (~40 s) to prove only
one motor is connected. Wire one motor at a time when assigning IDs. The
ID persists across power cycles. Avoid `0xC8` (the broadcast address).

### `raw` — protocol probing

```sh
m0601 raw "01 74 00 00 00 00 00 00 00"       # 9 bytes: CRC appended
m0601 raw "01 74 00 00 00 00 00 00 00 FF"    # 10 bytes: sent verbatim
```

Prints TX and RX in hex and, when the sent command elicits telemetry,
decodes the reply using the correct layout for that command. Accepts
spaces or commas and optional `0x` prefixes.

## Library

Add the crate (path or git, it's not on crates.io):

```toml
[dependencies]
m0601 = { path = "../m0601-rs/m0601" }
```

### Query without moving anything

```rust
use std::time::Duration;
use m0601::M0601;

fn main() -> m0601::Result<()> {
    let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;
    match motor.query()? {
        Some(fb) => println!(
            "{:+} RPM, {:.1}°, {:?} °C, faults: {}",
            fb.speed_rpm, fb.position_deg, fb.temp_c, fb.faults
        ),
        None => println!("no reply — check power, wiring, --id"),
    }
    Ok(())
}
```

`Ok(None)` means the bus stayed silent — **that is not an error** (wrong
ID, unpowered motor). `Err` always means the port or OS failed.

### Drive: your loop provides the 50 Hz cadence

```rust
use std::time::{Duration, Instant};
use m0601::M0601;

fn main() -> m0601::Result<()> {
    let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;

    // Spin at 100 RPM for 3 seconds: one frame every 20 ms.
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        motor.drive_velocity(100)?;
        std::thread::sleep(Duration::from_millis(20));
    }

    motor.safe_stop(); // force velocity mode, zero, brake. Never errors.
    Ok(())
}
```

Call `safe_stop()` on **every** exit path of a control loop (including
panic/signal handlers — it swallows I/O errors precisely so it can run
there). If your process dies anyway, the motor coasts to a stop on its
own once frames stop arriving.

`drive_velocity` uses acceleration `1` — the motor's **fastest** ramp. A
big step at accel 1 on a loaded wheel can spike current into the 3 A
protection; use `drive_velocity_accel(rpm, accel)` with a larger value
(units: 1 RPM per 0.1 ms; `0` = motor default) to ramp gently.

### Telemetry while driving — the two reply layouts

Every drive frame's reply carries telemetry too. Use `transact` to drive
and read in one exchange, as a 50 Hz loop should:

```rust
use m0601::protocol::frame_velocity;

let frame = frame_velocity(motor.id(), 100, 1);
if let Some(fb) = motor.transact(&frame, Duration::from_millis(6))? {
    // Drive replies: hi-res position (~0.011°), NO temperature.
    assert!(fb.temp_c.is_none());
    println!("{:+} RPM at {:.2}°", fb.speed_rpm, fb.position_deg);
}
```

The two layouts (see PROTOCOL.md for the bytes):

| Reply to | `temp_c` | `position_deg` resolution |
|---|---|---|
| `query()` / `0x74` | `Some(°C)` | ~1.4° (8-bit) |
| drive frame / broadcast | `None` | ~0.011° (16-bit) |

Pattern for long-running loops: `transact` the drive frame every cycle,
and `query()` every ~10th cycle to refresh temperature (cache the last
value — that's exactly what the CLI's `control` does).

### Modes

```rust
use m0601::Mode;

motor.set_mode(Mode::Current)?;      // sends the switch 5×, ~100 ms
motor.drive_current(4096)?;          // ≈ 1 A of torque; resend at 50 Hz

motor.set_mode(Mode::Position)?;     // only below 10 RPM!
motor.drive_position(16384)?;        // ≈ 180°; resend at 50 Hz to hold
```

- Ranges: velocity ±330 RPM; current ±32767 ≈ ±8 A; position 0..=32767 =
  0°..360°. Out-of-range setpoints **clamp**, never wrap.
- The motor never acknowledges a mode switch; read back `Feedback::mode`
  to confirm.
- Remember: zero means stop / coast / "go to 0°" depending on the mode.

### Two-wheel robot: shared bus + mirroring

RS485 is multi-drop — both wheels share one A/B pair, each with its own
ID (assign them one at a time with `m0601 set-id`):

```rust
use std::time::Duration;
use m0601::Bus;

fn main() -> m0601::Result<()> {
    let bus = Bus::open("/dev/ttyUSB0", Duration::from_millis(150))?;
    let mut left = bus.motor(0x01)?.mirrored(true); // FIT1042 (left)
    let mut right = bus.motor(0x02)?;               // FIT1038 (right)

    // With the left wheel mirrored, "+100" moves the robot forward on
    // both sides: setpoints are negated on the way out, and reported
    // speed/current signs are flipped on the way in.
    left.drive_velocity(100)?;
    right.drive_velocity(100)?;   // ...resend both at >=50 Hz
    Ok(())
}
```

Handles are `Clone` and `Send`; each exchange locks the bus for exactly
one transaction, so two threads can each drive their own wheel. Position
values are *not* mirror-adjusted (the correct transform depends on your
mechanical convention), and `Feedback::raw` always holds the untouched
wire bytes.

Don't run `scan(true, ...)` concurrently with a drive loop on the same
bus — the scan holds the bus for ~254 × timeout and the driven motor will
coast.

### Error handling

```rust
match M0601::open(port, 0x01, timeout) {
    Err(e) if e.is_permission_denied() => {
        eprintln!("add yourself to dialout: sudo usermod -aG dialout $USER");
    }
    Err(e) => eprintln!("open failed: {e}"),
    Ok(motor) => { /* ... */ }
}
```

Telemetry is never rejected on its checksum (`Feedback::crc_ok` is
informational; genuine replies normally have it `true`), and a reply from
the wrong motor ID is dropped, surfacing as `Ok(None)`.

### Testing your code without hardware

`MockTransport` scripts the bus in memory — this is how the crate's own
driver tests work, and it's public API:

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

It can also simulate a half-duplex TX echo (`echo_tx`), a truncated echo
(`echo_truncate`), silence (empty/missing replies), and I/O failure
(`fail_io`).

## Troubleshooting

| Symptom | Check |
|---|---|
| `scan` finds nothing | 18 V power on? A/B swapped (orange ↔ white)? Brown → GND? |
| Permission denied on the port | `sudo usermod -aG dialout $USER`, re-login |
| Motor found but wrong `--id` | `m0601 scan` shows the real ID |
| Moves briefly, then stops | your loop is below ~50 Hz — the motor coasts between frames |
| Intermittent garbage / dropouts | brown wire floating; missing 120 Ω termination on long cable |
| `P` refused in `control` | wheel above 10 RPM, or no telemetry yet |
| Motor ignores drive frames, fault bit set | a protection tripped (3 A bus / 4.6 A phase / 80 °C / stall) — it auto-clears in ~5 s (overheat: on cooling to 75 °C) |
| Two motors, chaos after set-id | the set-ID frame renamed both — reconnect one at a time and re-assign |

Safety, once more: the wheel is strong (2 N·m stall torque) and `control`
stops it fast, not gently. Keep it off the ground or clear of fingers and
cables before commanding motion.
