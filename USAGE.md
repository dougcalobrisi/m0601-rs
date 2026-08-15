# Using m0601 — CLI and library guide

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
m0601 scan          # should print the motor's ID within a few seconds
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
| `--timeout` | `0.15` | reply wait in seconds (0-3600). Governs `scan`/`info`/`monitor`/`set-id`; `raw` raises it to a 200 ms floor, and in `drive` it covers the port open and the position pre-flight check only — both 50 Hz loops use a fixed 6 ms wait |

### `scan` — who's on the bus?

```sh
m0601 scan            # broadcast + poll IDs 0x01..0x0F, ~3 s
m0601 scan --full     # poll every ID 0x01..0xFE, ~40 s
```

The default scan broadcasts one query, then polls IDs `0x01..0x0F`
individually — motors ship at `0x01` and small fleets stay low, so that
covers the common case in seconds. The output always says which range was
polled; motors assigned a higher ID need `--full`.

The broadcast is unarbitrated, so **two motors can collide and look like
one or none**. When the collision garbles the reply so badly that no ID
can be read anywhere, `scan` says so and automatically escalates to the
full poll; if it read some IDs but also garbage, it lists what it found
and suggests `--full`. When you need certainty (e.g. before `set-id`),
use `--full` directly.

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
m0601 monitor --hz 20 --csv log.csv  # also log rows to log.csv (overwrites it)
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
| `S`       | 0 RPM in velocity mode; hold the current angle in position mode; **zero torque — a coast, not a stop — in current mode** |
| `K`       | electric brake (velocity mode only; ignored in current and position mode) |
| `V`/`C`/`P` | switch mode: velocity / current / position |
| `Q` / `Esc` / `Ctrl-C` | quit — forces velocity mode, zeroes, then brakes |

Notes on behavior you'll actually notice:

- `P` (position mode) is refused at 10 RPM or above (protocol constraint) and
  when no telemetry has arrived — an unknown speed is not zero. Entering
  position mode holds the wheel's *current* angle; it never jumps to 0°.
- The dashboard shows the mode the **motor reports**; if it ever differs
  from the requested one it turns red.
- Temperature updates every ~200 ms (it only arrives in the periodic
  telemetry query — drive replies don't carry it); `--` until the first.
- Every exit path stops the wheel — quit keys, panics, SIGINT/SIGTERM/
  SIGHUP (a dropped SSH session included). The stop ramps to zero at a
  moderate acceleration (the library default `SAFE_STOP_ACCEL` = 5; the
  library can override it via `Bus::with_stop_accel` / `BusTiming`, though
  `control` itself always uses the default) and then brakes — gentler than
  a hard step, to reduce the chance of tripping the overcurrent protection
  mid-stop. On SIGKILL or power loss the polling stops and the motor
  coasts, per protocol.

### `drive` — scriptable motion in one mode

`control` is interactive; `drive` is its batch counterpart. It holds a single
setpoint in one mode, resending at 50 Hz, until `--secs` elapses or you press
Ctrl-C — then it brakes. Each mode takes its own natural units:

```sh
m0601 drive velocity --rpm 100            # spin at 100 RPM until Ctrl-C
m0601 drive velocity --rpm -80 --secs 3   # reverse at 80 RPM for 3 s, then stop
m0601 drive velocity --rpm 200 --accel 40 # gentler ramp (accel 1 is the fastest)
m0601 drive current --amps 1.5 --secs 2   # hold ~1.5 A of torque for 2 s
m0601 drive position --deg 180            # rotate to 180° and hold
```

- **`--secs`** bounds the run (0-3600 s); omit it to drive until Ctrl-C. Either way the
  motor is braked on exit.
- **Units convert to the wire ranges**: `--rpm` clamps to ±330, `--amps`
  maps through ±32767 ≈ ±8 A (so ±8 A is the reachable limit), `--deg` maps
  0..360 onto 0..32767. Out-of-range values are rejected up front by the
  argument parser, not silently clamped on the wire.
- **`--accel`** (velocity only) is the ramp byte: `1` is the motor's *fastest*
  ramp and the default; a large step at accel 1 on a loaded wheel can spike
  current into the 3 A protection. Raise it (larger = gentler; `0` =
  motor default) to ramp gently.
- **Position mode is refused at 10 RPM or above** (protocol constraint) and when no
  telemetry has arrived — an unknown speed is not a zero one. The pre-flight
  speed check is the only part of `drive` that waits the full `--timeout`; the
  50 Hz loop uses a fixed 6 ms reply wait like `control`.
- **`safe_stop` on every exit path** — clean end, `--secs` timeout, Ctrl-C,
  SIGTERM/SIGHUP, or a panic — forces velocity mode, zeroes, then brakes. On
  SIGKILL or power loss the polling simply stops and the motor coasts.
- A live one-line readout (mode, speed, current, position, temp, faults)
  updates ~10 times a second while it runs. Winding temperature comes from an
  extra 0x74 query interleaved every 10th cycle, since drive replies don't
  carry it.

Remember the polling corollary: a zero setpoint is not universally "stop". If
you script `drive` runs back to back, each one brakes at the end, so there is
no coasting gap to worry about between them.

### `set-id` — assign a bus address

```sh
m0601 set-id --new 0x02        # asks you to type 'yes' to confirm
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

Depend on the crate by git:

```toml
[dependencies]
m0601 = { git = "https://github.com/dougcalobrisi/m0601-rs.git" }
```

Pin to a known-good state with `rev = "<sha>"`, `tag = "..."`, or
`branch = "main"` (default). `cargo update -p m0601` pulls the latest
commit of the pinned branch.

Or, working from a local checkout:

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

`drive_velocity` uses acceleration `1` by default — the motor's **fastest**
ramp. A big step at accel 1 on a loaded wheel can spike current into the 3 A
protection; use `drive_velocity_accel(rpm, accel)` with a larger value
(larger = gentler; `0` = motor default) to ramp gently for one call, or
change the default `drive_velocity` uses with `Bus::with_default_accel(n)`
(whole bus) or `motor.with_default_accel(n)` (one handle).

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

### Multi-motor robot: shared bus, mirroring, frame spacing

RS485 is multi-drop — all wheels share one A/B pair, each with its own
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

Position values are *not* mirror-adjusted (the correct transform depends
on your mechanical convention), and `Feedback::raw` always holds the
untouched wire bytes.

**Why those back-to-back calls are safe:** every drive (`0x64`) frame
elicits a reply, even when nothing reads it. Sent with no gap, the second
frame would go out while the first frame's reply is still on the
half-duplex pair — both corrupt, and in a periodic loop the *same* frame
corrupts every cycle, so one motor simply never moves. The bus prevents
this by enforcing a minimum idle gap between frames (default 2.5 ms;
`Bus::with_min_gap` tunes it, `Duration::ZERO` opts out). The gap is a
property of the shared port, so it holds across cloned handles and
threads: two threads *can* each drive their own wheel — but prefer one
scheduler thread that owns all sends when cycle timing matters, because
the gap only guarantees frames don't collide, not that they leave on
schedule.

The idle gap is one field of **`BusTiming`**, the bus's tunable timing —
along with the safe-stop ramp (`stop_accel`) and gap, and the
mode-switch / set-ID / broadcast waits. Every field defaults to the value
the crate has always used, so an unconfigured bus is unchanged. Set one
field with the matching builder (`with_min_gap`, `with_stop_accel`) or the
whole struct straight from your own config:

```rust
use m0601::{Bus, BusTiming};

let bus = Bus::open("/dev/ttyUSB0", timeout)?
    .with_timing(BusTiming { stop_accel: 5, ..BusTiming::default() });
```

Like the gap, the timing lives on the shared port — set it once at open
time and every motor handle minted from the bus uses it. (`m0601-quad`
does exactly this: its `Config::bus_timing()` feeds `limits.accel` into
`stop_accel`, so a wheel decelerates on the same ramp it launches on.)

**Budgeting more than two motors:** each motor must see *its* drive frame
at ≥50 Hz, so N motors need ≥N×50 frames/s through one bus, plus their
replies, plus gaps. Four wheels at the default gap is ~13.5 ms of bus
occupancy per 20 ms cycle before any telemetry is read — 4 × (the ~0.9 ms
frame time + the 2.5 ms gap that follows each frame). Keep
per-transaction reply waits short (the CLI's loops use 6 ms), read
telemetry round-robin — one motor per cycle, not all four — and never
*substitute* a query for a drive frame: the motor coasts through the
hole.

**Group operations:** `Bus::set_mode_all` switches every wheel in ~100 ms
and `Bus::safe_stop_all` stops the whole vehicle in the same ~300 ms as
one wheel. Both go round-major (each step's frame to every motor, then
the shared gap) — a vehicle whose wheels stop one at a time yaws while it
does. `Bus` is `Clone`, so a stop guard or signal handler can hold its
own handle to the same port.

**USB adapter latency:** FTDI adapters hold received bytes for up to
their 16 ms latency timer — longer than a whole reply window.
`SerialTransport::open` asks the kernel for low-latency delivery
automatically (`ASYNC_LOW_LATENCY`, needs no privileges on kernel
≥ 4.12); `SerialTransport::low_latency()` reports whether it stuck. If it
didn't, set the timer with a udev rule instead:

```text
# /etc/udev/rules.d/99-m0601.rules
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

and verify with `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.

Don't run `scan(0x01..=0xFE, ...)` concurrently with a drive loop on the same
bus — the scan holds the bus for ~254 × timeout and the driven motor will
coast.

**Reference implementation:** the `m0601-quad` crate in this repository
is the canonical multi-motor consumer — a four-wheel skid-steer app
showing the one-thread-owns-the-bus scheduling above (four spaced drive
frames per 18 ms cycle, with one round-robin poll thinned to every 2nd
cycle to stay inside the bus budget), the group stop on every
exit path, fail-closed startup, and the `invert`/`mirrored` sign
convention applied exactly once. Start from its `pilot.rs` when building
your own multi-motor loop.

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
| `P` refused in `control` | wheel at 10 RPM or above, or no telemetry yet |
| Motor ignores drive frames, fault bit set | a protection tripped (3 A bus / 4.6 A phase / 80 °C / stall) — it auto-clears in ~5 s (overheat: on cooling to 75 °C) |
| Two motors, chaos after set-id | the set-ID frame renamed both — reconnect one at a time and re-assign |

Safety, once more: the wheel is strong (2 N·m stall torque) and `control`
stops it fast, not gently. Keep it off the ground or clear of fingers and
cables before commanding motion.
