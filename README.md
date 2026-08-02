# m0601-rs — M0601 hub motor library + CLI in Rust

A reusable driver crate ([`m0601/`](m0601)) and a CLI
([`m0601-cli/`](m0601-cli), binary `m0601`) for the DFRobot **M0601**
direct-drive hub motor over half-duplex RS485.

**M0601** is the motor model — a rebadged Direct Drive Tech **M0601C-111**;
**FIT1042** (left) and **FIT1038** (right) are DFRobot's SKUs for its
mirror-image builds. They are electrically identical and speak the same
protocol, so this one library covers both — see the mirror flag below.

The M0601 is **not Modbus**: fixed 10-byte frames at 115200 8N1, and a
*polling* protocol — the motor keeps moving only while drive frames keep
arriving at **~50 Hz** (official docs state only a 500 Hz maximum; the
50 Hz floor is the community consensus and matches observation). If the
host stops sending, the wheel coasts to a stop; that is the protocol's
built-in fail-safe.

Host frames carry a CRC-8/MAXIM in byte 9, with two exceptions: the
mode-switch frame puts the mode there instead, and the set-ID frame has no
checksum at all. Replies carry the same CRC (verified on real hardware,
though some reference implementations dispute it) — the driver still never
rejects telemetry on it.

Telemetry replies come in **two layouts**: a `0x74` query reply carries the
winding temperature and a coarse 8-bit position, while replies to drive
frames carry a fine 16-bit position and no temperature. The library decodes
each reply by the command that elicited it.

Documentation map:

- **[USAGE.md](USAGE.md)** — how to use it: hardware setup, every CLI
  subcommand, and a library cookbook (drive loops, modes, mirroring,
  testing with mocks, troubleshooting).
- **[PROTOCOL.md](PROTOCOL.md)** — the full protocol and hardware
  reference: spec tables, wiring, every frame byte-by-byte, both reply
  layouts, and the known contradictions between sources.
- **`cargo doc --open -p m0601`** — the library API contract.

One rule worth carrying into any code you write against this: **a zero
setpoint does not mean "stop".** It only does in velocity mode — the same
zero-valued frame commands a move to 0° in position mode and zero torque
(a coast) in current mode.

## Build & install

```sh
cargo build --release              # binary at target/release/m0601
cargo install --path m0601-cli     # or install `m0601` into ~/.cargo/bin
```

If opening `/dev/ttyUSB0` fails with a permission error, add yourself to the
`dialout` group: `sudo usermod -aG dialout $USER` (log out and back in).

## Usage

Global flags, valid before or after the subcommand: `--port /dev/ttyUSB0`,
`--id 0x01` (hex or decimal), `--timeout 0.15` (seconds).

```sh
m0601 scan                     # discover motor IDs (broadcast)
m0601 scan --full              # poll every ID 0x01..0xFE (~40 s)
m0601 info                     # config + one-shot live readout
m0601 monitor --hz 5           # live line dashboard, Ctrl+C to stop
m0601 monitor --csv log.csv    # ... also log rows to CSV
m0601 control --rpm 100        # full-screen keyboard control (see below)
m0601 set-id --new 0x02        # change the motor's persistent RS485 ID
m0601 set-id --new 0x02 --yes  # ... skipping the confirmation prompt
m0601 raw "01 74 00 00 00 00 00 00 00"   # arbitrary frame, CRC auto-added
```

`--timeout` governs `scan`, `info`, `monitor` and `set-id`. It does not
apply to `control`, whose 50 Hz loop uses its own fixed 6 ms reply wait, and
`raw` raises it to a 200 ms floor so a slow reply is not missed.

`set-id` polls all 254 IDs before writing, because the set-ID frame is
unaddressed and would rename *every* motor that hears it — a broadcast scan
cannot prove only one is connected. Expect it to take ~40 s.

### `control` keys

| Key | Action |
|-----|--------|
| `F` / `B` | forward / backward at the `--rpm` preset (switches to velocity mode) |
| `1`–`5`   | 50–250 RPM (switches to velocity mode) |
| `←` / `→` | nudge ±10 RPM (velocity mode only) |
| `S`       | stop — 0 RPM in velocity mode, hold current angle in position mode |
| `K`       | electric brake (velocity mode only) |
| `V`/`C`/`P` | switch mode: velocity / current / position |
| `Q` / `Esc` / `Ctrl-C` | quit — forces velocity mode, zeroes, then brakes |

`P` is refused at 10 RPM or above, and also when no telemetry has arrived —
without a reading the speed is unknown, not zero. Entering position mode
seeds the target with the wheel's present angle, so the switch itself never
commands a move.

The status line shows the mode the motor *reports*, in red alongside the
requested one if the two ever disagree.

Safety: the wheel is stopped on every exit path — quit keys, panics,
SIGINT/SIGTERM/SIGHUP (e.g. a dropped SSH session). `safe_stop` sends 5×
mode-switch-to-velocity, then 5× velocity-0, then 5× brake, ~300 ms in all;
it is a step to zero at the motor's fastest accel setting, not a gentle
ramp. On SIGKILL or power loss the polling simply stops and the motor
coasts, per protocol. Keep the wheel clear before spinning it.

## Library: multi-motor bus + left/right mirroring

RS485 is multi-drop: several motors share one A/B pair, each with a unique
ID (`0x01..=0xFE` — assign them one at a time with `m0601 set-id`). A `Bus`
owns the port and mints cheap, cloneable, thread-safe per-motor handles;
`mirrored(true)` makes "positive = robot forward" hold on a mirrored wheel
by negating velocity/current setpoints and flipping reported speed/current
signs (position passes through — angle mirroring depends on your mechanical
convention):

```rust
use std::time::Duration;
use m0601::Bus;

fn main() -> m0601::Result<()> {
    let bus = Bus::open("/dev/ttyUSB0", Duration::from_millis(150))?;
    let mut left = bus.motor(0x01)?.mirrored(true); // FIT1042 (left)
    let mut right = bus.motor(0x02)?;               // FIT1038 (right)
    // drive both "forward" — remember: resend at >=50 Hz to sustain motion
    left.drive_velocity(100)?;
    right.drive_velocity(100)?;
    Ok(())
}
```

## Wiring checklist (no motors found?)

- 18 V power on?
- Brown wire → GND?
- A/B swapped? (try orange ↔ white)
- Right `--id`? Run `m0601 scan`.

## Tests

```sh
cargo test --workspace                          # vectors + mock-bus + doctests
# hardware-in-loop (--test-threads=1 required: the serial port is exclusive)
M0601_PORT=/dev/ttyUSB0 cargo test -p m0601 --test hardware -- --ignored --test-threads=1
```

The protocol layer is validated byte-for-byte against golden vectors whose
expected bytes are written out as literals, derived from the DFRobot frame
layout and the CRC-8/MAXIM specification — the CRC implementation is
anchored to that algorithm's published check value (`crc8("123456789") ==
0xA1`), so no assertion recomputes its own expectation with the code under
test. A further set of known-answer frames is taken verbatim from two
independent implementations that have driven real hardware (see
[PROTOCOL.md](PROTOCOL.md)). Driver behavior (echo stripping, wrong-ID reply rejection, 5× frame
repeats, safe-stop sequencing) runs against an in-memory mock transport.
The `spin_and_stop` hardware test additionally requires
`M0601_ALLOW_MOTION=1` — it briefly spins the wheel.

## References

See [PROTOCOL.md](PROTOCOL.md) for the full spec with per-claim sourcing.

- [DFRobot FIT1042 protocol wiki](https://wiki.dfrobot.com/fit1042/docs/23322)
- [DDT M0601C-111 vendor sample code](https://github.com/tech-life-hacking/DDT_M0601C_111)
- [navigation_robot, an independent C driver with test vectors](https://github.com/Il1yasviel/navigation_robot)
- [MotorLink, an independent implementation](https://github.com/MukeshSankhla/MotorLink)
