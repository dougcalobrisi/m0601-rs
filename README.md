# m0601 — a Rust library + CLI for the DFRobot M0601 hub motor

[![CI](https://github.com/dougcalobrisi/m0601-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/dougcalobrisi/m0601-rs/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/m0601.svg)](https://crates.io/crates/m0601)
[![docs.rs](https://img.shields.io/docsrs/m0601)](https://docs.rs/m0601)
[![MSRV](https://img.shields.io/badge/MSRV-1.88-blue)](https://github.com/dougcalobrisi/m0601-rs#build--install)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/dougcalobrisi/m0601-rs/blob/main/LICENSE)

A reusable driver crate
([`m0601/`](https://github.com/dougcalobrisi/m0601-rs/tree/main/m0601)) and a CLI
([`m0601-cli/`](https://github.com/dougcalobrisi/m0601-rs/tree/main/m0601-cli),
binary `m0601`) for the DFRobot **M0601** direct-drive hub motor over half-duplex
RS485.

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
though some reference implementations dispute it) — by default the driver
does not reject telemetry on it, but the opt-in strict mode
(`Bus::with_strict_crc` / `M0601::with_strict_crc`) turns a bad checksum
into `Ok(None)`.

Telemetry replies come in **two layouts**: a `0x74` query reply carries the
winding temperature and a coarse 8-bit position, while replies to drive
frames carry a fine 16-bit position and no temperature. The library decodes
each reply by the command that elicited it.

Documentation map — the full docs live in
[`docs/content/docs/`](https://github.com/dougcalobrisi/m0601-rs/tree/main/docs/content/docs)
(a Hugo site; `cd docs && hugo server` to read it rendered):

- **[Getting started](https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/getting-started.md)**
  and the **[first-spin tutorial](https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/tutorial.md)**
  — build, wire, permissions, and a wheel actually turning.
- **[Safety](https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/safety.md)**
  — what brakes, what coasts, and what will hurt you. Short; read it before the
  wheel is on the ground.
- **[CLI guide](https://github.com/dougcalobrisi/m0601-rs/tree/main/docs/content/docs/cli)**
  — a page per subcommand: output samples, exit codes, footguns.
- **[Library guide](https://github.com/dougcalobrisi/m0601-rs/tree/main/docs/content/docs/library)**
  — drive loops, modes, telemetry, mirroring, bus budgeting, odometry, testing
  with mocks.
- **[Sample code](https://github.com/dougcalobrisi/m0601-rs/tree/main/docs/content/docs/samples)**
  — the runnable code in this repo (see below).
- **[Concepts](https://github.com/dougcalobrisi/m0601-rs/tree/main/docs/content/docs/concepts)**
  — why the driver behaves the way it does: the fail-safe, the bus, echoes,
  stopping, adapter latency.
- **[Protocol reference](https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/protocol.md)**
  — spec tables, wiring, every frame byte-by-byte, both reply layouts, and the
  known contradictions between sources.
- **[`docs.rs/m0601`](https://docs.rs/m0601)** (or `cargo doc --open -p m0601`) —
  the library API contract.
- **[CHANGELOG.md](https://github.com/dougcalobrisi/m0601-rs/blob/main/CHANGELOG.md)**
  — what changed, per release.

One rule worth carrying into any code you write against this: **a zero
setpoint does not mean "stop".** It only does in velocity mode — the same
zero-valued frame commands a move to 0° in position mode and zero torque
(a coast) in current mode.

## Build & install

Needs Rust **1.88** or newer (edition 2024 plus let-chains). Linux is the
tested platform; the serial layer is portable, but the `/dev/ttyUSB0` paths
and the `dialout` group below are Linux-specific.

```sh
git clone https://github.com/dougcalobrisi/m0601-rs.git
cd m0601-rs
cargo build --release              # binary at target/release/m0601
cargo install --path m0601-cli     # or install `m0601` into ~/.cargo/bin
```

If opening `/dev/ttyUSB0` fails with a permission error, add yourself to the
`dialout` group: `sudo usermod -aG dialout $USER` (log out and back in).

## Before you spin it

`control` and `drive` start driving the motor **immediately**, with no
confirmation prompt. A direct-drive hub motor has no gearbox to slow it
down, and the `1`–`5` presets reach 250 RPM.

- Clear the wheel, and secure the chassis so it cannot drive itself off the
  bench.
- Remember that **a zero setpoint does not mean "stop"** outside velocity
  mode (see above).
- `Ctrl-C` brakes. So does every other exit path — but only while the
  process is alive to do it.

## Usage

Global flags, valid before or after the subcommand: `--port /dev/ttyUSB0`,
`--id 0x01` (hex or decimal), `--timeout 0.15` (seconds); accepted ranges are in
the CLI overview page. Data goes to stdout and diagnostics to stderr, so
`m0601 info > readout.txt` captures only the readout.

```sh
m0601 scan                     # discover motor IDs (broadcast)
m0601 scan --full              # poll every ID 0x01..0xFE (~40 s)
m0601 info                     # config + one-shot live readout
m0601 monitor --hz 5           # live line dashboard, Ctrl+C to stop
m0601 monitor --csv log.csv    # ... also log rows to CSV
m0601 control --rpm 100        # full-screen keyboard control (see below)
m0601 control --accel 1        # ... with the motor's fastest ramp (default 3)
m0601 drive velocity --rpm 100 --secs 3  # spin at 100 RPM for 3 s, then brake
m0601 drive current --amps 1.0           # hold ~1 A of torque (until Ctrl-C)
m0601 drive position --deg 180           # rotate to 180° and hold (needs <10 RPM)
m0601 set-id --new 0x02        # change the motor's persistent RS485 ID
m0601 set-id --new 0x02 --yes  # ... skipping the confirmation prompt
m0601 raw "01 74 00 00 00 00 00 00 00"   # arbitrary frame, CRC auto-added
m0601 raw --yes "01 64 00 64 00 00 03 00 00"  # a motion frame needs --yes
```

`raw` refuses the two command bytes that can move the wheel — `0x64` (drive) and
`0xA0` (mode switch) — unless you pass `--yes`, and brakes the motor the frame
addressed (byte 0) on exit when it sends one — except a broadcast `C8` drive
frame, which commands every motor while a unicast brake covers only one. It still
has none of `drive`'s other rails: no loop, and no position-mode pre-flight
check.

`drive` is the scriptable counterpart to `control`: it holds one setpoint in
one mode — `velocity` (RPM), `current` (amps), or `position` (degrees) —
resending at 50 Hz until `--secs` elapses or you Ctrl-C, then it brakes.
Every exit path runs `safe_stop` (forces velocity, zeroes, brakes), so the
wheel is stopped on a clean exit, a signal, or a panic. Position mode is
refused at 10 RPM or above, per protocol, and also when no telemetry has
arrived — without a reading the speed is unknown, not zero.

`--secs` accepts 0–3600; omit it to drive until Ctrl-C.

`--timeout` governs `scan`, `info`, `monitor` and `set-id`. It does not apply
to `control` or `drive`, whose 50 Hz loops use a fixed 6 ms reply wait (only
`drive`'s pre-flight speed check, before entering position mode, waits the
full `--timeout`), and `raw` raises it to a 200 ms floor so a slow reply is
not missed.

`set-id` polls all 254 IDs before writing, because the set-ID frame is
unaddressed and would rename *every* motor that hears it — a broadcast scan
cannot prove only one is connected. Expect it to take ~40 s.

### `control` keys

| Key | Action |
|-----|--------|
| `F` / `B` | forward / backward at the `--rpm` preset (switches to velocity mode) |
| `1`–`5`   | 50–250 RPM (switches to velocity mode) |
| `←` / `→` | nudge ±10 RPM (velocity mode only) |
| `S`       | 0 RPM in velocity mode; hold the current angle in position mode; **zero torque — a coast, not a stop — in current mode** |
| `K`       | electric brake (velocity mode only; ignored in current and position mode) |
| `V`/`C`/`P` | switch mode: velocity / current / position |
| `Q` / `Esc` / `Ctrl-C` | quit — forces velocity mode, zeroes, then brakes |

**`control` latches: releasing a key does not stop the wheel.** `F`, `B`, and
`1`–`5` set a *sustained* setpoint that holds until `S`, `K`, `Q`, or a signal.
Do not walk away from a spinning wheel expecting it to stop on its own.

`--accel` sets the ramp used for active driving (default `3`, gentler than the
motor's fastest `1`) — a keystroke commands a large step, and the sharpest ramp
can trip the 3 A overcurrent protection on a loaded wheel. Larger is gentler;
`0` is **not** a middle setting, it selects the motor's default, which measures
identical to `1`. Keep it small — 120 RPM takes ~2 s at `5` and over 3 s at
`20`. See the [protocol notes](https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/protocol.md). The *stop* ramp
is separate; see below.

`P` is refused at 10 RPM or above, and also when no telemetry has arrived —
without a reading the speed is unknown, not zero. Entering position mode
seeds the target with the wheel's present angle, so the switch itself never
commands a move.

The status line shows the mode the motor *reports*, in red alongside the
requested one if the two ever disagree.

Safety: the wheel is stopped on every exit path — quit keys, panics,
SIGINT/SIGTERM/SIGHUP (e.g. a dropped SSH session). `safe_stop` sends 5×
mode-switch-to-velocity, then 5× velocity-0, then 5× brake, ~300 ms in all;
the velocity-0 rounds shed nearly half the speed (measured: 120 → ~64 RPM on
an unloaded wheel, against 119 RPM for coasting) and the brake rounds finish
the job. The `stop_accel` byte is tunable via `Bus::with_stop_accel` /
`BusTiming` but measures inert — `0` and `255` decelerate identically.
On SIGKILL or power loss the polling simply stops and the motor coasts, per
protocol. Keep the wheel clear before spinning it.

## Library: multi-motor bus + left/right mirroring

RS485 is multi-drop: several motors share one A/B pair, each with a unique
ID (`0x01..=0xFE` — assign them one at a time with `m0601 set-id`). A `Bus`
owns the port and mints cheap, cloneable, thread-safe per-motor handles.
The bus enforces a minimum idle gap between frames so no two can overlap on
the half-duplex wire (`with_min_gap` tunes it — drive frames elicit replies
even when unread, so unspaced sends corrupt), stops or mode-switches a
whole vehicle at once (`safe_stop_all` / `set_mode_all`, round-major so N
motors stop in the same ~300 ms as one), and requests low-latency delivery
from the kernel to defeat the FTDI 16 ms latency timer.
`mirrored(true)` makes "positive = robot forward" hold on a mirrored wheel
by negating velocity/current setpoints and flipping reported speed/current
signs (reported position passes through by default — angle mirroring depends on
your mechanical convention, so it's opt-in via `position_mirror`):

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

## Four-wheel example → full app

Start with the one-screen example — open a bus, mint four mirrored handles,
arm a stop guard, run a drive→poll→stop cycle, no TUI or scheduler in the way:

```console
cargo run --example four_wheel_minimal -- /dev/ttyUSB0
```

`m0601-quad` is that same wiring grown into a real application. It drives four
wheels as one skid-steer rover and doubles as
the reference implementation for multi-motor use of the library: a
TOML wheel map (`wheels.toml`) validated fail-closed, a single pilot
thread owning the bus at 55.6 Hz, latched fault handling with manual
re-arm, a 2×2 terminal dashboard, CSV logging, and a `--dry-run` mode
that opens no port. Bring-up order: `check --probe` → `monitor` →
`jog`/`calibrate` → `drive`. It is not published to crates.io — clone the repo
and run it from the workspace:

```console
cargo run -p m0601-quad -- --config m0601-quad/wheels.toml check
cargo run -p m0601-quad -- --config m0601-quad/wheels.toml drive --dry-run
```

Both are documented in full under
[Sample code](https://github.com/dougcalobrisi/m0601-rs/tree/main/docs/content/docs/samples).

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
test. A further set of known-answer frames is cross-checked against two
independent implementations that have driven real hardware — every byte of
those frames is mechanically determined by the frame layout and the CRC, so
they are reproducible facts about the wire rather than copied source (see
[the protocol reference](https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/protocol.md) and [NOTICE](https://github.com/dougcalobrisi/m0601-rs/blob/main/NOTICE)). Driver behavior (echo stripping, wrong-ID reply rejection, 5× frame
repeats, safe-stop sequencing) runs against an in-memory mock transport.
The `spin_and_stop` hardware test additionally requires
`M0601_ALLOW_MOTION=1` — it briefly spins the wheel.

## References

See [the protocol reference](https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/protocol.md) for the full spec with per-claim sourcing.

- [DDT M0601C_111 manual (PDF)](https://d2air1d4eqhwg2.cloudfront.net/media/files/a48110eb-432c-4083-a159-9e0f35913b23.pdf) — the manufacturer's 16-page datasheet
- [DDTRobot/motor-driver-examples](https://github.com/DDTRobot/motor-driver-examples) — the manufacturer's own sample code
- [DFRobot FIT1042 protocol wiki](https://wiki.dfrobot.com/fit1042/docs/23322)
- [DDT_M0601C_111, third-party samples (links the manual)](https://github.com/tech-life-hacking/DDT_M0601C_111)
- [navigation_robot, an independent C driver with test vectors](https://github.com/Il1yasviel/navigation_robot)
- [MotorLink, an independent implementation](https://github.com/MukeshSankhla/MotorLink)

## License

MIT — see [LICENSE](https://github.com/dougcalobrisi/m0601-rs/blob/main/LICENSE).

This project is not affiliated with DFRobot or Direct Drive Tech. It drives
physical hardware that can cause injury or damage; it comes with no warranty
of any kind, as set out in the license.
