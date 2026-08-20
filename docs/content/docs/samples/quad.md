---
title: m0601-quad
weight: 2
---

# `m0601-quad` — the four-wheel reference app

`m0601-quad` drives four M0601 wheels as one skid-steer rover, and it is this repo's
**reference implementation for multi-motor use of the library**. Where
[`four_wheel_minimal`]({{< relref "examples" >}}) shows you the API, `m0601-quad`
shows what an application built on that API has to add: a durable wheel map, a
dedicated scheduler thread, a safety state machine, a dashboard, and logging.

It is **not published to crates.io** (`publish = false`) — it's meant to be read and
adapted, not depended on. Clone the repo and run it from the workspace.

## Usage

```sh
cargo run -p m0601-quad -- --config m0601-quad/wheels.toml check
cargo run -p m0601-quad -- --config m0601-quad/wheels.toml drive --dry-run
```

Global options apply to every subcommand:

| Flag | Default | Meaning |
|---|---|---|
| `--config <PATH>` | `wheels.toml` | the wheel map |
| `--port <PATH>` | *(from the config)* | override the config's serial port |

| Subcommand | What it does |
|---|---|
| `drive` *(the default)* | the 2×2 driving dashboard |
| `check [--probe]` | validate the config and print the resolved wheel table; `--probe` also opens the port and reads each wheel once, read-only |
| `monitor` | headless telemetry + CSV; commands no motion |
| `jog --wheel <fl\|fr\|rl\|rr> [--rpm 60] [--secs 2.0]` | spin **one** wheel at a bounded speed for a bounded time, then stop |
| `calibrate` | walk all four wheels and print corrected `invert` lines to paste back into the config |
| `stop` | one-shot vehicle-wide safe stop |

`drive` takes four override flags, each of which relaxes one startup check:
`--dry-run` (simulate four wheels, open no serial port at all), `--ignore-silent`
(start even if a wheel doesn't answer the probe), `--ignore-motion` (start even if a
wheel is already turning above 5 RPM), and `--ignore-faults` (start even if a wheel
reports fault bits). The defaults **fail closed**; each flag is a deliberate override.

An unusable config exits `2` with a numbered list of what's wrong, before any port is
opened.

### Bring-up order

Work up the ladder — each step proves something the next one assumes:

From the workspace root, `--config` points at the wheel map (without it the commands
look for `wheels.toml` in the current directory and exit `2`):

```sh
CFG=m0601-quad/wheels.toml
cargo run -p m0601-quad -- --config $CFG check --probe   # config valid, all four wheels answer
cargo run -p m0601-quad -- --config $CFG monitor         # telemetry is sane; nothing moves
cargo run -p m0601-quad -- --config $CFG jog --wheel fl  # one wheel, bounded, turns the right way
cargo run -p m0601-quad -- --config $CFG calibrate       # fix the `invert` flags from observation
cargo run -p m0601-quad -- --config $CFG drive           # the whole vehicle
```

`--dry-run` on `drive` opens no serial port, so you can explore the dashboard with no
hardware at all.

> [!CAUTION]
> Everything past `monitor` moves a vehicle with four gearless hub motors. Get the
> wheels off the ground before `jog`, `calibrate`, or `drive`. →
> [Safety]({{< relref "../safety" >}})

## `wheels.toml` — the wheel map

The config is the single durable record of which motor ID sits at which corner.
Treat it like wiring: `check` validates it, `calibrate` tells you what to change, and
nothing ever writes it for you.

```toml
[bus]
port          = "/dev/ttyUSB0"
cycle_ms      = 18.0   # full 4-wheel cycle (55.6 Hz), under the 20 ms floor
min_gap_ms    = 2.0    # → Bus::with_min_gap; idle AFTER each frame
reply_wait_ms = 2.0    # per-poll reply window; measure it, don't guess

[limits]
max_rpm         = 120    # 100% throttle commands exactly this
accel           = 5      # NEVER 0 or 1 on a vehicle — see below
ramp_rpm_per_s  = 300.0  # host-side setpoint ramp; all-stop BYPASSES it
current_trip_a  = 2.5    # monitor trip, not a command clamp
current_trip_ms = 400.0  # debounce — start-up inrush is normal and shorter
stale_ms        = 500.0  # telemetry older than this: warn
dead_ms         = 1500.0 # telemetry older than this: stop the vehicle

[log]
path = "quad.csv"

[[wheel]]           # one block per corner
id       = 0x03
name     = "front driver"
side     = "driver" # left|right, aliases driver|pass
end      = "front"  # front|rear
invert   = false    # what YOU observed; `calibrate` tells you which to flip
mirrored = false    # the SKU's mechanical build (FIT1042 left / FIT1038 right)
```

Three of those values carry the lessons this repo learned the hard way:

- **`accel = 5`, never `0` or `1`.** Larger is gentler, and both `0` and `1` are the
  motor's *fastest* ramp — `0` selects the motor default, which
  [measures identical to `1`]({{< relref "../protocol" >}}#known-contradictions-between-sources).
  Four wheels launching off one supply at that ramp can trip the 3 A bus-overcurrent
  protection, so `check` warns on either value. Don't overshoot the other way: 120 RPM
  takes ~2 s at `5` and over 3 s at `20`, so past ~10 the rover crawls.
- **`cycle_ms = 18`, not 20.** Every wheel needs its drive frame at ≥50 Hz, so the
  cycle must stay *under* 20 ms; 18 leaves 2 ms of margin. A poll cycle consumes
  ~17.2 ms of that, which is why `check` says so out loud:

  ```
  [!] bus timing is tight: ~17.208ms of the 18ms cycle is occupied (<10% slack for OS jitter)
  ```

  That number is `bus_period(4, 1, 2 ms, 2 ms)` — see
  [Budgeting the wire]({{< relref "../library/budgeting" >}}). The 20 ms floor caps
  the total budget, so more cycle slack costs coast margin and vice versa; only a
  *measured* smaller `min_gap` buys both.
- **`invert` XOR `mirrored`.** `invert` is what you observed; `mirrored` is the SKU's
  build. Only their XOR matters, and the dashboard shows the effective direction as a
  `REV` badge — so you can record both truthfully instead of fudging one to fix the
  other.

## Demonstrated patterns

Read these in the source when you build your own multi-motor loop:

- **One thread owns the bus** (`pilot.rs`). Not thread-per-wheel: half-duplex RS485
  serializes everything anyway, so a second thread buys no parallelism and forfeits
  deterministic cycle timing. Each cycle sends four drive frames plus one safety
  verdict, and thins the round-robin telemetry poll to every *second* cycle — because
  **the four drive frames are never negotiable and the poll is the only optional
  exchange**. It's the "never substitute a query for a drive frame" rule enforced
  structurally.
- **The sign convention is applied exactly once** (`rover.rs`), at handle
  construction, so no downstream code negotiates left/right again.
- **Every exit path group-stops.** Quit key, `?`-propagation (the stop guard is armed
  before the first frame), a pilot panic (`catch_unwind`), a UI panic (terminal guard
  then stop guard), and SIGINT/SIGTERM/SIGHUP. On `SIGKILL` all four coast — which is
  documented in the crate, because a coasting rover *rolls*.
- **`BusTiming` filled from config.** `Config::bus_timing()` feeds `limits.accel`
  into `stop_accel`, so a wheel decelerates on the same ramp it launched on.
- **Latched faults with manual re-arm** (`safety.rs`), plus staleness tiers: one
  missed poll is expected, `stale_ms` warns, `dead_ms` stops the vehicle — because
  three driving wheels and one unknown wheel yaws the machine.

> [!NOTE]
> The crate's manifest carries a standing rule: **never set `panic = "abort"`** for
> `m0601-quad`. Both panic-recovery paths reach `safe_stop_all` by unwinding, and
> aborting would silently disable them.

## See also

- [Library → Multi-motor bus]({{< relref "../library/multi-motor" >}}) — the same
  ground from the API side.
- [Concepts → Where the driver ends]({{< relref "../concepts/driver-boundary" >}}) —
  why kinematics, PID, and config parsing live in `m0601-quad` and not in the driver.
