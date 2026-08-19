# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
The `m0601` library and the `m0601-cli` binary share one version and are released in
lockstep; `m0601-quad` is a sample and is not published.

## [Unreleased]

### Changed

- **The acceleration byte's ramp direction is no longer claimed anywhere.** The docs
  stated that "every source, the wiki included," says `1` is the fastest ramp and
  larger values are gentler. No source says that. The upstream DDT manual's only
  statement about the field is a unit — `RPM/0.1ms`, a *rate*, under which a larger
  byte ramps *harder* — and the DFRobot wiki contradicts that unit within the same
  sentence. Nothing has been measured against hardware. Every doc comment, CLI help
  string and docs-site page that asserted a direction now says it is unresolved, and
  `docs/content/docs/protocol.md` records what each source actually says
  ([#2](https://github.com/dougcalobrisi/m0601-rs/issues/2)).
- **Defaults that encoded the unverified direction are now `0`** — the motor's own
  default ramp — rather than a guess that may be backwards:
  `BusTiming::stop_accel` (was `5`), the `control` CLI's `--accel` (was `3`), and
  `m0601-quad`'s shipped `limits.accel` (was `5`). `m0601-quad` now warns on any
  nonzero `limits.accel` instead of only on `1`. `DEFAULT_DRIVE_ACCEL` stays `1`,
  which asserts nothing about direction: the wiki states the motor's default *is* `1`.

  Anyone relying on the old stop ramp can restore it with
  `Bus::with_stop_accel(5)` / `BusTiming { stop_accel: 5, .. }`.

  The direction-independent way to keep a launch or a stop under the 3 A
  bus-overcurrent trip is to bound the *step*, with `SlewLimiter` — not to pick a
  value for this byte. Settling the direction needs a hardware sweep; see
  [#2](https://github.com/dougcalobrisi/m0601-rs/issues/2).

## [0.1.0] — 2026-08-18

Initial public release.

### Added

- **`m0601`** — the driver library. Fixed 10-byte frame protocol at 115200 8N1 over
  half-duplex RS485, with:
  - `M0601` per-motor handles: `query`, `transact`, `set_mode`, `drive_velocity`,
    `drive_current`, `drive_position`, `brake`, and an infallible `safe_stop`.
  - `Bus`, which owns the port and mints cheap cloneable handles, enforces the
    inter-frame idle gap, and runs round-major group operations (`safe_stop_all`,
    `set_mode_all`) so a vehicle stops without yawing.
  - Left/right mirroring (`mirrored`, `PositionMirror`), so `+rpm` means "forward" on
    both sides of a chassis.
  - A pure, I/O-free `protocol` module: frame builders, both reply-layout parsers,
    CRC-8/MAXIM, and the scaling helpers.
  - `Telemetry` and `PositionAccumulator` for reconciling the two reply layouts and
    unwrapping a single-turn angle.
  - `BusTiming` for every pacing and stop tunable, plus `bus_period` / `frame_time` /
    `drive_floor` for sizing a multi-motor loop.
  - A `Transport` seam with `SerialTransport` and a public `MockTransport`, so driver
    logic is testable with no hardware.
  - Automatic `ASYNC_LOW_LATENCY` on Linux, defeating the FTDI 16 ms latency timer.
  - `#![deny(unsafe_code)]` with a single well-fenced exception, and workspace-wide
    `unwrap`/`expect`/`panic` denials — a panic in a 50 Hz drive loop is an
    uncontrolled motor.
- **`m0601-cli`** — the `m0601` binary: `scan`, `info`, `monitor`, `control`, `drive`,
  `set-id`, `raw`. Every motion path brakes on the way out — `drive` and `control`
  through a `Drop`-based stop guard, `raw --yes` through an explicit exit brake — so
  short of `SIGKILL`, power loss, or a signal landing after the handler failed to
  install (which the tools warn about at startup), an exit leaves the wheel braked.
- **`m0601-quad`** — a four-wheel skid-steer sample app and the reference
  implementation for multi-motor use (`publish = false`).
- The documentation site under `docs/`.

- Documentation for the examples and the `m0601-quad` sample app, plus library pages
  for wire budgeting (`bus_period`) and odometry (`PositionAccumulator`).
- Unit tests for `raw`'s exit-brake path: the post-parse logic now runs over any
  `Transport`, so a `MockTransport` verifies the brake chases the addressed motor,
  falls back to `--id` on broadcast, skips non-motion frames, and still fires when
  the exchange itself errors.

### Security / safety hardening

Pre-release hardening from a review of `m0601-cli`. The safety architecture — the
stop-guard funnel, fail-closed deadlines, the fail-closed position guard, and the
panic-free lint discipline — held up; these address what the review did surface.

- `control` no longer hardcodes the motor's *fastest* velocity ramp. A single
  keystroke commands a large instantaneous step (a jump to the full preset, or an
  `F`→`B` reversal), which at accel `1` can spike current past the 3 A
  bus-overcurrent trip on a loaded wheel. Driving now uses a gentler default of `3`,
  exposed as `control --accel`.
- Quitting `control` no longer lurches in position mode. `quit()` used to zero the
  target, which in position mode means "drive to 0°" — a spurious move for the one
  frame before the loop observed `running == false`.
- `--timeout` gained a `0.005 s` floor. It doubles as the serial reply window, and
  `--timeout 0` left ~0.9 ms, turning every read into a false "no response".
- `raw` gates the two motion command bytes (`0x64` drive, `0xA0` mode switch) behind
  `--yes`, and brakes the motor the frame addressed (byte 0) on exit when it sends
  one — even when the exchange errors mid-flight — falling back to `--id` for
  broadcast frames, where the output says only one motor was braked.
- An opt-in hold-to-run (momentary-keys) mode for `control` was considered and
  deferred; the latching behavior is documented prominently instead.
- `--id` is validated to `0x01..=0xFE` at the argument boundary.
- `drive` gives up after ~1 s of consecutive hard transact errors (braking via the
  stop guard) instead of spinning forever printing "still driving" when the adapter
  has been unplugged.
- Signal-handler-failure warnings now say plainly that *any* signal — `Ctrl-C`
  included — coasts rather than brakes when the handler could not be installed.

### Changed (CLI ergonomics)

- Diagnostics moved from stdout to stderr across `info`, `scan`, `set-id`, `raw`, and
  `drive`, so `m0601 info > log.txt` captures only readout data.
- `monitor --hz` is honest: polling uses a short bounded reply wait rather than
  collapsing to ~`1/--timeout` on a slow or silent motor.
- `set-id` shows a progress bar during its ~40 s exhaustive pre-write scan instead of
  freezing silently.
- `control` no longer flickers: each frame is a synchronized update, and identical
  frames are not repainted.


### Changed (documentation and `raw`)

- The Hugo site under `docs/content/` is now the canonical documentation. `USAGE.md`
  and `PROTOCOL.md` are pointers into it rather than parallel copies, which is what
  let the two drift apart before.
- `raw`'s exit brake now targets the motor the frame actually addressed (byte 0)
  when that is a valid unicast id, falling back to `--id` for broadcast (`0xC8`)
  and reserved addresses — and the brake is attempted (best-effort) even when the
  exchange itself errors, since the frame may already be on the wire.

### Packaging and attribution

- `NOTICE` records every source the protocol reference draws on, with each one's
  licence: the DFRobot wiki, the MIT-licensed DDT vendor sample, and two projects
  that state no licence. It also states that no third-party code is vendored here,
  and why the known-answer test frames are reproducible facts about the wire —
  each byte is fixed by the frame layout and CRC-8/MAXIM — rather than borrowed
  expression. The README and `tests/vectors.rs` now say *cross-checked against*
  instead of *taken verbatim from*.
- `LICENSE` and `NOTICE` ship inside both published crates. `license = "MIT"` is
  only an SPDX label and neither `include` nor `readme` can reach outside a package
  directory, so each crate keeps its own copy, guarded by a CI job that fails if a
  copy drifts from the workspace root.
- `m0601-quad`'s `wheels.toml` defaults to `/dev/ttyUSB0`, matching every other
  example in the tree, and labels its timing block as measured on one rig rather
  than as protocol-derived values to copy verbatim.

### Fixed before release

- Documentation resynced with the CLI hardening below — most importantly `raw`, whose
  page still described the pre-hardening behaviour of having no stop guard at all.
- Corrected the four-wheel bus-occupancy figure (≈13.5 ms per 20 ms cycle, not 10 ms)
  and the garbled speed/current rows in the protocol spec table.
- Qualified the `raw` exit-brake guarantee: the brake targets the addressed motor, but
  a frame sent to the broadcast address (`0xC8`) or a reserved one falls back to
  braking `--id` alone, which does not cover every motor a broadcast drive frame moved.
  Documented on the `raw` page, the FAQ, and the README.
- `four_wheel_minimal` now fits the bus budget it is cited as demonstrating: at the
  default 2.5 ms gap and a 5 ms reply wait its four drives plus one poll needed
  ~22.7 ms, more than its own 20 ms cycle. It tightens the gap and the reply wait to
  2 ms (~17.2 ms) and asserts the budget at startup.
- Qualified the "telemetry is never rejected on CRC" claim in the crate, `protocol`,
  and `Feedback::crc_ok` docs — true by default, but `with_strict_crc` opts out. The
  README and `concepts/telemetry-and-echo.md` were missed the first time and still
  stated it absolutely, contradicting `protocol.md` and `library/quickstart.md`.
- `m0601-quad`'s bring-up ladder now passes `--config`, so the commands work from the
  workspace root as the surrounding prose says they do.
- Runtime messages in `m0601-quad` and the doc-check example now point at the latency
  page for the udev rule rather than at `USAGE.md`, which no longer carries it. The
  two `m0601-quad` messages use the full GitHub URL (`LATENCY_DOC_URL`): a
  repo-relative path is unfollowable for anyone running an installed binary.
- `raw`'s exit-brake line no longer claims a fallback brake reached the addressed
  motor. It was keyed on `frame[0] == 0xC8` alone, so a reserved `0x00`/`0xFF` address
  — which falls back to `--id` exactly as broadcast does — printed "braked motor
  0xNN on exit" naming the address it had *not* braked. Now keyed on whether the
  brake target came from the frame, with a distinct line for the reserved case.
- The CLI's `control` default accel constant is renamed `CONTROL_DEFAULT_ACCEL`. It
  was `DEFAULT_DRIVE_ACCEL` (`3`), colliding by name with the library's
  `DEFAULT_DRIVE_ACCEL` (`1`) — a collision that had already produced a wrong FAQ
  answer about which default applies where.
- Corrected further docs-vs-code drift found in a sweep of the whole tree: the
  fault-reset timing on `library/telemetry.md` (overheat has no timer; the other four
  reset ~5 s after the *trip*, not after the condition clears), the claim that `drive`
  uses `Telemetry` (only `control` and `m0601-quad` do), the `budgeting.md` citation of
  a `check` subcommand and an `assert!` pattern `m0601-quad` deliberately does not use,
  the FAQ's `#empty-scan` answer (the CLI already auto-escalates that case), the
  `set-id` sample output (missing a printed line, and rendering the live progress bar
  as `... done.`), a dead `#known-contradictions` self-link in `protocol.md`, and the
  omission of `slew.rs` from the source-tree listing in `internals.md`.
- `usage_doc_check.rs` now covers `with_strict_crc`, `PositionMirror`,
  `PositionAccumulator`, and `bus_period`, so the "CI compiles these snippets"
  backstop actually spans the newer library pages.
- `SlewLimiter`'s documentation moved into the site, where the rest of it lives: the
  rationale is now `concepts/setpoint-shaping.md` and the cookbook entry is a section
  of `library/drive-loops.md`. `docs/setpoint-shaping.md` — a topic page outside
  `content/` — is gone, and the cookbook entry no longer sits in the gutted
  `USAGE.md`, where no reader is sent any more.


[Unreleased]: https://github.com/dougcalobrisi/m0601-rs/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/dougcalobrisi/m0601-rs/releases/tag/v0.1.0
