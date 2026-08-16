//! Driver for the DFRobot **M0601** direct-drive hub motor over half-duplex
//! RS485. Covers both SKUs — **FIT1042** (left) and **FIT1038** (right) are
//! mirror-image builds of the same motor and speak the identical protocol;
//! see [`M0601::mirrored`] for making "forward" mean the same thing on both
//! sides of a chassis.
//!
//! The M0601 is **not Modbus**. It speaks a fixed 10-byte frame protocol at
//! 115200 8N1 with a CRC-8/MAXIM checksum, and it is a *polling* device:
//! motion is sustained only while the host keeps resending drive frames.
//! RS485 is multi-drop: several motors share one A/B pair (IDs
//! `0x01..=0xFE`) — a [`Bus`] owns the port and mints per-motor [`M0601`]
//! handles.
//!
//! # Safety and the polling protocol
//!
//! **A single drive command will not keep the wheel spinning.** The motor
//! moves only while drive frames arrive at
//! ≥[`DRIVE_HZ_MIN`](protocol::DRIVE_HZ_MIN) (50) Hz, up to
//! [`CMD_HZ_MAX`](protocol::CMD_HZ_MAX) (500) Hz. If the host stops —
//! crash, unplugged adapter, power loss — the motor **coasts to a stop**.
//! That is the protocol's built-in fail-safe; [`M0601::safe_stop`] upgrades
//! a coast to an active braked stop for orderly shutdowns and should be
//! called on every exit path of a control loop.
//!
//! One consequence deserves stating outright: **a zero setpoint does not
//! mean "stop"** except in velocity mode. The same zero-valued `0x64` frame
//! commands a move to 0° in position mode and zero torque in current mode,
//! which is why [`M0601::safe_stop`] establishes velocity mode before it
//! sends anything else.
//!
//! The motor also protects itself in hardware (each auto-resets after ~5 s):
//!
//! | Protection        | Trip                    | Fault bit |
//! |-------------------|-------------------------|-----------|
//! | Sensor error      | hall/encoder fault      | `0x01`    |
//! | Bus overcurrent   | 3 A                     | `0x02`    |
//! | Phase overcurrent | 4.6 A                   | `0x04`    |
//! | Stall             | locked > 5 s            | `0x08`    |
//! | Over-temperature  | 80 °C (releases 75 °C)  | `0x10`    |
//!
//! # Wire format
//!
//! Host → motor frames (see [`protocol`]):
//!
//! | Byte | 0  | 1   | 2      | 3      | 4 | 5 | 6     | 7     | 8 | 9   |
//! |------|----|-----|--------|--------|---|---|-------|-------|---|-----|
//! |      | ID | CMD | VAL_HI | VAL_LO | 0 | 0 | ACCEL | BRAKE | 0 | CRC |
//!
//! - `CMD` is `0x64` (drive), `0x74` (feedback query) or `0xA0` (mode
//!   switch). **For `0xA0` the last byte is the mode (`01`/`02`/`03`), not
//!   a CRC.**
//! - `ACCEL` sets ramp steepness: `1` is the fastest ramp, larger values
//!   are gentler, `0` selects the motor default.
//! - `BRAKE` = `0xFF` engages the electric brake (velocity mode only).
//! - Two special unaddressed frames exist: the broadcast ID query
//!   (`C8 64 00×7 DE`) and set-ID (`AA 55 53 <id> 00×6`, no CRC, must be
//!   sent 5×, one motor on the bus).
//!
//! Motor → host telemetry replies come in **two layouts**, selected by the
//! command that elicited them ([`ReplyKind`]):
//!
//! Reply to a `0x74` feedback query ([`ReplyKind::Query`]):
//!
//! | Byte | 0  | 1    | 2–3                | 4–5             | 6       | 7            | 8      | 9   |
//! |------|----|------|--------------------|-----------------|---------|--------------|--------|-----|
//! |      | ID | mode | current (i16 BE)   | speed (i16 BE)  | temp °C | position u8  | faults | chk |
//!
//! Reply to a `0x64` drive frame or the broadcast ID query
//! ([`ReplyKind::Drive`]) — no temperature, but a 16-bit position:
//!
//! | Byte | 0  | 1    | 2–3                | 4–5             | 6–7                  | 8      | 9   |
//! |------|----|------|--------------------|-----------------|----------------------|--------|-----|
//! |      | ID | mode | current (i16 BE)   | speed (i16 BE)  | position (u16 BE)    | faults | chk |
//!
//! Current scales ×8/32767 to amps; the 8-bit position ×360/255 and the
//! 16-bit position ×360/32767 to degrees. Replies carry a CRC-8/MAXIM in
//! byte 9 (verified on hardware). **By default** telemetry is not rejected
//! on it — [`Feedback::crc_ok`] is informational — but the opt-in strict
//! mode ([`Bus::with_strict_crc`] / [`M0601::with_strict_crc`]) turns a bad
//! checksum into `Ok(None)`. See `PROTOCOL.md`.
//!
//! # Multiple motors on one bus
//!
//! [`Bus`] enforces a minimum idle gap between frames
//! ([`Bus::with_min_gap`]) so no two frames — or a frame and the reply an
//! earlier drive frame elicited — can overlap on the half-duplex pair;
//! [`Bus::set_mode_all`] and [`Bus::safe_stop_all`] switch or stop every
//! wheel round-major, so a vehicle stops in the same ~300 ms as one motor.
//! Budget the wire: each motor needs its drive frame at ≥50 Hz, so N
//! motors put ≥N×50 frames/s (plus replies, plus gaps) through one bus.
//! [`bus_period`] computes that occupancy from [`frame_time`] and the gap;
//! a loop's cycle must exceed it yet stay within [`drive_floor`]. See
//! [Budgeting the wire] for the worked arithmetic.
//!
//! [Budgeting the wire]: https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/library/budgeting.md
//!
//! Coming from another fieldbus or motor-control ecosystem, the concepts
//! map directly:
//!
//! | Here | Elsewhere |
//! |------|-----------|
//! | enforced inter-frame gap ([`Bus::with_min_gap`]) | Modbus RTU's 3.5-character silence; CANopen's PDO inhibit time |
//! | coast when drive frames stop (the 50 Hz floor) | a command watchdog / failsafe timeout, permanently enabled |
//! | [`Bus::set_mode_all`] / [`Bus::safe_stop_all`] (reply-less batching) | Dynamixel's broadcast Sync Write |
//! | automatic low-latency request ([`SerialTransport::low_latency`](transport::SerialTransport::low_latency)) | pyserial's `set_low_latency_mode(True)` |
//!
//! # Control modes
//!
//! | [`Mode`]              | Wire | Value range        | Meaning        |
//! |-----------------------|------|--------------------|----------------|
//! | [`Mode::Current`]     | 0x01 | −32767..=32767     | ≈ −8 A..+8 A   |
//! | [`Mode::Velocity`]    | 0x02 | −330..=330         | RPM            |
//! | [`Mode::Position`]    | 0x03 | 0..=32767          | 0°..360°       |
//!
//! Setpoints outside these ranges are clamped, never wrapped. Mode switches
//! must be sent five times ([`M0601::set_mode`] does). Switching to position
//! mode requires the wheel to be under 10 RPM.
//!
//! # Example
//!
//! Real hardware:
//!
//! ```no_run
//! use std::time::Duration;
//! use m0601::M0601;
//!
//! # fn main() -> m0601::Result<()> {
//! let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;
//! match motor.query()? {
//!     // `query()` replies always carry the winding temperature.
//!     Some(fb) if fb.temp_c.is_some_and(|t| t < 70) => {
//!         println!("{:+} RPM, faults: {}", fb.speed_rpm, fb.faults);
//!     }
//!     Some(fb) => println!("running hot: {:?} °C", fb.temp_c),
//!     None => println!("no reply — check 18 V power, wiring (brown → GND), A/B polarity"),
//! }
//! # Ok(())
//! # }
//! ```
//!
//! No hardware needed — every driver behavior runs against
//! [`MockTransport`]:
//!
//! ```
//! use std::time::Duration;
//! use m0601::{M0601, MockTransport};
//!
//! # fn main() -> m0601::Result<()> {
//! let mock = MockTransport::with_replies([
//!     vec![0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00],
//! ]);
//! let mut motor = M0601::with_transport(mock, 0x01, Duration::from_millis(150))?;
//! let fb = motor.query()?.unwrap();
//! assert_eq!(fb.speed_rpm, 100);
//! # Ok(())
//! # }
//! ```
//!
//! # References
//!
//! The repository's [protocol reference] is the full protocol and hardware
//! reference, with per-claim sourcing and the known contradictions between
//! sources (every `PROTOCOL.md` mention in these docs points there — the
//! root `PROTOCOL.md` is now a pointer to that page). Primary materials:
//!
//! [protocol reference]: https://github.com/dougcalobrisi/m0601-rs/blob/main/docs/content/docs/protocol.md
//!
//! - [DFRobot FIT1042 protocol wiki](https://wiki.dfrobot.com/fit1042/docs/23322)
//! - [DDT M0601C-111 vendor sample](https://github.com/tech-life-hacking/DDT_M0601C_111)
//!   (the M0601 is a rebadged DDT M0601C-111)
//! - [navigation_robot, independent C driver](https://github.com/Il1yasviel/navigation_robot)
//! - [MotorLink, independent implementation](https://github.com/MukeshSankhla/MotorLink)

// `deny`, not `forbid`: the single place unsafe exists is the pair of Linux
// TIOCGSERIAL/TIOCSSERIAL ioctls in `low_latency` (scoped allow there, with
// the safety argument). Everything else remains unsafe-free.
#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod bus;
pub mod error;
#[cfg(target_os = "linux")]
mod low_latency;
pub mod protocol;
pub mod slew;
pub mod transport;
pub mod types;

pub use bus::{
    Bus, BusTiming, DEFAULT_DRIVE_ACCEL, DEFAULT_MIN_GAP, M0601, PositionMirror, ScanReport,
    bus_period,
};
pub use error::{Error, Result};
pub use protocol::{ReplyKind, drive_floor, frame_time};
pub use slew::SlewLimiter;
pub use transport::{MockTransport, SerialTransport, Transport};
pub use types::{Faults, Feedback, Mode, PositionAccumulator, Telemetry};
