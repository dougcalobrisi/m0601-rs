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
//! | Bus overcurrent   | 3 A                     | `0x02`    |
//! | Phase overcurrent | 4.6 A                   | `0x04`    |
//! | Over-temperature  | 80 °C (releases 75 °C)  | —         |
//! | Stall             | locked > 5 s            | `0x08`    |
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
//! - `ACCEL` is in units of 1 RPM / 0.1 ms; `0` selects the default.
//! - `BRAKE` = `0xFF` engages the electric brake (velocity mode only).
//! - Two special unaddressed frames exist: the broadcast ID query
//!   (`C8 64 00×7 DE`) and set-ID (`AA 55 53 <id> 00×6`, no CRC, must be
//!   sent 5×, one motor on the bus).
//!
//! Motor → host feedback replies:
//!
//! | Byte | 0  | 1    | 2–3                | 4–5             | 6       | 7            | 8      | 9   |
//! |------|----|------|--------------------|-----------------|---------|--------------|--------|-----|
//! |      | ID | mode | current (i16 BE)   | speed (i16 BE)  | temp °C | position u8  | faults | chk |
//!
//! Current scales ×8/32767 to amps; position ×360/255 to degrees. **Byte 9
//! of a reply is *not* a CRC-8/MAXIM** — never reject telemetry on CRC
//! ([`Feedback::crc_ok`] is informational only).
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
//!     Some(fb) => println!("{:+} RPM, {:.1} °C, faults: {}", fb.speed_rpm, fb.temp_c, fb.faults),
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
//! - [DFRobot FIT1042 protocol wiki](https://wiki.dfrobot.com/fit1042/docs/23322)
//! - [MotorLink, an independent implementation](https://github.com/MukeshSankhla/MotorLink)

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod bus;
pub mod error;
pub mod protocol;
pub mod transport;
pub mod types;

pub use bus::{Bus, M0601};
pub use error::{Error, Result};
pub use transport::{MockTransport, SerialTransport, Transport};
pub use types::{Faults, Feedback, Mode};
