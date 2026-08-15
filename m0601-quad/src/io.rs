//! The app-level seam between the pilot and the wheels: [`WheelIo`] is
//! what the pilot needs from a wheel, `M0601` provides it for hardware,
//! and [`SimWheel`] provides it for `--dry-run` — building real 10-byte
//! frames and decoding them through `m0601::protocol::parse_feedback`, so
//! the simulator can never drift into a second source of truth about the
//! wire format.

use std::time::Duration;

use m0601::protocol::{ReplyKind, frame_query_reply, parse_feedback};
use m0601::transport::Transport;
use m0601::{Faults, Feedback, M0601, Mode};

/// What the pilot does to a wheel each cycle. Setpoints are in the
/// *rover's* frame — the sign convention is applied underneath (by
/// `M0601::mirrored` for hardware, internally by [`SimWheel`]).
pub trait WheelIo {
    /// One velocity drive frame, fire-and-forget.
    fn drive(&mut self, rpm: i16, accel: u8) -> m0601::Result<()>;
    /// One electric-brake frame.
    fn brake(&mut self) -> m0601::Result<()>;
    /// One `0x74` query (the only exchange that reads the bus — and the
    /// only one that drains the unread drive replies with it).
    fn poll(&mut self, wait: Duration) -> m0601::Result<Option<Feedback>>;
}

impl<T: Transport> WheelIo for M0601<T> {
    fn drive(&mut self, rpm: i16, accel: u8) -> m0601::Result<()> {
        self.drive_velocity_accel(rpm, accel)
    }

    fn brake(&mut self) -> m0601::Result<()> {
        M0601::brake(self)
    }

    fn poll(&mut self, wait: Duration) -> m0601::Result<Option<Feedback>> {
        // `query_with` sends the `0x74` feedback frame and decodes the reply —
        // exactly the `frame_feedback(id)` + `transact` this used to spell out.
        self.query_with(wait)
    }
}

/// A motor for `--dry-run`: first-order speed response toward the last
/// commanded setpoint, replies built as genuine wire frames.
pub struct SimWheel {
    id: u8,
    /// Mirrors `M0601::mirrored`: setpoints negate outbound, reported
    /// signs flip inbound — so the sim exercises the same sign path.
    reversed: bool,
    /// Wheel-frame speed (after the reversal), like a real motor's.
    speed: f32,
    target: f32,
    braking: bool,
}

impl SimWheel {
    pub fn new(id: u8, reversed: bool) -> Self {
        Self {
            id,
            reversed,
            speed: 0.0,
            target: 0.0,
            braking: false,
        }
    }
}

impl WheelIo for SimWheel {
    fn drive(&mut self, rpm: i16, _accel: u8) -> m0601::Result<()> {
        self.target = if self.reversed {
            -f32::from(rpm)
        } else {
            f32::from(rpm)
        };
        self.braking = false;
        // One cycle of first-order lag per drive frame — enough dynamics
        // for the dashboard to look alive and for `act` to chase `cmd`.
        self.speed += (self.target - self.speed) * 0.25;
        Ok(())
    }

    fn brake(&mut self) -> m0601::Result<()> {
        self.braking = true;
        self.target = 0.0;
        self.speed *= 0.5;
        Ok(())
    }

    fn poll(&mut self, _wait: Duration) -> m0601::Result<Option<Feedback>> {
        // Build the reply the way the motor would — the library's
        // `frame_query_reply` encodes the query layout (and its CRC) — then
        // DECODE it with the library's parser. If this app's idea of the
        // layout ever drifts, the driver is the arbiter at both ends.
        let speed = self.speed.round().clamp(-330.0, 330.0) as i16;
        let current_a = f32::from(speed) * 0.004 + 0.15;
        let frame = frame_query_reply(
            self.id,
            Mode::Velocity,
            current_a,
            speed,
            34,  // temperature °C
            0.0, // coarse position
            Faults(0),
        );
        let mut fb = parse_feedback(&frame, ReplyKind::Query);
        if self.reversed {
            // The same inbound adjustment M0601::mirrored applies.
            if let Some(fb) = fb.as_mut() {
                fb.speed_rpm = fb.speed_rpm.saturating_neg();
                fb.current_a = 0.0 - fb.current_a;
            }
        }
        Ok(fb)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_replies_decode_through_the_real_parser() {
        let mut w = SimWheel::new(0x03, false);
        for _ in 0..30 {
            w.drive(100, 5).expect("sim never fails");
        }
        let fb = w
            .poll(Duration::ZERO)
            .expect("sim never fails")
            .expect("always replies");
        assert_eq!(fb.id, 0x03);
        assert_eq!(fb.kind, ReplyKind::Query);
        assert!(fb.temp_c.is_some(), "query layout carries temperature");
        assert!((95..=100).contains(&fb.speed_rpm), "chases the setpoint");
    }

    #[test]
    fn a_reversed_sim_wheel_reports_in_the_rover_frame() {
        // Same contract as M0601::mirrored: command +100 in rover frame,
        // read back positive speed in rover frame.
        let mut w = SimWheel::new(0x01, true);
        for _ in 0..30 {
            w.drive(100, 5).expect("sim never fails");
        }
        let fb = w
            .poll(Duration::ZERO)
            .expect("sim never fails")
            .expect("always replies");
        assert!(
            fb.speed_rpm > 90,
            "reported in rover frame: {}",
            fb.speed_rpm
        );
    }
}
