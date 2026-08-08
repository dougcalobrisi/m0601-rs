//! Hardware wiring: config → open bus → four mirrored handles, plus the
//! stop guard every motion path arms before its first frame.

use std::time::Duration;

use m0601::{Bus, M0601};

use crate::config::Config;

/// OS-read backstop and probe reply wait. Generous on purpose — the drive
/// loop passes its own short `reply_wait` per transaction.
const OPEN_TIMEOUT: Duration = Duration::from_millis(150);

pub struct Rover {
    pub bus: Bus,
    /// Grid order (FL, FR, RL, RR), `mirrored()` already applied — from
    /// here on, +RPM means "rover forward" on every wheel and nothing
    /// else in the app ever touches a sign.
    pub wheels: Vec<M0601>,
    /// IDs in the same order, for the group operations.
    pub ids: Vec<u8>,
    /// Whether the kernel accepted the low-latency request.
    pub low_latency: bool,
}

pub fn open(cfg: &Config) -> m0601::Result<Rover> {
    let transport = m0601::SerialTransport::open(&cfg.bus.port, OPEN_TIMEOUT)?;
    let low_latency = transport.low_latency();
    let bus = Bus::with_transport(transport, OPEN_TIMEOUT).with_min_gap(cfg.min_gap());

    let mut wheels = Vec::new();
    let mut ids = Vec::new();
    for w in cfg.wheels_in_grid_order() {
        // The one place the sign convention is applied (invert XOR
        // mirrored); the library's tested transform does the rest.
        wheels.push(bus.motor(w.id)?.mirrored(w.reversed()));
        ids.push(w.id);
    }
    Ok(Rover {
        bus,
        wheels,
        ids,
        low_latency,
    })
}

/// Stops every wheel when dropped — armed BEFORE the first frame goes
/// out, so `?`-propagation and panics between "motors moving" and "loop
/// exited" still land on a vehicle-wide stop.
pub struct StopGuard {
    pub bus: Bus,
    pub ids: Vec<u8>,
}

impl Drop for StopGuard {
    fn drop(&mut self) {
        self.bus.safe_stop_all(&self.ids);
    }
}
