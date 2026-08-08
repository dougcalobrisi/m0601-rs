//! Time as a seam. The pilot never calls `Instant::now()` or
//! `thread::sleep` directly — through [`Clock`] a 1000-cycle scheduler
//! test runs in microseconds, which `MockTransport`'s zeroed pacing alone
//! cannot give us (the app's own deadline sleeps are outside the library).

#[cfg(test)]
use std::sync::{Arc, Mutex};
use std::time::Instant;

pub trait Clock {
    fn now(&self) -> Instant;
    /// Sleep until `deadline` (no-op if it already passed).
    fn sleep_until(&self, deadline: Instant);
}

/// Wall-clock time for the real rover.
#[derive(Clone, Copy, Default)]
pub struct RealClock;

impl Clock for RealClock {
    fn now(&self) -> Instant {
        Instant::now()
    }

    fn sleep_until(&self, deadline: Instant) {
        let now = Instant::now();
        if deadline > now {
            std::thread::sleep(deadline - now);
        }
    }
}

/// Simulated time: `sleep_until` jumps straight to the deadline.
#[cfg(test)]
#[derive(Clone)]
pub struct TestClock(Arc<Mutex<Instant>>);

#[cfg(test)]
impl TestClock {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(Instant::now())))
    }
}

#[cfg(test)]
impl Default for TestClock {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
impl Clock for TestClock {
    fn now(&self) -> Instant {
        *self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn sleep_until(&self, deadline: Instant) {
        let mut t = self.0.lock().unwrap_or_else(|e| e.into_inner());
        if deadline > *t {
            *t = deadline;
        }
    }
}
