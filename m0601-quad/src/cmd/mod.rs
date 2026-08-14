//! Subcommand implementations, split by role. Ordered like the bring-up
//! sequence: `check` (no port) → `check --probe` (read-only) → `monitor` (no
//! motion) → `jog`/`calibrate` (one wheel, bounded) → `drive` (the TUI).
//!
//! The read-only and single-wheel bring-up commands live in [`bringup`]; the
//! `drive` TUI and its thread orchestration in [`drive`](mod@drive). This holds
//! only what both share: the [`CmdResult`] alias, the [`PollQuery`] helper, and
//! [`find_wheel`]. The handlers are re-exported flat so callers use
//! `cmd::check`, `cmd::drive`, … regardless of which file they live in.

pub mod bringup;
pub mod drive;

pub use bringup::{calibrate, check, jog, monitor, stop};
pub use drive::{DriveFlags, drive};

use std::error::Error;
use std::time::Duration;

use crate::config::{Config, End, Side, WheelCfg};

pub(crate) type CmdResult = Result<(), Box<dyn Error>>;

/// Small extension so monitor/jog can query without importing protocol
/// details everywhere.
pub(crate) trait PollQuery {
    fn poll_query(&mut self, wait: Duration) -> m0601::Result<Option<m0601::Feedback>>;
}

impl PollQuery for m0601::M0601 {
    fn poll_query(&mut self, wait: Duration) -> m0601::Result<Option<m0601::Feedback>> {
        let frame = m0601::protocol::frame_feedback(self.id());
        self.transact(&frame, wait)
    }
}

/// Resolve `front-left` / `fl` / `front-driver` … to a configured wheel.
pub(crate) fn find_wheel<'c>(cfg: &'c Config, name: &str) -> Result<&'c WheelCfg, String> {
    let n = name.to_ascii_lowercase();
    let (end, side) = match n.as_str() {
        "front-left" | "fl" | "front-driver" => (End::Front, Side::Left),
        "front-right" | "fr" | "front-pass" => (End::Front, Side::Right),
        "rear-left" | "rl" | "rear-driver" | "back-left" => (End::Rear, Side::Left),
        "rear-right" | "rr" | "rear-pass" | "back-right" => (End::Rear, Side::Right),
        _ => {
            return Err(format!(
                "unknown wheel \"{name}\" (use front-left/front-right/rear-left/rear-right)"
            ));
        }
    };
    cfg.wheels
        .iter()
        .find(|w| w.end == end && w.side == side)
        .ok_or_else(|| format!("no wheel configured at {end}-{side}"))
}
