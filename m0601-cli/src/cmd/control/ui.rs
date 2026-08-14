//! The control TUI thread: a raw-mode event loop tying the dashboard renderer
//! ([`draw`](super::draw)) to key handling ([`keys`](super::keys)).
//!
//! This thread never touches the serial port; it only edits [`Shared`].

use std::io;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crossterm::event::{self, Event, KeyEventKind};

use super::draw::draw;
use super::keys::handle_key;
use super::state::Shared;

pub fn run(shared: &Shared, port: &str, id: u8, preset_rpm: i16) -> io::Result<()> {
    let mut out = io::stdout();
    while shared.running.load(Ordering::Relaxed) {
        draw(&mut out, shared, port, id)?;
        // ~10 Hz redraw; keys are handled as they arrive.
        if event::poll(Duration::from_millis(100))?
            && let Event::Key(key) = event::read()?
            && key.kind != KeyEventKind::Release
        {
            handle_key(shared, key.code, key.modifiers, preset_rpm);
        }
    }
    Ok(())
}
