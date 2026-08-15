//! The control TUI thread: a raw-mode event loop tying the dashboard renderer
//! ([`draw`](super::draw)) to key handling ([`keys`](super::keys)).
//!
//! This thread never touches the serial port; it only edits [`Shared`].

use std::io::{self, Write};
use std::sync::atomic::Ordering;
use std::time::Duration;

use crossterm::event::{self, Event, KeyEventKind};

use super::draw::draw;
use super::keys::handle_key;
use super::state::Shared;

pub fn run(shared: &Shared, port: &str, id: u8, preset_rpm: i16) -> io::Result<()> {
    let mut out = io::stdout();
    // Render into a buffer and write to the terminal only when the frame
    // actually changed. The poll thread refreshes telemetry ~10 Hz, but the
    // rendered dashboard is often identical tick-to-tick — skipping those
    // frames removes the idle repaint (and its flicker) and the wasted work.
    let mut frame: Vec<u8> = Vec::new();
    let mut last: Vec<u8> = Vec::new();
    while shared.running.load(Ordering::Relaxed) {
        frame.clear();
        draw(&mut frame, shared, port, id)?;
        if frame != last {
            out.write_all(&frame)?;
            out.flush()?;
            last.clone_from(&frame);
        }
        // ~10 Hz wake; keys are handled as they arrive.
        if event::poll(Duration::from_millis(100))?
            && let Event::Key(key) = event::read()?
            && key.kind != KeyEventKind::Release
        {
            handle_key(shared, key.code, key.modifiers, preset_rpm);
        }
    }
    Ok(())
}
