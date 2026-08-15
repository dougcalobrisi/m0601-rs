//! The 2×2 dashboard and keyboard handling. ASCII, not box-drawing —
//! safer over ssh. This thread never touches the bus: it queues intent,
//! the pilot executes it.
//!
//! Color discipline: **red is reserved for what the machine reports; a
//! commanded value is never red** — it is what the operator asked for.

use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::style::{Color, Print, ResetColor, SetForegroundColor};
use crossterm::terminal::{self, Clear, ClearType, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::{cursor, queue};

use crate::config::Config;
use crate::state::{Shared, WheelState, lock};

/// Static per-wheel display facts, in grid order (FL, FR, RL, RR).
pub struct WheelInfo {
    pub label: String,
    pub id: u8,
    pub reversed: bool,
}

/// Restores the terminal on every exit path, panics included.
struct TermGuard;

impl TermGuard {
    fn arm() -> std::io::Result<Self> {
        terminal::enable_raw_mode()?;
        // Construct the guard IMMEDIATELY: if entering the alternate
        // screen fails below, the `?` drops it and Drop restores raw
        // mode — instead of leaving an SSH session wedged.
        let guard = Self;
        let mut out = std::io::stdout();
        queue!(out, EnterAlternateScreen, cursor::Hide)?;
        out.flush()?;
        Ok(guard)
    }
}

impl Drop for TermGuard {
    fn drop(&mut self) {
        let mut out = std::io::stdout();
        let _ = queue!(out, LeaveAlternateScreen, cursor::Show, ResetColor);
        let _ = out.flush();
        let _ = terminal::disable_raw_mode();
    }
}

/// One piece of a rendered line, with an optional color.
struct Seg(String, Option<Color>);

fn plain(s: impl Into<String>) -> Seg {
    Seg(s.into(), None)
}

/// `[      ###|         ]` — 21 cells, pipe at center, fill toward the
/// sign of `v` in `-1.0..=1.0`.
fn bar(v: f32) -> String {
    let mut cells = [' '; 21];
    cells[10] = '|';
    let n = (v.abs() * 10.0).round() as usize;
    for i in 0..n.min(10) {
        let idx = if v >= 0.0 { 11 + i } else { 9 - i };
        cells[idx] = '#';
    }
    let mut s = String::from("[");
    s.extend(cells);
    s.push(']');
    s
}

/// The four text lines of one wheel's box (fixed width), colored.
fn wheel_lines(info: &WheelInfo, w: &WheelState, cfg: &Config, now: Instant) -> [Vec<Seg>; 4] {
    let age = w.last_reply.map(|t| now.duration_since(t));
    let stale = age.is_none_or(|a| a >= cfg.stale());
    let amber_age = !stale && age.is_some_and(|a| a * 2 >= cfg.stale());

    let mut head = vec![plain(format!(" {:<12} 0x{:02X}", info.label, info.id))];
    if info.reversed {
        head.push(Seg("  REV".into(), Some(Color::DarkYellow)));
    }

    // A stale wheel shows dashes, never a remembered number — a dashboard
    // still showing 52 RPM for a wheel that stopped answering is
    // actively dangerous.
    let act = if stale {
        Seg("   --".into(), Some(Color::Red))
    } else {
        let fb = w.telemetry.fb.map(|fb| fb.speed_rpm).unwrap_or(0);
        plain(format!("{fb:+5}"))
    };
    let cmd_line = vec![
        plain(format!(" cmd {:+5}    act ", w.cmd_rpm)),
        act,
        plain(" RPM"),
    ];

    let mut meter = Vec::new();
    match w.telemetry.fb {
        Some(fb) if !stale => {
            let hot_current = f64::from(fb.current_a.abs()) >= cfg.limits.current_trip_a;
            meter.push(Seg(
                format!(" {:+.2} A", fb.current_a),
                hot_current.then_some(Color::Red),
            ));
        }
        _ => meter.push(Seg("   -- A".into(), Some(Color::Red))),
    }
    match w.telemetry.temp_c {
        Some(t) => {
            let color = if t >= 75 {
                Some(Color::Red)
            } else if t >= 70 {
                Some(Color::DarkYellow)
            } else {
                None
            };
            meter.push(Seg(format!("  {t:>3} C"), color));
        }
        None => meter.push(plain("   -- C")),
    }
    let age_txt = match age {
        Some(a) => format!("{:>6} ms", a.as_millis()),
        None => "  never".to_owned(),
    };
    meter.push(Seg(
        format!("  {age_txt}"),
        if stale {
            Some(Color::Red)
        } else if amber_age {
            Some(Color::DarkYellow)
        } else {
            None
        },
    ));

    let mut status = Vec::new();
    match w.telemetry.fb {
        Some(fb) => {
            if !fb.faults.is_ok() {
                status.push(Seg(format!(" {}", fb.faults), Some(Color::Red)));
            } else if fb.mode != Some(m0601::Mode::Velocity) {
                status.push(Seg(
                    format!(" mode {} (want Velocity)", fb.mode_name()),
                    Some(Color::Red),
                ));
            } else {
                status.push(plain(" OK"));
            }
        }
        None => status.push(Seg(" no telemetry yet".into(), Some(Color::Red))),
    }

    [head, cmd_line, meter, status]
}

fn pad_to(segs: &[Seg], width: usize) -> usize {
    width.saturating_sub(segs.iter().map(|s| s.0.chars().count()).sum())
}

/// Queue one full dashboard line at `row`, advancing it.
fn emit_line<W: Write>(out: &mut W, row: &mut u16, segs: &[Seg]) -> std::io::Result<()> {
    queue!(out, cursor::MoveTo(0, *row))?;
    for Seg(text, color) in segs {
        match color {
            Some(c) => queue!(out, SetForegroundColor(*c), Print(text), ResetColor)?,
            None => queue!(out, Print(text))?,
        }
    }
    *row += 1;
    Ok(())
}

fn draw(
    out: &mut impl Write,
    infos: &[WheelInfo; 4],
    shared: &Shared,
    cfg: &Config,
    port_label: &str,
) -> std::io::Result<()> {
    const HALF: usize = 38;
    let now = Instant::now();
    let wheels = *lock(&shared.wheels);
    let intent = *lock(&shared.intent);
    let trip = lock(&shared.trip).clone();
    let msg = lock(&shared.msg).clone();
    let dropped = shared.dropped_log_rows.load(Ordering::Relaxed);
    let overruns = shared.overruns.load(Ordering::Relaxed);

    queue!(out, cursor::MoveTo(0, 0), Clear(ClearType::All))?;
    let mut row: u16 = 0;

    emit_line(
        out,
        &mut row,
        &[plain(format!(
            "  M0601 QUAD  4-wheel skid steer     {port_label}   VELOCITY   max {} RPM",
            cfg.limits.max_rpm
        ))],
    )?;
    emit_line(
        out,
        &mut row,
        &[plain(format!(
            "  THROTTLE {:+4.0}%  {}   TURN {:+4.0}%  {}",
            f64::from(intent.throttle) * 100.0,
            bar(intent.throttle),
            f64::from(intent.turn) * 100.0,
            bar(intent.turn),
        ))],
    )?;
    emit_line(out, &mut row, &[plain("-".repeat(HALF * 2 + 1))])?;

    for pair in [[0usize, 1], [2, 3]] {
        let left = wheel_lines(&infos[pair[0]], &wheels[pair[0]], cfg, now);
        let right = wheel_lines(&infos[pair[1]], &wheels[pair[1]], cfg, now);
        for (l, r) in left.iter().zip(right.iter()) {
            let mut segs: Vec<Seg> = Vec::new();
            segs.extend(l.iter().map(|Seg(t, c)| Seg(t.clone(), *c)));
            segs.push(plain(" ".repeat(pad_to(l, HALF))));
            segs.push(plain("|"));
            segs.extend(r.iter().map(|Seg(t, c)| Seg(t.clone(), *c)));
            emit_line(out, &mut row, &segs)?;
        }
        emit_line(
            out,
            &mut row,
            &[plain(format!("{}+{}", "-".repeat(HALF), "-".repeat(HALF)))],
        )?;
    }

    if let Some(reason) = &trip {
        emit_line(
            out,
            &mut row,
            &[Seg(
                format!("  TRIPPED: {reason}   [R] re-arm"),
                Some(Color::Red),
            )],
        )?;
    }
    emit_line(
        out,
        &mut row,
        &[plain(
            "  W/S throttle +-10%   A/D turn +-10%   arrows fine +-2%   X straighten",
        )],
    )?;
    emit_line(
        out,
        &mut row,
        &[plain(
            "  SPACE ALL STOP   1-5 throttle 20..100%   K brake   R re-arm   Q/Esc quit",
        )],
    )?;
    emit_line(
        out,
        &mut row,
        &[plain(
            "  Throttle LATCHES (this is not hold-to-drive). Any unbound key = ALL STOP.",
        )],
    )?;
    let mut foot = vec![plain(format!(">> {msg}"))];
    if dropped > 0 {
        foot.push(Seg(
            format!("   [log: {dropped} rows dropped]"),
            Some(Color::DarkYellow),
        ));
    }
    if overruns > 0 {
        foot.push(Seg(
            format!("   [overrun x{overruns}]"),
            Some(Color::DarkYellow),
        ));
    }
    emit_line(out, &mut row, &foot)?;
    out.flush()
}

/// Apply one keypress to the shared intent. Returns `false` to quit.
fn handle_key(code: KeyCode, mods: KeyModifiers, shared: &Shared) -> bool {
    // Ctrl-C first: raw mode swallows the signal, and Ctrl-S/D/W are one
    // slip away from live bindings.
    if mods.contains(KeyModifiers::CONTROL) {
        if matches!(code, KeyCode::Char('c') | KeyCode::Char('C')) {
            shared.running.store(false, Ordering::Relaxed);
            return false;
        }
        // Any other control chord: deliberate no-op (not an all-stop —
        // terminals emit stray control sequences on resize/paste).
        return true;
    }

    let mut i = lock(&shared.intent);
    let step = |v: f32, d: f32| (v + d).clamp(-1.0, 1.0);
    match code {
        KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => {
            drop(i);
            shared.running.store(false, Ordering::Relaxed);
            return false;
        }
        KeyCode::Char('w') | KeyCode::Char('W') => i.throttle = step(i.throttle, 0.10),
        KeyCode::Char('s') | KeyCode::Char('S') => i.throttle = step(i.throttle, -0.10),
        // A/Left = turn left = +turn (CCW, REP-103); D/Right = −turn.
        KeyCode::Char('a') | KeyCode::Char('A') => i.turn = step(i.turn, 0.10),
        KeyCode::Char('d') | KeyCode::Char('D') => i.turn = step(i.turn, -0.10),
        KeyCode::Up => i.throttle = step(i.throttle, 0.02),
        KeyCode::Down => i.throttle = step(i.throttle, -0.02),
        KeyCode::Left => i.turn = step(i.turn, 0.02),
        KeyCode::Right => i.turn = step(i.turn, -0.02),
        KeyCode::Char('x') | KeyCode::Char('X') => i.turn = 0.0,
        KeyCode::Char(' ') => i.all_stop = true,
        KeyCode::Char('k') | KeyCode::Char('K') => i.brake = !i.brake,
        KeyCode::Char('r') | KeyCode::Char('R') => i.rearm = true,
        KeyCode::Char(c @ '1'..='5') => {
            i.throttle = f32::from(c as u8 - b'0') * 0.2;
        }
        _ => {
            // The teleop_twist_keyboard convention: an unbound key stops
            // the vehicle. Mashing the keyboard in a panic must be safe.
            i.all_stop = true;
            drop(i);
            shared.set_msg("unbound key — ALL STOP");
            return true;
        }
    }
    true
}

/// Run the dashboard until quit or `running` clears. Never touches the
/// bus. `port_label` is the port path, or "DRY RUN (no port)".
pub fn run(
    shared: &Arc<Shared>,
    infos: &[WheelInfo; 4],
    cfg: &Config,
    port_label: &str,
) -> std::io::Result<()> {
    let _guard = TermGuard::arm()?;
    let mut out = std::io::BufWriter::new(std::io::stdout());

    while shared.running.load(Ordering::Relaxed) {
        // The heartbeat the pilot's watchdog feeds on.
        *lock(&shared.ui_tick) = Instant::now();

        if event::poll(Duration::from_millis(100))?
            && let Event::Key(k) = event::read()?
            && k.kind != KeyEventKind::Release
            && !handle_key(k.code, k.modifiers, shared)
        {
            break;
        }
        draw(&mut out, infos, shared, cfg, port_label)?;
    }
    shared.running.store(false, Ordering::Relaxed);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stale_wheel_shows_dashes_not_its_last_speed() {
        let cfg = Config::parse(include_str!("../wheels.toml")).expect("shipped config");
        let info = WheelInfo {
            label: "FRONT-LEFT".into(),
            id: 0x03,
            reversed: false,
        };
        let mut w = WheelState::default();
        // A healthy-looking last reading... from 2 seconds ago.
        let mut frame = [0u8; 10];
        frame[0] = 0x03;
        frame[1] = 0x02;
        frame[4..6].copy_from_slice(&52i16.to_be_bytes());
        frame[9] = m0601::protocol::crc8_maxim(&frame[..9]);
        let fb = m0601::protocol::parse_feedback(&frame, m0601::ReplyKind::Query).unwrap();
        w.telemetry.absorb(fb);
        let now = Instant::now();
        w.last_reply = Some(now - Duration::from_secs(2));

        let lines = wheel_lines(&info, &w, &cfg, now);
        let cmd_line: String = lines[1].iter().map(|Seg(t, _)| t.as_str()).collect();
        assert!(
            cmd_line.contains("--"),
            "stale act must be dashes: {cmd_line:?}"
        );
        assert!(
            !cmd_line.contains("52"),
            "a remembered 52 RPM on a dead wheel is actively dangerous: {cmd_line:?}"
        );

        // Fresh telemetry shows the number.
        w.last_reply = Some(now);
        let lines = wheel_lines(&info, &w, &cfg, now);
        let cmd_line: String = lines[1].iter().map(|Seg(t, _)| t.as_str()).collect();
        assert!(cmd_line.contains("+52"), "fresh act shows: {cmd_line:?}");
    }

    #[test]
    fn unbound_keys_are_an_all_stop() {
        let shared = Shared::new();
        assert!(handle_key(KeyCode::Char('z'), KeyModifiers::NONE, &shared));
        assert!(lock(&shared.intent).all_stop, "z is unbound: must all-stop");
    }

    #[test]
    fn number_keys_latch_throttle_presets() {
        let shared = Shared::new();
        handle_key(KeyCode::Char('3'), KeyModifiers::NONE, &shared);
        let t = lock(&shared.intent).throttle;
        assert!((t - 0.6).abs() < 1e-6, "3 = 60%: {t}");
    }

    #[test]
    fn the_bar_renders_sign_and_magnitude() {
        assert_eq!(bar(0.0), "[          |          ]");
        assert_eq!(bar(0.5), "[          |#####     ]");
        assert_eq!(bar(-0.3), "[       ###|          ]");
        assert_eq!(bar(1.0), "[          |##########]");
    }
}
