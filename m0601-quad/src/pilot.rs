//! The pilot thread: sole owner of the four wheels, the only code that
//! touches the bus. One thread — `Arc<Mutex<T>>` would permit
//! thread-per-wheel, but that gains no parallelism (half-duplex RS485
//! serializes everything) and forfeits deterministic cycle timing.
//!
//! Each cycle: four drive frames (the library spaces them on the wire) and
//! one safety verdict, and — every `POLL_EVERY`th cycle — one round-robin
//! telemetry poll. **The four drive frames are never negotiable; the poll
//! is the only optional exchange**, which is why it is the one thinned out
//! to keep cycles inside the bus budget. The poll is also the only exchange
//! that reads — and therefore drains — the bus, so the unread drive replies
//! that pile up between polls never accumulate unbounded. It runs at the
//! *top* of its cycle, a full sleep away from the previous cycle's last
//! drive frame, so that frame's late reply is already buffered (and
//! discarded by the poll's input clear) instead of landing inside the poll
//! window and being decoded as telemetry.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::SyncSender;
use std::time::Duration;

use m0601::SlewLimiter;

use crate::clock::Clock;
use crate::config::{Config, Side};
use crate::io::WheelIo;
use crate::mix::{DriveCmd, wheel_rpm};
use crate::safety::{Reaction, assess};
use crate::state::{Shared, WheelState, lock};

/// Zero all setpoints if the UI stops ticking for this long — a UI wedged
/// on a hung SSH pty must not leave four wheels driving.
pub const UI_WATCHDOG: Duration = Duration::from_secs(1);

/// Consecutive deadline overruns before the pilot declares itself unfit
/// to guarantee the 50 Hz floor and stops the vehicle.
const MAX_OVERRUNS: u64 = 5;

/// Poll one wheel every Nth cycle rather than every cycle. The four drive
/// frames are the hard floor of a cycle's bus time; the poll (which sleeps
/// out the whole reply window) is what pushes an occupied cycle to the edge
/// of the budget. Polling every 2nd cycle leaves the other cycles cheap, so
/// a scheduler jitter spike usually lands on a cycle with room to absorb it
/// and the `MAX_OVERRUNS` run never accumulates. At 2, each wheel is still
/// polled every 8 cycles (~144 ms at an 18 ms cycle); one miss (~288 ms)
/// stays under `stale_ms`, so "one dropped poll is expected" still holds.
const POLL_EVERY: u64 = 2;

/// A snapshot for the CSV logger, sent lossily (`try_send`) so a stalled
/// SD-card write can never block the drive loop.
pub struct LogRow {
    pub wheels: [WheelState; 4],
}

pub struct Pilot<W: WheelIo, C: Clock, S: FnMut()> {
    /// Grid order: FL, FR, RL, RR.
    wheels: [W; 4],
    sides: [Side; 4],
    labels: [String; 4],
    cfg: Config,
    clock: C,
    shared: Arc<Shared>,
    /// The group stop (hardware: `Bus::safe_stop_all`; sim: brake all).
    stop_all: S,
    log: Option<SyncSender<LogRow>>,
    /// `Some` only when a UI exists to feed the heartbeat.
    ui_watchdog: Option<Duration>,
    // Loop state lives on the struct so pauses/panics don't reset the
    // ramp or the round-robin phase (and tests can run the loop in
    // bounded slices).
    state: [WheelState; 4],
    /// Per-wheel host-side setpoint ramps. Stop and brake paths reset these
    /// to zero rather than stepping them — see the setpoint block in
    /// `one_cycle` and [`SlewLimiter::reset_to`].
    ramps: [SlewLimiter; 4],
    cycle_idx: u64,
    consecutive_overruns: u64,
    ui_starved: bool,
}

impl<W: WheelIo, C: Clock, S: FnMut()> Pilot<W, C, S> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        wheels: [W; 4],
        sides: [Side; 4],
        labels: [String; 4],
        cfg: Config,
        clock: C,
        shared: Arc<Shared>,
        stop_all: S,
        log: Option<SyncSender<LogRow>>,
        ui_watchdog: Option<Duration>,
    ) -> Self {
        // `Config::validate` guarantees a finite, positive rate, so the
        // fallback is only reachable for a Config that bypassed it — the same
        // reasoning as the timing accessors in `config.rs`. `GENTLE` errs
        // toward a rover that barely moves, never toward an unramped step.
        let ramp =
            SlewLimiter::new(cfg.limits.ramp_rpm_per_s as f32).unwrap_or(SlewLimiter::GENTLE);
        Self {
            wheels,
            sides,
            labels,
            cfg,
            clock,
            shared,
            stop_all,
            log,
            ui_watchdog,
            state: [WheelState::default(); 4],
            ramps: [ramp; 4],
            cycle_idx: 0,
            consecutive_overruns: 0,
            ui_starved: false,
        }
    }

    fn cycle_loop(&mut self) {
        let cycle = self.cfg.cycle();
        // Benefit of the doubt at start: a wheel that never answers trips
        // once it exceeds dead_ms from HERE, rather than on the first
        // cycle before it was ever polled. Truly absent wheels were
        // already refused by the startup gate.
        let start = self.clock.now();
        for w in self.state.iter_mut() {
            w.last_reply.get_or_insert(start);
        }
        let mut next = start + cycle;

        while self.shared.running.load(Ordering::Relaxed) {
            self.one_cycle();

            // Pacing + the overrun watchdog guarding the 2 ms of margin.
            self.cycle_idx += 1;
            let now = self.clock.now();
            if now > next {
                self.consecutive_overruns += 1;
                self.shared
                    .overruns
                    .store(self.consecutive_overruns, Ordering::Relaxed);
                if self.consecutive_overruns >= MAX_OVERRUNS {
                    self.trip("cycle overrun x5 — cannot guarantee the 50 Hz floor");
                }
                next = now + cycle; // re-anchor instead of bursting
            } else {
                self.consecutive_overruns = 0;
                self.shared.overruns.store(0, Ordering::Relaxed);
                self.clock.sleep_until(next);
                next += cycle;
            }
        }
    }

    fn one_cycle(&mut self) {
        let max_rpm = self.cfg.limits.max_rpm;
        let accel = self.cfg.limits.accel;
        // The ramp advances by the *nominal* cycle, not measured elapsed time:
        // a scheduling overrun must not license a bigger setpoint step, and
        // repeated overruns are already handled as a fault (MAX_OVERRUNS).
        let cycle = self.cfg.cycle();

        // -- round-robin poll, every POLL_EVERY-th cycle (module doc) -----
        // The polled wheel was also *driven* last cycle, and motors answer
        // drive frames too. Polling here, right after the inter-cycle sleep,
        // keeps that drive reply out of the poll window; polling after this
        // cycle's drives would put the last-driven wheel's reply inside its
        // own poll window (only the enforced min_gap apart), where a slow
        // adapter delivers it as garbage temperature/position. `send_recv`'s
        // input clear drains the drive replies that pile up between polls.
        // See POLL_EVERY for why the poll does not run every cycle.
        if self.cycle_idx.is_multiple_of(POLL_EVERY) {
            let k = ((self.cycle_idx / POLL_EVERY) % 4) as usize;
            match self.wheels[k].poll(self.cfg.reply_wait()) {
                Ok(Some(fb)) => {
                    self.state[k].telemetry.absorb(fb);
                    self.state[k].last_reply = Some(self.clock.now());
                    self.state[k].missed_polls = 0;
                    // Current-trip debounce bookkeeping.
                    if f64::from(fb.current_a.abs()) >= self.cfg.limits.current_trip_a {
                        let now = self.clock.now();
                        self.state[k].over_current_since.get_or_insert(now);
                    } else {
                        self.state[k].over_current_since = None;
                    }
                }
                // The debounce means "OBSERVED over the limit for the whole
                // window". A missed reply is not an observation: leaving the
                // timer armed would stretch one transient sample plus bus
                // noise into a latched trip.
                Ok(None) => {
                    self.state[k].over_current_since = None;
                    self.state[k].missed_polls += 1;
                }
                Err(e) => {
                    self.state[k].over_current_since = None;
                    self.state[k].missed_polls += 1;
                    self.shared
                        .set_msg(format!("bus error: {e} (still polling)"));
                }
            }
        }

        // -- operator intent (copy out; consume the one-shots) ------------
        let intent = {
            let mut i = lock(&self.shared.intent);
            let copy = *i;
            i.all_stop = false;
            i.rearm = false;
            copy
        };

        if intent.rearm && lock(&self.shared.trip).take().is_some() {
            // Re-arm never resumes the old speed: setpoints restart from
            // zero, like every drive re-enable convention.
            let mut i = lock(&self.shared.intent);
            i.throttle = 0.0;
            i.turn = 0.0;
            drop(i);
            self.shared
                .set_msg("re-armed — throttle zeroed, drive when ready");
        }
        let tripped = lock(&self.shared.trip).is_some();

        // -- UI heartbeat watchdog ----------------------------------------
        if let Some(window) = self.ui_watchdog {
            let stale = self.clock.now().duration_since(*lock(&self.shared.ui_tick));
            if stale > window {
                if !self.ui_starved {
                    self.ui_starved = true;
                    let mut i = lock(&self.shared.intent);
                    i.throttle = 0.0;
                    i.turn = 0.0;
                    drop(i);
                    self.shared.set_msg("UI stopped ticking — setpoints zeroed");
                }
            } else {
                self.ui_starved = false;
            }
        }

        // -- setpoints -----------------------------------------------------
        let halted = tripped || intent.all_stop || self.ui_starved;
        // While the brake is held the wheels are physically stopping, so
        // the ramp must not keep winding toward the latched throttle —
        // that would command the fully-ramped RPM in a single step on
        // release, bypassing the safety ramp exactly when four stopped
        // wheels would lurch off one supply. Brake-release ramps up from
        // zero, like any other start.
        let braking = intent.brake && !tripped;
        let cmd = if halted {
            DriveCmd::new(0.0, 0.0)
        } else {
            DriveCmd::new(intent.throttle, intent.turn)
        };
        for i in 0..4 {
            let shaped = if halted || braking {
                // Stop paths BYPASS the ramp. Every stack's failsafe
                // writes zero immediately; ours does too.
                self.ramps[i].reset_to(0.0);
                self.ramps[i].current_setpoint()
            } else {
                let target = f32::from(wheel_rpm(cmd, self.sides[i], max_rpm));
                self.ramps[i].step(target, cycle)
            };
            self.state[i].cmd_rpm = shaped.round() as i16;
        }
        if intent.all_stop {
            let mut i = lock(&self.shared.intent);
            i.throttle = 0.0;
            i.turn = 0.0;
            drop(i);
            self.shared.set_msg("ALL STOP");
        }

        // -- the four drive frames (never negotiable) ---------------------
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            let sent = if braking {
                wheel.brake()
            } else {
                wheel.drive(self.state[i].cmd_rpm, accel)
            };
            if let Err(e) = sent {
                // Transient bus errors must not kill the loop; the
                // protocol coasts the motor if we truly go quiet.
                self.shared
                    .set_msg(format!("bus error: {e} (still driving)"));
            }
        }

        // -- safety verdict -----------------------------------------------
        if !tripped {
            let now = self.clock.now();
            let mut verdict = Reaction::Continue;
            for (i, w) in self.state.iter().enumerate() {
                verdict = verdict.max(assess(w, &self.labels[i], &self.cfg, now));
            }
            match verdict {
                Reaction::Continue => {}
                Reaction::Warn(msg) => self.shared.set_msg(format!("[!] {msg}")),
                Reaction::FailStopNoBrake(reason) => self.trip_no_brake(&reason),
                Reaction::FailStopAll(reason) => self.trip(&reason),
            }
        }

        // -- publish + log -------------------------------------------------
        *lock(&self.shared.wheels) = self.state;
        if let Some(tx) = &self.log
            && tx.try_send(LogRow { wheels: self.state }).is_err()
        {
            self.shared.dropped_log_rows.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Latch, zero, group-stop. The pilot keeps cycling afterwards — a
    /// dashboard that vanishes at the moment of failure is the worst
    /// possible behavior; the operator needs to watch the temperature
    /// fall and decide when to re-arm.
    fn trip(&mut self, reason: &str) {
        if self.latch_and_zero(reason) {
            (self.stop_all)();
            self.shared
                .set_msg(format!("TRIPPED: {reason} — press R to re-arm"));
        }
    }

    /// The current-trip variant: latch and zero, but send NO brake frames
    /// — braking a jammed wheel draws more current. The stop is held by
    /// the zero-velocity stream the still-cycling pilot keeps sending.
    fn trip_no_brake(&mut self, reason: &str) {
        if self.latch_and_zero(reason) {
            self.shared.set_msg(format!(
                "TRIPPED: {reason} — held at zero, not braked; press R to re-arm"
            ));
        }
    }

    /// Returns false if a trip was already latched.
    fn latch_and_zero(&mut self, reason: &str) -> bool {
        if lock(&self.shared.trip).is_some() {
            return false;
        }
        *lock(&self.shared.trip) = Some(reason.to_owned());
        {
            let mut i = lock(&self.shared.intent);
            i.throttle = 0.0;
            i.turn = 0.0;
            i.brake = false;
        }
        // A latched trip is a stop path: bypass the ramp, don't decay through it.
        for r in self.ramps.iter_mut() {
            r.reset_to(0.0);
        }
        for w in self.state.iter_mut() {
            w.cmd_rpm = 0;
        }
        true
    }
}

/// Run a pilot with panic containment: a panic inside the loop clears
/// `running` FIRST (so a UI thread doesn't redraw over the unwind
/// message during the ~300 ms stop) and then group-stops.
///
/// Relies on unwinding — `panic = "abort"` must never be set for this
/// crate (see Cargo.toml).
pub fn run_guarded<W: WheelIo, C: Clock, S: FnMut()>(pilot: &mut Pilot<W, C, S>) {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| pilot.cycle_loop()));
    if result.is_err() {
        pilot.shared.running.store(false, Ordering::Relaxed);
        pilot
            .shared
            .set_msg("pilot thread panicked — stopping all wheels");
    }
    (pilot.stop_all)();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::TestClock;
    use m0601::Feedback;
    use m0601::protocol::{ReplyKind, crc8_maxim, parse_feedback};
    use std::sync::Mutex;
    use std::sync::atomic::AtomicU64;
    use std::time::Instant;

    /// Every drive a wheel received, stamped with sim time.
    type DriveLog = Arc<Mutex<Vec<(Instant, i16)>>>;

    /// Scripted wheel: records every drive with its sim-time stamp,
    /// replies healthily unless told otherwise. Wheel 0 also enforces the
    /// test's cycle budget by clearing `running` once enough cycles ran.
    struct TestWheel {
        id: u8,
        clock: TestClock,
        drives: DriveLog,
        brakes: Arc<Mutex<Vec<Instant>>>,
        budget: Option<(Arc<Shared>, Arc<AtomicU64>)>,
        silent: bool,
        fault_bits: u8,
        current_a: f32,
    }

    impl TestWheel {
        /// Frames of either kind — the budget must keep counting while
        /// the vehicle brakes, or a braking test would spin forever.
        fn spend_budget(&self) {
            if let Some((shared, budget)) = &self.budget {
                let sent = self.drives.lock().unwrap().len() + self.brakes.lock().unwrap().len();
                if sent as u64 >= budget.load(Ordering::Relaxed) {
                    shared.running.store(false, Ordering::Relaxed);
                }
            }
        }
    }

    impl TestWheel {
        fn reply(&self) -> Option<Feedback> {
            if self.silent {
                return None;
            }
            let raw_current = (self.current_a * 32767.0 / 8.0) as i16;
            let mut frame = [0u8; 10];
            frame[0] = self.id;
            frame[1] = 0x02;
            frame[2..4].copy_from_slice(&raw_current.to_be_bytes());
            frame[6] = 35;
            frame[8] = self.fault_bits;
            frame[9] = crc8_maxim(&frame[..9]);
            parse_feedback(&frame, ReplyKind::Query)
        }
    }

    impl WheelIo for TestWheel {
        fn drive(&mut self, rpm: i16, _accel: u8) -> m0601::Result<()> {
            self.drives.lock().unwrap().push((self.clock.now(), rpm));
            self.spend_budget();
            Ok(())
        }
        fn brake(&mut self) -> m0601::Result<()> {
            self.brakes.lock().unwrap().push(self.clock.now());
            self.spend_budget();
            Ok(())
        }
        fn poll(&mut self, _wait: Duration) -> m0601::Result<Option<Feedback>> {
            Ok(self.reply())
        }
    }

    struct Rig {
        shared: Arc<Shared>,
        drives: [DriveLog; 4],
        brakes: [Arc<Mutex<Vec<Instant>>>; 4],
        stops: Arc<Mutex<u32>>,
        budget: Arc<AtomicU64>,
    }

    impl Rig {
        /// Run the loop until wheel 0 has `total` frames (drive or brake)
        /// recorded.
        fn run_until<S: FnMut()>(&self, pilot: &mut Pilot<TestWheel, TestClock, S>, total: u64) {
            self.budget.store(total, Ordering::Relaxed);
            self.shared.running.store(true, Ordering::Relaxed);
            pilot.cycle_loop();
        }

        fn frames(&self, wheel: usize) -> Vec<(Instant, i16)> {
            self.drives[wheel].lock().unwrap().clone()
        }

        fn sent(&self, wheel: usize) -> u64 {
            (self.drives[wheel].lock().unwrap().len() + self.brakes[wheel].lock().unwrap().len())
                as u64
        }
    }

    fn rig(
        mutate: impl Fn(usize, &mut TestWheel),
    ) -> (Rig, Pilot<TestWheel, TestClock, impl FnMut()>) {
        let cfg = Config::parse(include_str!("../wheels.toml")).expect("shipped config");
        let clock = TestClock::new();
        let shared = Arc::new(Shared::new());
        let drives: [DriveLog; 4] = Default::default();
        let brakes: [Arc<Mutex<Vec<Instant>>>; 4] = Default::default();
        let budget = Arc::new(AtomicU64::new(u64::MAX));
        let ids = [0x03u8, 0x04, 0x01, 0x02];
        let mut wheels = Vec::new();
        for (i, id) in ids.into_iter().enumerate() {
            let mut w = TestWheel {
                id,
                clock: clock.clone(),
                drives: Arc::clone(&drives[i]),
                brakes: Arc::clone(&brakes[i]),
                budget: (i == 0).then(|| (Arc::clone(&shared), Arc::clone(&budget))),
                silent: false,
                fault_bits: 0,
                current_a: 0.3,
            };
            mutate(i, &mut w);
            wheels.push(w);
        }
        let wheels: [TestWheel; 4] = match wheels.try_into() {
            Ok(w) => w,
            Err(_) => unreachable!("built four wheels"),
        };
        let stops = Arc::new(Mutex::new(0u32));
        let stops2 = Arc::clone(&stops);
        let pilot = Pilot::new(
            wheels,
            [Side::Left, Side::Right, Side::Left, Side::Right],
            ["FL".into(), "FR".into(), "RL".into(), "RR".into()],
            cfg,
            clock,
            Arc::clone(&shared),
            move || *stops2.lock().unwrap() += 1,
            None,
            None,
        );
        (
            Rig {
                shared,
                drives,
                brakes,
                stops,
                budget,
            },
            pilot,
        )
    }

    #[test]
    fn the_cycle_honours_the_protocol_drive_rate_for_every_wheel() {
        // The quad analogue of the CLI's drive-rate test: across cycle
        // boundaries, no wheel's consecutive drive frames are ever
        // further apart than the 50 Hz floor allows.
        let (rig, mut pilot) = rig(|_, _| {});
        lock(&rig.shared.intent).throttle = 0.5;
        rig.run_until(&mut pilot, 12);
        let floor = m0601::drive_floor();
        for w in 0..4 {
            let log = rig.frames(w);
            assert!(log.len() >= 12, "wheel {w} driven every cycle");
            for pair in log.windows(2) {
                let gap = pair[1].0 - pair[0].0;
                assert!(gap <= floor, "wheel {w}: {gap:?} between drive frames");
            }
        }
    }

    #[test]
    fn setpoints_ramp_instead_of_stepping() {
        let (rig, mut pilot) = rig(|_, _| {});
        lock(&rig.shared.intent).throttle = 1.0;
        rig.run_until(&mut pilot, 3);
        let log = rig.frames(0);
        // 300 RPM/s at 18 ms cycles = 5.4 RPM/cycle: commanded values
        // must climb, not jump to 120.
        assert!(log[0].1 <= 6, "first cycle ramped: {}", log[0].1);
        assert!(
            log[1].1 > log[0].1 && log[1].1 <= 12,
            "second climbs: {}",
            log[1].1
        );
    }

    #[test]
    fn all_stop_is_not_rate_limited() {
        // The review's hard requirement: SPACE bypasses the ramp.
        let (rig, mut pilot) = rig(|_, _| {});
        lock(&rig.shared.intent).throttle = 1.0;
        rig.run_until(&mut pilot, 60); // long enough to ramp to 120
        let before = rig.frames(0);
        assert!(before.last().unwrap().1 > 100, "reached speed");
        lock(&rig.shared.intent).all_stop = true;
        rig.run_until(&mut pilot, before.len() as u64 + 1);
        let after = rig.frames(0);
        assert_eq!(
            after[before.len()].1,
            0,
            "the very next drive frame after ALL STOP is zero — no ramp-down"
        );
    }

    #[test]
    fn a_hard_fault_trips_stops_all_and_latches_until_rearm() {
        let (rig, mut pilot) = rig(|i, w| {
            if i == 2 {
                w.fault_bits = m0601::Faults::STALL;
            }
        });
        lock(&rig.shared.intent).throttle = 0.8;
        rig.run_until(&mut pilot, 16); // enough cycles to poll wheel 2
        assert!(lock(&rig.shared.trip).is_some(), "stall must latch a trip");
        assert!(*rig.stops.lock().unwrap() >= 1, "group stop ran");
        // While tripped: still cycling (frames continue) but always zero,
        // and operator throttle must not move the wheels.
        lock(&rig.shared.intent).throttle = 1.0;
        let n = rig.frames(0).len() as u64;
        rig.run_until(&mut pilot, n + 4);
        assert_eq!(
            rig.frames(0).last().unwrap().1,
            0,
            "latched: zero on the wire"
        );
        // Re-arm clears the latch; wheel 2 will re-trip on its next poll,
        // but the immediate cycles drive again from a zeroed throttle.
        lock(&rig.shared.intent).rearm = true;
        let n = rig.frames(0).len() as u64;
        rig.run_until(&mut pilot, n + 1);
        assert!(lock(&rig.shared.trip).is_none() || rig.frames(2).len() as u64 > n);
    }

    #[test]
    fn a_silent_wheel_is_a_telemetry_failure_not_a_control_failure() {
        // The systematic-dropout test: wheel 1 (FR) never answers a poll.
        // The vehicle keeps driving all four wheels through the stale
        // window and only fail-stops once the wheel is DEAD — proving
        // telemetry loss degrades monitoring, not control.
        let (rig, mut pilot) = rig(|i, w| {
            if i == 1 {
                w.silent = true;
            }
        });
        lock(&rig.shared.intent).throttle = 0.5;
        // dead_ms = 1500 at 18 ms cycles ≈ 84 cycles; run past it.
        rig.run_until(&mut pilot, 100);
        let reason = lock(&rig.shared.trip)
            .clone()
            .expect("dead wheel must stop the vehicle");
        assert!(reason.contains("FR"), "the dead wheel is named: {reason}");
        let counts: Vec<usize> = (0..4).map(|w| rig.frames(w).len()).collect();
        assert!(
            counts.iter().all(|&c| c >= 84),
            "all four driven throughout: {counts:?}"
        );
    }

    #[test]
    fn sustained_overcurrent_trips_after_debounce_but_inrush_does_not() {
        let (rig, mut pilot) = rig(|i, w| {
            if i == 0 {
                w.current_a = 3.0; // over the 2.5 A trip, permanently
            }
        });
        lock(&rig.shared.intent).throttle = 0.5;
        // Debounce is 400 ms ≈ 23 cycles; wheel 0 is polled every 8th.
        rig.run_until(&mut pilot, 12);
        assert!(lock(&rig.shared.trip).is_none(), "not before the window");
        rig.run_until(&mut pilot, 60);
        assert!(lock(&rig.shared.trip).is_some(), "after the window");
        // Policy: the current trip is held by zero-velocity frames, not
        // the braking group stop.
        assert_eq!(*rig.stops.lock().unwrap(), 0, "no brake on a current trip");
        assert_eq!(rig.frames(0).last().unwrap().1, 0, "held at zero");
    }

    #[test]
    fn a_wedged_ui_zeroes_the_setpoints() {
        let (rig, mut pilot) = rig(|_, _| {});
        pilot.ui_watchdog = Some(Duration::from_millis(200));
        // Stamp the heartbeat "now" (sim time base) and drive.
        *lock(&rig.shared.ui_tick) = pilot.clock.now();
        lock(&rig.shared.intent).throttle = 1.0;
        rig.run_until(&mut pilot, 5);
        assert!(
            rig.frames(0).last().unwrap().1 > 0,
            "driving while UI ticks... "
        );
        // Never stamp again: sim time runs 200 ms ahead in ~11 cycles.
        rig.run_until(&mut pilot, 30);
        assert_eq!(
            rig.frames(0).last().unwrap().1,
            0,
            "...zeroed once the heartbeat went stale"
        );
        assert!(
            lock(&rig.shared.trip).is_none(),
            "a starved UI is not a trip"
        );
    }

    #[test]
    fn brake_release_ramps_from_zero_not_from_the_latched_throttle() {
        let (rig, mut pilot) = rig(|_, _| {});
        lock(&rig.shared.intent).throttle = 1.0;
        rig.run_until(&mut pilot, 60); // ramp all the way up
        assert!(rig.frames(0).last().unwrap().1 > 100, "reached speed");
        // Hold the brake long enough that an integrator left running
        // would wind fully back up to the latched throttle.
        lock(&rig.shared.intent).brake = true;
        rig.run_until(&mut pilot, rig.sent(0) + 30);
        assert!(
            !rig.brakes[0].lock().unwrap().is_empty(),
            "brake frames went out"
        );
        // Release: the next drive frame must restart the ramp from zero,
        // not step to the fully-ramped RPM in one cycle.
        lock(&rig.shared.intent).brake = false;
        let drives_before = rig.frames(0).len();
        rig.run_until(&mut pilot, rig.sent(0) + 1);
        let first = rig.frames(0)[drives_before].1;
        assert!(
            first <= 6,
            "first drive after brake release ramps from zero: {first}"
        );
    }

    #[test]
    fn missed_polls_do_not_stretch_one_overcurrent_sample_into_a_trip() {
        let (rig, mut pilot) = rig(|i, w| {
            if i == 0 {
                w.current_a = 3.0; // one over-limit sample on the first poll…
            }
        });
        lock(&rig.shared.intent).throttle = 0.5;
        rig.run_until(&mut pilot, 4); // wheel 0 polled once: debounce armed
        pilot.wheels[0].silent = true; // …then every reply goes missing
        // Run far past the 400 ms debounce but short of the 1500 ms dead
        // window: missed replies are not observations, so the single
        // sample must not latch a sustained-overcurrent trip.
        rig.run_until(&mut pilot, 60);
        assert!(
            lock(&rig.shared.trip).is_none(),
            "one sample plus silence tripped: {:?}",
            lock(&rig.shared.trip)
        );
    }
}
