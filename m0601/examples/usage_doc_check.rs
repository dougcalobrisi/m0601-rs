//! Compile check for the code examples in USAGE.md — not meant to be run.
#![allow(dead_code, clippy::unwrap_used)]

use std::time::{Duration, Instant};

use m0601::protocol::frame_velocity;
use m0601::{Bus, M0601, MockTransport, Mode};

fn query_example() -> m0601::Result<()> {
    let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;
    match motor.query()? {
        Some(fb) => println!(
            "{:+} RPM, {:.1}°, {:?} °C, faults: {}",
            fb.speed_rpm, fb.position_deg, fb.temp_c, fb.faults
        ),
        None => println!("no reply — check power, wiring, --id"),
    }
    Ok(())
}

fn drive_example() -> m0601::Result<()> {
    let mut motor = M0601::open("/dev/ttyUSB0", 0x01, Duration::from_millis(150))?;
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        motor.drive_velocity(100)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    motor.safe_stop();
    Ok(())
}

fn transact_example(motor: &mut M0601) -> m0601::Result<()> {
    let frame = frame_velocity(motor.id(), 100, 1);
    if let Some(fb) = motor.transact(&frame, Duration::from_millis(6))? {
        assert!(fb.temp_c.is_none());
        println!("{:+} RPM at {:.2}°", fb.speed_rpm, fb.position_deg);
    }
    Ok(())
}

fn modes_example(motor: &mut M0601) -> m0601::Result<()> {
    motor.set_mode(Mode::Current)?;
    motor.drive_current(4096)?;
    motor.set_mode(Mode::Position)?;
    motor.drive_position(16384)?;
    Ok(())
}

fn bus_example() -> m0601::Result<()> {
    let bus = Bus::open("/dev/ttyUSB0", Duration::from_millis(150))?;
    let mut left = bus.motor(0x01)?.mirrored(true);
    let mut right = bus.motor(0x02)?;
    left.drive_velocity(100)?;
    right.drive_velocity(100)?;
    Ok(())
}

fn multi_motor_example() -> m0601::Result<()> {
    let ids = [0x01, 0x02, 0x03, 0x04];
    let bus = Bus::open("/dev/ttyUSB0", Duration::from_millis(150))?
        .with_min_gap(Duration::from_micros(2500));
    assert!(bus.min_gap() > Duration::ZERO);
    bus.set_mode_all(&ids, Mode::Velocity)?;
    let mut wheels = Vec::new();
    for id in ids {
        wheels.push(bus.motor(id)?);
    }
    for wheel in &mut wheels {
        wheel.drive_velocity(60)?; // the bus spaces these on the wire
    }
    let guard = bus.clone(); // e.g. for a stop guard / signal handler
    guard.safe_stop_all(&ids);
    Ok(())
}

fn timing_example() -> m0601::Result<()> {
    // Tunable bus timing (idle gap, stop ramp, mode/set-ID/broadcast waits)
    // and drive-accel defaults — set from your own config, defaults unchanged.
    let bus = Bus::open("/dev/ttyUSB0", Duration::from_millis(150))?
        .with_timing(m0601::BusTiming {
            stop_accel: 5,
            ..m0601::BusTiming::default()
        })
        .with_default_accel(10); // every motor's drive_velocity default
    let mut motor = bus.motor(0x01)?.with_default_accel(20); // just this one
    motor.drive_velocity(200)?; // uses accel 20
    Ok(())
}

fn low_latency_example() -> m0601::Result<()> {
    let transport = m0601::SerialTransport::open("/dev/ttyUSB0", Duration::from_millis(150))?;
    if !transport.low_latency() {
        eprintln!("[!] low-latency not set; see the udev rule in USAGE.md");
    }
    Ok(())
}

fn mock_example() -> m0601::Result<()> {
    let mock = MockTransport::with_replies([vec![
        0x01, 0x02, 0x00, 0x00, 0x00, 0x64, 0x28, 0x00, 0x00, 0x00,
    ]]);
    let mut motor = M0601::with_transport(mock, 0x01, Duration::from_millis(150))?;
    let fb = motor.query()?.unwrap();
    assert_eq!(fb.speed_rpm, 100);
    let mock = motor.into_transport().unwrap();
    assert_eq!(mock.sent.len(), 1);
    Ok(())
}

fn main() {}
