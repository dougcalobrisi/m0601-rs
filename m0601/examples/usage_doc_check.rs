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
