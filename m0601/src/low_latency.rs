//! Linux-only: ask the kernel for low-latency delivery on the serial port.
//!
//! FTDI adapters (the usual RS485 dongle) buffer received bytes until a USB
//! packet fills or their *latency timer* fires — 16 ms from the factory.
//! A 10-byte reply therefore sits in the adapter for up to 16 ms before the
//! host sees it, which dwarfs this protocol's whole reply budget and makes
//! short reply windows read nothing at all.
//!
//! Setting `ASYNC_LOW_LATENCY` on the tty (the `TIOCGSERIAL`/`TIOCSSERIAL`
//! ioctl pair) tells the `ftdi_sio` driver to program the timer down to
//! 1 ms. This is what `setserial /dev/ttyUSB0 low_latency` and pyserial's
//! `set_low_latency_mode(True)` do; since kernel 4.12 it needs no special
//! privileges beyond being able to open the port. The struct layout is the
//! kernel's stable UAPI (`include/uapi/linux/serial.h`).

// The only unsafe in the crate: two ioctls on a file descriptor we own,
// operating on a locally defined UAPI struct. Kept behind this scoped allow
// so the crate-level `deny(unsafe_code)` still covers everything else.
#![allow(unsafe_code)]

use std::io;
use std::mem::MaybeUninit;
use std::os::fd::RawFd;

/// `struct serial_struct` from `include/uapi/linux/serial.h`. Only `flags`
/// is read or written; the rest must be round-tripped untouched.
#[repr(C)]
#[allow(dead_code)]
struct SerialStruct {
    type_: libc::c_int,
    line: libc::c_int,
    port: libc::c_uint,
    irq: libc::c_int,
    flags: libc::c_int,
    xmit_fifo_size: libc::c_int,
    custom_divisor: libc::c_int,
    baud_base: libc::c_int,
    close_delay: libc::c_ushort,
    io_type: libc::c_char,
    reserved_char: [libc::c_char; 1],
    hub6: libc::c_int,
    closing_wait: libc::c_ushort,
    closing_wait2: libc::c_ushort,
    iomem_base: *mut libc::c_uchar,
    iomem_reg_shift: libc::c_ushort,
    port_high: libc::c_uint,
    iomap_base: libc::c_ulong,
}

/// `ASYNC_LOW_LATENCY` from `include/uapi/linux/tty_flags.h` (bit 13).
const ASYNC_LOW_LATENCY: libc::c_int = 1 << 13;

/// Set `ASYNC_LOW_LATENCY` on `fd`. Best-effort by design: drivers that
/// don't support the ioctl (or don't need it — only USB bridge chips
/// batch like this) return an error, which callers treat as "flag not
/// set", not as a broken port.
pub(crate) fn enable(fd: RawFd) -> io::Result<()> {
    let mut info = MaybeUninit::<SerialStruct>::uninit();
    // SAFETY: `fd` is a valid open tty descriptor for the lifetime of this
    // call (the caller borrows the open port), and the pointer handed to
    // TIOCGSERIAL is to a properly sized `SerialStruct` the kernel fills.
    if unsafe { libc::ioctl(fd, libc::TIOCGSERIAL as _, info.as_mut_ptr()) } != 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: a successful TIOCGSERIAL initialized the struct.
    let mut info = unsafe { info.assume_init() };
    if info.flags & ASYNC_LOW_LATENCY != 0 {
        return Ok(());
    }
    info.flags |= ASYNC_LOW_LATENCY;
    // SAFETY: same fd validity as above; TIOCSSERIAL only reads the struct.
    if unsafe { libc::ioctl(fd, libc::TIOCSSERIAL as _, &info) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}
