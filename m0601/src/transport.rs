//! Byte transport: the seam between the driver and the wire.
//!
//! [`SerialTransport`] talks to a real RS485 adapter; [`MockTransport`]
//! scripts a bus in memory so every driver behavior is testable without
//! hardware (deterministically — no timing races, and it can simulate the
//! interesting cases a loopback can't: silence, partial replies, TX echo).

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::Duration;

use serialport::{ClearBuffer, SerialPort};

use crate::error::{Error, Result};
use crate::protocol::BAUD;

/// A half-duplex frame transport.
pub trait Transport {
    /// Write all bytes; no reply is expected or read.
    fn send(&mut self, data: &[u8]) -> Result<()>;

    /// Flush the input buffer, write all bytes, wait `wait`, then return
    /// everything that arrived in the meantime (possibly nothing).
    ///
    /// An empty return is *not* an error — RS485 silence is an expected
    /// outcome (wrong ID, unpowered motor, scan probe).
    fn send_recv(&mut self, data: &[u8], wait: Duration) -> Result<Vec<u8>>;

    /// How long the driver should actually pause when the protocol wants a
    /// gap *between* frames. Real transports return `d`; mocks return zero
    /// so tests run instantly.
    ///
    /// The driver calls this under the bus lock but performs the sleep
    /// *outside* it, so one motor's mode-switch or safe-stop sequence does
    /// not stall another motor's 50 Hz drive loop on a shared bus.
    ///
    /// This applies to inter-frame gaps only. A single
    /// [`send_recv`](Self::send_recv) transaction necessarily holds the bus
    /// for its whole `wait` — on half-duplex RS485 another motor's frame
    /// must not be interleaved into it — so keep `wait` short on a shared
    /// bus, and see [`Bus::scan`](crate::Bus::scan) for the one operation
    /// that deliberately holds it for a long time.
    fn pace(&self, d: Duration) -> Duration {
        d
    }
}

/// [`Transport`] over a real serial port: 115200 8N1, half-duplex RS485.
pub struct SerialTransport {
    name: String,
    port: Box<dyn SerialPort>,
    low_latency: bool,
}

impl SerialTransport {
    /// Open `path` (e.g. `/dev/ttyUSB0`) at [`BAUD`] 8N1.
    ///
    /// `timeout` is a backstop for individual OS reads; per-transaction
    /// latency is governed by the `wait` passed to
    /// [`send_recv`](Transport::send_recv), not by this value.
    ///
    /// On Linux this also asks the kernel for low-latency delivery
    /// (`ASYNC_LOW_LATENCY` — what pyserial calls `set_low_latency_mode`),
    /// best-effort: FTDI adapters otherwise hold received bytes for up to
    /// their 16 ms latency timer, longer than this protocol's entire reply
    /// window. Failure is not an error; check
    /// [`low_latency`](Self::low_latency) when reply timing matters.
    pub fn open(path: &str, timeout: Duration) -> Result<Self> {
        let builder = serialport::new(path, BAUD)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(timeout);
        let serial_err = |source| Error::Serial {
            port: path.to_owned(),
            source,
        };

        #[cfg(target_os = "linux")]
        let (port, low_latency): (Box<dyn SerialPort>, bool) = {
            use std::os::fd::AsRawFd;
            let native = builder.open_native().map_err(serial_err)?;
            let low_latency = crate::low_latency::enable(native.as_raw_fd()).is_ok();
            (Box::new(native), low_latency)
        };
        #[cfg(not(target_os = "linux"))]
        let (port, low_latency): (Box<dyn SerialPort>, bool) =
            (builder.open().map_err(serial_err)?, false);

        Ok(Self {
            name: path.to_owned(),
            port,
            low_latency,
        })
    }

    /// The port path this transport was opened on.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Whether the kernel accepted the low-latency request made by
    /// [`open`](Self::open).
    ///
    /// `false` means the flag could not be set (non-Linux, or a driver
    /// without the ioctl) — not necessarily that latency is high; only USB
    /// bridge chips batch received bytes this way. If it matters and this
    /// is `false`, set the FTDI timer via udev instead:
    ///
    /// ```text
    /// # /etc/udev/rules.d/99-m0601.rules
    /// ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
    /// ```
    ///
    /// and verify with
    /// `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.
    pub fn low_latency(&self) -> bool {
        self.low_latency
    }

    fn serial_err(&self, source: serialport::Error) -> Error {
        Error::Serial {
            port: self.name.clone(),
            source,
        }
    }
}

// Manual impl: the boxed port is not Debug.
impl std::fmt::Debug for SerialTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialTransport")
            .field("name", &self.name)
            .field("low_latency", &self.low_latency)
            .finish_non_exhaustive()
    }
}

impl Transport for SerialTransport {
    fn send(&mut self, data: &[u8]) -> Result<()> {
        self.port.write_all(data)?;
        self.port.flush()?;
        Ok(())
    }

    /// Mirrors pyserial's `write(); sleep(wait); read_all()` pattern.
    ///
    /// Crucially this never issues a blocking read for data that hasn't
    /// arrived: after the wait it asks the OS how many bytes are buffered
    /// and reads exactly that many. A blocking read would silently add the
    /// port timeout to every transaction — fatal inside the 50 Hz drive
    /// loop. `TimedOut` from the OS is therefore treated as "no more data",
    /// never as a failure.
    fn send_recv(&mut self, data: &[u8], wait: Duration) -> Result<Vec<u8>> {
        self.port
            .clear(ClearBuffer::Input)
            .map_err(|e| self.serial_err(e))?;
        self.port.write_all(data)?;
        self.port.flush()?;
        std::thread::sleep(wait);

        let n = self.port.bytes_to_read().map_err(|e| self.serial_err(e))? as usize;
        let mut buf = vec![0u8; n];
        let mut filled = 0;
        while filled < n {
            match self.port.read(&mut buf[filled..]) {
                Ok(0) => break,
                Ok(k) => filled += k,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) if e.kind() == io::ErrorKind::TimedOut => break,
                Err(e) => return Err(e.into()),
            }
        }
        buf.truncate(filled);
        Ok(buf)
    }
}

/// In-memory [`Transport`] for tests and examples.
///
/// Every outgoing frame is recorded in [`sent`](Self::sent); replies are
/// scripted through [`replies`](Self::replies) (an empty `Vec` simulates a
/// silent bus, and a missing entry does too). Set
/// [`echo_tx`](Self::echo_tx) to simulate a half-duplex adapter that echoes
/// its own transmission back, [`echo_truncate`](Self::echo_truncate) to
/// simulate one whose echo only partly made it into the read buffer, or
/// [`fail_io`](Self::fail_io) to make every operation return an I/O error
/// (frames are still recorded first, so tests can assert what a best-effort
/// path attempted).
///
/// `sleep` is a no-op — mock tests run instantly.
#[derive(Debug, Default)]
pub struct MockTransport {
    /// Every frame sent, in order, via either `send` or `send_recv`.
    pub sent: Vec<Vec<u8>>,
    /// Scripted replies, consumed front-to-back by `send_recv`.
    pub replies: VecDeque<Vec<u8>>,
    /// Prepend each sent frame to its reply (half-duplex echo).
    pub echo_tx: bool,
    /// With [`echo_tx`](Self::echo_tx), keep only this many bytes of the
    /// echo — a partial echo is the case the driver's all-or-nothing
    /// `strip_prefix` cannot recognise, and it must not be mistaken for
    /// telemetry. `None` echoes the whole frame.
    pub echo_truncate: Option<usize>,
    /// Make every operation fail with a broken-pipe I/O error.
    pub fail_io: bool,
}

impl MockTransport {
    /// A mock that answers each `send_recv` with the given replies in order.
    pub fn with_replies<I>(replies: I) -> Self
    where
        I: IntoIterator<Item = Vec<u8>>,
    {
        Self {
            replies: replies.into_iter().collect(),
            ..Self::default()
        }
    }

    fn check_fail(&self) -> Result<()> {
        if self.fail_io {
            Err(Error::Io(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "mock transport failure",
            )))
        } else {
            Ok(())
        }
    }
}

impl Transport for MockTransport {
    fn send(&mut self, data: &[u8]) -> Result<()> {
        self.sent.push(data.to_vec());
        self.check_fail()
    }

    fn send_recv(&mut self, data: &[u8], _wait: Duration) -> Result<Vec<u8>> {
        self.sent.push(data.to_vec());
        self.check_fail()?;
        let reply = self.replies.pop_front().unwrap_or_default();
        if self.echo_tx {
            let keep = self.echo_truncate.unwrap_or(data.len()).min(data.len());
            let mut echoed = data[..keep].to_vec();
            echoed.extend_from_slice(&reply);
            Ok(echoed)
        } else {
            Ok(reply)
        }
    }

    fn pace(&self, _d: Duration) -> Duration {
        Duration::ZERO
    }
}
