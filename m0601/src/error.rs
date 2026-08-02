//! Error and result types for the driver.

use std::io;

/// Errors returned by the [`M0601`](crate::M0601) driver and its transports.
///
/// Note that a *silent bus* is **not** an error: a motor that does not reply
/// (wrong ID, unpowered, mid-scan probe) surfaces as `Ok(None)` from query
/// methods. `Err` always means the port or OS failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// The serial port could not be opened or configured.
    #[error("serial port {port}: {source}")]
    Serial {
        /// The port that failed (e.g. `/dev/ttyUSB0`).
        port: String,
        /// The underlying `serialport` error.
        #[source]
        source: serialport::Error,
    },

    /// An I/O error while talking to an already-open port.
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),

    /// A motor ID outside the valid range `0x01..=0xFE`.
    ///
    /// `0x00` and `0xFF` are reserved by the protocol. `0xC8` is *accepted*
    /// but inadvisable — see
    /// [`validate_id`](crate::protocol::validate_id).
    #[error("invalid motor ID 0x{0:02X} (must be 0x01..=0xFE)")]
    InvalidId(u8),

    /// A raw frame with the wrong length was supplied to
    /// [`frame_from_bytes`](crate::protocol::frame_from_bytes).
    ///
    /// Raw frames must be 9 bytes (a CRC-8/MAXIM is appended) or a full
    /// 10 bytes.
    #[error("invalid frame length {0} (need 9 or 10 bytes)")]
    InvalidFrameLen(usize),
}

impl Error {
    /// `true` when the underlying cause is a permission error on the port —
    /// on Linux this usually means the user is not in the `dialout` group.
    pub fn is_permission_denied(&self) -> bool {
        match self {
            Error::Serial { source, .. } => matches!(
                source.kind,
                serialport::ErrorKind::Io(io::ErrorKind::PermissionDenied)
            ),
            Error::Io(e) => e.kind() == io::ErrorKind::PermissionDenied,
            _ => false,
        }
    }
}

/// Convenience alias for `Result<T, m0601::Error>`.
pub type Result<T> = std::result::Result<T, Error>;
