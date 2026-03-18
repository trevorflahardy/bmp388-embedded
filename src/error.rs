//! Error types for the BMP388 driver.
//!
//! This module defines the [`Error`] type that can occur when interacting
//! with the BMP388 sensor. The error type is generic over the underlying
//! I2C error type, allowing it to work with any `embedded-hal` I2C
//! implementation.
//!
//! # Example: Error Handling
//!
//! ```
//! use bmp388_embedded::{Address, Bmp388, Error};
//!
//! # fn example<I2C, D, E>(i2c: I2C, delay: D) -> Result<(), Error<E>>
//! # where
//! #     I2C: embedded_hal::i2c::I2c<Error = E>,
//! #     D: embedded_hal::delay::DelayNs,
//! #     E: embedded_hal::i2c::Error,
//! # {
//! let mut sensor = Bmp388::new(i2c, delay, Address::Primary)?;
//!
//! match sensor.sensor_data() {
//!     Ok(measurement) => {
//!         let _ = measurement.temperature;
//!         let _ = measurement.pressure;
//!     }
//!     Err(Error::I2c(_e)) => {
//!         // Handle I2C communication error
//!     }
//!     Err(Error::InvalidChipId(id)) => {
//!         // Unexpected chip at this address
//!         let _ = id;
//!     }
//!     Err(Error::FatalError) => {
//!         // Sensor reported a fatal error
//!     }
//!     Err(Error::ConfigError) => {
//!         // Sensor reported a configuration error
//!     }
//! }
//! # Ok(())
//! # }
//! ```

use embedded_hal::i2c as ehal;

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// Underlying I2C bus error.
    I2c(E),
    /// The chip ID register returned an unexpected value.
    ///
    /// Contains the actual chip ID read from the sensor.
    InvalidChipId(u8),
    /// The sensor reported a fatal error in the ERR_REG register.
    FatalError,
    /// The sensor reported a configuration error in the ERR_REG register.
    ConfigError,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::I2c(e)
    }
}

impl<E> ehal::Error for Error<E>
where
    E: ehal::Error,
{
    fn kind(&self) -> ehal::ErrorKind {
        match self {
            Self::I2c(e) => e.kind(),
            Self::InvalidChipId(_) | Self::FatalError | Self::ConfigError => ehal::ErrorKind::Other,
        }
    }
}
