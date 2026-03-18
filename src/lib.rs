//! no_std driver for the BMP388 / BMP390 barometric pressure and temperature sensor.
//!
//! `bmp388-embedded` is a small, `no_std` driver built on top of the
//! [`embedded-hal`](https://crates.io/crates/embedded-hal) traits.
//!
//! This crate is a clean reimplementation based on the existing Rust crate:
//! - `bmp280-ehal` by Roma Sokolov & Alexander Zhuravlev:
//!   <https://github.com/copterust/bmp280>
//!
//! The compensation formulas are derived from the official Bosch BMP3 Sensor
//! API: <https://github.com/boschsensortec/BMP3_SensorAPI>
//!
//! ## What this crate provides
//! - A blocking driver: [`Bmp388`]
//! - Configuration types for oversampling, IIR filter, ODR, and power mode
//! - Compensated temperature (°C) and pressure (Pa) readings
//! - Optional async API behind the `async` feature: [`r#async::Bmp388Async`]
//!
//! ## Blocking usage
//!
//! ```
//! use bmp388_embedded::{Address, Bmp388, Error, Oversampling, PowerMode};
//!
//! fn example<I2C, D, E>(i2c: I2C, delay: D) -> Result<(), Error<E>>
//! where
//!     I2C: embedded_hal::i2c::I2c<Error = E>,
//!     D: embedded_hal::delay::DelayNs,
//!     E: embedded_hal::i2c::Error,
//! {
//!     let mut sensor = Bmp388::new(i2c, delay, Address::Primary)?;
//!
//!     // Configure oversampling for high accuracy.
//!     sensor.set_oversampling(Oversampling::X8, Oversampling::X2)?;
//!
//!     // One-shot measurement (forced mode, waits for data automatically):
//!     let data = sensor.forced_measurement()?;
//!     let _temperature_c = data.temperature;
//!     let _pressure_pa = data.pressure;
//!     Ok(())
//! }
//! ```
//!
//! ## Normal (continuous) mode
//!
//! ```
//! use bmp388_embedded::{Address, Bmp388, Error, PowerMode};
//!
//! fn example<I2C, D, E>(i2c: I2C, delay: D) -> Result<(), Error<E>>
//! where
//!     I2C: embedded_hal::i2c::I2c<Error = E>,
//!     D: embedded_hal::delay::DelayNs,
//!     E: embedded_hal::i2c::Error,
//! {
//!     let mut sensor = Bmp388::new(i2c, delay, Address::Primary)?;
//!
//!     // Enable both sensors and start continuous measurements.
//!     sensor.set_power_control(true, true, PowerMode::Normal)?;
//!
//!     // Read the latest measurement.
//!     let data = sensor.sensor_data()?;
//!     Ok(())
//! }
//! ```
//!
//! ## Async usage
//!
//! Enable the `async` feature to use the async driver API in [`r#async`].
//!
//! ```
//! # fn main() {}
//! #
//! # #[cfg(feature = "async")]
//! # mod async_example {
//! use bmp388_embedded::{Address, Error, Oversampling};
//! use bmp388_embedded::r#async::Bmp388Async;
//!
//! pub async fn example<I2C, D, E>(i2c: I2C, delay: D) -> Result<(), Error<E>>
//! where
//!     I2C: embedded_hal_async::i2c::I2c<Error = E>,
//!     D: embedded_hal_async::delay::DelayNs,
//!     E: embedded_hal::i2c::Error,
//! {
//!     let mut sensor = Bmp388Async::new(i2c, delay, Address::Primary).await?;
//!     let data = sensor.forced_measurement().await?;
//!     let _temperature_c = data.temperature;
//!     let _pressure_pa = data.pressure;
//!     Ok(())
//! }
//! # }
//! ```

#![no_std]
#![deny(unsafe_code)]
#![deny(missing_docs)]

#[cfg(test)]
extern crate std;

mod calibration;
mod error;
mod registers;
mod types;

mod bmp388;
pub use bmp388::Bmp388;

pub use error::Error;
pub use types::{
    Address, IirFilter, Measurement, OutputDataRate, Oversampling, PowerControl, PowerMode,
    SensorConfig, Status,
};

#[cfg(feature = "async")]
pub mod r#async;
