//! Public configuration types for the BMP388 driver.
//!
//! This module contains the core types used to configure and interact with
//! the BMP388 barometric pressure sensor:
//!
//! - [`Address`] — I2C address selection based on the SDO pin
//! - [`Oversampling`] — Oversampling rate for pressure and temperature
//! - [`PowerMode`] — Sensor operating mode (sleep, forced, normal)
//! - [`IirFilter`] — IIR filter coefficient for output smoothing
//! - [`OutputDataRate`] — Measurement output rate in normal mode
//! - [`SensorConfig`] — Combined sensor configuration
//! - [`Status`] — Sensor status flags
//! - [`PowerControl`] — Power and sensor enable settings
//!
//! # Address Configuration
//!
//! The BMP388 I2C address is determined by the SDO pin:
//!
//! ```
//! use bmp388_embedded::Address;
//!
//! // SDO connected to GND
//! let addr = Address::Primary;   // 0x76
//!
//! // SDO connected to VDDIO
//! let addr = Address::Secondary; // 0x77
//! ```
//!
//! # Oversampling
//!
//! Higher oversampling rates improve measurement accuracy at the cost of
//! increased measurement time and power consumption:
//!
//! ```
//! use bmp388_embedded::Oversampling;
//!
//! let os = Oversampling::X1;  // No oversampling (fastest)
//! let os = Oversampling::X32; // 32x oversampling (most accurate)
//! ```
//!
//! # IIR Filter
//!
//! The IIR filter smooths output data to reduce short-term noise:
//!
//! ```
//! use bmp388_embedded::IirFilter;
//!
//! let filter = IirFilter::Off;       // No filtering
//! let filter = IirFilter::Coeff127;  // Maximum smoothing
//! ```

/// I2C address selection for the BMP388.
///
/// The address is determined by the state of the SDO pin on the sensor.
/// The SDO pin **must not** be left floating.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Address {
    /// SDO pin connected to GND: `0x76`.
    Primary,
    /// SDO pin connected to VDDIO: `0x77`.
    Secondary,
    /// A custom 7-bit I2C address.
    Custom(u8),
}

impl Address {
    /// Resolve into a 7-bit I2C address.
    #[must_use]
    pub const fn addr(self) -> u8 {
        match self {
            Self::Primary => 0x76,
            Self::Secondary => 0x77,
            Self::Custom(a) => a,
        }
    }
}

/// Oversampling rate for pressure or temperature measurements.
///
/// Higher oversampling rates improve measurement precision at the cost
/// of increased measurement time. The sensor supports up to 32x
/// oversampling for both pressure and temperature independently.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Oversampling {
    /// No oversampling (1x).
    X1 = 0x00,
    /// 2x oversampling.
    X2 = 0x01,
    /// 4x oversampling.
    X4 = 0x02,
    /// 8x oversampling.
    X8 = 0x03,
    /// 16x oversampling.
    X16 = 0x04,
    /// 32x oversampling.
    X32 = 0x05,
}

/// Sensor power mode.
///
/// The BMP388 supports three operating modes:
///
/// - **Sleep**: No measurements are performed. Lowest power consumption.
/// - **Forced**: A single measurement is performed, then the sensor
///   returns to sleep mode automatically.
/// - **Normal**: Measurements are performed continuously at the
///   configured output data rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    /// No measurements; lowest power; registers remain accessible.
    Sleep = 0x00,
    /// Single measurement, then automatic return to sleep.
    Forced = 0x01,
    /// Continuous measurements at the configured ODR.
    Normal = 0x03,
}

/// IIR filter coefficient.
///
/// The infinite impulse response (IIR) filter suppresses short-term
/// disturbances in the output data (e.g., caused by door slamming or
/// wind). Higher coefficients provide more smoothing but slower
/// response to actual pressure changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IirFilter {
    /// Filter disabled (bypass).
    Off = 0x00,
    /// Coefficient 1.
    Coeff1 = 0x01,
    /// Coefficient 3.
    Coeff3 = 0x02,
    /// Coefficient 7.
    Coeff7 = 0x03,
    /// Coefficient 15.
    Coeff15 = 0x04,
    /// Coefficient 31.
    Coeff31 = 0x05,
    /// Coefficient 63.
    Coeff63 = 0x06,
    /// Coefficient 127.
    Coeff127 = 0x07,
}

/// Output data rate selection for normal mode.
///
/// Controls how frequently the sensor performs measurements in normal
/// mode. Lower rates reduce power consumption.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OutputDataRate {
    /// 200 Hz (5 ms period).
    Hz200 = 0x00,
    /// 100 Hz (10 ms period).
    Hz100 = 0x01,
    /// 50 Hz (20 ms period).
    Hz50 = 0x02,
    /// 25 Hz (40 ms period).
    Hz25 = 0x03,
    /// 12.5 Hz (80 ms period).
    Hz12p5 = 0x04,
    /// 6.25 Hz (160 ms period).
    Hz6p25 = 0x05,
    /// 3.1 Hz (320 ms period).
    Hz3p1 = 0x06,
    /// 1.5 Hz (640 ms period).
    Hz1p5 = 0x07,
    /// 0.78 Hz (1.28 s period).
    Hz0p78 = 0x08,
    /// 0.39 Hz (2.56 s period).
    Hz0p39 = 0x09,
    /// 0.2 Hz (5.12 s period).
    Hz0p2 = 0x0A,
    /// 0.1 Hz (10.24 s period).
    Hz0p1 = 0x0B,
    /// 0.05 Hz (20.48 s period).
    Hz0p05 = 0x0C,
    /// 0.02 Hz (40.96 s period).
    Hz0p02 = 0x0D,
    /// 0.01 Hz (81.92 s period).
    Hz0p01 = 0x0E,
    /// 0.006 Hz (163.84 s period).
    Hz0p006 = 0x0F,
    /// 0.003 Hz (327.68 s period).
    Hz0p003 = 0x10,
    /// 0.001 Hz (655.36 s period).
    Hz0p001 = 0x11,
}

/// Combined sensor configuration.
///
/// Groups the oversampling, IIR filter, and output data rate settings
/// into a single structure for convenient configuration.
///
/// # Example
///
/// ```
/// use bmp388_embedded::{IirFilter, OutputDataRate, Oversampling, SensorConfig};
///
/// let config = SensorConfig {
///     pressure_oversampling: Oversampling::X8,
///     temperature_oversampling: Oversampling::X2,
///     iir_filter: IirFilter::Coeff3,
///     output_data_rate: OutputDataRate::Hz50,
/// };
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorConfig {
    /// Pressure oversampling rate.
    pub pressure_oversampling: Oversampling,
    /// Temperature oversampling rate.
    pub temperature_oversampling: Oversampling,
    /// IIR filter coefficient.
    pub iir_filter: IirFilter,
    /// Output data rate (used in normal mode only).
    pub output_data_rate: OutputDataRate,
}

impl Default for SensorConfig {
    /// Returns the power-on-reset default configuration:
    /// no oversampling, filter off, 200 Hz ODR.
    fn default() -> Self {
        Self {
            pressure_oversampling: Oversampling::X1,
            temperature_oversampling: Oversampling::X1,
            iir_filter: IirFilter::Off,
            output_data_rate: OutputDataRate::Hz200,
        }
    }
}

/// Sensor status flags.
///
/// Indicates whether data is ready and whether the command decoder
/// is ready to accept a new command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// The command decoder is ready to accept a new command.
    pub command_ready: bool,
    /// New pressure data is available.
    pub pressure_data_ready: bool,
    /// New temperature data is available.
    pub temperature_data_ready: bool,
}

/// Power control settings.
///
/// Controls which sensors are enabled and the operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PowerControl {
    /// Enable the pressure sensor.
    pub pressure_enable: bool,
    /// Enable the temperature sensor.
    pub temperature_enable: bool,
    /// Operating mode.
    pub mode: PowerMode,
}

impl Default for PowerControl {
    /// Returns the power-on-reset default: both sensors disabled, sleep mode.
    fn default() -> Self {
        Self {
            pressure_enable: false,
            temperature_enable: false,
            mode: PowerMode::Sleep,
        }
    }
}

/// Compensated sensor measurement containing temperature and pressure.
///
/// Temperature is in degrees Celsius, pressure is in Pascals.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measurement {
    /// Temperature in degrees Celsius.
    pub temperature: f64,
    /// Pressure in Pascals.
    pub pressure: f64,
}
