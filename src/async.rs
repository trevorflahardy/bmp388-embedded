//! Async driver implementation for the BMP388 barometric pressure sensor.
//!
//! This module provides the [`Bmp388Async`] driver for asynchronous I2C
//! communication using the `embedded-hal-async` traits. This is useful
//! for non-blocking applications in async embedded environments.
//!
//! This module requires the `async` feature to be enabled:
//!
//! ```toml
//! [dependencies]
//! bmp388-embedded = { version = "0.1", features = ["async"] }
//! ```
//!
//! The async driver provides the same API as the blocking driver, but all
//! I2C operations and delays are asynchronous.
//!
//! # Example: One-Shot Measurement
//!
//! ```
//! # #[cfg(feature = "async")]
//! # async fn example_async() {
//! use bmp388_embedded::{Address, Oversampling};
//! use bmp388_embedded::r#async::Bmp388Async;
//!
//! # async fn inner<I2C, D, E>(i2c: I2C, delay: D) -> Result<(), bmp388_embedded::Error<E>>
//! # where
//! #     I2C: embedded_hal_async::i2c::I2c<Error = E>,
//! #     D: embedded_hal_async::delay::DelayNs,
//! #     E: embedded_hal::i2c::Error,
//! # {
//! let mut sensor = Bmp388Async::new(i2c, delay, Address::Primary).await?;
//!
//! let data = sensor.forced_measurement().await?;
//! let temperature_c = data.temperature;
//! let pressure_pa = data.pressure;
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! # Example: Normal (Continuous) Mode
//!
//! ```
//! # #[cfg(feature = "async")]
//! # async fn example_async() {
//! use bmp388_embedded::{Address, PowerMode};
//! use bmp388_embedded::r#async::Bmp388Async;
//!
//! # async fn inner<I2C, D, E>(i2c: I2C, delay: D) -> Result<(), bmp388_embedded::Error<E>>
//! # where
//! #     I2C: embedded_hal_async::i2c::I2c<Error = E>,
//! #     D: embedded_hal_async::delay::DelayNs,
//! #     E: embedded_hal::i2c::Error,
//! # {
//! let mut sensor = Bmp388Async::new(i2c, delay, Address::Primary).await?;
//!
//! sensor.set_power_control(true, true, PowerMode::Normal).await?;
//!
//! let data = sensor.sensor_data().await?;
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::calibration::{CalibrationCoefficients, RawCalibration};
use crate::registers::*;
use crate::types::*;
use crate::Error;

/// Async BMP388 barometric pressure sensor driver.
#[derive(Debug)]
pub struct Bmp388Async<I2C, D> {
    i2c: I2C,
    delay: D,
    address: u8,
    calibration: CalibrationCoefficients,
    t_lin: f64,
}

impl<I2C, D> Bmp388Async<I2C, D> {
    /// Destroy the driver and return the underlying I2C bus and delay.
    #[must_use]
    pub fn destroy(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    /// Get the configured I2C address.
    #[must_use]
    pub const fn address(&self) -> u8 {
        self.address
    }
}

impl<I2C, D, E> Bmp388Async<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    /// Create a new async BMP388 driver instance.
    ///
    /// This constructor:
    /// 1. Performs a soft reset to ensure a known state
    /// 2. Validates the chip ID (accepts BMP388 `0x50` and BMP390 `0x60`)
    /// 3. Reads the factory calibration coefficients from NVM
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidChipId`] if the chip ID is not recognized,
    /// or [`Error::I2c`] if communication fails.
    pub async fn new(i2c: I2C, delay: D, address: Address) -> Result<Self, Error<E>> {
        let mut sensor = Self {
            i2c,
            delay,
            address: address.addr(),
            calibration: CalibrationCoefficients::zeroed(),
            t_lin: 0.0,
        };

        sensor.soft_reset().await?;
        sensor.validate_chip_id().await?;
        sensor.calibration = sensor.read_calibration().await?;

        Ok(sensor)
    }

    // --- Public measurement methods ---

    /// Read the latest compensated sensor data (temperature and pressure).
    pub async fn sensor_data(&mut self) -> Result<Measurement, Error<E>> {
        let (raw_press, raw_temp) = self.read_raw_sensor_data().await?;

        let (temperature, t_lin) = self.calibration.compensate_temperature(raw_temp);
        self.t_lin = t_lin;
        let pressure = self.calibration.compensate_pressure(raw_press, t_lin);

        Ok(Measurement {
            temperature,
            pressure,
        })
    }

    /// Perform a forced (one-shot) measurement and return the result.
    pub async fn forced_measurement(&mut self) -> Result<Measurement, Error<E>> {
        self.set_power_control(true, true, PowerMode::Forced)
            .await?;
        self.wait_for_data().await?;
        self.sensor_data().await
    }

    /// Read only the compensated temperature in degrees Celsius.
    pub async fn temperature(&mut self) -> Result<f64, Error<E>> {
        Ok(self.sensor_data().await?.temperature)
    }

    /// Read only the compensated pressure in Pascals.
    pub async fn pressure(&mut self) -> Result<f64, Error<E>> {
        Ok(self.sensor_data().await?.pressure)
    }

    // --- Configuration methods ---

    /// Set the oversampling rates for pressure and temperature.
    pub async fn set_oversampling(
        &mut self,
        pressure: Oversampling,
        temperature: Oversampling,
    ) -> Result<(), Error<E>> {
        let value = (pressure as u8) | ((temperature as u8) << OSR_TEMP_POS);
        self.write_register(Register::Osr, value).await
    }

    /// Read the current oversampling settings.
    pub async fn oversampling(&mut self) -> Result<(Oversampling, Oversampling), Error<E>> {
        let osr = self.read_register(Register::Osr).await?;
        let press = parse_oversampling(osr & OSR_PRESS_MASK);
        let temp = parse_oversampling((osr & OSR_TEMP_MASK) >> OSR_TEMP_POS);
        Ok((press, temp))
    }

    /// Set the IIR filter coefficient.
    pub async fn set_iir_filter(&mut self, filter: IirFilter) -> Result<(), Error<E>> {
        let value = (filter as u8) << CONFIG_FILTER_POS;
        self.write_register(Register::Config, value).await
    }

    /// Read the current IIR filter setting.
    pub async fn iir_filter(&mut self) -> Result<IirFilter, Error<E>> {
        let config = self.read_register(Register::Config).await?;
        let raw = (config & CONFIG_FILTER_MASK) >> CONFIG_FILTER_POS;
        Ok(parse_iir_filter(raw))
    }

    /// Set the output data rate for normal mode.
    pub async fn set_output_data_rate(&mut self, odr: OutputDataRate) -> Result<(), Error<E>> {
        self.write_register(Register::Odr, odr as u8).await
    }

    /// Read the current output data rate setting.
    pub async fn output_data_rate(&mut self) -> Result<OutputDataRate, Error<E>> {
        let raw = self.read_register(Register::Odr).await? & ODR_SEL_MASK;
        Ok(parse_output_data_rate(raw))
    }

    /// Apply a complete sensor configuration.
    pub async fn set_sensor_config(&mut self, config: SensorConfig) -> Result<(), Error<E>> {
        self.set_oversampling(
            config.pressure_oversampling,
            config.temperature_oversampling,
        )
        .await?;
        self.set_iir_filter(config.iir_filter).await?;
        self.set_output_data_rate(config.output_data_rate).await
    }

    /// Set the power control register (sensor enables and mode).
    pub async fn set_power_control(
        &mut self,
        pressure_enable: bool,
        temperature_enable: bool,
        mode: PowerMode,
    ) -> Result<(), Error<E>> {
        let value = ((pressure_enable as u8) * PWR_CTRL_PRESS_EN)
            | ((temperature_enable as u8) * PWR_CTRL_TEMP_EN)
            | ((mode as u8) << PWR_CTRL_MODE_POS);
        self.write_register(Register::PwrCtrl, value).await
    }

    /// Read the current power control settings.
    pub async fn power_control(&mut self) -> Result<PowerControl, Error<E>> {
        let reg = self.read_register(Register::PwrCtrl).await?;
        Ok(PowerControl {
            pressure_enable: (reg & PWR_CTRL_PRESS_EN) != 0,
            temperature_enable: (reg & PWR_CTRL_TEMP_EN) != 0,
            mode: parse_power_mode((reg & PWR_CTRL_MODE_MASK) >> PWR_CTRL_MODE_POS),
        })
    }

    /// Read the sensor status register.
    pub async fn status(&mut self) -> Result<Status, Error<E>> {
        let reg = self.read_register(Register::Status).await?;
        Ok(Status {
            command_ready: (reg & STATUS_CMD_RDY) != 0,
            pressure_data_ready: (reg & STATUS_DRDY_PRESS) != 0,
            temperature_data_ready: (reg & STATUS_DRDY_TEMP) != 0,
        })
    }

    /// Read the chip ID register.
    pub async fn chip_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::ChipId).await
    }

    /// Perform a software reset.
    pub async fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write_register(Register::Cmd, SOFT_RESET_CMD).await?;
        self.delay.delay_ms(2).await;
        Ok(())
    }

    /// Check the error register for any reported errors.
    pub async fn check_errors(&mut self) -> Result<(), Error<E>> {
        let err = self.read_register(Register::ErrReg).await?;
        if (err & ERR_FATAL) != 0 {
            return Err(Error::FatalError);
        }
        if (err & ERR_CONF) != 0 {
            return Err(Error::ConfigError);
        }
        Ok(())
    }

    // --- Private helpers ---

    /// Validate that the chip ID matches a known BMP388/BMP390 value.
    async fn validate_chip_id(&mut self) -> Result<(), Error<E>> {
        let id = self.read_register(Register::ChipId).await?;
        if id != CHIP_ID_BMP388 && id != CHIP_ID_BMP390 {
            return Err(Error::InvalidChipId(id));
        }
        Ok(())
    }

    /// Read factory calibration coefficients from NVM.
    async fn read_calibration(&mut self) -> Result<CalibrationCoefficients, Error<E>> {
        let mut data = [0u8; CALIB_DATA_LEN];
        self.i2c
            .write_read(self.address, &[Register::CalibData.addr()], &mut data)
            .await?;
        let raw = RawCalibration::from_bytes(&data);
        Ok(CalibrationCoefficients::from(raw))
    }

    /// Read raw 24-bit pressure and temperature values as a burst read.
    async fn read_raw_sensor_data(&mut self) -> Result<(u32, u32), Error<E>> {
        let mut data = [0u8; SENSOR_DATA_LEN];
        self.i2c
            .write_read(self.address, &[Register::Data.addr()], &mut data)
            .await?;

        let raw_press = (data[2] as u32) << 16 | (data[1] as u32) << 8 | (data[0] as u32);
        let raw_temp = (data[5] as u32) << 16 | (data[4] as u32) << 8 | (data[3] as u32);

        Ok((raw_press, raw_temp))
    }

    /// Poll the status register until data is ready, with a timeout.
    async fn wait_for_data(&mut self) -> Result<(), Error<E>> {
        for _ in 0..100 {
            let status = self.status().await?;
            if status.pressure_data_ready && status.temperature_data_ready {
                return Ok(());
            }
            self.delay.delay_ms(2).await;
        }
        Ok(())
    }

    /// Write a single byte to a register.
    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<E>> {
        self.i2c.write(self.address, &[reg.addr(), value]).await?;
        Ok(())
    }

    /// Read a single byte from a register.
    async fn read_register(&mut self, reg: Register) -> Result<u8, Error<E>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[reg.addr()], &mut buf)
            .await?;
        Ok(buf[0])
    }
}

// --- Parsing helpers (shared with blocking driver) ---

/// Parse a 3-bit oversampling value into the enum.
fn parse_oversampling(raw: u8) -> Oversampling {
    match raw {
        0x00 => Oversampling::X1,
        0x01 => Oversampling::X2,
        0x02 => Oversampling::X4,
        0x03 => Oversampling::X8,
        0x04 => Oversampling::X16,
        _ => Oversampling::X32,
    }
}

/// Parse a 3-bit IIR filter value into the enum.
fn parse_iir_filter(raw: u8) -> IirFilter {
    match raw {
        0x00 => IirFilter::Off,
        0x01 => IirFilter::Coeff1,
        0x02 => IirFilter::Coeff3,
        0x03 => IirFilter::Coeff7,
        0x04 => IirFilter::Coeff15,
        0x05 => IirFilter::Coeff31,
        0x06 => IirFilter::Coeff63,
        _ => IirFilter::Coeff127,
    }
}

/// Parse a 2-bit power mode value into the enum.
fn parse_power_mode(raw: u8) -> PowerMode {
    match raw {
        0x00 => PowerMode::Sleep,
        0x01 | 0x02 => PowerMode::Forced,
        _ => PowerMode::Normal,
    }
}

/// Parse a 5-bit ODR value into the enum.
fn parse_output_data_rate(raw: u8) -> OutputDataRate {
    match raw {
        0x00 => OutputDataRate::Hz200,
        0x01 => OutputDataRate::Hz100,
        0x02 => OutputDataRate::Hz50,
        0x03 => OutputDataRate::Hz25,
        0x04 => OutputDataRate::Hz12p5,
        0x05 => OutputDataRate::Hz6p25,
        0x06 => OutputDataRate::Hz3p1,
        0x07 => OutputDataRate::Hz1p5,
        0x08 => OutputDataRate::Hz0p78,
        0x09 => OutputDataRate::Hz0p39,
        0x0A => OutputDataRate::Hz0p2,
        0x0B => OutputDataRate::Hz0p1,
        0x0C => OutputDataRate::Hz0p05,
        0x0D => OutputDataRate::Hz0p02,
        0x0E => OutputDataRate::Hz0p01,
        0x0F => OutputDataRate::Hz0p006,
        0x10 => OutputDataRate::Hz0p003,
        _ => OutputDataRate::Hz0p001,
    }
}
