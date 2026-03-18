//! BMP388 register addresses and bit-field constants.
//!
//! This module contains the hardware register map for the BMP388 sensor
//! as defined in the official datasheet (BST-BMP388-DS001). All register
//! addresses, bit masks, and command values are defined here.

/// BMP388 register addresses.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
#[allow(dead_code)]
pub(crate) enum Register {
    /// Chip identification register. Returns [`CHIP_ID`] for BMP388.
    ChipId = 0x00,
    /// Error register. Indicates fatal, command, or configuration errors.
    ErrReg = 0x02,
    /// Sensor status register.
    Status = 0x03,
    /// Start of 6-byte pressure + temperature data block.
    Data = 0x04,
    /// Event register (power-on-reset detection).
    Event = 0x10,
    /// Interrupt status register.
    IntStatus = 0x11,
    /// Interrupt control register.
    IntCtrl = 0x19,
    /// Interface configuration register.
    IfConf = 0x1A,
    /// Power control register (sensor enable + mode).
    PwrCtrl = 0x1B,
    /// Oversampling settings register.
    Osr = 0x1C,
    /// Output data rate register.
    Odr = 0x1D,
    /// IIR filter configuration register.
    Config = 0x1F,
    /// Start of 21-byte calibration data block.
    CalibData = 0x31,
    /// Command register.
    Cmd = 0x7E,
}

impl Register {
    /// Return the register address as a `u8`.
    pub(crate) const fn addr(self) -> u8 {
        self as u8
    }
}

// --- Chip identification ---

/// Expected chip ID for the BMP388.
pub(crate) const CHIP_ID_BMP388: u8 = 0x50;

/// Expected chip ID for the BMP390 (also compatible).
pub(crate) const CHIP_ID_BMP390: u8 = 0x60;

// --- Commands ---

/// Soft reset command value (written to [`Register::Cmd`]).
pub(crate) const SOFT_RESET_CMD: u8 = 0xB6;

// --- Data lengths ---

/// Number of calibration data bytes to read (registers 0x31–0x45).
pub(crate) const CALIB_DATA_LEN: usize = 21;

/// Number of data bytes for a combined pressure + temperature read.
pub(crate) const SENSOR_DATA_LEN: usize = 6;

// --- PWR_CTRL register (0x1B) bit masks ---

/// Bit 0: pressure sensor enable.
pub(crate) const PWR_CTRL_PRESS_EN: u8 = 0x01;

/// Bit 1: temperature sensor enable.
pub(crate) const PWR_CTRL_TEMP_EN: u8 = 0x02;

/// Bits [5:4]: power mode mask.
pub(crate) const PWR_CTRL_MODE_MASK: u8 = 0x30;

/// Bit position for the power mode field.
pub(crate) const PWR_CTRL_MODE_POS: u8 = 4;

// --- OSR register (0x1C) bit masks ---

/// Bits [2:0]: pressure oversampling mask.
pub(crate) const OSR_PRESS_MASK: u8 = 0x07;

/// Bits [5:3]: temperature oversampling mask.
pub(crate) const OSR_TEMP_MASK: u8 = 0x38;

/// Bit position for the temperature oversampling field.
pub(crate) const OSR_TEMP_POS: u8 = 3;

// --- CONFIG register (0x1F) bit masks ---

/// Bits [3:1]: IIR filter coefficient mask.
pub(crate) const CONFIG_FILTER_MASK: u8 = 0x0E;

/// Bit position for the IIR filter coefficient field.
pub(crate) const CONFIG_FILTER_POS: u8 = 1;

// --- ODR register (0x1D) bit masks ---

/// Bits [4:0]: output data rate selection mask.
pub(crate) const ODR_SEL_MASK: u8 = 0x1F;

// --- STATUS register (0x03) bit masks ---

/// Bit 4: command decoder ready.
pub(crate) const STATUS_CMD_RDY: u8 = 0x10;

/// Bit 5: pressure data ready.
pub(crate) const STATUS_DRDY_PRESS: u8 = 0x20;

/// Bit 6: temperature data ready.
pub(crate) const STATUS_DRDY_TEMP: u8 = 0x40;

// --- ERR_REG register (0x02) bit masks ---

/// Bit 0: fatal error.
pub(crate) const ERR_FATAL: u8 = 0x01;

/// Bit 1: command error.
#[allow(dead_code)]
pub(crate) const ERR_CMD: u8 = 0x02;

/// Bit 2: configuration error.
pub(crate) const ERR_CONF: u8 = 0x04;
