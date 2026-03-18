//! Calibration data and compensation formulas for the BMP388.
//!
//! The BMP388 stores factory-programmed calibration coefficients in
//! non-volatile memory (NVM). These coefficients are read during
//! initialization and used to convert raw ADC readings into calibrated
//! temperature (°C) and pressure (Pa) values.
//!
//! The compensation formulas in this module are derived from the official
//! Bosch BMP3-SensorAPI (bmp3.c, v2.0.6) using the floating-point method.

use crate::registers::CALIB_DATA_LEN;

/// Raw calibration coefficients as read from NVM registers 0x31–0x45.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RawCalibration {
    pub par_t1: u16,
    pub par_t2: u16,
    pub par_t3: i8,
    pub par_p1: i16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i8,
    pub par_p5: u16,
    pub par_p6: u16,
    pub par_p7: i8,
    pub par_p8: i8,
    pub par_p9: i16,
    pub par_p10: i8,
    pub par_p11: i8,
}

impl RawCalibration {
    /// Parse raw calibration data from a 21-byte buffer read from NVM.
    pub(crate) fn from_bytes(data: &[u8; CALIB_DATA_LEN]) -> Self {
        Self {
            par_t1: u16::from_le_bytes([data[0], data[1]]),
            par_t2: u16::from_le_bytes([data[2], data[3]]),
            par_t3: data[4] as i8,
            par_p1: i16::from_le_bytes([data[5], data[6]]),
            par_p2: i16::from_le_bytes([data[7], data[8]]),
            par_p3: data[9] as i8,
            par_p4: data[10] as i8,
            par_p5: u16::from_le_bytes([data[11], data[12]]),
            par_p6: u16::from_le_bytes([data[13], data[14]]),
            par_p7: data[15] as i8,
            par_p8: data[16] as i8,
            par_p9: i16::from_le_bytes([data[17], data[18]]),
            par_p10: data[19] as i8,
            par_p11: data[20] as i8,
        }
    }
}

/// Quantized (scaled) calibration coefficients ready for compensation.
///
/// These are the NVM coefficients scaled by the divisors specified in
/// the Bosch BMP3 sensor API. The quantization converts the raw integer
/// coefficients into floating-point values suitable for the compensation
/// formulas.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CalibrationCoefficients {
    pub par_t1: f64,
    pub par_t2: f64,
    pub par_t3: f64,
    pub par_p1: f64,
    pub par_p2: f64,
    pub par_p3: f64,
    pub par_p4: f64,
    pub par_p5: f64,
    pub par_p6: f64,
    pub par_p7: f64,
    pub par_p8: f64,
    pub par_p9: f64,
    pub par_p10: f64,
    pub par_p11: f64,
}

impl CalibrationCoefficients {
    /// Create a zeroed set of coefficients (used as a placeholder before
    /// the real calibration data is read from the sensor).
    pub(crate) const fn zeroed() -> Self {
        Self {
            par_t1: 0.0,
            par_t2: 0.0,
            par_t3: 0.0,
            par_p1: 0.0,
            par_p2: 0.0,
            par_p3: 0.0,
            par_p4: 0.0,
            par_p5: 0.0,
            par_p6: 0.0,
            par_p7: 0.0,
            par_p8: 0.0,
            par_p9: 0.0,
            par_p10: 0.0,
            par_p11: 0.0,
        }
    }
}

impl From<RawCalibration> for CalibrationCoefficients {
    /// Quantize raw NVM coefficients into floating-point values.
    ///
    /// Scaling factors are taken directly from the Bosch BMP3-SensorAPI.
    fn from(raw: RawCalibration) -> Self {
        Self {
            // Temperature coefficients
            par_t1: f64::from(raw.par_t1) * 256.0,
            par_t2: f64::from(raw.par_t2) / 1_073_741_824.0,
            par_t3: f64::from(raw.par_t3) / 281_474_976_710_656.0,

            // Pressure coefficients
            par_p1: (f64::from(raw.par_p1) - 16_384.0) / 1_048_576.0,
            par_p2: (f64::from(raw.par_p2) - 16_384.0) / 536_870_912.0,
            par_p3: f64::from(raw.par_p3) / 4_294_967_296.0,
            par_p4: f64::from(raw.par_p4) / 137_438_953_472.0,
            par_p5: f64::from(raw.par_p5) * 8.0,
            par_p6: f64::from(raw.par_p6) / 64.0,
            par_p7: f64::from(raw.par_p7) / 256.0,
            par_p8: f64::from(raw.par_p8) / 32_768.0,
            par_p9: f64::from(raw.par_p9) / 281_474_976_710_656.0,
            par_p10: f64::from(raw.par_p10) / 281_474_976_710_656.0,
            par_p11: f64::from(raw.par_p11) / 36_893_488_147_419_103_232.0,
        }
    }
}

impl CalibrationCoefficients {
    /// Compensate a raw temperature reading.
    ///
    /// Returns `(temperature_celsius, t_lin)` where `t_lin` is the
    /// linearized temperature value needed for pressure compensation.
    ///
    /// The output is clamped to the sensor's operating range: −40 °C to 85 °C.
    pub(crate) fn compensate_temperature(&self, uncomp_temp: u32) -> (f64, f64) {
        let uncomp = uncomp_temp as f64;

        let partial_data1 = uncomp - self.par_t1;
        let partial_data2 = partial_data1 * self.par_t2;
        let t_lin = partial_data2 + (partial_data1 * partial_data1) * self.par_t3;

        // Clamp to valid operating range.
        let temperature = t_lin.clamp(-40.0, 85.0);

        (temperature, t_lin)
    }

    /// Compensate a raw pressure reading using the linearized temperature.
    ///
    /// `t_lin` must be obtained from [`compensate_temperature`].
    ///
    /// Returns pressure in Pascals, clamped to 30000–125000 Pa.
    pub(crate) fn compensate_pressure(&self, uncomp_press: u32, t_lin: f64) -> f64 {
        let uncomp = uncomp_press as f64;
        let t2 = t_lin * t_lin;
        let t3 = t2 * t_lin;

        // Offset calculation.
        let partial_out1 = self.par_p5 + self.par_p6 * t_lin + self.par_p7 * t2 + self.par_p8 * t3;

        // Sensitivity calculation.
        let partial_out2 =
            uncomp * (self.par_p1 + self.par_p2 * t_lin + self.par_p3 * t2 + self.par_p4 * t3);

        // Second-order correction.
        let uncomp2 = uncomp * uncomp;
        let partial_data = self.par_p9 + self.par_p10 * t_lin;
        let partial_out3 = uncomp2 * partial_data + uncomp2 * uncomp * self.par_p11;

        let pressure = partial_out1 + partial_out2 + partial_out3;

        // Clamp to valid operating range.
        pressure.clamp(30_000.0, 125_000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test calibration parsing with known byte patterns.
    #[test]
    fn parse_calibration_bytes() {
        // All zeros should produce a valid (if meaningless) calibration.
        let data = [0u8; CALIB_DATA_LEN];
        let raw = RawCalibration::from_bytes(&data);
        assert_eq!(raw.par_t1, 0);
        assert_eq!(raw.par_t2, 0);
        assert_eq!(raw.par_t3, 0);
    }

    /// Test that known raw values produce expected quantized coefficients.
    #[test]
    fn quantize_calibration() {
        let raw = RawCalibration {
            par_t1: 256,
            par_t2: 0,
            par_t3: 0,
            par_p1: 0,
            par_p2: 0,
            par_p3: 0,
            par_p4: 0,
            par_p5: 0,
            par_p6: 0,
            par_p7: 0,
            par_p8: 0,
            par_p9: 0,
            par_p10: 0,
            par_p11: 0,
        };
        let coeff = CalibrationCoefficients::from(raw);
        // par_t1 = 256 * 256.0 = 65536.0
        assert!((coeff.par_t1 - 65_536.0).abs() < f64::EPSILON);
    }

    /// Temperature compensation clamps to operating range.
    #[test]
    fn temperature_clamps_to_range() {
        let raw = RawCalibration {
            par_t1: 0,
            par_t2: 0,
            par_t3: 0,
            par_p1: 0,
            par_p2: 0,
            par_p3: 0,
            par_p4: 0,
            par_p5: 0,
            par_p6: 0,
            par_p7: 0,
            par_p8: 0,
            par_p9: 0,
            par_p10: 0,
            par_p11: 0,
        };
        let coeff = CalibrationCoefficients::from(raw);

        // With all-zero coefficients, t_lin = 0, which is within range.
        let (temp, _) = coeff.compensate_temperature(0);
        assert!((-40.0..=85.0).contains(&temp));
    }

    /// Pressure compensation clamps to operating range.
    #[test]
    fn pressure_clamps_to_range() {
        let raw = RawCalibration {
            par_t1: 0,
            par_t2: 0,
            par_t3: 0,
            par_p1: 0,
            par_p2: 0,
            par_p3: 0,
            par_p4: 0,
            par_p5: 0,
            par_p6: 0,
            par_p7: 0,
            par_p8: 0,
            par_p9: 0,
            par_p10: 0,
            par_p11: 0,
        };
        let coeff = CalibrationCoefficients::from(raw);

        let pressure = coeff.compensate_pressure(0, 0.0);
        assert!((30_000.0..=125_000.0).contains(&pressure));
    }
}
