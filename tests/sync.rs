use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

use bmp388_embedded::{Address, Bmp388, Error, IirFilter, Oversampling, PowerMode};

#[derive(Debug, Default)]
struct NoDelay;

impl embedded_hal::delay::DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
    fn delay_us(&mut self, _us: u32) {}
    fn delay_ms(&mut self, _ms: u32) {}
}

/// BMP388 chip ID.
const CHIP_ID: u8 = 0x50;

/// Build the standard initialization transaction sequence:
/// 1. soft_reset: write [0x7E, 0xB6]
/// 2. read chip_id: write_read [0x00] -> [CHIP_ID]
/// 3. read calibration: write_read [0x31] -> [21 bytes]
fn init_transactions(calib: &[u8; 21]) -> Vec<I2cTransaction> {
    vec![
        // soft_reset
        I2cTransaction::write(0x76, vec![0x7E, 0xB6]),
        // read chip_id
        I2cTransaction::write_read(0x76, vec![0x00], vec![CHIP_ID]),
        // read calibration data
        I2cTransaction::write_read(0x76, vec![0x31], calib.to_vec()),
    ]
}

/// Default calibration data (all zeros — produces clamped but valid output).
fn default_calib() -> [u8; 21] {
    [0u8; 21]
}

#[test]
fn new_reads_chip_id_and_calibration() {
    let calib = default_calib();
    let expectations = init_transactions(&calib);

    let i2c = I2cMock::new(&expectations);
    let sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn new_rejects_wrong_chip_id() {
    let expectations = [
        // soft_reset
        I2cTransaction::write(0x76, vec![0x7E, 0xB6]),
        // read chip_id with wrong value
        I2cTransaction::write_read(0x76, vec![0x00], vec![0xFF]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let i2c_clone = i2c.clone();
    let result = Bmp388::new(i2c_clone, NoDelay, Address::Primary);

    match result {
        Err(Error::InvalidChipId(0xFF)) => {} // expected
        other => panic!("Expected InvalidChipId(0xFF), got {:?}", other),
    }

    i2c.done();
}

#[test]
fn set_oversampling_writes_correct_register() {
    let calib = default_calib();
    let mut expectations = init_transactions(&calib);
    // set_oversampling: OSR register = 0x1C
    // pressure X8 = 0x03, temperature X2 = 0x01 << 3 = 0x08
    // combined: 0x03 | 0x08 = 0x0B
    expectations.push(I2cTransaction::write(0x76, vec![0x1C, 0x0B]));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    sensor
        .set_oversampling(Oversampling::X8, Oversampling::X2)
        .unwrap();

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn set_iir_filter_writes_correct_register() {
    let calib = default_calib();
    let mut expectations = init_transactions(&calib);
    // set_iir_filter: CONFIG register = 0x1F
    // Coeff3 = 0x02, shifted left by 1 = 0x04
    expectations.push(I2cTransaction::write(0x76, vec![0x1F, 0x04]));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    sensor.set_iir_filter(IirFilter::Coeff3).unwrap();

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn set_power_control_writes_correct_register() {
    let calib = default_calib();
    let mut expectations = init_transactions(&calib);
    // set_power_control: PWR_CTRL register = 0x1B
    // press_en=1 (0x01), temp_en=1 (0x02), Normal=0x03 << 4 = 0x30
    // combined: 0x01 | 0x02 | 0x30 = 0x33
    expectations.push(I2cTransaction::write(0x76, vec![0x1B, 0x33]));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    sensor
        .set_power_control(true, true, PowerMode::Normal)
        .unwrap();

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn sensor_data_reads_6_bytes_and_returns_measurement() {
    let calib = default_calib();
    let mut expectations = init_transactions(&calib);
    // sensor_data: read 6 bytes from DATA register (0x04)
    // Provide some non-zero raw data.
    expectations.push(I2cTransaction::write_read(
        0x76,
        vec![0x04],
        vec![0x00, 0x00, 0x80, 0x00, 0x00, 0x80],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    let data = sensor.sensor_data().unwrap();

    // With all-zero calibration, values will be clamped.
    assert!(data.temperature >= -40.0 && data.temperature <= 85.0);
    assert!(data.pressure >= 30_000.0 && data.pressure <= 125_000.0);

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn forced_measurement_triggers_forced_mode_and_polls_status() {
    let calib = default_calib();
    let mut expectations = init_transactions(&calib);

    // forced_measurement:
    // 1. set_power_control (forced mode): write [0x1B, 0x13]
    //    press_en=1, temp_en=1, Forced=0x01 << 4 = 0x10 => 0x01|0x02|0x10 = 0x13
    expectations.push(I2cTransaction::write(0x76, vec![0x1B, 0x13]));
    // 2. poll status: read [0x03] -> both data ready (0x60)
    expectations.push(I2cTransaction::write_read(0x76, vec![0x03], vec![0x60]));
    // 3. read sensor data
    expectations.push(I2cTransaction::write_read(
        0x76,
        vec![0x04],
        vec![0x00, 0x00, 0x80, 0x00, 0x00, 0x80],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    let data = sensor.forced_measurement().unwrap();
    assert!(data.temperature >= -40.0);
    assert!(data.pressure >= 30_000.0);

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn soft_reset_writes_reset_command() {
    let calib = default_calib();
    let mut expectations = init_transactions(&calib);
    // An additional soft_reset after init.
    expectations.push(I2cTransaction::write(0x76, vec![0x7E, 0xB6]));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    sensor.soft_reset().unwrap();

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn status_parses_register_bits() {
    let calib = default_calib();
    let mut expectations = init_transactions(&calib);
    // Status register: cmd_rdy=1 (0x10), drdy_press=1 (0x20), drdy_temp=0
    expectations.push(I2cTransaction::write_read(0x76, vec![0x03], vec![0x30]));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Bmp388::new(i2c, NoDelay, Address::Primary).unwrap();

    let status = sensor.status().unwrap();
    assert!(status.command_ready);
    assert!(status.pressure_data_ready);
    assert!(!status.temperature_data_ready);

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}

#[test]
fn secondary_address_uses_0x77() {
    let expectations = [
        I2cTransaction::write(0x77, vec![0x7E, 0xB6]),
        I2cTransaction::write_read(0x77, vec![0x00], vec![CHIP_ID]),
        I2cTransaction::write_read(0x77, vec![0x31], vec![0u8; 21]),
    ];

    let i2c = I2cMock::new(&expectations);
    let sensor = Bmp388::new(i2c, NoDelay, Address::Secondary).unwrap();

    assert_eq!(sensor.address(), 0x77);

    let (mut i2c, _) = sensor.destroy();
    i2c.done();
}
