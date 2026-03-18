#![cfg(feature = "async")]

use core::cell::RefCell;

use bmp388_embedded::{r#async::Bmp388Async, Address, Error};

/// BMP388 chip ID.
const CHIP_ID: u8 = 0x50;

#[derive(Debug, Default)]
struct TestI2c {
    writes: RefCell<std::vec::Vec<(u8, std::vec::Vec<u8>)>>,
    read_responses: RefCell<std::vec::Vec<std::vec::Vec<u8>>>,
}

impl TestI2c {
    fn push_read_response(&self, data: std::vec::Vec<u8>) {
        self.read_responses.borrow_mut().push(data);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TestError;

impl embedded_hal::i2c::Error for TestError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl embedded_hal_async::i2c::ErrorType for TestI2c {
    type Error = TestError;
}

impl embedded_hal_async::i2c::I2c for TestI2c {
    async fn read(&mut self, _address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        let mut q = self.read_responses.borrow_mut();
        if q.is_empty() {
            read.fill(0);
            return Ok(());
        }
        let next = q.remove(0);
        read[..next.len()].copy_from_slice(&next);
        Ok(())
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.writes.borrow_mut().push((address, write.to_vec()));
        Ok(())
    }

    async fn write_read(
        &mut self,
        _address: u8,
        _write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut q = self.read_responses.borrow_mut();
        if q.is_empty() {
            read.fill(0);
            return Ok(());
        }
        let next = q.remove(0);
        read[..next.len()].copy_from_slice(&next);
        Ok(())
    }

    async fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug, Default)]
struct NoDelay;

impl embedded_hal_async::delay::DelayNs for NoDelay {
    async fn delay_ns(&mut self, _ns: u32) {}
}

/// Set up the mock I2C with standard init responses:
/// 1. chip_id read -> CHIP_ID
/// 2. calibration read -> 21 zero bytes
fn setup_init_responses(i2c: &TestI2c) {
    // chip_id response
    i2c.push_read_response(std::vec![CHIP_ID]);
    // calibration data response (21 bytes of zeros)
    i2c.push_read_response(std::vec![0u8; 21]);
}

#[tokio::test]
async fn async_new_validates_chip_id_and_reads_calibration() {
    let i2c = TestI2c::default();
    setup_init_responses(&i2c);

    let sensor = Bmp388Async::new(i2c, NoDelay, Address::Primary)
        .await
        .unwrap();

    assert_eq!(sensor.address(), 0x76);
}

#[tokio::test]
async fn async_new_rejects_wrong_chip_id() {
    let i2c = TestI2c::default();
    // Wrong chip ID
    i2c.push_read_response(std::vec![0xFF]);

    let result = Bmp388Async::new(i2c, NoDelay, Address::Primary).await;
    match result {
        Err(Error::InvalidChipId(0xFF)) => {} // expected
        other => panic!("Expected InvalidChipId(0xFF), got {:?}", other),
    }
}

#[tokio::test]
async fn async_forced_measurement_returns_valid_data() {
    let i2c = TestI2c::default();
    setup_init_responses(&i2c);
    // Status response: both data ready (0x60)
    i2c.push_read_response(std::vec![0x60]);
    // Sensor data: 6 bytes (pressure + temperature)
    i2c.push_read_response(std::vec![0x00, 0x00, 0x80, 0x00, 0x00, 0x80]);

    let mut sensor = Bmp388Async::new(i2c, NoDelay, Address::Primary)
        .await
        .unwrap();

    let data = sensor.forced_measurement().await.unwrap();
    assert!(data.temperature >= -40.0 && data.temperature <= 85.0);
    assert!(data.pressure >= 30_000.0 && data.pressure <= 125_000.0);
}

#[tokio::test]
async fn async_sensor_data_reads_measurement() {
    let i2c = TestI2c::default();
    setup_init_responses(&i2c);
    // Sensor data: 6 bytes
    i2c.push_read_response(std::vec![0x00, 0x00, 0x80, 0x00, 0x00, 0x80]);

    let mut sensor = Bmp388Async::new(i2c, NoDelay, Address::Primary)
        .await
        .unwrap();

    let data = sensor.sensor_data().await.unwrap();
    assert!(data.temperature >= -40.0);
    assert!(data.pressure >= 30_000.0);
}
