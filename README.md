# bmp388-embedded

A platform-agnostic, `no_std` driver for the **BMP388 / BMP390** barometric pressure and temperature sensor, built on the [`embedded-hal`](https://crates.io/crates/embedded-hal) traits.

This implementation is a clean reimplementation based on the existing Rust crate:

- `bmp280-ehal` by Roma Sokolov & Alexander Zhuravlev: <https://github.com/copterust/bmp280>

Compensation formulas are derived from the official Bosch BMP3 Sensor API:

- <https://github.com/boschsensortec/BMP3_SensorAPI>

## Features

- Blocking (synchronous) API using `embedded-hal` 1.0 (`I2c` + `DelayNs`)
- Optional async I2C + delay API behind the `async` feature using `embedded-hal-async`
- Configurable oversampling, IIR filter, output data rate, and power modes
- Compensated temperature (°C) and pressure (Pa) readings
- Proper error handling — no silently discarded errors
- Compatible with both BMP388 (chip ID `0x50`) and BMP390 (chip ID `0x60`)

### Feature flags

- `async`: enable async driver API (`embedded-hal-async`)

## Blocking example

```rust,ignore
use bmp388_embedded::{Address, Bmp388, Oversampling};

let mut sensor = Bmp388::new(i2c, delay, Address::Primary)?;

// Configure for high accuracy
sensor.set_oversampling(Oversampling::X8, Oversampling::X2)?;

// One-shot measurement (forced mode)
let data = sensor.forced_measurement()?;
let temperature_c = data.temperature;
let pressure_pa = data.pressure;
```

## Async example

Enable the feature:

```toml
[dependencies]
bmp388-embedded = { version = "1.0.0", features = ["async"] }
```

```rust,ignore
use bmp388_embedded::r#async::Bmp388Async;
use bmp388_embedded::{Address, Oversampling};

let mut sensor = Bmp388Async::new(i2c, delay, Address::Primary).await?;

let data = sensor.forced_measurement().await?;
let temperature_c = data.temperature;
let pressure_pa = data.pressure;
```

## License

MIT
