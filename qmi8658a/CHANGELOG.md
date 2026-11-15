# Changelog

All notable changes to this project will be documented in this file.
This project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Blocking driver on embedded-hal 1.0 and async driver on embedded-hal-async (`async` feature).
- I²C (at `0x6A` or `0x6B`) and SPI interfaces.
- Identity check, soft reset and interface setup (`init`).
- Accelerometer and gyroscope full scale, output data rate and enable.
- Acceleration (g), angular rate (dps) and temperature (°C) readings, with raw values.
- Status registers, INT1/INT2 output enable and the data-ready pulse on INT2.
