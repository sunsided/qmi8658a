# Changelog

All notable changes to this project will be documented in this file.
This project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `coordinate-frame` feature: `SensorFrame<T>`, the sensor axes as
  `coordinate_frame::EastNorthUp` (right-handed, +Z up; datasheet figure 10), for converting
  readings into other frames.

## [0.1.0] - 2026-11-15

[0.1.0]: https://github.com/sunsided/qmi8658a/releases/tag/v0.1.0

### Added

- Blocking driver on embedded-hal 1.0 and async driver on embedded-hal-async (`async` feature).
- I²C (at `0x6A` or `0x6B`) and SPI interfaces.
- Identity check, soft reset and interface setup (`init`), including the `STATUSINT.CmdDone`
  CTRL9 handshake (`CTRL8.bit7`).
- Accelerometer and gyroscope full scale, output data rate and enable.
- Acceleration (g), angular rate (dps) and temperature (°C) readings, with raw values.
- Status registers, INT1/INT2 output enable and the data-ready pulse on INT2.
- CTRL9 host command handshake (`ctrl9_command`) with a timeout.
- FIFO configuration, level, reset and drain, with decoded frames; address auto-increment
  is off during the drain and restored afterwards.
- Any-, no- and significant-motion, tap, pedometer and Wake on Motion configuration,
  enables, interrupt routing, status and step count.
- Accelerometer and gyroscope self-test with the datasheet's procedure and limits.
- Gyroscope calibration-on-demand, gyro gain save and restore, and accelerometer and
  gyroscope host delta offsets.
