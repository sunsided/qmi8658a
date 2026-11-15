# QST QMI8658A IMU Driver

[![Crates.io][crates-image]][crates-link]
[![Docs][docs-image]][docs-link]
[![Build Status][build-image]][build-link]
[![Coverage][codecov-image]][codecov-link]
[![Safety Dance][safety-image]][safety-link]
![MSRV][msrv-image]
[![EUPL 1.2 licensed][license-eupl-image]][license-eupl-link]
[![Apache 2.0 licensed][license-apache-image]][license-apache-link]
[![MIT licensed][license-mit-image]][license-mit-link]

> A platform-agnostic `embedded-hal` 1.0 driver for the QMI8658A 6-axis IMU
> (accelerometer and gyroscope).

> **Status:** tested against a behavioral model of the chip (host tests and Wokwi), not yet
> verified on real hardware.

- `no_std`, no `unsafe`, I²C or SPI. The bus is taken by value, so `embedded-hal-bus`
  sharing works.
- Blocking API, plus an equivalent async API on `embedded-hal-async` behind the `async` feature
- `WHO_AM_I` check, soft reset with the datasheet's reset status, interface setup (`init`)
- Accelerometer and gyroscope full scale and output data rate, sensor enable
- Acceleration in g, angular rate in dps and temperature in °C, plus the raw values
- Data-available status (`STATUS0`), `STATUSINT`
- INT1/INT2 output enable and the data-ready pulse on INT2
- CTRL9 host commands with the `STATUSINT.CmdDone` handshake and a timeout
- FIFO: bypass, FIFO and stream mode, size, watermark, level, reset, and a drain into a
  buffer with decoded accelerometer/gyroscope frames
- Motion engines: any-, no- and significant-motion, tap, pedometer and Wake on Motion,
  with their configuration, enables, interrupt routing and status
- Accelerometer and gyroscope self-test with the datasheet limits (200 mg, 300 dps)
- Gyroscope calibration-on-demand, gyro gain save/restore, and host delta offsets
- Typed access to every register via `read_register`, `write_register` and `modify_register`

```rust,ignore
use qmi8658a::{AccelFullScale, AccelOdr, GyroFullScale, GyroOdr, Qmi8658a};

let mut imu = Qmi8658a::new(i2c); // or Qmi8658a::new_spi(spi)
imu.init(&mut delay)?;
imu.set_accel(AccelFullScale::G8, AccelOdr::Hz56)?;
imu.set_gyro(GyroFullScale::Dps512, GyroOdr::Hz56)?;
imu.set_sensors(true, true)?;

if imu.status()?.a_da() {
    let (accel, gyro) = imu.read_motion()?;
    let [x, y, z] = accel.g();
    let [gx, gy, gz] = gyro.degrees_per_second();
}
```

`init` must run before the first output read: out of reset, the device neither advances
the register address during a burst nor outputs little-endian data.

The data-ready pulse exists only on INT2. On boards without INT2 wired, poll `status()`
(`STATUS0`); `STATUSINT` bit 0 mirrors the INT2 level and stays low while INT2 is high-Z.

`read_fifo` turns address auto-increment off while it drains `FIFO_DATA` and restores it
afterwards: with auto-increment on, a burst would walk past `FIFO_DATA` (datasheet 16.1).

SyncSample mode, the locking mechanism and the CTRL9 commands for the USID, pull-ups and
clock gating are reachable through the typed registers and `ctrl9_command` only.

## Features

- `async`: adds `qmi8658a::asynch::Qmi8658a` on `embedded-hal-async`.
- `defmt`: derives `defmt::Format` for the public types.
- `coordinate-frame`: adds `SensorFrame<T>`, see below.

## Coordinate frame

The sensor axes form a right-handed set with +Z out of the top face of the package
(datasheet section 2, figure 10); acceleration and angular rate share them. With the
`coordinate-frame` feature, `qmi8658a::SensorFrame<T>` names that frame as
[`coordinate_frame::EastNorthUp`](https://docs.rs/coordinate-frame). Wrap a reading and
convert it into the frame of your board:

```rust,ignore
use coordinate_frame::NorthEastDown;
use qmi8658a::SensorFrame;

let accel = SensorFrame::from(imu.read_acceleration()?.g());
let ned: NorthEastDown<f32> = accel.into();
```

The driver follows the QMI8658A datasheet (document 13-52-25, Rev A); see
[`qmi8658a-registers`](https://crates.io/crates/qmi8658a-registers) for the sources.
No code was taken from other QMI8658 drivers.

## License

Copyright © 2026 Markus Mayer

Triple licensed under your choice of either of:

- European Union Public Licence, Version 1.2, ([LICENSE-EUPL](LICENSE-EUPL)
  or https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)
- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

[crates-image]: https://img.shields.io/crates/v/qmi8658a
[crates-link]: https://crates.io/crates/qmi8658a
[docs-image]: https://docs.rs/qmi8658a/badge.svg
[docs-link]: https://docs.rs/qmi8658a/
[build-image]: https://github.com/sunsided/qmi8658a/workflows/Rust/badge.svg
[build-link]: https://github.com/sunsided/qmi8658a/actions
[codecov-image]: https://codecov.io/gh/sunsided/qmi8658a/graph/badge.svg
[codecov-link]: https://codecov.io/gh/sunsided/qmi8658a
[safety-image]: https://img.shields.io/badge/unsafe-forbidden-success.svg
[safety-link]: https://github.com/rust-secure-code/safety-dance/
[msrv-image]: https://img.shields.io/badge/rustc-1.85+-blue.svg
[license-eupl-image]: https://img.shields.io/badge/license-EUPL_1.2-blue.svg
[license-eupl-link]: https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12
[license-apache-image]: https://img.shields.io/badge/license-Apache_2.0-blue.svg
[license-apache-link]: https://www.apache.org/licenses/LICENSE-2.0
[license-mit-image]: https://img.shields.io/badge/license-MIT-blue.svg
[license-mit-link]: https://opensource.org/licenses/MIT
