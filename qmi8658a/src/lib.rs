//! # QST QMI8658A IMU driver
//!
//! > A platform-agnostic driver for the QST QMI8658A 6-axis IMU (accelerometer and
//! > gyroscope) over I²C or SPI.
//!
//! This driver was built using [`embedded-hal`] 1.0 and [`qmi8658a-registers`]. Enable the
//! `async` feature for an equivalent driver on [`embedded-hal-async`] in the `asynch` module.
//!
//! The driver takes the bus by value. To share the bus with other devices, pass a
//! bus handle such as `embedded_hal_bus::i2c::RefCellDevice`, or `&mut` to a bus.
//!
//! ## Example
//!
//! ```
//! # use core::cell::RefCell;
//! # use qmi8658a_sim::{FakeDelay, FakeI2c, Qmi8658aModel};
//! use qmi8658a::{AccelFullScale, AccelOdr, GyroFullScale, GyroOdr, Qmi8658a};
//! # let model = RefCell::new(Qmi8658aModel::new());
//! # let i2c = FakeI2c::new(&model);
//! # let mut delay = FakeDelay::new(&model);
//!
//! let mut imu = Qmi8658a::new(i2c);
//! imu.init(&mut delay)?;
//!
//! imu.set_accel(AccelFullScale::G8, AccelOdr::Hz56)?;
//! imu.set_gyro(GyroFullScale::Dps512, GyroOdr::Hz56)?;
//! imu.set_sensors(true, true)?;
//! # model.borrow_mut().set_acceleration_mg([0, 0, 1000]);
//! # model.borrow_mut().tick(20_000);
//!
//! if imu.status()?.a_da() {
//!     let (accel, gyro) = imu.read_motion()?;
//!     let [x, y, z] = accel.g();
//!     let [gx, gy, gz] = gyro.degrees_per_second();
//!     assert_eq!(z, 1.0);
//! }
//! # Ok::<(), qmi8658a::Error<embedded_hal::i2c::ErrorKind>>(())
//! ```
//!
//! ## Interface configuration
//!
//! Out of reset, the device neither advances the register address during a burst nor
//! outputs little-endian data (`CTRL1.ADDR_AI = 0`, `CTRL1.BE = 1`). [`Qmi8658a::init`]
//! and [`Qmi8658a::configure_interface`] set both, and the output reads rely on it.
//!
//! ## Data ready
//!
//! The data-ready pulse (DRDY) exists only on INT2. Without INT2, poll
//! [`Qmi8658a::status`] for new data instead of `STATUSINT` bit 0: outside SyncSample
//! mode, that bit mirrors the INT2 level (datasheet 6.2).
//!
//! ## Not covered
//!
//! FIFO, CTRL9 host commands, SyncSample mode, the motion engines, self-test and
//! calibration-on-demand have typed registers in [`registers`], reachable with
//! [`Qmi8658a::read_register`], [`Qmi8658a::write_register`] and
//! [`Qmi8658a::modify_register`], but no driver methods.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal
//! [`embedded-hal-async`]: https://crates.io/crates/embedded-hal-async
//! [`qmi8658a-registers`]: https://crates.io/crates/qmi8658a-registers

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]
#![forbid(unsafe_code)]
// Enables the `doc_cfg` feature when the `docsrs` configuration attribute is defined.
#![cfg_attr(docsrs, feature(doc_cfg))]

#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
pub mod asynch;
mod data;
mod driver;
mod interface;

pub use data::{Acceleration, AngularRate, Temperature};
pub use driver::{Error, RESET_POLLS};
#[cfg(feature = "async")]
pub use interface::AsyncInterface;
pub use interface::{I2cInterface, Interface, SpiInterface};
pub use qmi8658a_registers as registers;
pub use qmi8658a_registers::{
    ALTERNATIVE_DEVICE_ADDRESS, AccelFullScale, AccelOdr, DEFAULT_DEVICE_ADDRESS, GyroFullScale,
    GyroOdr, QMI8658A_WHO_AM_I, TURN_ON_TIME_MS,
};

pub use blocking::Qmi8658a;

/// The blocking driver.
mod blocking {
    crate::driver::define_driver!(crate::Interface, embedded_hal::delay::DelayNs, [], []);
}
