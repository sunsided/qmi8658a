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
//! ## CTRL9 host commands
//!
//! [`Qmi8658a::ctrl9_command`] runs any command of datasheet table 28 with the handshake of
//! section 5.10. [`Qmi8658a::configure_interface`] (part of [`Qmi8658a::init`]) selects the
//! `STATUSINT.CmdDone` handshake, so INT1 stays free. The typed methods for the FIFO, the
//! motion engines and calibration use it with [`CTRL9_TIMEOUT_MS`], or [`COD_TIMEOUT_MS`]
//! for calibration-on-demand.
//!
//! ## FIFO
//!
//! [`Qmi8658a::set_fifo`] selects mode, size and watermark. [`Qmi8658a::read_fifo`] drains
//! whole samples into a buffer, following datasheet 8.8: it enters read mode, turns address
//! auto-increment off so that the burst stays on `FIFO_DATA` (16.1), and restores it
//! afterwards. [`Qmi8658a::fifo_frames`] decodes the bytes into [`FifoFrame`]s.
//!
//! ## Motion engines, self-test and calibration
//!
//! - Any-, no- and significant-motion: [`Qmi8658a::configure_motion`],
//!   [`Qmi8658a::set_motion_detection`]
//! - Tap: [`Qmi8658a::configure_tap`], [`Qmi8658a::set_tap_detection`],
//!   [`Qmi8658a::tap_status`]
//! - Pedometer: [`Qmi8658a::configure_pedometer`], [`Qmi8658a::set_pedometer`],
//!   [`Qmi8658a::step_count`], [`Qmi8658a::reset_step_count`]
//! - Event flags and routing: [`Qmi8658a::motion_status`],
//!   [`Qmi8658a::set_activity_interrupt_pin`]
//! - Wake on Motion: [`Qmi8658a::enable_wake_on_motion`],
//!   [`Qmi8658a::disable_wake_on_motion`]
//! - Self-test: [`Qmi8658a::self_test_accel`], [`Qmi8658a::self_test_gyro`]
//! - Calibration: [`Qmi8658a::calibrate_gyro`], [`Qmi8658a::gyro_gains`],
//!   [`Qmi8658a::apply_gyro_gains`], [`Qmi8658a::apply_accel_delta_offset`],
//!   [`Qmi8658a::apply_gyro_delta_offset`]
//!
//! ## Not covered
//!
//! SyncSample mode and the locking mechanism, `CTRL_CMD_COPY_USID`, `CTRL_CMD_SET_RPU` and
//! `CTRL_CMD_AHB_CLOCK_GATING` have typed registers in [`registers`], reachable with
//! [`Qmi8658a::read_register`], [`Qmi8658a::write_register`],
//! [`Qmi8658a::modify_register`] and [`Qmi8658a::ctrl9_command`], but no typed methods.
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
mod engines;
mod fifo;
mod interface;
mod self_test;

pub use data::{Acceleration, AngularRate, Temperature};
pub use driver::{COD_TIMEOUT_MS, CTRL9_TIMEOUT_MS, Error, RESET_POLLS};
pub use engines::{MotionConfig, PedometerConfig, TapConfig, TapPriority, WomConfig};
pub use fifo::{FifoFrame, FifoFrames, FifoState};
#[cfg(feature = "async")]
pub use interface::AsyncInterface;
pub use interface::{I2cInterface, Interface, SpiInterface};
pub use qmi8658a_registers as registers;
pub use qmi8658a_registers::{
    ALTERNATIVE_DEVICE_ADDRESS, AccelFullScale, AccelOdr, Ctrl9Command, DEFAULT_DEVICE_ADDRESS,
    FifoMode, FifoSize, GyroFullScale, GyroOdr, QMI8658A_WHO_AM_I, TURN_ON_TIME_MS,
};
pub use self_test::{
    ACCEL_SELF_TEST_MIN_MG, AccelSelfTest, GYRO_SELF_TEST_MIN_DPS, GyroSelfTest,
    SELF_TEST_TIMEOUT_MS,
};

pub use blocking::Qmi8658a;

/// A vector in the sensor's own axes, as a coordinate-frame type.
///
/// Datasheet 13-52-25 Rev A, section 2, figure 10 ("Chip Orientation Coordinate System"):
/// +Z points out of the top face of the package, and X, Y, Z form a right-handed set.
/// Acceleration and angular rate share these axes. With +Z as Up, the right-handed X/Y
/// pair maps to East/North, so the sensor frame is [`EastNorthUp`](coordinate_frame::EastNorthUp).
///
/// Wrap a reading with `SensorFrame::from` and convert it into the board's frame with `.into()`:
///
/// ```
/// # use core::cell::RefCell;
/// # use qmi8658a_sim::{FakeDelay, FakeI2c, Qmi8658aModel};
/// use coordinate_frame::NorthEastDown;
/// use qmi8658a::{AccelFullScale, AccelOdr, Qmi8658a, SensorFrame};
/// # let model = RefCell::new(Qmi8658aModel::new());
/// # model.borrow_mut().set_acceleration_mg([0, 0, 1000]);
/// # let mut imu = Qmi8658a::new(FakeI2c::new(&model));
/// # imu.init(&mut FakeDelay::new(&model))?;
/// # imu.set_accel(AccelFullScale::G8, AccelOdr::Hz897)?;
/// # imu.set_sensors(true, false)?;
/// # model.borrow_mut().tick(1_000);
///
/// let accel = SensorFrame::from(imu.read_acceleration()?.g());
/// let ned: NorthEastDown<f32> = accel.into();
/// assert_eq!(ned.down(), -1.0);
/// # Ok::<(), qmi8658a::Error<embedded_hal::i2c::ErrorKind>>(())
/// ```
#[cfg(feature = "coordinate-frame")]
#[cfg_attr(docsrs, doc(cfg(feature = "coordinate-frame")))]
pub type SensorFrame<T> = coordinate_frame::EastNorthUp<T>;

/// The blocking driver.
mod blocking {
    crate::driver::define_driver!(crate::Interface, embedded_hal::delay::DelayNs, [], []);
}
