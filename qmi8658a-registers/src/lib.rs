//! # QST QMI8658A IMU Registers
//!
//! This crate provides a typed map of the QMI8658A's user interface registers, following
//! the QMI8658A datasheet (document 13-52-25, Rev A).
//!
//! Every register can be addressed over I²C and SPI. Multi-byte values (calibration,
//! timestamp, temperature, acceleration, angular rate, step count) are split into one
//! register per byte.
//!
//! Consecutive registers can be read in one burst only while
//! [`Ctrl1::addr_ai`] is set; with the power-on default (`0`), a burst reads the same
//! register over and over. The datasheet does not support burst writes to the
//! configuration registers ([`Ctrl1`] through [`Ctrl8`]); write them one at a time.
//!
//! On SPI, the first byte holds the read bit ([`SPI_READ`]) and the register address.

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]
#![forbid(unsafe_code)]
// bitfield-struct generates `with_*_checked` setters that return `Result<_, ()>`.
#![allow(clippy::result_unit_err)]
// Enables the `doc_cfg` feature when the `docsrs` configuration attribute is defined.
#![cfg_attr(docsrs, feature(doc_cfg))]

/// Exports commonly used traits.
pub mod prelude {
    pub use crate::{Register, WritableRegister};
    pub use hardware_registers::i2c::*;
    pub use hardware_registers::register_address::RegisterAddress8;
    pub use hardware_registers::sizes::R1;
    pub use hardware_registers::spi::{
        ReadableSPIRegister8, SPIRegister, SPIRegister8, WritableSPIRegister8,
    };
    pub use hardware_registers::{
        FromBits, HardwareRegister, ToBits, TryFromBits, WritableHardwareRegister,
    };
}

/// Implements the I²C and SPI register traits and `ToBits`, shared by all registers.
macro_rules! register_address {
    ($type:ident, $addr:expr) => {
        impl $crate::prelude::HardwareRegister<$crate::prelude::R1> for $type {}

        impl
            $crate::prelude::I2CRegister<
                $crate::prelude::DeviceAddress7,
                $crate::prelude::RegisterAddress8,
                $crate::prelude::R1,
            > for $type
        {
            type Backing = u8;

            const DEFAULT_DEVICE_ADDRESS: $crate::prelude::DeviceAddress7 =
                $crate::prelude::DeviceAddress7::new($crate::DEFAULT_DEVICE_ADDRESS);
            const REGISTER_ADDRESS: $crate::prelude::RegisterAddress8 =
                $crate::prelude::RegisterAddress8::new(($addr).addr());
        }

        impl $crate::prelude::SPIRegister<$crate::prelude::RegisterAddress8, $crate::prelude::R1>
            for $type
        {
            type Backing = u8;

            const REGISTER_ADDRESS: $crate::prelude::RegisterAddress8 =
                $crate::prelude::RegisterAddress8::new(($addr).addr());
        }

        impl $crate::prelude::ToBits for $type {
            type Target = u8;

            #[inline]
            fn to_bits(&self) -> Self::Target {
                (*self).into()
            }
        }
    };
}

macro_rules! readable_register {
    ($type:ident, $addr:expr) => {
        register_address!($type, $addr);
        impl $crate::Register for $type {}

        impl $crate::prelude::FromBits<u8> for $type {
            #[inline]
            fn from_bits(value: u8) -> Self {
                value.into()
            }

            #[inline]
            fn from_bits_ref(value: &u8) -> Self {
                (*value).into()
            }
        }
    };
}

macro_rules! writable_register {
    ($type:ident, $addr:expr) => {
        readable_register!($type, $addr);
        impl $crate::prelude::WritableHardwareRegister<$crate::prelude::R1> for $type {}
        impl $crate::WritableRegister for $type {}
    };
}

/// A register the datasheet lists as write-only: no [`Register`] implementation.
macro_rules! write_only_register {
    ($type:ident, $addr:expr) => {
        register_address!($type, $addr);
        impl $crate::prelude::WritableHardwareRegister<$crate::prelude::R1> for $type {}
        impl $crate::WritableRegister for $type {}
    };
}

mod registers;
mod types;

pub use registers::*;
pub use types::*;

/// The I²C address with the `SDO/SA0` pin pulled up or left unconnected (datasheet 16.3.1).
pub const DEFAULT_DEVICE_ADDRESS: u8 = 0x6A;

/// The I²C address with the `SDO/SA0` pin pulled down (datasheet 16.3.1).
pub const ALTERNATIVE_DEVICE_ADDRESS: u8 = 0x6B;

/// The read bit in the first byte of an SPI transfer. Writes leave it cleared
/// (datasheet figure 28).
pub const SPI_READ: u8 = 0x80;

/// The value of the [`WhoAmI`] register for a QMI8658A.
pub const QMI8658A_WHO_AM_I: u8 = 0x05;

/// The value written to [`Reset`] to trigger a soft reset (datasheet table 27).
///
/// Section 7.4 of the same datasheet names `0x0B` instead; the register table is followed here.
pub const SOFT_RESET_COMMAND: u8 = 0xB0;

/// The value of [`DqyL`] (`0x4D`) after a successful power-on or soft reset
/// (datasheet 5.9 and 7.4).
///
/// Enabling the sensors or running a CTRL9 command may overwrite it, so read it right after
/// the reset.
pub const RESET_DONE: u8 = 0x80;

/// The time the device needs after power-up or a soft reset before it accepts writes,
/// in milliseconds (System Turn On Time, datasheet tables 7 and 8 and section 7.3).
pub const TURN_ON_TIME_MS: u32 = 15;

/// The temperature sensitivity of the temperature registers, in LSB per °C (datasheet table 18).
///
/// A raw value of `0` corresponds to 0 °C: `T = TEMP_H + TEMP_L / 256`.
pub const TEMPERATURE_LSB_PER_CELSIUS: f32 = 256.0;

/// A readable register.
///
/// Every bit pattern decodes, so reading is infallible.
pub trait Register:
    prelude::ReadableI2CRegister8<prelude::DeviceAddress7>
    + prelude::ReadableSPIRegister8
    + prelude::FromBits<u8>
    + From<u8>
    + Into<u8>
{
}

/// A writable register.
///
/// [`Reset`] is the only writable register that is not also a [`Register`].
pub trait WritableRegister:
    prelude::WritableI2CRegister8<prelude::DeviceAddress7> + prelude::WritableSPIRegister8 + Into<u8>
{
}
