//! The driver body, shared by the blocking and the async driver.
//!
//! [`define_driver!`] expands once with plain functions on [`Interface`](crate::Interface)
//! and once with `async` functions on [`AsyncInterface`](crate::AsyncInterface), inserting
//! `.await` after every bus access. All decoding lives in plain functions outside the macro.

use qmi8658a_registers::{
    AccelFullScale, Ctrl2, Ctrl3, GyroFullScale, RegisterAddress, SOFT_RESET_COMMAND,
};

/// The number of 1 ms polls [`reset`](crate::Qmi8658a::reset) waits for the reset status
/// after the turn-on time.
pub const RESET_POLLS: u32 = 10;

/// The time [`ctrl9_command`](crate::Qmi8658a::ctrl9_command) waits for each half of the
/// handshake in the typed command methods, in milliseconds.
///
/// The datasheet gives no command duration except about 1.5 s for calibration-on-demand
/// (14.2).
pub const CTRL9_TIMEOUT_MS: u32 = 100;

/// An error of an operation that waits for or checks the device.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// A bus error.
    Bus(E),
    /// The device did not finish in time.
    Timeout,
    /// `WHO_AM_I` read this value instead of
    /// [`QMI8658A_WHO_AM_I`](crate::QMI8658A_WHO_AM_I).
    WrongIdentity(u8),
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}

/// The full scales the driver uses to convert samples, if known.
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
pub(crate) struct Scales {
    pub accel: Option<AccelFullScale>,
    pub gyro: Option<GyroFullScale>,
}

impl Scales {
    /// The full scales after a reset.
    pub const RESET: Self = Self {
        accel: Some(AccelFullScale::G2),
        gyro: Some(GyroFullScale::Dps16),
    };

    /// Updates the tracked full scales after writing `value` to `address`.
    pub fn track(&mut self, address: u8, value: u8) {
        if address == RegisterAddress::CTRL2.addr() {
            self.accel = Some(Ctrl2::from(value).a_fs());
        } else if address == RegisterAddress::CTRL3.addr() {
            self.gyro = Some(Ctrl3::from(value).g_fs());
        } else if address == RegisterAddress::RESET.addr() && value == SOFT_RESET_COMMAND {
            *self = Self::RESET;
        }
    }
}

/// Defines the driver struct and its methods.
///
/// `$iface` is the interface trait, `$delay` the delay trait, `$($async)?` is empty or
/// `async`, and `$($await)*` is empty or `.await`.
macro_rules! define_driver {
    ($iface:path, $delay:path, [$($async:tt)?], [$($await:tt)*]) => {
        use $crate::driver::{Error, Scales, CTRL9_TIMEOUT_MS, RESET_POLLS};
        use $crate::interface::{I2cInterface, SpiInterface};
        use $crate::{Acceleration, AngularRate, FifoFrames, FifoState, Temperature};
        use qmi8658a_registers::prelude::*;
        use qmi8658a_registers::*;

        /// QMI8658A driver.
        ///
        /// Call [`init`](Self::init) first: burst reads of the output registers need the
        /// interface configuration it writes.
        #[derive(Debug)]
        pub struct Qmi8658a<IFACE> {
            iface: IFACE,
            scales: Scales,
        }

        impl<I2C> Qmi8658a<I2cInterface<I2C>> {
            /// Creates a driver for a device on I²C at [`DEFAULT_DEVICE_ADDRESS`] (`SA0` high
            /// or unconnected). Performs no bus access.
            pub fn new(i2c: I2C) -> Self {
                Self::from_interface(I2cInterface::new(i2c))
            }

            /// Creates a driver for a device on I²C at the specified 7-bit address, such as
            /// [`ALTERNATIVE_DEVICE_ADDRESS`] (`SA0` low). Performs no bus access.
            pub fn with_address(i2c: I2C, address: u8) -> Self {
                Self::from_interface(I2cInterface::with_address(i2c, address))
            }

            /// Consumes the driver and returns the I²C bus.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Qmi8658a<SpiInterface<SPI>> {
            /// Creates a driver for a device on SPI. Performs no bus access.
            pub fn new_spi(spi: SPI) -> Self {
                Self::from_interface(SpiInterface::new(spi))
            }

            /// Consumes the driver and returns the SPI device.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<IFACE> Qmi8658a<IFACE> {
            /// Creates a driver on the specified interface. Performs no bus access.
            pub fn from_interface(iface: IFACE) -> Self {
                Self {
                    iface,
                    scales: Scales::default(),
                }
            }

            /// Consumes the driver and returns the interface.
            pub fn into_interface(self) -> IFACE {
                self.iface
            }
        }

        impl<IFACE> Qmi8658a<IFACE>
        where
            IFACE: $iface,
        {
            // --- identification and reset ----------------------------------------------------

            /// Reads the `WHO_AM_I` register.
            pub $($async)? fn who_am_i(&mut self) -> Result<u8, IFACE::Error> {
                Ok(self.read_register::<WhoAmI>() $($await)* ?.ident())
            }

            /// Checks whether `WHO_AM_I` matches [`QMI8658A_WHO_AM_I`].
            pub $($async)? fn identify(&mut self) -> Result<bool, IFACE::Error> {
                Ok(self.who_am_i() $($await)* ? == QMI8658A_WHO_AM_I)
            }

            /// Reads the `REVISION_ID` register.
            pub $($async)? fn revision_id(&mut self) -> Result<u8, IFACE::Error> {
                Ok(self.read_register::<RevisionId>() $($await)* ?.value())
            }

            /// Checks the identity, resets the device and configures the interface.
            ///
            /// Afterwards both sensors are off; enable them with [`Self::set_sensors`].
            pub $($async)? fn init<D: $delay>(&mut self, delay: &mut D) -> Result<(), Error<IFACE::Error>> {
                let id = self.who_am_i() $($await)* ?;
                if id != QMI8658A_WHO_AM_I {
                    return Err(Error::WrongIdentity(id));
                }

                self.reset(delay) $($await)* ?;
                self.configure_interface() $($await)* ?;
                Ok(())
            }

            /// Soft-resets the device and waits until the reset status reads
            /// [`RESET_DONE`].
            ///
            /// Waits [`TURN_ON_TIME_MS`] first, then polls up to [`RESET_POLLS`] times.
            /// The reset restores the interface defaults (no auto-increment, big-endian), so
            /// call [`Self::configure_interface`] afterwards, or use [`Self::init`].
            pub $($async)? fn reset<D: $delay>(&mut self, delay: &mut D) -> Result<(), Error<IFACE::Error>> {
                self.write_register(Reset::new().with_value(SOFT_RESET_COMMAND)) $($await)* ?;
                delay.delay_ms(TURN_ON_TIME_MS) $($await)*;
                for _ in 0..RESET_POLLS {
                    if self.read_register::<DqyL>() $($await)* ?.value() == RESET_DONE {
                        return Ok(());
                    }

                    delay.delay_ms(1) $($await)*;
                }

                Err(Error::Timeout)
            }

            /// Enables register address auto-increment and little-endian output, which the
            /// burst reads of this driver need, and signals CTRL9 completion on
            /// `STATUSINT.CmdDone` only (`CTRL8.bit7`), so that INT1 stays free.
            /// Keeps the other `CTRL1` and `CTRL8` bits.
            pub $($async)? fn configure_interface(&mut self) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl1| r.with_addr_ai(true).with_be(false)) $($await)* ?;
                self.modify_register(|r: Ctrl8| r.with_ctrl9_handshake_type(true)) $($await)*
            }

            // --- CTRL9 host commands ---------------------------------------------------------

            /// Runs a CTRL9 host command (datasheet 5.10): writes `CTRL9`, waits for
            /// `STATUSINT.CmdDone`, acknowledges with [`Ctrl9Command::Ack`] and waits for
            /// `CmdDone` to clear. Polls every millisecond, for at most `timeout_ms` per wait.
            ///
            /// Commands that take parameters read them from the CAL registers; write those
            /// first. Needs [`Self::configure_interface`] (done by [`Self::init`]).
            pub $($async)? fn ctrl9_command<D: $delay>(&mut self, command: Ctrl9Command, delay: &mut D, timeout_ms: u32) -> Result<(), Error<IFACE::Error>> {
                self.write_register(Ctrl9::new().with_cmd(command)) $($await)* ?;
                self.wait_cmd_done(true, delay, timeout_ms) $($await)* ?;
                self.write_register(Ctrl9::new().with_cmd(Ctrl9Command::Ack)) $($await)* ?;
                self.wait_cmd_done(false, delay, timeout_ms) $($await)*
            }

            $($async)? fn wait_cmd_done<D: $delay>(&mut self, done: bool, delay: &mut D, timeout_ms: u32) -> Result<(), Error<IFACE::Error>> {
                for _ in 0..=timeout_ms {
                    if self.status_int() $($await)* ?.cmd_done() == done {
                        return Ok(());
                    }

                    delay.delay_ms(1) $($await)*;
                }

                Err(Error::Timeout)
            }

            /// Writes the eight CAL registers, `CAL1_L` to `CAL4_H`, in one burst.
            pub $($async)? fn write_cal(&mut self, cal: [u8; 8]) -> Result<(), IFACE::Error> {
                self.iface.write(qmi8658a_registers::RegisterAddress::CAL1_L.addr(), &cal) $($await)*
            }

            // --- FIFO ------------------------------------------------------------------------

            /// Configures the FIFO: mode, size and watermark (in samples; `0` disables the
            /// watermark, and it should not exceed the size). Leaves read mode.
            ///
            /// With both sensors enabled, the datasheet requires the same ODR for both (8.2).
            /// Outside bypass mode, the data-ready pulse is off (6.2).
            pub $($async)? fn set_fifo(&mut self, mode: FifoMode, size: FifoSize, watermark: u8) -> Result<(), IFACE::Error> {
                self.write_register(FifoWtmTh::new().with_value(watermark)) $($await)* ?;
                self.write_register(FifoCtrl::new().with_fifo_mode(mode).with_fifo_size(size)) $($await)*
            }

            /// Routes the FIFO watermark interrupt to INT1 (`true`) or INT2 (`false`, the default).
            pub $($async)? fn set_fifo_interrupt_pin(&mut self, int1: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl1| r.with_fifo_int_sel(int1)) $($await)*
            }

            /// Reads the FIFO status and fill level.
            pub $($async)? fn fifo_state(&mut self) -> Result<FifoState, IFACE::Error> {
                let mut bytes = [0; 2];
                self.iface.read(qmi8658a_registers::RegisterAddress::FIFO_SMPL_CNT.addr(), &mut bytes) $($await)* ?;
                Ok(FifoState::from_bytes(bytes))
            }

            /// Empties the FIFO and clears its flags (`CTRL_CMD_RST_FIFO`).
            pub $($async)? fn reset_fifo<D: $delay>(&mut self, delay: &mut D) -> Result<(), Error<IFACE::Error>> {
                self.ctrl9_command(Ctrl9Command::ResetFifo, delay, CTRL9_TIMEOUT_MS) $($await)*
            }

            /// Enters FIFO read mode (`CTRL_CMD_REQ_FIFO`). New samples are dropped until
            /// read mode ends; [`Self::read_fifo`] does both.
            pub $($async)? fn request_fifo<D: $delay>(&mut self, delay: &mut D) -> Result<(), Error<IFACE::Error>> {
                self.ctrl9_command(Ctrl9Command::RequestFifo, delay, CTRL9_TIMEOUT_MS) $($await)*
            }

            /// Drains whole samples from the FIFO into `buffer` and returns the number of bytes
            /// read. Decode them with [`Self::fifo_frames`].
            ///
            /// Follows datasheet 8.8: reads the fill level, enters read mode, reads
            /// `FIFO_DATA` in one burst, and leaves read mode. Address auto-increment is off
            /// during the burst, so that it stays on `FIFO_DATA` (16.1), and is restored
            /// afterwards, also on error.
            pub $($async)? fn read_fifo<D: $delay>(&mut self, buffer: &mut [u8], delay: &mut D) -> Result<usize, Error<IFACE::Error>> {
                let ctrl7 = self.read_register::<Ctrl7>() $($await)* ?;
                let frame = FifoFrames::frame_len(ctrl7.a_en(), ctrl7.g_en()).max(6);
                let level = usize::from(self.fifo_state() $($await)* ?.bytes);
                let len = level.min(buffer.len()) / frame * frame;
                if len == 0 {
                    return Ok(0);
                }

                self.request_fifo(delay) $($await)* ?;
                let ctrl1 = self.read_register::<Ctrl1>() $($await)* ?;
                let result = match self.write_register(ctrl1.with_addr_ai(false)) $($await)* {
                    Ok(()) => self.iface.read(qmi8658a_registers::RegisterAddress::FIFO_DATA.addr(), &mut buffer[..len]) $($await)*,
                    Err(e) => Err(e),
                };

                let restore = self.write_register(ctrl1) $($await)*;
                let leave = self.modify_register(|r: FifoCtrl| r.with_fifo_rd_mode(false)) $($await)*;
                result?;
                restore?;
                leave?;
                Ok(len)
            }

            /// Decodes FIFO data read by [`Self::read_fifo`], with the enabled sensors and
            /// full scales. Reads `CTRL7`.
            pub $($async)? fn fifo_frames<'a>(&mut self, data: &'a [u8]) -> Result<FifoFrames<'a>, IFACE::Error> {
                let ctrl7 = self.read_register::<Ctrl7>() $($await)* ?;
                let accel = if ctrl7.a_en() { Some(self.accel_full_scale() $($await)* ?) } else { None };
                let gyro = if ctrl7.g_en() { Some(self.gyro_full_scale() $($await)* ?) } else { None };
                Ok(FifoFrames::new(data, accel, gyro))
            }

            // --- accelerometer and gyroscope configuration -----------------------------------

            /// Sets the accelerometer full scale and output data rate.
            ///
            /// Low-power rates ([`AccelOdr::is_low_power`]) need the gyroscope disabled.
            pub $($async)? fn set_accel(&mut self, scale: AccelFullScale, odr: AccelOdr) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl2| r.with_a_fs(scale).with_a_odr(odr)) $($await)*
            }

            /// Sets the gyroscope full scale and output data rate.
            pub $($async)? fn set_gyro(&mut self, scale: GyroFullScale, odr: GyroOdr) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl3| r.with_g_fs(scale).with_g_odr(odr)) $($await)*
            }

            /// Enables or disables the accelerometer and the gyroscope.
            ///
            /// The first data arrives after the turn-on time: 3 ms plus 3/ODR for the
            /// accelerometer, 150 ms plus 3/ODR for the gyroscope (datasheet tables 7 and 8).
            pub $($async)? fn set_sensors(&mut self, accel: bool, gyro: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl7| r.with_a_en(accel).with_g_en(gyro)) $($await)*
            }

            /// Returns the accelerometer full scale. Reads `CTRL2` only if the driver does not know it.
            pub $($async)? fn accel_full_scale(&mut self) -> Result<AccelFullScale, IFACE::Error> {
                match self.scales.accel {
                    Some(scale) => Ok(scale),
                    None => {
                        let scale = self.read_register::<Ctrl2>() $($await)* ?.a_fs();
                        self.scales.accel = Some(scale);
                        Ok(scale)
                    }
                }
            }

            /// Returns the gyroscope full scale. Reads `CTRL3` only if the driver does not know it.
            pub $($async)? fn gyro_full_scale(&mut self) -> Result<GyroFullScale, IFACE::Error> {
                match self.scales.gyro {
                    Some(scale) => Ok(scale),
                    None => {
                        let scale = self.read_register::<Ctrl3>() $($await)* ?.g_fs();
                        self.scales.gyro = Some(scale);
                        Ok(scale)
                    }
                }
            }

            // --- interrupts ------------------------------------------------------------------

            /// Enables the INT1 and INT2 outputs (push-pull). Disabled outputs are high-Z.
            pub $($async)? fn set_interrupt_outputs(&mut self, int1: bool, int2: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl1| r.with_int1_en(int1).with_int2_en(int2)) $($await)*
            }

            /// Drives the data-ready pulse (DRDY) on INT2 (the power-on default), or blocks it.
            ///
            /// DRDY needs the FIFO in bypass mode (the default) and the INT2 output enabled
            /// with [`Self::set_interrupt_outputs`]. Its rising edge marks new data.
            pub $($async)? fn set_data_ready_interrupt(&mut self, enabled: bool) -> Result<(), IFACE::Error> {
                self.modify_register(|r: Ctrl7| r.with_drdy_dis(!enabled)) $($await)*
            }

            // --- output data -----------------------------------------------------------------

            /// Reads `STATUS0`: new accelerometer and gyroscope data.
            pub $($async)? fn status(&mut self) -> Result<Status0, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads `STATUSINT`. Outside SyncSample mode, bits 0 and 1 mirror INT2 and INT1.
            pub $($async)? fn status_int(&mut self) -> Result<StatusInt, IFACE::Error> {
                self.read_register() $($await)*
            }

            /// Reads the accelerometer outputs in one burst.
            pub $($async)? fn read_acceleration(&mut self) -> Result<Acceleration, IFACE::Error> {
                let scale = self.accel_full_scale() $($await)* ?;
                let mut bytes = [0; 6];
                self.iface.read(qmi8658a_registers::RegisterAddress::AX_L.addr(), &mut bytes) $($await)* ?;
                Ok(Acceleration::from_bytes(&bytes, scale))
            }

            /// Reads the gyroscope outputs in one burst.
            pub $($async)? fn read_angular_rate(&mut self) -> Result<AngularRate, IFACE::Error> {
                let scale = self.gyro_full_scale() $($await)* ?;
                let mut bytes = [0; 6];
                self.iface.read(qmi8658a_registers::RegisterAddress::GX_L.addr(), &mut bytes) $($await)* ?;
                Ok(AngularRate::from_bytes(&bytes, scale))
            }

            /// Reads the accelerometer and gyroscope outputs in one burst, so that both come
            /// from the same sample.
            pub $($async)? fn read_motion(&mut self) -> Result<(Acceleration, AngularRate), IFACE::Error> {
                let accel = self.accel_full_scale() $($await)* ?;
                let gyro = self.gyro_full_scale() $($await)* ?;
                let mut bytes = [0; 12];
                self.iface.read(qmi8658a_registers::RegisterAddress::AX_L.addr(), &mut bytes) $($await)* ?;
                let (a, g) = bytes.split_at(6);
                Ok((
                    Acceleration { raw: $crate::data::i16x3(a), scale: accel },
                    AngularRate { raw: $crate::data::i16x3(g), scale: gyro },
                ))
            }

            /// Reads the temperature.
            pub $($async)? fn read_temperature(&mut self) -> Result<Temperature, IFACE::Error> {
                let mut bytes = [0; 2];
                self.iface.read(qmi8658a_registers::RegisterAddress::TEMP_L.addr(), &mut bytes) $($await)* ?;
                Ok(Temperature::from_bytes(bytes))
            }

            // --- register access -------------------------------------------------------------

            /// Reads a single register.
            pub $($async)? fn read_register<R: Register>(&mut self) -> Result<R, IFACE::Error> {
                let mut buffer = [0];
                self.iface.read(*<R as I2CRegister<_, _, _>>::REGISTER_ADDRESS, &mut buffer) $($await)* ?;
                Ok(R::from_bits(buffer[0]))
            }

            /// Writes a single register.
            pub $($async)? fn write_register<R: WritableRegister>(&mut self, register: R) -> Result<(), IFACE::Error> {
                let address = *<R as I2CRegister<_, _, _>>::REGISTER_ADDRESS;
                let bits = register.to_bits();
                self.iface.write(address, &[bits]) $($await)* ?;
                self.scales.track(address, bits);
                Ok(())
            }

            /// Reads, modifies and writes back a single register.
            pub $($async)? fn modify_register<R, F>(&mut self, f: F) -> Result<(), IFACE::Error>
            where
                R: Register + WritableRegister,
                F: FnOnce(R) -> R,
            {
                let register = self.read_register::<R>() $($await)* ?;
                self.write_register(f(register)) $($await)*
            }
        }
    };
}

pub(crate) use define_driver;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scale_tracking() {
        let mut scales = Scales::default();
        scales.track(RegisterAddress::CTRL2.addr(), 0x27);
        scales.track(RegisterAddress::CTRL3.addr(), 0x57);
        assert_eq!(scales.accel, Some(AccelFullScale::G8));
        assert_eq!(scales.gyro, Some(GyroFullScale::Dps512));
        scales.track(RegisterAddress::RESET.addr(), 0x0B);
        assert_eq!(scales.accel, Some(AccelFullScale::G8));
        scales.track(RegisterAddress::RESET.addr(), SOFT_RESET_COMMAND);
        assert_eq!(scales, Scales::RESET);
    }
}
