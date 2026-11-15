//! # QST QMI8658A behavioral model
//!
//! A register-level model of the QMI8658A 6-axis IMU, used in two places:
//!
//! - as a fake I²C bus ([`FakeI2c`]) or SPI device ([`FakeSpi`]) in host tests of the
//!   `qmi8658a` driver, with a delay ([`FakeDelay`]) that advances the model's time, and
//! - wrapped into a [Wokwi](https://wokwi.com) custom chip (`sim/wokwi-chip`).
//!
//! The model implements:
//!
//! - the I²C protocol (register pointer write followed by reads or writes) and the SPI
//!   protocol (read bit plus address, then data), with address auto-increment only while
//!   `CTRL1.ADDR_AI` is set: otherwise a burst accesses the same register over and over,
//! - `WHO_AM_I`, `REVISION_ID`, soft reset via `RESET` and the reset status in `dQY_L`,
//! - accelerometer and gyroscope enable, full scale and output data rate, with the
//!   accelerometer rate depending on whether the gyroscope runs (datasheet table 22),
//! - acceleration, angular rate and temperature outputs from settable inputs
//!   ([`Qmi8658aModel::set_acceleration_mg`], [`Qmi8658aModel::set_angular_rate_mdps`],
//!   [`Qmi8658aModel::set_temperature`]), saturating at the 16-bit range,
//! - the data-available flags in `STATUS0`, the sample timestamp,
//! - the data-ready pulse (DRDY) on INT2, the INT1/INT2 output enables and the INT1/INT2
//!   level mirror in `STATUSINT` bits 1 and 0,
//! - the `CTRL1.BE` byte order,
//! - the CTRL9 handshake: any command sets `STATUSINT.CmdDone`, `CTRL_CMD_ACK` clears it.
//!
//! ## Not modelled
//!
//! The FIFO (`FIFO_DATA` reads `0`), the effect of CTRL9 commands, SyncSample mode and
//! the locking mechanism, the motion engines (any-, no-, significant-motion, tap,
//! pedometer, Wake on Motion), self-test, calibration-on-demand, the low-pass filters,
//! low-power and snooze modes, power-down, turn-on times, the FIFO interrupt and the
//! INT1 sources. The bits can be written and read back, but do not change the output.
//!
//! ## Assumptions
//!
//! The datasheet (13-52-25 Rev A) leaves these points open; the model fills them in as follows.
//!
//! - **`STATUS0` clearing**: "no updates since last read" does not say which read clears
//!   `aDA`/`gDA`. The model clears both flags when `STATUS0` is read.
//! - **`CTRL1.BE`**: the datasheet only says "read data big-endian". The model swaps the
//!   two bytes of each 16-bit temperature, acceleration and angular rate output, so that
//!   the high byte sits at the lower (`_L`) address.
//! - **`STATUSINT` bits 0 and 1**: outside SyncSample mode they show "the same value" as
//!   INT2 and INT1 (datasheet 6.2). The model mirrors the driven pin level: with the pin
//!   in high-Z mode (`CTRL1.INT2_EN = 0`), bit 0 stays `0`.
//! - **DRDY pulse width**: the datasheet says DRDY "is cleared automatically after a short
//!   duration". The model raises INT2 on a sample and drops it at the start of the next
//!   [`Qmi8658aModel::tick`].
//! - **Soft reset**: table 27 names `0xB0` as the reset command, section 7.4 `0x0B`. The
//!   model resets on `0xB0` only, and completes the reset at once.
//! - **`REVISION_ID`**: table 19 gives `0x68`, table 21 `0x7C`. The model reads
//!   [`MODEL_REVISION_ID`].
//! - **Timestamp**: it counts samples "from sensor with highest ODR". The model counts
//!   gyroscope samples while the gyroscope runs, otherwise accelerometer samples.
//! - **Temperature**: the sensor refreshes at 8 Hz (table 18); the model writes the
//!   current input with every sample.

#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

mod bus;

pub use bus::{FakeDelay, FakeI2c, FakeSpi};

use qmi8658a_registers::{
    Ctrl1, Ctrl2, Ctrl3, Ctrl7, Ctrl9Command, DEFAULT_DEVICE_ADDRESS, FifoCtrl, FifoMode,
    QMI8658A_WHO_AM_I, RESET_DONE, RegisterAddress as Reg, SOFT_RESET_COMMAND, SPI_READ,
};

/// The value the model returns for `REVISION_ID` (datasheet table 21).
pub const MODEL_REVISION_ID: u8 = 0x7C;

/// The phase of an SPI transfer.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum SpiState {
    /// The next byte holds the read bit and the register address.
    Command,
    /// The transfer reads registers.
    Read,
    /// The transfer writes registers.
    Write,
}

/// Register addresses that accept writes, including the reserved `0x05` and `0x07`.
const WRITABLE: &[u8] = &[
    0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x12, 0x13, 0x14,
];

/// A behavioral model of the QMI8658A.
#[derive(Debug, Clone)]
pub struct Qmi8658aModel {
    regs: [u8; 256],
    address: u8,
    pointer: u8,
    /// `true` until the first byte of a write transaction set the register pointer.
    expect_pointer: bool,
    /// The acceleration seen by the sensor, in mg.
    acceleration_mg: [i32; 3],
    /// The angular rate seen by the sensor, in mdps.
    angular_rate_mdps: [i32; 3],
    /// The die temperature, in LSB (1/256 °C).
    temperature: i16,
    /// Time since the last accelerometer sample, in nanoseconds.
    accel_elapsed_ns: u64,
    /// Time since the last gyroscope sample, in nanoseconds.
    gyro_elapsed_ns: u64,
    /// The internal DRDY signal, before the INT2 output enable.
    drdy: bool,
    /// Converts the nanoseconds of [`fake_bus::Clock`] to the whole microseconds of
    /// [`Self::tick`].
    ticks: fake_bus::Micros,
    spi: SpiState,
}

impl Default for Qmi8658aModel {
    fn default() -> Self {
        Self::new()
    }
}

impl Qmi8658aModel {
    /// Creates a model at [`DEFAULT_DEVICE_ADDRESS`] (`SA0` high), freshly out of power-on reset.
    pub fn new() -> Self {
        Self::with_address(DEFAULT_DEVICE_ADDRESS)
    }

    /// Creates a model at the specified 7-bit I²C address, freshly out of power-on reset.
    pub fn with_address(address: u8) -> Self {
        let mut model = Self {
            regs: [0; 256],
            address,
            pointer: 0,
            expect_pointer: false,
            acceleration_mg: [0; 3],
            angular_rate_mdps: [0; 3],
            temperature: 0,
            accel_elapsed_ns: 0,
            gyro_elapsed_ns: 0,
            drdy: false,
            ticks: fake_bus::Micros::new(),
            spi: SpiState::Command,
        };

        model.reset();
        model
    }

    /// Resets all registers to their defaults, as by power-on or soft reset.
    pub fn reset(&mut self) {
        self.regs = [0; 256];
        self.set(Reg::WHO_AM_I, QMI8658A_WHO_AM_I);
        self.set(Reg::REVISION_ID, MODEL_REVISION_ID);
        self.set(Reg::CTRL1, Ctrl1::new().into());
        self.set(Reg::DQY_L, RESET_DONE);
        self.pointer = 0;
        self.expect_pointer = false;
        self.accel_elapsed_ns = 0;
        self.gyro_elapsed_ns = 0;
        self.drdy = false;
    }

    /// The 7-bit I²C address of the model.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Reads a register directly, bypassing the bus and its side effects.
    pub fn register(&self, reg: Reg) -> u8 {
        self.regs[usize::from(reg.addr())]
    }

    /// Sets the acceleration seen by the sensor, in mg.
    ///
    /// The value shows in the output registers with the next accelerometer sample.
    pub fn set_acceleration_mg(&mut self, mg: [i32; 3]) {
        self.acceleration_mg = mg;
    }

    /// Sets the angular rate seen by the sensor, in millidegrees per second.
    ///
    /// The value shows in the output registers with the next gyroscope sample.
    pub fn set_angular_rate_mdps(&mut self, mdps: [i32; 3]) {
        self.angular_rate_mdps = mdps;
    }

    /// Sets the die temperature, in LSB (256 LSB per °C, `0` at 0 °C).
    pub fn set_temperature(&mut self, raw: i16) {
        self.temperature = raw;
    }

    /// The level of the INT1 pin. The model has no INT1 sources, so it is always low.
    pub fn int1_pin(&self) -> bool {
        false
    }

    /// The level of the INT2 pin: the data-ready pulse, if the pin output is enabled.
    pub fn int2_pin(&self) -> bool {
        self.ctrl1().int2_en() && self.drdy
    }

    /// Advances time by `us` microseconds and samples the enabled sensors at their output
    /// data rates. Ends a data-ready pulse from a previous call.
    pub fn tick(&mut self, us: u32) {
        self.drdy = false;
        let ctrl7 = Ctrl7::from(self.register(Reg::CTRL7));
        let ns = u64::from(us) * 1_000;
        let gyro_runs = ctrl7.g_en() && !ctrl7.g_sn();

        let accel_hz = Ctrl2::from(self.register(Reg::CTRL2))
            .a_odr()
            .hz(ctrl7.g_en());
        if let (true, Some(hz)) = (ctrl7.a_en(), accel_hz) {
            for _ in 0..Self::due(&mut self.accel_elapsed_ns, ns, hz) {
                self.sample_accel(!gyro_runs);
            }
        }

        let gyro_hz = Ctrl3::from(self.register(Reg::CTRL3)).g_odr().hz();
        if let (true, Some(hz)) = (gyro_runs, gyro_hz) {
            for _ in 0..Self::due(&mut self.gyro_elapsed_ns, ns, hz) {
                self.sample_gyro();
            }
        }
    }

    /// Adds `ns` to `elapsed` and returns the number of whole sample periods that passed.
    fn due(elapsed: &mut u64, ns: u64, hz: f32) -> u64 {
        let period = (1.0e9 / hz) as u64;
        *elapsed += ns;
        let samples = *elapsed / period;
        *elapsed %= period;
        samples
    }

    fn ctrl1(&self) -> Ctrl1 {
        Ctrl1::from(self.register(Reg::CTRL1))
    }

    fn set(&mut self, reg: Reg, value: u8) {
        self.regs[usize::from(reg.addr())] = value;
    }

    /// Writes 16-bit values to consecutive register pairs, in the byte order of `CTRL1.BE`.
    fn set_i16s(&mut self, low: Reg, values: &[i16]) {
        let be = self.ctrl1().be();
        let mut addr = usize::from(low.addr());
        for value in values {
            let bytes = if be {
                value.to_be_bytes()
            } else {
                value.to_le_bytes()
            };

            self.regs[addr..addr + 2].copy_from_slice(&bytes);
            addr += 2;
        }
    }

    /// Converts physical values in milli-units to raw outputs, saturating.
    fn to_raw(milli: [i32; 3], lsb_per_unit: i64) -> [i16; 3] {
        milli.map(|v| {
            (i64::from(v) * lsb_per_unit / 1000).clamp(i64::from(i16::MIN), i64::from(i16::MAX))
                as i16
        })
    }

    fn sample_accel(&mut self, count_timestamp: bool) {
        let scale = Ctrl2::from(self.register(Reg::CTRL2)).a_fs();
        let raw = Self::to_raw(self.acceleration_mg, scale.lsb_per_g() as i64);
        self.set_i16s(Reg::AX_L, &raw);
        self.finish_sample(0b01, count_timestamp);
    }

    fn sample_gyro(&mut self) {
        let scale = Ctrl3::from(self.register(Reg::CTRL3)).g_fs();
        let raw = Self::to_raw(self.angular_rate_mdps, scale.lsb_per_dps() as i64);
        self.set_i16s(Reg::GX_L, &raw);
        self.finish_sample(0b10, true);
    }

    /// Updates temperature, status, timestamp and DRDY after a sample.
    fn finish_sample(&mut self, status: u8, count_timestamp: bool) {
        self.set_i16s(Reg::TEMP_L, &[self.temperature]);
        let status0 = self.register(Reg::STATUS0) | status;
        self.set(Reg::STATUS0, status0);

        if count_timestamp {
            let addr = usize::from(Reg::TIMESTAMP_LOW.addr());
            let mut bytes = [0; 4];
            bytes[..3].copy_from_slice(&self.regs[addr..addr + 3]);
            let next = (u32::from_le_bytes(bytes) + 1) & 0xFF_FFFF;
            self.regs[addr..addr + 3].copy_from_slice(&next.to_le_bytes()[..3]);
        }

        let fifo = FifoCtrl::from(self.register(Reg::FIFO_CTRL));
        let ctrl7 = Ctrl7::from(self.register(Reg::CTRL7));
        if fifo.fifo_mode() == FifoMode::Bypass && !ctrl7.drdy_dis() {
            self.drdy = true;
        }
    }

    /// `STATUSINT` as read from the bus.
    fn status_int(&self) -> u8 {
        let stored = self.register(Reg::STATUSINT) & 0x80;
        if Ctrl7::from(self.register(Reg::CTRL7)).sync_sample() {
            return stored;
        }

        stored | (u8::from(self.int1_pin()) << 1) | u8::from(self.int2_pin())
    }

    fn write_register(&mut self, addr: u8, value: u8) {
        if addr == Reg::RESET.addr() {
            if value == SOFT_RESET_COMMAND {
                self.reset();
            }

            return;
        }

        if !WRITABLE.contains(&addr) {
            return;
        }

        self.regs[usize::from(addr)] = value;
        if addr == Reg::CTRL9.addr() {
            // Any command completes at once; the acknowledgement ends the handshake.
            let done = Ctrl9Command::from_bits(value) != Ctrl9Command::Ack;
            self.set(Reg::STATUSINT, u8::from(done) << 7);
        }
    }

    /// Advances the register pointer if auto-increment is enabled.
    fn advance(&mut self) {
        if self.ctrl1().addr_ai() {
            self.pointer = self.pointer.wrapping_add(1);
        }
    }

    /// Writes a data byte at the register pointer and advances it.
    fn write_data(&mut self, byte: u8) {
        let addr = self.pointer;
        self.advance();
        self.write_register(addr, byte);
    }

    /// Reads the byte at the register pointer and advances it.
    fn read_data(&mut self) -> u8 {
        let addr = self.pointer;
        let value = match addr {
            a if a == Reg::STATUSINT.addr() => self.status_int(),
            a if a == Reg::STATUS0.addr() => {
                let value = self.register(Reg::STATUS0);
                self.set(Reg::STATUS0, 0);
                value
            }
            a if a == Reg::RESET.addr() => 0,
            a => self.regs[usize::from(a)],
        };

        self.advance();
        value
    }

    /// Starts an I²C transaction addressed to the model. Returns `true` to acknowledge.
    ///
    /// `is_read` selects a read (`true`) or write (`false`) transfer.
    pub fn i2c_start(&mut self, is_read: bool) -> bool {
        self.expect_pointer = !is_read;
        true
    }

    /// Writes a byte within an I²C write transfer. Returns `true` to acknowledge.
    ///
    /// The first byte sets the register pointer; subsequent bytes are written at the
    /// pointer. Writes to read-only registers are ignored.
    pub fn i2c_write(&mut self, byte: u8) -> bool {
        if self.expect_pointer {
            self.pointer = byte;
            self.expect_pointer = false;
        } else {
            self.write_data(byte);
        }

        true
    }

    /// Reads the byte at the register pointer within an I²C read transfer.
    ///
    /// Reading `STATUS0` clears the data-available flags.
    pub fn i2c_read(&mut self) -> u8 {
        self.read_data()
    }

    /// Starts an SPI transfer (chip select goes low).
    pub fn spi_select(&mut self) {
        self.spi = SpiState::Command;
    }

    /// Exchanges one byte within an SPI transfer and returns the byte shifted out.
    ///
    /// The first byte after [`Self::spi_select`] holds the read bit and the register
    /// address. The following bytes read or write at the register pointer.
    pub fn spi_transfer(&mut self, mosi: u8) -> u8 {
        match self.spi {
            SpiState::Command => {
                self.pointer = mosi & !SPI_READ;
                self.spi = if mosi & SPI_READ != 0 {
                    SpiState::Read
                } else {
                    SpiState::Write
                };

                0
            }
            SpiState::Read => self.read_data(),
            SpiState::Write => {
                self.write_data(mosi);
                0
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(model: &mut Qmi8658aModel, bytes: &[u8]) {
        assert!(model.i2c_start(false) && bytes.iter().all(|&b| model.i2c_write(b)));
    }

    fn read<const N: usize>(model: &mut Qmi8658aModel, reg: u8) -> [u8; N] {
        write(model, &[reg]);
        assert!(model.i2c_start(true));
        core::array::from_fn(|_| model.i2c_read())
    }

    /// Auto-increment, little-endian.
    fn configure_interface(model: &mut Qmi8658aModel) {
        write(model, &[0x02, 0x40]);
    }

    #[test]
    fn power_on_state() {
        let mut model = Qmi8658aModel::new();
        assert_eq!(read::<1>(&mut model, 0x00), [0x05]);
        assert_eq!(read::<1>(&mut model, 0x01), [MODEL_REVISION_ID]);
        assert_eq!(read::<1>(&mut model, 0x02), [0x20]);
        assert_eq!(read::<1>(&mut model, 0x4D), [0x80]);
    }

    #[test]
    fn burst_without_auto_increment_repeats_the_register() {
        let mut model = Qmi8658aModel::new();
        assert_eq!(read::<3>(&mut model, 0x00), [0x05; 3]);
        configure_interface(&mut model);
        assert_eq!(read::<3>(&mut model, 0x00), [0x05, MODEL_REVISION_ID, 0x40]);
    }

    #[test]
    fn soft_reset() {
        let mut model = Qmi8658aModel::new();
        configure_interface(&mut model);
        write(&mut model, &[0x08, 0x03]);
        write(&mut model, &[0x60, 0x0B]);
        assert_eq!(model.register(Reg::CTRL7), 0x03);
        write(&mut model, &[0x60, 0xB0]);
        assert_eq!(model.register(Reg::CTRL7), 0x00);
        assert_eq!(model.register(Reg::CTRL1), 0x20);
    }

    #[test]
    fn read_only_registers_ignore_writes() {
        let mut model = Qmi8658aModel::new();
        write(&mut model, &[0x00, 0x00]);
        write(&mut model, &[0x35, 0x12]);
        assert_eq!(model.register(Reg::WHO_AM_I), QMI8658A_WHO_AM_I);
        assert_eq!(model.register(Reg::AX_L), 0);
        assert_eq!(read::<1>(&mut model, 0x60), [0]);
    }

    #[test]
    fn six_dof_sampling() {
        let mut model = Qmi8658aModel::new();
        configure_interface(&mut model);
        model.set_acceleration_mg([1000, -500, 0]);
        model.set_angular_rate_mdps([-90_000, 0, 1_500]);
        model.set_temperature(25 * 256 + 128);
        // ±8 g at 56.05 Hz, ±512 dps at 56.05 Hz, both enabled.
        write(&mut model, &[0x03, 0x27]);
        write(&mut model, &[0x04, 0x57]);
        write(&mut model, &[0x08, 0x03]);

        model.tick(17_000);
        assert_eq!(read::<1>(&mut model, 0x2E), [0x00]);
        model.tick(1_000);
        assert_eq!(read::<1>(&mut model, 0x2E), [0x03]);
        assert_eq!(read::<1>(&mut model, 0x2E), [0x00]);
        assert_eq!(
            read::<2>(&mut model, 0x33),
            (25 * 256 + 128i16).to_le_bytes()
        );
        let mut expected = [0u8; 12];
        for (i, v) in [4096i16, -2048, 0, -5760, 0, 96].iter().enumerate() {
            expected[2 * i..2 * i + 2].copy_from_slice(&v.to_le_bytes());
        }
        assert_eq!(read::<12>(&mut model, 0x35), expected);
        assert_eq!(read::<3>(&mut model, 0x30), [1, 0, 0]);
    }

    #[test]
    fn accel_only_uses_the_oscillator_rate() {
        let mut model = Qmi8658aModel::new();
        // 62.5 Hz accelerometer only: 16 ms period.
        write(&mut model, &[0x03, 0x07]);
        write(&mut model, &[0x08, 0x01]);
        model.tick(15_999);
        assert_eq!(model.register(Reg::STATUS0), 0x00);
        model.tick(1);
        assert_eq!(model.register(Reg::STATUS0), 0x01);
        assert_eq!(model.register(Reg::TIMESTAMP_LOW), 1);
    }

    #[test]
    fn outputs_saturate() {
        let mut model = Qmi8658aModel::new();
        configure_interface(&mut model);
        model.set_acceleration_mg([3000, -3000, 0]);
        // ±2 g at 1 kHz, accelerometer only.
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x08, 0x01]);
        model.tick(1_000);
        assert_eq!(read::<4>(&mut model, 0x35), [0xFF, 0x7F, 0x00, 0x80]);
    }

    #[test]
    fn big_endian_by_default() {
        let mut model = Qmi8658aModel::new();
        model.set_acceleration_mg([0, 0, 1000]);
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x08, 0x01]);
        model.tick(1_000);
        // 16384 = 0x4000, high byte first.
        assert_eq!(model.register(Reg::AZ_L), 0x40);
        assert_eq!(model.register(Reg::AZ_H), 0x00);
    }

    #[test]
    fn drdy_on_int2_and_statusint() {
        let mut model = Qmi8658aModel::new();
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x08, 0x01]);
        model.tick(1_000);
        // INT2 in high-Z: neither the pin nor STATUSINT.bit0 rise.
        assert!(!model.int2_pin());
        assert_eq!(read::<1>(&mut model, 0x2D), [0x00]);

        write(&mut model, &[0x02, 0x10]);
        model.tick(1_000);
        assert!(model.int2_pin());
        assert_eq!(read::<1>(&mut model, 0x2D), [0x01]);
        model.tick(1);
        assert!(!model.int2_pin());

        // DRDY_DIS blocks the pulse.
        write(&mut model, &[0x08, 0x21]);
        model.tick(1_000);
        assert!(!model.int2_pin());
        assert!(!model.int1_pin());
    }

    #[test]
    fn ctrl9_handshake() {
        let mut model = Qmi8658aModel::new();
        write(&mut model, &[0x0A, 0x10]);
        assert_eq!(read::<1>(&mut model, 0x2D), [0x80]);
        write(&mut model, &[0x0A, 0x00]);
        assert_eq!(read::<1>(&mut model, 0x2D), [0x00]);
    }
}
