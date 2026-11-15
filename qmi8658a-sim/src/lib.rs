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
//! - the CTRL9 handshake (`STATUSINT.CmdDone`, on INT1 unless `CTRL8.bit7` is set) and the
//!   commands below,
//! - the FIFO: bypass, FIFO and stream mode, size, watermark, sample count, status flags,
//!   the FIFO interrupt, `CTRL_CMD_RST_FIFO` and `CTRL_CMD_REQ_FIFO` read mode,
//! - any-motion, no-motion and significant-motion detection on the acceleration slope,
//!   Wake on Motion, and the activity interrupt routing (`CTRL8.bit6`),
//! - the tap engine and the pedometer as far as their outputs: events come from
//!   [`Qmi8658aModel::tap`] and [`Qmi8658aModel::step`] instead of an algorithm,
//! - accelerometer and gyroscope self-test, with a settable response
//!   ([`Qmi8658aModel::set_self_test_response`]),
//! - calibration-on-demand, saving and applying gyroscope gains, and the host delta offsets.
//!
//! ## Not modelled
//!
//! SyncSample mode and the locking mechanism, the tap and pedometer algorithms and their
//! parameters (except `ped_sig_count`), the low-pass filters, low-power and snooze modes,
//! power-down, turn-on times, `CTRL_CMD_COPY_USID`, `CTRL_CMD_SET_RPU` and
//! `CTRL_CMD_AHB_CLOCK_GATING`. The bits can be written and read back, but do not change
//! the output.
//!
//! ## Assumptions
//!
//! The datasheet (13-52-25 Rev A) leaves these points open; the model fills them in as follows.
//!
//! - **`STATUS0` clearing**: "no updates since last read" does not say which read clears
//!   `aDA`/`gDA`. The model clears both flags when `STATUS0` is read.
//! - **`CTRL1.BE`**: the datasheet only says "read data big-endian". The model swaps the
//!   two bytes of each 16-bit temperature, acceleration and angular rate output, so that
//!   the high byte sits at the lower (`_L`) address. FIFO data always follows the
//!   low-byte-first pattern of section 8.9.
//! - **`STATUSINT` bits 0 and 1**: outside SyncSample mode they show "the same value" as
//!   INT2 and INT1 (datasheet 6.2). The model mirrors the driven pin level: with the pin
//!   in high-Z mode (`CTRL1.INT2_EN = 0`), bit 0 stays `0`. The self-test procedure
//!   (15.1, step 3) can be read either way.
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
//! - **CTRL9 timing**: the datasheet gives no command duration except about 1.5 s for
//!   calibration-on-demand (14.2). The model completes calibration-on-demand after
//!   [`COD_DURATION_US`] and every other command at once. After `CTRL_CMD_ACK`,
//!   `CmdDone` clears at once.
//! - **FIFO frames**: with both sensors enabled, the model writes one 12-byte frame per
//!   gyroscope sample with the latest accelerometer values. The datasheet requires both
//!   ODRs to match (8.2).
//! - **FIFO flags**: `FIFO_OVFLOW` is set when data is dropped and cleared only by
//!   `CTRL_CMD_RST_FIFO` and reset. `FIFO_DATA` reads `0` outside read mode. The FIFO
//!   interrupt follows `FIFO_WTM`.
//! - **`STATUS1`**: the datasheet only says that reading it clears the WoM bit (12.4). The
//!   model clears the tap, WoM and pedometer bits on read, and recomputes the any-,
//!   no- and significant-motion bits on every accelerometer sample from the current state.
//! - **Engine configuration**: sections 9.4, 10.3 and 11.2 say to configure "when
//!   accelerometer and gyroscope are disabled". The model ignores configuration commands
//!   while either sensor is enabled, but still completes the handshake.
//! - **`CAL4_H` set selector**: tables 34, 36 and 38 label the field `CAL4_H (7:4)`, the
//!   text says to write `0x01` or `0x02` to `CAL4_H`. The model takes the whole byte.
//! - **Motion windows**: `AnyMotionWindow` is listed as "5-bits fraction" with a resolution
//!   of one sample; the model treats it as an integer sample count.
//! - **Host delta offsets**: the formats (signed 4.12 and 11.5) are given, the units and
//!   sign are not. The model treats them as g and dps, subtracts them from the input and
//!   accumulates successive deltas.
//! - **Self-test**: the model completes the accelerometer test after 25 accelerometer-only
//!   ODR periods (15.1, at least 25 ms) and the gyroscope test after 400 ms (15.2).
//! - **Calibration-on-demand**: the model reports success with [`MODEL_GYRO_GAINS`],
//!   unless the gyroscope is enabled (`COD_STATUS = 0x03`) or a result was forced with
//!   [`Qmi8658aModel::set_cod_status`].

#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

mod bus;
mod engines;
mod fifo;
mod self_test;

pub use bus::{FakeDelay, FakeI2c, FakeSpi};

use engines::Engines;
use fifo::Fifo;
use qmi8658a_registers::{
    Ctrl1, Ctrl2, Ctrl3, Ctrl7, Ctrl8, Ctrl9Command, DEFAULT_DEVICE_ADDRESS, FifoCtrl, FifoMode,
    QMI8658A_WHO_AM_I, RESET_DONE, RegisterAddress as Reg, SOFT_RESET_COMMAND, SPI_READ,
};
use self_test::SelfTest;

/// The value the model returns for `REVISION_ID` (datasheet table 21).
pub const MODEL_REVISION_ID: u8 = 0x7C;

/// The time calibration-on-demand takes in the model, in microseconds (datasheet 14.2:
/// "about 1.5 seconds").
pub const COD_DURATION_US: u32 = 1_500_000;

/// The gyroscope gains the model reports after calibration-on-demand, for X, Y and Z.
pub const MODEL_GYRO_GAINS: [u16; 3] = [0x1234, 0x2345, 0x3456];

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
    /// The last raw accelerometer and gyroscope outputs, for FIFO frames.
    last_accel: [i16; 3],
    last_gyro: [i16; 3],
    /// The accumulated host delta offsets, in mg and mdps.
    accel_offset_mg: [i32; 3],
    gyro_offset_mdps: [i32; 3],
    /// The gyroscope gains in effect.
    gyro_gains: [u16; 3],
    /// The result calibration-on-demand reports, if forced.
    cod_status: Option<u8>,
    /// A CTRL9 command in progress, and the time it still takes.
    ctrl9: Option<(Ctrl9Command, u32)>,
    fifo: Fifo,
    engines: Engines,
    self_test: SelfTest,
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
            last_accel: [0; 3],
            last_gyro: [0; 3],
            accel_offset_mg: [0; 3],
            gyro_offset_mdps: [0; 3],
            gyro_gains: [0; 3],
            cod_status: None,
            ctrl9: None,
            fifo: Fifo::new(),
            engines: Engines::new(),
            self_test: SelfTest::new(),
            ticks: fake_bus::Micros::new(),
            spi: SpiState::Command,
        };

        model.reset();
        model
    }

    /// Resets all registers and internal state to their defaults, as by power-on or soft
    /// reset. Keeps the inputs and the settings made through the model's own setters.
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
        self.accel_offset_mg = [0; 3];
        self.gyro_offset_mdps = [0; 3];
        self.gyro_gains = [0; 3];
        self.ctrl9 = None;
        self.fifo = Fifo::new();
        self.engines.reset();
        self.self_test.reset();
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

    /// Forces the `COD_STATUS` value of the next calibration-on-demand. `0` is success.
    pub fn set_cod_status(&mut self, status: u8) {
        self.cod_status = Some(status);
    }

    /// The gyroscope gains in effect, for X, Y and Z: `0` after reset,
    /// [`MODEL_GYRO_GAINS`] after calibration-on-demand, or the applied gains.
    pub fn gyro_gains(&self) -> [u16; 3] {
        self.gyro_gains
    }

    /// The accumulated host delta offsets: acceleration in mg and angular rate in mdps.
    pub fn offsets(&self) -> ([i32; 3], [i32; 3]) {
        (self.accel_offset_mg, self.gyro_offset_mdps)
    }

    /// The level of the INT1 pin, if its output is enabled.
    pub fn int1_pin(&self) -> bool {
        self.ctrl1().int1_en() && self.int_line(false)
    }

    /// The level of the INT2 pin, if its output is enabled.
    pub fn int2_pin(&self) -> bool {
        self.ctrl1().int2_en() && self.int_line(true)
    }

    /// The internal signal of INT2 (`true`) or INT1 (`false`), before the output enable.
    fn int_line(&self, int2: bool) -> bool {
        if let Some(level) = self.engines.wom_line(int2) {
            return level;
        }

        let ctrl1 = self.ctrl1();
        let ctrl8 = Ctrl8::from(self.register(Reg::CTRL8));
        let fifo = self.fifo_watermark_hit() && ctrl1.fifo_int_sel() != int2;
        let activity = self.activity() && ctrl8.activity_int_sel() != int2;
        if int2 {
            self.drdy || fifo || activity || self.self_test.done()
        } else {
            let handshake =
                !ctrl8.ctrl9_handshake_type() && self.register(Reg::STATUSINT) & 0x80 != 0;
            handshake || fifo || activity
        }
    }

    /// Advances time by `us` microseconds and samples the enabled sensors at their output
    /// data rates. Ends a data-ready pulse from a previous call.
    pub fn tick(&mut self, us: u32) {
        self.drdy = false;
        self.tick_ctrl9(us);
        self.self_test_tick(us);
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
                self.sample_gyro(ctrl7.a_en());
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

    fn ctrl7(&self) -> Ctrl7 {
        Ctrl7::from(self.register(Reg::CTRL7))
    }

    fn set(&mut self, reg: Reg, value: u8) {
        self.regs[usize::from(reg.addr())] = value;
    }

    /// Reads a little-endian 16-bit value from a register pair.
    fn get_u16(&self, low: Reg) -> u16 {
        let addr = usize::from(low.addr());
        u16::from_le_bytes([self.regs[addr], self.regs[addr + 1]])
    }

    /// Writes little-endian 16-bit values to consecutive register pairs.
    fn set_le_u16s(&mut self, low: Reg, values: &[u16]) {
        let mut addr = usize::from(low.addr());
        for value in values {
            self.regs[addr..addr + 2].copy_from_slice(&value.to_le_bytes());
            addr += 2;
        }
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

    /// The acceleration after the host delta offset, in mg.
    fn measured_acceleration_mg(&self) -> [i32; 3] {
        core::array::from_fn(|i| self.acceleration_mg[i] - self.accel_offset_mg[i])
    }

    fn sample_accel(&mut self, count_timestamp: bool) {
        let scale = Ctrl2::from(self.register(Reg::CTRL2)).a_fs();
        let mg = self.measured_acceleration_mg();
        let raw = Self::to_raw(mg, scale.lsb_per_g() as i64);
        self.last_accel = raw;
        self.set_i16s(Reg::AX_L, &raw);
        self.engines_sample(mg);
        if !self.ctrl7().g_en() {
            self.fifo_push(&raw, None);
        }

        self.finish_sample(0b01, count_timestamp);
    }

    fn sample_gyro(&mut self, with_accel: bool) {
        let scale = Ctrl3::from(self.register(Reg::CTRL3)).g_fs();
        let mdps: [i32; 3] =
            core::array::from_fn(|i| self.angular_rate_mdps[i] - self.gyro_offset_mdps[i]);
        let raw = Self::to_raw(mdps, scale.lsb_per_dps() as i64);
        self.last_gyro = raw;
        self.set_i16s(Reg::GX_L, &raw);
        if with_accel {
            let accel = self.last_accel;
            self.fifo_push(&accel, Some(&raw));
        } else {
            self.fifo_push(&raw, None);
        }

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

        if !self.fifo_enabled() && !self.ctrl7().drdy_dis() {
            self.drdy = true;
        }
    }

    /// Whether the FIFO is in FIFO or stream mode.
    fn fifo_enabled(&self) -> bool {
        matches!(
            FifoCtrl::from(self.register(Reg::FIFO_CTRL)).fifo_mode(),
            FifoMode::Fifo | FifoMode::Stream
        )
    }

    /// `STATUSINT` as read from the bus.
    fn status_int(&self) -> u8 {
        let stored = self.register(Reg::STATUSINT) & 0x80;
        if self.ctrl7().sync_sample() {
            return stored;
        }

        stored | (u8::from(self.int1_pin()) << 1) | u8::from(self.int2_pin())
    }

    /// The eight CAL registers, `CAL1_L` to `CAL4_H`.
    fn cal(&self) -> [u8; 8] {
        let addr = usize::from(Reg::CAL1_L.addr());
        self.regs[addr..addr + 8]
            .try_into()
            .expect("eight CAL registers")
    }

    /// Starts a CTRL9 command, or ends the handshake on `CTRL_CMD_ACK`.
    fn start_ctrl9(&mut self, command: Ctrl9Command) {
        if command == Ctrl9Command::Ack {
            self.ctrl9 = None;
            self.set(Reg::STATUSINT, 0);
            return;
        }

        let duration = if command == Ctrl9Command::OnDemandCalibration {
            COD_DURATION_US
        } else {
            0
        };

        self.ctrl9 = Some((command, duration));
        self.tick_ctrl9(0);
    }

    /// Advances a CTRL9 command in progress and completes it when its time has passed.
    fn tick_ctrl9(&mut self, us: u32) {
        let Some((command, remaining)) = self.ctrl9 else {
            return;
        };

        if remaining > us {
            self.ctrl9 = Some((command, remaining - us));
            return;
        }

        self.ctrl9 = None;
        self.execute_ctrl9(command);
        self.set(Reg::STATUSINT, 0x80);
    }

    /// Carries out a CTRL9 command.
    fn execute_ctrl9(&mut self, command: Ctrl9Command) {
        let cal = self.cal();
        let sensors_off = !self.ctrl7().a_en() && !self.ctrl7().g_en();
        match command {
            Ctrl9Command::ResetFifo => self.fifo_reset(),
            Ctrl9Command::RequestFifo => {
                let ctrl = FifoCtrl::from(self.register(Reg::FIFO_CTRL)).with_fifo_rd_mode(true);
                self.set(Reg::FIFO_CTRL, ctrl.into());
            }
            Ctrl9Command::WriteWomSetting => self.configure_wom(cal),
            Ctrl9Command::ConfigureMotion if sensors_off => self.engines.configure_motion(cal),
            Ctrl9Command::ConfigureTap if sensors_off => self.engines.configure_tap(cal),
            Ctrl9Command::ConfigurePedometer if sensors_off => {
                self.engines.configure_pedometer(cal)
            }
            Ctrl9Command::ResetPedometer => self.set_step_count(0),
            Ctrl9Command::AccelHostDeltaOffset => {
                for (axis, offset) in self.accel_offset_mg.iter_mut().enumerate() {
                    let raw = i16::from_le_bytes([cal[2 * axis], cal[2 * axis + 1]]);
                    *offset += i32::from(raw) * 1000 / 4096;
                }
            }
            Ctrl9Command::GyroHostDeltaOffset => {
                for (axis, offset) in self.gyro_offset_mdps.iter_mut().enumerate() {
                    let raw = i16::from_le_bytes([cal[2 * axis], cal[2 * axis + 1]]);
                    *offset += i32::from(raw) * 1000 / 32;
                }
            }
            Ctrl9Command::OnDemandCalibration => {
                let status = if self.ctrl7().g_en() {
                    0x03
                } else {
                    self.cod_status.unwrap_or(0)
                };

                self.set(Reg::COD_STATUS, status);
                if status == 0 {
                    self.gyro_gains = MODEL_GYRO_GAINS;
                    self.set_le_u16s(Reg::DVX_L, &MODEL_GYRO_GAINS);
                }
            }
            Ctrl9Command::ApplyGyroGains => {
                self.gyro_gains = [Reg::CAL1_L, Reg::CAL2_L, Reg::CAL3_L].map(|r| self.get_u16(r));
            }
            _ => {}
        }
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

        let previous = self.regs[usize::from(addr)];
        self.regs[usize::from(addr)] = value;
        match addr {
            a if a == Reg::CTRL2.addr() || a == Reg::CTRL3.addr() => {
                self.self_test_write(addr, previous, value)
            }
            a if a == Reg::CTRL8.addr() => {
                // Setting Pedo_EN from 0 resets the step count (datasheet 11.6).
                if !Ctrl8::from(previous).pedo_en() && Ctrl8::from(value).pedo_en() {
                    self.set_step_count(0);
                }
            }
            a if a == Reg::CTRL9.addr() => self.start_ctrl9(Ctrl9Command::from_bits(value)),
            a if a == Reg::FIFO_CTRL.addr() || a == Reg::FIFO_WTM_TH.addr() => {
                self.fifo_update_registers()
            }
            _ => {}
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
            a if a == Reg::STATUS1.addr() => self.read_status1(),
            a if a == Reg::FIFO_DATA.addr() => self.fifo_read(),
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
    /// Reading `STATUS0` clears the data-available flags, reading `STATUS1` the latched
    /// event flags, and reading `FIFO_DATA` in read mode removes a byte from the FIFO.
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

    /// Runs a CTRL9 command with the given CAL registers and acknowledges it.
    fn command(model: &mut Qmi8658aModel, cal: [u8; 8], command: u8) {
        configure_interface(model);
        let mut bytes = [0x0B; 9];
        bytes[1..].copy_from_slice(&cal);
        write(model, &bytes);
        write(model, &[0x0A, command]);
        assert_eq!(read::<1>(model, 0x2D)[0] & 0x80, 0x80);
        write(model, &[0x0A, 0x00]);
    }

    #[test]
    fn ctrl9_handshake_on_int1() {
        let mut model = Qmi8658aModel::new();
        write(&mut model, &[0x02, 0x08]);
        write(&mut model, &[0x0A, 0x04]);
        assert!(model.int1_pin());
        write(&mut model, &[0x0A, 0x00]);
        assert!(!model.int1_pin());
        // CTRL8.bit7: CmdDone only.
        write(&mut model, &[0x09, 0x80]);
        write(&mut model, &[0x0A, 0x04]);
        assert!(!model.int1_pin());
        assert_eq!(read::<1>(&mut model, 0x2D), [0x80]);
    }

    #[test]
    fn fifo_fills_and_drains() {
        let mut model = Qmi8658aModel::new();
        configure_interface(&mut model);
        model.set_acceleration_mg([1000, 0, -1000]);
        // Watermark 2 samples, FIFO mode, 16 samples, accelerometer only at 1 kHz.
        write(&mut model, &[0x13, 2]);
        write(&mut model, &[0x14, 0x01]);
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x08, 0x01]);
        model.tick(1_000);
        assert!(!model.int2_pin());
        assert_eq!(read::<2>(&mut model, 0x15), [3, 0x10]);
        write(&mut model, &[0x02, 0x50]);
        model.tick(1_000);
        assert_eq!(read::<2>(&mut model, 0x15), [6, 0x50]);
        // The watermark drives INT2; DRDY is off with the FIFO enabled.
        assert!(model.int2_pin());

        // Outside read mode, FIFO_DATA reads 0 and keeps the data.
        assert_eq!(read::<1>(&mut model, 0x17), [0]);
        command(&mut model, [0; 8], 0x05);
        assert_eq!(model.register(Reg::FIFO_CTRL), 0x81);
        // No auto-increment for the drain.
        write(&mut model, &[0x02, 0x10]);
        let data = read::<12>(&mut model, 0x17);
        assert_eq!(data[..6], [0x00, 0x40, 0x00, 0x00, 0x00, 0xC0]);
        assert_eq!(data[..6], data[6..]);
        assert_eq!(model.register(Reg::FIFO_STATUS), 0x00);
        assert!(!model.int2_pin());
    }

    #[test]
    fn fifo_read_mode_drops_new_data() {
        let mut model = Qmi8658aModel::new();
        write(&mut model, &[0x14, 0x81]);
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x08, 0x01]);
        model.tick(5_000);
        assert_eq!(model.register(Reg::FIFO_SMPL_CNT), 0);
        write(&mut model, &[0x14, 0x01]);
        model.tick(1_000);
        assert_eq!(model.register(Reg::FIFO_SMPL_CNT), 3);
    }

    #[test]
    fn fifo_full_overflow_and_stream() {
        let mut model = Qmi8658aModel::new();
        // FIFO mode, 16 samples, 6DOF at 896.8 Hz: 12-byte frames.
        write(&mut model, &[0x14, 0x01]);
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x04, 0x03]);
        write(&mut model, &[0x08, 0x03]);
        model.tick(20_000);
        assert_eq!(model.register(Reg::FIFO_SMPL_CNT), 96);
        assert_eq!(model.register(Reg::FIFO_STATUS), 0xB0);

        command(&mut model, [0; 8], 0x04);
        assert_eq!(model.register(Reg::FIFO_STATUS), 0x00);
        assert_eq!(model.register(Reg::FIFO_SMPL_CNT), 0);

        // Stream mode keeps the newest samples.
        write(&mut model, &[0x14, 0x02]);
        model.set_angular_rate_mdps([0, 0, 1_000]);
        model.tick(20_000);
        assert_eq!(model.register(Reg::FIFO_STATUS), 0xB0);
        write(&mut model, &[0x14, 0x82]);
        write(&mut model, &[0x02, 0x00]);
        assert_eq!(read::<12>(&mut model, 0x17)[10..], [0x00, 0x08]);
    }

    #[test]
    fn any_and_no_motion() {
        let mut model = Qmi8658aModel::new();
        // Any-motion on X above 0.5 g for 2 samples; no-motion on X below 0.0625 g for 3.
        command(&mut model, [16, 0, 0, 2, 0, 0, 0x11, 0x01], 0x0E);
        command(&mut model, [2, 3, 0, 0, 0, 0, 0, 0x02], 0x0E);
        assert_eq!(model.motion_parameters()[1][1], 3);
        write(&mut model, &[0x09, 0x06]);
        write(&mut model, &[0x02, 0x58]);
        write(&mut model, &[0x03, 0x03]);
        // Accelerometer on, DRDY off.
        write(&mut model, &[0x08, 0x21]);

        for x in [0, 600, 0, 600] {
            model.set_acceleration_mg([x, 0, 0]);
            model.tick(1_000);
        }
        assert_eq!(model.register(Reg::STATUS1), 0x20);
        assert!(model.int2_pin());
        model.tick(3_000);
        assert_eq!(model.register(Reg::STATUS1), 0x40);
        // Activity on INT1 instead.
        write(&mut model, &[0x09, 0x46]);
        assert!(model.int1_pin() && !model.int2_pin());
    }

    #[test]
    fn engine_configuration_needs_sensors_off() {
        let mut model = Qmi8658aModel::new();
        write(&mut model, &[0x08, 0x01]);
        command(&mut model, [1, 2, 3, 4, 5, 6, 7, 0x01], 0x0C);
        assert_eq!(model.tap_parameters(), [[0; 8]; 2]);
        write(&mut model, &[0x08, 0x00]);
        command(&mut model, [1, 2, 3, 4, 5, 6, 7, 0x01], 0x0C);
        assert_eq!(model.tap_parameters()[0], [1, 2, 3, 4, 5, 6, 7, 0x01]);
    }

    #[test]
    fn significant_motion() {
        let mut model = Qmi8658aModel::new();
        // Any-motion on X above 0.5 g, no-motion below 0.0625 g for 10 samples; wait 2,
        // confirm 5.
        command(&mut model, [16, 0, 0, 2, 0, 0, 0x11, 0x01], 0x0E);
        command(&mut model, [1, 10, 2, 0, 5, 0, 0, 0x02], 0x0E);
        write(&mut model, &[0x09, 0x0E]);
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x08, 0x01]);
        for x in [0, 600, 600, 600, 0] {
            model.set_acceleration_mg([x, 0, 0]);
            model.tick(1_000);
        }
        assert_eq!(model.register(Reg::STATUS1) & 0x80, 0x80);
        model.tick(10_000);
        assert_eq!(model.register(Reg::STATUS1) & 0x80, 0x00);
    }

    #[test]
    fn wake_on_motion() {
        let mut model = Qmi8658aModel::new();
        // 100 mg, INT2 starting high, 1 sample blanking; 3 Hz low-power.
        command(&mut model, [100, 0xC1, 0, 0, 0, 0, 0, 0], 0x08);
        write(&mut model, &[0x02, 0x50]);
        write(&mut model, &[0x03, 0x0F]);
        write(&mut model, &[0x08, 0x01]);
        assert!(model.int2_pin());
        model.set_acceleration_mg([0, 500, 0]);
        model.tick(1_000_000);
        assert!(model.int2_pin());
        model.set_acceleration_mg([0, 0, 0]);
        model.tick(334_000);
        assert!(!model.int2_pin());
        assert_eq!(read::<1>(&mut model, 0x2F), [0x04]);
        assert!(model.int2_pin());
        assert_eq!(read::<1>(&mut model, 0x2F), [0x00]);

        // Threshold 0 leaves WoM mode.
        write(&mut model, &[0x08, 0x00]);
        command(&mut model, [0; 8], 0x08);
        assert!(!model.int2_pin());
    }

    #[test]
    fn tap_and_pedometer() {
        let mut model = Qmi8658aModel::new();
        command(&mut model, [0, 0, 0, 0, 0, 4, 0, 0x02], 0x0D);
        write(&mut model, &[0x08, 0x01]);
        model.tap(1, false, false);
        model.step(4);
        assert_eq!(model.register(Reg::STATUS1), 0x00);

        write(&mut model, &[0x09, 0x11]);
        model.tap(3, true, true);
        assert_eq!(model.register(Reg::TAP_STATUS), 0xB2);
        model.step(9);
        assert_eq!(read::<3>(&mut model, 0x5A), [8, 0, 0]);
        assert_eq!(read::<1>(&mut model, 0x2F), [0x12]);
        assert_eq!(read::<1>(&mut model, 0x2F), [0x00]);

        command(&mut model, [0; 8], 0x0F);
        assert_eq!(read::<3>(&mut model, 0x5A), [0, 0, 0]);
        model.step(8);
        // Setting Pedo_EN from 0 resets the count.
        write(&mut model, &[0x09, 0x01]);
        write(&mut model, &[0x09, 0x11]);
        assert_eq!(read::<3>(&mut model, 0x5A), [0, 0, 0]);
    }

    #[test]
    fn self_test() {
        let mut model = Qmi8658aModel::new();
        configure_interface(&mut model);
        write(&mut model, &[0x02, 0x50]);
        model.set_self_test_response([300, -300, 100], [400_000, 0, -1_000]);
        // Accelerometer at 1 kHz: 25 ms.
        write(&mut model, &[0x03, 0x83]);
        model.tick(24_000);
        assert_eq!(read::<1>(&mut model, 0x2D), [0x00]);
        model.tick(1_000);
        assert_eq!(read::<1>(&mut model, 0x2D), [0x01]);
        write(&mut model, &[0x03, 0x03]);
        assert_eq!(read::<1>(&mut model, 0x2D), [0x00]);
        let expected: [u8; 6] = [0x66, 0x02, 0x9A, 0xFD, 0xCC, 0x00];
        assert_eq!(read::<6>(&mut model, 0x51), expected);

        write(&mut model, &[0x04, 0x80]);
        model.tick(400_000);
        assert!(model.int2_pin());
        write(&mut model, &[0x04, 0x00]);
        assert_eq!(read::<6>(&mut model, 0x51), [0x00, 0x19, 0, 0, 0xF0, 0xFF]);
    }

    #[test]
    fn calibration_on_demand() {
        let mut model = Qmi8658aModel::new();
        configure_interface(&mut model);
        write(&mut model, &[0x0A, 0xA2]);
        model.tick(1_499_000);
        assert_eq!(read::<1>(&mut model, 0x2D), [0x00]);
        model.tick(1_000);
        assert_eq!(read::<1>(&mut model, 0x2D), [0x80]);
        write(&mut model, &[0x0A, 0x00]);
        assert_eq!(read::<1>(&mut model, 0x46), [0x00]);
        assert_eq!(read::<2>(&mut model, 0x51), 0x1234u16.to_le_bytes());
        assert_eq!(model.gyro_gains(), MODEL_GYRO_GAINS);

        command(&mut model, [1, 0, 2, 0, 3, 0, 0, 0], 0xAA);
        assert_eq!(model.gyro_gains(), [1, 2, 3]);

        write(&mut model, &[0x08, 0x02]);
        write(&mut model, &[0x0A, 0xA2]);
        model.tick(COD_DURATION_US);
        assert_eq!(read::<1>(&mut model, 0x46), [0x03]);
    }

    #[test]
    fn host_delta_offsets() {
        let mut model = Qmi8658aModel::new();
        // 0.5 g on X; 1 dps on Z, twice.
        command(&mut model, [0x00, 0x08, 0, 0, 0, 0, 0, 0], 0x09);
        command(&mut model, [0, 0, 0, 0, 32, 0, 0, 0], 0x0A);
        command(&mut model, [0, 0, 0, 0, 32, 0, 0, 0], 0x0A);
        assert_eq!(model.offsets(), ([500, 0, 0], [0, 0, 2_000]));
        model.set_acceleration_mg([1000, 0, 0]);
        write(&mut model, &[0x03, 0x03]);
        write(&mut model, &[0x08, 0x01]);
        model.tick(1_000);
        assert_eq!(read::<2>(&mut model, 0x35), 8192i16.to_le_bytes());
    }
}
