//! The user interface registers (datasheet table 19).

use crate::types::{
    AccelFullScale, AccelOdr, Ctrl9Command, FifoMode, FifoSize, GyroFullScale, GyroOdr, LowPassMode,
};
use bitfield_struct::bitfield;

/// Register addresses (datasheet table 19).
///
/// The reserved addresses `0x05` and `0x07` are not listed.
#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RegisterAddress {
    /// See [`WhoAmI`]. Read-only.
    WHO_AM_I = 0x00,
    /// See [`RevisionId`]. Read-only.
    REVISION_ID = 0x01,
    /// See [`Ctrl1`]. Read-write.
    CTRL1 = 0x02,
    /// See [`Ctrl2`]. Read-write.
    CTRL2 = 0x03,
    /// See [`Ctrl3`]. Read-write.
    CTRL3 = 0x04,
    /// See [`Ctrl5`]. Read-write.
    CTRL5 = 0x06,
    /// See [`Ctrl7`]. Read-write.
    CTRL7 = 0x08,
    /// See [`Ctrl8`]. Read-write.
    CTRL8 = 0x09,
    /// See [`Ctrl9`]. Read-write.
    CTRL9 = 0x0A,
    /// See [`Cal1L`]. Read-write.
    CAL1_L = 0x0B,
    /// See [`Cal1H`]. Read-write.
    CAL1_H = 0x0C,
    /// See [`Cal2L`]. Read-write.
    CAL2_L = 0x0D,
    /// See [`Cal2H`]. Read-write.
    CAL2_H = 0x0E,
    /// See [`Cal3L`]. Read-write.
    CAL3_L = 0x0F,
    /// See [`Cal3H`]. Read-write.
    CAL3_H = 0x10,
    /// See [`Cal4L`]. Read-write.
    CAL4_L = 0x11,
    /// See [`Cal4H`]. Read-write.
    CAL4_H = 0x12,
    /// See [`FifoWtmTh`]. Read-write.
    FIFO_WTM_TH = 0x13,
    /// See [`FifoCtrl`]. Read-write.
    FIFO_CTRL = 0x14,
    /// See [`FifoSmplCnt`]. Read-only.
    FIFO_SMPL_CNT = 0x15,
    /// See [`FifoStatus`]. Read-only.
    FIFO_STATUS = 0x16,
    /// See [`FifoData`]. Read-only.
    FIFO_DATA = 0x17,
    /// See [`StatusInt`]. Read-only.
    STATUSINT = 0x2D,
    /// See [`Status0`]. Read-only.
    STATUS0 = 0x2E,
    /// See [`Status1`]. Read-only.
    STATUS1 = 0x2F,
    /// See [`TimestampLow`]. Read-only.
    TIMESTAMP_LOW = 0x30,
    /// See [`TimestampMid`]. Read-only.
    TIMESTAMP_MID = 0x31,
    /// See [`TimestampHigh`]. Read-only.
    TIMESTAMP_HIGH = 0x32,
    /// See [`TempL`]. Read-only.
    TEMP_L = 0x33,
    /// See [`TempH`]. Read-only.
    TEMP_H = 0x34,
    /// See [`AxL`]. Read-only.
    AX_L = 0x35,
    /// See [`AxH`]. Read-only.
    AX_H = 0x36,
    /// See [`AyL`]. Read-only.
    AY_L = 0x37,
    /// See [`AyH`]. Read-only.
    AY_H = 0x38,
    /// See [`AzL`]. Read-only.
    AZ_L = 0x39,
    /// See [`AzH`]. Read-only.
    AZ_H = 0x3A,
    /// See [`GxL`]. Read-only.
    GX_L = 0x3B,
    /// See [`GxH`]. Read-only.
    GX_H = 0x3C,
    /// See [`GyL`]. Read-only.
    GY_L = 0x3D,
    /// See [`GyH`]. Read-only.
    GY_H = 0x3E,
    /// See [`GzL`]. Read-only.
    GZ_L = 0x3F,
    /// See [`GzH`]. Read-only.
    GZ_H = 0x40,
    /// See [`CodStatus`]. Read-only.
    COD_STATUS = 0x46,
    /// See [`DqwL`]. Read-only.
    DQW_L = 0x49,
    /// See [`DqwH`]. Read-only.
    DQW_H = 0x4A,
    /// See [`DqxL`]. Read-only.
    DQX_L = 0x4B,
    /// See [`DqxH`]. Read-only.
    DQX_H = 0x4C,
    /// See [`DqyL`]. Read-only.
    DQY_L = 0x4D,
    /// See [`DqyH`]. Read-only.
    DQY_H = 0x4E,
    /// See [`DqzL`]. Read-only.
    DQZ_L = 0x4F,
    /// See [`DqzH`]. Read-only.
    DQZ_H = 0x50,
    /// See [`DvxL`]. Read-only.
    DVX_L = 0x51,
    /// See [`DvxH`]. Read-only.
    DVX_H = 0x52,
    /// See [`DvyL`]. Read-only.
    DVY_L = 0x53,
    /// See [`DvyH`]. Read-only.
    DVY_H = 0x54,
    /// See [`DvzL`]. Read-only.
    DVZ_L = 0x55,
    /// See [`DvzH`]. Read-only.
    DVZ_H = 0x56,
    /// See [`TapStatus`]. Read-only.
    TAP_STATUS = 0x59,
    /// See [`StepCntLow`]. Read-only.
    STEP_CNT_LOW = 0x5A,
    /// See [`StepCntMid`]. Read-only.
    STEP_CNT_MID = 0x5B,
    /// See [`StepCntHigh`]. Read-only.
    STEP_CNT_HIGH = 0x5C,
    /// See [`Reset`]. Write-only.
    RESET = 0x60,
}

impl RegisterAddress {
    /// Returns the address of a register.
    pub const fn addr(&self) -> u8 {
        *self as u8
    }
}

impl From<RegisterAddress> for u8 {
    fn from(value: RegisterAddress) -> Self {
        value.addr()
    }
}

/// Declares a register that holds one byte of a multi-byte value, or a plain byte.
macro_rules! byte_register {
    ($(#[$meta:meta])* $type:ident, $addr:expr, $access:ident, $macro:ident) => {
        $(#[$meta])*
        #[bitfield(u8)]
        #[derive(PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub struct $type {
            /// The register value.
            #[bits(8, access = $access)]
            pub value: u8,
        }

        $macro!($type, $addr);
    };
}

/// [`WHO_AM_I`](RegisterAddress::WHO_AM_I) (00h): device identifier (datasheet table 21).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WhoAmI {
    /// The identification value. Always [`QMI8658A_WHO_AM_I`](crate::QMI8658A_WHO_AM_I).
    #[bits(8, access = RO, default = crate::QMI8658A_WHO_AM_I)]
    pub ident: u8,
}

readable_register!(WhoAmI, RegisterAddress::WHO_AM_I);

byte_register!(
    /// [`REVISION_ID`](RegisterAddress::REVISION_ID) (01h): device revision ID.
    ///
    /// The datasheet gives two different values: `0x68` in the register overview (table 19)
    /// and `0x7C` in the register description (table 21).
    RevisionId, RegisterAddress::REVISION_ID, RO, readable_register
);

/// [`CTRL1`](RegisterAddress::CTRL1) (02h): serial interface and sensor enable
/// (datasheet table 22). Defaults to `0x20`.
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl1 {
    /// Disables the internal high-speed oscillator (power-down mode, datasheet 7.1).
    #[bits(1, access = RW)]
    pub sensor_disable: bool,

    #[bits(1)]
    __: u8,

    /// Routes the FIFO interrupt to INT1 (`true`) or INT2 (`false`).
    #[bits(1, access = RW)]
    pub fifo_int_sel: bool,

    /// Enables the INT1 output (push-pull). If `false`, INT1 is high-Z.
    #[bits(1, access = RW)]
    pub int1_en: bool,

    /// Enables the INT2 output (push-pull). If `false`, INT2 is high-Z.
    #[bits(1, access = RW)]
    pub int2_en: bool,

    /// Read data big-endian (`true`, the default) or little-endian (`false`).
    #[bits(1, access = RW, default = true)]
    pub be: bool,

    /// Address auto-increment on burst reads and writes. If `false` (the default), a burst
    /// accesses the same register over and over (datasheet 16.1).
    #[bits(1, access = RW)]
    pub addr_ai: bool,

    /// Selects 3-wire (`true`) or 4-wire (`false`, the default) SPI.
    #[bits(1, access = RW)]
    pub sim: bool,
}

writable_register!(Ctrl1, RegisterAddress::CTRL1);

/// [`CTRL2`](RegisterAddress::CTRL2) (03h): accelerometer output data rate, full scale and
/// self-test (datasheet table 22).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl2 {
    /// Output data rate.
    #[bits(4, access = RW, default = AccelOdr::Hz7174)]
    pub a_odr: AccelOdr,

    /// Full scale.
    #[bits(3, access = RW, default = AccelFullScale::G2)]
    pub a_fs: AccelFullScale,

    /// Enables the accelerometer self-test.
    #[bits(1, access = RW)]
    pub a_st: bool,
}

writable_register!(Ctrl2, RegisterAddress::CTRL2);

/// [`CTRL3`](RegisterAddress::CTRL3) (04h): gyroscope output data rate, full scale and
/// self-test (datasheet table 22).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl3 {
    /// Output data rate.
    #[bits(4, access = RW, default = GyroOdr::Hz7174)]
    pub g_odr: GyroOdr,

    /// Full scale.
    #[bits(3, access = RW, default = GyroFullScale::Dps16)]
    pub g_fs: GyroFullScale,

    /// Enables the gyroscope self-test.
    #[bits(1, access = RW)]
    pub g_st: bool,
}

writable_register!(Ctrl3, RegisterAddress::CTRL3);

/// [`CTRL5`](RegisterAddress::CTRL5) (06h): low-pass filters (datasheet table 22).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl5 {
    /// Enables the accelerometer low-pass filter with [`Ctrl5::a_lpf_mode`].
    #[bits(1, access = RW)]
    pub a_lpf_en: bool,

    /// Accelerometer low-pass filter bandwidth.
    #[bits(2, access = RW, default = LowPassMode::Percent2_66)]
    pub a_lpf_mode: LowPassMode,

    #[bits(1)]
    __: u8,

    /// Enables the gyroscope low-pass filter with [`Ctrl5::g_lpf_mode`].
    #[bits(1, access = RW)]
    pub g_lpf_en: bool,

    /// Gyroscope low-pass filter bandwidth.
    #[bits(2, access = RW, default = LowPassMode::Percent2_66)]
    pub g_lpf_mode: LowPassMode,

    #[bits(1)]
    __: u8,
}

writable_register!(Ctrl5, RegisterAddress::CTRL5);

/// [`CTRL7`](RegisterAddress::CTRL7) (08h): sensor enable and data reads (datasheet table 22).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl7 {
    /// Enables the accelerometer.
    #[bits(1, access = RW)]
    pub a_en: bool,

    /// Enables the gyroscope.
    #[bits(1, access = RW)]
    pub g_en: bool,

    #[bits(2)]
    __: u8,

    /// Gyroscope snooze mode: drive only, no data. Effective only with [`Ctrl7::g_en`].
    #[bits(1, access = RW)]
    pub g_sn: bool,

    /// Blocks the data-ready signal (DRDY) from the INT2 pin.
    #[bits(1, access = RW)]
    pub drdy_dis: bool,

    #[bits(1)]
    __: u8,

    /// SyncSample mode: locks the data registers while they are read (datasheet 6.1, 13).
    #[bits(1, access = RW)]
    pub sync_sample: bool,
}

writable_register!(Ctrl7, RegisterAddress::CTRL7);

/// [`CTRL8`](RegisterAddress::CTRL8) (09h): motion detection control (datasheet table 22).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl8 {
    /// Enables the tap engine.
    #[bits(1, access = RW)]
    pub tap_en: bool,

    /// Enables the any-motion engine.
    #[bits(1, access = RW)]
    pub any_motion_en: bool,

    /// Enables the no-motion engine.
    #[bits(1, access = RW)]
    pub no_motion_en: bool,

    /// Enables the significant-motion engine.
    #[bits(1, access = RW)]
    pub sig_motion_en: bool,

    /// Enables the pedometer engine.
    #[bits(1, access = RW)]
    pub pedo_en: bool,

    #[bits(1)]
    __: u8,

    /// Routes the activity detection interrupts (any-, no-, significant-motion, pedometer,
    /// tap) to INT1 (`true`) or INT2 (`false`).
    #[bits(1, access = RW)]
    pub activity_int_sel: bool,

    /// Signals CTRL9 completion on [`StatusInt::cmd_done`] only (`true`), or also on INT1
    /// (`false`).
    #[bits(1, access = RW)]
    pub ctrl9_handshake_type: bool,
}

writable_register!(Ctrl8, RegisterAddress::CTRL8);

/// [`CTRL9`](RegisterAddress::CTRL9) (0Ah): host commands (datasheet 5.10).
///
/// After writing a command, wait for [`StatusInt::cmd_done`], then write
/// [`Ctrl9Command::Ack`].
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl9 {
    /// The command.
    #[bits(8, access = RW, default = Ctrl9Command::Ack)]
    pub cmd: Ctrl9Command,
}

writable_register!(Ctrl9, RegisterAddress::CTRL9);

byte_register!(
    /// [`FIFO_WTM_TH`](RegisterAddress::FIFO_WTM_TH) (13h): FIFO watermark, in samples.
    ///
    /// A non-zero value enables the watermark interrupt (datasheet 8.5).
    FifoWtmTh, RegisterAddress::FIFO_WTM_TH, RW, writable_register
);

/// [`FIFO_CTRL`](RegisterAddress::FIFO_CTRL) (14h): FIFO setup (datasheet table 23).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoCtrl {
    /// FIFO mode.
    #[bits(2, access = RW, default = FifoMode::Bypass)]
    pub fifo_mode: FifoMode,

    /// FIFO size.
    #[bits(2, access = RW, default = FifoSize::Samples16)]
    pub fifo_size: FifoSize,

    #[bits(3)]
    __: u8,

    /// FIFO read mode. Set by [`Ctrl9Command::RequestFifo`]; the host clears it after
    /// reading so that the FIFO fills again.
    #[bits(1, access = RW)]
    pub fifo_rd_mode: bool,
}

writable_register!(FifoCtrl, RegisterAddress::FIFO_CTRL);

byte_register!(
    /// [`FIFO_SMPL_CNT`](RegisterAddress::FIFO_SMPL_CNT) (15h): FIFO sample count, bits 7:0.
    ///
    /// The count is in words of two bytes; bits 9:8 are in [`FifoStatus::fifo_smpl_cnt_msb`].
    FifoSmplCnt, RegisterAddress::FIFO_SMPL_CNT, RO, readable_register
);

/// [`FIFO_STATUS`](RegisterAddress::FIFO_STATUS) (16h): FIFO status (datasheet table 23).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoStatus {
    /// FIFO sample count, bits 9:8, in words of two bytes.
    #[bits(2, access = RO)]
    pub fifo_smpl_cnt_msb: u8,

    #[bits(2)]
    __: u8,

    /// The FIFO is not empty.
    #[bits(1, access = RO)]
    pub fifo_not_empty: bool,

    /// Data was dropped because the FIFO overflowed.
    #[bits(1, access = RO)]
    pub fifo_ovflow: bool,

    /// The FIFO level reached the watermark.
    #[bits(1, access = RO)]
    pub fifo_wtm: bool,

    /// The FIFO is full.
    #[bits(1, access = RO)]
    pub fifo_full: bool,
}

readable_register!(FifoStatus, RegisterAddress::FIFO_STATUS);

byte_register!(
    /// [`FIFO_DATA`](RegisterAddress::FIFO_DATA) (17h): FIFO data output.
    ///
    /// Every read removes a byte from the FIFO (datasheet 8.8). To drain several bytes in
    /// one burst, the register address must not advance: see [`Ctrl1::addr_ai`].
    FifoData, RegisterAddress::FIFO_DATA, RO, readable_register
);

/// [`STATUSINT`](RegisterAddress::STATUSINT) (2Dh): sensor data available and lock, CTRL9
/// completion (datasheet table 24).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusInt {
    /// With [`Ctrl7::sync_sample`]: sensor data is available. Otherwise: the level of INT2
    /// (datasheet 6.2).
    #[bits(1, access = RO)]
    pub avail: bool,

    /// With [`Ctrl7::sync_sample`]: sensor data is locked. Otherwise: the level of INT1
    /// (datasheet 6.2).
    #[bits(1, access = RO)]
    pub locked: bool,

    #[bits(5)]
    __: u8,

    /// The CTRL9 command is done.
    #[bits(1, access = RO)]
    pub cmd_done: bool,
}

readable_register!(StatusInt, RegisterAddress::STATUSINT);

/// [`STATUS0`](RegisterAddress::STATUS0) (2Eh): output data available (datasheet table 24).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status0 {
    /// New accelerometer data is available.
    #[bits(1, access = RO)]
    pub a_da: bool,

    /// New gyroscope data is available.
    #[bits(1, access = RO)]
    pub g_da: bool,

    #[bits(6)]
    __: u8,
}

readable_register!(Status0, RegisterAddress::STATUS0);

/// [`STATUS1`](RegisterAddress::STATUS1) (2Fh): motion events (datasheet table 24).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status1 {
    #[bits(1)]
    __: u8,

    /// A tap was detected.
    #[bits(1, access = RO)]
    pub tap: bool,

    /// Wake on Motion was detected.
    #[bits(1, access = RO)]
    pub wom: bool,

    #[bits(1)]
    __: u8,

    /// A step was detected.
    #[bits(1, access = RO)]
    pub pedometer: bool,

    /// Any-motion was detected.
    #[bits(1, access = RO)]
    pub any_motion: bool,

    /// No-motion was detected.
    #[bits(1, access = RO)]
    pub no_motion: bool,

    /// Significant motion was detected.
    #[bits(1, access = RO)]
    pub significant_motion: bool,
}

readable_register!(Status1, RegisterAddress::STATUS1);

/// [`COD_STATUS`](RegisterAddress::COD_STATUS) (46h): calibration-on-demand status
/// (datasheet 5.7). Valid only after [`Ctrl9Command::OnDemandCalibration`].
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CodStatus {
    /// COD failed; no correction applied.
    #[bits(1, access = RO)]
    pub cod_failed: bool,

    /// COD was called while the gyroscope was enabled, and failed.
    #[bits(1, access = RO)]
    pub gyro_enabled: bool,

    /// The gyroscope failed to start up during COD.
    #[bits(1, access = RO)]
    pub startup_failed: bool,

    /// Significant vibration happened during COD.
    #[bits(1, access = RO)]
    pub accel_check: bool,

    /// The Y axis failed the high sensitivity limit.
    #[bits(1, access = RO)]
    pub y_limit_h_fail: bool,

    /// The Y axis failed the low sensitivity limit.
    #[bits(1, access = RO)]
    pub y_limit_l_fail: bool,

    /// The X axis failed the high sensitivity limit.
    #[bits(1, access = RO)]
    pub x_limit_h_fail: bool,

    /// The X axis failed the low sensitivity limit.
    #[bits(1, access = RO)]
    pub x_limit_l_fail: bool,
}

readable_register!(CodStatus, RegisterAddress::COD_STATUS);

/// [`TAP_STATUS`](RegisterAddress::TAP_STATUS) (59h): axis, direction and number of the
/// detected tap (datasheet table 26).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapStatus {
    /// `0`: no tap, `1`: single tap, `2`: double tap, `3`: N/A.
    #[bits(2, access = RO)]
    pub tap_num: u8,

    #[bits(2)]
    __: u8,

    /// `0`: no tap, `1`: X axis, `2`: Y axis, `3`: Z axis.
    #[bits(2, access = RO)]
    pub tap_axis: u8,

    #[bits(1)]
    __: u8,

    /// The tap was in the negative (`true`) or positive (`false`) direction of the axis.
    #[bits(1, access = RO)]
    pub tap_polarity: bool,
}

readable_register!(TapStatus, RegisterAddress::TAP_STATUS);

/// [`RESET`](RegisterAddress::RESET) (60h): soft reset (datasheet 5.9). Write-only.
///
/// Writing [`SOFT_RESET_COMMAND`](crate::SOFT_RESET_COMMAND) resets the device. Afterwards,
/// [`DqyL`] reads [`RESET_DONE`](crate::RESET_DONE).
#[bitfield(u8)]
#[derive(PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Reset {
    /// The reset command.
    #[bits(8, access = WO)]
    pub value: u8,
}

write_only_register!(Reset, RegisterAddress::RESET);

byte_register!(
    /// [`CAL1_L`](RegisterAddress::CAL1_L) (0Bh): calibration register 1, low byte.
    ///
    /// The CAL registers carry the parameters of CTRL9 commands (datasheet 5.10.3).
    Cal1L, RegisterAddress::CAL1_L, RW, writable_register
);
byte_register!(
    /// [`CAL1_H`](RegisterAddress::CAL1_H) (0Ch): calibration register 1, high byte.
    Cal1H, RegisterAddress::CAL1_H, RW, writable_register
);
byte_register!(
    /// [`CAL2_L`](RegisterAddress::CAL2_L) (0Dh): calibration register 2, low byte.
    Cal2L, RegisterAddress::CAL2_L, RW, writable_register
);
byte_register!(
    /// [`CAL2_H`](RegisterAddress::CAL2_H) (0Eh): calibration register 2, high byte.
    Cal2H, RegisterAddress::CAL2_H, RW, writable_register
);
byte_register!(
    /// [`CAL3_L`](RegisterAddress::CAL3_L) (0Fh): calibration register 3, low byte.
    Cal3L, RegisterAddress::CAL3_L, RW, writable_register
);
byte_register!(
    /// [`CAL3_H`](RegisterAddress::CAL3_H) (10h): calibration register 3, high byte.
    Cal3H, RegisterAddress::CAL3_H, RW, writable_register
);
byte_register!(
    /// [`CAL4_L`](RegisterAddress::CAL4_L) (11h): calibration register 4, low byte.
    Cal4L, RegisterAddress::CAL4_L, RW, writable_register
);
byte_register!(
    /// [`CAL4_H`](RegisterAddress::CAL4_H) (12h): calibration register 4, high byte.
    Cal4H, RegisterAddress::CAL4_H, RW, writable_register
);
byte_register!(
    /// [`TIMESTAMP_LOW`](RegisterAddress::TIMESTAMP_LOW) (30h): sample time stamp, bits 7:0.
    ///
    /// The 24-bit counter increments by one per sample of the sensor with the highest ODR
    /// and wraps at `0xFFFFFF` (datasheet table 24).
    TimestampLow, RegisterAddress::TIMESTAMP_LOW, RO, readable_register
);
byte_register!(
    /// [`TIMESTAMP_MID`](RegisterAddress::TIMESTAMP_MID) (31h): sample time stamp, bits 15:8.
    TimestampMid, RegisterAddress::TIMESTAMP_MID, RO, readable_register
);
byte_register!(
    /// [`TIMESTAMP_HIGH`](RegisterAddress::TIMESTAMP_HIGH) (32h): sample time stamp, bits 23:16.
    TimestampHigh, RegisterAddress::TIMESTAMP_HIGH, RO, readable_register
);
byte_register!(
    /// [`TEMP_L`](RegisterAddress::TEMP_L) (33h): temperature, low byte.
    ///
    /// The temperature is a 16-bit two's complement value at 256 LSB/°C:
    /// `T = TEMP_H + TEMP_L / 256` (datasheet table 25).
    TempL, RegisterAddress::TEMP_L, RO, readable_register
);
byte_register!(
    /// [`TEMP_H`](RegisterAddress::TEMP_H) (34h): temperature, high byte.
    TempH, RegisterAddress::TEMP_H, RO, readable_register
);
byte_register!(
    /// [`AX_L`](RegisterAddress::AX_L) (35h): X-axis acceleration, low byte.
    ///
    /// The outputs are 16-bit two's complement values; see
    /// [`AccelFullScale::lsb_per_g`](crate::AccelFullScale::lsb_per_g).
    AxL, RegisterAddress::AX_L, RO, readable_register
);
byte_register!(
    /// [`AX_H`](RegisterAddress::AX_H) (36h): X-axis acceleration, high byte.
    AxH, RegisterAddress::AX_H, RO, readable_register
);
byte_register!(
    /// [`AY_L`](RegisterAddress::AY_L) (37h): Y-axis acceleration, low byte.
    AyL, RegisterAddress::AY_L, RO, readable_register
);
byte_register!(
    /// [`AY_H`](RegisterAddress::AY_H) (38h): Y-axis acceleration, high byte.
    AyH, RegisterAddress::AY_H, RO, readable_register
);
byte_register!(
    /// [`AZ_L`](RegisterAddress::AZ_L) (39h): Z-axis acceleration, low byte.
    AzL, RegisterAddress::AZ_L, RO, readable_register
);
byte_register!(
    /// [`AZ_H`](RegisterAddress::AZ_H) (3Ah): Z-axis acceleration, high byte.
    AzH, RegisterAddress::AZ_H, RO, readable_register
);
byte_register!(
    /// [`GX_L`](RegisterAddress::GX_L) (3Bh): X-axis angular rate, low byte.
    ///
    /// The outputs are 16-bit two's complement values; see
    /// [`GyroFullScale::lsb_per_dps`](crate::GyroFullScale::lsb_per_dps).
    GxL, RegisterAddress::GX_L, RO, readable_register
);
byte_register!(
    /// [`GX_H`](RegisterAddress::GX_H) (3Ch): X-axis angular rate, high byte.
    GxH, RegisterAddress::GX_H, RO, readable_register
);
byte_register!(
    /// [`GY_L`](RegisterAddress::GY_L) (3Dh): Y-axis angular rate, low byte.
    GyL, RegisterAddress::GY_L, RO, readable_register
);
byte_register!(
    /// [`GY_H`](RegisterAddress::GY_H) (3Eh): Y-axis angular rate, high byte.
    GyH, RegisterAddress::GY_H, RO, readable_register
);
byte_register!(
    /// [`GZ_L`](RegisterAddress::GZ_L) (3Fh): Z-axis angular rate, low byte.
    GzL, RegisterAddress::GZ_L, RO, readable_register
);
byte_register!(
    /// [`GZ_H`](RegisterAddress::GZ_H) (40h): Z-axis angular rate, high byte.
    GzH, RegisterAddress::GZ_H, RO, readable_register
);
byte_register!(
    /// [`DQW_L`](RegisterAddress::DQW_L) (49h): general purpose register (firmware version byte 0).
    ///
    /// After a reset, and after [`Ctrl9Command::CopyUsid`](crate::Ctrl9Command::CopyUsid),
    /// `dQW_L` to `dQX_L` hold the firmware version and `dVX_L` to `dVZ_H` the unique
    /// device ID (USID, datasheet 5.10.6.11).
    DqwL, RegisterAddress::DQW_L, RO, readable_register
);
byte_register!(
    /// [`DQW_H`](RegisterAddress::DQW_H) (4Ah): general purpose register (firmware version byte 1).
    DqwH, RegisterAddress::DQW_H, RO, readable_register
);
byte_register!(
    /// [`DQX_L`](RegisterAddress::DQX_L) (4Bh): general purpose register (firmware version byte 2).
    DqxL, RegisterAddress::DQX_L, RO, readable_register
);
byte_register!(
    /// [`DQX_H`](RegisterAddress::DQX_H) (4Ch): reserved.
    DqxH, RegisterAddress::DQX_H, RO, readable_register
);
byte_register!(
    /// [`DQY_L`](RegisterAddress::DQY_L) (4Dh): general purpose register (reset status, see [`RESET_DONE`](crate::RESET_DONE)).
    DqyL, RegisterAddress::DQY_L, RO, readable_register
);
byte_register!(
    /// [`DQY_H`](RegisterAddress::DQY_H) (4Eh): reserved.
    DqyH, RegisterAddress::DQY_H, RO, readable_register
);
byte_register!(
    /// [`DQZ_L`](RegisterAddress::DQZ_L) (4Fh): reserved.
    DqzL, RegisterAddress::DQZ_L, RO, readable_register
);
byte_register!(
    /// [`DQZ_H`](RegisterAddress::DQZ_H) (50h): reserved.
    DqzH, RegisterAddress::DQZ_H, RO, readable_register
);
byte_register!(
    /// [`DVX_L`](RegisterAddress::DVX_L) (51h): general purpose register (USID byte 0).
    DvxL, RegisterAddress::DVX_L, RO, readable_register
);
byte_register!(
    /// [`DVX_H`](RegisterAddress::DVX_H) (52h): general purpose register (USID byte 1).
    DvxH, RegisterAddress::DVX_H, RO, readable_register
);
byte_register!(
    /// [`DVY_L`](RegisterAddress::DVY_L) (53h): general purpose register (USID byte 2).
    DvyL, RegisterAddress::DVY_L, RO, readable_register
);
byte_register!(
    /// [`DVY_H`](RegisterAddress::DVY_H) (54h): general purpose register (USID byte 3).
    DvyH, RegisterAddress::DVY_H, RO, readable_register
);
byte_register!(
    /// [`DVZ_L`](RegisterAddress::DVZ_L) (55h): general purpose register (USID byte 4).
    DvzL, RegisterAddress::DVZ_L, RO, readable_register
);
byte_register!(
    /// [`DVZ_H`](RegisterAddress::DVZ_H) (56h): general purpose register (USID byte 5).
    DvzH, RegisterAddress::DVZ_H, RO, readable_register
);
byte_register!(
    /// [`STEP_CNT_LOW`](RegisterAddress::STEP_CNT_LOW) (5Ah): pedometer step count, bits 7:0.
    StepCntLow, RegisterAddress::STEP_CNT_LOW, RO, readable_register
);
byte_register!(
    /// [`STEP_CNT_MID`](RegisterAddress::STEP_CNT_MID) (5Bh): pedometer step count, bits 15:8.
    StepCntMid, RegisterAddress::STEP_CNT_MID, RO, readable_register
);
byte_register!(
    /// [`STEP_CNT_HIGH`](RegisterAddress::STEP_CNT_HIGH) (5Ch): pedometer step count, bits 23:16.
    StepCntHigh, RegisterAddress::STEP_CNT_HIGH, RO, readable_register
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::*;

    #[test]
    fn power_on_defaults() {
        assert_eq!(u8::from(WhoAmI::new()), 0x05);
        assert_eq!(u8::from(Ctrl1::new()), 0x20);
        assert_eq!(u8::from(Ctrl2::new()), 0x00);
        assert_eq!(u8::from(Ctrl3::new()), 0x00);
        assert_eq!(u8::from(Ctrl5::new()), 0x00);
        assert_eq!(u8::from(Ctrl7::new()), 0x00);
        assert_eq!(u8::from(Ctrl8::new()), 0x00);
        assert_eq!(u8::from(Ctrl9::new()), 0x00);
        assert_eq!(u8::from(FifoCtrl::new()), 0x00);
    }

    #[test]
    fn ctrl1_layout() {
        let reg = Ctrl1::new()
            .with_be(false)
            .with_addr_ai(true)
            .with_int2_en(true);
        assert_eq!(u8::from(reg), 0b0101_0000);
        let reg = Ctrl1::from(0b1000_1101);
        assert!(reg.sim() && reg.int1_en() && reg.fifo_int_sel() && reg.sensor_disable());
    }

    #[test]
    fn sensor_configuration_layout() {
        let reg = Ctrl2::new()
            .with_a_fs(AccelFullScale::G8)
            .with_a_odr(AccelOdr::Hz56);
        assert_eq!(u8::from(reg), 0x27);
        let reg = Ctrl3::new()
            .with_g_fs(GyroFullScale::Dps512)
            .with_g_odr(GyroOdr::Hz56)
            .with_g_st(true);
        assert_eq!(u8::from(reg), 0xD7);
        let reg = Ctrl5::new()
            .with_g_lpf_en(true)
            .with_g_lpf_mode(LowPassMode::Percent13_37)
            .with_a_lpf_mode(LowPassMode::Percent3_63);
        assert_eq!(u8::from(reg), 0b0111_0010);
        let reg = Ctrl7::new()
            .with_a_en(true)
            .with_g_en(true)
            .with_drdy_dis(true);
        assert_eq!(u8::from(reg), 0b0010_0011);
        assert!(Ctrl8::from(0x80).ctrl9_handshake_type());
        assert_eq!(
            Ctrl9::new().with_cmd(Ctrl9Command::RequestFifo).into_bits(),
            0x05
        );
    }

    #[test]
    fn status_layout() {
        let s = StatusInt::from(0x81);
        assert!(s.cmd_done() && s.avail() && !s.locked());
        let s = Status0::from(0x02);
        assert!(s.g_da() && !s.a_da());
        let s = Status1::from(0x22);
        assert!(s.any_motion() && s.tap() && !s.wom());
        let s = FifoStatus::from(0b1001_0011);
        assert!(s.fifo_full() && s.fifo_not_empty() && !s.fifo_wtm());
        assert_eq!(s.fifo_smpl_cnt_msb(), 3);
        let s = TapStatus::from(0b1010_0010);
        assert!(s.tap_polarity());
        assert_eq!((s.tap_axis(), s.tap_num()), (2, 2));
        assert!(CodStatus::from(0x81).x_limit_l_fail());
    }

    #[test]
    fn addresses() {
        assert_eq!(
            <WhoAmI as I2CRegister<_, _, _>>::REGISTER_ADDRESS,
            RegisterAddress8::new(0x00)
        );
        assert_eq!(*<AxL as SPIRegister<_, _>>::REGISTER_ADDRESS, 0x35);
        assert_eq!(*<GzH as SPIRegister<_, _>>::REGISTER_ADDRESS, 0x40);
        assert_eq!(*<DqyL as SPIRegister<_, _>>::REGISTER_ADDRESS, 0x4D);
        assert_eq!(*<Reset as SPIRegister<_, _>>::REGISTER_ADDRESS, 0x60);
        assert_eq!(
            Reset::new().with_value(crate::SOFT_RESET_COMMAND).to_bits(),
            0xB0
        );
    }
}
