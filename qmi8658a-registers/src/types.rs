//! Value types used in register fields.
//!
//! Every type converts from any bit pattern. Codes the datasheet marks as reserved or N/A
//! are kept in a `Reserved` variant, so reading and writing back a register never changes it.

/// Declares a field enum with const `into_bits` / `from_bits`.
///
/// With `reserved`, the enum gets a `Reserved` catch-all for codes the datasheet does not
/// assign. Without it, the variants must cover every code of the mask.
macro_rules! field_enum {
    (
        $(#[$meta:meta])*
        $name:ident, $mask:expr, reserved, {
            $( $(#[$vmeta:meta])* $variant:ident = $value:expr ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum $name {
            $( $(#[$vmeta])* $variant, )+
            /// A code the datasheet marks as reserved or N/A. Holds the raw field value.
            Reserved(u8),
        }

        impl $name {
            /// Converts the value into its register representation.
            pub const fn into_bits(self) -> u8 {
                match self {
                    $( Self::$variant => $value, )+
                    Self::Reserved(bits) => bits & $mask,
                }
            }

            /// Converts the lowest bits of a register value into this type.
            pub const fn from_bits(value: u8) -> Self {
                match value & $mask {
                    $( $value => Self::$variant, )+
                    bits => Self::Reserved(bits),
                }
            }
        }
    };
    (
        $(#[$meta:meta])*
        $name:ident, $mask:expr, {
            $( $(#[$vmeta:meta])* $variant:ident = $value:expr ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[repr(u8)]
        pub enum $name {
            $( $(#[$vmeta])* $variant = $value, )+
        }

        impl $name {
            /// Converts the value into its register representation.
            pub const fn into_bits(self) -> u8 {
                self as u8
            }

            /// Converts the lowest bits of a register value into this type.
            pub const fn from_bits(value: u8) -> Self {
                match value & $mask {
                    $( $value => Self::$variant, )+
                    _ => unreachable!(),
                }
            }
        }
    };
}

field_enum!(
    /// Accelerometer full scale (`CTRL2.aFS`, datasheet table 22).
    AccelFullScale, 0b111, reserved, {
        /// ±2 g, 16384 LSB/g. The default.
        G2 = 0b000,
        /// ±4 g, 8192 LSB/g.
        G4 = 0b001,
        /// ±8 g, 4096 LSB/g.
        G8 = 0b010,
        /// ±16 g, 2048 LSB/g.
        G16 = 0b011,
    }
);

impl AccelFullScale {
    /// Returns the sensitivity in LSB per g (datasheet table 7).
    ///
    /// Reserved codes return the sensitivity of the default, [`AccelFullScale::G2`].
    pub const fn lsb_per_g(self) -> f32 {
        match self {
            Self::G2 | Self::Reserved(_) => 16384.0,
            Self::G4 => 8192.0,
            Self::G8 => 4096.0,
            Self::G16 => 2048.0,
        }
    }
}

field_enum!(
    /// Gyroscope full scale (`CTRL3.gFS`, datasheet table 22).
    GyroFullScale, 0b111, {
        /// ±16 dps, 2048 LSB/dps. The default.
        Dps16 = 0b000,
        /// ±32 dps, 1024 LSB/dps.
        Dps32 = 0b001,
        /// ±64 dps, 512 LSB/dps.
        Dps64 = 0b010,
        /// ±128 dps, 256 LSB/dps.
        Dps128 = 0b011,
        /// ±256 dps, 128 LSB/dps.
        Dps256 = 0b100,
        /// ±512 dps, 64 LSB/dps.
        Dps512 = 0b101,
        /// ±1024 dps, 32 LSB/dps.
        Dps1024 = 0b110,
        /// ±2048 dps, 16 LSB/dps.
        Dps2048 = 0b111,
    }
);

impl GyroFullScale {
    /// Returns the sensitivity in LSB per degree per second (datasheet table 8).
    pub const fn lsb_per_dps(self) -> f32 {
        // 2048 LSB/dps at ±16 dps, halving with every doubling of the range.
        (2048u16 >> self.into_bits()) as f32
    }
}

field_enum!(
    /// Accelerometer output data rate (`CTRL2.aODR`, datasheet table 22).
    ///
    /// The rate of a code depends on whether the gyroscope is enabled: with the gyroscope,
    /// all rates derive from its natural frequency (6DOF column); without it, from the
    /// internal oscillator (accelerometer-only column). The variants are named after the
    /// 6DOF rate; see [`AccelOdr::hz`].
    AccelOdr, 0b1111, reserved, {
        /// 7174.4 Hz (6DOF only). The default.
        Hz7174 = 0b0000,
        /// 3587.2 Hz (6DOF only).
        Hz3587 = 0b0001,
        /// 1793.6 Hz (6DOF only).
        Hz1794 = 0b0010,
        /// 896.8 Hz (6DOF), 1000 Hz (accelerometer only).
        Hz897 = 0b0011,
        /// 448.4 Hz (6DOF), 500 Hz (accelerometer only).
        Hz448 = 0b0100,
        /// 224.2 Hz (6DOF), 250 Hz (accelerometer only).
        Hz224 = 0b0101,
        /// 112.1 Hz (6DOF), 125 Hz (accelerometer only).
        Hz112 = 0b0110,
        /// 56.05 Hz (6DOF), 62.5 Hz (accelerometer only).
        Hz56 = 0b0111,
        /// 28.025 Hz (6DOF), 31.25 Hz (accelerometer only).
        Hz28 = 0b1000,
        /// 128 Hz low-power mode (accelerometer only), 100 % duty cycle.
        LowPowerHz128 = 0b1100,
        /// 21 Hz low-power mode (accelerometer only), 58 % duty cycle.
        LowPowerHz21 = 0b1101,
        /// 11 Hz low-power mode (accelerometer only), 31 % duty cycle.
        LowPowerHz11 = 0b1110,
        /// 3 Hz low-power mode (accelerometer only), 8.5 % duty cycle.
        LowPowerHz3 = 0b1111,
    }
);

impl AccelOdr {
    /// Returns the rate in Hz with the gyroscope enabled (6DOF) or disabled
    /// (accelerometer only), or `None` if the datasheet marks the code N/A for that mode.
    pub const fn hz(self, gyro_enabled: bool) -> Option<f32> {
        match (self, gyro_enabled) {
            (Self::Hz7174, true) => Some(7174.4),
            (Self::Hz3587, true) => Some(3587.2),
            (Self::Hz1794, true) => Some(1793.6),
            (Self::Hz897, true) => Some(896.8),
            (Self::Hz897, false) => Some(1000.0),
            (Self::Hz448, true) => Some(448.4),
            (Self::Hz448, false) => Some(500.0),
            (Self::Hz224, true) => Some(224.2),
            (Self::Hz224, false) => Some(250.0),
            (Self::Hz112, true) => Some(112.1),
            (Self::Hz112, false) => Some(125.0),
            (Self::Hz56, true) => Some(56.05),
            (Self::Hz56, false) => Some(62.5),
            (Self::Hz28, true) => Some(28.025),
            (Self::Hz28, false) => Some(31.25),
            (Self::LowPowerHz128, false) => Some(128.0),
            (Self::LowPowerHz21, false) => Some(21.0),
            (Self::LowPowerHz11, false) => Some(11.0),
            (Self::LowPowerHz3, false) => Some(3.0),
            _ => None,
        }
    }

    /// Whether this is a low-power rate (`aODR = 11xx`), available only with the gyroscope
    /// disabled.
    pub const fn is_low_power(self) -> bool {
        self.into_bits() & 0b1100 == 0b1100
    }
}

field_enum!(
    /// Gyroscope output data rate (`CTRL3.gODR`, datasheet table 22).
    ///
    /// All rates derive from the gyroscope's natural frequency. The datasheet does not
    /// recommend rates above 1 kHz where gyroscope noise matters (table 12).
    GyroOdr, 0b1111, reserved, {
        /// 7174.4 Hz. The default.
        Hz7174 = 0b0000,
        /// 3587.2 Hz.
        Hz3587 = 0b0001,
        /// 1793.6 Hz.
        Hz1794 = 0b0010,
        /// 896.8 Hz.
        Hz897 = 0b0011,
        /// 448.4 Hz.
        Hz448 = 0b0100,
        /// 224.2 Hz.
        Hz224 = 0b0101,
        /// 112.1 Hz.
        Hz112 = 0b0110,
        /// 56.05 Hz.
        Hz56 = 0b0111,
        /// 28.025 Hz.
        Hz28 = 0b1000,
    }
);

impl GyroOdr {
    /// Returns the rate in Hz, or `None` for reserved codes.
    pub const fn hz(self) -> Option<f32> {
        AccelOdr::from_bits(self.into_bits()).hz(true)
    }
}

field_enum!(
    /// Low-pass filter bandwidth (`CTRL5.aLPF_MODE`, `CTRL5.gLPF_MODE`, datasheet table 22).
    LowPassMode, 0b11, {
        /// 2.66 % of the ODR. The default.
        Percent2_66 = 0b00,
        /// 3.63 % of the ODR.
        Percent3_63 = 0b01,
        /// 5.39 % of the ODR.
        Percent5_39 = 0b10,
        /// 13.37 % of the ODR.
        Percent13_37 = 0b11,
    }
);

field_enum!(
    /// FIFO size (`FIFO_CTRL.FIFO_SIZE`, datasheet table 23).
    ///
    /// A sample is 6 bytes with one sensor enabled and 12 bytes with both.
    FifoSize, 0b11, {
        /// 16 samples. The default.
        Samples16 = 0b00,
        /// 32 samples.
        Samples32 = 0b01,
        /// 64 samples.
        Samples64 = 0b10,
        /// 128 samples.
        Samples128 = 0b11,
    }
);

field_enum!(
    /// FIFO mode (`FIFO_CTRL.FIFO_MODE`, datasheet table 23).
    FifoMode, 0b11, reserved, {
        /// The FIFO is disabled; data-ready (DRDY) is enabled. The default.
        Bypass = 0b00,
        /// Filling stops when the FIFO is full.
        Fifo = 0b01,
        /// When the FIFO is full, the oldest data is discarded.
        Stream = 0b10,
    }
);

field_enum!(
    /// A CTRL9 host command (datasheet table 28).
    Ctrl9Command, 0xFF, reserved, {
        /// `CTRL_CMD_ACK`: acknowledges a completed command. The default.
        Ack = 0x00,
        /// `CTRL_CMD_RST_FIFO`: resets the FIFO.
        ResetFifo = 0x04,
        /// `CTRL_CMD_REQ_FIFO`: enters FIFO read mode.
        RequestFifo = 0x05,
        /// `CTRL_CMD_WRITE_WOM_SETTING`: sets up and enables Wake on Motion.
        WriteWomSetting = 0x08,
        /// `CTRL_CMD_ACCEL_HOST_DELTA_OFFSET`: changes the accelerometer offset.
        AccelHostDeltaOffset = 0x09,
        /// `CTRL_CMD_GYRO_HOST_DELTA_OFFSET`: changes the gyroscope offset.
        GyroHostDeltaOffset = 0x0A,
        /// `CTRL_CMD_CONFIGURE_TAP`: configures tap detection.
        ConfigureTap = 0x0C,
        /// `CTRL_CMD_CONFIGURE_PEDOMETER`: configures the pedometer.
        ConfigurePedometer = 0x0D,
        /// `CTRL_CMD_CONFIGURE_MOTION`: configures any-, no- and significant-motion detection.
        ConfigureMotion = 0x0E,
        /// `CTRL_CMD_RESET_PEDOMETER`: resets the step count.
        ResetPedometer = 0x0F,
        /// `CTRL_CMD_COPY_USID`: copies the USID and firmware version to the UI registers.
        CopyUsid = 0x10,
        /// `CTRL_CMD_SET_RPU`: configures the I/O pull-up resistors.
        SetRpu = 0x11,
        /// `CTRL_CMD_AHB_CLOCK_GATING`: switches the internal AHB clock gating.
        AhbClockGating = 0x12,
        /// `CTRL_CMD_ON_DEMAND_CALIBRATION`: runs gyroscope calibration-on-demand (COD).
        OnDemandCalibration = 0xA2,
        /// `CTRL_CMD_APPLY_GYRO_GAINS`: restores saved gyroscope gains.
        ApplyGyroGains = 0xAA,
    }
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips() {
        for bits in 0..=u8::MAX {
            assert_eq!(AccelFullScale::from_bits(bits).into_bits(), bits & 0x7);
            assert_eq!(GyroFullScale::from_bits(bits).into_bits(), bits & 0x7);
            assert_eq!(AccelOdr::from_bits(bits).into_bits(), bits & 0xF);
            assert_eq!(GyroOdr::from_bits(bits).into_bits(), bits & 0xF);
            assert_eq!(LowPassMode::from_bits(bits).into_bits(), bits & 0x3);
            assert_eq!(FifoSize::from_bits(bits).into_bits(), bits & 0x3);
            assert_eq!(FifoMode::from_bits(bits).into_bits(), bits & 0x3);
            assert_eq!(Ctrl9Command::from_bits(bits).into_bits(), bits);
        }
    }

    #[test]
    fn sensitivities() {
        assert_eq!(AccelFullScale::G8.lsb_per_g(), 4096.0);
        assert_eq!(AccelFullScale::Reserved(4).lsb_per_g(), 16384.0);
        assert_eq!(GyroFullScale::Dps16.lsb_per_dps(), 2048.0);
        assert_eq!(GyroFullScale::Dps512.lsb_per_dps(), 64.0);
        assert_eq!(GyroFullScale::Dps2048.lsb_per_dps(), 16.0);
    }

    #[test]
    fn rates() {
        assert_eq!(AccelOdr::Hz56.hz(true), Some(56.05));
        assert_eq!(AccelOdr::Hz56.hz(false), Some(62.5));
        assert_eq!(AccelOdr::Hz7174.hz(false), None);
        assert_eq!(AccelOdr::LowPowerHz3.hz(true), None);
        assert_eq!(AccelOdr::Reserved(9).hz(true), None);
        assert!(AccelOdr::LowPowerHz21.is_low_power() && !AccelOdr::Hz28.is_low_power());
        assert_eq!(GyroOdr::Hz28.hz(), Some(28.025));
        assert_eq!(GyroOdr::Reserved(12).hz(), None);
    }
}
