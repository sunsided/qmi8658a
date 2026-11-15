//! Parameters of the motion engines: any-, no- and significant-motion (datasheet 9),
//! tap (10), pedometer (11) and Wake on Motion (12).
//!
//! Each `cal` method returns the CAL register contents (`CAL1_L` to `CAL4_H`) of the
//! CTRL9 command(s) that configure the engine.

/// Any-, no- and significant-motion parameters (datasheet table 32).
///
/// Significant-motion detection builds on any- and no-motion, so configure and enable
/// all three for it (9.1.3).
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionConfig {
    /// Any-motion slope thresholds for X, Y and Z, at 1/32 g per LSB.
    pub any_motion_threshold: [u8; 3],
    /// The axes any-motion detection uses: X, Y, Z.
    pub any_motion_axes: [bool; 3],
    /// Any-motion needs all enabled axes (`true`, logic AND) or any of them (logic OR).
    pub any_motion_all_axes: bool,
    /// The number of consecutive samples above the threshold for any-motion.
    pub any_motion_window: u8,
    /// No-motion slope thresholds for X, Y and Z, at 1/32 g per LSB.
    pub no_motion_threshold: [u8; 3],
    /// The axes no-motion detection uses: X, Y, Z.
    pub no_motion_axes: [bool; 3],
    /// No-motion needs all enabled axes (`true`, logic AND) or any of them (logic OR).
    pub no_motion_all_axes: bool,
    /// The number of consecutive samples below the threshold for no-motion.
    pub no_motion_window: u8,
    /// Samples after the first any-motion event before a second one counts.
    pub sig_motion_wait_window: u16,
    /// Samples after the first any-motion event within which a second one confirms
    /// significant motion.
    pub sig_motion_confirm_window: u16,
}

impl MotionConfig {
    /// The `MOTION_MODE_CTRL` byte (datasheet table 33).
    pub fn mode(&self) -> u8 {
        let axes = |axes: [bool; 3]| {
            axes.iter()
                .enumerate()
                .fold(0, |bits, (i, &on)| bits | (u8::from(on) << i))
        };

        axes(self.any_motion_axes)
            | (u8::from(self.any_motion_all_axes) << 3)
            | (axes(self.no_motion_axes) << 4)
            | (u8::from(self.no_motion_all_axes) << 7)
    }

    /// The CAL registers of the two `CTRL_CMD_CONFIGURE_MOTION` commands (datasheet table 34).
    pub fn cal(&self) -> [[u8; 8]; 2] {
        let [a0, a1, a2] = self.any_motion_threshold;
        let [n0, n1, n2] = self.no_motion_threshold;
        let [w0, w1] = self.sig_motion_wait_window.to_le_bytes();
        let [c0, c1] = self.sig_motion_confirm_window.to_le_bytes();
        [
            [a0, a1, a2, n0, n1, n2, self.mode(), 0x01],
            [
                self.any_motion_window,
                self.no_motion_window,
                w0,
                w1,
                c0,
                c1,
                0,
                0x02,
            ],
        ]
    }
}

/// The axis a tap is attributed to when several axes peak at once (datasheet table 35).
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TapPriority {
    /// X > Y > Z.
    #[default]
    Xyz = 0,
    /// X > Z > Y.
    Xzy = 1,
    /// Y > X > Z.
    Yxz = 2,
    /// Y > Z > X.
    Yzx = 3,
    /// Z > X > Y.
    Zxy = 4,
    /// Z > Y > X.
    Zyx = 5,
}

/// Tap detection parameters (datasheet table 35). Windows count accelerometer samples.
///
/// The datasheet recommends an accelerometer ODR above 200 Hz for tap detection (10.4).
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapConfig {
    /// The maximum duration of a valid peak.
    pub peak_window: u8,
    /// The tap axis when several axes peak at once.
    pub priority: TapPriority,
    /// The minimum quiet time after the first tap.
    pub tap_window: u16,
    /// The maximum time for the second tap of a double tap, from the first peak.
    pub double_tap_window: u16,
    /// The weight of the latest sample in the acceleration average, at 1/128 per LSB.
    pub alpha: u8,
    /// The weight of the latest sample in the movement magnitude average, at 1/128 per LSB.
    pub gamma: u8,
    /// The peak detection threshold, at 1/1024 g² per LSB.
    pub peak_magnitude_threshold: u16,
    /// The quiet threshold of the linear acceleration ("undefined motion"), at 1/1024 g per
    /// LSB.
    pub undefined_motion_threshold: u16,
}

impl TapConfig {
    /// The example values of datasheet table 35, given for 500 Hz: peak window 20, tap
    /// window 50, double-tap window 250, alpha 0.0625, gamma 0.25, peak threshold 0.8 g²,
    /// quiet threshold `0x0190`.
    pub const EXAMPLE_500HZ: Self = Self {
        peak_window: 20,
        priority: TapPriority::Xyz,
        tap_window: 50,
        double_tap_window: 250,
        alpha: 8,
        gamma: 32,
        peak_magnitude_threshold: 0x0320,
        undefined_motion_threshold: 0x0190,
    };

    /// The CAL registers of the two `CTRL_CMD_CONFIGURE_TAP` commands (datasheet table 36).
    pub fn cal(&self) -> [[u8; 8]; 2] {
        let [t0, t1] = self.tap_window.to_le_bytes();
        let [d0, d1] = self.double_tap_window.to_le_bytes();
        let [p0, p1] = self.peak_magnitude_threshold.to_le_bytes();
        let [u0, u1] = self.undefined_motion_threshold.to_le_bytes();
        [
            [
                self.peak_window,
                self.priority as u8,
                t0,
                t1,
                d0,
                d1,
                0,
                0x01,
            ],
            [self.alpha, self.gamma, p0, p1, u0, u1, 0, 0x02],
        ]
    }
}

/// Pedometer parameters (datasheet table 37). Durations count accelerometer samples.
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PedometerConfig {
    /// `ped_sample_cnt`: the samples per calculation window.
    pub sample_count: u16,
    /// `ped_fix_peak2peak`: the peak-to-peak threshold, at 1/1024 g per LSB.
    pub peak_to_peak: u16,
    /// `ped_fix_peak`: the peak threshold against the average, at 1/1024 g per LSB.
    pub peak: u16,
    /// `ped_time_up`: the maximum duration of a step.
    pub time_up: u16,
    /// `ped_time_low`: the minimum duration of a step.
    pub time_low: u8,
    /// `ped_time_cnt_entry`: the consecutive steps needed to start counting.
    pub entry_count: u8,
    /// `ped_fix_precision`: `0` is recommended.
    pub precision: u8,
    /// `ped_sig_count`: the step count updates, and the pedometer event fires, every this
    /// many steps.
    pub sig_count: u8,
}

impl PedometerConfig {
    /// The example of datasheet 11.1, for 50 Hz: 1 s window, 200 mg peak-to-peak, 100 mg
    /// peak, steps between 0.4 s and 4 s, counting after 10 steps, updates every 4 steps.
    pub const EXAMPLE_50HZ: Self = Self {
        sample_count: 50,
        peak_to_peak: 0x00CC,
        peak: 0x0066,
        time_up: 200,
        time_low: 20,
        entry_count: 10,
        precision: 0,
        sig_count: 4,
    };

    /// The CAL registers of the two `CTRL_CMD_CONFIGURE_PEDOMETER` commands (datasheet
    /// table 38).
    pub fn cal(&self) -> [[u8; 8]; 2] {
        let [s0, s1] = self.sample_count.to_le_bytes();
        let [pp0, pp1] = self.peak_to_peak.to_le_bytes();
        let [p0, p1] = self.peak.to_le_bytes();
        let [u0, u1] = self.time_up.to_le_bytes();
        [
            [s0, s1, pp0, pp1, p0, p1, 0, 0x01],
            [
                u0,
                u1,
                self.time_low,
                self.entry_count,
                self.precision,
                self.sig_count,
                0,
                0x02,
            ],
        ]
    }
}

/// Wake on Motion parameters (datasheet table 39).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WomConfig {
    /// The slope threshold, in mg. `0` disables WoM, so use a non-zero value.
    pub threshold_mg: u8,
    /// Signal on INT1 (`true`) or INT2 (`false`).
    pub int1: bool,
    /// The initial level of the interrupt line. It toggles with every WoM event and returns
    /// to this level when `STATUS1` is read (12.4).
    pub initial_high: bool,
    /// Accelerometer samples to ignore after enabling WoM, up to 63.
    pub blanking_samples: u8,
}

impl WomConfig {
    /// The CAL registers of `CTRL_CMD_WRITE_WOM_SETTING`.
    pub fn cal(&self) -> [u8; 8] {
        let select = (u8::from(self.initial_high) << 7)
            | (u8::from(!self.int1) << 6)
            | (self.blanking_samples & 0x3F);
        [self.threshold_mg, select, 0, 0, 0, 0, 0, 0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion() {
        let config = MotionConfig {
            any_motion_threshold: [1, 2, 3],
            any_motion_axes: [true, false, true],
            any_motion_all_axes: true,
            any_motion_window: 7,
            no_motion_threshold: [4, 5, 6],
            no_motion_axes: [false, true, false],
            no_motion_all_axes: false,
            no_motion_window: 8,
            sig_motion_wait_window: 0x0102,
            sig_motion_confirm_window: 0x0304,
        };
        assert_eq!(config.mode(), 0b0010_1101);
        assert_eq!(
            config.cal(),
            [
                [1, 2, 3, 4, 5, 6, 0x2D, 1],
                [7, 8, 0x02, 0x01, 0x04, 0x03, 0, 2]
            ]
        );
    }

    #[test]
    fn tap() {
        assert_eq!(
            TapConfig::EXAMPLE_500HZ.cal(),
            [
                [20, 0, 50, 0, 250, 0, 0, 1],
                [8, 32, 0x20, 0x03, 0x90, 0x01, 0, 2]
            ]
        );
    }

    #[test]
    fn pedometer() {
        assert_eq!(
            PedometerConfig::EXAMPLE_50HZ.cal(),
            [
                [50, 0, 0xCC, 0, 0x66, 0, 0, 1],
                [200, 0, 20, 10, 0, 4, 0, 2]
            ]
        );
    }

    #[test]
    fn wom() {
        let config = WomConfig {
            threshold_mg: 200,
            int1: false,
            initial_high: true,
            blanking_samples: 70,
        };
        assert_eq!(config.cal(), [200, 0b1100_0110, 0, 0, 0, 0, 0, 0]);
        let config = WomConfig {
            int1: true,
            initial_high: false,
            ..config
        };
        assert_eq!(config.cal()[1], 0b0000_0110);
    }
}
