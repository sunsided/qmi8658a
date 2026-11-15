//! Motion engines: any-, no- and significant-motion (datasheet 9), tap (10), pedometer
//! (11) and Wake on Motion (12).

use crate::Qmi8658aModel;
use qmi8658a_registers::{Ctrl8, RegisterAddress as Reg};

/// `STATUS1` bits.
const TAP: u8 = 1 << 1;
const WOM: u8 = 1 << 2;
const PEDOMETER: u8 = 1 << 4;
const ANY_MOTION: u8 = 1 << 5;
const NO_MOTION: u8 = 1 << 6;
const SIGNIFICANT_MOTION: u8 = 1 << 7;

/// The bits that `STATUS1` reads clear.
const LATCHED: u8 = TAP | WOM | PEDOMETER;

/// Wake on Motion settings and state (datasheet table 39).
#[derive(Debug, Copy, Clone)]
struct Wom {
    /// The slope threshold, in mg.
    threshold_mg: u8,
    /// The interrupt line: INT2 (`true`) or INT1.
    int2: bool,
    /// The initial level of the interrupt line.
    initial: bool,
    /// Accelerometer samples still to ignore.
    blanking: u8,
    /// The current level of the interrupt line.
    line: bool,
}

/// Engine parameters and state.
#[derive(Debug, Clone)]
pub(crate) struct Engines {
    /// The two CAL parameter sets of `CTRL_CMD_CONFIGURE_MOTION`.
    motion: [[u8; 8]; 2],
    /// The two CAL parameter sets of `CTRL_CMD_CONFIGURE_TAP`.
    tap: [[u8; 8]; 2],
    /// The two CAL parameter sets of `CTRL_CMD_CONFIGURE_PEDOMETER`.
    pedometer: [[u8; 8]; 2],
    /// The previous acceleration, in mg, for the slope.
    previous: Option<[i32; 3]>,
    /// The number of accelerometer samples so far.
    sample: u32,
    any_count: u32,
    no_count: u32,
    /// The sample of the first any-motion event of a significant-motion detection.
    first_any: Option<u32>,
    significant: bool,
    wom: Option<Wom>,
    /// Steps since the last step count update.
    pending_steps: u32,
}

impl Engines {
    pub fn new() -> Self {
        Self {
            motion: [[0; 8]; 2],
            tap: [[0; 8]; 2],
            pedometer: [[0; 8]; 2],
            previous: None,
            sample: 0,
            any_count: 0,
            no_count: 0,
            first_any: None,
            significant: false,
            wom: None,
            pending_steps: 0,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Stores one of two parameter sets, selected by `CAL4_H` (`0x01` or `0x02`).
    fn store(sets: &mut [[u8; 8]; 2], cal: [u8; 8]) {
        if let Some(set) = (cal[7] as usize)
            .checked_sub(1)
            .and_then(|i| sets.get_mut(i))
        {
            *set = cal;
        }
    }

    pub fn configure_motion(&mut self, cal: [u8; 8]) {
        Self::store(&mut self.motion, cal);
    }

    pub fn configure_tap(&mut self, cal: [u8; 8]) {
        Self::store(&mut self.tap, cal);
    }

    pub fn configure_pedometer(&mut self, cal: [u8; 8]) {
        Self::store(&mut self.pedometer, cal);
    }

    /// The level Wake on Motion drives on INT2 (`true`) or INT1, if WoM uses that line.
    pub fn wom_line(&self, int2: bool) -> Option<bool> {
        self.wom.filter(|wom| wom.int2 == int2).map(|wom| wom.line)
    }
}

/// Combines the per-axis results of the enabled axes with logic AND (`all`) or OR.
fn combine(axes: u8, all: bool, hit: impl Fn(usize) -> bool) -> bool {
    let mut enabled = (0..3).filter(|&i| axes & (1 << i) != 0).peekable();
    if enabled.peek().is_none() {
        return false;
    }

    if all {
        enabled.all(hit)
    } else {
        enabled.any(hit)
    }
}

impl Qmi8658aModel {
    /// The two parameter sets the model received with `CTRL_CMD_CONFIGURE_MOTION`.
    pub fn motion_parameters(&self) -> [[u8; 8]; 2] {
        self.engines.motion
    }

    /// The two parameter sets the model received with `CTRL_CMD_CONFIGURE_TAP`.
    pub fn tap_parameters(&self) -> [[u8; 8]; 2] {
        self.engines.tap
    }

    /// The two parameter sets the model received with `CTRL_CMD_CONFIGURE_PEDOMETER`.
    pub fn pedometer_parameters(&self) -> [[u8; 8]; 2] {
        self.engines.pedometer
    }

    /// Reports a tap, as the tap engine would: on `axis` 1 (X), 2 (Y) or 3 (Z), in the
    /// negative direction, single or double. Ignored unless the tap engine and the
    /// accelerometer are enabled.
    pub fn tap(&mut self, axis: u8, negative: bool, double: bool) {
        let ctrl8 = Ctrl8::from(self.register(Reg::CTRL8));
        if !ctrl8.tap_en() || !self.ctrl7().a_en() {
            return;
        }

        let status = (u8::from(negative) << 7) | ((axis & 0b11) << 4) | (1 + u8::from(double));
        self.set(Reg::TAP_STATUS, status);
        self.set(Reg::STATUS1, self.register(Reg::STATUS1) | TAP);
    }

    /// Reports `steps` steps, as the pedometer would. The step count advances in batches of
    /// `ped_sig_count` steps, each raising the pedometer event (datasheet 11.4, 11.5).
    /// Ignored unless the pedometer and the accelerometer are enabled.
    pub fn step(&mut self, steps: u32) {
        let ctrl8 = Ctrl8::from(self.register(Reg::CTRL8));
        if !ctrl8.pedo_en() || !self.ctrl7().a_en() {
            return;
        }

        let batch = u32::from(self.engines.pedometer[1][5]).max(1);
        self.engines.pending_steps += steps;
        while self.engines.pending_steps >= batch {
            self.engines.pending_steps -= batch;
            let count = self.step_count().wrapping_add(batch);
            self.write_step_count(count);
            self.set(Reg::STATUS1, self.register(Reg::STATUS1) | PEDOMETER);
        }
    }

    /// The 24-bit step count.
    fn step_count(&self) -> u32 {
        let addr = usize::from(Reg::STEP_CNT_LOW.addr());
        u32::from_le_bytes([self.regs[addr], self.regs[addr + 1], self.regs[addr + 2], 0])
    }

    /// Sets the step count and drops steps not yet counted.
    pub(crate) fn set_step_count(&mut self, count: u32) {
        self.write_step_count(count);
        self.engines.pending_steps = 0;
    }

    /// Writes the 24-bit step count, wrapping after `0xFFFFFF` (datasheet 11.5).
    fn write_step_count(&mut self, count: u32) {
        let addr = usize::from(Reg::STEP_CNT_LOW.addr());
        self.regs[addr..addr + 3].copy_from_slice(&count.to_le_bytes()[..3]);
    }

    /// Whether an activity event drives the activity interrupt (datasheet 9.6, 10.5, 11.4).
    pub(crate) fn activity(&self) -> bool {
        self.register(Reg::STATUS1) & !WOM != 0
    }

    /// Reads `STATUS1`: clears the latched flags and resets the WoM line (datasheet 12.4).
    pub(crate) fn read_status1(&mut self) -> u8 {
        let value = self.register(Reg::STATUS1);
        self.set(Reg::STATUS1, value & !LATCHED);
        if let Some(wom) = &mut self.engines.wom {
            wom.line = wom.initial;
        }

        value
    }

    /// `CTRL_CMD_WRITE_WOM_SETTING`: a threshold of `0` leaves WoM mode (datasheet 12.2).
    pub(crate) fn configure_wom(&mut self, cal: [u8; 8]) {
        self.engines.wom = (cal[0] != 0).then(|| Wom {
            threshold_mg: cal[0],
            int2: cal[1] & 0x40 != 0,
            initial: cal[1] & 0x80 != 0,
            blanking: cal[1] & 0x3F,
            line: cal[1] & 0x80 != 0,
        });
        self.set(Reg::STATUS1, self.register(Reg::STATUS1) & !WOM);
    }

    /// Runs the engines on an accelerometer sample, in mg.
    pub(crate) fn engines_sample(&mut self, mg: [i32; 3]) {
        let ctrl8 = Ctrl8::from(self.register(Reg::CTRL8));
        let engines = &mut self.engines;
        engines.sample = engines.sample.wrapping_add(1);
        let Some(previous) = engines.previous.replace(mg) else {
            return;
        };

        let slope: [i32; 3] = core::array::from_fn(|i| (mg[i] - previous[i]).abs());
        let mut status = self.regs[usize::from(Reg::STATUS1.addr())] & LATCHED;

        if let Some(wom) = &mut engines.wom {
            if wom.blanking > 0 {
                wom.blanking -= 1;
            } else if slope.iter().any(|&s| s > i32::from(wom.threshold_mg)) {
                wom.line = !wom.line;
                status |= WOM;
            }
        }

        // Thresholds in 1/32 g (table 32): slope_mg > threshold * 1000 / 32.
        let [first, second] = engines.motion;
        let mode = first[6];
        let above = |i: usize| slope[i] * 32 > i32::from(first[i]) * 1000;
        let below = |i: usize| slope[i] * 32 < i32::from(first[3 + i]) * 1000;
        let any_hit = combine(mode & 0b111, mode & 0x08 != 0, above);
        let no_hit = combine((mode >> 4) & 0b111, mode & 0x80 != 0, below);

        engines.any_count = if ctrl8.any_motion_en() && any_hit {
            engines.any_count + 1
        } else {
            0
        };
        engines.no_count = if ctrl8.no_motion_en() && no_hit {
            engines.no_count + 1
        } else {
            0
        };
        let any = engines.any_count > 0 && engines.any_count >= u32::from(second[0]);
        let no = engines.no_count > 0 && engines.no_count >= u32::from(second[1]);

        if !ctrl8.sig_motion_en() || no {
            engines.significant = false;
            engines.first_any = None;
        } else if any {
            let wait = u32::from(u16::from_le_bytes([second[2], second[3]]));
            let confirm = u32::from(u16::from_le_bytes([second[4], second[5]]));
            match engines.first_any {
                Some(first) if (wait..=confirm).contains(&(engines.sample - first)) => {
                    engines.significant = true;
                }
                Some(first) if engines.sample - first > confirm => {
                    engines.first_any = Some(engines.sample);
                }
                Some(_) => {}
                None => engines.first_any = Some(engines.sample),
            }
        }

        if any {
            status |= ANY_MOTION;
        }

        if no {
            status |= NO_MOTION;
        }

        if engines.significant {
            status |= SIGNIFICANT_MOTION;
        }

        self.set(Reg::STATUS1, status);
    }
}
