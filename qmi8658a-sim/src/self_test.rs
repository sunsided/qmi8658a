//! Accelerometer and gyroscope self-test (datasheet section 15).

use crate::Qmi8658aModel;
use qmi8658a_registers::{AccelOdr, Ctrl2, RegisterAddress as Reg};

/// The gyroscope self-test duration, in microseconds (datasheet 15.2: about 400 ms).
const GYRO_DURATION_US: u32 = 400_000;

/// The self-test response the model reports by default: 500 mg and 500 dps per axis.
pub const DEFAULT_RESPONSE: ([i32; 3], [i32; 3]) = ([500; 3], [500_000; 3]);

/// Self-test state.
#[derive(Debug, Clone)]
pub(crate) struct SelfTest {
    /// A test in progress: gyroscope (`true`) or accelerometer, and the time it still takes.
    running: Option<(bool, u32)>,
    /// The test finished; INT2 and `STATUSINT.bit0` are high until `aST`/`gST` clears.
    done: bool,
    /// The response: acceleration in mg and angular rate in mdps.
    response: ([i32; 3], [i32; 3]),
}

impl SelfTest {
    pub fn new() -> Self {
        Self {
            running: None,
            done: false,
            response: DEFAULT_RESPONSE,
        }
    }

    /// Ends a test; keeps the response.
    pub fn reset(&mut self) {
        self.running = None;
        self.done = false;
    }

    pub fn done(&self) -> bool {
        self.done
    }
}

impl Qmi8658aModel {
    /// Sets the self-test response the model reports: acceleration in mg and angular rate
    /// in mdps, per axis. The datasheet passes above 200 mg and 300 dps.
    pub fn set_self_test_response(&mut self, accel_mg: [i32; 3], gyro_mdps: [i32; 3]) {
        self.self_test.response = (accel_mg, gyro_mdps);
    }

    /// Starts or ends a self-test on a write of `aST` (`CTRL2.bit7`) or `gST` (`CTRL3.bit7`).
    pub(crate) fn self_test_write(&mut self, addr: u8, previous: u8, value: u8) {
        let gyro = addr == Reg::CTRL3.addr();
        let (was, is) = (previous & 0x80 != 0, value & 0x80 != 0);
        if !is {
            if was {
                self.self_test.reset();
            }

            return;
        }

        if was || self.ctrl7().a_en() || self.ctrl7().g_en() {
            return;
        }

        let duration = if gyro {
            GYRO_DURATION_US
        } else {
            // About 25 ODRs (15.1), at the accelerometer-only rate.
            let odr = Ctrl2::from(value).a_odr();
            let hz = odr
                .hz(false)
                .unwrap_or(AccelOdr::Hz897.hz(false).unwrap_or(1000.0));
            (25.0e6 / hz) as u32
        };

        self.self_test.running = Some((gyro, duration));
    }

    /// Advances a self-test and writes the result to `dVX_L` to `dVZ_H` when done.
    pub(crate) fn self_test_tick(&mut self, us: u32) {
        let Some((gyro, remaining)) = self.self_test.running else {
            return;
        };

        if remaining > us {
            self.self_test.running = Some((gyro, remaining - us));
            return;
        }

        self.self_test.running = None;
        self.self_test.done = true;
        let (accel_mg, gyro_mdps) = self.self_test.response;
        // Accelerometer: 1/2^11 g per LSB (15.1). Gyroscope: 1/2^4 dps per LSB (15.2).
        let raw = if gyro {
            gyro_mdps.map(|v| (i64::from(v) * 16 / 1000) as i16)
        } else {
            accel_mg.map(|v| (i64::from(v) * 2048 / 1000) as i16)
        };

        self.set_le_u16s(Reg::DVX_L, &raw.map(|v| v as u16));
    }
}
