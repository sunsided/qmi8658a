//! Self-test (check-alive) results and limits (datasheet section 15).

/// The minimum accelerometer self-test response per axis, in mg (datasheet 15.1).
pub const ACCEL_SELF_TEST_MIN_MG: f32 = 200.0;

/// The minimum gyroscope self-test response per axis, in dps (datasheet 15.2).
pub const GYRO_SELF_TEST_MIN_DPS: f32 = 300.0;

/// The time each half of a self-test may take, in milliseconds.
///
/// The datasheet gives about 25 ODRs for the accelerometer (2.2 s at 11 Hz) and about
/// 400 ms for the gyroscope.
pub const SELF_TEST_TIMEOUT_MS: u32 = 3_000;

/// The accelerometer self-test response, from `dVX_L` to `dVZ_H` (datasheet 15.1).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelSelfTest {
    /// The raw X, Y and Z response, at 1/2048 g per LSB.
    pub raw: [i16; 3],
}

impl AccelSelfTest {
    /// The response in mg.
    pub fn milli_g(&self) -> [f32; 3] {
        self.raw.map(|v| f32::from(v) * 1000.0 / 2048.0)
    }

    /// Whether every axis responded with more than [`ACCEL_SELF_TEST_MIN_MG`].
    pub fn passed(&self) -> bool {
        self.milli_g()
            .iter()
            .all(|v| v.abs() > ACCEL_SELF_TEST_MIN_MG)
    }
}

/// The gyroscope self-test response, from `dVX_L` to `dVZ_H` (datasheet 15.2).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroSelfTest {
    /// The raw X, Y and Z response, at 1/16 dps per LSB.
    pub raw: [i16; 3],
}

impl GyroSelfTest {
    /// The response in degrees per second.
    pub fn degrees_per_second(&self) -> [f32; 3] {
        self.raw.map(|v| f32::from(v) / 16.0)
    }

    /// Whether every axis responded with more than [`GYRO_SELF_TEST_MIN_DPS`].
    pub fn passed(&self) -> bool {
        self.degrees_per_second()
            .iter()
            .all(|v| v.abs() > GYRO_SELF_TEST_MIN_DPS)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limits() {
        let accel = AccelSelfTest {
            raw: [2048, -411, 409],
        };
        assert_eq!(accel.milli_g()[0], 1000.0);
        assert!(!accel.passed());
        assert!(
            AccelSelfTest {
                raw: [411, -411, 411]
            }
            .passed()
        );

        let gyro = GyroSelfTest {
            raw: [4816, -4816, 4800],
        };
        assert_eq!(gyro.degrees_per_second()[2], 300.0);
        assert!(!gyro.passed());
        assert!(GyroSelfTest { raw: [4816; 3] }.passed());
    }
}
