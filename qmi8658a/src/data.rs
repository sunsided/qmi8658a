//! Measurement values and their decoding from register bytes.

use qmi8658a_registers::{AccelFullScale, GyroFullScale, TEMPERATURE_LSB_PER_CELSIUS};

/// Decodes three little-endian `i16` values.
pub(crate) const fn i16x3(bytes: &[u8]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

/// An accelerometer sample.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Acceleration {
    /// The raw X, Y and Z outputs.
    pub raw: [i16; 3],
    /// The full scale the sample was taken at.
    pub scale: AccelFullScale,
}

impl Acceleration {
    /// Decodes the six output bytes `AX_L` to `AZ_H` (little-endian, `CTRL1.BE = 0`).
    pub const fn from_bytes(bytes: &[u8; 6], scale: AccelFullScale) -> Self {
        Self {
            raw: i16x3(bytes),
            scale,
        }
    }

    /// Returns the X, Y and Z acceleration in g.
    pub fn g(&self) -> [f32; 3] {
        self.raw.map(|v| f32::from(v) / self.scale.lsb_per_g())
    }
}

/// A gyroscope sample.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AngularRate {
    /// The raw X, Y and Z outputs.
    pub raw: [i16; 3],
    /// The full scale the sample was taken at.
    pub scale: GyroFullScale,
}

impl AngularRate {
    /// Decodes the six output bytes `GX_L` to `GZ_H` (little-endian, `CTRL1.BE = 0`).
    pub const fn from_bytes(bytes: &[u8; 6], scale: GyroFullScale) -> Self {
        Self {
            raw: i16x3(bytes),
            scale,
        }
    }

    /// Returns the X, Y and Z angular rate in degrees per second.
    pub fn degrees_per_second(&self) -> [f32; 3] {
        self.raw.map(|v| f32::from(v) / self.scale.lsb_per_dps())
    }
}

/// A temperature sample.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Temperature {
    /// The raw output (256 LSB per °C, `0` at 0 °C).
    pub raw: i16,
}

impl Temperature {
    /// Decodes `TEMP_L` and `TEMP_H` (little-endian, `CTRL1.BE = 0`).
    pub const fn from_bytes(bytes: [u8; 2]) -> Self {
        Self {
            raw: i16::from_le_bytes(bytes),
        }
    }

    /// Returns the temperature in °C: `TEMP_H + TEMP_L / 256` (datasheet 3.9).
    pub fn celsius(&self) -> f32 {
        f32::from(self.raw) / TEMPERATURE_LSB_PER_CELSIUS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn acceleration() {
        let a = Acceleration::from_bytes(&[0x00, 0x10, 0x00, 0xF0, 0x00, 0x00], AccelFullScale::G8);
        assert_eq!(a.raw, [4096, -4096, 0]);
        assert_eq!(a.g(), [1.0, -1.0, 0.0]);
        let a = Acceleration::from_bytes(&[0x00, 0x40, 0, 0, 0, 0], AccelFullScale::G2);
        assert_eq!(a.g()[0], 1.0);
    }

    #[test]
    fn angular_rate() {
        let r = AngularRate::from_bytes(&[0x80, 0xE9, 0x40, 0x00, 0, 0], GyroFullScale::Dps512);
        assert_eq!(r.raw, [-5760, 64, 0]);
        assert_eq!(r.degrees_per_second(), [-90.0, 1.0, 0.0]);
    }

    #[test]
    fn temperature() {
        assert_eq!(Temperature::from_bytes([0x80, 0x19]).celsius(), 25.5);
        assert_eq!(Temperature::from_bytes([0x00, 0xFF]).celsius(), -1.0);
    }
}
