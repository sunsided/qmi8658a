//! FIFO level and frame decoding (datasheet section 8).

use crate::data::i16x3;
use crate::{Acceleration, AngularRate};
use qmi8658a_registers::{AccelFullScale, FifoStatus, GyroFullScale};

/// The FIFO status and fill level.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoState {
    /// `FIFO_STATUS`: full, watermark, overflow and not-empty flags.
    pub status: FifoStatus,
    /// The fill level in bytes: `2 * (FIFO_SMPL_CNT + 256 * FIFO_STATUS[1:0])` (datasheet 8.4).
    pub bytes: u16,
}

impl FifoState {
    /// Decodes `FIFO_SMPL_CNT` and `FIFO_STATUS`.
    pub const fn from_bytes(bytes: [u8; 2]) -> Self {
        let status = FifoStatus::from_bits(bytes[1]);
        let words = bytes[0] as u16 | ((status.fifo_smpl_cnt_msb() as u16) << 8);
        Self {
            status,
            bytes: 2 * words,
        }
    }
}

/// One FIFO sample: accelerometer and/or gyroscope values, depending on which sensors
/// were enabled (datasheet 8.9).
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoFrame {
    /// The accelerometer values, if the accelerometer is enabled.
    pub acceleration: Option<Acceleration>,
    /// The gyroscope values, if the gyroscope is enabled.
    pub angular_rate: Option<AngularRate>,
}

/// Decodes FIFO data into [`FifoFrame`]s.
///
/// A frame holds 6 bytes per enabled sensor, accelerometer first (datasheet 8.9).
/// Trailing bytes that do not fill a frame are ignored.
#[derive(Debug, Clone)]
pub struct FifoFrames<'a> {
    chunks: core::slice::ChunksExact<'a, u8>,
    accel: Option<AccelFullScale>,
    gyro: Option<GyroFullScale>,
}

impl<'a> FifoFrames<'a> {
    /// Decodes `data` with the full scale of each enabled sensor, or `None` for a disabled one.
    pub fn new(data: &'a [u8], accel: Option<AccelFullScale>, gyro: Option<GyroFullScale>) -> Self {
        let frame = Self::frame_len(accel.is_some(), gyro.is_some()).max(1);
        // With no sensor enabled, there are no frames.
        let data = if accel.is_none() && gyro.is_none() {
            &data[..0]
        } else {
            data
        };

        Self {
            chunks: data.chunks_exact(frame),
            accel,
            gyro,
        }
    }

    /// The bytes per frame: 6 per enabled sensor.
    pub const fn frame_len(accel: bool, gyro: bool) -> usize {
        6 * (accel as usize + gyro as usize)
    }
}

impl Iterator for FifoFrames<'_> {
    type Item = FifoFrame;

    fn next(&mut self) -> Option<Self::Item> {
        let chunk = self.chunks.next()?;
        let (accel_bytes, gyro_bytes) = chunk.split_at(if self.accel.is_some() { 6 } else { 0 });
        Some(FifoFrame {
            acceleration: self.accel.map(|scale| Acceleration {
                raw: i16x3(accel_bytes),
                scale,
            }),
            angular_rate: self.gyro.map(|scale| AngularRate {
                raw: i16x3(gyro_bytes),
                scale,
            }),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state() {
        let state = FifoState::from_bytes([0x10, 0b1101_0011]);
        assert_eq!(state.bytes, 2 * 0x310);
        assert!(state.status.fifo_full() && state.status.fifo_wtm());
    }

    #[test]
    fn frames() {
        let data = [0x00, 0x10, 0, 0, 0, 0, 0x40, 0, 0, 0, 0, 0, 0xFF];
        let mut frames =
            FifoFrames::new(&data, Some(AccelFullScale::G8), Some(GyroFullScale::Dps512));
        let frame = frames.next().unwrap();
        assert!(frames.next().is_none());
        assert_eq!(frame.acceleration.unwrap().g(), [1.0, 0.0, 0.0]);
        assert_eq!(
            frame.angular_rate.unwrap().degrees_per_second(),
            [1.0, 0.0, 0.0]
        );

        let mut gyro_only = FifoFrames::new(&data, None, Some(GyroFullScale::Dps16));
        assert_eq!(gyro_only.next().unwrap().acceleration, None);
        assert_eq!(
            gyro_only.next().unwrap().angular_rate.unwrap().raw,
            [64, 0, 0]
        );
        assert!(gyro_only.next().is_none());
        assert_eq!(FifoFrames::new(&data, None, None).count(), 0);
    }
}
