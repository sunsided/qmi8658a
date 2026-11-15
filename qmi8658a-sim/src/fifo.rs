//! The FIFO (datasheet section 8).

use crate::Qmi8658aModel;
use qmi8658a_registers::{FifoCtrl, FifoMode, RegisterAddress as Reg};

/// The FIFO memory, 1536 bytes (datasheet 8.1).
const FIFO_BYTES: usize = 1536;

/// The FIFO contents: a ring buffer of bytes.
#[derive(Debug, Clone)]
pub(crate) struct Fifo {
    data: [u8; FIFO_BYTES],
    start: usize,
    len: usize,
    overflow: bool,
}

impl Fifo {
    pub fn new() -> Self {
        Self {
            data: [0; FIFO_BYTES],
            start: 0,
            len: 0,
            overflow: false,
        }
    }

    fn push_byte(&mut self, byte: u8) {
        self.data[(self.start + self.len) % FIFO_BYTES] = byte;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }

        let byte = self.data[self.start];
        self.start = (self.start + 1) % FIFO_BYTES;
        self.len -= 1;
        Some(byte)
    }
}

impl Qmi8658aModel {
    /// The bytes per FIFO sample: 12 with both sensors enabled, otherwise 6 (datasheet 8.2).
    fn fifo_sample_bytes(&self) -> usize {
        let ctrl7 = self.ctrl7();
        if ctrl7.a_en() && ctrl7.g_en() { 12 } else { 6 }
    }

    /// The FIFO size in bytes, from `FIFO_CTRL.FIFO_SIZE`.
    fn fifo_capacity(&self) -> usize {
        let size = FifoCtrl::from(self.register(Reg::FIFO_CTRL)).fifo_size();
        (16 << size.into_bits()) * self.fifo_sample_bytes()
    }

    /// Whether the FIFO level reached the non-zero watermark (datasheet 8.5).
    pub(crate) fn fifo_watermark_hit(&self) -> bool {
        let watermark = usize::from(self.register(Reg::FIFO_WTM_TH));
        self.fifo_enabled()
            && watermark != 0
            && self.fifo.len / self.fifo_sample_bytes() >= watermark
    }

    /// Appends a sample: accelerometer or gyroscope values, or both. Outside FIFO and
    /// stream mode, and in read mode (datasheet 8.7), the sample is dropped.
    pub(crate) fn fifo_push(&mut self, first: &[i16; 3], second: Option<&[i16; 3]>) {
        let ctrl = FifoCtrl::from(self.register(Reg::FIFO_CTRL));
        if !self.fifo_enabled() || ctrl.fifo_rd_mode() {
            return;
        }

        let bytes = 6 * (1 + usize::from(second.is_some()));
        let capacity = self.fifo_capacity();
        if self.fifo.len + bytes > capacity {
            self.fifo.overflow = true;
            if ctrl.fifo_mode() != FifoMode::Stream {
                self.fifo_update_registers();
                return;
            }

            while self.fifo.len + bytes > capacity {
                self.fifo.pop();
            }
        }

        for value in first.iter().chain(second.into_iter().flatten()) {
            for byte in value.to_le_bytes() {
                self.fifo.push_byte(byte);
            }
        }

        self.fifo_update_registers();
    }

    /// Reads `FIFO_DATA`: removes a byte in read mode, otherwise reads `0`.
    pub(crate) fn fifo_read(&mut self) -> u8 {
        if !FifoCtrl::from(self.register(Reg::FIFO_CTRL)).fifo_rd_mode() {
            return 0;
        }

        let byte = self.fifo.pop().unwrap_or(0);
        self.fifo_update_registers();
        byte
    }

    /// Empties the FIFO and clears its flags (`CTRL_CMD_RST_FIFO`).
    pub(crate) fn fifo_reset(&mut self) {
        self.fifo = Fifo::new();
        self.fifo_update_registers();
    }

    /// Updates `FIFO_SMPL_CNT` and `FIFO_STATUS` from the FIFO contents.
    pub(crate) fn fifo_update_registers(&mut self) {
        let words = self.fifo.len / 2;
        // FIFO_FULL (7), FIFO_WTM (6), FIFO_OVFLOW (5), FIFO_NOT_EMPTY (4), count MSBs (1:0).
        let status = (u8::from(self.fifo.len >= self.fifo_capacity()) << 7)
            | (u8::from(self.fifo_watermark_hit()) << 6)
            | (u8::from(self.fifo.overflow) << 5)
            | (u8::from(self.fifo.len > 0) << 4)
            | ((words >> 8) as u8 & 0b11);
        self.set(Reg::FIFO_SMPL_CNT, words as u8);
        self.set(Reg::FIFO_STATUS, status);
    }
}
