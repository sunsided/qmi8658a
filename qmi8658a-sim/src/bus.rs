//! [`fake_bus`] adapters for the [`Qmi8658aModel`].

use crate::Qmi8658aModel;
use core::cell::RefCell;
use fake_bus::{Clock, I2cTarget, SpiTarget};

/// An I²C bus with a single [`Qmi8658aModel`] attached.
///
/// The model is shared through a [`RefCell`] so that tests can change the motion inputs
/// while a driver owns the bus. For a bus shared with other models, use
/// [`fake_bus::FakeI2c`] with an array of targets.
pub type FakeI2c<'a> = fake_bus::FakeI2c<&'a RefCell<Qmi8658aModel>>;

/// An SPI device (bus plus chip select) with a [`Qmi8658aModel`] attached.
///
/// Every transaction is one chip-select cycle. The model does not distinguish between
/// 3-wire and 4-wire mode.
pub type FakeSpi<'a> = fake_bus::FakeSpi<'a, Qmi8658aModel>;

/// A delay provider that advances the time of a [`Qmi8658aModel`] instead of waiting.
pub type FakeDelay<'a> = fake_bus::FakeDelay<'a, Qmi8658aModel>;

impl I2cTarget for Qmi8658aModel {
    fn address(&self) -> u8 {
        Qmi8658aModel::address(self)
    }

    fn start(&mut self, is_read: bool) -> bool {
        self.i2c_start(is_read)
    }

    fn write(&mut self, byte: u8) -> bool {
        self.i2c_write(byte)
    }

    fn read(&mut self) -> u8 {
        self.i2c_read()
    }
}

impl SpiTarget for Qmi8658aModel {
    fn select(&mut self) {
        self.spi_select();
    }

    fn transfer(&mut self, mosi: u8) -> u8 {
        self.spi_transfer(mosi)
    }
}

impl Clock for Qmi8658aModel {
    fn advance_ns(&mut self, ns: u64) {
        for us in self.ticks.advance_u32(ns) {
            self.tick(us);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fake_bus::embedded_hal::delay::DelayNs;
    use fake_bus::embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource, Operation};
    use fake_bus::embedded_hal::spi::SpiDevice;
    use qmi8658a_registers::RegisterAddress;

    #[test]
    fn i2c_write_read() {
        let model = RefCell::new(Qmi8658aModel::new());
        let mut bus = FakeI2c::new(&model);
        let mut id = [0];
        bus.write_read(0x6A, &[0x00], &mut id).unwrap();
        assert_eq!(id, [0x05]);
    }

    #[test]
    fn i2c_alternative_address() {
        let model = RefCell::new(Qmi8658aModel::with_address(0x6B));
        let mut bus = FakeI2c::new(&model);
        assert_eq!(
            bus.write(0x6A, &[0x00]),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        );
        let mut id = [0];
        bus.write_read(0x6B, &[0x00], &mut id).unwrap();
        assert_eq!(id, [0x05]);
    }

    #[test]
    fn i2c_adjacent_writes_form_one_transfer() {
        let model = RefCell::new(Qmi8658aModel::new());
        let mut bus = FakeI2c::new(&model);
        bus.transaction(
            0x6A,
            &mut [Operation::Write(&[0x02]), Operation::Write(&[0x40])],
        )
        .unwrap();
        assert_eq!(model.borrow().register(RegisterAddress::CTRL1), 0x40);
    }

    #[test]
    fn i2c_async_write_read() {
        let model = RefCell::new(Qmi8658aModel::new());
        let mut bus = FakeI2c::new(&model);
        let mut id = [0];
        embassy_futures::block_on(fake_bus::embedded_hal_async::i2c::I2c::write_read(
            &mut bus,
            0x6A,
            &[0x00],
            &mut id,
        ))
        .unwrap();
        assert_eq!(id, [0x05]);
    }

    #[test]
    fn spi_read_who_am_i() {
        let model = RefCell::new(Qmi8658aModel::new());
        let mut spi = FakeSpi::new(&model);
        let mut buf = [0x80, 0];
        spi.transfer_in_place(&mut buf).unwrap();
        assert_eq!(buf[1], 0x05);
    }

    #[test]
    fn spi_write_then_read_burst() {
        use fake_bus::embedded_hal::spi::Operation;
        let model = RefCell::new(Qmi8658aModel::new());
        let mut spi = FakeSpi::new(&model);
        spi.write(&[0x02, 0x40]).unwrap();
        spi.write(&[0x0B, 1]).unwrap();
        spi.write(&[0x0C, 2]).unwrap();

        let mut buf = [0; 2];
        spi.transaction(&mut [Operation::Write(&[0x80 | 0x0B]), Operation::Read(&mut buf)])
            .unwrap();
        assert_eq!(buf, [1, 2]);
    }

    #[test]
    fn delay_advances_the_model() {
        let model = RefCell::new(Qmi8658aModel::new());
        model.borrow_mut().i2c_start(false);
        model.borrow_mut().i2c_write(0x03);
        model.borrow_mut().i2c_write(0x03);
        model.borrow_mut().i2c_start(false);
        model.borrow_mut().i2c_write(0x08);
        model.borrow_mut().i2c_write(0x01);
        let mut delay = FakeDelay::new(&model);

        // 1 ms at 1 kHz in steps of 999 ns plus one short step: no time may get lost.
        for _ in 0..1_001 {
            delay.delay_ns(999);
        }
        delay.delay_ns(1);
        assert_eq!(model.borrow().register(RegisterAddress::STATUS0), 0x01);
    }
}
