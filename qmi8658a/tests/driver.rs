//! Driver tests against the behavioral model.

use core::cell::RefCell;
use qmi8658a::registers::{Ctrl1, Ctrl7, RegisterAddress};
use qmi8658a::{
    AccelFullScale, AccelOdr, Error, GyroFullScale, GyroOdr, QMI8658A_WHO_AM_I, Qmi8658a,
};
use qmi8658a_sim::{FakeDelay, FakeI2c, MODEL_REVISION_ID, Qmi8658aModel};

/// A driver on a fresh model, initialised, with both sensors at ±8 g / ±512 dps, 56 Hz.
fn running(model: &RefCell<Qmi8658aModel>) -> Qmi8658a<qmi8658a::I2cInterface<FakeI2c<'_>>> {
    let mut imu = Qmi8658a::new(FakeI2c::new(model));
    imu.init(&mut FakeDelay::new(model)).unwrap();
    imu.set_accel(AccelFullScale::G8, AccelOdr::Hz56).unwrap();
    imu.set_gyro(GyroFullScale::Dps512, GyroOdr::Hz56).unwrap();
    imu.set_sensors(true, true).unwrap();
    imu
}

#[test]
fn identity() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = Qmi8658a::new(FakeI2c::new(&model));
    assert_eq!(imu.who_am_i().unwrap(), QMI8658A_WHO_AM_I);
    assert!(imu.identify().unwrap());
    assert_eq!(imu.revision_id().unwrap(), MODEL_REVISION_ID);
}

#[test]
fn init_resets_and_configures_the_interface() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = Qmi8658a::new(FakeI2c::new(&model));
    imu.set_sensors(true, true).unwrap();
    imu.init(&mut FakeDelay::new(&model)).unwrap();

    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x00);
    let ctrl1 = Ctrl1::from(model.borrow().register(RegisterAddress::CTRL1));
    assert!(ctrl1.addr_ai() && !ctrl1.be());
    assert_eq!(imu.accel_full_scale().unwrap(), AccelFullScale::G2);
    assert_eq!(imu.gyro_full_scale().unwrap(), GyroFullScale::Dps16);
}

/// An I²C bus on which every read returns the same byte.
struct Constant(u8);

impl embedded_hal::i2c::ErrorType for Constant {
    type Error = embedded_hal::i2c::ErrorKind;
}

impl embedded_hal::i2c::I2c for Constant {
    fn transaction(
        &mut self,
        _address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            if let embedded_hal::i2c::Operation::Read(buffer) = op {
                buffer.fill(self.0);
            }
        }

        Ok(())
    }
}

struct NoDelay;

impl embedded_hal::delay::DelayNs for NoDelay {
    fn delay_ns(&mut self, _: u32) {}
}

#[test]
fn init_rejects_other_devices() {
    let mut imu = Qmi8658a::new(Constant(0x68));
    assert!(!imu.identify().unwrap());
    assert_eq!(imu.init(&mut NoDelay), Err(Error::WrongIdentity(0x68)));
}

#[test]
fn init_fails_on_a_missing_device() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = Qmi8658a::with_address(FakeI2c::new(&model), 0x6B);
    assert!(matches!(imu.init(&mut NoDelay), Err(Error::Bus(_))));
}

#[test]
fn reset_times_out_without_reset_status() {
    let mut imu = Qmi8658a::new(Constant(0x00));
    assert_eq!(imu.reset(&mut NoDelay), Err(Error::Timeout));
}

#[test]
fn alternative_address() {
    let model = RefCell::new(Qmi8658aModel::with_address(
        qmi8658a::ALTERNATIVE_DEVICE_ADDRESS,
    ));
    let mut imu = Qmi8658a::with_address(FakeI2c::new(&model), 0x6B);
    imu.init(&mut FakeDelay::new(&model)).unwrap();
    let _bus = imu.release();
}

#[test]
fn reset_restores_defaults() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    imu.reset(&mut FakeDelay::new(&model)).unwrap();
    assert_eq!(model.borrow().register(RegisterAddress::CTRL1), 0x20);
    assert_eq!(model.borrow().register(RegisterAddress::CTRL2), 0x00);
    assert_eq!(imu.accel_full_scale().unwrap(), AccelFullScale::G2);
}

#[test]
fn configuration() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    assert_eq!(model.borrow().register(RegisterAddress::CTRL2), 0x27);
    assert_eq!(model.borrow().register(RegisterAddress::CTRL3), 0x57);
    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x03);
    assert_eq!(imu.accel_full_scale().unwrap(), AccelFullScale::G8);
    assert_eq!(imu.gyro_full_scale().unwrap(), GyroFullScale::Dps512);

    imu.set_sensors(true, false).unwrap();
    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x01);
}

#[test]
fn full_scale_is_read_back_by_a_new_driver() {
    let model = RefCell::new(Qmi8658aModel::new());
    let imu = running(&model);
    let mut imu = Qmi8658a::new(imu.release());
    assert_eq!(imu.accel_full_scale().unwrap(), AccelFullScale::G8);
    assert_eq!(imu.gyro_full_scale().unwrap(), GyroFullScale::Dps512);
}

#[test]
fn reads_motion_and_temperature() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    model.borrow_mut().set_acceleration_mg([1000, -500, 250]);
    model
        .borrow_mut()
        .set_angular_rate_mdps([-90_000, 0, 512_000]);
    model.borrow_mut().set_temperature(25 * 256 + 64);

    assert!(!imu.status().unwrap().a_da());
    model.borrow_mut().tick(18_000);
    let status = imu.status().unwrap();
    assert!(status.a_da() && status.g_da());
    assert!(!imu.status().unwrap().a_da());

    let accel = imu.read_acceleration().unwrap();
    assert_eq!(accel.raw, [4096, -2048, 1024]);
    assert_eq!(accel.g(), [1.0, -0.5, 0.25]);
    let gyro = imu.read_angular_rate().unwrap();
    assert_eq!(gyro.degrees_per_second(), [-90.0, 0.0, 511.984_38]);
    assert_eq!(imu.read_motion().unwrap(), (accel, gyro));
    assert_eq!(imu.read_temperature().unwrap().celsius(), 25.25);
}

#[test]
fn burst_reads_need_the_interface_configuration() {
    let model = RefCell::new(Qmi8658aModel::new());
    model.borrow_mut().set_acceleration_mg([1000, 0, 0]);
    let mut imu = Qmi8658a::new(FakeI2c::new(&model));
    imu.set_accel(AccelFullScale::G2, AccelOdr::Hz897).unwrap();
    imu.set_sensors(true, false).unwrap();
    model.borrow_mut().tick(1_000);
    // No auto-increment, big-endian: the burst repeats AX_L (0x40, the high byte of 16384).
    assert_eq!(imu.read_acceleration().unwrap().raw, [0x4040; 3]);

    imu.configure_interface().unwrap();
    model.borrow_mut().tick(1_000);
    assert_eq!(imu.read_acceleration().unwrap().raw, [16384, 0, 0]);
}

#[test]
fn data_ready_on_int2() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    model.borrow_mut().tick(18_000);
    assert!(!model.borrow().int2_pin());
    assert!(!imu.status_int().unwrap().avail());

    imu.set_interrupt_outputs(false, true).unwrap();
    model.borrow_mut().tick(18_000);
    assert!(model.borrow().int2_pin());
    assert!(imu.status_int().unwrap().avail());

    imu.set_data_ready_interrupt(false).unwrap();
    assert!(Ctrl7::from(model.borrow().register(RegisterAddress::CTRL7)).drdy_dis());
    model.borrow_mut().tick(18_000);
    assert!(!model.borrow().int2_pin());

    imu.set_data_ready_interrupt(true).unwrap();
    imu.set_interrupt_outputs(true, false).unwrap();
    let ctrl1 = Ctrl1::from(model.borrow().register(RegisterAddress::CTRL1));
    assert!(ctrl1.int1_en() && !ctrl1.int2_en() && ctrl1.addr_ai());
}

#[test]
fn spi_interface() {
    use qmi8658a_sim::FakeSpi;
    let model = RefCell::new(Qmi8658aModel::new());
    model.borrow_mut().set_acceleration_mg([0, 0, -1000]);
    let mut imu = Qmi8658a::new_spi(FakeSpi::new(&model));
    imu.init(&mut FakeDelay::new(&model)).unwrap();
    imu.set_accel(AccelFullScale::G4, AccelOdr::Hz897).unwrap();
    imu.set_sensors(true, false).unwrap();
    model.borrow_mut().tick(1_000);
    assert_eq!(imu.read_acceleration().unwrap().g(), [0.0, 0.0, -1.0]);
    let _spi = imu.release();
}

#[cfg(feature = "async")]
#[test]
fn async_driver() {
    use embassy_futures::block_on;
    let model = RefCell::new(Qmi8658aModel::new());
    model
        .borrow_mut()
        .set_angular_rate_mdps([10_000, 20_000, -30_000]);
    let mut imu = qmi8658a::asynch::Qmi8658a::new(FakeI2c::new(&model));
    block_on(async {
        imu.init(&mut FakeDelay::new(&model)).await.unwrap();
        assert!(imu.identify().await.unwrap());
        imu.set_gyro(GyroFullScale::Dps32, GyroOdr::Hz112)
            .await
            .unwrap();
        imu.set_sensors(false, true).await.unwrap();
        model.borrow_mut().tick(10_000);
        assert!(imu.status().await.unwrap().g_da());
        assert_eq!(
            imu.read_angular_rate().await.unwrap().degrees_per_second(),
            [10.0, 20.0, -30.0]
        );
    });
}
