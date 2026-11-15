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

#[test]
fn ctrl9_command_handshake() {
    use qmi8658a::registers::{Ctrl8, StatusInt};
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = Qmi8658a::new(FakeI2c::new(&model));
    imu.init(&mut FakeDelay::new(&model)).unwrap();
    assert!(Ctrl8::from(model.borrow().register(RegisterAddress::CTRL8)).ctrl9_handshake_type());

    imu.set_interrupt_outputs(true, false).unwrap();
    imu.ctrl9_command(
        qmi8658a::Ctrl9Command::CopyUsid,
        &mut FakeDelay::new(&model),
        qmi8658a::CTRL9_TIMEOUT_MS,
    )
    .unwrap();
    assert!(!imu.read_register::<StatusInt>().unwrap().cmd_done());
    assert!(!model.borrow().int1_pin());
}

#[test]
fn ctrl9_command_times_out() {
    let mut imu = Qmi8658a::new(Constant(0x00));
    assert_eq!(
        imu.ctrl9_command(qmi8658a::Ctrl9Command::ResetFifo, &mut NoDelay, 5),
        Err(Error::Timeout)
    );
}

#[test]
fn fifo_drain_and_frames() {
    use qmi8658a::{FifoMode, FifoSize};
    let model = RefCell::new(Qmi8658aModel::new());
    model.borrow_mut().set_acceleration_mg([0, 0, 1000]);
    let mut imu = Qmi8658a::new(FakeI2c::new(&model));
    let mut delay = FakeDelay::new(&model);
    imu.init(&mut delay).unwrap();
    imu.set_fifo(FifoMode::Fifo, FifoSize::Samples16, 4)
        .unwrap();
    imu.set_fifo_interrupt_pin(true).unwrap();
    imu.set_interrupt_outputs(true, false).unwrap();
    imu.set_accel(AccelFullScale::G4, AccelOdr::Hz897).unwrap();
    imu.set_sensors(true, false).unwrap();
    model.borrow_mut().tick(5_000);

    let state = imu.fifo_state().unwrap();
    assert_eq!(state.bytes, 30);
    assert!(state.status.fifo_wtm() && state.status.fifo_not_empty());
    assert!(model.borrow().int1_pin());

    // A 16-byte buffer holds two whole samples.
    let mut buffer = [0; 16];
    assert_eq!(imu.read_fifo(&mut buffer, &mut delay).unwrap(), 12);
    let ctrl1 = Ctrl1::from(model.borrow().register(RegisterAddress::CTRL1));
    assert!(ctrl1.addr_ai());
    assert_eq!(
        model.borrow().register(RegisterAddress::FIFO_CTRL) & 0x80,
        0
    );
    let frames = imu.fifo_frames(&buffer[..12]).unwrap();
    for frame in frames {
        assert_eq!(frame.acceleration.unwrap().g(), [0.0, 0.0, 1.0]);
        assert_eq!(frame.angular_rate, None);
    }

    assert_eq!(imu.fifo_state().unwrap().bytes, 18);
    imu.reset_fifo(&mut delay).unwrap();
    assert_eq!(imu.fifo_state().unwrap().bytes, 0);
    assert_eq!(imu.read_fifo(&mut buffer, &mut delay).unwrap(), 0);
}

#[test]
fn fifo_six_axis_frames() {
    use qmi8658a::{FifoMode, FifoSize};
    let model = RefCell::new(Qmi8658aModel::new());
    model.borrow_mut().set_acceleration_mg([500, 0, 0]);
    model.borrow_mut().set_angular_rate_mdps([0, -90_000, 0]);
    let mut imu = running(&model);
    let mut delay = FakeDelay::new(&model);
    imu.set_fifo(FifoMode::Stream, FifoSize::Samples32, 0)
        .unwrap();
    model.borrow_mut().tick(100_000);
    assert_eq!(imu.fifo_state().unwrap().bytes, 5 * 12);

    let mut buffer = [0; 30];
    assert_eq!(imu.read_fifo(&mut buffer, &mut delay).unwrap(), 24);
    let mut frames = imu.fifo_frames(&buffer[..24]).unwrap();
    let frame = frames.next().unwrap();
    assert_eq!(frame.acceleration.unwrap().g(), [0.5, 0.0, 0.0]);
    assert_eq!(
        frame.angular_rate.unwrap().degrees_per_second(),
        [0.0, -90.0, 0.0]
    );
    assert_eq!(frames.count(), 1);
}

#[test]
fn configure_engines_restores_sensors() {
    use qmi8658a::{MotionConfig, PedometerConfig, TapConfig};
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    let mut delay = FakeDelay::new(&model);
    let motion = MotionConfig {
        any_motion_threshold: [16; 3],
        any_motion_axes: [true; 3],
        any_motion_window: 2,
        ..MotionConfig::default()
    };
    imu.configure_motion(&motion, &mut delay).unwrap();
    imu.configure_tap(&TapConfig::EXAMPLE_500HZ, &mut delay)
        .unwrap();
    imu.configure_pedometer(&PedometerConfig::EXAMPLE_50HZ, &mut delay)
        .unwrap();

    // The model ignores parameters sent while a sensor runs.
    assert_eq!(model.borrow().motion_parameters(), motion.cal());
    assert_eq!(
        model.borrow().tap_parameters(),
        TapConfig::EXAMPLE_500HZ.cal()
    );
    assert_eq!(
        model.borrow().pedometer_parameters(),
        PedometerConfig::EXAMPLE_50HZ.cal()
    );
    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x03);
}

#[test]
fn any_motion_on_int1() {
    use qmi8658a::MotionConfig;
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = Qmi8658a::new(FakeI2c::new(&model));
    let mut delay = FakeDelay::new(&model);
    imu.init(&mut delay).unwrap();
    let motion = MotionConfig {
        any_motion_threshold: [16; 3],
        any_motion_axes: [true, true, true],
        any_motion_window: 1,
        ..MotionConfig::default()
    };
    imu.configure_motion(&motion, &mut delay).unwrap();
    imu.set_motion_detection(true, false, false).unwrap();
    imu.set_activity_interrupt_pin(true).unwrap();
    imu.set_interrupt_outputs(true, false).unwrap();
    imu.set_accel(AccelFullScale::G4, AccelOdr::Hz897).unwrap();
    imu.set_sensors(true, false).unwrap();

    model.borrow_mut().tick(1_000);
    assert!(!imu.motion_status().unwrap().any_motion());
    model.borrow_mut().set_acceleration_mg([0, 800, 0]);
    model.borrow_mut().tick(1_000);
    assert!(model.borrow().int1_pin());
    assert!(imu.motion_status().unwrap().any_motion());
    model.borrow_mut().tick(1_000);
    assert!(!model.borrow().int1_pin());
}

#[test]
fn tap_and_steps() {
    use qmi8658a::{PedometerConfig, TapConfig};
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    let mut delay = FakeDelay::new(&model);
    imu.configure_tap(&TapConfig::EXAMPLE_500HZ, &mut delay)
        .unwrap();
    imu.configure_pedometer(&PedometerConfig::EXAMPLE_50HZ, &mut delay)
        .unwrap();
    imu.set_tap_detection(true).unwrap();
    imu.set_pedometer(true).unwrap();

    model.borrow_mut().tap(2, true, false);
    model.borrow_mut().step(10);
    let status = imu.motion_status().unwrap();
    assert!(status.tap() && status.pedometer());
    assert!(!imu.motion_status().unwrap().tap());
    let tap = imu.tap_status().unwrap();
    assert_eq!(
        (tap.tap_axis(), tap.tap_num(), tap.tap_polarity()),
        (2, 1, true)
    );
    assert_eq!(imu.step_count().unwrap(), 8);

    imu.reset_step_count(&mut delay).unwrap();
    assert_eq!(imu.step_count().unwrap(), 0);
    model.borrow_mut().step(4);
    assert_eq!(imu.step_count().unwrap(), 4);
    imu.set_pedometer(false).unwrap();
    imu.set_pedometer(true).unwrap();
    assert_eq!(imu.step_count().unwrap(), 0);
}

#[test]
fn wake_on_motion() {
    use qmi8658a::WomConfig;
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    let mut delay = FakeDelay::new(&model);
    let config = WomConfig {
        threshold_mg: 100,
        int1: true,
        initial_high: false,
        blanking_samples: 0,
    };
    imu.enable_wake_on_motion(
        AccelFullScale::G2,
        AccelOdr::LowPowerHz128,
        &config,
        &mut delay,
    )
    .unwrap();
    imu.set_interrupt_outputs(true, false).unwrap();
    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x01);
    assert_eq!(model.borrow().register(RegisterAddress::CTRL2), 0x0C);

    model.borrow_mut().tick(10_000);
    assert!(!model.borrow().int1_pin());
    model.borrow_mut().set_acceleration_mg([300, 0, 0]);
    model.borrow_mut().tick(8_000);
    assert!(model.borrow().int1_pin());
    assert!(imu.motion_status().unwrap().wom());
    assert!(!model.borrow().int1_pin());

    imu.disable_wake_on_motion(&mut delay).unwrap();
    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x00);
}

#[test]
fn self_tests_pass_and_restore_configuration() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    let mut delay = FakeDelay::new(&model);
    let before = [
        RegisterAddress::CTRL1,
        RegisterAddress::CTRL2,
        RegisterAddress::CTRL3,
        RegisterAddress::CTRL7,
    ]
    .map(|r| model.borrow().register(r));

    let accel = imu.self_test_accel(&mut delay).unwrap();
    assert_eq!(accel.milli_g(), [500.0; 3]);
    assert!(accel.passed());
    let gyro = imu.self_test_gyro(&mut delay).unwrap();
    assert_eq!(gyro.degrees_per_second(), [500.0; 3]);
    assert!(gyro.passed());

    let after = [
        RegisterAddress::CTRL1,
        RegisterAddress::CTRL2,
        RegisterAddress::CTRL3,
        RegisterAddress::CTRL7,
    ]
    .map(|r| model.borrow().register(r));
    assert_eq!(before, after);
}

#[test]
fn self_test_fails_below_the_limits() {
    let model = RefCell::new(Qmi8658aModel::new());
    model
        .borrow_mut()
        .set_self_test_response([250, -150, 300], [400_000, 400_000, -250_000]);
    let mut imu = Qmi8658a::new(FakeI2c::new(&model));
    let mut delay = FakeDelay::new(&model);
    imu.init(&mut delay).unwrap();
    assert!(!imu.self_test_accel(&mut delay).unwrap().passed());
    assert!(!imu.self_test_gyro(&mut delay).unwrap().passed());
}

#[test]
fn self_test_times_out() {
    let mut imu = Qmi8658a::new(Constant(0x00));
    assert_eq!(imu.self_test_gyro(&mut NoDelay), Err(Error::Timeout));
}

#[test]
fn gyro_calibration_and_gains() {
    use qmi8658a_sim::MODEL_GYRO_GAINS;
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    let mut delay = FakeDelay::new(&model);
    let status = imu.calibrate_gyro(&mut delay).unwrap();
    assert_eq!(u8::from(status), 0);
    assert_eq!(imu.gyro_gains().unwrap(), MODEL_GYRO_GAINS);
    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x03);

    imu.apply_gyro_gains([1, 2, 0xFFFF], &mut delay).unwrap();
    assert_eq!(model.borrow().gyro_gains(), [1, 2, 0xFFFF]);
    assert_eq!(model.borrow().register(RegisterAddress::CTRL7), 0x03);

    model.borrow_mut().set_cod_status(0x08);
    assert!(imu.calibrate_gyro(&mut delay).unwrap().accel_check());
}

#[test]
fn host_delta_offsets() {
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = running(&model);
    let mut delay = FakeDelay::new(&model);
    // 0.25 g on Z; -2 dps on X.
    imu.apply_accel_delta_offset([0, 0, 1024], &mut delay)
        .unwrap();
    imu.apply_gyro_delta_offset([-64, 0, 0], &mut delay)
        .unwrap();
    assert_eq!(model.borrow().offsets(), ([0, 0, 250], [-2_000, 0, 0]));

    model.borrow_mut().set_acceleration_mg([0, 0, 1000]);
    model.borrow_mut().tick(18_000);
    let (accel, gyro) = imu.read_motion().unwrap();
    assert_eq!(accel.g(), [0.0, 0.0, 0.75]);
    assert_eq!(gyro.degrees_per_second(), [2.0, 0.0, 0.0]);
}

#[cfg(feature = "async")]
#[test]
fn async_commands_fifo_and_self_test() {
    use embassy_futures::block_on;
    use qmi8658a::{FifoMode, FifoSize, MotionConfig};
    let model = RefCell::new(Qmi8658aModel::new());
    let mut imu = qmi8658a::asynch::Qmi8658a::new(FakeI2c::new(&model));
    let mut delay = FakeDelay::new(&model);
    block_on(async {
        imu.init(&mut delay).await.unwrap();
        assert!(imu.self_test_accel(&mut delay).await.unwrap().passed());
        assert_eq!(u8::from(imu.calibrate_gyro(&mut delay).await.unwrap()), 0);
        imu.configure_motion(&MotionConfig::default(), &mut delay)
            .await
            .unwrap();

        imu.set_fifo(FifoMode::Fifo, FifoSize::Samples16, 0)
            .await
            .unwrap();
        imu.set_accel(AccelFullScale::G2, AccelOdr::Hz897)
            .await
            .unwrap();
        imu.set_sensors(true, false).await.unwrap();
        model.borrow_mut().tick(3_000);
        let mut buffer = [0; 64];
        let len = imu.read_fifo(&mut buffer, &mut delay).await.unwrap();
        assert_eq!(len, 18);
        let frames = imu.fifo_frames(&buffer[..len]).await.unwrap();
        assert_eq!(frames.count(), 3);
    });
}
