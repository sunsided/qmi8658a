//! Reads acceleration, angular rate and temperature from a QMI8658A and prints them over the
//! serial console whenever they change.
//!
//! Wiring: SDA = GPIO4, SCL = GPIO5. The firmware polls `STATUS0` for new data and needs
//! no interrupt line, as on the Kode Dot, where INT2 is not connected.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::Rate;
use esp_println::println;
use qmi8658a::{AccelFullScale, AccelOdr, GyroFullScale, GyroOdr, Qmi8658a};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut delay = Delay::new();

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("valid I2C config")
    .with_sda(peripherals.GPIO4)
    .with_scl(peripherals.GPIO5);

    // The device accepts no writes during its turn-on time after power-up.
    delay.delay_millis(qmi8658a::TURN_ON_TIME_MS);
    let mut imu = Qmi8658a::new(i2c);
    let who_am_i = imu.who_am_i().expect("device responds");
    let revision = imu.revision_id().expect("device responds");
    println!("QMI8658A ok who_am_i=0x{who_am_i:02X} revision=0x{revision:02X}");

    imu.init(&mut delay).expect("device initializes");
    // A game reading 30 to 60 times a second needs no more than 56 Hz.
    imu.set_accel(AccelFullScale::G8, AccelOdr::Hz56).unwrap();
    imu.set_gyro(GyroFullScale::Dps512, GyroOdr::Hz56).unwrap();
    // The self-test runs at the configured accelerometer ODR, with the sensors off.
    let accel = imu.self_test_accel(&mut delay).expect("self-test runs");
    let gyro = imu.self_test_gyro(&mut delay).expect("self-test runs");
    println!("self-test accel={} gyro={}", accel.passed(), gyro.passed());

    imu.set_sensors(true, true).unwrap();
    println!("running accel=8g gyro=512dps odr=56Hz");

    let mut last = None;
    loop {
        let status = imu.status().unwrap();
        if status.a_da() || status.g_da() {
            let (accel, gyro) = imu.read_motion().unwrap();
            let temperature = imu.read_temperature().unwrap();
            if last != Some((accel.raw, gyro.raw, temperature.raw)) {
                let [ax, ay, az] = accel.g();
                let [gx, gy, gz] = gyro.degrees_per_second();
                println!(
                    "accel x={ax:.2} y={ay:.2} z={az:.2} g gyro x={gx:.1} y={gy:.1} z={gz:.1} dps temp={:.1}C",
                    temperature.celsius()
                );
                last = Some((accel.raw, gyro.raw, temperature.raw));
            }
        }

        delay.delay_millis(5);
    }
}
