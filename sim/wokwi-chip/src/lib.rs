//! Wokwi custom chip wrapping the `qmi8658a-sim` behavioral model.
//!
//! Pins: `VCC`, `GND`, `SDA`, `SCL` (I²C at `0x6A`), `INT1` and `INT2`.
//!
//! Controls (also settable as attributes and via `set-control` in scenarios): `accelX`,
//! `accelY`, `accelZ` in mg, `gyroX`, `gyroY`, `gyroZ` in dps and `temperature` in °C.
//! They are sampled every [`TICK_US`], which is also the time step of the model.

use qmi8658a_sim::Qmi8658aModel;
use std::cell::RefCell;
use std::ffi::{CString, c_void};
use wokwi_chip_ll::*;

/// The model's time step and control sampling period, in microseconds.
const TICK_US: u32 = 1_000;

struct Chip {
    model: Qmi8658aModel,
    int1: PinId,
    int2: PinId,
    accel: [AttrId; 3],
    gyro: [AttrId; 3],
    temperature: AttrId,
}

thread_local! {
    static CHIP: RefCell<Option<Chip>> = const { RefCell::new(None) };
}

fn with_chip<R>(f: impl FnOnce(&mut Chip) -> R) -> R {
    CHIP.with(|chip| f(chip.borrow_mut().as_mut().expect("chipInit not called")))
}

fn cstr(s: &str) -> CString {
    CString::new(s).expect("no interior NUL")
}

fn debug(message: &str) {
    let message = cstr(message);
    // SAFETY: `message` is a valid NUL-terminated string for the duration of the call.
    unsafe { debugPrint(message.as_ptr()) };
}

/// Converts a control value to an integer in thousandths (mg to mg, dps to mdps).
fn milli(value: f32, per_unit: f32) -> i32 {
    (value * per_unit).round() as i32
}

/// Converts °C to the temperature register (256 LSB per °C).
fn celsius_to_lsb(celsius: f32) -> i16 {
    (celsius * 256.0).round() as i16
}

impl Chip {
    fn update_pins(&self) {
        let level = |high: bool| if high { HIGH } else { LOW };
        // SAFETY: The pin IDs were obtained from the simulator in `chipInit`.
        unsafe {
            pinWrite(self.int1, level(self.model.int1_pin()));
            pinWrite(self.int2, level(self.model.int2_pin()));
        }
    }

    fn tick(&mut self) {
        // SAFETY: Attribute IDs were obtained from the simulator in `chipInit`.
        // `attrRead` returns `u32`, which cannot hold negative slider values.
        let (accel, gyro, t) = unsafe {
            (
                self.accel.map(|id| attrReadFloat(id) as f32),
                self.gyro.map(|id| attrReadFloat(id) as f32),
                attrReadFloat(self.temperature) as f32,
            )
        };

        self.model.set_acceleration_mg(accel.map(|v| milli(v, 1.0)));
        self.model
            .set_angular_rate_mdps(gyro.map(|v| milli(v, 1000.0)));
        self.model.set_temperature(celsius_to_lsb(t));
        self.model.tick(TICK_US);
        self.update_pins();
    }
}

extern "C" fn on_connect(_: *mut c_void, _address: u32, is_read: bool) -> bool {
    with_chip(|chip| chip.model.i2c_start(is_read))
}

extern "C" fn on_read(_: *mut c_void) -> u8 {
    with_chip(|chip| chip.model.i2c_read())
}

extern "C" fn on_write(_: *mut c_void, data: u8) -> bool {
    with_chip(|chip| chip.model.i2c_write(data))
}

extern "C" fn on_disconnect(_: *mut c_void) {
    // Writes can change the pins: output enables, DRDY_DIS, reset.
    with_chip(|chip| chip.update_pins());
}

extern "C" fn on_tick(_: *mut c_void) {
    with_chip(Chip::tick);
}

/// Entry point called by the simulator.
#[unsafe(no_mangle)]
pub extern "C" fn chipInit() {
    // SAFETY: All pointers passed to the simulator are valid for the duration of each
    // call; configs are copied by the simulator. Init functions are only called here.
    unsafe {
        let int1 = pinInit(cstr("INT1").as_ptr(), OUTPUT_LOW);
        let int2 = pinInit(cstr("INT2").as_ptr(), OUTPUT_LOW);
        let sda = pinInit(cstr("SDA").as_ptr(), INPUT);
        let scl = pinInit(cstr("SCL").as_ptr(), INPUT);

        let model = Qmi8658aModel::new();
        i2cInit(&I2CConfig {
            user_data: std::ptr::null(),
            address: u32::from(model.address()),
            scl,
            sda,
            connect: on_connect,
            read: on_read,
            write: on_write,
            disconnect: on_disconnect,
        });

        let tick_timer = timerInit(&TimerConfig {
            user_data: std::ptr::null(),
            callback: on_tick,
        });

        let chip = Chip {
            model,
            int1,
            int2,
            accel: ["accelX", "accelY", "accelZ"].map(|id| attrInit(cstr(id).as_ptr(), 0.0)),
            gyro: ["gyroX", "gyroY", "gyroZ"].map(|id| attrInit(cstr(id).as_ptr(), 0.0)),
            temperature: attrInit(cstr("temperature").as_ptr(), 25.0),
        };

        CHIP.with(|c| *c.borrow_mut() = Some(chip));

        timerStart(tick_timer, TICK_US, true);
    }

    debug("qmi8658a: chip initialized");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversions() {
        assert_eq!(milli(-90.0, 1000.0), -90_000);
        assert_eq!(milli(1000.0, 1.0), 1000);
        assert_eq!(celsius_to_lsb(25.0), 6400);
        assert_eq!(celsius_to_lsb(-0.5), -128);
    }
}
