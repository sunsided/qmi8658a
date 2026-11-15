# QMI8658A on ESP32-C3

Runs the accelerometer and gyroscope self-test, then reads a QMI8658A with the blocking
`qmi8658a` driver at ±8 g / ±512 dps and 56 Hz, and prints acceleration (g), angular rate (dps) and temperature whenever they change. It polls
`STATUS0` for new data, so it needs no interrupt line.

| ESP32-C3 | QMI8658A          |
|----------|-------------------|
| GPIO4    | SDA               |
| GPIO5    | SCL               |
| 3V3      | `SDO/SA0` (`0x6A`) |

The configuration mirrors the Kode Dot: the QMI8658A sits at `0x6A` on I²C0 at 400 kHz,
INT1 goes to a TCA6408A I/O expander (P5) and INT2 is not connected. Without INT2 there
is no data-ready pulse, and `STATUSINT` bit 0, which mirrors the INT2 level, never rises,
so the firmware polls `STATUS0` instead.

## Run in Wokwi

Requires `rustup target add riscv32imc-unknown-none-elf wasm32-unknown-unknown`,
[`wokwi-cli`](https://docs.wokwi.com/wokwi-ci/cli-installation) and `WOKWI_CLI_TOKEN`.

```sh
../../sim/wokwi-chip/build.sh
cargo build --release
wokwi-cli . --scenario scenarios/motion.yaml --timeout 30000
```

Or open this directory in VS Code with the Wokwi extension and drag the sliders.

## Run on hardware

```sh
cargo run --release   # uses espflash
```
