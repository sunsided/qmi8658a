# QMI8658A Wokwi custom chip

Wraps the [`qmi8658a-sim`](../../qmi8658a-sim) behavioral model into a
[Wokwi custom chip](https://docs.wokwi.com/chips-api/getting-started).

```sh
./build.sh   # produces qmi8658a.chip.wasm next to qmi8658a.chip.json
```

Reference it from a project's `wokwi.toml`:

```toml
[[chip]]
name = 'qmi8658a'
binary = '../../sim/wokwi-chip/qmi8658a.chip.wasm'
```

and add a `chip-qmi8658a` part to `diagram.json`.

| Pin    | Function                                              |
|--------|-------------------------------------------------------|
| `SDA`  | I²C data (address `0x6A`)                             |
| `SCL`  | I²C clock                                             |
| `INT1` | INT1 output: FIFO, activity, WoM or CTRL9 handshake  |
| `INT2` | INT2 output: DRDY, FIFO, activity, WoM or self-test   |

Controls `accelX`, `accelY`, `accelZ` (mg), `gyroX`, `gyroY`, `gyroZ` (dps) and
`temperature` (°C) are sampled every millisecond, which is also the model's time step.
Drive them from the Wokwi UI or with `set-control` in automation scenarios.

The chip models I²C only; SPI is covered by host tests with `FakeSpi`.
