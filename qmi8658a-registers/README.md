# QST QMI8658A IMU Registers

[![Crates.io][crates-image]][crates-link]
[![Docs][docs-image]][docs-link]
[![Build Status][build-image]][build-link]
[![Coverage][codecov-image]][codecov-link]
[![Safety Dance][safety-image]][safety-link]
![MSRV][msrv-image]
[![EUPL 1.2 licensed][license-eupl-image]][license-eupl-link]
[![Apache 2.0 licensed][license-apache-image]][license-apache-link]
[![MIT licensed][license-mit-image]][license-mit-link]

> A typed map of the QMI8658A's I²C/SPI-accessible registers.

Provides functions and named fields to read and modify the IMU registers,
built on [`hardware-registers`](https://crates.io/crates/hardware-registers).
The register map follows the QMI8658A datasheet (document 13-52-25, Rev A).

See also the [`qmi8658a`](https://crates.io/crates/qmi8658a) driver.

## Sources

All facts come from the QST QMI8658A datasheet, document 13-52-25, Rev A (June 20, 2022),
abbreviated DS below.

| Fact | Source |
|------|--------|
| I²C address `0x6A` with `SA0` high or unconnected, `0x6B` with `SA0` low | DS table 3, 16.3.1 |
| I²C standard and fast mode up to 400 kHz; SPI mode 0 or 3 up to 15 MHz, 4-wire by default | DS table 14, 16.2.1, table 43 |
| SPI address byte: MSB set for reads | DS figure 28 |
| Register map: addresses, access, defaults | DS table 19 |
| `WHO_AM_I` (`0x00`) reads `0x05` | DS table 21 |
| `CTRL1`, `CTRL2`, `CTRL3`, `CTRL5`, `CTRL7`, `CTRL8` fields | DS table 22 |
| `CTRL1.BE` defaults to big-endian, `CTRL1.ADDR_AI` to no auto-increment | DS table 22 |
| Burst reads repeat one register unless `ADDR_AI` is set; no burst writes to the configuration registers | DS 16.1, 16.2.1 |
| Accelerometer full scales and sensitivities (16384 to 2048 LSB/g) | DS table 7, table 22 |
| Gyroscope full scales and sensitivities (2048 to 16 LSB/dps) | DS table 8, table 22 |
| Output data rates: 6DOF rates from the gyroscope, accelerometer-only rates from the oscillator, low-power rates only without gyroscope | DS 3.5, table 22 |
| Gyroscope noise rises above 1 kHz ODR | DS table 12 |
| Temperature: 256 LSB/°C, `T = TEMP_H + TEMP_L / 256` | DS 3.9, table 18, table 25 |
| Soft reset: write `0xB0` to `RESET` (`0x60`) | DS table 27 |
| Reset status: `0x4D` reads `0x80` after a successful reset | DS 5.9, 7.4 |
| System turn-on time 15 ms after power-up or reset | DS tables 7 and 8, 7.3 |
| Turn-on: accelerometer 3 ms + 3/ODR, gyroscope 150 ms + 3/ODR | DS tables 7 and 8, 7.3 |
| INT1/INT2 are high-Z by default, push-pull when enabled in `CTRL1` | DS 6, table 22 |
| DRDY: a pulse on INT2 only, with the FIFO in bypass and `DRDY_DIS = 0` | DS 6.2, 6.3 |
| `STATUSINT` bits 0 and 1 show the INT2 and INT1 levels outside SyncSample mode | DS table 24, 6.2 |
| `STATUS0` data-available flags, `STATUS1` motion flags, 24-bit timestamp | DS table 24 |
| FIFO registers and read sequence | DS table 23, 8 |
| CTRL9 commands and handshake | DS 5.10, table 28 |
| `COD_STATUS`, `TAP_STATUS`, step count, `dQ*`/`dV*` contents | DS 5.7, 5.8, 5.10.6.11 |
| CTRL9 handshake: CmdDone in `STATUSINT.bit7`, ACK with `0x00`, INT1 unless `CTRL8.bit7` | DS 5.10.1, 5.10.4, 5.10.5, 6.2 |
| FIFO: 1536 bytes, 16 to 128 samples of 6 or 12 bytes, sample count in words, watermark in samples, read mode via `CTRL_CMD_REQ_FIFO`, data pattern | DS 8.1 to 8.10, table 23 |
| Any-, no-, significant-motion parameters and `MOTION_MODE_CTRL` | DS 9.1, 9.3, 9.4, tables 32 to 34 |
| Tap parameters, example values, `TAP_STATUS` | DS 10.2, 10.3, 10.6, tables 35, 36 |
| Pedometer parameters and example, step count update and reset | DS 11.1 to 11.6, tables 37, 38 |
| Activity interrupts on INT1/INT2 by `CTRL8.bit6`, flags in `STATUS1` | DS 9.6, 10.5, 11.4 |
| Wake on Motion: threshold in mg, line select and initial level, blanking, procedure, `STATUS1` read resets the line | DS 12.2 to 12.6, table 39 |
| Self-test procedures, 1/2048 g and 1/16 dps results, 200 mg and 300 dps limits, about 25 ODRs and 400 ms | DS 15.1, 15.2 |
| Calibration-on-demand: sensors off, about 1.5 s, `COD_STATUS`; gyro gains in `dVX`-`dVZ`, restore via `CTRL_CMD_APPLY_GYRO_GAINS` | DS 14.2 to 14.4 |
| Host delta offsets: signed 4.12 (accelerometer) and 11.5 (gyroscope) in `CAL1`-`CAL3` | DS 5.10.6.5, 5.10.6.6 |

### Gaps and contradictions

| Topic | What the datasheet says | What this crate does |
|-------|-------------------------|----------------------|
| Soft reset value | `0xB0` (table 27), `0x0B` (7.4) | Writes `0xB0` |
| `REVISION_ID` | `0x68` (table 19), `0x7C` (table 21) | Does not check it |
| `CTRL1.BE` | "read data big-endian", no byte layout | The driver selects little-endian; the model swaps the bytes of each 16-bit output |
| `STATUS0` flags | "no updates since last read", not which read | The model clears them when `STATUS0` is read |
| `STATUSINT` bit 0 with INT2 high-Z | mirrors "INT2 level", high-Z not covered | The driver polls `STATUS0`; the model keeps the bit low |
| DRDY pulse width | "cleared automatically after a short duration" | The model holds INT2 high until its next time step |
| Timestamp with both sensors | counts samples of "sensor with highest ODR" | The model counts gyroscope samples |
| FIFO burst reads | FIFO data is read "in burst reads" (8.1), yet a burst advances the address while `ADDR_AI` is set (16.1) | `read_fifo` clears `ADDR_AI` for the `FIFO_DATA` burst and restores it |
| CTRL9 durations | None, except about 1.5 s for calibration-on-demand (14.2) | Waits up to 100 ms per handshake step, 3 s for calibration-on-demand |
| CTRL9 ACK | `CmdDone` clears after the ACK (5.10.1) | Waits for `CmdDone` to clear |
| `CAL4_H` set selector | Tables 34, 36, 38 say `CAL4_H (7:4)`, the text says write `0x01`/`0x02` to `CAL4_H` | Writes `0x01`/`0x02` to the whole byte |
| `AnyMotionWindow` | "1-byte format unsigned, 5-bits fraction" with "1 sample" resolution (table 32) | Treats it as a sample count |
| Units of the host delta offsets | Formats only (4.12, 11.5), no unit or sign | Passes raw values; the model assumes g and dps, subtracted from the output |
| Self-test completion signal | "INT2 to High, if INT2 is enabled (CTRL1.bit4 = 1), or STATUSINT.bit0 is set to 1" (15.1) | Enables INT2 during the test and polls `STATUSINT.bit0` |
| Self-test ODR | "Set proper accelerometer ODR"; no rule for 6DOF-only codes | Uses the configured ODR |
| Self-test result format | Called "signed U5.11" and "signed U12.4" | Reads signed 16-bit values at 1/2048 g and 1/16 dps |
| `STATUS1` clearing | Only the WoM bit is said to clear on read (12.4) | The model clears tap, WoM and pedometer on read and keeps the motion bits as levels |
| Tap and pedometer algorithms | Described in prose and figures, not precisely enough to reproduce | The model injects tap and step events instead |
| FIFO overflow flag | Set on data drop, no clearing condition | The model clears it on `CTRL_CMD_RST_FIFO` and reset |
| SyncSample mode and the locking mechanism (13) | Documented, with AHB clock gating and `Data_Lock_Delay` tables | Not implemented (not in scope); the FIFO and the motion engines need non-SyncSample mode (6.1, 9, 10, 11) |
| `CTRL_CMD_COPY_USID`, `CTRL_CMD_SET_RPU`, `CTRL_CMD_AHB_CLOCK_GATING` | Documented, but not calibration | No typed methods; use `ctrl9_command` |

## License

Copyright © 2026 Markus Mayer

Triple licensed under your choice of either of:

- European Union Public Licence, Version 1.2, ([LICENSE-EUPL](LICENSE-EUPL)
  or https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)
- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

[crates-image]: https://img.shields.io/crates/v/qmi8658a-registers
[crates-link]: https://crates.io/crates/qmi8658a-registers
[docs-image]: https://docs.rs/qmi8658a-registers/badge.svg
[docs-link]: https://docs.rs/qmi8658a-registers/
[build-image]: https://github.com/sunsided/qmi8658a/workflows/Rust/badge.svg
[build-link]: https://github.com/sunsided/qmi8658a/actions
[codecov-image]: https://codecov.io/gh/sunsided/qmi8658a/graph/badge.svg
[codecov-link]: https://codecov.io/gh/sunsided/qmi8658a
[safety-image]: https://img.shields.io/badge/unsafe-forbidden-success.svg
[safety-link]: https://github.com/rust-secure-code/safety-dance/
[msrv-image]: https://img.shields.io/badge/rustc-1.85+-blue.svg
[license-eupl-image]: https://img.shields.io/badge/license-EUPL_1.2-blue.svg
[license-eupl-link]: https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12
[license-apache-image]: https://img.shields.io/badge/license-Apache_2.0-blue.svg
[license-apache-link]: https://www.apache.org/licenses/LICENSE-2.0
[license-mit-image]: https://img.shields.io/badge/license-MIT-blue.svg
[license-mit-link]: https://opensource.org/licenses/MIT
