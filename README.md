# QST QMI8658A 6-Axis IMU

[![Build Status](https://github.com/sunsided/qmi8658a/workflows/Rust/badge.svg)](https://github.com/sunsided/qmi8658a/actions)
[![codecov](https://codecov.io/gh/sunsided/qmi8658a/graph/badge.svg)](https://codecov.io/gh/sunsided/qmi8658a)

> Platform-agnostic Rust support for the QST QMI8658A 6-axis IMU (accelerometer and gyroscope).

> **Status:** tested against a behavioral model of the chip (host tests and Wokwi), not yet
> verified on real hardware.

| Crate                                      | Description                                                 |
|--------------------------------------------|-------------------------------------------------------------|
| [`qmi8658a-registers`](qmi8658a-registers) | A typed map of the QMI8658A's I²C/SPI registers.            |
| [`qmi8658a`](qmi8658a)                     | An `embedded-hal` 1.0 I²C/SPI driver (blocking and async).  |
| [`qmi8658a-sim`](qmi8658a-sim)             | A behavioral model of the chip for tests and emulation.     |

The register map and the driver follow the QMI8658A datasheet (document 13-52-25, Rev A).
No code was taken from other QMI8658 drivers. The crate name `qmi8658` belongs to an
unrelated project.

## Emulation

The behavioral model in `qmi8658a-sim` is used both as a fake I²C bus and SPI device in
host tests and as a [Wokwi](https://wokwi.com) custom chip (`sim/wokwi-chip`). Example
firmware in [`examples/esp32c3`](examples/esp32c3) runs against the emulated chip using
`wokwi-cli`. The model's assumptions about what the datasheet leaves open are listed in
its crate documentation and under [Gaps and contradictions](#gaps-and-contradictions).

## Datasheet notes

**FIFO burst reads.** Section 8.1 says FIFO data is read "in burst reads", and section 8.8
only says to read `FIFO_DATA` "per FIFO_Sample_Count". Section 16.1 says that during burst
reads the register address advances while `CTRL1.ADDR_AI` is set and stays put while it is
clear. The datasheet makes no exception for `FIFO_DATA`, and never says to clear `ADDR_AI`
for a FIFO drain. Read together, a burst from `FIFO_DATA` with `ADDR_AI = 1` walks on to
`0x18` and beyond, so a drain needs `ADDR_AI = 0` (or one byte per transfer).
`Qmi8658a::read_fifo` clears `ADDR_AI` for the drain and restores it afterwards; the other
reads keep `ADDR_AI = 1`.

**`STATUSINT` bit 0.** Table 24 and section 6.2 confirm it: outside SyncSample mode, bit 0
shows the INT2 level and bit 1 the INT1 level. The datasheet does not say what the bits
read while the pin is high-Z (`CTRL1.INT2_EN = 0`, the default), and DRDY on INT2 is a
short pulse, not a latched flag (6.3). The driver therefore polls `STATUS0` for new data,
which works without INT2. The self-test procedure (15.1, 15.2) waits for "INT2 to High, if
INT2 is enabled, or STATUSINT.bit0 is set to 1", which can be read either way, so the
driver enables INT2 for the duration of a self-test.

## Sources

All facts come from the QST QMI8658A datasheet, document 13-52-25, Rev A (June 20, 2022),
abbreviated DS below.

| Fact | Source |
|------|--------|
| Sensor axes: right-handed, +Z out of the top face; acceleration and angular rate share them (`SensorFrame` = `EastNorthUp`) | DS 2, figure 10 |
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

## Releasing

Both crates share one version. From `main`, with the changes listed under
`## [Unreleased]` in each crate's `CHANGELOG.md`:

```sh
cargo release <patch|minor|major>            # dry run
cargo release <patch|minor|major> --execute
```

This commits `Release X.Y.Z` and pushes the tag `vX.Y.Z`. The tag runs the release workflow,
which publishes `qmi8658a-registers` and `qmi8658a` to crates.io via
[trusted publishing](https://crates.io/docs/trusted-publishing) (no API token in the
repository) and then creates a GitHub Release named `X.Y.Z` with the changelog sections
as notes. The first version of each crate has to be published by hand, because trusted
publishing can only be configured for an existing crate.

Each GitHub Release carries the packaged `.crate` files and the Wokwi chip as `chip.zip`,
all with [build provenance attestations](https://docs.github.com/actions/security-for-github-actions/using-artifact-attestations):

```sh
gh attestation verify chip.zip -R sunsided/qmi8658a
```

## Code of Conduct

We abide by the [Contributor Covenant][cc] and ask that you do as well.

## License

Copyright © 2026 Markus Mayer

Triple licensed under your choice of either of:

- European Union Public Licence, Version 1.2, ([LICENSE-EUPL](LICENSE-EUPL)
  or https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)
- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

[cc]: https://contributor-covenant.org
