# Changelog

All notable changes to this project will be documented in this file.
This project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.2] - 2026-11-15

[0.1.2]: https://github.com/sunsided/qmi8658a/releases/tag/v0.1.2

### Changed

- Dropped `#![deny(warnings)]`, so a warning added by a newer compiler can no longer
  fail the docs.rs build.

## [0.1.1] - 2026-11-15

[0.1.1]: https://github.com/sunsided/qmi8658a/releases/tag/v0.1.1

## [0.1.0] - 2026-11-15

[0.1.0]: https://github.com/sunsided/qmi8658a/releases/tag/v0.1.0

### Added

- Register map following the QMI8658A datasheet (13-52-25 Rev A), with I²C and SPI register traits.
