#!/usr/bin/env sh
# Builds the chip and places it next to qmi8658a.chip.json, as Wokwi expects.
set -eu
cd "$(dirname "$0")"
cargo build --release
cp target/wasm32-unknown-unknown/release/qmi8658a_wokwi_chip.wasm qmi8658a.chip.wasm
