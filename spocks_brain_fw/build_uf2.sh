#!/usr/bin/env bash
# `cargo build` only produces the ELF; BOOTSEL flashing needs a UF2 from elf2uf2-rs.
set -euo pipefail
cd "$(dirname "$0")"
cargo build --release
exec elf2uf2-rs \
  target/thumbv6m-none-eabi/release/spocks_brain_fw \
  target/thumbv6m-none-eabi/release/spocks_brain_fw.uf2
