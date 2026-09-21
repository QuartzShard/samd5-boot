#!/usr/bin/env bash
#
# Build the end-to-end install demo:
#   1. compile trial-app (the payload, an app-role image)
#   2. stamp its manifest (magic, length, CRCs) so it passes verification
#   3. embed the stamped image into boot-demo and build boot-demo
#   4. emit boot-demo.bin for flashing
#
# Then provision boot-demo.bin to both banks and watch RTT:
#   CHIP=ATSAMD51J20A FLASH_SIZE=0x100000 ../scripts/provision-boot.sh boot-demo.bin
#
# Needs rust-objcopy (cargo install cargo-binutils; rustup component add
# llvm-tools). Override with OBJCOPY=... if you use a different one.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TRIPLE=thumbv7em-none-eabihf
OBJCOPY="${OBJCOPY:-rust-objcopy}"

echo "[1/4] build trial-app"
(cd "$HERE/trial-app" && cargo build --release)
"$OBJCOPY" -O binary \
    "$HERE/trial-app/target/$TRIPLE/release/trial-app" \
    "$HERE/trial-app-raw.bin"

echo "[2/4] stamp manifest"
cargo run --quiet --release --manifest-path "$ROOT/tools/manifest-tool/Cargo.toml" -- \
    "$HERE/trial-app-raw.bin" "$HERE/boot-demo/trial-app.bin" --version 1

echo "[3/4] build boot-demo (embeds the stamped image)"
(cd "$HERE/boot-demo" && cargo build --release)

echo "[4/4] emit boot-demo.bin"
"$OBJCOPY" -O binary \
    "$HERE/boot-demo/target/$TRIPLE/release/boot-demo" \
    "$HERE/boot-demo.bin"

echo
echo "Built $HERE/boot-demo.bin"
echo "Flash it to both banks, then watch RTT:"
echo "  CHIP=ATSAMD51J20A FLASH_SIZE=0x100000 $ROOT/scripts/provision-boot.sh $HERE/boot-demo.bin"
