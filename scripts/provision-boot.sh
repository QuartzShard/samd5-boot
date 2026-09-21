#!/usr/bin/env bash
#
# Bench provisioning for a samd5-boot device, via probe-rs. Installs the
# same BOOT image at the head of BOTH physical banks so the revert agent is
# present whichever bank the chip resets into, then resets so the BOOT
# firmware's first-boot provisioning can set the BOOTPROT fuse.
#
# This is a starting point meant to be folded into a project xtask; it does
# the reliable, chip-agnostic parts and documents the fuse steps that depend
# on your firmware and hardening choices.
#
# Prerequisites:
#   - probe-rs installed, a debug probe attached, the target powered.
#   - A FRESH chip (BOOTPROT unset). Once BOOTPROT protects bank A's head,
#     re-flashing it needs a chip-erase or BOOTPROT-disable first.
#   - A BOOT .bin (not .elf); build your BOOT crate and objcopy it, e.g.
#       cargo build --release
#       rust-objcopy -O binary target/<triple>/release/<boot> boot.bin
#
# Usage:
#   CHIP=ATSAMD51J20A FLASH_SIZE=0x100000 ./provision-boot.sh boot.bin
#
# FLASH_SIZE is the part's total flash; the inactive bank begins at half of
# it (256K->0x20000, 512K->0x40000, 1M->0x80000).

set -euo pipefail

BOOT_BIN="${1:?usage: [CHIP=..] [FLASH_SIZE=..] provision-boot.sh <boot.bin>}"
CHIP="${CHIP:?set CHIP, e.g. ATSAMD51J20A}"
FLASH_SIZE="${FLASH_SIZE:?set FLASH_SIZE, e.g. 0x100000 for 1 MiB}"

BANK_A=0x0
BANK_B=$(( FLASH_SIZE / 2 ))
BANK_B_HEX=$(printf '0x%x' "$BANK_B")

echo "Provisioning $CHIP with $BOOT_BIN"
echo "  bank A (active-mapped) head: $BANK_A"
echo "  bank B (inactive) head:      $BANK_B_HEX"

# Both bank heads get the byte-identical BOOT. Bank B is addressed directly
# in the upper half of the address space (valid while STATUS.AFIRST is at its
# fresh-chip default, which it is before any BKSWRST).
probe-rs download --chip "$CHIP" --binary-format bin --base-address "$BANK_A"     "$BOOT_BIN"
probe-rs download --chip "$CHIP" --binary-format bin --base-address "$BANK_B_HEX" "$BOOT_BIN"

# Reset into the freshly installed BOOT.
probe-rs reset --chip "$CHIP"

cat <<'NOTE'

Flashed both banks and reset. Remaining fuse steps:

  BOOTPROT: the crate provisions this in software. Have your BOOT binary call
  `Boot::init` when `Boot::new` returns `BootprotMisconfigured` (a fresh chip);
  it writes BOOTPROT to match BOOT_SIZE and resets to reload it. After that
  first provisioning boot the device is BOOTPROT-protected. (The skeleton
  example parks on `new` failure instead; a shipping BOOT should fall back to
  `init`.)

  Region locks (optional hardening): to hardware-protect the INACTIVE bank's
  BOOT copy as well (BOOTPROT only covers the active/mapped-low one), lock the
  flash regions covering each bank's head. This is a user-page write and is
  left to your xtask/procedure; it is not required for a first bench bring-up.

  SmartEEPROM: if your app uses SEE, the SBLK/PSZ fuses must be set before the
  store works; that is application configuration, not BOOT provisioning.
NOTE
