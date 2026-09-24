# samd5-boot

A dual-bank A/B bootloader for ATSAMD5x / ATSAME5x microcontrollers, built
on the silicon's own bank swap rather than on a copier.

The flash of a SAM D5x/E5x is two equal banks, mapped into the main address
space in either order according to `STATUS.AFIRST`. [Section 25.6.7 of the
datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf#_OPENTOPIC_TOC_PROCESSING_d99375e230392)
describes using that for safe updates: write the new image into the bank
that is not running, then issue `BKSWRST`, which swaps the mapping and
resets in one atomic step. There is no window in which a half-written image
is the one that boots.

This crate implements that procedure, with a pluggable transport, image
verification by DSU CRC32, and rollback on failure driven by the watchdog.

## Status

The install, verify, trial, confirm, reject, attempts-exhausted rollback
and app-requested update paths are exercised on real silicon by
[`examples/update-rig`](examples/update-rig), which is both the worked
integration and the test suite: eight checks, each asserted against a
device that really rebooted into the image under test. Resuming an
interrupted install, rolling back from an interrupted revert, and coming
up with nothing bootable have no test yet.

The rig runs over RTT by default, so reproducing it needs nothing but a
debug probe, and over RS485 on request, which is the configuration that
proves an update works with no debugger in the loop.

The ABI is not frozen. P-256 image signing is reserved in the manifest and
not implemented, and so is anti-rollback: the manifest's `version` field
and the stored `Rollback` watermark are written but never compared. A
manifest length field is still to be added before 1.0.

Provisioning a fresh part is `samd5-boot-tools provision`, and placing BOOT
at both bank heads on a protected part is `samd5-boot-tools flash`; both are
proven on silicon.

## Using it

```toml
[dependencies]
samd5-boot = { version = "0.1", features = ["samd51j20a"] }
```

Selecting a part is how firmware asks for the driver; a part feature brings
everything that touches the silicon with it. A plain dependency with no
features is the flash ABI alone, which builds on a host: that is what host
tooling depends on, so a stamper and a bootloader cannot disagree about a
layout or a fuse encoding.

Two binaries, both built against this crate with the same part and
`bootprot-*` features, and both selecting their role in `memory.x`:

```
INCLUDE samd5_boot_boot.x    /* the bootloader: the head of each bank */
INCLUDE samd5_boot_app.x     /* the application: the rest of the bank */
```

BOOT owns NVMCTRL, the DSU and the watchdog. It reads the boot record,
decides what this boot should be, and either jumps to the application or
waits for an image on whatever transport you plug in:

```rust
let boot = Boot::new(nvm, dsu, wdt, config)?;
let boot = boot.boot_or_enter_download(&mut store);   // returns only to enter download
let Aborted { error, boot } = boot.install(&mut store, record, image_bytes);
```

`install` takes an `Iterator<Item = u8>`, so an image is pulled through one
flash page buffer at a time and never has to fit in RAM. On success it does
not return: the image is verified, the trial is recorded, and the bank swap
reboots the part.

The application reserves a manifest slot with `install_manifest!`, and a
post-link step fills in the length and CRCs with
[`manifest::stamp`](https://docs.rs/samd5-boot/latest/samd5_boot/manifest/fn.stamp.html),
which is in this crate and compiles on a host. An image that has not been
stamped will not verify. On a trial boot the application marks itself good:

```rust
BootClient::new(store).confirm(&mut wdt, Some(timeout))?;
```

Without that call the watchdog fires, BOOT swaps back, and the previous
image is running again.

## Getting a bootloader onto a part

The operations that need a debug probe rather than a compiler (writing the
fuses, placing BOOT at both bank heads) are
[`samd5-boot-tools`](samd5-boot-tools), a separate crate because it drives
probe-rs, which has no place in a `no_std` dependency. It carries the
post-link `stamp` step too, which needs no probe.

```
cargo install samd5-boot-tools
samd5-boot-tools --chip ATSAMD51J20A info
samd5-boot-tools --chip ATSAMD51J20A provision --dry-run
samd5-boot-tools --chip ATSAMD51J20A flash boot.bin
```

It is a library first: a project's own `xtask` can call `provision`, `flash`
and `stamp` directly and keep them in its existing build, which is what the
`cli` feature exists to be turned off for.

## The rig

`examples/update-rig` lives in the
[repository](https://github.com/QuartzShard/samd5-boot) and is not published:
it is the worked integration, not a product. Clone it to run the suite.

It is also the test suite: eight checks driven from a host and reported as
PASS/FAIL, over RTT (which needs only a debug probe) or RS485. On an
ATSAMD51J20A with a probe attached:

```
cargo xtask provision --chip ATSAMD51J20A --dry-run   # review, then drop --dry-run
cargo xtask test --chip ATSAMD51J20A
```

`test` builds both application images, stamps their manifests, places BOOT
at the head of both banks and runs the suite over RTT. Add
`--port /dev/serial/by-id/usb-...` to run the same suite over RS485 instead,
which needs a transceiver on PB02 / PB03 / PB00.
[`examples/update-rig/README.md`](examples/update-rig/README.md) breaks that
into its steps and explains what each assertion proves.

`cargo xtask` drives the rig: it builds and stamps the demo images, speaks
their wire protocol, and runs the suite, delegating the device operations to
`samd5-boot-tools`. Both depend on this crate with no features, so the
manifest layout, the CRC convention and the fuse encodings all come from the
same definitions the firmware compiles against.

```
cargo xtask info      --chip <CHIP>   what a part is configured as
cargo xtask provision --chip <CHIP>   write and verify the fuses
cargo xtask build     [--rs485]       build and stamp the demo images
cargo xtask flash     --chip <CHIP>   place BOOT at both bank heads
cargo xtask stamp <in> <out> --version N
cargo xtask link      --chip <CHIP> <ping|state|update|...>
cargo xtask request-update --chip <CHIP>   make BOOT wait, from the debugger
cargo xtask test      --chip <CHIP> [--port <dev>]
```

## Hazards worth knowing

- **Interrupts must be off across `BKSWRST`.** The swap stalls the AHB and
  forbids any NVM fetch, and with SmartEEPROM enabled it also reallocates
  sectors, which widens that window to milliseconds. `swap_reboot` disables
  them; anything that reaches `BKSWRST` by another route must too.
- **BOOTPROT covers only the active bank's head.** The inactive copy is
  writable, so replacing a bootloader means writing the inactive head,
  issuing `BKSWRST`, writing the head that is now inactive, and swapping
  back, rather than writing one head and trusting the image being replaced
  to swap into it. Two swaps leave the same bank active as before.
- **A chip erase does not clear BOOTPROT**, the region locks, or the rest of
  the user page.

## License

MPL-2.0. This binds modifications to the files in this repository and
nothing else: linking it into a larger work, proprietary or otherwise,
carries no obligation beyond keeping changes to these files open.
