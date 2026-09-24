# samd5-boot-tools

Bench and manufacturing operations for a
[samd5-boot](https://github.com/QuartzShard/samd5-boot) device, over a debug
probe: writing the fuses that make the bootloader protected and placing it at
the head of both banks. It also stamps an application image so the bootloader
will accept it, which needs no probe.

```
cargo install samd5-boot-tools

samd5-boot-tools --chip ATSAMD51J20A info
samd5-boot-tools --chip ATSAMD51J20A provision --dry-run   # then without it
samd5-boot-tools --chip ATSAMD51J20A flash boot.bin
samd5-boot-tools stamp app-raw.bin app.bin --version 1
samd5-boot-tools --chip ATSAMD51J20A request-update --record-addr 0x47000000
```

`provision`, `flash` and `request-update` take `--dry-run`: the reads happen
against the real part and the register writes are printed instead of issued.
`info` only reads, so the flag changes nothing there, and `stamp` never
touches a part and always writes its output file.

## As a library

```toml
[dependencies]
samd5-boot-tools = { version = "0.1", default-features = false }
```

The default `cli` feature is only the command line front end. Off, this is a
library for a project's own `xtask` to call, which is the form that composes
with a build you already have: `provision::info`, `provision::run`,
`flash::run`, `image::stamp` and `provision::request_update` are the same
operations the subcommands wrap.

The layouts and encodings come from `samd5-boot` itself, so a part programmed
by this agrees with the firmware by construction rather than by two
implementations happening to match.

## Why a probe, and why probe-rs as a library

`provision` rewrites the user page, and that cannot be done with one-shot CLI
pokes. The page buffer is volatile and auxiliary pages have no
read-while-write, so the erase, the buffer fill and the commit have to be one
uninterrupted sequence against a halted core that is not itself driving
NVMCTRL. A CLI that attaches and detaches per invocation cannot hold that
together, and the failure is silent: errata NVM101-7 cache pollution makes the
read-back look convincing.

`flash` has a related problem. BOOTPROT protects the head of the *active*
bank, so on a part whose BOOTPROT is set the two copies cannot both be
written where they sit. It writes the inactive head, issues `BKSWRST` to
swap, writes the head that is now inactive, and swaps back, leaving the same
bank active as before. With BOOTPROT unset it writes both heads directly.

## Recovery

`provision` saves the user page to `userpage-<chip>.bak` in `--backup-dir`
(the working directory by default) before erasing it, and `--restore` writes
one back verbatim. A blank page is not backed up, and an existing backup is
never overwritten. The erase is what makes that necessary: between it and the
last commit the part has no fuses at all.

## License

MPL-2.0.
