# The update rig: samd5-boot's self-test

This is the worked integration of `samd5-boot`, and it doubles as the
crate's test suite: eleven checks over install, verify, trial, confirm,
reject, rollback, the update window and an abandoned download, each driven
from a host and reported as a PASS/FAIL line.

```
$ cargo xtask test --chip ATSAMD51J20A
link: RTT on ATSAMD51J20A (up 1, down 0)

[PASS] link: Ping is answered
[PASS] install: a good image is accepted, booted, and confirms itself
[PASS] verify: a corrupted image is rejected as VerifyFailed
[PASS] verify: the running image survives a rejected update
[PASS] trial: an unconfirmed image boots and reports confirmed=false
[PASS] revert: an application that rejects itself is rolled back
[PASS] revert: an unconfirmed image is rolled back once attempts run out
[PASS] window: an application-requested reboot accepts a new image
[PASS] verify: a download that delivers no bytes is rejected
[PASS] verify: the running image survives a download that delivers no bytes
[PASS] install: a failed download leaves its target bank condemned

11 passed, 0 failed
```

The last three are about a download that ends before it has delivered
anything. Verification reads the slot it wrote, not the stream it read, so
an empty download over a bank that still holds a whole image would
otherwise verify that image and install it as a fresh trial; and the bank a
download was writing must not be left recorded as bootable, because
`Boot::fall_back` swaps to a `Valid` bank without verifying it first. The
suite puts a verified image in the target bank before it sends nothing, so
a device that took the stale one fails rather than passes.

The assertions are on silicon, not on a model of it: each one ends in a
`GetState` the application itself answers, so a PASS means the device really
did come back up running the image the test expected.

## Two transports

The rig speaks a byte stream, and it does not much care what carries it.

- **RTT** (default) needs nothing but the debug probe that put BOOT on the
  part in the first place. Two ring buffers in the target's RAM, read and
  written through the debug port while the core runs. Use this to reproduce
  the suite on any SAM D5x.
- **RS485** (`--rs485` when building, `--port` when running) needs a
  transceiver on PB02 / PB03 / PB00. It is the one that proves an update
  works over a field transport with **no debugger in the loop**, which is
  the property RTT cannot demonstrate.

The firmware picks one at compile time (`rtt` / `rs485` features on `boot`
and `app`); `cargo xtask` builds and drives whichever you asked for. A
mismatch shows up two ways: over RTT the attach fails, naming the channels
it did find, while a serial port opens perfectly well and simply stays
quiet. Neither costs you a step, because `cargo xtask test` does not ask
the installed image to stand aside at all: it takes the link from BOOT
through the debugger before it starts. See the last section.

## What you need

- An ATSAMD51J20A board (1 MiB, the part the demo crates select).
- A debug probe. `cargo xtask` drives it through probe-rs as a library, so
  the probe-rs CLI is not needed.
- `rust-objcopy` (`cargo install cargo-binutils`, `rustup component add
  llvm-tools`).
- For the RS485 transport only: a transceiver on PB02 (TxD), PB03 (RxD),
  PB00 (driver enable), and a USB adapter on the host.

### The fuse prerequisite

`Boot::new` refuses to run unless BOOTPROT matches the configured BOOT size,
so the part must be fused before any of this works (32 KiB BOOT means the
user page's BOOTPROT field reads 11). An unfused part prints `boot: fuses
not provisioned` and parks.

```
cargo xtask info --chip ATSAMD51J20A                  # what is it set to now
cargo xtask provision --chip ATSAMD51J20A --dry-run   # what would change
cargo xtask provision --chip ATSAMD51J20A
```

`provision` writes the user page from one halted debug session, then resets
and reads the fuses back, so it reports whether they actually latched rather
than assuming it. It saves the page to `userpage-<chip>.bak` before erasing;
`--restore` puts one back.

## Run it

All `cargo xtask` commands are run from the repository root.

```
cargo xtask test --chip ATSAMD51J20A
```

builds, stamps, places BOOT at both bank heads and runs the suite. Over
RS485 instead:

```
cargo xtask test --chip ATSAMD51J20A --port /dev/serial/by-id/usb-...
```

`--see` runs the same suite with the boot record in SmartEEPROM instead of
backup RAM, which is the other `BootStorage` the crate ships:

```
cargo xtask provision --chip ATSAMD51J20A --sblk 1
cargo xtask test --chip ATSAMD51J20A --see
```

Both firmware roles are rebuilt with the `see-store` feature, so they agree
on where the record lives. SBLK must already be non-zero or BOOT parks with
a message saying so; `cargo xtask info` reports the live value.

The steps are also available separately:

```
cargo xtask build [--rs485] [--see]     both app images, stamped, plus BOOT
cargo xtask flash --chip <CHIP>         place BOOT at both bank heads
cargo xtask request-update --chip <CHIP> [--see]  make BOOT wait, from the debugger
cargo xtask link  --chip <CHIP> <op>
```

where `<op>` is one of `ping`, `state`, `banks`, `update <image>`,
`bogus <image>`, `reboot`, `reset`, `reject`. Add `--port <dev>` to any of
them to use RS485.

Add `--log` to `test` to forward the target's `rprintln!` output to stderr,
prefixed `target|`; `link` forwards it already, and `--quiet` turns it off.
RTT only, and the first thing to turn on when a step fails for no visible
reason.

`flash` cannot simply write both heads on a provisioned part, because
BOOTPROT protects the active bank's head: it writes the inactive head,
issues `BKSWRST` to swap, writes the head that is now inactive, and swaps
back. Two swaps leave the same bank active as before, so whichever
application was running still is. Both heads are then read back and compared
with the NVM cache off.

`BKSWRST` resets the part, so the BOOT at the newly active head would
otherwise start running mid-dance, read the boot record and act on it. One
of the things it can decide is to swap straight back, because a bank
recorded `Invalid` is one `fall_back` reverts out of, and a failed download
leaves exactly that. So the swap arms the reset vector catch first and holds
the core halted for the length of the dance rather than racing it.

## How it is put together

- `proto`: postcard messages in COBS frames. A firmware image follows a
  `BeginUpdate` frame as raw bytes, but not until BOOT has answered it with
  `Ready`, so it never has to fit in RAM and never arrives before BOOT can
  take it. Variants are positional on the wire, so this enum only ever grows
  at the end; inserting one silently breaks every device already in the
  field.
- `demo-rig`: the handful of facts both firmware roles must agree on, the
  backup-RAM store offset above all. A mismatch there is silent until a
  confirm fails to take and every image reverts.
- `demo-rtt` and `demo-serial`: the two transports, offering the same `Link`
  surface so `boot` and `app` differ only in how they construct it.
- `boot`: the BOOT role. No listening window: a fixed one is a race the host
  has to win, and a general purpose host has no timing guarantees at all.
  The application sets the mailbox request flag before it resets and BOOT
  then waits with no deadline.
- `app`: the payload, in two builds. One confirms itself; the other does not
  and reports a different `app_version`, which is both what makes the
  rollback paths reachable and how the host tells the two apart. It also
  answers `GetBanks` with what the record says about each bank, which is
  the only view the host gets of BOOT's own bookkeeping.

The host end lives in `xtask` (`src/link/`, `src/selftest.rs`), because the
RTT transport needs probe-rs and the xtask already has it.

The trial record lives in backup RAM by default, so it survives the BKSWRST
reset while a power cut reads back as a blank slate, and the rig's
bookkeeping stays independent of the SEE fuses. `--see` puts it in
SmartEEPROM instead and runs the same eleven checks, which is how the other
`BootStorage` gets exercised on silicon rather than only compiled.

## Why the transports behave differently

### What a buffered transport costs

Buffering in the target's RAM has one sharp edge. A reply written into a RAM
ring is not gone from the device the way bytes put on a wire are: the image
that boots next zeroes `.bss` on its way up, and over RTT that is the very
buffer holding it. BOOT therefore holds the link open after reporting a failed
install until the host says something, so the report cannot be wiped before
it is read. Without that, a rejected image looks to the host exactly like a
successful one, since a successful install is the case where BOOT swaps and
never replies at all.

### Why the host waits for `Ready`

BOOT answers `BeginUpdate` with `Ready` and the host holds the image body
until it arrives. The gap between the two is `Boot::condemn` writing the
boot record, which on a SmartEEPROM store is a flash program, and BOOT is
not reading the wire while it happens: this BOOT's receive path is its own
foreground loop, not an interrupt. Bytes sent into that gap have nowhere to
go but the link's receive buffer, and with a 4 KiB RTT down channel there
are not many of them before it is full.

`Boot::install` takes its image source as a closure for this reason. The
closure runs after the record is written and before the first byte is read,
which is the only moment the signal is neither premature nor too late, and
it is handed the `Condemned` that proves the record was written. A
bootloader whose receiver fills a ring under interrupt, as `demo-serial`
does, does not need the signal, because its receiver never stops.

### Why RTT makes the bootloader's life easier

Streaming an image into `Boot::install` is awkward because the writer stops
reading for tens of milliseconds at every page program, and again at every
block erase. Over a UART those bytes are simply gone, which is why
`demo-serial` carries an 8 KiB interrupt-fed ring and why an unpaced host
used to fail right after page 0.

Over RTT the buffer *is* the transport. The host writes into a ring in RAM
and the target drains it when it gets around to it, so an ordinary
page-program stall costs a short write and a retry rather than data.

It does not stretch indefinitely. Filling the 4 KiB down channel to the last
byte is where probe-rs 0.30 stops reporting short counts and starts refusing
the channel outright, as `write pointer is 0x1000 while buffer size is
0x1000`. That is reachable, because this BOOT's receiver is its own
foreground loop and drains nothing while the boot record is being written.
Hence the `Ready` handshake above, rather than a deeper buffer: the depth
needed depends on how slow the store is, and the handshake does not.

### Finding the link again after a reset

Every reset hands the link from BOOT to the application or back, and their
control blocks are at different addresses, so the host has to find the new
one. Searching RAM for it is far too slow to repeat: a trial image is
watchdog-reset every few seconds by design, and a scan does not reliably fit
between two resets.

So the firmware publishes the address in a fixed backup-RAM slot, which
works because the linker never allocates from backup RAM and a `static`
cannot be pinned to a literal address from Rust alone. **The slot is
single-use**: the host clears the magic once it has used the address. That
turns it from a value that might be stale into a statement of fact, since
backup RAM survives a reset and a left-behind address would go on naming a
block that had since been zeroed. A magic that is present means firmware has
come up and published since anyone last looked, which is exactly the reset
signal the host needs; a magic that is absent means nothing has changed
hands.

## When the application will not step aside

BOOT has no listening window, so the application is what normally asks for
an update: it sets the mailbox request flag and resets, and BOOT then waits
with no deadline. An application that cannot be asked therefore leaves no
way in over the link itself, and there are two ways to end up with one.

Switching transports is the visible one: BOOT is an `rtt` build but the
image already installed speaks RS485, so nothing answers and the RTT attach
reports finding a control block with only a `Terminal` channel in it.

Switching stores is the invisible one, and it is worse. An image built
against the other `BootStorage` answers `Ping` and `GetState` perfectly
well, and does set the request flag when asked, in a record this BOOT never
reads. Nothing about it looks wrong from the host, so there is no state to
test for: the device simply refuses to step aside, forever.

The flag lives in the boot record, and a debugger can write that as easily
as the application can:

```
cargo xtask request-update --chip ATSAMD51J20A [--see]
```

It reads the record, sets the flag, reseals the checksum and resets, so the
trial bookkeeping already in it survives. `--see` amends the SmartEEPROM
record rather than the backup-RAM one, for firmware built with `see-store`.

`cargo xtask test` does this unconditionally, for the store it is about to
use, rather than waiting to see whether anything answers. Neither failure
above is something the host can reliably detect, and one of them cannot be
detected at all, so the suite does not try: it takes the link from BOOT
through the debugger and starts from there.

A shipping product wants the same hatch without a probe: a GPIO checked at
reset, or a window that opens only when no valid image exists.
