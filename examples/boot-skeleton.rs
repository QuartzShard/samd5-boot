//! Minimal end-to-end BOOT-role binary: decide what to do from the
//! stored boot record (boot / roll back / update), and when that decision
//! is "wait for an update", stream a new image in over a transport. TRNG
//! stands in as the transport here. It needs no pins, so the skeleton
//! stays valid for every supported part, and it exercises the real
//! byte-stream seam (a deployment swaps in any `Iterator<Item = u8>`:
//! UART, USB, CAN, …). Random bytes never pass verification, so this
//! also drives the reject path.
//!
//! Also tracks `.text` against the 32 KiB budget. Build with e.g.
//! `cargo build --release --example boot-skeleton --target thumbv7em-none-eabihf --features samd51j20a`
#![no_std]
#![no_main]

use atsamd_hal as hal;
use hal::trng::Trng;
use hal::watchdog::{Watchdog, WatchdogTimeout};
use samd5_boot::{
    Boot, BootConfig,
    persist::{BootStorage, SmartEepromStore},
};

/// A plausible image length to pull from the stand-in transport.
const DEMO_IMAGE_LEN: usize = 4096;

struct TrngBytes(Trng);

impl Iterator for TrngBytes {
    type Item = u8;
    fn next(&mut self) -> Option<u8> {
        Some(self.0.random_u8())
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut peripherals = hal::pac::Peripherals::take().expect("Infallible");
    let nvm = hal::nvm::Nvm::new(peripherals.nvmctrl);
    let Ok(dsu) = hal::dsu::Dsu::new(peripherals.dsu, &peripherals.pac) else {
        park()
    };
    let wdt = Watchdog::new(peripherals.wdt);
    let config = BootConfig {
        max_boot_attempts: 3,
        trial_timeout: WatchdogTimeout::Cycles16K,
    };
    let Ok(boot) = Boot::new(nvm, dsu, wdt, config) else {
        park()
    };

    let Ok(mut store) = SmartEepromStore::<0>::new() else {
        park()
    };

    // Boots or swaps and never returns, unless the decision is to wait
    // for an update, then it returns for the download below.
    let boot = boot.boot_or_enter_download(&mut store);

    let record = store.read().unwrap_or_default();
    let source = TrngBytes(Trng::new(&mut peripherals.mclk, peripherals.trng)).take(DEMO_IMAGE_LEN);
    let _ = boot.install(&mut store, record, source);
    park()
}

fn park() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    park()
}
