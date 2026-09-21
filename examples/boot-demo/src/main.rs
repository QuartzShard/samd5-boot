//! End-to-end install demo, the easy on-ramp for a first bench bring-up.
//! It embeds a stamped `trial-app` image (so no transport is needed),
//! provisions BOOTPROT on a fresh chip, and on the first boot installs the
//! app into the inactive bank and swaps into it; from then on it boots the
//! app directly. Watch RTT to see BOOT install and hand off, then the app
//! print its ticks.
//!
//! Flow on a fresh chip: `Boot::init` sets BOOTPROT and resets once;
//! `boot_or_enter_download` finds no valid app and returns; `install`
//! streams the embedded image in and swaps. After the swap the app is
//! active and verifies, so `boot_or_enter_download` jumps straight to it.
//!
//! Build the whole demo with `examples/build-demo.sh`, then provision the
//! resulting `boot-demo.bin` to both banks with `scripts/provision-boot.sh`.
#![no_std]
#![no_main]

use atsamd_hal as hal;
use hal::watchdog::{Watchdog, WatchdogTimeout};
use rtt_target::{rprintln, rtt_init_print};
use samd5_boot::{
    Boot, BootConfig,
    boot_info::{self, BootInfo},
    consts, install_boot_info,
    persist::{BootStorage, NoStore},
};

/// The stamped trial-app image, embedded so the demo needs no transport.
/// `build-demo.sh` writes this file next to the crate.
const TRIAL_APP: &[u8] = include_bytes!("../trial-app.bin");

install_boot_info!(BootInfo {
    magic: boot_info::MAGIC,
    abi_version: boot_info::ABI_VERSION,
    transport_version: 1,
    boot_size: consts::BOOT_SIZE as u32,
    build_id: 0,
});

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("boot-demo: BOOT starting");

    let peripherals = hal::pac::Peripherals::take().expect("Infallible");
    let nvm = hal::nvm::Nvm::new(peripherals.nvmctrl);
    let Ok(dsu) = hal::dsu::Dsu::new(peripherals.dsu, &peripherals.pac) else {
        rprintln!("boot-demo: DSU init failed");
        park()
    };
    let wdt = Watchdog::new(peripherals.wdt);
    let config = BootConfig {
        max_boot_attempts: 3,
        trial_timeout: WatchdogTimeout::Cycles16K,
    };

    // Provisions BOOTPROT on a fresh chip (resets once), then returns a Boot.
    let boot = Boot::init(nvm, dsu, wdt, config);

    // NoStore keeps the demo simple: no trial bookkeeping, so a verifying
    // active app boots directly and there is nothing to confirm.
    let mut store = NoStore;
    let boot = boot.boot_or_enter_download(&mut store);

    rprintln!(
        "boot-demo: no valid app; installing {} bytes and swapping",
        TRIAL_APP.len()
    );
    let record = store.read().unwrap_or_default();
    let _ = boot.install(&mut store, record, TRIAL_APP.iter().copied());

    // install swaps and reboots on success, so reaching here is a failure.
    rprintln!("boot-demo: install failed (image did not verify?)");
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
