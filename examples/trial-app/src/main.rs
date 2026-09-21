//! The payload for the install demo: a samd5-boot application image that
//! prints over RTT so you can watch it come up after BOOT installs and
//! swaps into it. It reserves a manifest slot with [`install_manifest!`];
//! `examples/build-demo.sh` stamps the real magic/length/CRCs into that
//! slot after linking so the image passes verification.
//!
//! This uses `NoStore` on the BOOT side, so there is no trial to confirm:
//! the app just runs. To exercise the trial/confirm path, configure
//! SmartEEPROM and have this call `client::BootClient::confirm` early.
#![no_std]
#![no_main]

use rtt_target::{rprintln, rtt_init_print};
use samd5_boot::{install_manifest, manifest::AppManifest};

install_manifest!(AppManifest::placeholder());

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("trial-app: booted from flash after install + swap");
    let mut tick = 0u32;
    loop {
        rprintln!("trial-app: alive, tick {}", tick);
        tick = tick.wrapping_add(1);
        // ~0.2 s at 120 MHz; crude, no clock setup needed for the demo.
        cortex_m::asm::delay(24_000_000);
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
