//! The protocol over RTT: two ring buffers in the target's RAM, read and
//! written through the debug probe while the core runs.
//!
//! [`RttLink::attach`] claims the probe and yields the [`Transport`] that
//! [`Link`](super::Link) frames over. Channel 1 up and channel 0 down
//! carry the protocol; channel 0 up carries `rprintln!` output, forwarded
//! only when `log` is set. Dropping the link releases the probe, which
//! anything that needs its own debug session (`flash`, `provision`) is
//! waiting on.
//!
//! # Surviving a reset
//!
//! Every reset in the test hands the link from BOOT to the application or
//! back, and both ends of that have to be picked up again.
//!
//! The control block moves, because the two images are separate binaries
//! whose blocks sit at different addresses, so a reconnect has to be able
//! to re-find it. Scanning RAM for that is far too slow to do repeatedly,
//! for the reason `demo_rig::RTT_POINTER_OFFSET` documents, so the
//! firmware publishes the address instead in that fixed backup-RAM slot.
//! A reconnect is then one two-word read and an attach at the address it
//! holds.
//!
//! **The slot is single-use.** Backup RAM survives a reset, so a value left
//! in it would go on naming the previous image's block long after that block
//! had been zeroed, and there would be no way to tell a fresh publication
//! from a stale one. The host therefore clears the magic the moment it has
//! used the address: a magic that is present means firmware has come up and
//! published since anyone last looked, and an absent one means nothing has
//! changed hands, so whatever block is already attached is still the right
//! one.
//!
//! The *debug session* can also die outright: a reset taken while the probe
//! was mid-transaction leaves it unusable, and no amount of retrying on it
//! recovers, so it is rebuilt from scratch when the core stops answering.
//!
//! # When nothing has been published
//!
//! A RAM scan is the fallback, and a stale block does not survive to
//! confuse it: both binaries place theirs inside `.bss`, and both `.bss`
//! regions start at the base of RAM and are within a few bytes of the same
//! length, so whichever image comes up zeroes the other's block before
//! `main` runs. If those layouts ever diverge, the deterministic fix is
//! `Rtt::attach_at` with the `_SEGGER_RTT` address read out of each ELF.

use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::{MemoryInterface, Session, SessionConfig};

use super::Transport;

/// Channel 0 up is the `rprintln!` terminal; the protocol has its own.
const LOG: usize = 0;
const UP: usize = 1;
const DOWN: usize = 0;
/// What `demo-rtt` names its protocol channels, so attaching to some other
/// firmware's control block is an error rather than a silent timeout.
const CHANNEL_NAME: &str = "samd5-boot";

/// How long to keep rescanning for a control block before giving up.
/// Both images build theirs at the top of `main`, so this covers the
/// reset itself and, when the application is the one coming up, BOOT's
/// CRC verify ahead of it.
const ATTACH_TIMEOUT: Duration = Duration::from_secs(10);

pub struct RttLink {
    /// `None` while the session is being rebuilt, which is also what
    /// releases the probe before it is claimed again.
    session: Option<Session>,
    rtt: Option<Rtt>,
    chip: String,
    /// Forward the target's `rprintln!` output to stderr. `xtask test`
    /// leaves it off unless `--log`, so the PASS/FAIL lines stand alone;
    /// `xtask link` forwards unless `--quiet`.
    log: bool,
    /// Partial line held back so a log line split across two reads is not
    /// printed as two.
    log_line: Vec<u8>,
}

impl RttLink {
    pub fn attach(chip: &str, log: bool) -> Result<Self> {
        let mut session = open_session(chip)?;
        let deadline = Instant::now() + ATTACH_TIMEOUT;
        let rtt = loop {
            match attach_published(&mut session).or_else(|_| attach_scan(&mut session)) {
                Ok(rtt) => break rtt,
                Err(e) if Instant::now() >= deadline => {
                    return Err(e.context(format!(
                        "no samd5-boot RTT link appeared within {ATTACH_TIMEOUT:?}"
                    )));
                }
                Err(_) => {}
            }
        };
        Ok(Self {
            session: Some(session),
            rtt: Some(rtt),
            chip: chip.to_string(),
            log,
            log_line: Vec::new(),
        })
    }

    /// Drop the debug session and build a new one.
    fn open_session(&mut self) -> Result<()> {
        self.session = None;
        self.rtt = None;
        self.session = Some(open_session(&self.chip)?);
        Ok(())
    }

    /// Whether the core still answers. A dead session fails here and is the
    /// signal to rebuild rather than retry.
    fn core_responds(&mut self) -> bool {
        self.session
            .as_mut()
            .is_some_and(|s| s.core(0).is_ok())
    }

    /// Whether the control block being held is still the live one. The image
    /// that comes up after a reset zeroes it on its way through `.bss`, so
    /// the `SEGGER RTT` ID string going away is how a handover is noticed.
    fn block_is_live(&mut self) -> bool {
        let Some((session, rtt)) = self.parts() else {
            return false;
        };
        let held = rtt.ptr();
        let Ok(mut core) = session.core(0) else {
            return false;
        };
        let mut id = [0u8; 16];
        core.read_8(held, &mut id).is_ok() && id.starts_with(b"SEGGER RTT")
    }

    fn parts(&mut self) -> Option<(&mut Session, &mut Rtt)> {
        Some((self.session.as_mut()?, self.rtt.as_mut()?))
    }

    fn read_channel(&mut self, ch: usize, buf: &mut [u8]) -> usize {
        let Some((session, rtt)) = self.parts() else {
            return 0;
        };
        let Ok(mut core) = session.core(0) else {
            return 0;
        };
        let Some(channel) = rtt.up_channel(ch) else {
            return 0;
        };
        channel.read(&mut core, buf).unwrap_or(0)
    }

    /// Drain the terminal channel and print whole lines to stderr, so target
    /// output interleaves with the host's without corrupting either.
    fn drain_log(&mut self) {
        if !self.log {
            return;
        }
        let mut buf = [0u8; 1024];
        let n = self.read_channel(LOG, &mut buf);
        if n == 0 {
            return;
        }
        self.log_line.extend_from_slice(&buf[..n]);
        while let Some(at) = self.log_line.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.log_line.drain(..=at).collect();
            eprintln!(
                "target| {}",
                String::from_utf8_lossy(&line).trim_end_matches(['\r', '\n'])
            );
        }
    }
}

/// Search RAM. Slow, so it is the fallback rather than the routine path.
fn attach_scan(session: &mut Session) -> Result<Rtt> {
    attach(session, &ScanRegion::Ram)
}

/// Go straight to the address the firmware published.
fn attach_published(session: &mut Session) -> Result<Rtt> {
    let addr = {
        let mut core = session.core(0).context("selecting core 0")?;
        published_ptr(&mut core).context("no unread publication")?
    };
    let rtt = attach(session, &ScanRegion::Exact(addr))?;
    // Consumed only once it has worked, so a failed attach does not throw
    // the address away.
    let mut core = session.core(0).context("selecting core 0")?;
    consume_published(&mut core);
    Ok(rtt)
}

/// Clear the magic, so this publication is not read a second time.
fn consume_published(core: &mut probe_rs::Core<'_>) {
    let _ = core.write_word_32(demo_rig::RTT_POINTER_ADDR as u64, 0);
}

/// The address in the rig's backup-RAM slot, if it holds an unread one.
fn published_ptr(core: &mut probe_rs::Core<'_>) -> Option<u64> {
    let mut slot = [0u32; 2];
    core.read_32(demo_rig::RTT_POINTER_ADDR as u64, &mut slot).ok()?;
    (slot[0] == demo_rig::RTT_POINTER_MAGIC).then_some(slot[1] as u64)
}

/// A fresh debug session, claiming the probe.
fn open_session(chip: &str) -> Result<Session> {
    Session::auto_attach(chip, SessionConfig::default())
        .with_context(|| format!("attaching to {chip}"))
}

/// Fails while the firmware has not yet built its control block, which is
/// normal for the first few attempts after a reset.
fn attach(session: &mut Session, region: &ScanRegion) -> Result<Rtt> {
    let mut core = session.core(0).context("selecting core 0")?;
    // The core has to be running both to build the block and to drain
    // anything written into it afterwards.
    if core.core_halted().unwrap_or(false) {
        core.run().context("resuming the core")?;
    }
    let rtt = Rtt::attach_region(&mut core, region)?;
    let name = rtt.up_channels.get(UP).and_then(|c| c.name()).unwrap_or("");
    if name != CHANNEL_NAME {
        let found: Vec<&str> = rtt
            .up_channels
            .iter()
            .map(|c| c.name().unwrap_or("<unnamed>"))
            .collect();
        bail!(
            "found an RTT control block at {:#x} with up channels {found:?}, but channel {UP} \
             is not {CHANNEL_NAME:?}.\n\
             The firmware answering is not an `rtt` build: an image that only calls \
             `rtt_init_print!` has exactly one channel and looks like this.\n\
             If BOOT is an `rtt` build but an older application is being booted ahead of it, \
             `cargo xtask request-update` sets the update request through the debugger so \
             BOOT waits instead.",
            rtt.ptr()
        );
    }
    Ok(rtt)
}

impl Transport for RttLink {
    /// Never fails: a dropped session, a missing channel and a failed read
    /// all report as no bytes, since the usual cause is a device mid-reset.
    /// The caller's own deadline decides when to give up, and
    /// [`Transport::reconnect`] is what puts the link back together.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.drain_log();
        Ok(self.read_channel(UP, buf))
    }

    fn write(&mut self, bytes: &[u8]) -> Result<()> {
        let mut rest = bytes;
        let deadline = Instant::now() + Duration::from_secs(30);
        while !rest.is_empty() {
            let Some((session, rtt)) = self.parts() else {
                bail!("the RTT link is not attached");
            };
            let mut core = session.core(0).context("selecting core 0")?;
            let channel = rtt
                .down_channel(DOWN)
                .ok_or_else(|| anyhow::anyhow!("down channel {DOWN} is missing"))?;
            // A short write means the target has not drained the ring yet,
            // and retrying costs nothing. What retrying cannot ride out is
            // the ring filling to the last byte, which probe-rs reports as
            // a corrupt control block rather than a short count; keeping
            // the body out of that window is `Link::begin_update`'s job,
            // not this loop's.
            let n = channel.write(&mut core, rest).context("writing to RTT")?;
            rest = &rest[n..];
            if n == 0 && Instant::now() >= deadline {
                bail!(
                    "the target stopped draining the RTT ring with {} bytes left",
                    rest.len()
                );
            }
        }
        Ok(())
    }

    fn clear_input(&mut self) -> Result<()> {
        let mut buf = [0u8; 1024];
        while self.read(&mut buf)? > 0 {}
        Ok(())
    }

    /// Cheap when nothing has moved: a fresh publication wins if there is
    /// one, otherwise the block in hand is checked in place, and only a
    /// handover with nothing published falls through to a RAM scan.
    fn reconnect(&mut self) -> Result<()> {
        self.drain_log();

        if !self.core_responds() {
            self.open_session()?;
        }
        // A publication that is still there is firmware announcing itself
        // since anyone last looked, and it wins over whatever is attached:
        // the new image's block can be somewhere else while the old one is
        // briefly still intact.
        if let Some(session) = self.session.as_mut()
            && let Ok(rtt) = attach_published(session)
        {
            self.log_line.clear();
            self.rtt = Some(rtt);
            return Ok(());
        }
        if self.block_is_live() {
            return Ok(());
        }
        self.log_line.clear();
        if let Some(session) = self.session.as_mut()
            && let Ok(rtt) = attach_scan(session)
        {
            self.rtt = Some(rtt);
        }
        // Not finding a block yet is normal mid-reset; the caller polls.
        Ok(())
    }

    fn describe(&self) -> String {
        format!("RTT on {} (up {UP}, down {DOWN})", self.chip)
    }
}
