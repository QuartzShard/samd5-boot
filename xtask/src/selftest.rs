//! The rig's self-test: drives the bootloader's update paths over the link
//! and prints one PASS/FAIL line per check. [`run`] is the whole suite, in
//! order and against one device: install, verify-and-reject, trial, both
//! rollbacks, and the application-requested update window. Each check
//! starts from the state the one before it left.

use std::fs;
use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use proto::{Message, Status};
use samd5_boot::{manifest, persist::RevertReason};

use crate::image;
use crate::link::{Link, Transport};

/// Long enough for a reset, BOOT's verify, and the app coming up.
const APP_TIMEOUT: Duration = Duration::from_secs(30);
/// A rejected install answers quickly; a successful one never answers.
const INSTALL_TIMEOUT: Duration = Duration::from_secs(20);

struct Report {
    passed: usize,
    failed: usize,
}

impl Report {
    fn check(&mut self, name: &str, outcome: Result<()>) {
        match outcome {
            Ok(()) => {
                self.passed += 1;
                println!("[PASS] {name}");
            }
            Err(why) => {
                self.failed += 1;
                println!("[FAIL] {name}: {why:#}");
            }
        }
    }
}

struct AppState {
    version: u16,
    revert_reason: u8,
    confirmed: bool,
}

impl AppState {
    fn reason(&self) -> Option<RevertReason> {
        RevertReason::from_u8(self.revert_reason)
    }
}

/// Poll `GetState` until the application answers.
///
/// `State` comes from the application alone, which is what makes it the
/// signal that a boot has actually completed.
fn wait_for_app<T: Transport>(link: &mut Link<T>, timeout: Duration) -> Result<AppState> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        // The image that answers may not be the one that was running when
        // this started, and on RTT that means a different control block.
        let _ = link.resync();
        if link.send(&Message::GetState).is_err() {
            continue;
        }
        let reply_by = Instant::now() + Duration::from_millis(600);
        while let Ok(Some(msg)) = link.recv(reply_by) {
            if let Message::State {
                app_version,
                revert_reason,
                confirmed,
            } = msg
            {
                return Ok(AppState {
                    version: app_version,
                    revert_reason,
                    confirmed,
                });
            }
        }
    }
    bail!("no application answered within {timeout:?}")
}

/// Push an image and report what the device said.
///
/// `Ok(None)` means the device never answered, which is the success case:
/// `Boot::install` swaps and reboots rather than replying.
fn send_image<T: Transport>(link: &mut Link<T>, image: &[u8]) -> Result<Option<Status>> {
    let len: u32 = image.len().try_into().context("image too large")?;
    link.flush_input()?;
    link.send(&Message::BeginUpdate { len })?;
    link.write_raw(image)?;

    let deadline = Instant::now() + INSTALL_TIMEOUT;
    loop {
        match link.recv(deadline) {
            Ok(Some(Message::UpdateResult(status))) => return Ok(Some(status)),
            Ok(Some(_)) => continue,
            Ok(None) => break,
            Err(e) => return Err(e),
        }
    }
    Ok(None)
}

/// Ask the application to request an update and reset, so BOOT waits for
/// the host rather than booting on.
fn reboot<T: Transport>(link: &mut Link<T>) -> Result<()> {
    link.resync()?;
    link.send(&Message::Update)
}

/// Reset the application without requesting an update, which spends one
/// trial attempt.
fn plain_reset<T: Transport>(link: &mut Link<T>) -> Result<()> {
    link.resync()?;
    link.send(&Message::Reset)
}

/// Wait until the application has gone, so BOOT owns the link.
///
/// BOOT waits for an image without a deadline, so there is no window to
/// race: the only thing to establish is that the application is no longer
/// the one answering. `GetState` is answered by the application alone,
/// which makes its silence the signal.
///
/// It can take two resets. When the running image has just confirmed a
/// trial, the next boot is the promotion, which outranks the update
/// request in `Boot::disposition` and boots the application again with the
/// request still pending; the trial watchdog it re-armed in `confirm`
/// supplies the second reset. Hence the 20 s deadline.
fn wait_for_boot<T: Transport>(link: &mut Link<T>) -> Result<()> {
    let deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < deadline {
        let _ = link.resync();
        if link.send(&Message::GetState).is_err() {
            continue;
        }
        let by = Instant::now() + Duration::from_millis(400);
        let mut app_answered = false;
        while let Ok(Some(msg)) = link.recv(by) {
            if matches!(msg, Message::State { .. }) {
                app_answered = true;
            }
        }
        if !app_answered {
            return Ok(());
        }
    }
    bail!("the application never stopped answering")
}

/// Install `image`, waiting for the application to come back up.
fn install_and_wait<T: Transport>(link: &mut Link<T>, image: &[u8]) -> Result<AppState> {
    reboot(link)?;
    wait_for_boot(link)?;
    if let Some(status) = send_image(link, image)? {
        bail!("device rejected the image: {status:?}");
    }
    wait_for_app(link, APP_TIMEOUT)
}

/// Run every check against the device on `link`, returning whether they all
/// passed.
///
/// `good` is the confirming build and `noconfirm` the one that never
/// confirms. Both must be stamped, and with different versions, or the
/// rollback checks cannot tell which image answered.
pub fn run<T: Transport>(link: &mut Link<T>, good: &Path, noconfirm: &Path) -> Result<bool> {
    let good_image = fs::read(good).with_context(|| good.display().to_string())?;
    let noconfirm_image = fs::read(noconfirm).with_context(|| noconfirm.display().to_string())?;

    // Each image reports a version on the wire from its own `APP_VERSION`
    // const, and `xtask build` stamps the matching value into the manifest,
    // so read it from the file rather than restating it here. The two
    // definitions have to agree for these comparisons to mean anything.
    let good_version = image_version(&good_image, good)?;
    let noconfirm_version = image_version(&noconfirm_image, noconfirm)?;
    if good_version == noconfirm_version {
        bail!(
            "both images report version {good_version}; the second must be the \
             --features noconfirm build, or the rollback paths cannot be told apart"
        );
    }

    let mut r = Report {
        passed: 0,
        failed: 0,
    };
    println!("running the rig self-test, this takes about a minute\n");

    // The link itself: whichever of BOOT and the application is up answers.
    r.check(
        "link: Ping is answered",
        (|| {
            let echo = link.ping(*b"selftest", Duration::from_secs(3))?;
            if &echo == b"selftest" {
                Ok(())
            } else {
                bail!("payload did not echo")
            }
        })(),
    );

    // Install, verify, swap, boot: the happy path, ending in an image that
    // has confirmed itself, which promotes it out of trial on its next boot.
    let baseline = install_and_wait(link, &good_image);
    r.check(
        "install: a good image is accepted, booted, and confirms itself",
        baseline.and_then(|s| {
            if s.version != good_version {
                bail!("came back running version {}", s.version)
            } else if !s.confirmed {
                bail!("image did not confirm")
            } else {
                Ok(())
            }
        }),
    );

    // A corrupted image must be refused by CRC and must not displace the
    // image already installed.
    let mut corrupted = good_image.clone();
    image::corrupt_body(&mut corrupted);
    r.check(
        "verify: a corrupted image is rejected as VerifyFailed",
        (|| {
            reboot(link)?;
            wait_for_boot(link)?;
            match send_image(link, &corrupted)? {
                Some(Status::VerifyFailed) => Ok(()),
                Some(other) => bail!("device said {other:?}"),
                None => bail!("device accepted a corrupted image"),
            }
        })(),
    );
    r.check(
        "verify: the running image survives a rejected update",
        wait_for_app(link, APP_TIMEOUT).and_then(|s| {
            if s.version == good_version {
                Ok(())
            } else {
                bail!("running version {} instead", s.version)
            }
        }),
    );

    // An image that never confirms stays on trial.
    r.check(
        "trial: an unconfirmed image boots and reports confirmed=false",
        install_and_wait(link, &noconfirm_image).and_then(|s| {
            if s.version != noconfirm_version {
                bail!("came back running version {}", s.version)
            } else if s.confirmed {
                bail!("image claimed to be confirmed")
            } else {
                Ok(())
            }
        }),
    );

    // The application condemning itself rolls the device straight back.
    r.check(
        "revert: an application that rejects itself is rolled back",
        (|| {
            link.resync()?;
            link.send(&Message::Reject)?;
            let s = wait_for_app(link, APP_TIMEOUT)?;
            if s.version != good_version {
                bail!("rolled back to version {} instead", s.version)
            } else if s.reason() != Some(RevertReason::AppRejected) {
                bail!("revert reason was {:?}", s.reason())
            } else {
                Ok(())
            }
        })(),
    );

    // Left to run, an unconfirmed image is rolled back once its trial
    // attempts are spent.
    r.check(
        "revert: an unconfirmed image is rolled back once attempts run out",
        (|| {
            let trial = install_and_wait(link, &noconfirm_image)?;
            if trial.version != noconfirm_version {
                bail!("trial image did not boot, saw {}", trial.version);
            }
            // Each reset spends one attempt; the demo BOOT allows 3, then
            // reverts.
            for _ in 0..6 {
                if plain_reset(link).is_err() {
                    break;
                }
                if let Ok(s) = wait_for_app(link, APP_TIMEOUT)
                    && s.version == good_version
                {
                    return if s.reason() == Some(RevertReason::AttemptsExhausted) {
                        Ok(())
                    } else {
                        Err(anyhow!("revert reason was {:?}", s.reason()))
                    };
                }
            }
            bail!("device never rolled back")
        })(),
    );

    // Back to the confirming image, which re-exercises the
    // application-requested update window.
    r.check(
        "window: an application-requested reboot accepts a new image",
        install_and_wait(link, &good_image).and_then(|s| {
            if s.version == good_version && s.reason() == Some(RevertReason::None) && s.confirmed {
                Ok(())
            } else {
                bail!(
                    "version {} reason {:?} confirmed {}",
                    s.version,
                    s.reason(),
                    s.confirmed
                )
            }
        }),
    );

    println!("\n{} passed, {} failed", r.passed, r.failed);
    Ok(r.failed == 0)
}

/// The version a stamped image will report once it is running.
fn image_version(image: &[u8], path: &Path) -> Result<u16> {
    manifest::read_version(image).with_context(|| {
        format!(
            "{} carries no samd5-boot manifest; stamp it with `cargo xtask stamp`",
            path.display()
        )
    })
}
