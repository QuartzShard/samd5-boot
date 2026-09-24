//! Streaming writer for the inactive slot. [`words`] and [`pages`]
//! re-chunk a byte stream (little-endian words, 0xFF-padded tails, a
//! page only exists once a real byte arrived); [`FlashWriter::write`]
//! burns the page stream, the first page of each 16-page block erasing
//! it ahead so no separate erase pass is needed. The errata 2.14.1
//! cache disable is held for the writer's lifetime and Drop restores
//! the previous `CACHEDIS0`/`CACHEDIS1` bits, nothing more. The writer
//! buffers no partial page, since [`pages`] only ever yields whole
//! 0xFF-padded pages, so an early `?`-exit stops on a page boundary.

use crate::consts::{self, ERASED, PAGE_SIZE_WORDS};
use atsamd_hal::nvm::{self, Nvm, WriteGranularity};

/// A failure while writing an image into the inactive slot
pub enum FlashError {
    /// An erase or program command returned an NVMCTRL error
    Nvm(nvm::Error),
    /// The image ran past the end of the destination region: the inactive
    /// slot's application area, less the live SmartEEPROM reserve.
    ImageTooLarge,
}

impl From<nvm::Error> for FlashError {
    fn from(e: nvm::Error) -> Self {
        Self::Nvm(e)
    }
}

type Page = [u32; PAGE_SIZE_WORDS];

const ERASED_BYTE: u8 = ERASED.to_le_bytes()[0];

/// Page-at-a-time writer for one flash region, erasing a block ahead of the
/// first page of each
pub struct FlashWriter<'nvm> {
    nvm: &'nvm mut Nvm,
    begin: usize,
    end: usize,
    saved_cachedis: (bool, bool),
}

impl<'nvm> FlashWriter<'nvm> {
    /// # Safety
    ///
    /// `begin` and `end` are absolute flash addresses, `end` exclusive.
    /// Both must be block-aligned
    /// ([`ERASE_BLOCK_SIZE`](crate::consts::ERASE_BLOCK_SIZE)): the
    /// erase-ahead erases a whole block at a time and is not bounded by
    /// `end`, so an `end` inside a block destroys the rest of that block as
    /// well. `begin..end` must be valid to erase and program and must hold
    /// no currently executing code; the erase-ahead destroys it.
    ///
    /// `Nvmctrl.CTRLA.{CACHEDIS0,CACHEDIS1}` must not be altered while this
    /// type is alive.
    pub unsafe fn new(nvm: &'nvm mut Nvm, begin: usize, end: usize) -> Self {
        // Errata 2.14.1: NVM reads corrupt while the page buffer is being
        // written; workaround = CTRLA.CACHEDIS0/1 while programming.
        // `modify`, not `write`: CTRLA also carries RWS/AUTOWS/WMODE.
        let saved_cachedis = unsafe {
            let ctrla = nvm.registers().ctrla();
            let prior = ctrla.read();
            ctrla.modify(|_, w| {
                w.cachedis0().set_bit();
                w.cachedis1().set_bit()
            });
            (prior.cachedis0().bit(), prior.cachedis1().bit())
        };

        Self {
            nvm,
            begin,
            end,
            saved_cachedis,
        }
    }

    /// Burn the whole page stream; consumes the writer, so the cache
    /// configuration is restored on return, error paths included
    pub fn write(mut self, pages: impl Iterator<Item = Page>) -> Result<(), FlashError> {
        pages
            .enumerate()
            .try_for_each(|(i, page)| self.write_page(i, &page))
    }

    fn write_page(&mut self, index: usize, page: &Page) -> Result<(), FlashError> {
        let addr = self.begin + index * consts::PAGE_SIZE;
        if addr + consts::PAGE_SIZE > self.end {
            return Err(FlashError::ImageTooLarge);
        }
        // A block is 16 pages: the first page of each erases it ahead
        if index.is_multiple_of(consts::PAGES_PER_BLOCK) {
            unsafe { self.nvm.erase_flash(addr as *mut u32, 1)? }
        }
        unsafe {
            self.nvm
                .write_flash_from_slice(addr as *mut _, page, WriteGranularity::Page)?
        }
        Ok(())
    }
}

impl Drop for FlashWriter<'_> {
    fn drop(&mut self) {
        let (dis0, dis1) = self.saved_cachedis;
        unsafe {
            self.nvm.registers().ctrla().modify(|_, w| {
                w.cachedis0().bit(dis0);
                w.cachedis1().bit(dis1)
            });
        }
    }
}

/// Re-chunk a byte stream into little-endian words, 0xFF-padding a short
/// tail
pub fn words(mut bytes: impl Iterator<Item = u8>) -> impl Iterator<Item = u32> {
    core::iter::from_fn(move || {
        Some(u32::from_le_bytes([
            bytes.next()?,
            bytes.next().unwrap_or(ERASED_BYTE),
            bytes.next().unwrap_or(ERASED_BYTE),
            bytes.next().unwrap_or(ERASED_BYTE),
        ]))
    })
}

/// Re-chunk a word stream into whole pages, 0xFF-padding the last one
pub fn pages(mut words: impl Iterator<Item = u32>) -> impl Iterator<Item = Page> {
    core::iter::from_fn(move || {
        let mut page = [ERASED; PAGE_SIZE_WORDS];
        page[0] = words.next()?;
        for w in &mut page[1..] {
            *w = words.next().unwrap_or(ERASED);
        }
        Some(page)
    })
}
