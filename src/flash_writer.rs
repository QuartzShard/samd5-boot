//! Streaming writer for the inactive slot. [`words`] and [`pages`]
//! re-chunk a byte stream (little-endian words, 0xFF-padded tails, a
//! page only exists once a real byte arrived); [`FlashWriter::write`]
//! burns the page stream, the first page of each 16-page block erasing
//! it ahead so no separate erase pass is needed. The errata 2.14.1
//! cache disable is held for the writer's lifetime; Drop restores the
//! previous cache configuration and nothing else, so an early `?`-exit
//! never writes a half-filled page.

use crate::consts::{self, ERASED, PAGE_SIZE_WORDS};
use atsamd_hal::nvm::{self, Nvm, WriteGranularity};

pub enum FlashError {
    Nvm(nvm::Error),
    /// The stream ran past the writer's `end` bound.
    ImageTooLarge,
}

impl From<nvm::Error> for FlashError {
    fn from(e: nvm::Error) -> Self {
        Self::Nvm(e)
    }
}

type Page = [u32; PAGE_SIZE_WORDS];

pub struct FlashWriter<'nvm> {
    nvm: &'nvm mut Nvm,
    begin: usize,
    end: usize,
    saved_cachedis: (bool, bool),
}

impl<'nvm> FlashWriter<'nvm> {
    /// # Safety
    ///
    /// `begin` must be block-aligned and `begin..end` valid to erase
    /// and program, holding no currently executing code. The
    /// erase-ahead destroys it.
    ///
    /// NVM.CTRLA.CACHEDIS0/1 must not be altered while this type is alive
    pub unsafe fn new(nvm: &'nvm mut Nvm, begin: usize, end: usize) -> Self {
        // Errata 2.14.1: NVM reads corrupt while the page buffer is being
        // written; workaround = CTRLA.CACHEDIS0/1 while programming.
        // `modify`, not `write`: CTRLA also carries RWS/AUTOWS/WMODE. The
        // prior cache state is restored on drop rather than assumed on.
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
    /// configuration is restored on return, error paths included.
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

pub fn words(mut bytes: impl Iterator<Item = u8>) -> impl Iterator<Item = u32> {
    core::iter::from_fn(move || {
        Some(u32::from_le_bytes([
            bytes.next()?,
            bytes.next().unwrap_or(0xFF),
            bytes.next().unwrap_or(0xFF),
            bytes.next().unwrap_or(0xFF),
        ]))
    })
}

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
