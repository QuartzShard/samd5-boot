//! The host end of the demo protocol, over whichever transport the rig has.
//!
//! Framing is the same either way (COBS-delimited postcard, with a raw image
//! body following a `BeginUpdate`), so it lives here once and the transports
//! only have to move bytes.

use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use proto::{Feed, MAX_FRAME, Message};

pub mod rtt;
pub mod serial;

pub trait Transport {
    /// Whatever has arrived since the last call, which may be nothing.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize>;
    fn write(&mut self, bytes: &[u8]) -> Result<()>;
    /// Drop anything the device said before the current exchange started.
    fn clear_input(&mut self) -> Result<()>;
    /// Re-establish the link after the device resets. A serial port does not
    /// notice a reset; an RTT session does, because the control block is
    /// rebuilt by the firmware that comes up.
    fn reconnect(&mut self) -> Result<()> {
        Ok(())
    }
    fn describe(&self) -> String;
}

/// So the CLI can choose a transport at runtime without the harness being
/// generic over it twice.
impl Transport for Box<dyn Transport> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        (**self).read(buf)
    }
    fn write(&mut self, bytes: &[u8]) -> Result<()> {
        (**self).write(bytes)
    }
    fn clear_input(&mut self) -> Result<()> {
        (**self).clear_input()
    }
    fn reconnect(&mut self) -> Result<()> {
        (**self).reconnect()
    }
    fn describe(&self) -> String {
        (**self).describe()
    }
}

/// A transport plus the receive-side frame accumulator and any bytes that
/// arrived past a completed frame.
pub struct Link<T> {
    transport: T,
    decoder: proto::Decoder,
    pending: Vec<u8>,
}

impl<T: Transport> Link<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            decoder: proto::Decoder::new(),
            pending: Vec::new(),
        }
    }

    pub fn describe(&self) -> String {
        self.transport.describe()
    }

    pub fn flush_input(&mut self) -> Result<()> {
        self.pending.clear();
        self.decoder = proto::Decoder::new();
        self.transport.clear_input()
    }

    /// Pick the link back up after a reset, discarding anything half-decoded
    /// from before it.
    pub fn reconnect(&mut self) -> Result<()> {
        self.pending.clear();
        self.decoder = proto::Decoder::new();
        self.transport.reconnect()
    }

    /// Reconnect, then drop anything the device said before now.
    ///
    /// A failed reconnect is not itself an error here: the decoder reset
    /// still has to happen, and the device is most likely only mid-reset.
    pub fn resync(&mut self) -> Result<()> {
        let _ = self.reconnect();
        self.flush_input()
    }

    pub fn send(&mut self, msg: &Message) -> Result<()> {
        let mut buf = [0u8; MAX_FRAME];
        let frame = proto::encode(msg, &mut buf).context("encoding a frame")?;
        self.write_raw(frame)
    }

    pub fn write_raw(&mut self, bytes: &[u8]) -> Result<()> {
        self.transport.write(bytes)
    }

    /// Next decoded message, or `None` once `deadline` passes.
    pub fn recv(&mut self, deadline: Instant) -> Result<Option<Message>> {
        loop {
            while !self.pending.is_empty() {
                let input = std::mem::take(&mut self.pending);
                match self.decoder.feed(&input) {
                    Feed::Consumed => {}
                    Feed::Frame { msg, remaining } => {
                        self.pending = remaining.to_vec();
                        return Ok(Some(msg));
                    }
                    Feed::DeserError { remaining } => {
                        eprintln!("<- undecodable frame, resyncing");
                        self.pending = remaining.to_vec();
                    }
                    Feed::Overfull { remaining } => {
                        eprintln!("<- oversized frame, resyncing");
                        self.pending = remaining.to_vec();
                    }
                }
            }

            if Instant::now() >= deadline {
                return Ok(None);
            }

            let mut buf = [0u8; 512];
            let n = self.transport.read(&mut buf)?;
            if n > 0 {
                self.pending.extend_from_slice(&buf[..n]);
            }
        }
    }

    /// Round-trip a `Ping`, returning the payload the device echoed.
    ///
    /// BOOT answers pings whether or not an application exists, so a reply
    /// proves only that something on the far end is alive.
    pub fn ping(&mut self, payload: [u8; 8], timeout: Duration) -> Result<[u8; 8]> {
        self.resync()?;
        self.send(&Message::Ping(payload))?;
        let deadline = Instant::now() + timeout;
        while let Some(msg) = self.recv(deadline)? {
            if let Message::Pong(echo) = msg {
                return Ok(echo);
            }
        }
        bail!("no Pong within {timeout:?}")
    }
}
