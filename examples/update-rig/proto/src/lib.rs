//! Wire protocol shared by the RS485 host tool and the SAMD51 firmware.
//!
//! Control traffic is a single [`Message`] enum, postcard-serialized and
//! COBS-framed (a `0x00` delimiter ends each frame). After a
//! [`Message::BeginUpdate`] frame the sender streams `len` RAW image bytes
//! that are NOT a `Message`: decode the BeginUpdate frame, then treat
//! [`Feed::Frame::remaining`] and everything after it as the raw image so it
//! can flow straight into `Boot::install` without buffering.

#![cfg_attr(not(feature = "std"), no_std)]

use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::{Deserialize, Serialize};

/// Upper bound on a single COBS frame in bytes: the encode buffer and the
/// decoder's internal accumulator are both sized to this. The largest
/// `Message` is `Ping`/`Pong` at 9 postcard bytes (1 tag + 8 payload); COBS
/// plus the delimiter add a few more. 32 leaves comfortable headroom.
pub const MAX_FRAME: usize = 32;

/// Outcome of an install attempt, reported device -> host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Status {
    /// The device never sends this: a successful install swaps banks and
    /// reboots, so success is signalled by silence.
    Ok,
    VerifyFailed,
    FlashError,
    TooLarge,
}

/// A single control message. Raw image bytes stream after a `BeginUpdate`
/// frame and are deliberately not represented here.
///
/// Variants are positional on the wire: this enum only ever grows at the end,
/// since inserting one silently breaks every device already in the field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Message {
    /// host -> device: liveness probe carrying an arbitrary payload.
    Ping([u8; 8]),
    /// device -> host: echoes the `Ping` payload.
    Pong([u8; 8]),
    /// host -> device: exactly `len` raw image bytes follow this frame.
    BeginUpdate { len: u32 },
    /// device -> host.
    UpdateResult(Status),
    /// host -> app: flag an update request and reset, so BOOT waits for an
    /// image instead of booting.
    Update,
    /// host -> app: reset with no request flag, so BOOT takes its normal
    /// path. Used to spend trial attempts.
    Reset,
    /// host -> device; answered with `State`.
    GetState,
    /// device -> host: `app_version` is the running image's own build
    /// version, `revert_reason` is a `persist::RevertReason` code recording
    /// the last rollback, carried raw so a code this build cannot decode
    /// still arrives (a postcard enum would reject the whole frame), and
    /// `confirmed` says whether this image marked itself good on this boot.
    State {
        app_version: u16,
        revert_reason: u8,
        confirmed: bool,
    },
    /// host -> app: condemn the running image and reset, so BOOT rolls back.
    Reject,
}

/// Serialize and COBS-frame `msg` into `buf`, returning the used prefix.
///
/// `buf` should be at least [`MAX_FRAME`] bytes; a too-small buffer yields
/// `postcard::Error::SerializeBufferFull`.
pub fn encode<'a>(msg: &Message, buf: &'a mut [u8]) -> postcard::Result<&'a mut [u8]> {
    postcard::to_slice_cobs(msg, buf)
}

/// Result of feeding a chunk of received bytes to a [`Decoder`].
///
/// `remaining` is the input tail past the frame boundary; after a
/// [`Frame`](Feed::Frame) it belongs to the next frame (or, following a
/// `BeginUpdate`, to the raw image stream).
pub enum Feed<'a> {
    /// No complete frame yet; all input was buffered.
    Consumed,
    /// A complete frame decoded into a [`Message`].
    Frame { msg: Message, remaining: &'a [u8] },
    /// A complete frame arrived but failed to deserialize; decoder was reset.
    DeserError { remaining: &'a [u8] },
    /// A frame exceeded [`MAX_FRAME`] before its delimiter; decoder was reset.
    Overfull { remaining: &'a [u8] },
}

/// Receive-side COBS frame accumulator that yields decoded [`Message`]s.
///
/// Feed it whatever bytes arrive; it holds a partial frame across calls and
/// surfaces a [`Message`] the moment a delimiter completes one.
pub struct Decoder {
    acc: CobsAccumulator<MAX_FRAME>,
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            acc: CobsAccumulator::new(),
        }
    }

    /// Feed received bytes; see [`Feed`] for the outcomes.
    ///
    /// When `remaining` on the returned variant is non-empty, feed it back in
    /// (or, past a `BeginUpdate`, hand it to the raw image sink).
    pub fn feed<'a>(&mut self, input: &'a [u8]) -> Feed<'a> {
        match self.acc.feed::<Message>(input) {
            FeedResult::Consumed => Feed::Consumed,
            FeedResult::Success { data, remaining } => Feed::Frame {
                msg: data,
                remaining,
            },
            FeedResult::DeserError(remaining) => Feed::DeserError { remaining },
            FeedResult::OverFull(remaining) => Feed::Overfull { remaining },
        }
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(msg: Message) {
        let mut buf = [0u8; MAX_FRAME];
        let frame = encode(&msg, &mut buf).unwrap();
        let mut dec = Decoder::new();
        match dec.feed(frame) {
            Feed::Frame { msg: got, remaining } => {
                assert_eq!(got, msg);
                assert!(remaining.is_empty());
            }
            _ => panic!("expected a frame"),
        }
    }

    #[test]
    fn every_variant_roundtrips() {
        roundtrip(Message::Ping([1, 2, 3, 4, 5, 6, 7, 8]));
        roundtrip(Message::Pong([9, 10, 11, 12, 13, 14, 15, 16]));
        roundtrip(Message::BeginUpdate { len: 0xDEAD_BEEF });
        roundtrip(Message::UpdateResult(Status::VerifyFailed));
        roundtrip(Message::Update);
        roundtrip(Message::Reset);
        roundtrip(Message::GetState);
        roundtrip(Message::State {
            app_version: 0x1234,
            revert_reason: 3,
            confirmed: true,
        });
        roundtrip(Message::Reject);
    }

    #[test]
    fn frame_boundary_surfaces_trailing_raw_bytes() {
        // A BeginUpdate frame immediately followed by raw image bytes in the
        // same chunk: the raw tail must come back as `remaining`.
        let mut buf = [0u8; MAX_FRAME];
        let frame = encode(&Message::BeginUpdate { len: 4 }, &mut buf).unwrap();
        let mut wire = frame.to_vec();
        wire.extend_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);

        let mut dec = Decoder::new();
        match dec.feed(&wire) {
            Feed::Frame { msg, remaining } => {
                assert_eq!(msg, Message::BeginUpdate { len: 4 });
                assert_eq!(remaining, &[0xAA, 0xBB, 0xCC, 0xDD]);
            }
            _ => panic!("expected a frame"),
        }
    }

    #[test]
    fn split_frame_accumulates_across_feeds() {
        let mut buf = [0u8; MAX_FRAME];
        let frame = encode(&Message::Ping([7; 8]), &mut buf).unwrap().to_vec();
        let (a, b) = frame.split_at(3);

        let mut dec = Decoder::new();
        assert!(matches!(dec.feed(a), Feed::Consumed));
        match dec.feed(b) {
            Feed::Frame { msg, .. } => assert_eq!(msg, Message::Ping([7; 8])),
            _ => panic!("expected a frame"),
        }
    }
}
