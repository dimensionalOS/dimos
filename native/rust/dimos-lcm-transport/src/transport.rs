// Copyright 2025-2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use byteorder::{BigEndian, ByteOrder};
use socket2::{Domain, Protocol, Socket, Type};
use std::collections::HashMap;
use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;

const MAGIC_SHORT: u32 = 0x4c433032; // "LC02"
const MAGIC_LONG: u32 = 0x4c433033; // "LC03"
const SHORT_HEADER_SIZE: usize = 8;
const FRAGMENT_HEADER_SIZE: usize = 20;
const MAX_DATAGRAM_SIZE: usize = 65507;
// Bound incomplete UDP messages independently of sender-supplied lengths.
const MAX_MESSAGE_SIZE: usize = 64 * 1024 * 1024;
const MAX_REASSEMBLY_BYTES: usize = 128 * 1024 * 1024;
const MAX_INCOMPLETE_MESSAGES: usize = 64;
const REASSEMBLY_TIMEOUT: Duration = Duration::from_secs(5);

/// Default LCM multicast group address.
pub const DEFAULT_MULTICAST_GROUP: Ipv4Addr = Ipv4Addr::new(239, 255, 76, 67);
/// Default LCM multicast port.
pub const DEFAULT_PORT: u16 = 7667;

static SEQ: AtomicU32 = AtomicU32::new(0);

struct FragmentBuffer {
    channel: Option<String>,
    fragments: Vec<Option<(usize, usize)>>,
    received: usize,
    received_bytes: usize,
    created: Instant,
    data: Vec<u8>,
}

/// Configuration for an LCM transport instance.
#[derive(Debug, Clone)]
pub struct LcmOptions {
    /// Multicast group address (default: 239.255.76.67).
    pub multicast_group: Ipv4Addr,
    /// UDP port (default: 7667).
    pub port: u16,
    /// Multicast TTL (default: 1).
    pub ttl: u32,
    /// Network interface to bind to (default: any).
    pub interface: Ipv4Addr,
}

impl Default for LcmOptions {
    fn default() -> Self {
        Self {
            multicast_group: DEFAULT_MULTICAST_GROUP,
            port: DEFAULT_PORT,
            ttl: 1,
            interface: Ipv4Addr::UNSPECIFIED,
        }
    }
}

/// A received LCM message.
#[derive(Debug, Clone)]
pub struct ReceivedMessage {
    /// Channel name.
    pub channel: String,
    /// Encoded message payload.
    pub data: Vec<u8>,
}

/// Returns the first and subsequent payload sizes, and the number of fragments
fn fragment_params(msg_size: usize, channel_len: usize) -> (usize, usize, usize) {
    let first_payload_size = MAX_DATAGRAM_SIZE - FRAGMENT_HEADER_SIZE - channel_len - 1;
    let subsequent_payload_size = MAX_DATAGRAM_SIZE - FRAGMENT_HEADER_SIZE;
    let num_fragments = if msg_size <= first_payload_size {
        1
    } else {
        1 + msg_size
            .saturating_sub(first_payload_size)
            .div_ceil(subsequent_payload_size)
    };
    (first_payload_size, subsequent_payload_size, num_fragments)
}

/// Pure Rust LCM UDP multicast transport.
pub struct Lcm {
    socket: UdpSocket,
    multicast_addr: SocketAddrV4,
    reassembly: Mutex<HashMap<(SocketAddr, u32), FragmentBuffer>>,
}

impl Lcm {
    /// Create a new LCM transport with default options.
    pub async fn new() -> io::Result<Self> {
        Self::with_options(LcmOptions::default()).await
    }

    /// Create a new LCM transport with custom options.
    pub async fn with_options(opts: LcmOptions) -> io::Result<Self> {
        let s2 = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
        s2.set_reuse_address(true)?;
        // Sensor messages arrive as bursts of large fragments. Request enough
        // queue space to reassemble an image without relying on kernel defaults.
        s2.set_recv_buffer_size(4 * 1024 * 1024)?;
        #[cfg(not(target_os = "windows"))]
        s2.set_reuse_port(true)?;

        let bind_addr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, opts.port);
        s2.bind(&bind_addr.into())?;

        let std_socket: std::net::UdpSocket = s2.into();
        std_socket.set_nonblocking(true)?;
        let socket = UdpSocket::from_std(std_socket)?;

        socket.join_multicast_v4(opts.multicast_group, opts.interface)?;
        socket.set_multicast_ttl_v4(opts.ttl)?;

        Ok(Self {
            socket,
            multicast_addr: SocketAddrV4::new(opts.multicast_group, opts.port),
            reassembly: Mutex::new(HashMap::new()),
        })
    }

    /// Publish encoded message data on the given channel.
    pub async fn publish(&self, channel: &str, data: &[u8]) -> io::Result<()> {
        let channel_bytes = channel.as_bytes();
        if channel_bytes.len() > 63 || channel_bytes.contains(&0) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "LCM channel must fit in 63 bytes and contain no NUL",
            ));
        }
        if data.len() > MAX_MESSAGE_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "LCM message exceeds the 64 MiB transport limit",
            ));
        }
        let total = SHORT_HEADER_SIZE + channel_bytes.len() + 1 + data.len();
        let seqno = SEQ.fetch_add(1, Ordering::Relaxed);

        if total > MAX_DATAGRAM_SIZE {
            self.publish_fragmented(channel_bytes, data, seqno).await
        } else {
            self.publish_small(channel_bytes, data, seqno).await
        }
    }

    async fn publish_small(&self, channel_bytes: &[u8], data: &[u8], seqno: u32) -> io::Result<()> {
        let total = SHORT_HEADER_SIZE + channel_bytes.len() + 1 + data.len();
        let mut buf = vec![0u8; total];

        BigEndian::write_u32(&mut buf[0..4], MAGIC_SHORT);
        BigEndian::write_u32(&mut buf[4..8], seqno);

        buf[SHORT_HEADER_SIZE..SHORT_HEADER_SIZE + channel_bytes.len()]
            .copy_from_slice(channel_bytes);
        // null terminator already 0 from vec![0u8; ..]
        let payload_start = SHORT_HEADER_SIZE + channel_bytes.len() + 1;
        buf[payload_start..].copy_from_slice(data);

        self.socket.send_to(&buf, self.multicast_addr).await?;
        Ok(())
    }

    async fn publish_fragmented(
        &self,
        channel_bytes: &[u8],
        data: &[u8],
        seqno: u32,
    ) -> io::Result<()> {
        let msg_size = data.len();
        let (first_payload_size, subsequent_payload_size, num_fragments) =
            fragment_params(msg_size, channel_bytes.len());

        let mut payload_offset = 0;

        for fragment_no in 0..num_fragments {
            let is_first = fragment_no == 0;
            let channel_size = if is_first { channel_bytes.len() + 1 } else { 0 };
            let max_payload = if is_first {
                first_payload_size
            } else {
                subsequent_payload_size
            };
            let payload_len = (msg_size - payload_offset).min(max_payload);

            let datagram_size = FRAGMENT_HEADER_SIZE + channel_size + payload_len;
            let mut buf = vec![0u8; datagram_size];

            BigEndian::write_u32(&mut buf[0..4], MAGIC_LONG);
            BigEndian::write_u32(&mut buf[4..8], seqno);
            BigEndian::write_u32(&mut buf[8..12], msg_size as u32);
            BigEndian::write_u32(&mut buf[12..16], payload_offset as u32);
            BigEndian::write_u16(&mut buf[16..18], fragment_no as u16);
            BigEndian::write_u16(&mut buf[18..20], num_fragments as u16);

            let mut offset = FRAGMENT_HEADER_SIZE;

            if is_first {
                buf[offset..offset + channel_bytes.len()].copy_from_slice(channel_bytes);
                // null terminator already 0
                offset += channel_bytes.len() + 1;
            }

            buf[offset..offset + payload_len]
                .copy_from_slice(&data[payload_offset..payload_offset + payload_len]);

            self.socket.send_to(&buf, self.multicast_addr).await?;
            payload_offset += payload_len;
        }

        Ok(())
    }

    /// Receive one LCM message asynchronously.
    ///
    /// Waits until a complete message arrives, reassembling fragments if necessary.
    pub async fn recv(&self) -> io::Result<ReceivedMessage> {
        let mut buf = vec![0u8; MAX_DATAGRAM_SIZE];
        loop {
            let (n, sender) = self.socket.recv_from(&mut buf).await?;
            let pkt = &buf[..n];

            if pkt.len() < 4 {
                continue;
            }
            let magic = BigEndian::read_u32(&pkt[0..4]);

            if magic == MAGIC_SHORT {
                if let Some(msg) = Self::decode_small(pkt)? {
                    return Ok(msg);
                }
            } else if magic == MAGIC_LONG {
                if let Some(msg) = self.process_fragment(sender, pkt)? {
                    return Ok(msg);
                }
            }
            // Unknown magic or incomplete fragment — wait for the next datagram
        }
    }

    fn process_fragment(
        &self,
        sender: SocketAddr,
        buf: &[u8],
    ) -> io::Result<Option<ReceivedMessage>> {
        if buf.len() < FRAGMENT_HEADER_SIZE {
            return Ok(None);
        }

        let seqno = BigEndian::read_u32(&buf[4..8]);
        let total_size = BigEndian::read_u32(&buf[8..12]) as usize;
        let fragment_offset = BigEndian::read_u32(&buf[12..16]) as usize;
        let fragment_no = BigEndian::read_u16(&buf[16..18]);
        let num_fragments = BigEndian::read_u16(&buf[18..20]);

        let mut offset = FRAGMENT_HEADER_SIZE;

        if total_size == 0
            || total_size > MAX_MESSAGE_SIZE
            || num_fragments == 0
            || fragment_no >= num_fragments
            || usize::from(num_fragments) > total_size
            || (fragment_no == 0 && fragment_offset != 0)
        {
            return Ok(None);
        }

        // Only fragment zero carries the channel. Reject invalid UTF-8 rather
        // than routing a replacement-character channel to the wrong subscriber.
        let channel = if fragment_no == 0 {
            let Some(pos) = buf[offset..].iter().position(|&b| b == 0) else {
                return Ok(None);
            };
            if pos > 63 {
                return Ok(None);
            }
            let Ok(ch) = std::str::from_utf8(&buf[offset..offset + pos]) else {
                return Ok(None);
            };
            offset += pos + 1;
            Some(ch.to_owned())
        } else {
            None
        };
        let payload = &buf[offset..];
        let Some(end) = fragment_offset.checked_add(payload.len()) else {
            return Ok(None);
        };
        if payload.is_empty() || end > total_size {
            return Ok(None);
        }

        let key = (sender, seqno);
        let now = Instant::now();
        let mut reassembly = self.reassembly.lock().unwrap();
        reassembly.retain(|_, entry| now.duration_since(entry.created) < REASSEMBLY_TIMEOUT);
        if !reassembly.contains_key(&key) {
            let used: usize = reassembly.values().map(|entry| entry.data.len()).sum();
            if reassembly.len() >= MAX_INCOMPLETE_MESSAGES
                || total_size > MAX_REASSEMBLY_BYTES - used
            {
                return Ok(None);
            }
        }
        let entry = reassembly.entry(key).or_insert_with(|| FragmentBuffer {
            channel: None,
            fragments: vec![None; usize::from(num_fragments)],
            received: 0,
            received_bytes: 0,
            created: now,
            data: vec![0u8; total_size],
        });
        if entry.data.len() != total_size || entry.fragments.len() != usize::from(num_fragments) {
            reassembly.remove(&key);
            return Ok(None);
        }
        if let Some(range) = entry.fragments[usize::from(fragment_no)] {
            // Repeated datagrams do not advance completion. Conflicting repeats
            // invalidate this message instead of silently replacing its bytes.
            if range != (fragment_offset, end)
                || entry.data[fragment_offset..end] != *payload
                || (fragment_no == 0 && entry.channel != channel)
            {
                reassembly.remove(&key);
            }
            return Ok(None);
        }
        if entry.received_bytes + payload.len() > total_size {
            reassembly.remove(&key);
            return Ok(None);
        }
        if channel.is_some() {
            entry.channel = channel;
        }
        entry.data[fragment_offset..end].copy_from_slice(payload);
        entry.fragments[usize::from(fragment_no)] = Some((fragment_offset, end));
        entry.received += 1;
        entry.received_bytes += payload.len();

        if entry.received == entry.fragments.len() {
            let complete = reassembly.remove(&key).unwrap();
            if complete.received_bytes != total_size {
                return Ok(None);
            }
            let mut ranges: Vec<_> = complete.fragments.into_iter().flatten().collect();
            ranges.sort_unstable();
            let mut expected = 0;
            for (start, end) in ranges {
                if start != expected {
                    return Ok(None); // overlap or hole
                }
                expected = end;
            }
            if expected == total_size {
                if let Some(channel) = complete.channel {
                    return Ok(Some(ReceivedMessage {
                        channel,
                        data: complete.data,
                    }));
                }
            }
        }
        Ok(None)
    }

    fn decode_small(buf: &[u8]) -> io::Result<Option<ReceivedMessage>> {
        if buf.len() < SHORT_HEADER_SIZE || BigEndian::read_u32(&buf[0..4]) != MAGIC_SHORT {
            return Ok(None);
        }
        let channel_start = SHORT_HEADER_SIZE;
        let channel_end = match buf[channel_start..].iter().position(|&b| b == 0) {
            Some(pos) => channel_start + pos,
            None => return Ok(None),
        };
        if channel_end - channel_start > 63 {
            return Ok(None);
        }
        let Ok(channel) = std::str::from_utf8(&buf[channel_start..channel_end]) else {
            return Ok(None);
        };
        let channel = channel.to_owned();
        let data = buf[channel_end + 1..].to_vec();
        Ok(Some(ReceivedMessage { channel, data }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn invalid_channels_are_rejected_before_sending() {
        let lcm = Lcm::with_options(LcmOptions {
            port: 0,
            ..Default::default()
        })
        .await
        .unwrap();
        for channel in ["x".repeat(64), "é".repeat(32), "invalid\0channel".into()] {
            let error = lcm.publish(&channel, b"payload").await.unwrap_err();
            assert_eq!(error.kind(), io::ErrorKind::InvalidInput);
        }
    }

    async fn receiver() -> Lcm {
        Lcm::with_options(LcmOptions {
            port: 0,
            ..Default::default()
        })
        .await
        .unwrap()
    }

    fn fragment(seq: u32, size: u32, offset: u32, index: u16, count: u16, data: &[u8]) -> Vec<u8> {
        let mut packet = vec![0; FRAGMENT_HEADER_SIZE];
        BigEndian::write_u32(&mut packet[0..4], MAGIC_LONG);
        BigEndian::write_u32(&mut packet[4..8], seq);
        BigEndian::write_u32(&mut packet[8..12], size);
        BigEndian::write_u32(&mut packet[12..16], offset);
        BigEndian::write_u16(&mut packet[16..18], index);
        BigEndian::write_u16(&mut packet[18..20], count);
        if index == 0 {
            packet.extend_from_slice(b"CHAN\0");
        }
        packet.extend_from_slice(data);
        packet
    }

    fn accept(lcm: &Lcm, packet: &[u8]) -> Option<ReceivedMessage> {
        lcm.process_fragment("127.0.0.1:1234".parse().unwrap(), packet)
            .unwrap()
    }

    #[tokio::test]
    async fn reordered_and_repeated_fragments_complete_once_with_exact_bytes() {
        let lcm = receiver().await;
        let last = fragment(1, 6, 4, 2, 3, b"ef");
        assert!(accept(&lcm, &last).is_none());
        assert!(accept(&lcm, &last).is_none());
        assert!(accept(&lcm, &fragment(1, 6, 0, 0, 3, b"ab")).is_none());
        let message = accept(&lcm, &fragment(1, 6, 2, 1, 3, b"cd")).unwrap();
        assert_eq!(message.channel, "CHAN");
        assert_eq!(message.data, b"abcdef");
        assert!(lcm.reassembly.lock().unwrap().is_empty());
    }

    #[tokio::test]
    async fn invalid_headers_and_ranges_do_not_allocate_reassembly() {
        let lcm = receiver().await;
        for packet in [
            fragment(1, u32::MAX, 0, 0, 2, b"ab"),
            fragment(1, 4, u32::MAX, 1, 2, b"cd"),
            fragment(1, 4, 3, 1, 2, b"cd"),
            fragment(1, 4, 0, 0, 0, b"ab"),
            fragment(1, 4, 0, 2, 2, b"ab"),
            fragment(1, 1, 0, 0, 2, b"a"),
            fragment(1, 4, 1, 0, 2, b"ab"),
            fragment(1, 4, 0, 0, 2, b""),
        ] {
            assert!(accept(&lcm, &packet).is_none());
            assert!(lcm.reassembly.lock().unwrap().is_empty());
        }
    }

    #[tokio::test]
    async fn conflicting_metadata_and_repeated_payloads_invalidate_message() {
        for conflicting in [
            fragment(1, 5, 2, 1, 2, b"cd"),
            fragment(1, 4, 2, 1, 3, b"cd"),
            fragment(1, 4, 0, 0, 2, b"zz"),
        ] {
            let lcm = receiver().await;
            assert!(accept(&lcm, &fragment(1, 4, 0, 0, 2, b"ab")).is_none());
            assert!(accept(&lcm, &conflicting).is_none());
            assert!(lcm.reassembly.lock().unwrap().is_empty());
        }
    }

    #[tokio::test]
    async fn overlaps_and_holes_never_produce_a_message() {
        for second in [
            fragment(1, 4, 1, 1, 2, b"cd"),  // overlap plus a hole
            fragment(1, 4, 3, 1, 2, b"d"),   // uncovered byte
            fragment(1, 4, 1, 1, 2, b"cde"), // too many bytes
        ] {
            let lcm = receiver().await;
            assert!(accept(&lcm, &fragment(1, 4, 0, 0, 2, b"ab")).is_none());
            assert!(accept(&lcm, &second).is_none());
            assert!(lcm.reassembly.lock().unwrap().is_empty());
        }
    }

    #[tokio::test]
    async fn incomplete_messages_are_bounded_and_expired_slots_are_reused() {
        let lcm = receiver().await;
        for seq in 0..MAX_INCOMPLETE_MESSAGES as u32 {
            assert!(accept(&lcm, &fragment(seq, 4, 0, 0, 2, b"ab")).is_none());
        }
        assert!(accept(&lcm, &fragment(100, 4, 0, 0, 2, b"ab")).is_none());
        assert_eq!(
            lcm.reassembly.lock().unwrap().len(),
            MAX_INCOMPLETE_MESSAGES
        );
        {
            let mut buffers = lcm.reassembly.lock().unwrap();
            for buffer in buffers.values_mut() {
                buffer.created = Instant::now() - REASSEMBLY_TIMEOUT;
            }
        }
        assert!(accept(&lcm, &fragment(100, 4, 0, 0, 2, b"ab")).is_none());
        assert_eq!(lcm.reassembly.lock().unwrap().len(), 1);
        assert_eq!(
            accept(&lcm, &fragment(100, 4, 2, 1, 2, b"cd"))
                .unwrap()
                .data,
            b"abcd"
        );
    }

    #[tokio::test]
    async fn incomplete_payload_storage_is_bounded_across_senders() {
        let lcm = receiver().await;
        for seq in 0..2 {
            assert!(accept(&lcm, &fragment(seq, MAX_MESSAGE_SIZE as u32, 0, 0, 2, b"a")).is_none());
        }
        assert!(accept(&lcm, &fragment(3, 4, 0, 0, 2, b"ab")).is_none());
        let buffers = lcm.reassembly.lock().unwrap();
        assert_eq!(buffers.len(), 2);
        assert_eq!(
            buffers
                .values()
                .map(|entry| entry.data.len())
                .sum::<usize>(),
            MAX_REASSEMBLY_BYTES
        );
    }

    #[tokio::test]
    async fn recv_continues_after_malformed_datagrams() {
        let lcm = receiver().await;
        let sender = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let destination =
            SocketAddrV4::new(Ipv4Addr::LOCALHOST, lcm.socket.local_addr().unwrap().port());
        for packet in [
            fragment(1, 4, u32::MAX, 1, 2, b"cd"),
            make_small_packet(&[0xff], b"invalid channel"),
            make_small_packet(b"GOOD", b"intact"),
        ] {
            sender.send_to(&packet, destination).await.unwrap();
        }
        let message = tokio::time::timeout(Duration::from_secs(2), lcm.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(message.channel, "GOOD");
        assert_eq!(message.data, b"intact");
    }

    #[test]
    fn invalid_short_channels_are_dropped() {
        for channel in [vec![b'x'; 64], vec![0xff]] {
            assert!(Lcm::decode_small(&make_small_packet(&channel, b"data"))
                .unwrap()
                .is_none());
        }
    }

    const TEST_CHANNEL_LEN: usize = 13; // "/test_channel"
    const FIRST_PAYLOAD: usize = MAX_DATAGRAM_SIZE - FRAGMENT_HEADER_SIZE - TEST_CHANNEL_LEN - 1;
    const SUBSEQUENT_PAYLOAD: usize = MAX_DATAGRAM_SIZE - FRAGMENT_HEADER_SIZE;

    #[test]
    fn fragment_count_fits_in_one() {
        let (_, _, n) = fragment_params(FIRST_PAYLOAD, TEST_CHANNEL_LEN);
        assert_eq!(n, 1);
    }

    #[test]
    fn fragment_count_spills_into_two() {
        let (_, _, n) = fragment_params(FIRST_PAYLOAD + 1, TEST_CHANNEL_LEN);
        assert_eq!(n, 2);
    }

    #[test]
    fn fragment_count_spills_into_three() {
        let (_, _, n) = fragment_params(FIRST_PAYLOAD + SUBSEQUENT_PAYLOAD + 1, TEST_CHANNEL_LEN);
        assert_eq!(n, 3);
    }

    #[test]
    fn fragment_count_one_mb() {
        let (_, _, n) = fragment_params(1024 * 1024, TEST_CHANNEL_LEN);
        assert_eq!(n, 17);
    }

    fn make_small_packet(channel: &[u8], payload: &[u8]) -> Vec<u8> {
        let mut buf = vec![0u8; SHORT_HEADER_SIZE + channel.len() + 1 + payload.len()];
        BigEndian::write_u32(&mut buf[0..4], MAGIC_SHORT);
        BigEndian::write_u32(&mut buf[4..8], 0);
        buf[SHORT_HEADER_SIZE..SHORT_HEADER_SIZE + channel.len()].copy_from_slice(channel);
        buf[SHORT_HEADER_SIZE + channel.len() + 1..].copy_from_slice(payload);
        buf
    }

    #[test]
    fn decode_small_known_good() {
        let buf = make_small_packet(b"CHAN", &[1, 2, 3]);
        let msg = Lcm::decode_small(&buf).unwrap().unwrap();
        assert_eq!(msg.channel, "CHAN");
        assert_eq!(msg.data, [1u8, 2, 3]);
    }

    #[test]
    fn decode_small_empty_payload() {
        let buf = make_small_packet(b"CHAN", &[]);
        let msg = Lcm::decode_small(&buf).unwrap().unwrap();
        assert_eq!(msg.channel, "CHAN");
        assert!(msg.data.is_empty());
    }

    #[test]
    fn decode_small_wrong_magic() {
        let mut buf = make_small_packet(b"CHAN", &[1, 2, 3]);
        BigEndian::write_u32(&mut buf[0..4], 0xDEADBEEF);
        assert!(Lcm::decode_small(&buf).unwrap().is_none());
    }

    #[test]
    fn decode_small_truncated() {
        // Shorter than SHORT_HEADER_SIZE
        let buf = vec![0x4C, 0x43, 0x30, 0x32, 0x00];
        assert!(Lcm::decode_small(&buf).unwrap().is_none());
    }

    #[test]
    fn decode_small_missing_null_terminator() {
        // Valid header but channel bytes have no null terminator
        let mut buf = vec![0u8; SHORT_HEADER_SIZE + 4];
        BigEndian::write_u32(&mut buf[0..4], MAGIC_SHORT);
        BigEndian::write_u32(&mut buf[4..8], 0);
        buf[SHORT_HEADER_SIZE..SHORT_HEADER_SIZE + 4].copy_from_slice(b"CHAN");
        assert!(Lcm::decode_small(&buf).unwrap().is_none());
    }
}
