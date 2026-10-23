// Copyright 2026 Dimensional Inc.
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

//! H.264 RTP depacketizer (RFC 6184): the WebRTC camera track reassembled into Annex-B access
//! units, each handed on at its marker packet.

pub struct Packet<'a> {
    pub sequence_number: u16,
    pub timestamp: u32,
    pub marker: bool,
    pub payload: &'a [u8],
}

const START_CODE: [u8; 4] = [0, 0, 0, 1];

/// One reassembled frame, Annex-B encoded.
pub struct AccessUnit {
    pub data: Vec<u8>,
    /// RTP timestamp of the frame.
    pub rtp_ts: u32,
    /// Contains an IDR slice with SPS/PPS in front: a decodable entry point.
    pub keyframe: bool,
}

/// Packets in, whole frames out; a sequence gap drops the frame and waits for a keyframe.
#[derive(Default)]
pub struct Depacketizer {
    /// a frame in progress past this is a markerless stream, not video: dropped, resync
    max_frame_bytes: usize,
    au: Vec<u8>,
    au_rtp_ts: u32,
    keyframe: bool,
    has_params: bool,
    fu: Vec<u8>,
    sps: Option<Vec<u8>>,
    pps: Option<Vec<u8>>,
    synced: bool,
    last_seq: Option<u16>,
}

impl Depacketizer {
    pub fn new(max_frame_bytes: usize) -> Self {
        Self {
            max_frame_bytes,
            ..Self::default()
        }
    }

    /// Whether frames are flowing: false from the start or a gap until the next keyframe.
    pub fn synced(&self) -> bool {
        self.synced
    }

    pub fn push(&mut self, pkt: &Packet<'_>) -> Option<AccessUnit> {
        // A sequence gap corrupts the frame in progress: drop it, resync on a keyframe.
        let gap = self
            .last_seq
            .is_some_and(|last| pkt.sequence_number != last.wrapping_add(1));
        if gap || self.au.len() + self.fu.len() > self.max_frame_bytes {
            self.au.clear();
            self.fu.clear();
            self.keyframe = false;
            self.has_params = false;
            self.synced = false;
        }
        self.last_seq = Some(pkt.sequence_number);

        let payload = pkt.payload;
        if payload.is_empty() {
            return None;
        }
        self.au_rtp_ts = pkt.timestamp;

        match payload[0] & 0x1F {
            1..=23 => self.push_nal(payload),
            24 => {
                // STAP-A: [indicator] ( [u16 size][nal] )*
                let mut p = &payload[1..];
                while p.len() >= 2 {
                    let size = u16::from_be_bytes([p[0], p[1]]) as usize;
                    if p.len() < 2 + size {
                        break;
                    }
                    self.push_nal(&p[2..2 + size]);
                    p = &p[2 + size..];
                }
            }
            28 => {
                // FU-A: [indicator][fu header][fragment]
                if payload.len() < 3 {
                    return None;
                }
                let (start, end) = (payload[1] & 0x80 != 0, payload[1] & 0x40 != 0);
                if start {
                    self.fu.clear();
                    self.fu.push((payload[0] & 0xE0) | (payload[1] & 0x1F));
                }
                if start || !self.fu.is_empty() {
                    self.fu.extend_from_slice(&payload[2..]);
                    if end {
                        let nal = std::mem::take(&mut self.fu);
                        self.push_nal(&nal);
                    }
                }
            }
            _ => {}
        }

        if pkt.marker {
            return self.finish_au();
        }
        None
    }

    fn push_nal(&mut self, nal: &[u8]) {
        if nal.is_empty() {
            return;
        }
        match nal[0] & 0x1F {
            7 => {
                self.sps = Some(nal.to_vec());
                self.has_params = true;
            }
            8 => self.pps = Some(nal.to_vec()),
            5 => self.keyframe = true,
            _ => {}
        }
        self.au.extend_from_slice(&START_CODE);
        self.au.extend_from_slice(nal);
    }

    fn finish_au(&mut self) -> Option<AccessUnit> {
        let mut data = std::mem::take(&mut self.au);
        let keyframe = self.keyframe;
        let has_params = self.has_params;
        self.keyframe = false;
        self.has_params = false;
        if data.is_empty() {
            return None;
        }
        if keyframe && !has_params {
            let (Some(sps), Some(pps)) = (&self.sps, &self.pps) else {
                return None;
            };
            let mut with_params = Vec::with_capacity(sps.len() + pps.len() + 8 + data.len());
            with_params.extend_from_slice(&START_CODE);
            with_params.extend_from_slice(sps);
            with_params.extend_from_slice(&START_CODE);
            with_params.extend_from_slice(pps);
            with_params.extend_from_slice(&data);
            data = with_params;
        }
        if !self.synced {
            if !(keyframe && self.sps.is_some() && self.pps.is_some()) {
                return None;
            }
            self.synced = true;
        }
        Some(AccessUnit {
            data,
            rtp_ts: self.au_rtp_ts,
            keyframe,
        })
    }
}

/// Puts RTP packets back in sequence order: one that comes early waits for the ones before it, up to `wait_ns`
/// (long enough for a NACK's retransmit, however many packets a keyframe burst puts in between); past that the
/// missing ones count as lost.
pub struct Reorder<T> {
    wait_ns: u64,
    next: Option<i64>,
    held: std::collections::BTreeMap<i64, T>,
    /// when the packet at `next` started being waited for
    gap_since: Option<u64>,
}

impl<T> Reorder<T> {
    pub fn new(wait_ns: u64) -> Self {
        Self {
            wait_ns,
            next: None,
            held: Default::default(),
            gap_since: None,
        }
    }

    /// The packets now ready, in order; a duplicate or one that comes after its turn is dropped.
    pub fn push(&mut self, seq: u16, item: T, arrived_ns: u64) -> Vec<T> {
        let next = *self.next.get_or_insert(seq as i64);
        let key = next + seq.wrapping_sub(next as u16) as i16 as i64;
        if key < next {
            return Vec::new();
        }
        self.held.insert(key, item);
        let mut ready = Vec::new();
        let mut next = next;
        loop {
            if let Some(item) = self.held.remove(&next) {
                ready.push(item);
                next += 1;
                self.gap_since = None;
            } else if self.held.is_empty() {
                break;
            } else if arrived_ns.saturating_sub(*self.gap_since.get_or_insert(arrived_ns))
                > self.wait_ns
            {
                next = *self.held.keys().next().expect("held isn't empty");
                self.gap_since = None;
            } else {
                break;
            }
        }
        self.next = Some(next);
        ready
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX_AU_BYTES: usize = 4 << 20;

    #[test]
    fn reorder_puts_packets_back_in_sequence() {
        let mut reorder = Reorder::new(100);
        assert_eq!(reorder.push(65534, 'a', 0), vec!['a']);
        assert_eq!(reorder.push(0, 'c', 1), Vec::<char>::new());
        assert_eq!(reorder.push(65535, 'b', 2), vec!['b', 'c']);
        // a duplicate of one already handed out
        assert_eq!(reorder.push(65535, 'b', 3), Vec::<char>::new());
        // 1 never comes: however many packets wait, it counts as lost only once it's 100 ns late
        for seq in 2..500u16 {
            assert_eq!(reorder.push(seq, 'e', 10), Vec::<char>::new());
        }
        assert_eq!(reorder.push(500, 'f', 111).len(), 499);
        // a late retransmit within the wait still lands in order
        assert_eq!(reorder.push(502, 'h', 120), Vec::<char>::new());
        assert_eq!(reorder.push(501, 'g', 150), vec!['g', 'h']);
    }

    fn pkt(seq: u16, ts: u32, marker: bool, payload: &[u8]) -> Packet<'_> {
        Packet {
            sequence_number: seq,
            timestamp: ts,
            marker,
            payload,
        }
    }

    const SPS: &[u8] = &[0x67, 0xAA, 0xBB];
    const PPS: &[u8] = &[0x68, 0xCC];
    const IDR: &[u8] = &[0x65, 1, 2, 3];
    const P_SLICE: &[u8] = &[0x41, 9, 9];

    fn stap_a(nals: &[&[u8]]) -> Vec<u8> {
        let mut v = vec![24u8];
        for n in nals {
            v.extend_from_slice(&(n.len() as u16).to_be_bytes());
            v.extend_from_slice(n);
        }
        v
    }

    fn annex_b(nals: &[&[u8]]) -> Vec<u8> {
        let mut want = Vec::new();
        for n in nals {
            want.extend_from_slice(&START_CODE);
            want.extend_from_slice(n);
        }
        want
    }

    #[test]
    fn no_output_before_first_keyframe() {
        let mut d = Depacketizer::new(MAX_AU_BYTES);
        assert!(d.push(&pkt(1, 100, true, P_SLICE)).is_none());
    }

    #[test]
    fn stap_a_keyframe_emits_annex_b() {
        let mut d = Depacketizer::new(MAX_AU_BYTES);
        let au = d
            .push(&pkt(1, 100, true, &stap_a(&[SPS, PPS, IDR])))
            .expect("keyframe AU");
        assert_eq!(au.data, annex_b(&[SPS, PPS, IDR]));
        assert_eq!(au.rtp_ts, 100);
        assert!(d.push(&pkt(2, 200, true, P_SLICE)).is_some());
    }

    #[test]
    fn fu_a_reassembles_and_replays_params() {
        let mut d = Depacketizer::new(MAX_AU_BYTES);
        assert!(d.push(&pkt(1, 100, false, &stap_a(&[SPS, PPS]))).is_none());
        let ind = (IDR[0] & 0xE0) | 28;
        let fu_start = [ind, 0x80 | (IDR[0] & 0x1F), IDR[1], IDR[2]];
        let fu_end = [ind, 0x40 | (IDR[0] & 0x1F), IDR[3]];
        assert!(d.push(&pkt(2, 100, false, &fu_start)).is_none());
        let au = d.push(&pkt(3, 100, true, &fu_end)).expect("reassembled AU");
        assert_eq!(au.data, annex_b(&[SPS, PPS, IDR]));
        // The next IDR arrives without in-band params: cached SPS/PPS replayed.
        let au2 = d.push(&pkt(4, 200, true, IDR)).expect("AU");
        assert_eq!(au2.data, annex_b(&[SPS, PPS, IDR]));
    }

    #[test]
    fn sequence_gap_resyncs_on_keyframe() {
        let mut d = Depacketizer::new(MAX_AU_BYTES);
        assert!(d
            .push(&pkt(1, 100, true, &stap_a(&[SPS, PPS, IDR])))
            .is_some());
        assert!(d.push(&pkt(3, 300, true, P_SLICE)).is_none());
        assert!(d.push(&pkt(4, 400, true, IDR)).is_some());
        assert!(d.push(&pkt(5, 500, true, P_SLICE)).is_some());
    }

    #[test]
    fn markerless_stream_stays_bounded() {
        let mut d = Depacketizer::new(MAX_AU_BYTES);
        let slice = [0x41; 1200];
        for seq in 0..8192u16 {
            assert!(d.push(&pkt(seq, seq as u32, false, &slice)).is_none());
        }
        assert!(d.au.len() <= MAX_AU_BYTES + 1300);
    }
}
