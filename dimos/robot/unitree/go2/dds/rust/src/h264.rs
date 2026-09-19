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

//! H.264 RTP depacketizer (RFC 6184): the videohub RTP stream reassembled into
//! Annex-B access units. The Go2 encoder only produces single-NAL, STAP-A and FU-A.

use crate::rtp::Packet;

const START_CODE: [u8; 4] = [0, 0, 0, 1];

/// One reassembled frame, Annex-B encoded.
pub struct AccessUnit {
    pub data: Vec<u8>,
    /// RTP timestamp (90 kHz clock) of the frame.
    pub rtp_ts: u32,
    /// Contains an IDR slice with SPS/PPS in front: a decodable entry point.
    pub keyframe: bool,
}

/// Maps RTP 90 kHz timestamps to unix nanoseconds: anchored at the first frame's
/// receive time, then following the encoder clock so inter-frame spacing survives.
#[derive(Default)]
pub struct RtpClock {
    anchor: Option<(u64, u32)>,
    last_ext: i64,
}

impl RtpClock {
    pub fn to_ns(&mut self, rtp_ts: u32, now_ns: u64) -> u64 {
        let (anchor_ns, anchor_ts) = *self.anchor.get_or_insert((now_ns, rtp_ts));
        let prev_ts = anchor_ts.wrapping_add(self.last_ext as u32);
        self.last_ext += rtp_ts.wrapping_sub(prev_ts) as i32 as i64;
        let t = anchor_ns.saturating_add_signed(self.last_ext * 1_000_000_000 / 90_000);
        // The encoder does not hold its nominal rate; re-anchor once a second adrift.
        if t.abs_diff(now_ns) > 1_000_000_000 {
            self.anchor = Some((now_ns, rtp_ts));
            self.last_ext = 0;
            return now_ns;
        }
        t
    }
}

/// Feed packets in arrival order, get an `AccessUnit` whenever the marker bit closes a
/// frame. Nothing is emitted before the first keyframe, and cached SPS/PPS are replayed
/// in front of every IDR, so any output stream starts decodable.
#[derive(Default)]
pub struct Depacketizer {
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
    pub fn push(&mut self, pkt: &Packet<'_>) -> Option<AccessUnit> {
        // A sequence gap corrupts the frame in progress: drop it, resync on a keyframe.
        if let Some(last) = self.last_seq {
            if pkt.sequence_number != last.wrapping_add(1) {
                self.au.clear();
                self.fu.clear();
                self.keyframe = false;
                self.has_params = false;
                self.synced = false;
            }
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

#[cfg(test)]
mod tests {
    use super::*;

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
        let mut d = Depacketizer::default();
        assert!(d.push(&pkt(1, 100, true, P_SLICE)).is_none());
    }

    #[test]
    fn stap_a_keyframe_emits_annex_b() {
        let mut d = Depacketizer::default();
        let au = d
            .push(&pkt(1, 100, true, &stap_a(&[SPS, PPS, IDR])))
            .expect("keyframe AU");
        assert_eq!(au.data, annex_b(&[SPS, PPS, IDR]));
        assert_eq!(au.rtp_ts, 100);
        assert!(d.push(&pkt(2, 200, true, P_SLICE)).is_some());
    }

    #[test]
    fn fu_a_reassembles_and_replays_params() {
        let mut d = Depacketizer::default();
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
        let mut d = Depacketizer::default();
        assert!(d
            .push(&pkt(1, 100, true, &stap_a(&[SPS, PPS, IDR])))
            .is_some());
        assert!(d.push(&pkt(3, 300, true, P_SLICE)).is_none());
        assert!(d.push(&pkt(4, 400, true, IDR)).is_some());
        assert!(d.push(&pkt(5, 500, true, P_SLICE)).is_some());
    }

    #[test]
    fn rtp_clock_tracks_90khz_and_wraps() {
        let mut c = RtpClock::default();
        let t0 = c.to_ns(u32::MAX - 45_000, 1_000_000_000_000);
        assert_eq!(t0, 1_000_000_000_000);
        let t1 = c.to_ns(44_999, 1_000_000_000_000);
        assert_eq!(t1 - t0, 1_000_000_000);
    }
}
