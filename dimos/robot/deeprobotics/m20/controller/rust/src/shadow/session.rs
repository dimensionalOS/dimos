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

//! A recorded session (ecan_capture record.sh: ecan.pcapng, net.pcap, journal.log) as shadow input, in time order:
//! the same engine and log as live, for checking the shadow against the replay on the Mac.

use std::path::Path;

use anyhow::{bail, Context, Result};

use super::journal::{parse_line, Event};
use super::sniff::{Parser, Sample};

pub enum Input {
    /// (time ns, sent by rl_deploy, Ethernet frame)
    Frame(u64, bool, Vec<u8>),
    Sample(Sample),
    Event(f64, Event),
}

impl Input {
    pub fn time(&self) -> f64 {
        match self {
            Input::Frame(t, _, _) => *t as f64 * 1e-9,
            Input::Sample(Sample::Imu(i)) => i.t,
            Input::Sample(Sample::Steer(s)) => s.t,
            Input::Event(t, _) => *t,
        }
    }
}

/// (time ns, direction flags, packet) per enhanced packet block of a pcapng file.
pub fn pcapng(bytes: &[u8]) -> Result<Vec<(u64, u32, &[u8])>> {
    let mut out = Vec::new();
    let mut at = 0usize;
    let mut resol = 1e-6f64;
    let u32_at = |i: usize| u32::from_le_bytes(bytes[i..i + 4].try_into().unwrap());
    while at + 12 <= bytes.len() {
        let kind = u32_at(at);
        let len = u32_at(at + 4) as usize;
        if len < 12 || at + len > bytes.len() {
            break;
        }
        let body = &bytes[at + 8..at + len - 4];
        if kind == 1 {
            // interface description: look for if_tsresol (9)
            let mut o = 8;
            while o + 4 <= body.len() {
                let code = u16::from_le_bytes([body[o], body[o + 1]]);
                let n = u16::from_le_bytes([body[o + 2], body[o + 3]]) as usize;
                if code == 0 {
                    break;
                }
                if code == 9 && n >= 1 {
                    let r = body[o + 4];
                    resol = if r & 0x80 != 0 {
                        2f64.powi(-((r & 0x7f) as i32))
                    } else {
                        10f64.powi(-(r as i32))
                    };
                }
                o += 4 + n.div_ceil(4) * 4;
            }
        } else if kind == 6 && body.len() >= 20 {
            let ts = ((u32::from_le_bytes(body[4..8].try_into().unwrap()) as u64) << 32)
                | u32::from_le_bytes(body[8..12].try_into().unwrap()) as u64;
            let cap = u32::from_le_bytes(body[12..16].try_into().unwrap()) as usize;
            let data = &body[20..20 + cap.min(body.len() - 20)];
            let mut flags = 0u32;
            let mut o = 20 + cap.div_ceil(4) * 4;
            while o + 4 <= body.len() {
                let code = u16::from_le_bytes([body[o], body[o + 1]]);
                let n = u16::from_le_bytes([body[o + 2], body[o + 3]]) as usize;
                if code == 0 {
                    break;
                }
                if code == 2 && n == 4 {
                    flags = u32::from_le_bytes(body[o + 4..o + 8].try_into().unwrap());
                }
                o += 4 + n.div_ceil(4) * 4;
            }
            out.push(((ts as f64 * resol * 1e9).round() as u64, flags, data));
        }
        at += len;
    }
    Ok(out)
}

/// IPv4 packets of a classic pcap (Ethernet or Linux cooked), with their time.
pub fn pcap_ipv4(bytes: &[u8]) -> Result<Vec<(f64, &[u8])>> {
    if bytes.len() < 24 {
        bail!("short pcap");
    }
    let magic = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
    let nano = magic == 0xa1b2_3c4d;
    let link = u32::from_le_bytes(bytes[20..24].try_into().unwrap());
    let off = if link == 113 { 16 } else { 14 };
    let mut out = Vec::new();
    let mut at = 24;
    while at + 16 <= bytes.len() {
        let w = |i: usize| u32::from_le_bytes(bytes[at + i..at + i + 4].try_into().unwrap());
        let (sec, frac, incl) = (w(0), w(4), w(8) as usize);
        let Some(d) = bytes.get(at + 16..at + 16 + incl) else {
            break;
        };
        at += 16 + incl;
        if d.len() > off && d[off - 2..off] == [0x08, 0x00] {
            out.push((
                sec as f64 + frac as f64 * if nano { 1e-9 } else { 1e-6 },
                &d[off..],
            ));
        }
    }
    Ok(out)
}

/// Every input of a session directory, time-sorted.
pub fn load(dir: &Path, parser: &mut Parser) -> Result<Vec<Input>> {
    let mut inputs = Vec::new();
    let ecan = std::fs::read(dir.join("ecan.pcapng")).context("ecan.pcapng")?;
    for (t, flags, data) in pcapng(&ecan)? {
        // epb_flags bits 0-1: 1 inbound (to rl_deploy), 2 outbound (sent by it)
        inputs.push(Input::Frame(t, flags & 3 == 2, data.to_vec()));
    }
    let net = std::fs::read(dir.join("net.pcap")).context("net.pcap")?;
    for (t, ip) in pcap_ipv4(&net)? {
        for s in parser.packet(t, ip) {
            inputs.push(Input::Sample(s));
        }
    }
    let journal = std::fs::read_to_string(dir.join("journal.log")).context("journal.log")?;
    for line in journal.lines() {
        if let Some((t, e)) = parse_line(line) {
            inputs.push(Input::Event(t, e));
        }
    }
    inputs.sort_by(|a, b| a.time().total_cmp(&b.time()));
    Ok(inputs)
}
