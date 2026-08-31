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

//! Pcap replay: parse recorded Mid-360 UDP captures and feed them to the
//! pipeline as a `PacketSource`.
//!
//! Sensor timestamps are replayed unmodified, so downstream output is
//! deterministic and identical at any replay rate. (The virtual device
//! shifts timestamps to look live; a driver replaying for itself must not.)

use crate::pipeline::PacketSource;
use std::io;
use std::time::{Duration, Instant};

/// One data-plane UDP payload from a capture.
#[derive(Debug, Clone, PartialEq)]
pub struct PcapPacket {
    /// Capture timestamp in seconds.
    pub ts: f64,
    pub src_port: u16,
    pub payload: Vec<u8>,
}

/// Parse a classic little-endian pcap (magic d4c3b2a1) into its UDP packets.
pub fn parse_pcap(path: &str) -> io::Result<Vec<PcapPacket>> {
    let buffer = std::fs::read(path)?;
    if buffer.len() < 24 || buffer[0..4] != [0xd4, 0xc3, 0xb2, 0xa1] {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unsupported pcap (need classic little-endian, magic d4c3b2a1) at {path}"),
        ));
    }
    let mut out = Vec::new();
    let mut offset = 24usize;
    while offset + 16 <= buffer.len() {
        let ts_sec = u32::from_le_bytes(buffer[offset..offset + 4].try_into().unwrap());
        let ts_usec = u32::from_le_bytes(buffer[offset + 4..offset + 8].try_into().unwrap());
        let captured_len =
            u32::from_le_bytes(buffer[offset + 8..offset + 12].try_into().unwrap()) as usize;
        offset += 16;
        if offset + captured_len > buffer.len() {
            break;
        }
        let frame = &buffer[offset..offset + captured_len];
        offset += captured_len;
        // Ethernet(14) -> IPv4 -> UDP
        if frame.len() < 14 + 20 + 8 || frame[12] != 0x08 || frame[13] != 0x00 {
            continue;
        }
        let ip_header_len = ((frame[14] & 0x0f) as usize) * 4;
        if frame[14 + 9] != 17 {
            continue; // not UDP
        }
        let udp_offset = 14 + ip_header_len;
        if frame.len() < udp_offset + 8 {
            continue;
        }
        let src_port = u16::from_be_bytes([frame[udp_offset], frame[udp_offset + 1]]);
        let udp_len = u16::from_be_bytes([frame[udp_offset + 4], frame[udp_offset + 5]]) as usize;
        let payload_start = udp_offset + 8;
        let payload_end = (udp_offset + udp_len).min(frame.len());
        if payload_end <= payload_start {
            continue;
        }
        out.push(PcapPacket {
            ts: ts_sec as f64 + ts_usec as f64 / 1e6,
            src_port,
            payload: frame[payload_start..payload_end].to_vec(),
        });
    }
    Ok(out)
}

/// Replays a capture's point/IMU packets through the `PacketSource` seam.
pub struct PcapSource {
    packets: std::vec::IntoIter<PcapPacket>,
    point_port: u16,
    imu_port: u16,
    /// Replay speed relative to capture time. `None` runs flat-out.
    rate: Option<f64>,
    started: Option<(Instant, f64)>,
}

impl PcapSource {
    pub fn new(
        packets: Vec<PcapPacket>,
        point_port: u16,
        imu_port: u16,
        rate: Option<f64>,
    ) -> Self {
        PcapSource {
            packets: packets.into_iter(),
            point_port,
            imu_port,
            rate,
            started: None,
        }
    }

    pub fn from_file(
        path: &str,
        point_port: u16,
        imu_port: u16,
        rate: Option<f64>,
    ) -> io::Result<Self> {
        Ok(Self::new(parse_pcap(path)?, point_port, imu_port, rate))
    }

    fn pace(&mut self, capture_ts: f64) {
        let Some(rate) = self.rate else {
            return;
        };
        let (wall_start, capture_start) = *self.started.get_or_insert((Instant::now(), capture_ts));
        let target = (capture_ts - capture_start) / rate;
        let elapsed = wall_start.elapsed().as_secs_f64();
        if target > elapsed {
            std::thread::sleep(Duration::from_secs_f64(target - elapsed));
        }
    }
}

impl PacketSource for PcapSource {
    fn recv(&mut self, buf: &mut [u8]) -> Option<usize> {
        loop {
            let packet = self.packets.next()?;
            if packet.src_port != self.point_port && packet.src_port != self.imu_port {
                continue;
            }
            self.pace(packet.ts);
            let len = packet.payload.len().min(buf.len());
            buf[..len].copy_from_slice(&packet.payload[..len]);
            return Some(len);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::FrameAssembler;
    use crate::wire::{self, build_points_high, DataPacket, DataType, PointHigh};

    /// Wrap UDP payloads into a classic little-endian pcap byte stream.
    fn synth_pcap(packets: &[(f64, u16, Vec<u8>)]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&[0xd4, 0xc3, 0xb2, 0xa1]); // magic
        out.extend_from_slice(&[0x02, 0x00, 0x04, 0x00]); // version 2.4
        out.extend_from_slice(&[0u8; 12]); // thiszone, sigfigs, snaplen(0)
        out.extend_from_slice(&1u32.to_le_bytes()); // linktype ethernet

        for (ts, src_port, payload) in packets {
            let udp_len = 8 + payload.len();
            let frame_len = 14 + 20 + udp_len;
            out.extend_from_slice(&(*ts as u32).to_le_bytes());
            out.extend_from_slice(&(((ts.fract()) * 1e6) as u32).to_le_bytes());
            out.extend_from_slice(&(frame_len as u32).to_le_bytes()); // captured
            out.extend_from_slice(&(frame_len as u32).to_le_bytes()); // original

            out.extend_from_slice(&[0u8; 12]); // eth dst+src
            out.extend_from_slice(&[0x08, 0x00]); // ipv4
            let mut ip = [0u8; 20];
            ip[0] = 0x45; // version 4, ihl 5
            ip[9] = 17; // udp
            out.extend_from_slice(&ip);
            out.extend_from_slice(&src_port.to_be_bytes());
            out.extend_from_slice(&wire::HOST_POINT_PORT.to_be_bytes());
            out.extend_from_slice(&(udp_len as u16).to_be_bytes());
            out.extend_from_slice(&[0, 0]); // checksum
            out.extend_from_slice(payload);
        }
        out
    }

    fn point_packet(ts_ns: u64, x_mm: i32) -> Vec<u8> {
        let payload = build_points_high(&[PointHigh {
            x_mm,
            y_mm: 0,
            z_mm: 0,
            reflectivity: 255,
            tag: 0,
        }]);
        DataPacket {
            time_interval: 0,
            dot_num: 1,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::CartesianHigh,
            time_type: 0,
            timestamp_ns: ts_ns,
            payload: &payload,
        }
        .build()
    }

    fn write_temp_pcap(name: &str, bytes: &[u8]) -> String {
        let path = std::env::temp_dir().join(format!(
            "dimos_livox_test_{name}_{}.pcap",
            std::process::id()
        ));
        std::fs::write(&path, bytes).unwrap();
        path.to_str().unwrap().to_string()
    }

    #[test]
    fn parses_and_filters_data_ports() {
        let pcap = synth_pcap(&[
            (1.0, wire::LIDAR_POINT_PORT, point_packet(1_000, 100)),
            (1.1, wire::LIDAR_PUSH_MSG_PORT, vec![9, 9, 9]), // status, filtered
            (1.2, wire::LIDAR_POINT_PORT, point_packet(2_000, 200)),
        ]);
        let path = write_temp_pcap("filter", &pcap);
        let parsed = parse_pcap(&path).unwrap();
        assert_eq!(parsed.len(), 3);
        assert_eq!(parsed[1].src_port, wire::LIDAR_PUSH_MSG_PORT);

        let mut source =
            PcapSource::from_file(&path, wire::LIDAR_POINT_PORT, wire::LIDAR_IMU_PORT, None)
                .unwrap();
        let mut buf = [0u8; 4096];
        let mut seen = Vec::new();
        while let Some(len) = source.recv(&mut buf) {
            seen.push(DataPacket::parse(&buf[..len]).unwrap().timestamp_ns);
        }
        assert_eq!(seen, vec![1_000, 2_000]);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn replay_through_assembler_is_deterministic() {
        let pcap = synth_pcap(&[
            (
                0.0,
                wire::LIDAR_POINT_PORT,
                point_packet(1_000_000_000, 500),
            ),
            (
                0.1,
                wire::LIDAR_POINT_PORT,
                point_packet(1_150_000_000, 600),
            ),
        ]);
        let path = write_temp_pcap("assemble", &pcap);
        let mut frames = Vec::new();
        let mut source =
            PcapSource::from_file(&path, wire::LIDAR_POINT_PORT, wire::LIDAR_IMU_PORT, None)
                .unwrap();
        let mut assembler = FrameAssembler::new(10.0);
        let mut buf = [0u8; 4096];
        while let Some(len) = source.recv(&mut buf) {
            let packet = DataPacket::parse(&buf[..len]).unwrap();
            if let Some(frame) = assembler.push(&packet) {
                frames.push(frame);
            }
        }
        frames.extend(assembler.flush());

        // 150 ms apart at 10 Hz: the second packet opens a new frame, and
        // sensor timestamps pass through unshifted.
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].start_ns, 1_000_000_000);
        assert_eq!(frames[1].start_ns, 1_150_000_000);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn rejects_non_pcap_files() {
        let path = write_temp_pcap("garbage", b"not a pcap at all");
        assert!(parse_pcap(&path).is_err());
        std::fs::remove_file(&path).ok();
    }
}
