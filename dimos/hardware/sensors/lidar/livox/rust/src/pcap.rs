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
use std::fs::File;
use std::io::{self, BufReader, Read};
use std::time::{Duration, Instant};

const LINKTYPE_ETHERNET: u32 = 1;

/// One data-plane UDP payload from a capture.
#[derive(Debug, Clone, PartialEq)]
pub struct PcapPacket {
    /// Capture timestamp in seconds.
    pub ts: f64,
    pub src_port: u16,
    pub payload: Vec<u8>,
}

/// Streams UDP packets out of a classic little-endian Ethernet pcap.
struct PcapReader<R> {
    reader: R,
}

impl<R: Read> PcapReader<R> {
    fn new(mut reader: R) -> io::Result<Self> {
        let mut header = [0u8; 24];
        reader
            .read_exact(&mut header)
            .map_err(|_| invalid("shorter than a pcap file header"))?;
        if header[0..4] != [0xd4, 0xc3, 0xb2, 0xa1] {
            return Err(invalid(
                "unsupported pcap (need classic little-endian, magic d4c3b2a1)",
            ));
        }
        let link_type = u32::from_le_bytes(header[20..24].try_into().unwrap());
        if link_type != LINKTYPE_ETHERNET {
            return Err(invalid(&format!(
                "unsupported pcap link-type {link_type}, need {LINKTYPE_ETHERNET} (Ethernet); \
                 capture with tcpdump -i <nic>, not -i any"
            )));
        }
        Ok(PcapReader { reader })
    }

    /// Next UDP packet. A truncated record ends the stream like a clean EOF.
    fn next_packet(&mut self) -> Option<PcapPacket> {
        loop {
            let mut record = [0u8; 16];
            self.reader.read_exact(&mut record).ok()?;
            let ts_sec = u32::from_le_bytes(record[0..4].try_into().unwrap());
            let ts_usec = u32::from_le_bytes(record[4..8].try_into().unwrap());
            let captured_len = u32::from_le_bytes(record[8..12].try_into().unwrap()) as usize;
            let mut frame = vec![0u8; captured_len];
            self.reader.read_exact(&mut frame).ok()?;
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
            let udp_len =
                u16::from_be_bytes([frame[udp_offset + 4], frame[udp_offset + 5]]) as usize;
            let payload_start = udp_offset + 8;
            let payload_end = (udp_offset + udp_len).min(frame.len());
            if payload_end <= payload_start {
                continue;
            }
            frame.copy_within(payload_start..payload_end, 0);
            frame.truncate(payload_end - payload_start);
            return Some(PcapPacket {
                ts: ts_sec as f64 + ts_usec as f64 / 1e6,
                src_port,
                payload: frame,
            });
        }
    }
}

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

/// Parse a classic little-endian pcap (magic d4c3b2a1) into its UDP packets.
pub fn parse_pcap(path: &str) -> io::Result<Vec<PcapPacket>> {
    let mut reader = PcapReader::new(BufReader::new(File::open(path)?))
        .map_err(|err| invalid(&format!("{err} at {path}")))?;
    let mut out = Vec::new();
    while let Some(packet) = reader.next_packet() {
        out.push(packet);
    }
    Ok(out)
}

/// Replays a capture's point/IMU packets through the `PacketSource` seam.
///
/// Streams record by record, so memory stays flat regardless of capture size.
pub struct PcapSource {
    reader: PcapReader<BufReader<File>>,
    point_port: u16,
    imu_port: u16,
    /// Replay speed relative to capture time. `None` runs flat-out.
    rate: Option<f64>,
    started: Option<(Instant, f64)>,
}

impl PcapSource {
    /// Errors on a capture the replay could never use: bad header, wrong
    /// link-type, or no packets on the configured data ports.
    pub fn from_file(
        path: &str,
        point_port: u16,
        imu_port: u16,
        rate: Option<f64>,
    ) -> io::Result<Self> {
        let mut scan = PcapReader::new(BufReader::new(File::open(path)?))
            .map_err(|err| invalid(&format!("{err} at {path}")))?;
        loop {
            match scan.next_packet() {
                Some(packet) if packet.src_port == point_port || packet.src_port == imu_port => {
                    break;
                }
                Some(_) => continue,
                None => {
                    return Err(invalid(&format!(
                        "no Livox data packets on ports {point_port}/{imu_port} in {path}"
                    )));
                }
            }
        }
        Ok(PcapSource {
            reader: PcapReader::new(BufReader::new(File::open(path)?))?,
            point_port,
            imu_port,
            rate,
            started: None,
        })
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
            let packet = self.reader.next_packet()?;
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
    use std::io::Write;

    /// Wrap UDP payloads into a classic little-endian pcap byte stream.
    fn synth_pcap(packets: &[(f64, u16, Vec<u8>)]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&[0xd4, 0xc3, 0xb2, 0xa1]); // magic
        out.extend_from_slice(&[0x02, 0x00, 0x04, 0x00]); // version 2.4
        out.extend_from_slice(&[0u8; 12]); // thiszone, sigfigs, snaplen(0)
        out.extend_from_slice(&LINKTYPE_ETHERNET.to_le_bytes());

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

    fn write_temp_pcap(bytes: &[u8]) -> tempfile::NamedTempFile {
        let mut file = tempfile::NamedTempFile::new().unwrap();
        file.write_all(bytes).unwrap();
        file
    }

    fn path_of(file: &tempfile::NamedTempFile) -> &str {
        file.path().to_str().unwrap()
    }

    #[test]
    fn parses_and_filters_data_ports() {
        let pcap = synth_pcap(&[
            (1.0, wire::LIDAR_POINT_PORT, point_packet(1_000, 100)),
            (1.1, wire::LIDAR_PUSH_MSG_PORT, vec![9, 9, 9]), // status, filtered
            (1.2, wire::LIDAR_POINT_PORT, point_packet(2_000, 200)),
        ]);
        let file = write_temp_pcap(&pcap);
        let parsed = parse_pcap(path_of(&file)).unwrap();
        assert_eq!(parsed.len(), 3);
        assert_eq!(parsed[1].src_port, wire::LIDAR_PUSH_MSG_PORT);

        let mut source = PcapSource::from_file(
            path_of(&file),
            wire::LIDAR_POINT_PORT,
            wire::LIDAR_IMU_PORT,
            None,
        )
        .unwrap();
        let mut buf = [0u8; 4096];
        let mut seen = Vec::new();
        while let Some(len) = source.recv(&mut buf) {
            seen.push(DataPacket::parse(&buf[..len]).unwrap().timestamp_ns);
        }
        assert_eq!(seen, vec![1_000, 2_000]);
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
        let file = write_temp_pcap(&pcap);
        let mut frames = Vec::new();
        let mut source = PcapSource::from_file(
            path_of(&file),
            wire::LIDAR_POINT_PORT,
            wire::LIDAR_IMU_PORT,
            None,
        )
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
    }

    #[test]
    fn rejects_non_pcap_files() {
        let file = write_temp_pcap(b"not a pcap at all");
        assert!(parse_pcap(path_of(&file)).is_err());
    }

    #[test]
    fn rejects_unsupported_link_type() {
        let mut pcap = synth_pcap(&[(1.0, wire::LIDAR_POINT_PORT, point_packet(1_000, 100))]);
        pcap[20..24].copy_from_slice(&113u32.to_le_bytes()); // LINUX_SLL
        let file = write_temp_pcap(&pcap);
        let err = PcapSource::from_file(
            path_of(&file),
            wire::LIDAR_POINT_PORT,
            wire::LIDAR_IMU_PORT,
            None,
        )
        .err()
        .unwrap();
        assert!(err.to_string().contains("link-type 113"), "{err}");
    }

    #[test]
    fn rejects_capture_without_data_packets() {
        let pcap = synth_pcap(&[(1.0, wire::LIDAR_PUSH_MSG_PORT, vec![9, 9, 9])]);
        let file = write_temp_pcap(&pcap);
        let err = PcapSource::from_file(
            path_of(&file),
            wire::LIDAR_POINT_PORT,
            wire::LIDAR_IMU_PORT,
            None,
        )
        .err()
        .unwrap();
        assert!(err.to_string().contains("no Livox data packets"), "{err}");
    }

    #[test]
    fn pace_respects_rate() {
        let pcap = synth_pcap(&[
            (1.00, wire::LIDAR_POINT_PORT, point_packet(1_000, 100)),
            (1.05, wire::LIDAR_POINT_PORT, point_packet(2_000, 200)),
        ]);
        let file = write_temp_pcap(&pcap);
        let mut buf = [0u8; 4096];

        // 50 ms of capture at half speed takes at least 100 ms of wall time.
        let mut paced = PcapSource::from_file(
            path_of(&file),
            wire::LIDAR_POINT_PORT,
            wire::LIDAR_IMU_PORT,
            Some(0.5),
        )
        .unwrap();
        let start = Instant::now();
        while paced.recv(&mut buf).is_some() {}
        assert!(start.elapsed() >= Duration::from_millis(90));

        let mut flat_out = PcapSource::from_file(
            path_of(&file),
            wire::LIDAR_POINT_PORT,
            wire::LIDAR_IMU_PORT,
            None,
        )
        .unwrap();
        let start = Instant::now();
        while flat_out.recv(&mut buf).is_some() {}
        assert!(start.elapsed() < Duration::from_millis(80));
    }
}
