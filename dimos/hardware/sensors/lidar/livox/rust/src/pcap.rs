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
//! deterministic and identical at any replay rate.

use crate::pipeline::PacketSource;
use etherparse::{SlicedPacket, TransportSlice};
use pcap_parser::traits::PcapReaderIterator;
use pcap_parser::{LegacyPcapReader, Linktype, PcapBlockOwned, PcapError};
use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};
use std::time::{Duration, Instant};

/// Comfortably above any Ethernet frame a capture can hold.
const BUFFER_CAPACITY: usize = 65536;

/// One data-plane UDP payload from a capture.
#[derive(Debug, Clone, PartialEq)]
pub struct PcapPacket {
    /// Capture timestamp in seconds.
    pub ts: f64,
    pub src_port: u16,
    pub payload: Vec<u8>,
}

/// Decoding state that survives across blocks.
#[derive(Default)]
struct ReaderState {
    nanos: bool,
    failure: Option<String>,
    stalled_at: usize,
}

impl ReaderState {
    fn handle(&mut self, block: PcapBlockOwned<'_>) -> Option<PcapPacket> {
        match block {
            PcapBlockOwned::LegacyHeader(header) => {
                if header.network != Linktype::ETHERNET {
                    self.failure = Some(format!(
                        "unsupported pcap link-type {}, need 1 (Ethernet); \
                         capture with tcpdump -i <nic>, not -i any",
                        header.network.0
                    ));
                }
                self.nanos = header.is_nanosecond_precision();
                None
            }
            PcapBlockOwned::Legacy(record) => {
                let frac = if self.nanos { 1e-9 } else { 1e-6 };
                decode_udp(
                    record.ts_sec as f64 + record.ts_usec as f64 * frac,
                    record.data,
                )
            }
            PcapBlockOwned::NG(_) => None,
        }
    }
}

fn decode_udp(ts: f64, frame: &[u8]) -> Option<PcapPacket> {
    let sliced = SlicedPacket::from_ethernet(frame).ok()?;
    match sliced.transport {
        Some(TransportSlice::Udp(udp)) => Some(PcapPacket {
            ts,
            src_port: udp.source_port(),
            payload: udp.payload().to_vec(),
        }),
        _ => None,
    }
}

/// Streams UDP packets out of a classic pcap capture.
struct PcapReader {
    reader: LegacyPcapReader<File>,
    state: ReaderState,
}

impl PcapReader {
    fn new(mut file: File) -> io::Result<Self> {
        let mut magic = [0u8; 4];
        let peeked = file.read(&mut magic)?;
        file.seek(SeekFrom::Start(0))?;
        if peeked == 4 && magic == [0x0A, 0x0D, 0x0D, 0x0A] {
            return Err(invalid(
                "pcapng capture, need classic pcap; convert with editcap -F pcap",
            ));
        }
        let reader = LegacyPcapReader::new(BUFFER_CAPACITY, file)
            .map_err(|_| invalid("not a pcap capture"))?;
        Ok(PcapReader {
            reader,
            state: ReaderState::default(),
        })
    }

    /// Next UDP packet. A truncated record ends the stream like a clean EOF.
    /// Anything else that ends it early is recorded in `state.failure`.
    fn next_packet(&mut self) -> Option<PcapPacket> {
        loop {
            match self.reader.next() {
                Ok((offset, block)) => {
                    let packet = self.state.handle(block);
                    self.reader.consume(offset);
                    if let Some(packet) = packet {
                        return Some(packet);
                    }
                    if self.state.failure.is_some() {
                        return None;
                    }
                }
                // A truncated final record reads as UnexpectedEof.
                Err(PcapError::Eof) | Err(PcapError::UnexpectedEof) => return None,
                Err(PcapError::Incomplete(_)) => {
                    if self.reader.consumed() == self.state.stalled_at {
                        self.state.failure = Some("capture record larger than buffer".to_string());
                        return None;
                    }
                    self.state.stalled_at = self.reader.consumed();
                    if self.reader.refill().is_err() {
                        self.state.failure = Some("read failed mid-capture".to_string());
                        return None;
                    }
                }
                Err(err) => {
                    self.state.failure = Some(format!("bad capture record: {err:?}"));
                    return None;
                }
            }
        }
    }
}

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

/// Parse a classic pcap capture into its UDP packets.
pub fn parse_pcap(path: &str) -> io::Result<Vec<PcapPacket>> {
    let mut reader =
        PcapReader::new(File::open(path)?).map_err(|err| invalid(&format!("{err} at {path}")))?;
    let mut out = Vec::new();
    while let Some(packet) = reader.next_packet() {
        out.push(packet);
    }
    match reader.state.failure {
        Some(failure) => Err(invalid(&format!("{failure} at {path}"))),
        None => Ok(out),
    }
}

/// Replays a capture's point/IMU packets through the `PacketSource` seam.
///
/// Streams record by record, so memory stays flat regardless of capture size.
pub struct PcapSource {
    reader: PcapReader,
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
        let mut scan = PcapReader::new(File::open(path)?)
            .map_err(|err| invalid(&format!("{err} at {path}")))?;
        loop {
            match scan.next_packet() {
                Some(packet) if packet.src_port == point_port || packet.src_port == imu_port => {
                    break;
                }
                Some(_) => continue,
                None => {
                    return Err(match scan.state.failure {
                        Some(failure) => invalid(&format!("{failure} at {path}")),
                        None => invalid(&format!(
                            "no Livox data packets on ports {point_port}/{imu_port} in {path}"
                        )),
                    });
                }
            }
        }
        Ok(PcapSource {
            reader: PcapReader::new(File::open(path)?)?,
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

    fn failure(&self) -> Option<String> {
        self.reader.state.failure.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{self, build_points_high, DataPacket, DataType, PointHigh};
    use std::io::Write;

    const LINKTYPE_ETHERNET: u32 = 1;

    fn eth_frame(src_port: u16, payload: &[u8]) -> Vec<u8> {
        let udp_len = 8 + payload.len();
        let mut frame = Vec::new();
        frame.extend_from_slice(&[0u8; 12]); // eth dst+src
        frame.extend_from_slice(&[0x08, 0x00]); // ipv4
        let mut ip = [0u8; 20];
        ip[0] = 0x45; // version 4, ihl 5
        ip[2..4].copy_from_slice(&((20 + udp_len) as u16).to_be_bytes());
        ip[8] = 64; // ttl
        ip[9] = 17; // udp
        frame.extend_from_slice(&ip);
        frame.extend_from_slice(&src_port.to_be_bytes());
        frame.extend_from_slice(&wire::HOST_POINT_PORT.to_be_bytes());
        frame.extend_from_slice(&(udp_len as u16).to_be_bytes());
        frame.extend_from_slice(&[0, 0]); // checksum
        frame.extend_from_slice(payload);
        frame
    }

    fn pcap_record(ts: f64, frame: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&(ts as u32).to_le_bytes());
        out.extend_from_slice(&((ts.fract() * 1e6) as u32).to_le_bytes());
        out.extend_from_slice(&(frame.len() as u32).to_le_bytes()); // captured
        out.extend_from_slice(&(frame.len() as u32).to_le_bytes()); // original
        out.extend_from_slice(frame);
        out
    }

    /// Wrap UDP payloads into a classic little-endian pcap byte stream.
    fn synth_pcap(packets: &[(f64, u16, Vec<u8>)]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&[0xd4, 0xc3, 0xb2, 0xa1]); // magic
        out.extend_from_slice(&[0x02, 0x00, 0x04, 0x00]); // version 2.4
        out.extend_from_slice(&[0u8; 12]); // thiszone, sigfigs, snaplen(0)
        out.extend_from_slice(&LINKTYPE_ETHERNET.to_le_bytes());

        for (ts, src_port, payload) in packets {
            out.extend_from_slice(&pcap_record(*ts, &eth_frame(*src_port, payload)));
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
        assert_eq!(source.failure(), None);
    }

    #[test]
    fn rejects_unusable_captures() {
        let assert_err = |bytes: &[u8], needle: &str| {
            let file = write_temp_pcap(bytes);
            let err = PcapSource::from_file(
                path_of(&file),
                wire::LIDAR_POINT_PORT,
                wire::LIDAR_IMU_PORT,
                None,
            )
            .err()
            .unwrap();
            assert!(err.to_string().contains(needle), "{err}");
        };

        assert_err(b"not a pcap at all", "not a pcap");
        assert_err(&[0x0A, 0x0D, 0x0D, 0x0A], "editcap");
        let mut sll = synth_pcap(&[(1.0, wire::LIDAR_POINT_PORT, point_packet(1_000, 100))]);
        sll[20..24].copy_from_slice(&113u32.to_le_bytes()); // LINUX_SLL
        assert_err(&sll, "link-type 113");
        let no_data = synth_pcap(&[(1.0, wire::LIDAR_PUSH_MSG_PORT, vec![9, 9, 9])]);
        assert_err(&no_data, "no Livox data packets");
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
    }
}
