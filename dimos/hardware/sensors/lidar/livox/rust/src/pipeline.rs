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

//! Pure packet-to-frame pipeline, shared by the live and replay paths.
//!
//! Sources produce raw data-plane packets; the assembler cuts them into frames
//! on packet time, never wall clock, so replay is deterministic at any speed.

use crate::wire::{DataPacket, DataType};

/// Accel conversion from g on the wire to m/s^2 on the output.
pub const GRAVITY_MS2: f64 = 9.80665;

/// Produces raw data-plane packets (point and IMU ports only).
pub trait PacketSource {
    /// Receive the next packet into `buf`, returning its length.
    /// `None` means end of stream or shutdown.
    fn recv(&mut self, buf: &mut [u8]) -> Option<usize>;
}

/// A source over an in-memory packet list, for tests and fixtures.
pub struct VecSource {
    packets: std::vec::IntoIter<Vec<u8>>,
}

impl VecSource {
    pub fn new(packets: Vec<Vec<u8>>) -> Self {
        VecSource {
            packets: packets.into_iter(),
        }
    }
}

impl PacketSource for VecSource {
    fn recv(&mut self, buf: &mut [u8]) -> Option<usize> {
        let packet = self.packets.next()?;
        let len = packet.len().min(buf.len());
        buf[..len].copy_from_slice(&packet[..len]);
        Some(len)
    }
}

/// One assembled lidar point in the sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawPoint {
    pub xyz_m: [f32; 3],
    /// Reflectivity scaled to [0, 1].
    pub intensity: f32,
    /// Nanoseconds since the frame start, saturated at u32::MAX.
    pub offset_ns: u32,
    pub tag: u8,
}

/// One assembled frame. `start_ns` is the timestamp of the frame's first
/// packet and the timebase every `offset_ns` is relative to.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub start_ns: u64,
    pub points: Vec<RawPoint>,
}

/// One IMU sample with units converted for publishing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuRecord {
    pub ts_ns: u64,
    pub gyro_rads: [f64; 3],
    pub acc_ms2: [f64; 3],
}

/// Convert an IMU packet's samples to publishable records.
pub fn imu_records<'a>(packet: &'a DataPacket<'a>) -> impl Iterator<Item = ImuRecord> + 'a {
    let ts_ns = packet.timestamp_ns;
    packet.imu_samples().map(move |sample| ImuRecord {
        ts_ns,
        gyro_rads: sample.gyro.map(f64::from),
        acc_ms2: sample.acc_g.map(|a| f64::from(a) * GRAVITY_MS2),
    })
}

/// Accumulates point packets into frames cut on packet time.
pub struct FrameAssembler {
    frame_interval_ns: u64,
    frame_start_ns: Option<u64>,
    points: Vec<RawPoint>,
}

impl FrameAssembler {
    pub fn new(frequency_hz: f64) -> Self {
        assert!(frequency_hz > 0.0, "frame frequency must be positive");
        FrameAssembler {
            frame_interval_ns: (1e9 / frequency_hz) as u64,
            frame_start_ns: None,
            points: Vec::new(),
        }
    }

    /// Feed one point packet. Returns the completed frame when this packet's
    /// timestamp crosses the frame boundary; the packet's own points open the
    /// next frame.
    pub fn push(&mut self, packet: &DataPacket<'_>) -> Option<Frame> {
        if !matches!(
            packet.data_type,
            DataType::CartesianHigh | DataType::CartesianLow
        ) {
            return None;
        }

        let ts_ns = packet.timestamp_ns;
        let completed = match self.frame_start_ns {
            Some(start) if ts_ns.saturating_sub(start) >= self.frame_interval_ns => {
                let frame = self.take_frame(start);
                self.frame_start_ns = Some(ts_ns);
                frame
            }
            Some(_) => None,
            None => {
                self.frame_start_ns = Some(ts_ns);
                None
            }
        };

        let frame_start = self.frame_start_ns.expect("set above");
        // Offset 0 = "at the frame stamp": clamp rather than wrap when a UDP
        // packet arrives out of order with a stamp older than the frame start.
        let packet_offset_ns = ts_ns.saturating_sub(frame_start);
        let point_interval_ns = packet.point_interval_ns();

        let offset = |i: usize| {
            let ns = packet_offset_ns + i as u64 * point_interval_ns;
            ns.min(u64::from(u32::MAX)) as u32
        };
        match packet.data_type {
            DataType::CartesianHigh => {
                for (i, p) in packet.points_high().enumerate() {
                    self.points.push(RawPoint {
                        xyz_m: [
                            p.x_mm as f32 / 1000.0,
                            p.y_mm as f32 / 1000.0,
                            p.z_mm as f32 / 1000.0,
                        ],
                        intensity: f32::from(p.reflectivity) / 255.0,
                        offset_ns: offset(i),
                        tag: p.tag,
                    });
                }
            }
            DataType::CartesianLow => {
                for (i, p) in packet.points_low().enumerate() {
                    self.points.push(RawPoint {
                        xyz_m: [
                            f32::from(p.x_cm) / 100.0,
                            f32::from(p.y_cm) / 100.0,
                            f32::from(p.z_cm) / 100.0,
                        ],
                        intensity: f32::from(p.reflectivity) / 255.0,
                        offset_ns: offset(i),
                        tag: p.tag,
                    });
                }
            }
            DataType::Imu => unreachable!("filtered above"),
        }

        completed
    }

    /// Emit whatever is accumulated, e.g. at end of stream.
    pub fn flush(&mut self) -> Option<Frame> {
        let start = self.frame_start_ns.take()?;
        self.take_frame(start)
    }

    fn take_frame(&mut self, start_ns: u64) -> Option<Frame> {
        if self.points.is_empty() {
            return None;
        }
        Some(Frame {
            start_ns,
            points: std::mem::take(&mut self.points),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{build_imu_samples, build_points_high, ImuSample, PointHigh};

    fn point_packet(ts_ns: u64, time_interval: u16, points: &[PointHigh]) -> Vec<u8> {
        let payload = build_points_high(points);
        DataPacket {
            time_interval,
            dot_num: points.len() as u16,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::CartesianHigh,
            time_type: 0,
            timestamp_ns: ts_ns,
            payload: &payload,
        }
        .build()
    }

    fn simple_point(x_mm: i32) -> PointHigh {
        PointHigh {
            x_mm,
            y_mm: 0,
            z_mm: 0,
            reflectivity: 255,
            tag: 0,
        }
    }

    #[test]
    fn frames_cut_on_packet_time() {
        let mut assembler = FrameAssembler::new(10.0); // 100 ms frames
        let base = 1_000_000_000u64;

        // Two packets inside the frame, third crosses the boundary.
        let packets = [
            point_packet(base, 1000, &[simple_point(1000), simple_point(2000)]),
            point_packet(base + 50_000_000, 1000, &[simple_point(3000)]),
            point_packet(base + 100_000_000, 1000, &[simple_point(4000)]),
        ];
        let mut frames = Vec::new();
        for bytes in &packets {
            let packet = DataPacket::parse(bytes).unwrap();
            if let Some(frame) = assembler.push(&packet) {
                frames.push(frame);
            }
        }

        assert_eq!(frames.len(), 1);
        let frame = &frames[0];
        assert_eq!(frame.start_ns, base);
        assert_eq!(frame.points.len(), 3);
        assert_eq!(frame.points[0].xyz_m, [1.0, 0.0, 0.0]);
        assert_eq!(frame.points[0].offset_ns, 0);
        // 100 us packet span over 2 points -> 50 us point spacing.
        assert_eq!(frame.points[1].offset_ns, 50_000);
        // Second packet: 50 ms after the frame start.
        assert_eq!(frame.points[2].offset_ns, 50_000_000);

        // The boundary packet opened the next frame.
        let tail = assembler.flush().unwrap();
        assert_eq!(tail.start_ns, base + 100_000_000);
        assert_eq!(tail.points.len(), 1);
        assert_eq!(tail.points[0].offset_ns, 0);
        assert!(assembler.flush().is_none());
    }

    #[test]
    fn out_of_order_packet_clamps_to_frame_start() {
        let mut assembler = FrameAssembler::new(10.0);
        let base = 5_000_000_000u64;
        let first = point_packet(base, 0, &[simple_point(1000)]);
        let stale = point_packet(base - 10_000, 0, &[simple_point(2000)]);
        assembler.push(&DataPacket::parse(&first).unwrap());
        assembler.push(&DataPacket::parse(&stale).unwrap());
        let frame = assembler.flush().unwrap();
        assert_eq!(frame.points[1].offset_ns, 0);
    }

    #[test]
    fn offsets_saturate_instead_of_wrapping() {
        let mut assembler = FrameAssembler::new(0.1); // 10 s frames
        let base = 0u64;
        let first = point_packet(base, 0, &[simple_point(1)]);
        // ~5 s into the frame, beyond the ~4.29 s u32 range.
        let late = point_packet(base + 5_000_000_000, 0, &[simple_point(2)]);
        assembler.push(&DataPacket::parse(&first).unwrap());
        assembler.push(&DataPacket::parse(&late).unwrap());
        let frame = assembler.flush().unwrap();
        assert_eq!(frame.points[1].offset_ns, u32::MAX);
    }

    #[test]
    fn imu_units_convert_to_ms2() {
        let payload = build_imu_samples(&[ImuSample {
            gyro: [0.1, -0.2, 0.3],
            acc_g: [0.0, 0.0, 1.0],
        }]);
        let bytes = DataPacket {
            time_interval: 0,
            dot_num: 1,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::Imu,
            time_type: 0,
            timestamp_ns: 42,
            payload: &payload,
        }
        .build();
        let packet = DataPacket::parse(&bytes).unwrap();
        let records: Vec<ImuRecord> = imu_records(&packet).collect();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].ts_ns, 42);
        assert!((records[0].acc_ms2[2] - GRAVITY_MS2).abs() < 1e-9);
        assert!((records[0].gyro_rads[0] - 0.1).abs() < 1e-7);
    }

    #[test]
    fn imu_packets_do_not_disturb_frames() {
        let mut assembler = FrameAssembler::new(10.0);
        let imu_payload = build_imu_samples(&[ImuSample {
            gyro: [0.0; 3],
            acc_g: [0.0, 0.0, 1.0],
        }]);
        let imu_bytes = DataPacket {
            time_interval: 0,
            dot_num: 1,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::Imu,
            time_type: 0,
            timestamp_ns: 7,
            payload: &imu_payload,
        }
        .build();
        assert!(assembler
            .push(&DataPacket::parse(&imu_bytes).unwrap())
            .is_none());
        assert!(assembler.flush().is_none());
    }

    #[test]
    fn vec_source_replays_in_order() {
        let mut source = VecSource::new(vec![vec![1, 2, 3], vec![4]]);
        let mut buf = [0u8; 16];
        assert_eq!(source.recv(&mut buf), Some(3));
        assert_eq!(&buf[..3], &[1, 2, 3]);
        assert_eq!(source.recv(&mut buf), Some(1));
        assert_eq!(source.recv(&mut buf), None);
    }
}
