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
//! Sources produce raw data-plane packets. The assembler cuts them into frames
//! on packet time, never wall clock, so replay is deterministic at any speed.
//! Published stamps are the device clock re-based onto the host clock through
//! each packet's arrival time, so a sensor with no time sync still lands on
//! the same axis as everything else in the graph.

use crate::wire::{DataPacket, DataType};
use std::collections::VecDeque;
use std::time::{SystemTime, UNIX_EPOCH};

/// Accel conversion from g on the wire to m/s^2 on the output.
pub const GRAVITY_MS2: f64 = 9.80665;

/// Seconds since the Unix epoch, the host clock every stamp is anchored to.
pub fn unix_now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// One received packet: its length in the caller's buffer and when the host
/// side saw it. Live sources stamp at socket receipt, replay uses the
/// capture record's timestamp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Received {
    pub len: usize,
    pub arrival_secs: f64,
}

/// Produces raw data-plane packets (point and IMU ports only).
pub trait PacketSource {
    /// Receive the next packet into `buf`.
    /// `None` means end of stream or shutdown.
    fn recv(&mut self, buf: &mut [u8]) -> Option<Received>;

    /// Why the stream ended, if it died rather than completing or stopping.
    fn failure(&self) -> Option<String> {
        None
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

const NANOS_PER_SEC: i64 = 1_000_000_000;
/// How long the host-clock estimate remembers arrivals. Long enough to ride
/// out a burst of late deliveries, short enough to follow device drift.
const HOST_CLOCK_WINDOW_SECS: i64 = 30;
/// An offset this far from the estimate is a clock step on one side or the
/// other, not jitter, so the offset history no longer applies.
const HOST_CLOCK_RESET_NS: i64 = NANOS_PER_SEC;

/// Maps the device clock onto the host clock.
///
/// Delivery latency only ever makes a packet arrive later, so the smallest
/// `arrival - device` offset seen recently is the closest estimate of the true
/// one. One minimum per second of arrival time keeps the window cheap. A step
/// in either clock restarts the estimate, so stamps follow the host through a
/// time adjustment instead of lagging it for a window.
#[derive(Debug, Default)]
pub struct HostClock {
    minima: VecDeque<(i64, i64)>,
    offset_ns: Option<i64>,
}

impl HostClock {
    /// Fold in one packet's device stamp and host arrival time.
    pub fn observe(&mut self, device_ns: u64, arrival_secs: f64) {
        let arrival_ns = secs_to_ns(arrival_secs);
        let offset = arrival_ns - device_ns as i64;
        if self
            .offset_ns
            .is_some_and(|estimate| (offset - estimate).abs() >= HOST_CLOCK_RESET_NS)
        {
            tracing::warn!(
                estimate_ns = self.offset_ns,
                offset_ns = offset,
                "clock offset jumped, restarting host clock estimate"
            );
            self.minima.clear();
        }
        let second = arrival_ns.div_euclid(NANOS_PER_SEC);
        match self.minima.back_mut() {
            Some(last) if last.0 == second => last.1 = last.1.min(offset),
            _ => {
                self.minima.push_back((second, offset));
                while self
                    .minima
                    .front()
                    .is_some_and(|f| f.0 < second - HOST_CLOCK_WINDOW_SECS)
                {
                    self.minima.pop_front();
                }
            }
        }
        self.offset_ns = self.minima.iter().map(|m| m.1).min();
    }

    /// The device stamp on the host clock. Unchanged until the first observation.
    pub fn host_ns(&self, device_ns: u64) -> u64 {
        match self.offset_ns {
            Some(offset) => (device_ns as i64).saturating_add(offset).max(0) as u64,
            None => device_ns,
        }
    }
}

/// Whole and fractional seconds converted separately, so an epoch-sized value
/// keeps sub-microsecond resolution.
fn secs_to_ns(secs: f64) -> i64 {
    secs.trunc() as i64 * NANOS_PER_SEC + (secs.fract() * NANOS_PER_SEC as f64).round() as i64
}

/// Limit number of points, in case time stamps get stalled for example
/// This is like 10 seconds of points from the mid360
const MAX_FRAME_POINTS: usize = 2_000_000;

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
    /// timestamp crosses the frame boundary. The packet's own points open the
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
            // A full interval backwards is a clock discontinuity, not
            // reordering. Re-anchor or no frame would ever cut again.
            Some(start) if start.saturating_sub(ts_ns) >= self.frame_interval_ns => {
                tracing::warn!(
                    anchor_ns = start,
                    packet_ns = ts_ns,
                    "device clock jumped backwards, re-anchoring frame"
                );
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
        self.append_points(packet, ts_ns.saturating_sub(frame_start));

        if completed.is_none() && self.points.len() >= MAX_FRAME_POINTS {
            tracing::warn!(
                points = self.points.len(),
                "frame point cap hit without a time boundary, forcing cut"
            );
            self.frame_start_ns = Some(ts_ns);
            return self.take_frame(frame_start);
        }
        completed
    }

    /// Convert one packet's points to meters and frame-relative offsets.
    fn append_points(&mut self, packet: &DataPacket<'_>, packet_offset_ns: u64) {
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

/// Packets in, host-stamped frames and IMU records out.
pub struct Pipeline {
    assembler: FrameAssembler,
    clock: HostClock,
}

impl Pipeline {
    pub fn new(frequency_hz: f64) -> Self {
        Pipeline {
            assembler: FrameAssembler::new(frequency_hz),
            clock: HostClock::default(),
        }
    }

    /// Feed one packet. A frame this packet completes is stamped with the
    /// clock as it stood before the packet, so a packet that resets the
    /// clock never re-dates the frame it closes.
    pub fn push(&mut self, packet: &DataPacket<'_>, arrival_secs: f64) -> Option<Frame> {
        let completed = self.assembler.push(packet).map(|f| self.stamp(f));
        self.clock.observe(packet.timestamp_ns, arrival_secs);
        completed
    }

    /// An IMU packet's samples on the host clock. Call after `push`.
    pub fn imu_records(&self, packet: &DataPacket<'_>) -> Vec<ImuRecord> {
        imu_records(packet)
            .map(|mut record| {
                record.ts_ns = self.clock.host_ns(record.ts_ns);
                record
            })
            .collect()
    }

    /// Emit whatever is accumulated, e.g. at end of stream.
    pub fn flush(&mut self) -> Option<Frame> {
        self.assembler.flush().map(|f| self.stamp(f))
    }

    fn stamp(&self, mut frame: Frame) -> Frame {
        frame.start_ns = self.clock.host_ns(frame.start_ns);
        frame
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
            data_type: DataType::CartesianHigh,
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
    fn host_clock_keeps_the_smallest_offset_in_the_window() {
        let mut clock = HostClock::default();
        assert_eq!(clock.host_ns(5), 5);
        // Device at 1000 s, host at 1.7e9 s: the true offset is the first,
        // undelayed arrival. Later packets arrive late and must not move it.
        let device = 1_000_000_000_000u64;
        clock.observe(device, 1_700_000_000.0);
        clock.observe(device + 5_000_000, 1_700_000_000.009);
        clock.observe(device + 10_000_000, 1_700_000_000.030);
        assert_eq!(clock.host_ns(device), 1_700_000_000_000_000_000);
        assert_eq!(clock.host_ns(device + 7), 1_700_000_000_000_000_007);
    }

    fn assert_near(actual: u64, expected: i64) {
        assert!(
            (actual as i64 - expected).abs() < 1_000,
            "{actual} vs {expected}"
        );
    }

    #[test]
    fn host_clock_forgets_offsets_outside_the_window() {
        let mut clock = HostClock::default();
        let device = 1_000_000_000_000u64;
        clock.observe(device, 1_700_000_000.0);
        // The device drifts 2 ms behind, so every later arrival carries a
        // larger offset. Inside the window the old minimum still wins.
        let mid_device = device + 10_000_000_000;
        clock.observe(mid_device, 1_700_000_010.002);
        assert_near(clock.host_ns(mid_device), 1_700_000_010_000_000_000);
        // Past the window the old minimum is gone and the estimate follows.
        let late_device = device + 60_000_000_000;
        clock.observe(late_device, 1_700_000_060.002);
        assert_near(clock.host_ns(late_device), 1_700_000_060_002_000_000);
    }

    #[test]
    fn host_clock_ignores_small_backwards_steps() {
        let mut clock = HostClock::default();
        let device = 1_000_000_000_000u64;
        clock.observe(device, 1_700_000_000.0);
        // Half a second back is reordering, not a reset, so the minimum holds.
        clock.observe(device - 500_000_000, 1_700_000_000.01);
        assert_eq!(clock.host_ns(device), 1_700_000_000_000_000_000);
    }

    #[test]
    fn host_clock_restarts_after_a_device_reset() {
        let mut clock = HostClock::default();
        clock.observe(1_000_000_000_000, 1_700_000_000.0);
        // The sensor reboots: device time drops to 2 s while the host moves on.
        clock.observe(2_000_000_000, 1_700_000_100.0);
        assert_eq!(clock.host_ns(2_000_000_000), 1_700_000_100_000_000_000);
    }

    #[test]
    fn host_clock_follows_a_host_clock_step() {
        // Either direction: the stamps step with the host at the next packet
        // instead of holding the stale minimum for the rest of the window.
        for step in [5.0, -2.0] {
            let mut clock = HostClock::default();
            let device = 1_000_000_000_000u64;
            for i in 0..10u64 {
                clock.observe(device + i * 5_000_000, 1_700_000_000.0 + i as f64 * 0.005);
            }
            let stepped_device = device + 50_000_000;
            let stepped_arrival = 1_700_000_000.05 + step;
            clock.observe(stepped_device, stepped_arrival);
            assert_near(clock.host_ns(stepped_device), secs_to_ns(stepped_arrival));
        }
    }

    #[test]
    fn pipeline_stamps_a_frame_with_the_clock_that_captured_it() {
        let mut pipeline = Pipeline::new(10.0); // 100 ms frames
        let device = 1_000_000_000_000u64;
        let host = 1_700_000_000.0;
        for i in 0..2u64 {
            let bytes = point_packet(device + i * 50_000_000, 0, &[simple_point(1)]);
            let arrival = host + i as f64 * 0.05;
            assert!(pipeline
                .push(&DataPacket::parse(&bytes).unwrap(), arrival)
                .is_none());
        }
        // The sensor reboots: device time drops to 2 s, 100 s of host time on.
        let reset = point_packet(2_000_000_000, 0, &[simple_point(2)]);
        let frame = pipeline
            .push(&DataPacket::parse(&reset).unwrap(), host + 100.0)
            .expect("the reset packet closes the open frame");
        // The closed frame keeps the offset it was captured under.
        assert_near(frame.start_ns, 1_700_000_000_000_000_000);
        // Everything after the reset is on the new offset.
        let imu = build_imu_samples(&[ImuSample {
            gyro: [0.0; 3],
            acc_g: [0.0, 0.0, 1.0],
        }]);
        let imu_bytes = DataPacket {
            time_interval: 0,
            dot_num: 1,
            data_type: DataType::Imu,
            timestamp_ns: 2_005_000_000,
            payload: &imu,
        }
        .build();
        let imu_packet = DataPacket::parse(&imu_bytes).unwrap();
        assert!(pipeline.push(&imu_packet, host + 100.005).is_none());
        let records = pipeline.imu_records(&imu_packet);
        assert_near(records[0].ts_ns, 1_700_000_100_005_000_000);
        assert_near(
            pipeline.flush().unwrap().start_ns,
            1_700_000_100_000_000_000,
        );
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
        // 100 us first-to-last span over 2 points -> 100 us point spacing.
        assert_eq!(frame.points[1].offset_ns, 100_000);
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
        assert!(assembler
            .push(&DataPacket::parse(&stale).unwrap())
            .is_none());
        let frame = assembler.flush().unwrap();
        assert_eq!(frame.points.len(), 2);
        assert_eq!(frame.points[1].offset_ns, 0);
    }

    #[test]
    fn backwards_clock_jump_reanchors_instead_of_stalling() {
        let mut assembler = FrameAssembler::new(10.0); // 100 ms frames
        let start = 1_700_000_000_000_000_000u64;
        for i in 0..3 {
            let bytes = point_packet(start + i * 50_000_000, 0, &[simple_point(1)]);
            assembler.push(&DataPacket::parse(&bytes).unwrap());
        }
        // A capture seam jumps the stamp far backwards.
        let mut frames = Vec::new();
        for i in 0..21 {
            let bytes = point_packet(i * 50_000_000, 0, &[simple_point(2)]);
            if let Some(frame) = assembler.push(&DataPacket::parse(&bytes).unwrap()) {
                frames.push(frame);
            }
        }

        // The jump packet flushes the stale frame and re-anchors.
        assert_eq!(frames.len(), 11);
        assert_eq!(frames[0].start_ns, start + 100_000_000);
        assert_eq!(frames[0].points.len(), 1);
        assert_eq!(frames[1].start_ns, 0);
        assert_eq!(frames[1].points[0].offset_ns, 0);
        assert!(frames[1..].iter().all(|f| f.points.len() == 2));
        assert_eq!(assembler.flush().unwrap().points.len(), 1);
    }

    #[test]
    fn frozen_timestamp_cuts_at_point_cap() {
        let mut assembler = FrameAssembler::new(10.0);
        let points: Vec<PointHigh> = (0..1000).map(|_| simple_point(1)).collect();
        let bytes = point_packet(7, 0, &points);
        let packet = DataPacket::parse(&bytes).unwrap();

        let mut cuts = 0;
        for _ in 0..2 * MAX_FRAME_POINTS / points.len() {
            if let Some(frame) = assembler.push(&packet) {
                assert_eq!(frame.start_ns, 7);
                assert_eq!(frame.points.len(), MAX_FRAME_POINTS);
                cuts += 1;
            }
        }
        assert_eq!(cuts, 2);
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
            data_type: DataType::Imu,
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
            data_type: DataType::Imu,
            timestamp_ns: 7,
            payload: &imu_payload,
        }
        .build();
        assert!(assembler
            .push(&DataPacket::parse(&imu_bytes).unwrap())
            .is_none());
        assert!(assembler.flush().is_none());
    }
}
