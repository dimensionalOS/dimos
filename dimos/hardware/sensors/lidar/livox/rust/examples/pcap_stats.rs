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

// Replay a Mid-360 pcap through PcapSource + FrameAssembler and print
// stream statistics. Quick sanity check for any capture:
//   cargo run --release -p dimos-livox --example pcap_stats -- <pcap>
//
// Everything is a running fold, so memory stays flat for any capture size.

use dimos_livox::pipeline::{imu_records, Frame, FrameAssembler, PacketSource};
use dimos_livox::wire::{self, DataPacket, DataType};
use std::time::{Duration, Instant};

#[derive(Default)]
struct FrameStats {
    count: u64,
    total_points: u64,
    min_points: u64,
    max_points: u64,
    first_start_ns: u64,
    last_start_ns: u64,
    unsorted: u64,
    max_offset_ns: u32,
}

impl FrameStats {
    fn note(&mut self, frame: &Frame) {
        let points = frame.points.len() as u64;
        if self.count == 0 {
            self.first_start_ns = frame.start_ns;
            self.min_points = points;
        }
        self.count += 1;
        self.total_points += points;
        self.min_points = self.min_points.min(points);
        self.max_points = self.max_points.max(points);
        self.last_start_ns = frame.start_ns;
        let sorted = frame
            .points
            .windows(2)
            .all(|w| w[0].offset_ns <= w[1].offset_ns);
        if !sorted {
            self.unsorted += 1;
        }
        if let Some(max) = frame.points.iter().map(|p| p.offset_ns).max() {
            self.max_offset_ns = self.max_offset_ns.max(max);
        }
    }
}

fn main() {
    let path = std::env::args().nth(1).expect("usage: pcap_stats <pcap>");
    let mut source = dimos_livox::pcap::PcapSource::from_file(
        &path,
        wire::LIDAR_POINT_PORT,
        wire::LIDAR_IMU_PORT,
        None,
    )
    .expect("parse pcap");

    let mut assembler = FrameAssembler::new(10.0);
    let mut buf = [0u8; 4096];
    let mut point_packets = 0u64;
    let mut imu_packets = 0u64;
    let mut imu_samples = 0u64;
    let mut bad_packets = 0u64;
    let mut first_imu = None;
    let mut last_imu = None;
    let mut stats = FrameStats::default();
    let mut frame_time_sum = 0f64;
    let mut frame_time_max = Duration::ZERO;
    let mut frame_times = 0u64;
    let started = Instant::now();
    let mut frame_started = started;

    while let Some(len) = source.recv(&mut buf) {
        let packet = match DataPacket::parse(&buf[..len]) {
            Ok(p) => p,
            Err(_) => {
                bad_packets += 1;
                continue;
            }
        };
        match packet.data_type {
            DataType::Imu => {
                imu_packets += 1;
                for record in imu_records(&packet) {
                    imu_samples += 1;
                    first_imu.get_or_insert(record.ts_ns);
                    last_imu = Some(record.ts_ns);
                }
            }
            _ => {
                point_packets += 1;
                if let Some(frame) = assembler.push(&packet) {
                    let elapsed = frame_started.elapsed();
                    frame_time_sum += elapsed.as_secs_f64();
                    frame_time_max = frame_time_max.max(elapsed);
                    frame_times += 1;
                    frame_started = Instant::now();
                    stats.note(&frame);
                }
            }
        }
    }
    if let Some(frame) = assembler.flush() {
        stats.note(&frame);
    }
    let total_time = started.elapsed();

    let span_s = (stats.last_start_ns.saturating_sub(stats.first_start_ns)) as f64 / 1e9;

    println!("point packets:   {point_packets}");
    println!("imu packets:     {imu_packets} ({imu_samples} samples)");
    println!("bad packets:     {bad_packets}");
    println!(
        "processing:      {:.0} ms total, {:.2} M pts/s",
        total_time.as_secs_f64() * 1e3,
        stats.total_points as f64 / total_time.as_secs_f64() / 1e6
    );
    if frame_times > 0 {
        println!(
            "frame process:   {:.3} ms avg, {:.3} ms max",
            frame_time_sum / frame_times as f64 * 1e3,
            frame_time_max.as_secs_f64() * 1e3
        );
    }
    println!(
        "frames:          {} over {span_s:.1} s of sensor time",
        stats.count
    );
    if stats.count > 0 {
        if stats.count > 1 && span_s > 0.0 {
            println!(
                "frame rate:      {:.2} Hz (data time)",
                (stats.count - 1) as f64 / span_s
            );
        } else {
            println!("frame rate:      n/a (single frame)");
        }
        println!(
            "points/frame:    {:.0} avg ({}..{})",
            stats.total_points as f64 / stats.count as f64,
            stats.min_points,
            stats.max_points
        );
        println!("offsets sorted:  {}", stats.unsorted == 0);
        println!(
            "max offset:      {:.1} ms",
            stats.max_offset_ns as f64 / 1e6
        );
    }
    if let (Some(a), Some(b)) = (first_imu, last_imu) {
        let imu_span = (b.saturating_sub(a)) as f64 / 1e9;
        if imu_samples > 1 && imu_span > 0.0 {
            println!(
                "imu rate:        {:.1} Hz over {imu_span:.1} s",
                (imu_samples - 1) as f64 / imu_span
            );
        } else {
            println!("imu rate:        n/a (single sample)");
        }
    }
}
