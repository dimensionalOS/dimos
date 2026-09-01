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

use dimos_livox::pipeline::{imu_records, FrameAssembler, PacketSource};
use dimos_livox::wire::{self, DataPacket, DataType};

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
    let mut frames = Vec::new();
    let mut first_imu = None;
    let mut last_imu = None;

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
                    frames.push(frame);
                }
            }
        }
    }
    frames.extend(assembler.flush());

    let total_points: usize = frames.iter().map(|f| f.points.len()).sum();
    let first = frames.first().map(|f| f.start_ns).unwrap_or(0);
    let last = frames.last().map(|f| f.start_ns).unwrap_or(0);
    let span_s = (last.saturating_sub(first)) as f64 / 1e9;
    let monotonic = frames.iter().all(|f| {
        f.points
            .windows(2)
            .all(|w| w[0].offset_ns <= w[1].offset_ns)
    });

    println!("point packets:   {point_packets}");
    println!("imu packets:     {imu_packets} ({imu_samples} samples)");
    println!("bad packets:     {bad_packets}");
    println!(
        "frames:          {} over {span_s:.1} s of sensor time",
        frames.len()
    );
    if !frames.is_empty() {
        if frames.len() > 1 && span_s > 0.0 {
            println!(
                "frame rate:      {:.2} Hz (data time)",
                (frames.len() - 1) as f64 / span_s
            );
        } else {
            println!("frame rate:      n/a (single frame)");
        }
        println!(
            "points/frame:    {:.0} avg",
            total_points as f64 / frames.len() as f64
        );
        println!("offsets sorted:  {monotonic}");
        let f = &frames[frames.len() / 2];
        let max_off = f.points.iter().map(|p| p.offset_ns).max().unwrap_or(0);
        println!(
            "mid frame:       {} pts, start {:.3} s, max offset {:.1} ms",
            f.points.len(),
            f.start_ns as f64 / 1e9,
            max_off as f64 / 1e6
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
