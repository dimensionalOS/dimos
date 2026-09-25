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

// Replay a Mid-360 pcap through the estimator and write the trajectory as TUM.
//
// Frame by frame rather than wall-clock paced: the next packet is read only once
// the estimator has finished the last, so no frame drops and the output does not
// depend on machine load. The module's LCM path has no backpressure.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use dimos_livox::pcap::PcapSource;
use dimos_livox::pipeline::{imu_records, FrameAssembler, PacketSource, RawPoint, GRAVITY_MS2};
use dimos_livox::wire::{DataPacket, DataType, LIDAR_IMU_PORT, LIDAR_POINT_PORT};
use pointlio_core::{Config, LivoxPoint, PointLio};

fn flag<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    let i = args.iter().position(|a| a == name)?;
    args.get(i + 1).map(String::as_str)
}

fn to_livox(p: &RawPoint) -> LivoxPoint {
    LivoxPoint {
        x: p.xyz_m[0],
        y: p.xyz_m[1],
        z: p.xyz_m[2],
        reflectivity: (p.intensity * 255.0).round() as u16,
        tag: p.tag,
        line: 0,
        offset_time: u64::from(p.offset_ns),
    }
}

fn main() -> io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(pcap) = flag(&args, "--pcap") else {
        eprintln!("usage: pointlio_replay --pcap f.pcap [--config c.json] [--hz 10] [--out traj.tum] [--clouds world.bin]");
        std::process::exit(2);
    };
    let hz: f64 = flag(&args, "--hz")
        .unwrap_or("10")
        .parse()
        .map_err(io::Error::other)?;
    let cfg = match flag(&args, "--config") {
        Some(path) => serde_json::from_reader(File::open(path)?)?,
        None => Config::default(),
    };

    let mut lio = PointLio::new(&cfg);
    let mut out: Box<dyn Write> = match flag(&args, "--out") {
        Some(path) => Box::new(BufWriter::new(File::create(path)?)),
        None => Box::new(BufWriter::new(io::stdout())),
    };

    let stop = Arc::new(AtomicBool::new(false));
    // Optional world-frame cloud dump: per frame `u32 n` then n * (f32 x, y, z).
    let mut clouds = flag(&args, "--clouds")
        .map(|p| File::create(p).map(BufWriter::new))
        .transpose()?;

    let mut source = PcapSource::from_file(pcap, LIDAR_POINT_PORT, LIDAR_IMU_PORT, None, stop)?;
    let mut assembler = FrameAssembler::new(hz);
    let mut buf = [0u8; 4096];
    let mut frames = 0u32;

    let mut feed = |lio: &mut PointLio,
                    out: &mut dyn Write,
                    clouds: &mut Option<BufWriter<File>>,
                    start_ns,
                    points: &[RawPoint]| {
        let pts: Vec<LivoxPoint> = points.iter().map(to_livox).collect();
        lio.feed_lidar(start_ns, &pts);
        // process() is false both on "no frame" and on the map-init frame, so
        // stop only once it has stopped consuming.
        loop {
            let before = lio.lidar_buffer_len();
            if lio.process() {
                let o = lio.odometry();
                writeln!(
                    out,
                    "{:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9}",
                    o.ts, o.pos[0], o.pos[1], o.pos[2], o.quat[0], o.quat[1], o.quat[2], o.quat[3]
                )?;
                if let Some(w) = clouds.as_mut() {
                    let cloud = lio.world_cloud();
                    w.write_all(&(cloud.len() as u32).to_le_bytes())?;
                    for p in &cloud {
                        for v in [p.x, p.y, p.z] {
                            w.write_all(&v.to_le_bytes())?;
                        }
                    }
                }
                frames += 1;
            }
            if lio.lidar_buffer_len() == before {
                return io::Result::Ok(());
            }
        }
    };

    while let Some(len) = source.recv(&mut buf) {
        let Ok(packet) = DataPacket::parse(&buf[..len]) else {
            continue;
        };
        match packet.data_type {
            DataType::Imu => {
                for r in imu_records(&packet) {
                    lio.feed_imu(
                        r.ts_ns as f64 / 1e9,
                        r.gyro_rads,
                        r.acc_ms2.map(|a| a / GRAVITY_MS2),
                    );
                }
            }
            _ => {
                if let Some(f) = assembler.push(&packet) {
                    feed(&mut lio, &mut *out, &mut clouds, f.start_ns, &f.points)?;
                }
            }
        }
    }
    if let Some(reason) = source.failure() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, reason));
    }
    if let Some(f) = assembler.flush() {
        feed(&mut lio, &mut *out, &mut clouds, f.start_ns, &f.points)?;
    }
    out.flush()?;
    if let Some(mut w) = clouds {
        w.flush()?;
    }
    eprintln!("{frames} frames");
    Ok(())
}
