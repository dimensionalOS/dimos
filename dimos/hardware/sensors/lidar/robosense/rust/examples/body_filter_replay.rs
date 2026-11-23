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

//! Replays recorded Airy scans through the module's body filter, for checking it against another filter and timing it.
//!
//! ```text
//! body_filter_replay decode CAPTURE.pcap SCANS.bin CONFIG.json   # both lidars' scans, decoded as the module does
//! body_filter_replay cut SCANS.bin CONFIG.json MASKS.bin [REPEATS] # each scan cut at joint_sets[i % len]; mask + time
//! ```
//! CONFIG.json: {"urdf_path", "padding_m", "links" (null = all), "min_distance", "max_distance", "voxel",
//! "lidars": {frame_id: {"msop_port", "difop_port", "xyz", "rpy"}}, "joint_sets": [{joint: position}, ...]}.
//! SCANS.bin, per scan: u32 frame_id length, frame_id, f64 stamp, u32 n, n x (f32 x, y, z).
//! MASKS.bin, per scan: u32 joint set, f64 best cut ms over the repeats, u32 n, n x u8 (1 = on the body).

use std::collections::{BTreeMap, HashMap};
use std::io::{Read, Write};

use dimos_robosense::body_cut::{BodyCut, CutConfig, JointSource};
use robosense_airy::{
    classify, pcap, Decoder, DecoderConfig, FilterConfig, PacketKind, Point, Scan, SplitMode,
};
use serde::Deserialize;

#[derive(Deserialize)]
struct Lidar {
    msop_port: u16,
    difop_port: u16,
    xyz: [f64; 3],
    rpy: [f64; 3],
}

#[derive(Deserialize)]
struct Config {
    urdf_path: String,
    padding_m: f64,
    links: Option<Vec<String>>,
    min_distance: f32,
    max_distance: f32,
    voxel: f32,
    lidars: BTreeMap<String, Lidar>,
    joint_sets: Vec<HashMap<String, f64>>,
}

fn read_config(path: &str) -> Config {
    serde_json::from_str(&std::fs::read_to_string(path).expect("read config"))
        .expect("parse config")
}

fn decode(pcap_path: &str, out: &str, config: &Config) {
    let mut datagrams = pcap::read_udp(pcap_path).expect("read pcap");
    datagrams.sort_by(|a, b| a.time.total_cmp(&b.time));
    // the module's decoder settings (dimos_robosense::module::Config::sensor_config)
    let decoder_config = DecoderConfig {
        min_distance: config.min_distance,
        max_distance: config.max_distance,
        use_lidar_clock: true,
        dense_points: true,
        ts_first_point: true,
        wait_for_difop: true,
        start_angle: 0.0,
        end_angle: 360.0,
        split: SplitMode::Angle(0.0),
        transform: None,
        fused_multiply_add: false,
    };
    let filter = FilterConfig {
        voxel: config.voxel,
        decimate_points: 1,
        decimate_scans: 1,
    };
    let mut decoders: Vec<(&String, &Lidar, Decoder)> = config
        .lidars
        .iter()
        .map(|(frame, lidar)| (frame, lidar, Decoder::new(decoder_config.clone())))
        .collect();
    for (_, lidar, decoder) in &mut decoders {
        for d in datagrams.iter().filter(|d| d.dst_port == lidar.difop_port) {
            let _ = decoder.decode_difop(&d.payload);
        }
    }
    let mut file = std::io::BufWriter::new(std::fs::File::create(out).expect("create scans file"));
    let mut count = 0;
    for d in &datagrams {
        if classify(&d.payload) != Some(PacketKind::Msop) {
            continue;
        }
        for (frame, lidar, decoder) in &mut decoders {
            if d.dst_port != lidar.msop_port {
                continue;
            }
            if let Ok(Some(mut scan)) = decoder.decode_msop(&d.payload) {
                if !scan.complete {
                    continue;
                }
                robosense_airy::filter::apply(&mut scan.points, &filter);
                file.write_all(&(frame.len() as u32).to_le_bytes()).unwrap();
                file.write_all(frame.as_bytes()).unwrap();
                file.write_all(&scan.timestamp.to_le_bytes()).unwrap();
                file.write_all(&(scan.points.len() as u32).to_le_bytes())
                    .unwrap();
                for p in &scan.points {
                    for v in [p.x, p.y, p.z] {
                        file.write_all(&v.to_le_bytes()).unwrap();
                    }
                }
                count += 1;
            }
        }
    }
    eprintln!("{count} scans -> {out}");
}

fn take<const N: usize>(bytes: &[u8], at: &mut usize) -> [u8; N] {
    let out = bytes[*at..*at + N].try_into().unwrap();
    *at += N;
    out
}

fn read_scans(path: &str) -> Vec<(String, Scan)> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)
        .expect("open scans")
        .read_to_end(&mut bytes)
        .unwrap();
    let (mut at, mut scans) = (0, Vec::new());
    while at < bytes.len() {
        let len = u32::from_le_bytes(take(&bytes, &mut at)) as usize;
        let frame = String::from_utf8(bytes[at..at + len].to_vec()).unwrap();
        at += len;
        let stamp = f64::from_le_bytes(take(&bytes, &mut at));
        let n = u32::from_le_bytes(take(&bytes, &mut at)) as usize;
        let points = (0..n)
            .map(|_| {
                let [x, y, z] = [0; 3].map(|_| f32::from_le_bytes(take(&bytes, &mut at)));
                Point {
                    x,
                    y,
                    z,
                    intensity: 0,
                    ring: 0,
                    time: stamp,
                    feature: 0,
                }
            })
            .collect();
        scans.push((
            frame,
            Scan {
                seq: 0,
                timestamp: stamp,
                rings: 96,
                points,
                complete: true,
            },
        ));
    }
    scans
}

fn cut(scans_path: &str, config: &Config, out: &str, repeats: usize) {
    let urdf = std::fs::read_to_string(&config.urdf_path).expect("read urdf");
    let scans = read_scans(scans_path);
    let mut file = std::io::BufWriter::new(std::fs::File::create(out).expect("create masks file"));
    let mut cutters: HashMap<(String, usize), BodyCut> = HashMap::new();
    let mut times = Vec::with_capacity(scans.len());
    for (i, (frame, scan)) in scans.iter().enumerate() {
        let set = i % config.joint_sets.len();
        let cutter = cutters.entry((frame.clone(), set)).or_insert_with(|| {
            let lidar = &config.lidars[frame];
            BodyCut::new(CutConfig {
                urdf: urdf.clone(),
                padding_m: config.padding_m,
                links: config.links.clone(),
                source: JointSource::Fixed,
                fixed_positions: config.joint_sets[set].clone(),
                state_tolerance_s: 0.0,
                mount_parent: "base_link".into(),
                root_from_lidar: urdf_body_filter::Transform::from_xyz_rpy(lidar.xyz, lidar.rpy),
            })
            .expect("body filter")
        });
        // the module's per-scan work: pose, then retain in place (timed best of `repeats`)
        let mut best = f64::INFINITY;
        let mut kept = Vec::new();
        for _ in 0..repeats.max(1) {
            let mut copy = scan.clone();
            let start = std::time::Instant::now();
            assert!(cutter.cut(&mut copy));
            best = best.min(start.elapsed().as_secs_f64() * 1e3);
            kept = copy.points;
        }
        times.push(best);
        // the mask from the kept points (retain keeps order, so walk both)
        let mut mask = vec![1u8; scan.points.len()];
        let mut k = 0;
        for (j, p) in scan.points.iter().enumerate() {
            if k < kept.len() && kept[k] == *p {
                mask[j] = 0;
                k += 1;
            }
        }
        assert_eq!(k, kept.len(), "kept points are not an ordered subset");
        file.write_all(&(set as u32).to_le_bytes()).unwrap();
        file.write_all(&best.to_le_bytes()).unwrap();
        file.write_all(&(mask.len() as u32).to_le_bytes()).unwrap();
        file.write_all(&mask).unwrap();
    }
    times.sort_by(f64::total_cmp);
    let pct = |q: f64| times[((times.len() - 1) as f64 * q).round() as usize];
    let points: usize = scans.iter().map(|(_, s)| s.points.len()).sum();
    eprintln!(
        "{} scans, {:.0} points/scan: cut ms median {:.3} p95 {:.3} max {:.3}",
        times.len(),
        points as f64 / scans.len() as f64,
        pct(0.5),
        pct(0.95),
        pct(1.0)
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("decode") if args.len() == 5 => decode(&args[2], &args[3], &read_config(&args[4])),
        Some("cut") if args.len() >= 5 => {
            let repeats = args.get(5).map_or(1, |r| r.parse().expect("repeats"));
            cut(&args[2], &read_config(&args[3]), &args[4], repeats)
        }
        _ => {
            eprintln!("usage: body_filter_replay decode CAPTURE.pcap SCANS.bin CONFIG.json | cut SCANS.bin CONFIG.json MASKS.bin [REPEATS]");
            std::process::exit(2)
        }
    }
}
