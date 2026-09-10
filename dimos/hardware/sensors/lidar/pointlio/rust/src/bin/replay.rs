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

// pointlio_replay: dump a Mid-360 pcap to an L0 file, summarise one, replay it through
// the Rust core (same CLI as ../cpp/harness.cpp), or compare two golden directories.

use dimos_livox::pcap::PcapSource;
use dimos_livox::pipeline::{imu_records, FrameAssembler, PacketSource, GRAVITY_MS2};
use dimos_livox::wire::{DataPacket, DataType, LIDAR_IMU_PORT, LIDAR_POINT_PORT};
use dimos_pointlio::compare::{self, Tol};
use dimos_pointlio::golden::{Frame, PointDump, Pose};
use dimos_pointlio::replay::{self, RawPoint, Reader, Record};
use pointlio_core::{Config, LivoxPoint, PointLio};
use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Instant;

const USAGE: &str = "usage:
  pointlio_replay dump --pcap <file> --out <file> [--hz 10] [--max-seconds N]
  pointlio_replay info <file>
  pointlio_replay run --replay f.plio --config c.json --out dir [--frames K=30] [--max-frames N] [--stats]
  pointlio_replay compare <golden_dir> <candidate_dir> [--tol-state 1e-9] [--tol-plane-ulp 4]
  pointlio_replay perturb <in.plio> <out.plio> [--point-ulp F] [--imu-drop F] [--imu-ulp F]
      F = lidar frame index: every point x +1 ULP / the first IMU record after that frame";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("dump") => dump(&args[1..]),
        Some("info") if args.len() == 2 => info(&args[1]),
        Some("run") => run(&args[1..]),
        Some("compare") if args.len() >= 3 => compare_dirs(&args[1..]),
        Some("perturb") if args.len() >= 3 => perturb(&args[1..]),
        _ => {
            eprintln!("{USAGE}");
            std::process::exit(2);
        }
    };
    match result {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(err) => {
            eprintln!("error: {err}");
            std::process::exit(1);
        }
    }
}

fn parse<T: std::str::FromStr>(args: &[String], name: &str, default: T) -> io::Result<T>
where
    T::Err: std::fmt::Display,
{
    flag(args, name).map_or(Ok(default), |s| {
        s.parse()
            .map_err(|e| io::Error::other(format!("{name}: {e}")))
    })
}

fn parse_opt<T: std::str::FromStr>(args: &[String], name: &str) -> io::Result<Option<T>>
where
    T::Err: std::fmt::Display,
{
    flag(args, name)
        .map(|s| {
            s.parse()
                .map_err(|e| io::Error::other(format!("{name}: {e}")))
        })
        .transpose()
}

fn flag(args: &[String], name: &str) -> Option<String> {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1).cloned())
}

fn required(args: &[String], name: &str) -> io::Result<String> {
    flag(args, name).ok_or_else(|| io::Error::other(format!("{name} is required\n{USAGE}")))
}

/// Same packet classification and feed as the Mid360 module's run_pipeline.
fn dump(args: &[String]) -> io::Result<bool> {
    let pcap = required(args, "--pcap")?;
    let out = required(args, "--out")?;
    let hz: f64 = parse(args, "--hz", 10.0)?;
    let max_ns: Option<u64> = flag(args, "--max-seconds")
        .map(|s| {
            s.parse::<f64>()
                .map(|s| (s * 1e9) as u64)
                .map_err(io::Error::other)
        })
        .transpose()?;

    let stop = Arc::new(AtomicBool::new(false));
    let mut source = PcapSource::from_file(&pcap, LIDAR_POINT_PORT, LIDAR_IMU_PORT, None, stop)?;
    let mut assembler = FrameAssembler::new(hz);
    let mut records = Vec::new();
    let mut buf = [0u8; 4096];
    let mut first_ns = None;
    // Raw-stream regressions per type [imu, point], as (count, largest jump ns).
    let mut prev = [0u64; 2];
    let mut back = [(0usize, 0u64); 2];
    let mut undecodable = 0usize;
    while let Some(len) = source.recv(&mut buf) {
        let Ok(packet) = DataPacket::parse(&buf[..len]) else {
            undecodable += 1;
            continue;
        };
        let ts = packet.timestamp_ns;
        let first = *first_ns.get_or_insert(ts);
        if max_ns.is_some_and(|max| ts.saturating_sub(first) >= max) {
            break;
        }
        let slot = usize::from(packet.data_type != DataType::Imu);
        if ts < prev[slot] {
            back[slot].0 += 1;
            back[slot].1 = back[slot].1.max(prev[slot] - ts);
        }
        prev[slot] = ts;
        match packet.data_type {
            DataType::Imu => records.extend(imu_records(&packet).map(|r| Record::Imu {
                ts_ns: r.ts_ns,
                gyro: r.gyro_rads,
                acc: r.acc_ms2,
            })),
            _ => {
                if let Some(f) = assembler.push(&packet) {
                    records.push(Record::Lidar {
                        start_ns: f.start_ns,
                        points: f.points,
                    });
                }
            }
        }
    }
    if let Some(reason) = source.failure() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, reason));
    }
    if let Some(f) = assembler.flush() {
        records.push(Record::Lidar {
            start_ns: f.start_ns,
            points: f.points,
        });
    }
    replay::write(&out, hz, &records)?;
    eprintln!(
        "wrote {out}: {} records, {undecodable} undecodable packets, raw ts regressions imu {}x (max {:.3} s) point {}x (max {:.3} s)",
        records.len(),
        back[0].0,
        back[0].1 as f64 * 1e-9,
        back[1].0,
        back[1].1 as f64 * 1e-9,
    );
    info(&out)
}

fn info(path: &str) -> io::Result<bool> {
    let reader = Reader::open(path)?;
    let hz = reader.hz;
    let (mut imu, mut frames, mut points, mut empty, mut regressions) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    let (mut min_pts, mut max_pts) = (u32::MAX, 0u32);
    // (first, last, max gap) per stream, in ns.
    let mut imu_span = (0u64, 0u64, 0u64);
    let mut frame_span = (0u64, 0u64, 0u64);
    let mut avail = (0u64, 0u64);
    for record in reader {
        let record = record?;
        let a = record.avail_ns();
        if imu + frames == 0 {
            avail.0 = a;
        } else if a < avail.1 {
            regressions += 1;
        }
        avail.1 = a;
        let (n, span, ts) = match &record {
            Record::Imu { ts_ns, .. } => (&mut imu, &mut imu_span, *ts_ns),
            Record::Lidar {
                start_ns,
                points: pts,
            } => {
                let len = pts.len() as u32;
                points += u64::from(len);
                empty += u64::from(len == 0);
                min_pts = min_pts.min(len);
                max_pts = max_pts.max(len);
                (&mut frames, &mut frame_span, *start_ns)
            }
        };
        if *n == 0 {
            span.0 = ts;
        } else {
            span.2 = span.2.max(ts.saturating_sub(span.1));
        }
        span.1 = ts;
        *n += 1;
    }
    let s = |ns: u64| ns as f64 * 1e-9;
    let rate = |n: u64, span: (u64, u64, u64)| {
        (n.saturating_sub(1)) as f64 / s(span.1.saturating_sub(span.0)).max(f64::MIN_POSITIVE)
    };
    println!("{path}");
    println!("  frame_hz (header):  {hz}");
    println!(
        "  duration:           {:.3} s (avail {} .. {})",
        s(avail.1.saturating_sub(avail.0)),
        avail.0,
        avail.1
    );
    println!(
        "  imu:                {imu} records, {:.2} Hz, max gap {:.4} s",
        rate(imu, imu_span),
        s(imu_span.2)
    );
    println!(
        "  lidar:              {frames} frames, {:.3} Hz, max start gap {:.4} s, pts/frame mean {:.1} min {} max {}, empty {empty}",
        rate(frames, frame_span),
        s(frame_span.2),
        points as f64 / frames.max(1) as f64,
        if frames == 0 { 0 } else { min_pts },
        max_pts,
    );
    println!("  ts regressions:     {regressions}");
    Ok(true)
}

/// Harness `feed_lidar`: reflectivity = lround(intensity * 255), line 0.
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

fn to_frame(idx: u32, d: pointlio_core::FrameDump) -> Frame {
    Frame {
        idx,
        lidar_ts: d.lidar_ts,
        feats_down_body: d.feats_down_body,
        n_eff: d.n_eff,
        state: d.state,
        p: d.p,
        points: d
            .points
            .into_iter()
            .map(|p| PointDump {
                selected: p.selected,
                neighbours: p.neighbours,
                plane: p.plane,
            })
            .collect(),
    }
}

/// Same feed sequence, drain rule and dump as harness.cpp; `--max-frames` stops early.
fn run(args: &[String]) -> io::Result<bool> {
    let replay = required(args, "--replay")?;
    let config = required(args, "--config")?;
    let out = required(args, "--out")?;
    let frames: u32 = parse(args, "--frames", 30)?;
    let max_frames: u32 = parse(args, "--max-frames", u32::MAX)?;
    let stats = args.iter().any(|a| a == "--stats");

    let cfg: Config = serde_json::from_reader(File::open(&config)?)?;
    let mut lio = PointLio::new(&cfg);
    let out = Path::new(&out);
    fs::create_dir_all(out)?;
    fs::copy(&config, out.join("config.json"))?;
    let mut tum = BufWriter::new(File::create(out.join("trajectory.tum"))?);
    let mut fbin = BufWriter::new(File::create(out.join("frames.bin"))?);

    let mut processed = 0u32;
    let mut lidar_ms = Vec::new();
    let t_start = Instant::now();
    for record in Reader::open(&replay)? {
        match record? {
            Record::Imu { ts_ns, gyro, acc } => {
                lio.feed_imu(ts_ns as f64 / 1e9, gyro, acc.map(|a| a / GRAVITY_MS2));
            }
            Record::Lidar { start_ns, points } => {
                let t0 = Instant::now();
                let pts: Vec<LivoxPoint> = points.iter().map(to_livox).collect();
                lio.feed_lidar(start_ns, &pts);
                // Drain: process() is false both on "no frame" and on the map-init frame,
                // so stop only when it consumed nothing.
                loop {
                    let before = lio.lidar_buffer_len();
                    if lio.process() {
                        let o = lio.odometry();
                        Pose {
                            ts: o.ts,
                            pos: o.pos,
                            quat: o.quat,
                        }
                        .write_to(&mut tum)?;
                        if processed < frames {
                            to_frame(processed, lio.frame_dump()).write_to(&mut fbin)?;
                        }
                        processed += 1;
                    }
                    if lio.lidar_buffer_len() == before {
                        break;
                    }
                }
                lidar_ms.push(t0.elapsed().as_secs_f64() * 1e3);
                if processed >= max_frames {
                    break;
                }
            }
        }
    }
    let total_s = t_start.elapsed().as_secs_f64();
    eprintln!(
        "frames processed: {processed} (dumped {})",
        processed.min(frames)
    );
    if stats {
        let pct = |p: f64| {
            let mut v = lidar_ms.clone();
            v.sort_by(f64::total_cmp);
            v.get((p * v.len() as f64) as usize)
                .or(v.last())
                .copied()
                .unwrap_or(0.0)
        };
        eprintln!(
            "lidar records: {}  per-record ms p50 {:.3} p99 {:.3} max {:.3}  total {total_s:.3} s",
            lidar_ms.len(),
            pct(0.5),
            pct(0.99),
            lidar_ms.iter().copied().fold(0.0, f64::max),
        );
    }
    Ok(true)
}

fn compare_dirs(args: &[String]) -> io::Result<bool> {
    let tol = Tol {
        state: parse(args, "--tol-state", 1e-9)?,
        plane_ulp: parse(args, "--tol-plane-ulp", 4)?,
    };
    let mut out = io::stdout().lock();
    compare::compare(Path::new(&args[0]), Path::new(&args[1]), &tol, &mut out)
}

/// Position of the k-th lidar record.
fn lidar_pos(records: &[Record], k: usize) -> io::Result<usize> {
    records
        .iter()
        .enumerate()
        .filter(|(_, r)| matches!(r, Record::Lidar { .. }))
        .nth(k)
        .map(|(i, _)| i)
        .ok_or_else(|| io::Error::other(format!("no lidar frame {k}")))
}

/// Position of the first IMU record after the k-th lidar record.
fn imu_after(records: &[Record], k: usize) -> io::Result<usize> {
    (lidar_pos(records, k)?..records.len())
        .find(|&i| matches!(records[i], Record::Imu { .. }))
        .ok_or_else(|| io::Error::other(format!("no IMU record after frame {k}")))
}

/// Noise-band probes: 1 f32 ULP on a frame's x, one IMU record dropped, 1 f64 ULP on one gyro.
fn perturb(args: &[String]) -> io::Result<bool> {
    let point_ulp: Option<usize> = parse_opt(args, "--point-ulp")?;
    let imu_drop: Option<usize> = parse_opt(args, "--imu-drop")?;
    let imu_ulp: Option<usize> = parse_opt(args, "--imu-ulp")?;
    let reader = Reader::open(&args[0])?;
    let hz = reader.hz;
    let mut records: Vec<Record> = reader.collect::<io::Result<_>>()?;
    if let Some(k) = point_ulp {
        let at = lidar_pos(&records, k)?;
        let Record::Lidar { points, .. } = &mut records[at] else {
            unreachable!()
        };
        // Whole frame: a single-point ULP is absorbed by the f32 voxel-centroid rounding.
        for p in points.iter_mut() {
            p.xyz_m[0] = p.xyz_m[0].next_up();
        }
        eprintln!("frame {k}: {} points x += 1 ULP", points.len());
    }
    if let Some(k) = imu_ulp {
        let at = imu_after(&records, k)?;
        let Record::Imu { ts_ns, gyro, .. } = &mut records[at] else {
            unreachable!()
        };
        let g = gyro[0];
        gyro[0] = g.next_up();
        eprintln!(
            "imu {ts_ns} (after frame {k}): gyro x {g:e} -> {:e}",
            gyro[0]
        );
    }
    if let Some(k) = imu_drop {
        let at = imu_after(&records, k)?;
        let Record::Imu { ts_ns, .. } = records.remove(at) else {
            unreachable!()
        };
        eprintln!("dropped imu {ts_ns} (after frame {k})");
    }
    replay::write(&args[1], hz, &records)?;
    eprintln!("wrote {}: {} records", args[1], records.len());
    Ok(true)
}
