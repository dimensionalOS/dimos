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

//
// Offline per-frame benchmark of the Rust mapper ports, stage-for-stage
// comparable with scripts/benchmarks/baseline_mappers.py (the Python side):
//
//   add_frame  every frame       (voxelize + naive column carve + insert)
//   emit       every 5th frame   (rebuild global map + pack PointCloud2 bytes)
//   costmap    every 5th frame   (f64 upcast + height_cost_occupancy)
//
// Input: tests/golden_data/frames.bin (dump_golden.py), i.e. the identical
// 461 recorded go2_short.db frames the Python baseline consumed.
//
// Usage: cargo run --release --bin bench_mappers [out.csv]

use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

use dimos_mappers::cloud::points_to_cloud;
use dimos_mappers::occupancy::{height_cost_occupancy, HeightCostConfig};
use dimos_mappers::voxel_grid::VoxelGrid;
use lcm_msgs::std_msgs::Time;

const EMIT_EVERY: usize = 5;
const VOXEL_SIZE: f32 = 0.05;

fn read_frames(path: PathBuf) -> Vec<Vec<(f32, f32, f32)>> {
    let buf = fs::read(&path)
        .unwrap_or_else(|_| panic!("missing {path:?}; run scripts/benchmarks/dump_golden.py"));
    let mut frames = Vec::new();
    let mut off = 0;
    while off < buf.len() {
        let n = u32::from_le_bytes(buf[off..off + 4].try_into().unwrap()) as usize;
        off += 4;
        let mut pts = Vec::with_capacity(n);
        for i in 0..n {
            let base = off + i * 12;
            pts.push((
                f32::from_le_bytes(buf[base..base + 4].try_into().unwrap()),
                f32::from_le_bytes(buf[base + 4..base + 8].try_into().unwrap()),
                f32::from_le_bytes(buf[base + 8..base + 12].try_into().unwrap()),
            ));
        }
        off += n * 12;
        frames.push(pts);
    }
    frames
}

fn pct(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let idx = ((p / 100.0) * (sorted.len() as f64 - 1.0)).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn summarize(name: &str, vals: &[f64]) {
    let mut s = vals.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean = vals.iter().sum::<f64>() / vals.len() as f64;
    println!(
        "  {name:10}  n={:4}  mean={mean:8.3}  p50={:8.3}  p95={:8.3}  max={:8.3}  (ms)",
        vals.len(),
        pct(&s, 50.0),
        pct(&s, 95.0),
        s.last().copied().unwrap_or(f64::NAN),
    );
}

fn quarters(vals: &[f64]) -> (f64, f64) {
    let q = (vals.len() / 4).max(1);
    let first = vals[..q].iter().sum::<f64>() / q as f64;
    let last = vals[vals.len() - q..].iter().sum::<f64>() / q as f64;
    (first, last)
}

fn main() {
    let out_csv = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "baseline_rust.csv".into());
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden_data");
    let frames = read_frames(dir.join("frames.bin"));
    println!("frames: {}", frames.len());

    let cost_cfg = HeightCostConfig {
        resolution: 0.05,
        can_pass_under: 0.6,
        can_climb: 0.15,
        ignore_noise: 0.05,
        smoothing: 1.0,
    };

    let mut grid = VoxelGrid::default();
    let mut add_ms = Vec::new();
    let mut emit_ms = Vec::new();
    let mut cost_ms = Vec::new();
    let mut csv = String::from("frame,n_points,voxels,add_ms,emit_ms,cost_ms\n");

    let t_start = Instant::now();
    for (i, frame) in frames.iter().enumerate() {
        let n = i + 1;

        let t0 = Instant::now();
        grid.add_frame(frame, VOXEL_SIZE, true);
        let t_add = t0.elapsed().as_secs_f64() * 1e3;
        add_ms.push(t_add);

        let (mut t_emit, mut t_cost) = (f64::NAN, f64::NAN);
        if n % EMIT_EVERY == 0 {
            let t0 = Instant::now();
            let centers = grid.global_points(VOXEL_SIZE);
            let cloud = points_to_cloud(&centers, "world", Time { sec: 0, nsec: 0 });
            t_emit = t0.elapsed().as_secs_f64() * 1e3;
            emit_ms.push(t_emit);
            std::hint::black_box(&cloud);

            let t0 = Instant::now();
            let pts64: Vec<(f64, f64, f64)> = centers
                .iter()
                .map(|&(x, y, z)| (x as f64, y as f64, z as f64))
                .collect();
            let og = height_cost_occupancy(&pts64, &cost_cfg);
            t_cost = t0.elapsed().as_secs_f64() * 1e3;
            cost_ms.push(t_cost);
            std::hint::black_box(&og);
        }

        csv.push_str(&format!(
            "{n},{},{},{t_add:.4},{},{}\n",
            frame.len(),
            grid.len(),
            if t_emit.is_nan() {
                String::new()
            } else {
                format!("{t_emit:.4}")
            },
            if t_cost.is_nan() {
                String::new()
            } else {
                format!("{t_cost:.4}")
            },
        ));
    }
    let wall = t_start.elapsed().as_secs_f64();

    let mut f = fs::File::create(&out_csv).unwrap();
    f.write_all(csv.as_bytes()).unwrap();
    println!("wrote {out_csv}");
    println!("final voxel count: {}", grid.len());
    println!(
        "processed {} frames in {wall:.2}s wall ({:.1} frames/s offline)\n",
        frames.len(),
        frames.len() as f64 / wall
    );

    println!("per-stage latency (ms):");
    summarize("add_frame", &add_ms);
    summarize("emit", &emit_ms);
    summarize("costmap", &cost_ms);

    println!("\ngrowth (first-quarter mean -> last-quarter mean, ms):");
    for (name, vals) in [
        ("add_frame", &add_ms),
        ("emit", &emit_ms),
        ("costmap", &cost_ms),
    ] {
        let (a, b) = quarters(vals);
        println!(
            "  {name:10}  {a:8.3} -> {b:8.3}   ({:.1}x)",
            b / a.max(1e-9)
        );
    }
}
