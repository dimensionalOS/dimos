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

//! Replays dumped lidar scans through the Mapper the way the module does and times each frame.
//! Usage: replay_bench <scans.bin> <pre_trace_subsample> <ray_subsample> <every> (replays every Nth scan)
//! scans.bin: u32 count, then per scan 7 f32 pose (x y z qx qy qz qw), u32 n, n*3 f32 xyz.
use dimos_voxel_ray_tracing::mapper::{Mapper, Point, Pose};
use dimos_voxel_ray_tracing::voxel_ray_tracer::Config;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let bytes = std::fs::read(&args[1]).unwrap();
    let arg = |i: usize| args[i].parse::<u32>().unwrap();
    let (pre, ray, every) = (arg(2), arg(3), arg(4));
    let mut at = 0;
    let mut f32_at = |at: &mut usize| {
        let v = f32::from_le_bytes(bytes[*at..*at + 4].try_into().unwrap());
        *at += 4;
        v
    };
    let count = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
    at += 4;
    let mut scans: Vec<(Pose, Vec<Point>)> = Vec::new();
    for _ in 0..count {
        let p: Vec<f32> = (0..7).map(|_| f32_at(&mut at)).collect();
        let n = u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()) as usize;
        at += 4;
        let points = (0..n)
            .map(|_| (f32_at(&mut at), f32_at(&mut at), f32_at(&mut at)))
            .collect();
        scans.push((
            Pose {
                position: (p[0], p[1], p[2]),
                orientation: (p[3], p[4], p[5], p[6]),
            },
            points,
        ));
    }
    let mut mapper = Mapper::new(Config {
        voxel_size: 0.08,
        fine_divisor: 3,
        emit_fine: false,
        max_range: 25.0,
        pre_trace_subsample: pre,
        ray_subsample: ray,
        shadow_depth: 0.1,
        grace_depth: 0.2,
        min_health: -1,
        max_health: 5,
        graze_cos: 0.7,
        graze_recent_frames: u32::MAX,
        support_min: 4,
        emit_every: 1,
        global_emit_every: 25,
        region_percentile: 95.0,
        world_frame: "map".to_string(),
        tf_match_tolerance_s: 0.1,
        max_cloud_age_s: 0.0,
        tf_wait_timeout_s: 0.05,
        worker_threads: 3,
        region_m: 4.0,
        viz_cell_m: 4.0,
        viz_emit_every: 0,
        viz_sweep_cells: 32,
    });
    let mut frame_ms: Vec<f64> = Vec::new();
    let mut local_points = 0;
    let wall = Instant::now();
    for (i, (pose, points)) in scans.into_iter().enumerate() {
        if i as u32 % every.max(1) != 0 {
            continue;
        }
        let start = Instant::now();
        mapper.add_frame(points, pose);
        if mapper.local_due() {
            let cylinder = mapper.take_local_bounds().bounds();
            local_points = mapper.local_points(&cylinder).len() / 3;
        }
        if mapper.global_due() {
            let _ = mapper.global_points();
        }
        frame_ms.push(start.elapsed().as_secs_f64() * 1e3);
    }
    let total_s = wall.elapsed().as_secs_f64();
    frame_ms.sort_by(|a, b| a.total_cmp(b));
    let pct = |q: f64| frame_ms[((frame_ms.len() - 1) as f64 * q) as usize];
    let mean = frame_ms.iter().sum::<f64>() / frame_ms.len() as f64;
    println!(
        "pre={pre} ray={ray} every={every} frames={} mean_ms={mean:.1} p50={:.1} p95={:.1} max={:.1} total_s={total_s:.2} last_local_points={local_points}",
        frame_ms.len(), pct(0.5), pct(0.95), pct(1.0)
    );
}
