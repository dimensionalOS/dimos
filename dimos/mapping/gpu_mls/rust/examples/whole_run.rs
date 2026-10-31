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

//! Every frame of a recording through one map, streamed from an
//! `export_frames.py` file so the recording never sits in memory.
//! `cargo run --release --example whole_run -- <frames> cpu|gpu [voxel_size]`
//! runs one map per process, so the peak RSS it prints is that map's.
//! `parity` runs both in lockstep, then compares the final maps voxel for
//! voxel and their surfaces on boxes along the trajectory.

use std::fs::File;
use std::io::{BufReader, Read};
use std::time::Instant;

use ahash::AHashSet;
use dimos_gpu_mls::brick_map::{BrickMap, MapConfig, VoxelKey, VoxelSnapshot};
use dimos_gpu_mls::gpu::Gpu;
use dimos_gpu_mls::surfaces::{SurfaceConfig, SurfaceExtractor, Window};
use dimos_mls_planner::mls_planner::{Config as PlannerConfig, Planner};
use dimos_mls_planner::{boundary_distances, surface_cells};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{update_map, Config, VoxelMap};

#[path = "../tests/common/mod.rs"]
#[allow(dead_code)]
mod common;

type Point = (f32, f32, f32);

/// MapStats::overflow's bit for a full brick pool.
const POOL_FULL: u32 = 512;
/// Frames between progress lines.
const REPORT_EVERY: usize = 1000;
/// A frame slower than this is logged, to tie stalls to pool growth.
const SLOW_FRAME_MS: f64 = 100.0;
/// Surface boxes (x, y, z meters) compared along the trajectory, and how many.
const BOX_M: Point = (20.0, 20.0, 9.0);
const BOXES: usize = 8;
/// surface_parity.rs's bounds on the share of the CPU's cells that differ.
const STANDABLE_DIVERGENCE: f64 = 0.15;
const BOUNDARY_DIVERGENCE: f64 = 0.30;
/// map_parity.rs's bounds: mismatched voxels as a share, and moment error.
const MAP_MISMATCH: f64 = 1e-3;
const MOMENT_TOLERANCE: f64 = 1e-4;

/// Yields (origin, points) frames one at a time.
struct Frames(BufReader<File>);

impl Iterator for Frames {
    type Item = (Point, Vec<Point>);

    fn next(&mut self) -> Option<Self::Item> {
        let mut head = [0u8; 16];
        self.0.read_exact(&mut head).ok()?;
        let f32_at = |b: &[u8], i: usize| f32::from_le_bytes(b[i..i + 4].try_into().unwrap());
        let n = u32::from_le_bytes(head[0..4].try_into().unwrap()) as usize;
        let origin = (f32_at(&head, 4), f32_at(&head, 8), f32_at(&head, 12));
        let mut body = vec![0u8; 12 * n];
        self.0.read_exact(&mut body).expect("a whole frame");
        let points = (0..n)
            .map(|k| {
                (
                    f32_at(&body, 12 * k),
                    f32_at(&body, 12 * k + 4),
                    f32_at(&body, 12 * k + 8),
                )
            })
            .collect();
        Some((origin, points))
    }
}

fn open_frames(path: &str) -> Frames {
    Frames(BufReader::with_capacity(
        1 << 22,
        File::open(path).expect("a frames file"),
    ))
}

/// Peak resident set size in MB, from /proc on Linux.
fn peak_rss_mb() -> Option<f64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|l| l.starts_with("VmHWM:"))?;
    let kb: f64 = line.split_whitespace().nth(1)?.parse().ok()?;
    Some(kb / 1024.0)
}

fn cpu_config(voxel_size: f32) -> Config {
    Config {
        voxel_size,
        fine_divisor: 3,
        emit_fine: false,
        max_range: 30.0,
        ray_subsample: 1,
        shadow_depth: 0.1,
        grace_depth: 0.2,
        min_health: -1,
        max_health: 5,
        graze_cos: 0.7,
        graze_recent_frames: u32::MAX,
        gpu: false,
        gpu_max_points: 1 << 18,
        gpu_list_capacity: 1 << 20,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        max_cloud_age_s: 0.0,
        tf_wait_timeout_s: 0.05,
        worker_threads: 8,
        region_m: 4.0,
        viz_cell_m: 4.0,
        viz_emit_every: 0,
        viz_sweep_cells: 0,
    }
}

/// Mean and 99th percentile of per-frame milliseconds.
fn summarize(mut ms: Vec<f64>) -> String {
    if ms.is_empty() {
        return "no frames".to_string();
    }
    let mean = ms.iter().sum::<f64>() / ms.len() as f64;
    ms.sort_by(f64::total_cmp);
    let p99 = ms[(ms.len() * 99 / 100).min(ms.len() - 1)];
    format!(
        "{mean:.2} ms mean, {p99:.2} ms p99, {:.2} ms max",
        ms[ms.len() - 1]
    )
}

fn run_cpu(path: &str, voxel_size: f32) {
    let cfg = cpu_config(voxel_size);
    let mut map = VoxelMap::default();
    let mut times = Vec::new();
    let mut points = 0usize;
    for (i, (origin, pts)) in open_frames(path).enumerate() {
        points += pts.len();
        let t = Instant::now();
        update_map(&mut map, origin, &pts, &cfg);
        times.push(t.elapsed().as_secs_f64() * 1e3);
        if (i + 1) % REPORT_EVERY == 0 {
            println!(
                "frame {}: {} voxels, {} chunks, peak RSS {:.0} MB",
                i + 1,
                map.voxels.len(),
                map.voxels.chunk_count(),
                peak_rss_mb().unwrap_or(f64::NAN)
            );
        }
    }
    let healthy = map.voxels.healthy_len();
    println!(
        "CPU {voxel_size} m: {} frames, {points} points; update_map {}; {} voxels ({healthy} healthy) in {} chunks; peak RSS {:.0} MB",
        times.len(),
        summarize(times),
        map.voxels.len(),
        map.voxels.chunk_count(),
        peak_rss_mb().unwrap_or(f64::NAN)
    );
}

fn gpu_map(gpu: &std::sync::Arc<Gpu>, voxel_size: f32) -> BrickMap {
    let cfg = cpu_config(voxel_size);
    BrickMap::new(
        gpu.clone(),
        MapConfig {
            voxel_size,
            fine_divisor: cfg.fine_divisor,
            max_range: cfg.max_range,
            ray_subsample: 1,
            shadow_depth: cfg.shadow_depth,
            grace_depth: cfg.grace_depth,
            min_health: cfg.min_health,
            max_health: cfg.max_health,
            graze_cos: cfg.graze_cos,
            brick_capacity: BrickMap::max_brick_capacity(&gpu),
            voxel_capacity: BrickMap::max_voxel_capacity(&gpu),
            max_points: 1 << 18,
            list_capacity: 1 << 22,
        },
    )
    .expect("brick map fits the device")
}

fn run_gpu(path: &str, voxel_size: f32) {
    let gpu = Gpu::new().expect("a GPU");
    println!(
        "adapter {} ({:?})",
        gpu.adapter_info.name, gpu.adapter_info.backend
    );
    let mut map = gpu_map(&gpu, voxel_size);
    let mut times = Vec::new();
    let mut points = 0usize;
    let mut overflow = 0u32;
    let mut first_full = None;
    for (i, (origin, pts)) in open_frames(path).enumerate() {
        points += pts.len();
        let t = Instant::now();
        map.integrate(origin, &pts);
        map.finish();
        times.push(t.elapsed().as_secs_f64() * 1e3);
        let stats = map.stats();
        let ms = times[times.len() - 1];
        if ms > SLOW_FRAME_MS {
            println!(
                "slow frame {i}: {ms:.0} ms, {} bricks, {} voxels, map {} MB",
                stats.bricks,
                stats.voxels,
                map.gpu_bytes() >> 20
            );
        }
        overflow |= stats.overflow;
        if stats.overflow & POOL_FULL != 0 && first_full.is_none() {
            first_full = Some(i);
        }
        if (i + 1) % REPORT_EVERY == 0 {
            println!(
                "frame {}: {} bricks, {} voxels, map {} MB, overflow {:#x}, peak RSS {:.0} MB",
                i + 1,
                stats.bricks,
                stats.voxels,
                map.gpu_bytes() >> 20,
                overflow,
                peak_rss_mb().unwrap_or(f64::NAN)
            );
        }
    }
    let stats = map.stats();
    println!(
        "GPU {voxel_size} m: {} frames, {points} points; integrate {}; {} bricks, {} voxels, map {} MB on the GPU; overflow {overflow:#x}{}; peak RSS {:.0} MB",
        times.len(),
        summarize(times),
        stats.bricks,
        stats.voxels,
        map.gpu_bytes() >> 20,
        first_full.map_or(String::new(), |f| format!(" (pool full from frame {f})")),
        peak_rss_mb().unwrap_or(f64::NAN)
    );
}

/// The share of `cpu`'s cells the two sets differ on.
fn divergence(label: &str, cpu: &AHashSet<VoxelKey>, gpu: &AHashSet<VoxelKey>) -> f64 {
    let only_cpu = cpu.difference(gpu).count();
    let only_gpu = gpu.difference(cpu).count();
    let share = (only_cpu + only_gpu) as f64 / cpu.len().max(1) as f64;
    println!(
        "  {label}: cpu {} gpu {} only_cpu {only_cpu} only_gpu {only_gpu} ({:.2}% of the cpu's)",
        cpu.len(),
        gpu.len(),
        100.0 * share
    );
    share
}

/// The surface_parity.rs checks on the final GPU map inside one box: GPU
/// stages against the CPU's on the same occupied voxels. True when within bounds.
fn check_box(
    map: &mut BrickMap,
    healthy: &AHashSet<VoxelKey>,
    center: Point,
    voxel_size: f32,
) -> bool {
    let config = SurfaceConfig::mls_defaults();
    let (clearance, closing, step) = (
        config.clearance_cells(voxel_size),
        config.closing_cells(voxel_size),
        config.step_cells(voxel_size),
    );
    let cell = |c: f32, half: f32| {
        (
            ((c - half) / voxel_size).floor() as i32,
            ((c + half) / voxel_size).ceil() as i32,
        )
    };
    let (x, y, z) = (
        cell(center.0, BOX_M.0 / 2.0),
        cell(center.1, BOX_M.1 / 2.0),
        cell(center.2, BOX_M.2 / 2.0),
    );
    let in_box: Vec<VoxelKey> = healthy
        .iter()
        .copied()
        .filter(|k| {
            (x.0..x.1).contains(&k.0) && (y.0..y.1).contains(&k.1) && (z.0..z.1).contains(&k.2)
        })
        .collect();
    let Some(window) = Window::covering(in_box.iter().copied(), closing + 1) else {
        println!("box at {center:?}: empty");
        return true;
    };
    // The extractor sees every map voxel in the window, so the CPU gets the same.
    let (lo, size) = (window.min, window.size);
    let occupied: AHashSet<VoxelKey> = healthy
        .iter()
        .copied()
        .filter(|k| {
            (0..size.0 as i32).contains(&(k.0 - lo.0))
                && (0..size.1 as i32).contains(&(k.1 - lo.1))
                && (0..size.2 as i32).contains(&(k.2 - lo.2))
        })
        .collect();
    let mut extractor = SurfaceExtractor::new(map, window, config).expect("window fits the device");
    extractor.extract(map);
    map.finish();
    let result = extractor.read();
    println!(
        "box at ({:.1}, {:.1}, {:.1}): window {:?}, {} occupied",
        center.0,
        center.1,
        center.2,
        window.size,
        occupied.len()
    );
    let cpu_standable: AHashSet<VoxelKey> =
        surface_cells(&occupied, clearance, 0).into_iter().collect();
    let gpu_standable: AHashSet<VoxelKey> = result.standable.iter().copied().collect();
    let standable = divergence("standable", &cpu_standable, &gpu_standable);
    let cpu_boundaries: AHashSet<VoxelKey> =
        boundary_distances(&occupied, &result.surface, clearance, step, voxel_size)
            .into_iter()
            .filter(|(_, d)| *d == 0.0)
            .map(|(k, _)| k)
            .collect();
    let gpu_boundaries: AHashSet<VoxelKey> = result.boundaries.iter().copied().collect();
    let boundaries = divergence("boundary cells", &cpu_boundaries, &gpu_boundaries);
    explain_boundaries(
        &result,
        &cpu_boundaries,
        &gpu_boundaries,
        &cpu_standable,
        &gpu_standable,
        step,
        config.step_threshold_m,
    );
    explain_standable(
        &extractor,
        &cpu_standable,
        &gpu_standable,
        config.max_slope_deg,
    );
    standable <= STANDABLE_DIVERGENCE && boundaries <= BOUNDARY_DIVERGENCE
}

/// Splits the boundaries only the GPU has: a neighbor the CPU links (within its
/// whole-voxel step) that the GPU's plane heights put more than the step
/// threshold in meters away, or one the GPU turned down as standable.
#[allow(clippy::too_many_arguments)]
fn explain_boundaries(
    result: &dimos_gpu_mls::surfaces::SurfaceResult,
    cpu_boundaries: &AHashSet<VoxelKey>,
    gpu_boundaries: &AHashSet<VoxelKey>,
    cpu_standable: &AHashSet<VoxelKey>,
    gpu_standable: &AHashSet<VoxelKey>,
    step_cells: i32,
    step_m: f32,
) {
    let height: ahash::AHashMap<VoxelKey, f32> = result
        .surface
        .iter()
        .copied()
        .zip(result.heights.iter().copied())
        .collect();
    let only_gpu: Vec<VoxelKey> = gpu_boundaries.difference(cpu_boundaries).copied().collect();
    let in_band = only_gpu
        .iter()
        .filter(|k| {
            let h = height[k];
            [(1, 0), (-1, 0), (0, 1), (0, -1)].iter().any(|(dx, dy)| {
                (-step_cells..=step_cells).any(|dz| {
                    height
                        .get(&(k.0 + dx, k.1 + dy, k.2 + dz))
                        .is_some_and(|n| (n - h).abs() > step_m)
                })
            })
        })
        .count();
    let neighbors = |k: &VoxelKey| {
        let k = *k;
        [(1, 0), (-1, 0), (0, 1), (0, -1)]
            .into_iter()
            .flat_map(move |(dx, dy)| {
                (-step_cells..=step_cells).map(move |dz| (k.0 + dx, k.1 + dy, k.2 + dz))
            })
    };
    let beside_lost = only_gpu
        .iter()
        .filter(|k| neighbors(k).any(|n| cpu_standable.contains(&n) && !gpu_standable.contains(&n)))
        .count();
    let either = only_gpu
        .iter()
        .filter(|k| {
            let h = height[k];
            neighbors(k).any(|n| {
                (cpu_standable.contains(&n) && !gpu_standable.contains(&n))
                    || height.get(&n).is_some_and(|m| (m - h).abs() > step_m)
            })
        })
        .count();
    let share = |n: usize| 100.0 * n as f64 / only_gpu.len().max(1) as f64;
    println!(
        "    of the {} boundaries only on the gpu: {in_band} ({:.0}%) face a cpu-linked neighbor rising more than {step_m} m, {beside_lost} ({:.0}%) border a cell the gpu turned down as standable, {either} ({:.0}%) either",
        only_gpu.len(),
        share(in_band),
        share(beside_lost),
        share(either)
    );
}

/// Splits the standable cells only the CPU has by the GPU's two reasons to
/// turn a cell down: its fitted plane is too steep, or its headroom in meters
/// is short.
fn explain_standable(
    extractor: &SurfaceExtractor,
    cpu: &AHashSet<VoxelKey>,
    gpu: &AHashSet<VoxelKey>,
    max_slope_deg: f32,
) {
    let only_cpu: Vec<VoxelKey> = cpu.difference(gpu).copied().collect();
    let min_up = max_slope_deg.to_radians().cos();
    let steep = extractor
        .plane_normals(&only_cpu)
        .iter()
        .filter(|n| n.is_some_and(|n| n[2] < min_up))
        .count();
    println!(
        "    of the {} standable cells only on the cpu: {steep} ({:.0}%) have a plane steeper than {max_slope_deg} deg, {} ({:.0}%) are short of headroom in meters",
        only_cpu.len(),
        100.0 * steep as f64 / only_cpu.len().max(1) as f64,
        only_cpu.len() - steep,
        100.0 * (only_cpu.len() - steep) as f64 / only_cpu.len().max(1) as f64
    );
}

fn run_parity(path: &str, voxel_size: f32, max_frames: usize) {
    let gpu = Gpu::new().expect("a GPU");
    println!(
        "adapter {} ({:?})",
        gpu.adapter_info.name, gpu.adapter_info.backend
    );
    let cfg = cpu_config(voxel_size);
    let mut cpu = VoxelMap::default();
    let mut map = gpu_map(&gpu, voxel_size);
    let mut origins = Vec::new();
    let mut overflow = 0;
    for (i, (origin, pts)) in open_frames(path).take(max_frames).enumerate() {
        update_map(&mut cpu, origin, &pts, &cfg);
        map.integrate(origin, &pts);
        overflow |= map.stats().overflow;
        origins.push(origin);
        if (i + 1) % REPORT_EVERY == 0 {
            println!(
                "frame {}: cpu {} voxels, gpu {} voxels",
                i + 1,
                cpu.voxels.len(),
                map.stats().voxels
            );
        }
    }
    map.finish();
    let snapshot = map.snapshot();
    let d = common::diff_maps(&cpu, &snapshot, voxel_size);
    let mismatch = d.mismatched() as f64 / d.cpu_voxels.max(1) as f64;
    println!(
        "map after {} frames: cpu {} gpu {} only_cpu {} only_gpu {} health {} count {} fine {} \
         ({:.4}% mismatched), moment_rel_err {:.2e}, overflow {overflow:#x}",
        origins.len(),
        d.cpu_voxels,
        d.gpu_voxels,
        d.only_cpu.len(),
        d.only_gpu.len(),
        d.health,
        d.count,
        d.fine,
        100.0 * mismatch,
        d.moment_rel_err
    );
    if std::env::var_os("PARITY_DETAIL").is_some() {
        let by_key: ahash::AHashMap<VoxelKey, _> = snapshot.iter().map(|v| (v.key, v)).collect();
        let mut shown = 0;
        for (key, v) in cpu.voxels.iter() {
            let g = by_key.get(&key);
            let (n, _, _) = v.moments();
            if g.is_some_and(|g| g.health == v.health() && g.num_pts == n) {
                continue;
            }
            println!(
                "  {key:?}: cpu health {} pts {n} fine {:#x}; gpu {}",
                v.health(),
                v.fine_mask(),
                g.map_or("absent".to_string(), |g| format!(
                    "health {} pts {} fine {:#x} stale {}",
                    g.health, g.num_pts, g.fine, g.stale
                ))
            );
            shown += 1;
            if shown == 12 {
                break;
            }
        }
    }
    let healthy: AHashSet<VoxelKey> = snapshot
        .iter()
        .filter(|v| v.health > 0)
        .map(|v| v.key)
        .collect();
    drop(snapshot);
    let mut boxes_ok = 0;
    for b in 0..BOXES {
        let center = origins[(b * origins.len() + origins.len() / 2) / BOXES];
        boxes_ok += check_box(&mut map, &healthy, center, voxel_size) as usize;
    }
    let map_ok = mismatch <= MAP_MISMATCH && d.moment_rel_err < MOMENT_TOLERANCE && overflow == 0;
    println!(
        "PARITY {}: map {}, surfaces within bounds in {boxes_ok} of {BOXES} boxes",
        if map_ok && boxes_ok == BOXES {
            "PASS"
        } else {
            "FAIL"
        },
        if map_ok {
            "within bounds"
        } else {
            "out of bounds"
        }
    );
}

/// The noise floor for `parity`'s map diff: the CPU map against itself with
/// every point coordinate moved by one float step.
fn run_control(path: &str, voxel_size: f32, max_frames: usize) {
    let cfg = cpu_config(voxel_size);
    let (mut a, mut b) = (VoxelMap::default(), VoxelMap::default());
    let nudge = |c: f32| f32::from_bits(c.to_bits() + 1);
    let mut frames = 0;
    for (origin, pts) in open_frames(path).take(max_frames) {
        update_map(&mut a, origin, &pts, &cfg);
        let nudged: Vec<Point> = pts
            .iter()
            .map(|p| (nudge(p.0), nudge(p.1), nudge(p.2)))
            .collect();
        update_map(&mut b, origin, &nudged, &cfg);
        frames += 1;
    }
    let snapshot: Vec<VoxelSnapshot> = b
        .voxels
        .iter()
        .map(|(key, v)| {
            let (n, sum, m2) = v.moments();
            VoxelSnapshot {
                key,
                health: v.health(),
                num_pts: n,
                sum: [0, 1, 2].map(|i| sum[i] as f64),
                m2: [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)]
                    .map(|(r, c)| m2[(r, c)] as f64),
                fine: v.fine_mask(),
                stale: false,
                normal: None,
            }
        })
        .collect();
    let d = common::diff_maps(&a, &snapshot, voxel_size);
    println!(
        "control after {frames} frames: cpu {} nudged {} only_cpu {} only_nudged {} health {} count {} fine {} ({:.4}% mismatched)",
        d.cpu_voxels,
        d.gpu_voxels,
        d.only_cpu.len(),
        d.only_gpu.len(),
        d.health,
        d.count,
        d.fine,
        100.0 * d.mismatched() as f64 / d.cpu_voxels.max(1) as f64
    );
}

/// Voxel classes for `export`, in the order the viewer's legend lists them.
const EXPORT_CLASSES: [&str; 10] = [
    "occupied",
    "standable on both",
    "standable only on cpu: plane steeper than the slope limit",
    "standable only on cpu: short of headroom in meters",
    "standable only on gpu",
    "boundary on both",
    "boundary only on gpu: rise over the step threshold",
    "boundary only on gpu: beside a cell the gpu turned down",
    "boundary only on gpu: other",
    "boundary only on cpu",
];

/// One box of a stretch of the recording as JSON for a voxel viewer: every
/// occupied or surface cell with its class in EXPORT_CLASSES.
fn run_export(
    path: &str,
    voxel_size: f32,
    first: usize,
    count: usize,
    center: Point,
    half_m: f32,
    out: &str,
) {
    let gpu = Gpu::new().expect("a GPU");
    let mut map = gpu_map(&gpu, voxel_size);
    for (origin, pts) in open_frames(path).skip(first).take(count) {
        map.integrate(origin, &pts);
    }
    map.finish();
    let snapshot = map.snapshot();
    let config = SurfaceConfig::mls_defaults();
    let (clearance, closing, step) = (
        config.clearance_cells(voxel_size),
        config.closing_cells(voxel_size),
        config.step_cells(voxel_size),
    );
    let cell = |c: f32, half: f32| {
        (
            ((c - half) / voxel_size).floor() as i32,
            ((c + half) / voxel_size).ceil() as i32,
        )
    };
    let (x, y, z) = (
        cell(center.0, half_m),
        cell(center.1, half_m),
        cell(center.2, BOX_M.2 / 2.0),
    );
    let inside = |k: &VoxelKey| {
        (x.0..x.1).contains(&k.0) && (y.0..y.1).contains(&k.1) && (z.0..z.1).contains(&k.2)
    };
    let healthy: Vec<&VoxelSnapshot> = snapshot.iter().filter(|v| v.health > 0).collect();
    let in_box: Vec<VoxelKey> = healthy
        .iter()
        .map(|v| v.key)
        .filter(|k| inside(k))
        .collect();
    let window = Window::covering(in_box.iter().copied(), closing + 1).expect("an occupied box");
    let (lo, size) = (window.min, window.size);
    let in_window = |k: &VoxelKey| {
        (0..size.0 as i32).contains(&(k.0 - lo.0))
            && (0..size.1 as i32).contains(&(k.1 - lo.1))
            && (0..size.2 as i32).contains(&(k.2 - lo.2))
    };
    let occupied: AHashSet<VoxelKey> = healthy
        .iter()
        .map(|v| v.key)
        .filter(|k| in_window(k))
        .collect();
    let mut extractor =
        SurfaceExtractor::new(&map, window, config).expect("window fits the device");
    extractor.extract(&mut map);
    map.finish();
    let result = extractor.read();
    let cpu_standable: AHashSet<VoxelKey> =
        surface_cells(&occupied, clearance, 0).into_iter().collect();
    let gpu_standable: AHashSet<VoxelKey> = result.standable.iter().copied().collect();
    let cpu_boundaries: AHashSet<VoxelKey> =
        boundary_distances(&occupied, &result.surface, clearance, step, voxel_size)
            .into_iter()
            .filter(|(_, d)| *d == 0.0)
            .map(|(k, _)| k)
            .collect();
    let gpu_boundaries: AHashSet<VoxelKey> = result.boundaries.iter().copied().collect();
    let height: ahash::AHashMap<VoxelKey, f32> = result
        .surface
        .iter()
        .copied()
        .zip(result.heights.iter().copied())
        .collect();
    let neighbors = |k: VoxelKey| {
        [(1, 0), (-1, 0), (0, 1), (0, -1)]
            .into_iter()
            .flat_map(move |(dx, dy)| (-step..=step).map(move |dz| (k.0 + dx, k.1 + dy, k.2 + dz)))
    };
    let min_up = config.max_slope_deg.to_radians().cos();
    let only_cpu: Vec<VoxelKey> = cpu_standable.difference(&gpu_standable).copied().collect();
    let steep: AHashSet<VoxelKey> = only_cpu
        .iter()
        .zip(extractor.plane_normals(&only_cpu))
        .filter(|(_, n)| n.is_some_and(|n| n[2] < min_up))
        .map(|(k, _)| *k)
        .collect();
    let class = |k: VoxelKey| -> usize {
        let (cw, gw) = (cpu_boundaries.contains(&k), gpu_boundaries.contains(&k));
        if cw && gw {
            return 5;
        }
        if gw {
            let h = height.get(&k).copied().unwrap_or(f32::NAN);
            if neighbors(k).any(|n| {
                height
                    .get(&n)
                    .is_some_and(|m| (m - h).abs() > config.step_threshold_m)
            }) {
                return 6;
            }
            if neighbors(k).any(|n| cpu_standable.contains(&n) && !gpu_standable.contains(&n)) {
                return 7;
            }
            return 8;
        }
        if cw {
            return 9;
        }
        match (cpu_standable.contains(&k), gpu_standable.contains(&k)) {
            (true, true) => 1,
            (true, false) if steep.contains(&k) => 2,
            (true, false) => 3,
            (false, true) => 4,
            (false, false) => 0,
        }
    };
    let mut cells: Vec<VoxelKey> = occupied
        .iter()
        .copied()
        .chain(result.surface.iter().copied())
        .filter(|k| inside(k))
        .collect();
    cells.sort();
    cells.dedup();
    let mut counts = [0usize; EXPORT_CLASSES.len()];
    let mut flat = Vec::with_capacity(cells.len() * 4);
    for k in &cells {
        let c = class(*k);
        counts[c] += 1;
        flat.extend([k.0 - x.0, k.1 - y.0, k.2 - z.0, c as i32]);
    }
    let json = format!(
        "{{\"voxel_size\":{voxel_size},\"min\":[{},{},{}],\"frames\":[{first},{}],\"classes\":[{}],\"counts\":{:?},\"cells\":{:?}}}",
        x.0,
        y.0,
        z.0,
        first + count,
        EXPORT_CLASSES.map(|c| format!("\"{c}\"")).join(","),
        counts,
        flat
    );
    std::fs::write(out, json).expect("a writable output");
    println!("{} cells to {out}", cells.len());
    for (name, n) in EXPORT_CLASSES.iter().zip(counts) {
        println!("  {name}: {n}");
    }
}

/// `reach`'s surface tiles: the core each tile keeps, and the margin of
/// map it sees around that core so closing and plane fits match an untiled run.
const TILE_M: f32 = 20.0;
const TILE_MARGIN_M: f32 = 1.0;
/// `reach` puts a goal every this many meters along the walked trajectory.
const GOAL_SPACING_M: f32 = 2.0;

/// The MLS module's defaults (mls_planner_native.py) at this voxel size.
fn planner_config(voxel_size: f32) -> PlannerConfig {
    let surface = SurfaceConfig::mls_defaults();
    PlannerConfig {
        world_frame: String::new(),
        base_frame: String::new(),
        voxel_size,
        robot_height: surface.robot_height_m,
        start_z_offset_m: 0.0,
        max_overhead_m: 2.0,
        surface_closing_radius: surface.closing_radius_m,
        node_spacing_m: 1.0,
        boundary_clearance_m: 0.1,
        boundary_buffer_m: 0.75,
        boundary_buffer_weight: 100.0,
        step_threshold_m: surface.step_threshold_m,
        step_penalty_weight: 4.0,
        goal_tolerance: 0.3,
        viz_publish_hz: 0.0,
        viz_cell_m: 4.0,
        viz_sweep_cells: 0,
        gpu_surfaces: false,
        gpu_boundary_distance: false,
        max_slope_deg: surface.max_slope_deg,
        surface_change_m: 0.01,
        worker_threads: 8,
    }
}

/// The GPU surface of the whole map, extracted tile by tile: each cell with
/// its plane height and boundary distance in meters (infinite with no boundary).
fn tiled_surface(
    map: &mut BrickMap,
    healthy: &[VoxelKey],
    voxel_size: f32,
) -> Vec<(VoxelKey, f32, f32)> {
    let tile = (TILE_M / voxel_size) as i32;
    let margin = (TILE_MARGIN_M / voxel_size).ceil() as i32;
    let mut tiles: ahash::AHashMap<(i32, i32), Vec<VoxelKey>> = ahash::AHashMap::new();
    for &k in healthy {
        let (tx, ty) = (k.0.div_euclid(tile), k.1.div_euclid(tile));
        for dx in -1..=1 {
            for dy in -1..=1 {
                let (cx, cy) = (tx + dx, ty + dy);
                let near = |c: i32, v: i32| v >= c * tile - margin && v < (c + 1) * tile + margin;
                if near(cx, k.0) && near(cy, k.1) {
                    tiles.entry((cx, cy)).or_default().push(k);
                }
            }
        }
    }
    let mut out = Vec::new();
    let mut keys: Vec<(i32, i32)> = tiles.keys().copied().collect();
    keys.sort_unstable();
    for t in keys {
        let seen = &tiles[&t];
        let window = Window::covering(seen.iter().copied(), 0).expect("a non-empty tile");
        let mut extractor = SurfaceExtractor::new(map, window, SurfaceConfig::mls_defaults())
            .expect("a tile fits the device");
        extractor.extract(map);
        map.finish();
        let (surface, heights, boundaries) = extractor.read_heights(true);
        let boundaries = boundaries.expect("asked for boundaries");
        let core = |v: i32, c: i32| v.div_euclid(tile) == c;
        for ((k, h), w) in surface.into_iter().zip(heights).zip(boundaries) {
            if core(k.0, t.0) && core(k.1, t.1) {
                out.push((k, h, w.unwrap_or(f32::INFINITY)));
            }
        }
    }
    out
}

/// The top of the nearest occupied voxel within 2 m below `o`, in its own
/// column or, failing that, the 3 x 3 around it.
fn floor_below(o: Point, occupied: &AHashSet<VoxelKey>, voxel_size: f32) -> Option<f32> {
    let (ix, iy) = (
        (o.0 / voxel_size).floor() as i32,
        (o.1 / voxel_size).floor() as i32,
    );
    let iz = (o.2 / voxel_size).floor() as i32;
    let reach = (2.0 / voxel_size) as i32;
    let top = |dx: i32, dy: i32| {
        (1..reach)
            .find(|d| occupied.contains(&(ix + dx, iy + dy, iz - d)))
            .map(|d| (iz - d + 1) as f32 * voxel_size)
    };
    top(0, 0).or_else(|| {
        (-1..=1)
            .flat_map(|dx| (-1..=1).map(move |dy| (dx, dy)))
            .filter_map(|(dx, dy)| top(dx, dy))
            .max_by(f32::total_cmp)
    })
}

/// How far the sensor rides above the floor: the median gap from each origin
/// down to the floor below it.
fn sensor_height(origins: &[Point], occupied: &AHashSet<VoxelKey>, voxel_size: f32) -> f32 {
    let mut gaps: Vec<f32> = origins
        .iter()
        .step_by(10)
        .filter_map(|o| floor_below(*o, occupied, voxel_size).map(|f| o.2 - f))
        .collect();
    gaps.sort_by(f32::total_cmp);
    gaps[gaps.len() / 2]
}

/// The whole recording on the GPU map, then the MLS planner on its healthy
/// voxels twice: with the CPU's whole-voxel surface, and with the GPU's
/// sub-voxel surface and plane heights, and its boundary distances with `gpu-boundaries`. Goals sit along the walked
/// trajectory, so every one should be reachable from the start.
fn run_reach(path: &str, voxel_size: f32, out: &str, gpu_boundary_distance: bool) {
    let gpu = Gpu::new().expect("a GPU");
    let mut map = gpu_map(&gpu, voxel_size);
    let mut origins = Vec::new();
    for (origin, pts) in open_frames(path) {
        map.integrate(origin, &pts);
        origins.push(origin);
    }
    if origins.is_empty() {
        eprintln!("{path} has no frames");
        std::process::exit(1);
    }
    map.finish();
    let healthy: Vec<VoxelKey> = map
        .snapshot()
        .iter()
        .filter(|v| v.health > 0)
        .map(|v| v.key)
        .collect();
    let occupied: AHashSet<VoxelKey> = healthy.iter().copied().collect();
    let config = planner_config(voxel_size);
    let lift = sensor_height(&origins, &occupied, voxel_size);
    println!(
        "{} frames, {} healthy voxels; sensor rides {lift:.2} m above the floor",
        origins.len(),
        healthy.len()
    );

    let started = Instant::now();
    let mut cpu = Planner::new(config.worker_threads);
    let points: Vec<Point> = healthy
        .iter()
        .map(|k| {
            (
                (k.0 as f32 + 0.5) * voxel_size,
                (k.1 as f32 + 0.5) * voxel_size,
                (k.2 as f32 + 0.5) * voxel_size,
            )
        })
        .collect();
    cpu.update_global_map(&points, &config);
    let cpu_build = started.elapsed();

    let started = Instant::now();
    let surface = tiled_surface(&mut map, &healthy, voxel_size);
    let gpu_surface_time = started.elapsed();
    let started = Instant::now();
    let mut gpu_planner = Planner::new(config.worker_threads);
    let cells: Vec<VoxelKey> = surface.iter().map(|s| s.0).collect();
    let heights: Vec<f32> = surface.iter().map(|s| s.1).collect();
    let boundaries: Vec<f32> = surface.iter().map(|s| s.2).collect();
    let gpu_config = PlannerConfig {
        gpu_boundary_distance,
        ..planner_config(voxel_size)
    };
    let gpu_boundaries = Some(boundaries.as_slice()).filter(|_| gpu_boundary_distance);
    gpu_planner.update_from_surface(
        &healthy,
        &cells,
        Some(&heights),
        gpu_boundaries,
        &gpu_config,
    );
    let gpu_graph_time = started.elapsed();
    println!(
        "cpu planner build {:.0} ms ({} surface cells, {} nodes); gpu surface {:.0} ms + graph {:.0} ms ({} surface cells, {} nodes)",
        cpu_build.as_secs_f64() * 1e3,
        cpu.graph().cells.ids().count(),
        cpu.graph().nodes.len(),
        gpu_surface_time.as_secs_f64() * 1e3,
        gpu_graph_time.as_secs_f64() * 1e3,
        cells.len(),
        gpu_planner.graph().nodes.len()
    );

    // Each goal sits on the floor mapped under it, since a handheld sensor
    // rides at no one height. Poses with no floor below (the lidar is blind
    // close in, so the first one often has none) make no goal.
    let floor_at =
        |o: Point| floor_below(o, &occupied, voxel_size).filter(|f| o.2 - f <= 2.0 * lift);
    let ground = |o: Point| {
        let floor = floor_at(o).unwrap_or(o.2 - lift);
        (o.0, o.1, floor + 0.5 * voxel_size)
    };
    let floored: Vec<Point> = origins
        .iter()
        .copied()
        .filter(|o| floor_at(*o).is_some())
        .collect();
    println!(
        "{} of {} poses have a floor mapped below them",
        floored.len(),
        origins.len()
    );
    let mut goals = vec![floored[0]];
    for &o in &floored {
        let last = goals[goals.len() - 1];
        let d = ((o.0 - last.0).powi(2) + (o.1 - last.1).powi(2) + (o.2 - last.2).powi(2)).sqrt();
        if d >= GOAL_SPACING_M {
            goals.push(o);
        }
    }
    let start = ground(goals[0]);
    let mut rows = Vec::new();
    let (mut both, mut only_cpu, mut only_gpu, mut neither) = (0, 0, 0, 0);
    let (mut cpu_plan, mut gpu_plan) = (std::time::Duration::ZERO, std::time::Duration::ZERO);
    let path_json = |p: &Option<Vec<Point>>| match p {
        None => "null".to_string(),
        Some(wp) => format!(
            "[{}]",
            wp.iter()
                .map(|w| format!("[{:.2},{:.2},{:.2}]", w.0, w.1, w.2))
                .collect::<Vec<_>>()
                .join(",")
        ),
    };
    for (i, &g) in goals.iter().enumerate() {
        let goal = ground(g);
        let t = Instant::now();
        let c = cpu.plan(start, goal, &config);
        cpu_plan += t.elapsed();
        let t = Instant::now();
        let p = gpu_planner.plan(start, goal, &gpu_config);
        gpu_plan += t.elapsed();
        match (c.is_some(), p.is_some()) {
            (true, true) => both += 1,
            (true, false) => only_cpu += 1,
            (false, true) => only_gpu += 1,
            (false, false) => neither += 1,
        }
        rows.push(format!(
            "{{\"i\":{i},\"goal\":[{:.2},{:.2},{:.2}],\"cpu\":{},\"gpu\":{}}}",
            goal.0,
            goal.1,
            goal.2,
            path_json(&c),
            path_json(&p)
        ));
    }
    let n = goals.len() as f64;
    println!(
        "REACH from the start to {} goals {GOAL_SPACING_M} m apart along the trajectory: both {both}, only cpu {only_cpu}, only gpu {only_gpu}, neither {neither}; cpu reaches {:.0}%, gpu {:.0}%; mean plan cpu {:.1} ms, gpu {:.1} ms",
        goals.len(),
        100.0 * (both + only_cpu) as f64 / n,
        100.0 * (both + only_gpu) as f64 / n,
        cpu_plan.as_secs_f64() * 1e3 / n,
        gpu_plan.as_secs_f64() * 1e3 / n
    );
    let surface_json = |cells: Vec<VoxelKey>| {
        cells
            .iter()
            .step_by(4)
            .map(|k| format!("[{},{},{}]", k.0, k.1, k.2))
            .collect::<Vec<_>>()
            .join(",")
    };
    std::fs::write(
        out,
        format!(
            "{{\"voxel_size\":{voxel_size},\"sensor_height\":{lift},\"goals\":[{}],\"cpu_surface\":[{}],\"gpu_surface\":[{}]}}",
            rows.join(","),
            surface_json(cpu.surface().collect()),
            surface_json(cells)
        ),
    )
    .expect("writable output");
    println!("wrote {out}");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let usage = "usage: whole_run <frames> cpu|gpu|parity|control [voxel_size] [max_frames], or reach <voxel_size> <out.json> [gpu-boundaries], or export <voxel_size> <first> <count> <x> <y> <z> <half_m> <out.json>";
    let path = args.get(1).expect(usage);
    let voxel_size = args
        .get(3)
        .map_or(0.05, |v| v.parse().expect("a voxel size"));
    match args.get(2).map(String::as_str) {
        Some("cpu") => run_cpu(path, voxel_size),
        Some("gpu") => run_gpu(path, voxel_size),
        Some("export") => {
            let num = |i: usize| -> f32 { args.get(i).expect(usage).parse().expect("a number") };
            run_export(
                path,
                voxel_size,
                num(4) as usize,
                num(5) as usize,
                (num(6), num(7), num(8)),
                num(9),
                args.get(10).expect(usage),
            )
        }
        Some("reach") => run_reach(
            path,
            voxel_size,
            args.get(4).expect(usage),
            args.get(5).is_some_and(|a| a == "gpu-boundaries"),
        ),
        Some("control") => run_control(
            path,
            voxel_size,
            args.get(4)
                .map_or(usize::MAX, |n| n.parse().expect("a frame count")),
        ),
        Some("parity") => run_parity(
            path,
            voxel_size,
            args.get(4)
                .map_or(usize::MAX, |n| n.parse().expect("a frame count")),
        ),
        _ => panic!("{usage}"),
    }
}
