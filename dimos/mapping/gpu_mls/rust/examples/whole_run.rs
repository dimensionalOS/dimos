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
use dimos_mls_planner::{surface_cells, wall_distances};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{update_map, Config, VoxelMap};

#[path = "../tests/common/mod.rs"]
#[allow(dead_code)]
mod common;

type Point = (f32, f32, f32);

/// MapStats::overflow's bit for a full brick pool.
const POOL_FULL: u32 = 512;
/// Frames between progress lines.
const REPORT_EVERY: usize = 1000;
/// Surface boxes (x, y, z meters) compared along the trajectory, and how many.
const BOX_M: Point = (20.0, 20.0, 9.0);
const BOXES: usize = 8;
/// surface_parity.rs's bounds on the share of the CPU's cells that differ.
const STANDABLE_DIVERGENCE: f64 = 0.15;
const WALL_DIVERGENCE: f64 = 0.30;
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
    normals: &ahash::AHashMap<VoxelKey, [f32; 3]>,
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
    let cpu_walls: AHashSet<VoxelKey> =
        wall_distances(&occupied, &result.surface, clearance, step, voxel_size)
            .into_iter()
            .filter(|(_, d)| *d == 0.0)
            .map(|(k, _)| k)
            .collect();
    let gpu_walls: AHashSet<VoxelKey> = result.walls.iter().copied().collect();
    let walls = divergence("wall cells", &cpu_walls, &gpu_walls);
    explain_walls(
        &result,
        &cpu_walls,
        &gpu_walls,
        &cpu_standable,
        &gpu_standable,
        step,
        config.step_threshold_m,
    );
    explain_standable(
        &occupied,
        normals,
        &cpu_standable,
        &gpu_standable,
        clearance,
        config.max_slope_deg,
    );
    standable <= STANDABLE_DIVERGENCE && walls <= WALL_DIVERGENCE
}

/// Splits the walls only the GPU has: a neighbor the CPU links (within its
/// whole-voxel step) that the GPU's plane heights put more than the step
/// threshold in meters away, or one the GPU turned down as standable.
#[allow(clippy::too_many_arguments)]
fn explain_walls(
    result: &dimos_gpu_mls::surfaces::SurfaceResult,
    cpu_walls: &AHashSet<VoxelKey>,
    gpu_walls: &AHashSet<VoxelKey>,
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
    let only_gpu: Vec<VoxelKey> = gpu_walls.difference(cpu_walls).copied().collect();
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
        "    of the {} walls only on the gpu: {in_band} ({:.0}%) face a cpu-linked neighbor rising more than {step_m} m, {beside_lost} ({:.0}%) border a cell the gpu turned down as standable, {either} ({:.0}%) either",
        only_gpu.len(),
        share(in_band),
        share(beside_lost),
        share(either)
    );
}

/// Splits the standable cells only the CPU has by the nearest occupied voxel
/// above: just past the CPU's whole-voxel clearance, where the GPU's
/// headroom in meters can fall short, versus the rest.
fn explain_standable(
    occupied: &AHashSet<VoxelKey>,
    normals: &ahash::AHashMap<VoxelKey, [f32; 3]>,
    cpu: &AHashSet<VoxelKey>,
    gpu: &AHashSet<VoxelKey>,
    clearance: i32,
    max_slope_deg: f32,
) {
    let only_cpu: Vec<VoxelKey> = cpu.difference(gpu).copied().collect();
    let min_up = max_slope_deg.to_radians().cos();
    let near_ceiling = |k: &VoxelKey| {
        (clearance + 1..=clearance + 2).any(|dz| occupied.contains(&(k.0, k.1, k.2 + dz)))
    };
    let steep = |k: &VoxelKey| normals.get(k).is_some_and(|n| n[2].abs() < min_up);
    let planeless = |k: &VoxelKey| !normals.contains_key(k);
    let count = |f: &dyn Fn(&VoxelKey) -> bool| only_cpu.iter().filter(|k| f(k)).count();
    let (c_steep, c_planeless, c_ceiling) =
        (count(&steep), count(&planeless), count(&near_ceiling));
    let either = count(&|k| steep(k) || planeless(k) || near_ceiling(k));
    let share = |n: usize| 100.0 * n as f64 / only_cpu.len().max(1) as f64;
    println!(
        "    of the {} standable cells only on the cpu: {c_steep} ({:.0}%) steeper than {max_slope_deg} deg by their voxel's plane, {c_planeless} ({:.0}%) planeless, {c_ceiling} ({:.0}%) a voxel 1 to 2 cells past the cpu's clearance, {either} ({:.0}%) any",
        only_cpu.len(),
        share(c_steep),
        share(c_planeless),
        share(c_ceiling),
        share(either)
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
    let normals: ahash::AHashMap<VoxelKey, [f32; 3]> = snapshot
        .iter()
        .filter(|v| v.health > 0)
        .filter_map(|v| v.normal.map(|n| (v.key, n)))
        .collect();
    drop(snapshot);
    let mut boxes_ok = 0;
    for b in 0..BOXES {
        let center = origins[(b * origins.len() + origins.len() / 2) / BOXES];
        boxes_ok += check_box(&mut map, &healthy, &normals, center, voxel_size) as usize;
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

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let usage = "usage: whole_run <frames> cpu|gpu|parity|control [voxel_size] [max_frames]";
    let path = args.get(1).expect(usage);
    let voxel_size = args
        .get(3)
        .map_or(0.05, |v| v.parse().expect("a voxel size"));
    match args.get(2).map(String::as_str) {
        Some("cpu") => run_cpu(path, voxel_size),
        Some("gpu") => run_gpu(path, voxel_size),
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
