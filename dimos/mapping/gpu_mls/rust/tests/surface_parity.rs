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

//! GPU surface extraction and wall distance against the MLS planner's CPU
//! stages, on a synthetic room and on a real map cloud when it is pulled.
//! The GPU judges headroom, slope and steps from sub-voxel planes, so where
//! the CPU's whole-voxel stages differ these report the divergence and bound
//! it; subvoxel.rs checks those verdicts against ground truth instead.

mod common;

use ahash::{AHashMap, AHashSet};
use common::{cpu_config, gpu, gpu_config, lfs_cloud};
use dimos_gpu_mls::brick_map::{BrickMap, VoxelKey};
use dimos_gpu_mls::surfaces::{SurfaceConfig, SurfaceExtractor, Window};
use dimos_gpu_mls::synthetic::{Lcg, Scene};
use dimos_mls_planner::{surface_cells, wall_distances};

const WALL_CLEARANCE: f32 = 0.1;
/// Largest share of the CPU's standable cells the sub-voxel stages may
/// disagree on: mostly wall tops and other cells the slope gate turns down.
const STANDABLE_DIVERGENCE: f64 = 0.15;
/// ...and of its wall cells: mostly where the CPU links three voxels of rise
/// (15 to 20 cm at 5 cm voxels) that the 16 cm threshold in meters blocks.
const WALL_DIVERGENCE: f64 = 0.30;
/// Meters two float distances may differ by and still agree.
const DISTANCE_TOLERANCE: f32 = 1e-3;
/// Real map named in the report; pulled with get_data.
const REAL_MAP: &str = "go2_hongkong_office_twopass_map.pc2.lcm";

/// The planner defaults, and the CPU's whole-voxel equivalents of them.
struct Cells {
    clearance: i32,
    closing: i32,
    step: i32,
}

fn surface_config(voxel_size: f32) -> (SurfaceConfig, Cells) {
    let config = SurfaceConfig::mls_defaults();
    let cells = Cells {
        clearance: config.clearance_cells(voxel_size),
        closing: config.closing_cells(voxel_size),
        step: config.step_cells(voxel_size),
    };
    (config, cells)
}

/// Report how far two sets differ and bound the difference by a share of the CPU's.
fn expect_divergence(label: &str, cpu: &AHashSet<VoxelKey>, gpu: &AHashSet<VoxelKey>, bound: f64) {
    let (only_cpu, only_gpu) = compare(label, "cpu", cpu, "gpu", gpu);
    let share = (only_cpu + only_gpu) as f64 / cpu.len().max(1) as f64;
    eprintln!("  expected divergence: {:.2}% of the cpu's", 100.0 * share);
    assert!(share <= bound, "{label} diverges on {:.1}%", 100.0 * share);
}

fn synthetic_cloud() -> Vec<(f32, f32, f32)> {
    let scene = Scene::room(12.0, 2.5);
    let mut rng = Lcg(7);
    scene
        .trajectory(12, 0.4)
        .into_iter()
        .flat_map(|o| scene.scan(o, 40_000, &mut rng, 0.01, 30.0))
        .collect()
}

/// Seed `cloud` on the GPU and run the surface stages over the map's box.
fn extract(cloud: &[(f32, f32, f32)], voxel_size: f32) -> (AHashSet<VoxelKey>, SurfaceExtractor) {
    let cfg = cpu_config(voxel_size);
    let mut map = BrickMap::new(gpu(), gpu_config(&cfg, 1 << 16, cloud.len() as u32))
        .expect("brick map fits the device");
    map.seed(cloud);
    let occupied: AHashSet<VoxelKey> = map
        .snapshot()
        .into_iter()
        .filter(|v| v.health > 0)
        .map(|v| v.key)
        .collect();
    let (config, cells) = surface_config(voxel_size);
    let window =
        Window::covering(occupied.iter().copied(), cells.closing + 1).expect("an occupied map");
    let mut surfaces = SurfaceExtractor::new(&map, window, config).expect("window fits the device");
    surfaces.extract(&mut map);
    map.finish();
    (occupied, surfaces)
}

/// Exact Euclidean closing of each z slice of `standable`, then the CPU's
/// support filter on the added cells; headroom is left to the GPU's meters.
fn euclidean_closing(
    standable: &[VoxelKey],
    occupied: &AHashSet<VoxelKey>,
    n: i32,
) -> AHashSet<VoxelKey> {
    let disk: Vec<(i32, i32)> = (-n..=n)
        .flat_map(|dx| (-n..=n).map(move |dy| (dx, dy)))
        .filter(|(dx, dy)| dx * dx + dy * dy <= n * n)
        .collect();
    let mut by_z: AHashMap<i32, AHashSet<(i32, i32)>> = AHashMap::new();
    for &(x, y, z) in standable {
        by_z.entry(z).or_default().insert((x, y));
    }
    let mut columns: AHashMap<(i32, i32), Vec<i32>> = AHashMap::new();
    for &(x, y, z) in occupied {
        columns.entry((x, y)).or_default().push(z);
    }
    let supported = |x: i32, y: i32, z: i32| {
        (-3..=3).any(|dx| {
            (-3..=3).any(|dy| {
                columns
                    .get(&(x + dx, y + dy))
                    .is_some_and(|zs| zs.iter().any(|&o| (o - z).abs() <= 3))
            })
        })
    };
    let mut out: AHashSet<VoxelKey> = standable.iter().copied().collect();
    for (z, cells) in by_z {
        let grown: AHashSet<(i32, i32)> = cells
            .iter()
            .flat_map(|&(x, y)| disk.iter().map(move |&(dx, dy)| (x + dx, y + dy)))
            .collect();
        for &(x, y) in &grown {
            if cells.contains(&(x, y)) {
                continue;
            }
            let closed = disk
                .iter()
                .all(|&(dx, dy)| grown.contains(&(x + dx, y + dy)));
            if closed && supported(x, y, z) {
                out.insert((x, y, z));
            }
        }
    }
    out
}

fn compare(
    label: &str,
    a_name: &str,
    a: &AHashSet<VoxelKey>,
    b_name: &str,
    b: &AHashSet<VoxelKey>,
) -> (usize, usize) {
    let only_a = a.difference(b).count();
    let only_b = b.difference(a).count();
    eprintln!(
        "{label}: {a_name} {} {b_name} {} shared {} only_{a_name} {only_a} only_{b_name} {only_b}",
        a.len(),
        b.len(),
        a.intersection(b).count()
    );
    (only_a, only_b)
}

fn percentile(sorted: &[f32], p: f32) -> f32 {
    if sorted.is_empty() {
        return f32::NAN;
    }
    sorted[((sorted.len() - 1) as f32 * p).round() as usize]
}

fn check_surfaces(label: &str, cloud: &[(f32, f32, f32)], voxel_size: f32) {
    let (config, cells) = surface_config(voxel_size);
    let (occupied, mut surfaces) = extract(cloud, voxel_size);
    let gpu = surfaces.read();
    eprintln!(
        "[{label} @ {voxel_size}] window {:?}, {} occupied, {config:?}",
        surfaces.window,
        occupied.len()
    );

    let cpu_standable: AHashSet<VoxelKey> = surface_cells(&occupied, cells.clearance, 0)
        .into_iter()
        .collect();
    let gpu_standable: AHashSet<VoxelKey> = gpu.standable.iter().copied().collect();
    expect_divergence(
        "standable",
        &cpu_standable,
        &gpu_standable,
        STANDABLE_DIVERGENCE,
    );

    let gpu_surface: AHashSet<VoxelKey> = gpu.surface.iter().copied().collect();
    let reference = euclidean_closing(&gpu.standable, &occupied, cells.closing);
    let (a, b) = compare(
        "closing vs exact euclidean",
        "exact",
        &reference,
        "gpu",
        &gpu_surface,
    );
    eprintln!("  filled cells the meter headroom turned down: {a}");
    assert_eq!(b, 0, "the round closing adds nothing beyond the exact one");
    let cpu_surface: AHashSet<VoxelKey> =
        surface_cells(&occupied, cells.clearance, cells.closing as u32)
            .into_iter()
            .collect();
    compare(
        "closing vs cpu diamond",
        "cpu",
        &cpu_surface,
        "gpu",
        &gpu_surface,
    );
    let filled_cpu = cpu_surface.len() - cpu_standable.len();
    let filled_gpu = gpu_surface.len() - gpu_standable.len();
    eprintln!("  filled cells: cpu {filled_cpu} gpu {filled_gpu}");

    // Wall distance on the GPU's surface, so only the distance itself differs.
    let surface_list: Vec<VoxelKey> = gpu.surface.clone();
    let cpu_dist: AHashMap<VoxelKey, f32> = wall_distances(
        &occupied,
        &surface_list,
        cells.clearance,
        cells.step,
        voxel_size,
    )
    .into_iter()
    .collect();
    let cpu_walls: AHashSet<VoxelKey> = cpu_dist
        .iter()
        .filter(|(_, d)| **d == 0.0)
        .map(|(k, _)| *k)
        .collect();
    let gpu_walls: AHashSet<VoxelKey> = gpu.walls.iter().copied().collect();
    expect_divergence("wall cells", &cpu_walls, &gpu_walls, WALL_DIVERGENCE);

    let gpu_dist = surfaces.wall_distances();
    let stats = surfaces.plane_stats;
    let mut ratio = Vec::new();
    let mut excess = Vec::new();
    let (mut only_cpu, mut only_gpu, mut both_none, mut clearance_flips) = (0, 0, 0, 0);
    for &(k, g) in &gpu_dist {
        let c = cpu_dist.get(&k).copied().filter(|d| d.is_finite());
        match (c, g) {
            (Some(c), Some(g)) => {
                if c > 0.0 {
                    ratio.push(g / c);
                }
                excess.push(c - g);
                clearance_flips += ((c < WALL_CLEARANCE) != (g < WALL_CLEARANCE)) as usize;
            }
            (Some(_), None) => only_cpu += 1,
            (None, Some(_)) => only_gpu += 1,
            (None, None) => both_none += 1,
        }
    }
    ratio.sort_by(f32::total_cmp);
    excess.sort_by(f32::total_cmp);
    eprintln!(
        "  wall distance over {} cells, {} planes ({} plane cells): gpu/cpu ratio p5 {:.3} p50 {:.3} p95 {:.3} max {:.3}; \
         cpu-gpu meters p50 {:.3} p95 {:.3} max {:.3}; finite only on cpu {only_cpu}, only on gpu {only_gpu}, neither {both_none}; \
         sides of the {WALL_CLEARANCE} m clearance flipped {clearance_flips}",
        gpu_dist.len(),
        stats.planes,
        stats.plane_cells,
        percentile(&ratio, 0.05),
        percentile(&ratio, 0.5),
        percentile(&ratio, 0.95),
        percentile(&ratio, 1.0),
        percentile(&excess, 0.5),
        percentile(&excess, 0.95),
        percentile(&excess, 1.0),
    );
    // Jump flooding against an exact nearest-wall search per plane, on a sample.
    let planes: AHashMap<VoxelKey, u32> = surfaces.plane_ids().into_iter().collect();
    let components = host_components(&gpu.surface, cells.step);
    let mut planes_per_component: AHashMap<u32, AHashSet<u32>> = AHashMap::new();
    for (k, c) in &components {
        planes_per_component
            .entry(*c)
            .or_default()
            .insert(planes[k]);
    }
    let split = planes_per_component
        .values()
        .filter(|p| p.len() > 1)
        .count();
    eprintln!(
        "  {} connected surfaces on the host, {} split into more than one plane",
        planes_per_component.len(),
        split
    );
    let mut walls_by_plane: AHashMap<u32, Vec<VoxelKey>> = AHashMap::new();
    for &w in &gpu.walls {
        walls_by_plane.entry(planes[&w]).or_default().push(w);
    }
    let mut jfa_errors = 0;
    let sample: Vec<&(VoxelKey, Option<f32>)> = gpu_dist
        .iter()
        .step_by((gpu_dist.len() / 2000).max(1))
        .collect();
    for &&(k, g) in &sample {
        let exact = walls_by_plane.get(&planes[&k]).map(|ws| {
            ws.iter()
                .map(|w| (((w.0 - k.0).pow(2) + (w.1 - k.1).pow(2)) as f32).sqrt() * voxel_size)
                .fold(f32::INFINITY, f32::min)
        });
        let exact = exact.map(|e| {
            if gpu_walls.contains(&k) {
                0.0
            } else {
                e.max(voxel_size)
            }
        });
        let differs = match (exact, g) {
            (Some(e), Some(g)) => (e - g).abs() > DISTANCE_TOLERANCE,
            (e, g) => e.is_some() != g.is_some(),
        };
        if differs {
            jfa_errors += 1;
        }
    }
    eprintln!(
        "  jump flood vs exact per-plane nearest wall: {jfa_errors} of {} sampled cells differ",
        sample.len()
    );
    assert!(jfa_errors as f64 <= 0.01 * sample.len() as f64);
    // Straight lines never exceed walked paths, except across the seam of a split plane.
    let longer = excess
        .iter()
        .filter(|e| **e < -voxel_size - DISTANCE_TOLERANCE)
        .count();
    eprintln!("  gpu longer than cpu by more than a voxel: {longer} cells");
    assert!(longer as f64 <= 0.01 * excess.len() as f64);
}

/// Connected surfaces by breadth-first search over the CPU link rule.
fn host_components(surface: &[VoxelKey], step_cells: i32) -> AHashMap<VoxelKey, u32> {
    let cells: AHashSet<VoxelKey> = surface.iter().copied().collect();
    let mut label: AHashMap<VoxelKey, u32> = AHashMap::new();
    let mut next = 0;
    for &start in surface {
        if label.contains_key(&start) {
            continue;
        }
        let mut queue = vec![start];
        label.insert(start, next);
        while let Some((x, y, z)) = queue.pop() {
            for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                for dz in -step_cells..=step_cells {
                    let n = (x + dx, y + dy, z + dz);
                    if cells.contains(&n) && !label.contains_key(&n) {
                        label.insert(n, next);
                        queue.push(n);
                    }
                }
            }
        }
        next += 1;
    }
    label
}

#[test]
fn synthetic_room_surfaces_match_cpu() {
    let cloud = synthetic_cloud();
    for voxel_size in [0.05, 0.08] {
        check_surfaces("synthetic", &cloud, voxel_size);
    }
}

#[test]
fn real_map_surfaces_match_cpu() {
    let Some(cloud) = lfs_cloud(REAL_MAP) else {
        eprintln!(
            "skipping: {} is missing; set DIMOS_DATA_DIR",
            dimos_gpu_mls::data_dir().join(REAL_MAP).display()
        );
        return;
    };
    for voxel_size in [0.05, 0.08] {
        check_surfaces(REAL_MAP, &cloud, voxel_size);
    }
}
