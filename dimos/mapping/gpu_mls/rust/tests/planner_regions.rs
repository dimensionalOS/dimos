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

//! The live pipeline in one process: the GPU ray tracer's per-region
//! surfaces streamed into the planner's region updates, against one
//! whole-map build from the same GPU map.

mod common;

use ahash::AHashSet;
use dimos_gpu_mls::synthetic::{Lcg, Scene};
use dimos_mls_planner::mls_planner::{Config as PlannerConfig, Planner, RegionBounds};
use dimos_mls_planner::voxel::{voxelize, VoxelKey};
use dimos_voxel_ray_tracing::mapper::{Mapper, SurfaceRequest, SURFACE_CELL_FLOATS};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{Config, Cylinder};

const VOXEL_SIZE: f32 = 0.08;
const FRAMES: usize = 40;
const POINTS_PER_FRAME: usize = 20_000;
/// Cells the streamed surface may differ on from the whole-map build, as a
/// fraction: each region's window sees only the map up to its frame.
const SURFACE_TOLERANCE: f64 = 0.02;
/// Meters the GPU's straight-line boundary distance (streamed) may sit from the
/// CPU's walk along the surface (whole-map) at the median and 95th percentile:
/// the GPU measures within a plane, the CPU around corners.
const BOUNDARY_DELTA_MEDIAN: f32 = 0.1;
const BOUNDARY_DELTA_P95: f32 = 0.4;
/// Fraction of shared cells whose side of the boundary clearance may flip.
const BOUNDARY_CLEARANCE_FLIPS: f64 = 0.02;

fn planner_config(gpu_boundary_distance: bool) -> PlannerConfig {
    PlannerConfig {
        world_frame: String::new(),
        base_frame: String::new(),
        voxel_size: VOXEL_SIZE,
        robot_height: 0.3,
        start_z_offset_m: 0.0,
        max_overhead_m: 2.0,
        surface_closing_radius: 0.3,
        node_spacing_m: 1.0,
        boundary_clearance_m: 0.1,
        boundary_buffer_m: 0.75,
        boundary_buffer_weight: 100.0,
        step_threshold_m: 0.16,
        step_penalty_weight: 4.0,
        goal_tolerance: 0.3,
        viz_publish_hz: 0.0,
        viz_cell_m: 4.0,
        viz_sweep_cells: 0,
        gpu_surfaces: true,
        gpu_boundary_distance,
        max_slope_deg: 45.0,
        surface_change_m: 0.01,
        worker_threads: 8,
    }
}

fn request(cfg: &PlannerConfig) -> SurfaceRequest {
    SurfaceRequest {
        robot_height_m: cfg.robot_height,
        closing_radius_m: cfg.surface_closing_radius,
        step_threshold_m: cfg.step_threshold_m,
        max_slope_deg: cfg.max_slope_deg,
        boundary_distance: cfg.gpu_boundary_distance,
        boundary_reach_m: cfg.boundary_clearance_m + cfg.boundary_buffer_m,
        max_overhead_m: cfg.max_overhead_m,
    }
}

type Cell = (f32, f32, f32, f32, f32);

fn split_cells(cells: &[f32]) -> Vec<Cell> {
    cells
        .chunks_exact(SURFACE_CELL_FLOATS)
        .map(|c| (c[0], c[1], c[2], c[3], c[4]))
        .collect()
}

fn percentile(sorted: &[f32], q: f64) -> f32 {
    sorted[((sorted.len() - 1) as f64 * q).round() as usize]
}

#[test]
fn streamed_gpu_surfaces_match_a_whole_map_build_and_plan() {
    streamed_matches_whole_map(true);
}

#[test]
fn streamed_gpu_surfaces_with_cpu_boundary_distance_match_a_whole_map_build_and_plan() {
    streamed_matches_whole_map(false);
}

fn streamed_matches_whole_map(gpu_boundary_distance: bool) {
    let planner_cfg = planner_config(gpu_boundary_distance);
    let request = request(&planner_cfg);
    let mut mapper = Mapper::new(Config {
        gpu: true,
        support_min: 4,
        max_range: 15.0,
        ..common::cpu_config(VOXEL_SIZE)
    });
    if !mapper.on_gpu() {
        eprintln!("no hardware GPU adapter, skipped");
        return;
    }
    let scene = Scene::room(12.0, 2.5);
    let trajectory = scene.trajectory(FRAMES, 0.4);
    let mut rng = Lcg(11);
    let mut streamed = Planner::new(planner_cfg.worker_threads);
    for &origin in &trajectory {
        let points = scene.scan(origin, POINTS_PER_FRAME, &mut rng, 0.01, 20.0);
        mapper.add_frame_world(points, origin);
        let c = mapper.take_local_bounds();
        let local: Vec<(f32, f32, f32)> = mapper
            .local_points(&c.bounds())
            .chunks_exact(3)
            .map(|p| (p[0], p[1], p[2]))
            .collect();
        let cells = mapper
            .surfaces(&c, &request)
            .expect("a GPU map has surfaces");
        let region = RegionBounds {
            origin_x: c.cx,
            origin_y: c.cy,
            radius: c.radius,
            z_min: c.z_min,
            z_max: c.z_max,
        };
        streamed.update_region_with_surface(&local, &split_cells(&cells), &region, &planner_cfg);
    }

    let everything = Cylinder {
        cx: 6.0,
        cy: 6.0,
        radius: 12.0,
        z_min: -1.0,
        z_max: 4.0,
    };
    mapper.mark_surfaces_dirty(&everything);
    let whole: Vec<Cell> = split_cells(&mapper.surfaces(&everything, &request).expect("surfaces"))
        .into_iter()
        .filter(|c| c.2.is_finite())
        .collect();
    let occupied: Vec<VoxelKey> = mapper
        .local_points(&everything.bounds())
        .chunks_exact(3)
        .map(|p| voxelize((p[0], p[1], p[2]), VOXEL_SIZE))
        .collect();
    let surface: Vec<VoxelKey> = whole
        .iter()
        .map(|&(x, y, z, _, _)| voxelize((x, y, z), VOXEL_SIZE))
        .collect();
    let heights: Vec<f32> = whole.iter().map(|q| q.3).collect();
    let mut built = Planner::new(planner_cfg.worker_threads);
    built.update_from_surface(&occupied, &surface, Some(&heights), None, &planner_cfg);

    let a: AHashSet<VoxelKey> = streamed.surface().collect();
    let b: AHashSet<VoxelKey> = built.surface().collect();
    let differ = a.symmetric_difference(&b).count() as f64 / a.union(&b).count().max(1) as f64;
    eprintln!(
        "streamed {} vs whole-map {} surface cells, {differ:.4} differ",
        a.len(),
        b.len()
    );
    assert!(
        !a.is_empty() && differ <= SURFACE_TOLERANCE,
        "surfaces differ on {differ}"
    );

    let clearance = planner_cfg.boundary_clearance_m;
    let mut deltas: Vec<f32> = Vec::new();
    let (mut flips, mut only_one_finite) = (0usize, 0usize);
    for k in a.intersection(&b) {
        let (Some(gpu), Some(cpu)) = (streamed.boundary_distance(*k), built.boundary_distance(*k))
        else {
            continue;
        };
        match (gpu.is_finite(), cpu.is_finite()) {
            (true, true) => {
                deltas.push((gpu - cpu).abs());
                flips += ((gpu < clearance) != (cpu < clearance)) as usize;
            }
            (false, false) => {}
            _ => only_one_finite += 1,
        }
    }
    deltas.sort_by(f32::total_cmp);
    let shared = a.intersection(&b).count();
    let flip_rate = flips as f64 / shared.max(1) as f64;
    let (median, p95) = (percentile(&deltas, 0.5), percentile(&deltas, 0.95));
    eprintln!(
        "boundary distance gpu vs cpu over {} of {shared} shared cells: |delta| median {median:.3} p95 {p95:.3} max {:.3} m; \
         {flips} flipped sides of the {clearance} m clearance, {only_one_finite} finite on one side only",
        deltas.len(),
        percentile(&deltas, 1.0),
    );
    assert!(!deltas.is_empty(), "no shared boundary distances");
    assert!(
        median <= BOUNDARY_DELTA_MEDIAN && p95 <= BOUNDARY_DELTA_P95,
        "boundary distance median {median} p95 {p95}"
    );
    assert!(
        flip_rate <= BOUNDARY_CLEARANCE_FLIPS,
        "clearance flips {flip_rate}"
    );

    let ground = |(x, y, _): (f32, f32, f32)| (x, y, 0.0);
    let (start, goal) = (ground(trajectory[0]), ground(trajectory[FRAMES / 2]));
    let found: Vec<(&str, Option<usize>)> = [("streamed", &streamed), ("whole-map", &built)]
        .into_iter()
        .map(|(label, planner)| {
            (
                label,
                planner.plan(start, goal, &planner_cfg).map(|p| p.len()),
            )
        })
        .collect();
    eprintln!("paths {start:?} -> {goal:?}: {found:?}");
    assert!(
        found.iter().all(|(_, n)| n.is_some_and(|n| n >= 2)),
        "{found:?}"
    );
}
