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

//! Small worlds where whole voxels misread a surface and the ray tracer's planes read it right.

use super::*;

const VOXEL: f32 = 0.08;
/// Columns along x, and the corridor's width in y.
const LENGTH: i32 = 30;
const WIDTH: i32 = 6;

struct Scene {
    name: &'static str,
    step_threshold_m: f32,
    /// Surface height and slope along x at each column.
    surface: fn(i32) -> (f32, f32),
    /// Voxels over the corridor that are not surface, as (ix from, ix to, iz).
    beam: Option<(i32, i32, i32)>,
    /// Planes cross where whole voxels cannot, else the reverse.
    planes_cross: bool,
}

fn scenes() -> [Scene; 5] {
    [
        Scene {
            name: "A 2 mm rise across a grid line",
            step_threshold_m: 0.05,
            surface: |ix| (if ix < LENGTH / 2 { 0.079 } else { 0.081 }, 0.0),
            beam: None,
            planes_cross: true,
        },
        Scene {
            name: "A 24 cm rise read as 16 cm",
            step_threshold_m: 0.16,
            surface: |ix| (if ix < LENGTH / 2 { 0.001 } else { 0.239 }, 0.0),
            beam: None,
            planes_cross: false,
        },
        Scene {
            name: "A 30 degree ramp",
            step_threshold_m: 0.05,
            surface: |ix| {
                let slope = 30.0_f32.to_radians().tan();
                (0.01 + (ix as f32 + 0.5) * VOXEL * slope, slope)
            },
            beam: None,
            planes_cross: true,
        },
        Scene {
            name: "A floor on a grid line",
            step_threshold_m: 0.05,
            surface: |ix| (if ix % 2 == 0 { 0.0799 } else { 0.0801 }, 0.0),
            beam: None,
            planes_cross: true,
        },
        Scene {
            name: "Headroom under a beam",
            step_threshold_m: 0.16,
            surface: |_| (0.001, 0.0),
            beam: Some((LENGTH / 2 - 2, LENGTH / 2 + 2, 8)),
            planes_cross: true,
        },
    ]
}

fn config(step_threshold_m: f32) -> Config {
    Config {
        world_frame: String::new(),
        base_frame: String::new(),
        voxel_size: VOXEL,
        robot_height: 0.6,
        start_z_offset_m: 0.0,
        max_overhead_m: 2.0,
        surface_closing_radius: 0.0,
        node_spacing_m: 1.0,
        wall_clearance_m: 0.0,
        wall_buffer_m: 0.0,
        wall_buffer_weight: 0.0,
        step_threshold_m,
        max_surface_slope_deg: 45.0,
        step_penalty_weight: 0.0,
        goal_tolerance: 0.3,
        viz_publish_hz: 0.0,
        viz_cell_m: 4.0,
        viz_sweep_cells: 0,
        worker_threads: 2,
    }
}

fn center(ix: i32, iy: i32, iz: i32) -> (f32, f32, f32) {
    (
        (ix as f32 + 0.5) * VOXEL,
        (iy as f32 + 0.5) * VOXEL,
        (iz as f32 + 0.5) * VOXEL,
    )
}

/// The scene's voxels with the planes the tracer fits them.
fn voxels(scene: &Scene) -> Vec<MapPoint> {
    let mut out = Vec::new();
    for ix in 0..LENGTH {
        let (z, gx) = (scene.surface)(ix);
        let iz = (z / VOXEL).floor() as i32;
        let norm = gx.hypot(1.0);
        for iy in 0..WIDTH {
            let c = center(ix, iy, iz);
            out.push((c, [-gx / norm, 0.0, 1.0 / norm, (z - c.2) / norm]));
        }
    }
    if let Some((from, to, iz)) = scene.beam {
        for ix in from..to {
            for iy in 0..WIDTH {
                out.push((center(ix, iy, iz), [0.0, 0.0, 1.0, 0.0]));
            }
        }
    }
    out
}

/// The same voxels with their planes dropped, the whole-voxel model.
fn whole_voxels(points: &[MapPoint]) -> Vec<MapPoint> {
    points.iter().map(|&(p, _)| (p, [f32::NAN; 4])).collect()
}

/// Whether the planner over `points` finds a path end to end along the corridor.
fn crosses(points: &[MapPoint], scene: &Scene) -> bool {
    let cfg = config(scene.step_threshold_m);
    let mut planner = Planner::new(cfg.worker_threads);
    planner.update_global_map(points, &cfg);
    let y = 0.5 * WIDTH as f32 * VOXEL;
    let at = |ix: i32| ((ix as f32 + 0.5) * VOXEL, y, (scene.surface)(ix).0 + 0.01);
    planner.plan(at(0), at(LENGTH - 1), &cfg).is_some()
}

/// The scene at `index` in `scenes` crosses from planes and not from whole voxels, or the reverse.
fn check(index: usize) {
    let scene = &scenes()[index];
    let points = voxels(scene);
    let whole = crosses(&whole_voxels(&points), scene);
    let planes = crosses(&points, scene);
    assert_eq!(
        (whole, planes),
        (!scene.planes_cross, scene.planes_cross),
        "{}",
        scene.name
    );
}

#[test]
fn a_tiny_rise_across_a_grid_line_is_no_step() {
    check(0);
}

#[test]
fn a_tall_rise_read_as_two_voxels_is_still_a_wall() {
    check(1);
}

#[test]
fn a_ramp_is_not_a_staircase() {
    check(2);
}

#[test]
fn a_floor_straddling_a_grid_line_stays_one_level() {
    check(3);
}

#[test]
fn headroom_counts_from_the_surface_not_the_top_face() {
    check(4);
}
