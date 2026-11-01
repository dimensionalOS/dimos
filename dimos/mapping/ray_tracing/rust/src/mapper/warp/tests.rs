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

use super::*;
use crate::reform::{KeyframeRecord, MovedRecord, Pose, ReformParams, ReformState};
use crate::voxel_ray_tracer::{voxel_moves, Config};
use fast_reform::{warp_positions, Quat, Vec3};

const VOXEL: f32 = 0.1;

fn config() -> Config {
    Config {
        voxel_size: VOXEL,
        fine_divisor: 0,
        emit_fine: false,
        max_range: 100.0,
        ray_subsample: 1,
        shadow_depth: 0.0,
        grace_depth: 0.0,
        min_health: 0,
        max_health: 3,
        graze_cos: 0.5,
        support_min: 0,
        emit_every: 0,
        global_emit_every: 0,
        region_percentile: 95.0,
        world_frame: "map".to_string(),
        tf_match_tolerance_s: 0.1,
        max_cloud_age_s: 0.0,
        tf_wait_timeout_s: 0.05,
        worker_threads: 2,
        region_m: 4.0,
        viz_cell_m: 4.0,
        viz_emit_every: 0,
        viz_sweep_cells: 0,
    }
}

fn at(x: f64) -> Pose {
    Pose {
        rotation: Quat::IDENTITY,
        translation: Vec3::new(x, 0.0, 0.0),
    }
}

/// A drive along x, one keyframe a meter and a second, each frame seeing the
/// wall at y = 1.03 beside it, inside its voxels rather than on their edge.
fn driven() -> (Mapper, ReformState) {
    driven_with(config())
}

fn driven_with(config: Config) -> (Mapper, ReformState) {
    let mut mapper = Mapper::new(config);
    mapper.track_voxel_times();
    let mut state = ReformState::default();
    for i in 0..20 {
        mapper.set_clock(i as f64);
        let wall: Vec<(f32, f32, f32)> = (0..20)
            .flat_map(|dx| {
                (0..3).map(move |z| (i as f32 - 0.5 + dx as f32 * 0.05, 1.03, z as f32 * 0.1))
            })
            .collect();
        mapper.add_frame_world(wall, (i as f32, 0.0, 0.1));
        state.add_keyframes(&[KeyframeRecord {
            id: i,
            ts: i as f64,
            pose: at(i as f64),
        }]);
    }
    (mapper, state)
}

/// The second half of the drive pulled 0.5 m to +y.
fn closure(state: &mut ReformState) -> Warp {
    let shift = Pose {
        rotation: Quat::IDENTITY,
        translation: Vec3::new(0.0, 0.5, 0.0),
    };
    let moved: Vec<MovedRecord> = (10..20)
        .map(|i| MovedRecord {
            keyframe: KeyframeRecord {
                id: i,
                ts: i as f64,
                pose: at(i as f64),
            },
            delta: shift,
        })
        .collect();
    let params = ReformParams {
        blend_sigma_m: 1.0,
        blend_time_sigma_s: 1.0,
        min_shift_m: 0.04,
        thin_cell_m: -1.0,
    };
    state
        .closure(&moved, &params)
        .expect("the closure moves past min_shift_m")
}

fn key(x: f32, y: f32, z: f32) -> VoxelKey {
    (
        (x / VOXEL).floor() as i32,
        (y / VOXEL).floor() as i32,
        (z / VOXEL).floor() as i32,
    )
}

/// Whether a voxel holds this key.
fn has(mapper: &Mapper, x: f32, y: f32, z: f32) -> bool {
    mapper.map().voxels.contains_key(&key(x, y, z))
}

/// The keys where the healthy voxels' points sit, re-keyed or not.
fn placed(mapper: &Mapper) -> ahash::AHashSet<VoxelKey> {
    mapper
        .healthy_centers()
        .chunks(3)
        .map(|p| key(p[0], p[1], p[2]))
        .collect()
}

fn sits(mapper: &Mapper, x: f32, y: f32, z: f32) -> bool {
    placed(mapper).contains(&key(x, y, z))
}

#[test]
fn a_closure_moves_the_wall_it_corrected_and_holds_the_rest() {
    let (mut mapper, mut state) = driven();
    assert!(has(&mapper, 15.05, 1.05, 0.05) && has(&mapper, 2.05, 1.05, 0.05));
    mapper.apply_warp(&closure(&mut state));
    assert!(
        sits(&mapper, 15.05, 1.55, 0.05),
        "the corrected wall moved with its keyframes"
    );
    assert!(!sits(&mapper, 15.05, 1.05, 0.05), "and left its old place");
    assert!(
        sits(&mapper, 2.05, 1.05, 0.05),
        "the uncorrected wall stayed"
    );
    // A frame from beside it re-keys it, and its time goes along.
    mapper.add_frame_world(vec![(15.05, 1.55, 0.05)], (15.0, 0.0, 0.1));
    assert!(has(&mapper, 15.05, 1.55, 0.05) && !has(&mapper, 15.05, 1.05, 0.05));
    // Times follow the voxels to their new keys.
    let k = (
        (15.05 / VOXEL).floor() as i32,
        (1.55 / VOXEL).floor() as i32,
        0,
    );
    assert!((mapper.voxel_time(&k) - 15.0).abs() < 1.0);
}

#[test]
fn far_voxels_move_in_place_and_rebin_when_the_local_area_reaches_them() {
    let (mut mapper, mut state) = driven_with(Config {
        max_range: 3.0,
        ..config()
    });
    mapper.apply_warp(&closure(&mut state));
    // The last frame was from x = 19, so the wall at x = 15 is past ray reach:
    // it keeps its keys but is placed where the closure moved it.
    assert!(has(&mapper, 15.05, 1.05, 0.05) && !has(&mapper, 15.05, 1.55, 0.05));
    let placed = mapper.healthy_centers();
    let at = |x: f32, y: f32| {
        placed
            .chunks(3)
            .any(|p| (p[0] - x).abs() < 0.01 && (p[1] - y).abs() < 0.01 && p[2] < 0.1)
    };
    assert!(at(15.05, 1.55) && !at(15.05, 1.05));
    // A frame from beside it, its rays reaching the wall, re-keys it.
    mapper.add_frame_world(vec![(15.05, 1.55, 0.05)], (15.0, 0.0, 0.1));
    assert!(has(&mapper, 15.05, 1.55, 0.05) && !has(&mapper, 15.05, 1.05, 0.05));
}

#[test]
fn voxels_land_where_plain_fast_reform_puts_them() {
    let (mut mapper, mut state) = driven();
    let warp = closure(&mut state);
    let mut keys: Vec<VoxelKey> = mapper.map().voxels.iter().map(|(k, _)| k).collect();
    keys.sort();
    let times: Vec<f64> = keys.iter().map(|k| mapper.voxel_time(k)).collect();
    let means = mapper.map().voxel_means(&keys, VOXEL);
    let reference = warp_positions(&means, Some(&times), &warp.graph_delta().to_blend_arrays());
    let expected = voxel_moves(&keys, &means, &reference, VOXEL, |_| false);
    mapper.apply_warp(&warp);
    let placed = placed(&mapper);
    let landed = expected.iter().filter(|m| placed.contains(&m.to)).count();
    assert!(
        landed as f64 >= 0.99 * expected.len() as f64,
        "{landed} of {} voxels are where fast-reform puts them",
        expected.len()
    );
}

#[test]
fn shifts_smaller_than_a_voxel_add_up_over_closures() {
    let mut mapper = Mapper::new(config());
    mapper.track_voxel_times();
    let mut state = ReformState::default();
    for i in 0..10 {
        mapper.set_clock(i as f64);
        // A wall whose points sit at its voxels' centers, y = 1.05.
        let wall: Vec<(f32, f32, f32)> = (0..10)
            .flat_map(|dx| {
                (0..3).map(move |z| {
                    (
                        i as f32 + 0.05 + dx as f32 * 0.1,
                        1.05,
                        0.05 + z as f32 * 0.1,
                    )
                })
            })
            .collect();
        mapper.add_frame_world(wall, (i as f32, 0.0, 0.1));
        state.add_keyframes(&[KeyframeRecord {
            id: i,
            ts: i as f64,
            pose: at(i as f64),
        }]);
    }
    let params = ReformParams {
        blend_sigma_m: 1.0,
        blend_time_sigma_s: 1.0,
        min_shift_m: 0.02,
        thin_cell_m: -1.0,
    };
    // Each keyframe's pose before the nudge is where the last one left it.
    let nudge = |state: &mut ReformState, done: f64| {
        let moved: Vec<MovedRecord> = (0..10)
            .map(|i| {
                let mut pose = at(i as f64);
                pose.translation.y += 0.03 * done;
                MovedRecord {
                    keyframe: KeyframeRecord {
                        id: i,
                        ts: i as f64,
                        pose,
                    },
                    delta: Pose {
                        rotation: Quat::IDENTITY,
                        translation: Vec3::new(0.0, 0.03, 0.0),
                    },
                }
            })
            .collect();
        state
            .closure(&moved, &params)
            .expect("a 3 cm shift passes min_shift_m")
    };
    // 3 cm keeps the wall in its voxels; another 3 cm takes it past their edge.
    mapper.apply_warp(&nudge(&mut state, 0.0));
    assert!(sits(&mapper, 4.05, 1.05, 0.05) && !sits(&mapper, 4.05, 1.15, 0.05));
    mapper.apply_warp(&nudge(&mut state, 1.0));
    assert!(sits(&mapper, 4.05, 1.15, 0.05) && !sits(&mapper, 4.05, 1.05, 0.05));
}
