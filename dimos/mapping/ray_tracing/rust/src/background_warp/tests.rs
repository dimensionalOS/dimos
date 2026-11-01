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
use crate::reform::{self, KeyframeRecord, MovedRecord, ReformParams, ReformState};
use crate::voxel_ray_tracer::Config;
use fast_reform::{Quat, Vec3};

fn config() -> Config {
    Config {
        voxel_size: 0.1,
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

/// Scan `i` of a drive along x: the wall at y = 1.03 beside it, in the sensor frame.
fn scan(i: u32) -> (f64, Vec<Point>, Pose) {
    let position = (i as f32, 0.0, 0.1);
    let points = (0..20)
        .flat_map(|dx| (0..3).map(move |z| (-0.5 + dx as f32 * 0.05, 1.03, z as f32 * 0.1 - 0.1)))
        .collect();
    let pose = Pose {
        position,
        orientation: (0.0, 0.0, 0.0, 1.0),
    };
    (i as f64, points, pose)
}

fn keyframe(i: u32) -> KeyframeRecord {
    KeyframeRecord {
        id: i,
        ts: i as f64,
        pose: reform::Pose {
            rotation: Quat::IDENTITY,
            translation: Vec3::new(i as f64, 0.0, 0.0),
        },
    }
}

/// Keyframes `moved` pulled by `shift` (m, along y).
fn closure(state: &mut ReformState, moved: std::ops::Range<u32>, shift: f64) -> Warp {
    let delta = reform::Pose {
        rotation: Quat::IDENTITY,
        translation: Vec3::new(0.0, shift, 0.0),
    };
    let moved: Vec<MovedRecord> = moved
        .map(|i| MovedRecord {
            keyframe: keyframe(i),
            delta,
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

fn mapper_with_keyframes(scans: std::ops::Range<u32>) -> (Mapper, ReformState) {
    let mut mapper = Mapper::new(config());
    mapper.track_voxel_times();
    let mut state = ReformState::default();
    for i in scans {
        add(&mut mapper, i);
        state.add_keyframes(&[keyframe(i)]);
    }
    (mapper, state)
}

fn add(mapper: &mut Mapper, i: u32) {
    let (stamp, points, pose) = scan(i);
    mapper.set_clock(stamp);
    mapper.add_frame(points, pose);
}

/// A scan landing while a warp may be running, as the module feeds it.
fn add_live(background: &mut BackgroundWarp, mapper: &mut Mapper, i: u32) {
    let (stamp, points, pose) = scan(i);
    background.on_frame(stamp, &points, pose);
    mapper.set_clock(stamp);
    mapper.add_frame(points, pose);
}

fn sorted_centers(mapper: &Mapper) -> Vec<[f32; 3]> {
    let mut centers: Vec<[f32; 3]> = mapper
        .healthy_centers()
        .chunks(3)
        .map(|p| [p[0], p[1], p[2]])
        .collect();
    centers.sort_by(|a, b| a.partial_cmp(b).unwrap());
    centers
}

#[test]
fn a_background_warp_gives_the_map_an_in_place_one_does() {
    let (mut in_place, mut state) = mapper_with_keyframes(0..20);
    in_place.apply_warp(&closure(&mut state, 10..20, 0.5));
    for i in 20..26 {
        add(&mut in_place, i);
    }

    let (mut live, mut state) = mapper_with_keyframes(0..20);
    let mut background = BackgroundWarp::default();
    let mut job = background
        .on_warp(closure(&mut state, 10..20, 0.5), &live)
        .expect("nothing is running, so the warp starts");
    for i in 20..26 {
        add_live(&mut background, &mut live, i);
    }
    job.map.apply_warp(&job.warp);
    assert!(background.on_done(job.epoch, job.map, &mut live).is_none());

    assert_eq!(sorted_centers(&live), sorted_centers(&in_place));
}

#[test]
fn a_warp_that_arrives_mid_warp_waits_and_no_scan_is_warped_twice() {
    let (mut in_place, mut state) = mapper_with_keyframes(0..20);
    in_place.apply_warp(&closure(&mut state, 10..20, 0.5));
    for i in 20..23 {
        add(&mut in_place, i);
    }
    in_place.apply_warp(&closure(&mut state, 15..20, 0.3));
    for i in 23..26 {
        add(&mut in_place, i);
    }

    let (mut live, mut state) = mapper_with_keyframes(0..20);
    let mut background = BackgroundWarp::default();
    let mut first = background
        .on_warp(closure(&mut state, 10..20, 0.5), &live)
        .expect("nothing is running, so the warp starts");
    for i in 20..23 {
        add_live(&mut background, &mut live, i);
    }
    assert!(background
        .on_warp(closure(&mut state, 15..20, 0.3), &live)
        .is_none());
    for i in 23..26 {
        add_live(&mut background, &mut live, i);
    }
    first.map.apply_warp(&first.warp);
    let mut second = background
        .on_done(first.epoch, first.map, &mut live)
        .expect("the waiting warp starts once the first is swapped in");
    second.map.apply_warp(&second.warp);
    assert!(background
        .on_done(second.epoch, second.map, &mut live)
        .is_none());

    assert_eq!(sorted_centers(&live), sorted_centers(&in_place));
}

/// How long the worker holds still to copy a full-size map for a background warp.
/// `cargo test --release clone_time -- --ignored --nocapture`
#[test]
#[ignore]
fn clone_time_at_full_map_size() {
    let mut mapper = Mapper::new(config());
    mapper.track_voxel_times();
    // ~7M voxels, the size of the huge_loop_go2 map.
    let side = 1900;
    let points: Vec<Point> = (0..side * side)
        .flat_map(|i| {
            let (x, y) = ((i % side) as f32 * 0.1, (i / side) as f32 * 0.1);
            [(x, y, 0.05), (x, y, 0.15)]
        })
        .collect();
    let created = mapper.seed_points(&points);
    let start = std::time::Instant::now();
    let copy = mapper.clone();
    let clone_ms = start.elapsed().as_secs_f64() * 1e3;
    drop(copy);
    let mut state = ReformState::default();
    for i in 0..20 {
        state.add_keyframes(&[keyframe(i)]);
    }
    let warp = closure(&mut state, 10..20, 0.5);
    let start = std::time::Instant::now();
    let report = mapper.apply_warp(&warp);
    let warp_ms = start.elapsed().as_secs_f64() * 1e3;
    println!(
        "voxels {created}, clone {clone_ms:.0} ms, in-place warp {warp_ms:.0} ms ({} warped)",
        report.voxels_warped
    );
}
