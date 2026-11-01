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

//! The warp (fast-reform with its far-anchor cutoff) against brute-force fast-reform over every
//! keyframe, on real closures from `pgo-eval --event-dump`.
//!
//! MLS_REFORM_EVENTS=<events.bin> cargo test --release --test reform_against_fast_reform -- --ignored --nocapture

use dimos_mls_planner::reform::{KeyframeRecord, MovedRecord, Pose, ReformParams, ReformState};
use fast_reform::{warp_positions_brute_force, Quat, Vec3};

const RECORD: usize = 96;

fn f32_at(b: &[u8], o: usize) -> f64 {
    f32::from_le_bytes(b[o..o + 4].try_into().unwrap()) as f64
}

fn pose_at(b: &[u8], o: usize) -> Pose {
    Pose {
        translation: Vec3::new(f32_at(b, o), f32_at(b, o + 4), f32_at(b, o + 8)),
        rotation: Quat::new(
            f32_at(b, o + 12),
            f32_at(b, o + 16),
            f32_at(b, o + 20),
            f32_at(b, o + 24),
        )
        .normalized_or_identity(),
    }
}

/// Per closure: (keyframes at their pre pose, the moved ones with their delta).
fn closures(raw: &[u8]) -> Vec<(Vec<KeyframeRecord>, Vec<MovedRecord>)> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < raw.len() {
        let n = u32::from_le_bytes(raw[at + 8..at + 12].try_into().unwrap()) as usize;
        at += 12;
        let mut keyframes = Vec::with_capacity(n);
        let mut moved = Vec::new();
        for i in 0..n {
            let r = &raw[at + i * RECORD..at + (i + 1) * RECORD];
            let id = u32::from_le_bytes(r[0..4].try_into().unwrap());
            let ts = f64::from_le_bytes(r[4..12].try_into().unwrap());
            let (pre, post) = (pose_at(r, 12), pose_at(r, 40));
            let keyframe = KeyframeRecord { id, ts, pose: pre };
            keyframes.push(keyframe);
            if r[12..40] != r[40..68] {
                moved.push(MovedRecord {
                    keyframe,
                    delta: post.compose(&pre.inverse()),
                });
            }
        }
        at += n * RECORD;
        out.push((keyframes, moved));
    }
    out
}

#[test]
#[ignore = "needs MLS_REFORM_EVENTS, a pgo-eval --event-dump file"]
fn the_warp_matches_brute_force_fast_reform_over_every_keyframe() {
    let path = std::env::var("MLS_REFORM_EVENTS").expect("MLS_REFORM_EVENTS");
    let raw = std::fs::read(path).unwrap();
    let params = ReformParams {
        blend_sigma_m: 0.0,
        blend_time_sigma_s: 5.0,
        min_shift_m: 0.04,
        thin_cell_m: -1.0,
    };
    let mut state = ReformState::default();
    let mut worst = 0.0_f64;
    let mut checked = 0;
    for (k, (keyframes, moved)) in closures(&raw).into_iter().enumerate() {
        state.add_keyframes(&keyframes);
        let Some(warp) = state.closure(&moved, &params) else {
            continue;
        };
        if warp.moved_keyframes() < 500 {
            continue;
        }
        // Points around the drive: each keyframe's place, pushed up to 4 m
        // off it, seen up to 3 s after it.
        let mut points = Vec::new();
        let mut times = Vec::new();
        for (i, kf) in keyframes.iter().enumerate().step_by(3) {
            let a = i as f64 * 0.618;
            let r = (i % 9) as f64 * 0.5;
            let p = kf.pose.translation;
            points.push([
                (p.x + r * a.cos()) as f32,
                (p.y + r * a.sin()) as f32,
                (p.z - 0.5) as f32,
            ]);
            times.push(kf.ts + (i % 4) as f64);
        }
        let warped = warp.apply(&points, &times);
        let reference = warp_positions_brute_force(
            &points,
            Some(&times),
            &warp.graph_delta().to_blend_arrays(),
        );
        let mut errors: Vec<f64> = warped
            .iter()
            .zip(&reference)
            .map(|(a, b)| {
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
                    as f64
            })
            .collect();
        errors.sort_by(f64::total_cmp);
        let moved_by: Vec<f64> = points
            .iter()
            .zip(&reference)
            .map(|(a, b)| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt() as f64)
            .collect();
        let p99 = errors[errors.len() * 99 / 100];
        let max = *errors.last().unwrap();
        println!(
            "closure {k}: {} keyframes moved, sigma {:.2} m, points {}, max move {:.3} m, \
             warp vs brute force: p99 {:.4} m, max {:.4} m",
            warp.moved_keyframes(),
            warp.sigma(),
            points.len(),
            moved_by.iter().cloned().fold(0.0, f64::max),
            p99,
            max,
        );
        worst = worst.max(p99);
        checked += 1;
    }
    assert!(checked > 0, "no big closure in the dump");
    assert!(
        worst < 0.02,
        "the warp is {worst} m off brute-force fast-reform at p99"
    );
}
