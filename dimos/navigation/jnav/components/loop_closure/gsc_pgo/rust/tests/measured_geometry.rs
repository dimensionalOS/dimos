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

//! The measured closure geometry on synthetic data: the half-normal width fit and the inlier distance measured from the
//! odometry (`closure_inlier_distance_from_odometry`).

use std::sync::Arc;

use dimos_gsc_pgo::connection_fit::half_normal_width;
use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3;

/// A small deterministic generator (xorshift64*), uniform on [0, 1) and standard normal (Box-Muller).
struct Random(u64);
impl Random {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let (a, b) = (self.uniform().max(1e-300), self.uniform());
        (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
    }
}

#[test]
fn half_normal_width_recovers_the_shared_surface_width_under_uniform_misses() {
    let mut random = Random(7);
    for (width, share) in [(0.02, 0.8), (0.05, 0.6), (0.1, 0.4)] {
        let distances: Vec<f64> = (0..20000)
            .map(|_| {
                if random.uniform() < share {
                    (width * random.normal()).abs()
                } else {
                    10.0 * random.uniform()
                }
            })
            .collect();
        let fitted = half_normal_width(&distances, 10.0).unwrap();
        assert!(
            (fitted / width - 1.0).abs() < 0.05,
            "width {width} share {share}: fitted {fitted}"
        );
    }
    assert!(half_normal_width(&[0.1; 10], 10.0).is_none());
}

/// A 30 x 8 m room with 3 m walls, its surface sampled every `spacing` m and each sample moved by `noise` (m) of
/// normal noise per axis, seen from `x` along the room's middle out to `reach` (m), in the robot's frame.
fn room_scan(random: &mut Random, x: f64, spacing: f64, noise: f64, reach: f64) -> Vec<[f32; 3]> {
    let mut points = Vec::new();
    let mut add = |random: &mut Random, p: [f64; 3]| {
        let local = [p[0] - x, p[1], p[2]];
        if local[0].hypot(local[1]) < reach {
            points.push(local.map(|v| (v + noise * random.normal()) as f32));
        }
    };
    let steps = |length: f64| (0..=(length / spacing) as usize).map(move |i| i as f64 * spacing);
    for along in steps(30.0) {
        for up in steps(3.0) {
            add(random, [along, -4.0, up]);
            add(random, [along, 4.0, up]);
        }
        for across in steps(8.0) {
            add(random, [along, across - 4.0, 3.0]);
        }
    }
    for across in steps(8.0) {
        for up in steps(3.0) {
            add(random, [0.0, across - 4.0, up]);
            add(random, [30.0, across - 4.0, up]);
        }
    }
    points
}

/// A GscPgo fed `keyframes` scans of the room, `step` m apart along x, with exact odometry.
fn walk(config: Config, keyframes: usize, step: f64, noise: f64, reach: f64) -> GscPgo {
    let mut random = Random(11);
    let mut pgo = GscPgo::new(config);
    for k in 0..keyframes {
        let x = 1.0 + k as f64 * step;
        let mut pose = PoseWithTime::new(mat3::identity(), [x, 0.0, 0.0]);
        pose.set_time(k as i32, 0);
        let cloud = room_scan(&mut random, x, 0.05, noise, reach);
        pgo.add_key_pose(&CloudWithPose {
            cloud: Some(Arc::new(cloud)),
            pose,
        });
    }
    pgo
}

#[test]
fn measured_inlier_distance_follows_the_sensor_noise() {
    let config = Config {
        closure_inlier_distance_from_odometry: true,
        ..Config::default()
    };
    let measured = |noise: f64| {
        walk(config.clone(), 30, 0.6, noise, 12.0)
            .closure_inlier_squared_distance()
            .sqrt()
    };
    let (quiet, noisy) = (measured(0.01), measured(0.05));
    // a right alignment's width is the clouds' noise and the 0.1 m voxel spacing together, so the distance grows with
    // the noise, and stays within the 3-sigma reach of either source alone
    assert!(
        quiet > 0.0 && noisy > quiet * 1.3,
        "quiet {quiet} noisy {noisy}"
    );
    assert!(
        noisy < 3.0 * (0.05f64 * 3f64.sqrt()).hypot(0.1),
        "noisy {noisy}"
    );
    // off, the configured value holds
    let configured = walk(Config::default(), 30, 0.6, 0.05, 12.0).closure_inlier_squared_distance();
    assert_eq!(configured, Config::default().closure_score_thresh);
}

#[test]
fn odometry_drift_is_a_random_walk_in_distance_and_turn_and_starts_the_next_run_from_the_config() {
    let config = Config {
        odom_xy_drift_per_sqrt_m: 0.01,
        odom_xy_drift_per_sqrt_rad: 0.02,
        ..Config::default()
    };
    // 4 m travelled while turning 0.25 rad: (1 cm)^2 x 4 + (2 cm)^2 x 0.25
    let [xy, ..] = dimos_gsc_pgo::big_picture_fit::motion_variances(&config, 4.0, 0.25);
    assert!((xy - (1e-4 * 4.0 + 4e-4 * 0.25)).abs() < 1e-12, "{xy}");
    // before any loop the next run's drift is the configured one, measured nowhere
    let (drift, measured) = GscPgo::new(config.clone()).odometry_drift_for_next_run();
    assert_eq!(drift[0], [0.01, 0.02]);
    assert_eq!(measured, [false; 4]);
}
