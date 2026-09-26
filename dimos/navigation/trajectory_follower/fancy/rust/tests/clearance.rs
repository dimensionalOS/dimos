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

//! The room hint's cases, and the proof that its grid never disagrees with brute force.
//! Brute force is the definition; `control/test_rust_parity.py` pins the same answer against scipy.

use dimos_trajectory_follower::clearance::path_clearance;

/// Seeded LCG: the crate's dependencies stay at pyo3/numpy.
struct Rng(u64);

impl Rng {
    fn next_f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
}

/// The definition: scan every obstacle point.
fn brute(xy: &[[f64; 2]], points: &[[f32; 3]], half_width: f64) -> Vec<f64> {
    xy.iter()
        .map(|q| {
            let mut best = f64::INFINITY;
            for p in points {
                let (dx, dy) = (p[0] as f64 - q[0], p[1] as f64 - q[1]);
                let d = (dx * dx + dy * dy).sqrt();
                if d < best {
                    best = d;
                }
            }
            best - half_width
        })
        .collect()
}

#[test]
fn the_grid_never_disagrees_with_brute_force() {
    let mut rng = Rng(0x5eed);
    for case in 0..200 {
        // tighter and wider than one cell: first-ring hits and long ring walks
        let spread = [0.05, 0.5, 5.0, 40.0][case % 4];
        let n_points = 1 + case * 3;
        let points: Vec<[f32; 3]> = (0..n_points)
            .map(|_| {
                [
                    rng.range(-spread, spread) as f32,
                    rng.range(-spread, spread) as f32,
                    // spread over z: the hint reads none of it
                    rng.range(-0.2, 0.7) as f32,
                ]
            })
            .collect();
        let xy: Vec<[f64; 2]> = (0..12)
            .map(|_| [rng.range(-spread, spread), rng.range(-spread, spread)])
            .collect();

        let got = path_clearance(&xy, &points, 0.25);
        let want = brute(&xy, &points, 0.25);
        for (k, (a, b)) in got.iter().zip(want.iter()).enumerate() {
            assert_eq!(
                a, b,
                "case {case} waypoint {k}: grid {a} vs brute force {b}"
            );
        }
    }
}

#[test]
fn a_query_far_outside_the_cloud_still_terminates() {
    // the ring walk must stop once it outgrows the grid
    let points = vec![[0.0f32, 0.0, 0.2]];
    let got = path_clearance(&[[1000.0, -1000.0]], &points, 0.0);
    let want = (1000.0f64 * 1000.0 + 1000.0 * 1000.0).sqrt();
    assert!((got[0] - want).abs() < 1e-9);
}

#[test]
fn every_point_handed_in_takes_room_away_whatever_its_z() {
    // the model already decided; re-judging z here would take the hint off the world the plan was priced on
    let q = [[0.0, 0.0]];
    // compared against the widened f32: 0.1f32 is not 0.1
    for z in [-0.5f32, 0.0, 0.04, 0.2, 0.46, 2.0] {
        let pts = vec![[0.1f32, 0.0, z]];
        assert_eq!(path_clearance(&q, &pts, 0.0)[0], 0.1f32 as f64, "z {z}");
    }
    assert_eq!(path_clearance(&q, &[], 0.0)[0], f64::INFINITY);
}

#[test]
fn the_body_is_subtracted_and_may_go_negative() {
    // inside the footprint is negative; the governor floors it, this must not
    let q = [[0.0, 0.0]];
    let points = vec![[0.1f32, 0.0, 0.2]];
    assert_eq!(path_clearance(&q, &points, 0.25)[0], 0.1f32 as f64 - 0.25);
}

#[test]
fn no_obstacles_or_no_path_is_infinite_room() {
    // a map the model kept nothing from is an empty map, not a tight one
    let points = vec![[0.0f32, 0.0, 5.0]];
    assert_eq!(path_clearance(&[[0.0, 0.0]], &[], 0.25)[0], f64::INFINITY);
    assert!(path_clearance(&[], &points, 0.25).is_empty());
}
