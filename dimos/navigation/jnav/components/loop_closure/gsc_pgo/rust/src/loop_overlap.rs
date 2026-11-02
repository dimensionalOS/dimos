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

//! How likely the newest keyframe's scan is to overlap an earlier stretch of the path that no closure has tied it to.
//!
//! The newest keyframe's position as seen from each earlier keyframe (in that keyframe's own frame, so the map's overall
//! heading uncertainty does not count) is uncertain: up to the newest solve, by the solved graph's covariance
//! (`big_picture_fit::relative_position_covariances`); after it, by the odometry chain, where each
//! step's translation noise adds up and its rotation noise swings everything after it by the lever arm to the newest
//! keyframe. The chance the newest scan overlaps keyframe j's is the probability product kernel (PPK) of that
//! uncertainty with a sensor-range blob, and the part of it that is unresolved is weighted by the chance j is off by more
//! than `closure_search_radius` (the position search would find it otherwise).

use crate::big_picture_fit::odometry_edges;
use crate::gsc_pgo::GscPgo;
use crate::mat3::{self, Mat3, Vec3};

#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct LoopOverlap {
    /// Chance the newest scan overlaps at least one earlier, unresolved stretch of the path (keyframes a sensor range
    /// apart along the path counted as independent).
    pub unresolved_loop_probability: f64,
    /// Sum over those keyframes of the chance of overlapping each (resolved or not).
    pub expected_overlaps: f64,
    /// The keyframe with the highest unresolved overlap chance, and that chance.
    pub most_likely_keyframe: usize,
    pub most_likely_probability: f64,
}

type Sums = [Mat3; 4];

fn skew(v: &Vec3) -> Mat3 {
    [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
}

fn add_m(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] + b[i][j]))
}

fn sub_m(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] - b[i][j]))
}

fn det(m: &Mat3) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// `v^T m^-1 v` for a symmetric positive-definite `m` (Cramer's rule).
fn quadratic_inverse(m: &Mat3, v: &Vec3) -> f64 {
    let d = det(m);
    let adjugate: Mat3 = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            let (r, c) = ([(j + 1) % 3, (j + 2) % 3], [(i + 1) % 3, (i + 2) % 3]);
            m[r[0]][c[0]] * m[r[1]][c[1]] - m[r[0]][c[1]] * m[r[1]][c[0]]
        })
    });
    (0..3)
        .map(|i| (0..3).map(|j| v[i] * adjugate[i][j] * v[j]).sum::<f64>())
        .sum::<f64>()
        / d
}

/// Complementary error function (Abramowitz & Stegun 7.1.26, |error| < 1.5e-7).
fn erfc(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    let tail = poly * (-x * x).exp();
    if x >= 0.0 {
        tail
    } else {
        2.0 - tail
    }
}

impl GscPgo {
    /// Running sums along the odometry chain, `sums[i]` over steps before keyframe i: each step's translation noise and
    /// rotation noise in world axes, with the rotation noise also as A S(q)^T and S(q) A S(q)^T, q the keyframe the step
    /// turns at. The lever-arm sum over steps j..t of S(a - q) A S(a - q)^T then expands into these, for any newest
    /// position a, in O(1).
    fn odometry_noise_sums(&self) -> Vec<Sums> {
        let zero = [[0.0; 3]; 3];
        let mut sums = vec![[zero; 4]];
        for edge in odometry_edges(&self.key_poses, &self.config) {
            let (from, to) = (&self.key_poses[edge.from], &self.key_poses[edge.to]);
            let world = |rotation: &Mat3, variances: [f64; 3]| -> Mat3 {
                let scaled: Mat3 =
                    std::array::from_fn(|i| std::array::from_fn(|j| rotation[i][j] * variances[j]));
                mat3::mat_mul(&scaled, &mat3::transpose(rotation))
            };
            let translation = world(
                &from.rotation_global,
                [edge.variances[3], edge.variances[4], edge.variances[5]],
            );
            let rotation = world(
                &to.rotation_global,
                [edge.variances[0], edge.variances[1], edge.variances[2]],
            );
            let lever = skew(&to.translation_global);
            let rotation_lever = mat3::mat_mul(&rotation, &mat3::transpose(&lever));
            let lever_rotation_lever = mat3::mat_mul(&lever, &rotation_lever);
            let last = sums.last().unwrap();
            sums.push([
                add_m(&last[0], &translation),
                add_m(&last[1], &rotation),
                add_m(&last[2], &rotation_lever),
                add_m(&last[3], &lever_rotation_lever),
            ]);
        }
        sums
    }

    /// Position covariance of keyframe `j` relative to the newest one along the odometry chain from j (see
    /// `odometry_noise_sums`).
    fn odometry_covariance(sums: &[Sums], j: usize, newest: &Vec3) -> Mat3 {
        let last = sums.last().unwrap();
        let between: Sums = std::array::from_fn(|k| sub_m(&last[k], &sums[j][k]));
        let lever = skew(newest);
        let cross = mat3::mat_mul(&lever, &between[2]);
        let swing = sub_m(
            &add_m(
                &mat3::mat_mul(
                    &mat3::mat_mul(&lever, &between[1]),
                    &mat3::transpose(&lever),
                ),
                &between[3],
            ),
            &add_m(&cross, &mat3::transpose(&cross)),
        );
        add_m(&between[0], &swing)
    }

    /// See `LoopOverlap`; None until a keyframe's scan has given a sensor range (the mean of the keyframes' reaches).
    pub fn loop_overlap(&self) -> Option<LoopOverlap> {
        let (reach_sum, reach_count) = self.keyframe_reach;
        let sensor_range = reach_sum / reach_count.max(1) as f64;
        if sensor_range <= 0.0 || self.key_poses.len() < 2 {
            return None;
        }
        let newest = self.key_poses.last().unwrap().translation_global;
        let sums = self.odometry_noise_sums();
        let (solved_tip, solved) = match &self.solved_relative_covariance {
            Some((tip, covariance)) if *tip < self.key_poses.len() => (*tip, covariance.as_slice()),
            _ => (0, &[] as &[Mat3]),
        };
        // a solved keyframe: the solve's tip as seen from it, plus the odometry chain from the tip to now, swung by the
        // tip's heading uncertainty as seen from it (taken from the odometry between them, an upper bound: the solve's
        // closures may have tightened it)
        let since_solve = Self::odometry_covariance(&sums, solved_tip, &newest);
        let tip_lever = skew(&mat3::sub(
            &newest,
            &self.key_poses[solved_tip].translation_global,
        ));
        let relative = |j: usize| -> Mat3 {
            match solved.get(j) {
                Some(covariance) if j <= solved_tip => {
                    let heading = sub_m(&sums[solved_tip][1], &sums[j][1]);
                    let swing = mat3::mat_mul(
                        &mat3::mat_mul(&tip_lever, &heading),
                        &mat3::transpose(&tip_lever),
                    );
                    add_m(&add_m(covariance, &since_solve), &swing)
                }
                _ => Self::odometry_covariance(&sums, j, &newest),
            }
        };
        // the sensor's reach as a Gaussian of standard deviation sensor_range (the kernel is e^-1/2 at its edge)
        let sensor = sensor_range * sensor_range;
        let mut overlap = LoopOverlap::default();
        let mut none_overlap = 1.0;
        let mut last_counted: Option<Vec3> = None;
        for j in 0..self.newest_closure_target() {
            let position = self.key_poses[j].translation_global;
            // keyframes closer than a sensor range along the path see the same place: count one of them
            if last_counted.is_some_and(|p| mat3::norm(&mat3::sub(&p, &position)) < sensor_range) {
                continue;
            }
            last_counted = Some(position);
            let covariance = relative(j);
            let blurred = add_m(
                &covariance,
                &[[sensor, 0.0, 0.0], [0.0, sensor, 0.0], [0.0, 0.0, sensor]],
            );
            let offset = mat3::sub(&position, &newest);
            let probability = (sensor.powi(3) / det(&blurred)).sqrt()
                * (-0.5 * quadratic_inverse(&blurred, &offset)).exp();
            // unresolved: j may be off by more than the position search's radius (3-dof chi tail, isotropic)
            let spread = ((covariance[0][0] + covariance[1][1] + covariance[2][2]) / 3.0)
                .max(0.0)
                .sqrt();
            let unresolved = if spread > 0.0 {
                let x = self.config.closure_search_radius / spread;
                (erfc(x / std::f64::consts::SQRT_2)
                    + (2.0 / std::f64::consts::PI).sqrt() * x * (-0.5 * x * x).exp())
                .clamp(0.0, 1.0)
            } else {
                0.0
            };
            let chance = (probability * unresolved).clamp(0.0, 1.0);
            overlap.expected_overlaps += probability;
            none_overlap *= 1.0 - chance;
            if chance > overlap.most_likely_probability {
                overlap.most_likely_probability = chance;
                overlap.most_likely_keyframe = j;
            }
        }
        overlap.unresolved_loop_probability = 1.0 - none_overlap;
        Some(overlap)
    }
}
