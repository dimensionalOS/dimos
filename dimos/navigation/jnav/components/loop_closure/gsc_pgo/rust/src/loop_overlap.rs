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
use crate::gsc_pgo::{Config, GscPgo, KeyPoseWithCloud};
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

pub(crate) type Sums = [Mat3; 4];

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

/// Running sums along the odometry chain, `sums[i]` over steps before keyframe i: each step's translation noise and
/// rotation noise in world axes, with the rotation noise also as A S(q)^T and S(q) A S(q)^T, q the keyframe the step
/// turns at. The lever-arm sum over steps j..t of S(a - q) A S(a - q)^T then expands into these, for any newest
/// position a, in O(1).
pub(crate) fn odometry_noise_sums(key_poses: &[KeyPoseWithCloud], config: &Config) -> Vec<Sums> {
    let zero = [[0.0; 3]; 3];
    let mut sums = vec![[zero; 4]];
    for edge in odometry_edges(key_poses, config) {
        let (from, to) = (&key_poses[edge.from], &key_poses[edge.to]);
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

/// Position covariance of keyframe `to` (at `newest`) as seen from keyframe `j`, along the odometry chain between
/// them (see `odometry_noise_sums`).
pub(crate) fn odometry_covariance(sums: &[Sums], j: usize, to: usize, newest: &Vec3) -> Mat3 {
    let last = &sums[to];
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

/// The squared Mahalanobis distance a 3-D Gaussian stays under with probability `p` (chi-square, 3 degrees of freedom:
/// CDF = erf(sqrt(x / 2)) - sqrt(2x / pi) e^(-x / 2)).
pub(crate) fn chi2_inverse_3dof(p: f64) -> f64 {
    let (mut lo, mut hi) = (0.0f64, 200.0f64);
    for _ in 0..100 {
        let x = 0.5 * (lo + hi);
        let cdf = 1.0
            - erfc((x / 2.0).sqrt())
            - (2.0 * x / std::f64::consts::PI).sqrt() * (-x / 2.0).exp();
        if cdf < p {
            lo = x;
        } else {
            hi = x;
        }
    }
    0.5 * (lo + hi)
}

/// Whether keyframes `a` and `b` could be the same place: their positions differ by no more than the odometry between
/// them could have drifted plus `radius` (the position search's reach), within the `cutoff` squared Mahalanobis
/// distance. Two visits of one spot in a drifted graph sit far apart in position yet within that drift.
pub(crate) fn could_be_same_place(
    key_poses: &[KeyPoseWithCloud],
    sums: &[Sums],
    a: usize,
    b: usize,
    radius: f64,
    cutoff: f64,
) -> bool {
    let (from, to) = (a.min(b), a.max(b));
    let (p_from, p_to) = (
        key_poses[from].translation_global,
        key_poses[to].translation_global,
    );
    let covariance = add_m(
        &odometry_covariance(sums, from, to, &p_to),
        &[
            [radius * radius, 0.0, 0.0],
            [0.0, radius * radius, 0.0],
            [0.0, 0.0, radius * radius],
        ],
    );
    quadratic_inverse(&covariance, &mat3::sub(&p_to, &p_from)) <= cutoff
}

impl GscPgo {
    /// See `LoopOverlap`; None until a keyframe's scan has given a sensor range (the mean of the keyframes' reaches).
    /// The uncertainty of each earlier keyframe as seen from keyframe `reference`, and the scans' reach; None until a
    /// keyframe's scan has given a reach (the mean of the keyframes' reaches).
    pub(crate) fn overlap_model(&self, reference: usize) -> Option<OverlapModel<'_>> {
        let (reach_sum, reach_count) = self.keyframe_reach;
        let sensor_range = reach_sum / reach_count.max(1) as f64;
        if sensor_range <= 0.0 || reference == 0 || reference >= self.key_poses.len() {
            return None;
        }
        let newest = self.key_poses[reference].translation_global;
        let sums = odometry_noise_sums(&self.key_poses, &self.config);
        let (solved_tip, solved) = match &self.solved_relative_covariance {
            Some((tip, covariance)) if *tip <= reference => (*tip, covariance.as_slice()),
            _ => (0, &[] as &[Mat3]),
        };
        // a solved keyframe: the solve's tip as seen from it, plus the odometry chain from the tip to the reference, swung
        // by the tip's heading uncertainty as seen from it (taken from the odometry between them, an upper bound: the
        // solve's closures may have tightened it)
        let since_solve = odometry_covariance(&sums, solved_tip, reference, &newest);
        let tip_lever = skew(&mat3::sub(
            &newest,
            &self.key_poses[solved_tip].translation_global,
        ));
        Some(OverlapModel {
            key_poses: &self.key_poses,
            sums,
            solved,
            solved_tip,
            since_solve,
            tip_lever,
            reference,
            newest,
            sensor_range,
            search_radius: self.config.closure_search_radius,
        })
    }

    /// See `LoopOverlap`; None until a keyframe's scan has given a sensor range.
    pub fn loop_overlap(&self) -> Option<LoopOverlap> {
        let model = self.overlap_model(self.key_poses.len().checked_sub(1)?)?;
        let mut overlap = LoopOverlap::default();
        let mut none_overlap = 1.0;
        let mut last_counted: Option<Vec3> = None;
        for j in 0..self.newest_closure_target() {
            let position = self.key_poses[j].translation_global;
            // keyframes closer than a sensor range along the path see the same place: count one of them
            if last_counted
                .is_some_and(|p| mat3::norm(&mat3::sub(&p, &position)) < model.sensor_range)
            {
                continue;
            }
            last_counted = Some(position);
            let (probability, unresolved) = model.overlap(j);
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

/// Each earlier keyframe's position uncertainty as seen from a reference keyframe (see `GscPgo::overlap_model`).
pub(crate) struct OverlapModel<'a> {
    key_poses: &'a [KeyPoseWithCloud],
    sums: Vec<Sums>,
    solved: &'a [Mat3],
    solved_tip: usize,
    since_solve: Mat3,
    tip_lever: Mat3,
    reference: usize,
    newest: Vec3,
    pub(crate) sensor_range: f64,
    search_radius: f64,
}

impl OverlapModel<'_> {
    /// Keyframe `j`'s position covariance as seen from the reference keyframe.
    pub(crate) fn relative(&self, j: usize) -> Mat3 {
        match self.solved.get(j) {
            Some(covariance) if j <= self.solved_tip => {
                let heading = sub_m(&self.sums[self.solved_tip][1], &self.sums[j][1]);
                let swing = mat3::mat_mul(
                    &mat3::mat_mul(&self.tip_lever, &heading),
                    &mat3::transpose(&self.tip_lever),
                );
                add_m(&add_m(covariance, &self.since_solve), &swing)
            }
            _ => odometry_covariance(&self.sums, j, self.reference, &self.newest),
        }
    }
    /// The chance the reference scan overlaps keyframe `j`'s (the probability product kernel of j's uncertainty with a
    /// sensor-reach Gaussian of standard deviation sensor_range, e^-1/2 at its edge), and the chance j is off by more than
    /// the position search's radius (3-dof chi tail, isotropic), i.e. that only a search by appearance could find it.
    pub(crate) fn overlap(&self, j: usize) -> (f64, f64) {
        let covariance = self.relative(j);
        let sensor = self.sensor_range * self.sensor_range;
        let blurred = add_m(
            &covariance,
            &[[sensor, 0.0, 0.0], [0.0, sensor, 0.0], [0.0, 0.0, sensor]],
        );
        let offset = mat3::sub(&self.key_poses[j].translation_global, &self.newest);
        let probability = ((sensor.powi(3) / det(&blurred)).sqrt()
            * (-0.5 * quadratic_inverse(&blurred, &offset)).exp())
        .clamp(0.0, 1.0);
        let spread = ((covariance[0][0] + covariance[1][1] + covariance[2][2]) / 3.0)
            .max(0.0)
            .sqrt();
        let unresolved = if spread > 0.0 {
            let x = self.search_radius / spread;
            (erfc(x / std::f64::consts::SQRT_2)
                + (2.0 / std::f64::consts::PI).sqrt() * x * (-0.5 * x * x).exp())
            .clamp(0.0, 1.0)
        } else {
            0.0
        };
        (probability, unresolved)
    }
}
