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

//! Chance the newest scan overlaps an earlier, unclosed stretch: PPK of each keyframe's relative position uncertainty
//! (solved covariance, then odometry chain) with a sensor-range blob, weighted by P(off by > `closure_search_radius`).

use crate::big_picture_fit::odometry_edges;
use crate::gsc_pgo::{Config, GscPgo};
use crate::mat3::{self, add_mat, det, skew, sub_mat, Mat3, Vec3};
use crate::store::KeyFrames;

#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct LoopOverlap {
    /// Chance the newest scan overlaps an earlier unresolved stretch (keyframes a sensor range apart are independent).
    pub unresolved_loop_probability: f64,
    /// log10 of hazard H (probability = 1 - e^-H), finite where probability underflows; -inf if nothing can overlap.
    pub log10_hazard: f64,
    /// Display pressure in [0, 1]: log-compressed hazard below 0.5 (`soft_pressure_half_log10_hazard`), linear above.
    pub soft_pressure: f64,
    /// Sum over those keyframes of the overlap chance (resolved or not).
    pub expected_overlaps: f64,
    /// The keyframe with the highest unresolved overlap chance, and that chance.
    pub most_likely_keyframe: usize,
    pub most_likely_probability: f64,
}

pub(crate) type Sums = [Mat3; 4];

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

/// Prefix sums over odometry steps (translation T, rotation A, A S(q)^T, S(q) A S(q)^T) so any lever-arm sum is O(1).
pub(crate) fn odometry_error_estimate_sums(
    key_poses: &KeyFrames,
    config: &Config,
    noise: [f64; 4],
) -> Vec<Sums> {
    let zero = [[0.0; 3]; 3];
    let mut sums = vec![[zero; 4]];
    for edge in odometry_edges(key_poses, config, noise) {
        let (from, to) = (&key_poses.get(edge.from), &key_poses.get(edge.to));
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
            add_mat(&last[0], &translation),
            add_mat(&last[1], &rotation),
            add_mat(&last[2], &rotation_lever),
            add_mat(&last[3], &lever_rotation_lever),
        ]);
    }
    sums
}

/// Position covariance of keyframe `to` (at `newest`) as seen from keyframe `j`, along the odometry chain.
pub(crate) fn odometry_covariance(sums: &[Sums], j: usize, to: usize, newest: &Vec3) -> Mat3 {
    let last = &sums[to];
    let between: Sums = std::array::from_fn(|k| sub_mat(&last[k], &sums[j][k]));
    let lever = skew(newest);
    let cross = mat3::mat_mul(&lever, &between[2]);
    let swing = sub_mat(
        &add_mat(
            &mat3::mat_mul(
                &mat3::mat_mul(&lever, &between[1]),
                &mat3::transpose(&lever),
            ),
            &between[3],
        ),
        &add_mat(&cross, &mat3::transpose(&cross)),
    );
    add_mat(&between[0], &swing)
}

/// Squared Mahalanobis distance a 3-D Gaussian stays under with probability `p` (inverse chi-square, 3 dof).
pub(crate) fn chi2_inverse_3dof(p: f64) -> f64 {
    chi2_inverse(p, |x| {
        1.0 - erfc((x / 2.0).sqrt()) - (2.0 * x / std::f64::consts::PI).sqrt() * (-x / 2.0).exp()
    })
}

/// The x in [0, 200] where the increasing `cdf` reaches `p`, by 100 bisection steps.
pub(crate) fn chi2_inverse(p: f64, cdf: impl Fn(f64) -> f64) -> f64 {
    let (mut lo, mut hi) = (0.0f64, 200.0f64);
    for _ in 0..100 {
        let x = 0.5 * (lo + hi);
        if cdf(x) < p {
            lo = x;
        } else {
            hi = x;
        }
    }
    0.5 * (lo + hi)
}

/// Whether `a` and `b` could be one place: offset within `cutoff` Mahalanobis of their odometry drift plus `radius`.
pub(crate) fn could_be_same_place(
    key_poses: &KeyFrames,
    sums: &[Sums],
    a: usize,
    b: usize,
    radius: f64,
    cutoff: f64,
) -> bool {
    let (from, to) = (a.min(b), a.max(b));
    let (p_from, p_to) = (key_poses.position(from), key_poses.position(to));
    let covariance = add_mat(
        &odometry_covariance(sums, from, to, &p_to),
        &[
            [radius * radius, 0.0, 0.0],
            [0.0, radius * radius, 0.0],
            [0.0, 0.0, radius * radius],
        ],
    );
    quadratic_inverse(&covariance, &mat3::sub(&p_to, &p_from)) <= cutoff
}

/// What the loop-overlap pressure reads; `keyframe_reach` is (sum, count), `solved_relative_covariance` is (tip, covs).
pub(crate) struct OverlapInputs<'a> {
    pub(crate) key_poses: &'a KeyFrames,
    pub(crate) config: &'a Config,
    pub(crate) keyframe_reach: (f64, usize),
    pub(crate) error_estimate: [f64; 4],
    pub(crate) solved_relative_covariance: Option<(usize, &'a [Mat3])>,
}

/// Owned `OverlapInputs`, taken by a pressure job at dispatch.
#[derive(Clone)]
pub(crate) struct OverlapSnapshot {
    key_poses: KeyFrames,
    config: Config,
    keyframe_reach: (f64, usize),
    error_estimate: [f64; 4],
    solved_relative_covariance: Option<(usize, std::sync::Arc<Vec<Mat3>>)>,
    committed_closures: crate::store::Closures,
}

impl OverlapSnapshot {
    pub(crate) fn inputs(&self) -> OverlapInputs<'_> {
        OverlapInputs {
            key_poses: &self.key_poses,
            config: &self.config,
            keyframe_reach: self.keyframe_reach,
            error_estimate: self.error_estimate,
            solved_relative_covariance: self
                .solved_relative_covariance
                .as_ref()
                .map(|(tip, covariance)| (*tip, covariance.as_slice())),
        }
    }
    /// (keyframes, committed closures): what a pressure job's cost grows with.
    pub(crate) fn size(&self) -> (usize, usize) {
        (self.key_poses.len(), self.committed_closures.len())
    }
}

impl GscPgo {
    /// O(1) snapshot of the pressure inputs (shared handles, no copies).
    pub(crate) fn overlap_snapshot(&self) -> OverlapSnapshot {
        OverlapSnapshot {
            key_poses: self.key_poses.clone(),
            config: self.config.clone(),
            keyframe_reach: self.keyframe_reach,
            error_estimate: self.odometry_error_estimate(),
            solved_relative_covariance: self.solved_relative_covariance.clone(),
            committed_closures: self.committed_closures.clone(),
        }
    }
}

impl<'a> OverlapInputs<'a> {
    /// Mirrors `GscPgo::newest_closure_target`.
    fn newest_closure_target(&self) -> usize {
        (self.key_poses.len() - 1).saturating_sub(self.config.target_half_range(1) + 1)
    }
    /// Earlier keyframes' uncertainty as seen from `reference`; None before any scan reach (sensor range = mean).
    pub(crate) fn overlap_model(&self, reference: usize) -> Option<OverlapModel<'a>> {
        let (reach_sum, reach_count) = self.keyframe_reach;
        let sensor_range = reach_sum / reach_count.max(1) as f64;
        if sensor_range <= 0.0 || reference == 0 || reference >= self.key_poses.len() {
            return None;
        }
        let newest = self.key_poses.position(reference);
        let sums = odometry_error_estimate_sums(self.key_poses, self.config, self.error_estimate);
        let (solved_tip, solved) = match self.solved_relative_covariance {
            Some((tip, covariance)) if tip <= reference => (tip, covariance),
            _ => (0, &[] as &[Mat3]),
        };
        // solved keyframes: solved covariance + odometry tip->reference + tip heading swing (odometry upper bound)
        let since_solve = odometry_covariance(&sums, solved_tip, reference, &newest);
        let tip_lever = skew(&mat3::sub(&newest, &self.key_poses.position(solved_tip)));
        Some(OverlapModel {
            key_poses: self.key_poses,
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
    pub(crate) fn loop_overlap(&self) -> Option<LoopOverlap> {
        let model = self.overlap_model(self.key_poses.len().checked_sub(1)?)?;
        let mut overlap = LoopOverlap::default();
        let mut none_overlap = 1.0;
        let mut last_counted: Option<Vec3> = None;
        let mut ln_hazards = Vec::new();
        for j in 0..self.newest_closure_target().min(self.key_poses.len()) {
            let position = self.key_poses.position(j);
            // keyframes closer than a sensor range along the path see the same place: count one of them
            if last_counted
                .is_some_and(|p| mat3::norm(&mat3::sub(&p, &position)) < model.sensor_range)
            {
                continue;
            }
            last_counted = Some(position);
            let (probability, unresolved, ln_chance) = model.overlap(j);
            let chance = (probability * unresolved).clamp(0.0, 1.0);
            // ln of hazard -ln(1 - chance); for tiny chance use ln(chance) to avoid underflow
            let ln_hazard = if chance > 1e-8 {
                (-(-chance).ln_1p()).ln()
            } else {
                ln_chance.min(0.0)
            };
            ln_hazards.push(ln_hazard);
            overlap.expected_overlaps += probability;
            none_overlap *= 1.0 - chance;
            if chance > overlap.most_likely_probability {
                overlap.most_likely_probability = chance;
                overlap.most_likely_keyframe = j;
            }
        }
        overlap.unresolved_loop_probability = 1.0 - none_overlap;
        let top = ln_hazards.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        overlap.log10_hazard = if top.is_finite() {
            (top + ln_hazards.iter().map(|h| (h - top).exp()).sum::<f64>().ln())
                / std::f64::consts::LN_10
        } else {
            f64::NEG_INFINITY
        };
        overlap.soft_pressure = soft_pressure(
            overlap.unresolved_loop_probability,
            overlap.log10_hazard,
            self.config.soft_pressure_half_log10_hazard,
            self.config.soft_pressure_decades,
        );
        Some(overlap)
    }
}

/// Below `half_log10`: 0.5 (1 - ln(1 + d) / ln(1 + decades)), d = decades under; above: linear in probability to 1.
pub fn soft_pressure(probability: f64, log10_hazard: f64, half_log10: f64, decades: f64) -> f64 {
    if !log10_hazard.is_finite() {
        return 0.0;
    }
    let below = half_log10 - log10_hazard;
    if below > 0.0 {
        return (0.5 * (1.0 - below.ln_1p() / decades.ln_1p())).max(0.0);
    }
    let half = -(-(10f64.powf(half_log10))).exp_m1();
    0.5 + 0.5 * ((probability - half) / (1.0 - half)).clamp(0.0, 1.0)
}

/// Each earlier keyframe's position uncertainty as seen from a reference keyframe (see `GscPgo::overlap_model`).
pub(crate) struct OverlapModel<'a> {
    key_poses: &'a KeyFrames,
    sums: Vec<Sums>,
    solved: &'a [Mat3],
    solved_tip: usize,
    since_solve: Mat3,
    tip_lever: Mat3,
    reference: usize,
    pub(crate) newest: Vec3,
    pub(crate) sensor_range: f64,
    search_radius: f64,
}

impl OverlapModel<'_> {
    /// Keyframe `j`'s position covariance as seen from the reference keyframe.
    pub(crate) fn relative(&self, j: usize) -> Mat3 {
        match self.solved.get(j) {
            Some(covariance) if j <= self.solved_tip => {
                let heading = sub_mat(&self.sums[self.solved_tip][1], &self.sums[j][1]);
                let swing = mat3::mat_mul(
                    &mat3::mat_mul(&self.tip_lever, &heading),
                    &mat3::transpose(&self.tip_lever),
                );
                add_mat(&add_mat(covariance, &self.since_solve), &swing)
            }
            _ => odometry_covariance(&self.sums, j, self.reference, &self.newest),
        }
    }
    /// (P(overlap j) by PPK with a sensor-range Gaussian, P(j off by > search radius), finite ln of their product).
    pub(crate) fn overlap(&self, j: usize) -> (f64, f64, f64) {
        let covariance = self.relative(j);
        let sensor = self.sensor_range * self.sensor_range;
        let blurred = add_mat(
            &covariance,
            &[[sensor, 0.0, 0.0], [0.0, sensor, 0.0], [0.0, 0.0, sensor]],
        );
        let offset = mat3::sub(&self.key_poses.position(j), &self.newest);
        let ln_probability = 0.5 * (sensor.powi(3) / det(&blurred)).ln()
            - 0.5 * quadratic_inverse(&blurred, &offset);
        let probability = ((sensor.powi(3) / det(&blurred)).sqrt()
            * (-0.5 * quadratic_inverse(&blurred, &offset)).exp())
        .clamp(0.0, 1.0);
        let spread = ((covariance[0][0] + covariance[1][1] + covariance[2][2]) / 3.0)
            .max(0.0)
            .sqrt();
        let (unresolved, ln_unresolved) = if spread > 0.0 {
            let x = self.search_radius / spread;
            let tail = (erfc(x / std::f64::consts::SQRT_2)
                + (2.0 / std::f64::consts::PI).sqrt() * x * (-0.5 * x * x).exp())
            .clamp(0.0, 1.0);
            // far tail: ln of asymptote sqrt(2/pi) (x + 1/x) e^(-x^2/2), which does not underflow
            let ln_tail = if tail > 1e-300 {
                tail.ln()
            } else {
                ((2.0 / std::f64::consts::PI).sqrt() * (x + 1.0 / x)).ln() - 0.5 * x * x
            };
            (tail, ln_tail)
        } else {
            (0.0, f64::NEG_INFINITY)
        };
        (
            probability,
            unresolved,
            ln_probability.min(0.0) + ln_unresolved,
        )
    }
}
