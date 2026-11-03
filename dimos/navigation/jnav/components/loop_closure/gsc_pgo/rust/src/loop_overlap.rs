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

use crate::store::KeyFrames;
use crate::big_picture_fit::odometry_edges;
use crate::gsc_pgo::{Config, GscPgo};
use crate::mat3::{self, add_mat, det, skew, sub_mat, Mat3, Vec3};

#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct LoopOverlap {
    /// Chance the newest scan overlaps at least one earlier, unresolved stretch of the path (keyframes a sensor range
    /// apart along the path counted as independent).
    pub unresolved_loop_probability: f64,
    /// log10 of the hazard H behind it (unresolved_loop_probability = 1 - e^-H), summed in log space so it stays finite
    /// where the probability underflows to 0; -inf when nothing could overlap.
    pub log10_hazard: f64,
    /// The same probability without the cliff, for display: below 50% log-compressed in decades of hazard under
    /// `soft_pressure_half_log10_hazard`, above it linear in the probability up to 1 (see Config).
    pub soft_pressure: f64,
    /// Sum over those keyframes of the chance of overlapping each (resolved or not).
    pub expected_overlaps: f64,
    /// Pressure 2: the chance the newest scan overlaps at least one earlier stretch of the path that no kept loop closure
    /// ties to the newest stretch, without the position-search factor: 1 when the odometry is exact and an earlier
    /// stretch is within sensor reach (the newest keyframe's own recent stretch, `newest_closure_target` on, excluded).
    pub unclosed_overlap_probability: f64,
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

/// Running sums along the odometry chain, `sums[i]` over steps before keyframe i: each step's translation noise and
/// rotation noise in world axes, with the rotation noise also as A S(q)^T and S(q) A S(q)^T, q the keyframe the step
/// turns at. The lever-arm sum over steps j..t of S(a - q) A S(a - q)^T then expands into these, for any newest
/// position a, in O(1).
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

/// Position covariance of keyframe `to` (at `newest`) as seen from keyframe `j`, along the odometry chain between
/// them (see `odometry_error_estimate_sums`).
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

/// The squared Mahalanobis distance a 3-D Gaussian stays under with probability `p` (chi-square, 3 degrees of freedom:
/// CDF = erf(sqrt(x / 2)) - sqrt(2x / pi) e^(-x / 2)).
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

/// Whether keyframes `a` and `b` could be the same place: their positions differ by no more than the odometry between
/// them could have drifted plus `radius` (the position search's reach), within the `cutoff` squared Mahalanobis
/// distance. Two visits of one spot in a drifted graph sit far apart in position yet within that drift.
pub(crate) fn could_be_same_place(
    key_poses: &KeyFrames,
    sums: &[Sums],
    a: usize,
    b: usize,
    radius: f64,
    cutoff: f64,
) -> bool {
    let (from, to) = (a.min(b), a.max(b));
    let (p_from, p_to) = (
        key_poses.position(from),
        key_poses.position(to),
    );
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

/// What the loop-overlap pressure reads: the keyframes, the config, the keyframes' reach sum and count, the odometry error
/// estimate, the newest applied solve's tip and relative covariances, and the committed closures.
pub(crate) struct OverlapInputs<'a> {
    pub(crate) key_poses: &'a KeyFrames,
    pub(crate) config: &'a Config,
    pub(crate) keyframe_reach: (f64, usize),
    pub(crate) error_estimate: [f64; 4],
    pub(crate) solved_relative_covariance: Option<(usize, &'a [Mat3])>,
    pub(crate) committed_closures: &'a crate::store::Closures,
}

/// `OverlapInputs` owned, a snapshot a pressure job takes at its dispatch (`pressure_let_enabled`).
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
            committed_closures: &self.committed_closures,
        }
    }
    /// The keyframes and committed closures it holds (what a pressure job's cost grows with).
    pub(crate) fn size(&self) -> (usize, usize) {
        (self.key_poses.len(), self.committed_closures.len())
    }
}

impl GscPgo {
    /// What the loop-overlap pressure reads, as an O(1) snapshot (`pressure_let_enabled`): shared handles, no copies of
    /// the keyframes, closures or covariances.
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
    /// `GscPgo::newest_closure_target`: the newest keyframe a closure can target from the newest one.
    fn newest_closure_target(&self) -> usize {
        (self.key_poses.len() - 1).saturating_sub(self.config.target_half_range(1) + 1)
    }
    /// The uncertainty of each earlier keyframe as seen from keyframe `reference`, and the scans' reach; None until a
    /// keyframe's scan has given a reach (the mean of the keyframes' reaches).
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
        // a solved keyframe: the solve's tip as seen from it, plus the odometry chain from the tip to the reference, swung
        // by the tip's heading uncertainty as seen from it (taken from the odometry between them, an upper bound: the
        // solve's closures may have tightened it)
        let since_solve = odometry_covariance(&sums, solved_tip, reference, &newest);
        let tip_lever = skew(&mat3::sub(
            &newest,
            &self.key_poses.position(solved_tip),
        ));
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
        // pressure 2: a kept closure from the stretch just walked (its source within a sensor range back along the path)
        // closes the earlier places within a sensor range of its target
        let near = |a: &Vec3, b: &Vec3| mat3::norm(&mat3::sub(a, b)) < model.sensor_range;
        let mut none_unclosed = 1.0;
        // path length from each keyframe to the newest
        let mut path_from = vec![0.0; self.key_poses.len()];
        for k in (0..self.key_poses.len().saturating_sub(1)).rev() {
            path_from[k] = path_from[k + 1]
                + mat3::norm(&mat3::sub(
                    &self.key_poses.position(k + 1),
                    &self.key_poses.position(k),
                ));
        }
        let closed_places: Vec<Vec3> = self
            .committed_closures
            .iter()
            .filter(|c| c.gnc_weight > 0.5)
            .flat_map(|c| [(c.source_id, c.target_id), (c.target_id, c.source_id)])
            .filter(|&(recent, _)| {
                path_from
                    .get(recent)
                    .is_some_and(|&d| d <= model.sensor_range)
            })
            .map(|(_, earlier)| self.key_poses.position(earlier))
            .collect();
        for (j, &path_from_j) in path_from.iter().enumerate().take(self.newest_closure_target()) {
            let position = self.key_poses.position(j);
            // keyframes closer than a sensor range along the path see the same place: count one of them
            if last_counted
                .is_some_and(|p| mat3::norm(&mat3::sub(&p, &position)) < model.sensor_range)
            {
                continue;
            }
            last_counted = Some(position);
            let (probability, unresolved, ln_chance) = model.overlap(j);
            let closed = closed_places.iter().any(|place| near(place, &position));
            // the stretch walked within a sensor range back along the path is continuous odometry, not a loop
            if !closed && path_from_j > model.sensor_range {
                none_unclosed *= 1.0 - model.revisit(j);
            }
            let chance = (probability * unresolved).clamp(0.0, 1.0);
            // this place's hazard -ln(1 - chance), in log space: ln(chance) where -ln(1 - chance) ~ chance
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
        overlap.unclosed_overlap_probability = 1.0 - none_unclosed;
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

/// `LoopOverlap::soft_pressure`: below half, 0.5 (1 - ln(1 + d) / ln(1 + decades)) with d the decades of hazard under
/// `half_log10`; above, linear in the probability from the half point's to 1.
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
    /// The chance the reference scan overlaps keyframe `j`'s (the probability product kernel of j's uncertainty with a
    /// sensor-reach Gaussian of standard deviation sensor_range, e^-1/2 at its edge), and the chance j is off by more than
    /// the position search's radius (3-dof chi tail, isotropic), i.e. that only a search by appearance could find it; and
    /// the log of their product, finite where the product underflows.
    /// The chance the newest keyframe is back on keyframe j's spot: within the position search's radius of it, with the
    /// relative position uncertain as `relative` (the same kernel as `overlap`, the radius in place of the sensor reach).
    pub(crate) fn revisit(&self, j: usize) -> f64 {
        let covariance = self.relative(j);
        let radius = self.search_radius * self.search_radius;
        let blurred = add_mat(
            &covariance,
            &[[radius, 0.0, 0.0], [0.0, radius, 0.0], [0.0, 0.0, radius]],
        );
        let offset = mat3::sub(&self.key_poses.position(j), &self.newest);
        ((radius.powi(3) / det(&blurred)).sqrt()
            * (-0.5 * quadratic_inverse(&blurred, &offset)).exp())
        .clamp(0.0, 1.0)
    }
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
            // far in the tail: ln of its asymptote sqrt(2/pi) (x + 1/x) e^(-x^2/2), which does not underflow
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
