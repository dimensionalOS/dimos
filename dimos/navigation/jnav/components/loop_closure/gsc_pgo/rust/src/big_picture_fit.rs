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

//! A pose graph over keyframe poses: Levenberg-Marquardt on a sparse Cholesky, with graduated non-convexity (truncated least squares) deciding which closures are inliers by
//! how well they agree with the odometry and with each other, then a Huber pass on the kept ones.

use faer::dyn_stack::{MemBuffer, MemStack};
use faer::linalg::solvers::SolveCore;
use faer::sparse::linalg::cholesky::{
    factorize_symbolic_cholesky, CholeskySymbolicParams, SymbolicCholeskyRaw, SymmetricOrdering,
};
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::linalg::SupernodalThreshold;
use faer::sparse::{Argsort, Pair, SparseColMat, SymbolicSparseColMat};
use faer::{Conj, Mat, Par, Side};

use crate::gsc_pgo::{CommittedClosure, Config, KeyPoseWithCloud};
use crate::landmarks::{self, Sighting};
use crate::mat3::{self, Mat3, Vec3};

pub use crate::landmarks::{bearing_and_range, LandmarkSighting};
pub use crate::world_positions::{solve_with_priors, PositionPrior};

/// A relative-pose measurement from keyframe `from` to keyframe `to`, in `from`'s body frame
/// (R_to = R_from * rotation, p_to = p_from + R_from * translation), with variances for the rotation
/// (about x, y, z, radians²) and the translation (x, y, z, m²).
#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
    pub rotation: Mat3,
    pub translation: Vec3,
    pub variances: [f64; 6],
}

#[derive(Clone)]
pub struct SolveConfig {
    /// Closure variances are multiplied by this while GNC decides inliers (looser, so drift alone is not an outlier).
    pub gnc_variance_scale: f64,
    /// Probability that an inlier closure's residual is within GNC's threshold (sets the chi-square cutoff).
    pub gnc_inlier_probability: f64,
    /// Huber threshold on a kept closure's whitened residual; 0 = plain least squares.
    pub huber_k: f64,
    /// Per closure: true where the previous adopted solve kept it, so GNC holds it in (as PCM's consistent set) instead
    /// of re-deciding it cold; empty for none. A committed loop is dropped only by a verdict that contradicts it
    /// (`closure_commitments`, decided by the caller).
    pub committed: Vec<bool>,
    /// The noise evidence reads only the kept closures' PCM core (`odometry_error_estimate_evidence_core`).
    pub evidence_core: bool,
}

pub struct Solution {
    pub rotations: Vec<Mat3>,
    pub positions: Vec<Vec3>,
    /// Per closure: 1 kept, 0 rejected (GNC's final weight, rounded at 0.5).
    pub closure_weights: Vec<f64>,
    /// Per position prior (`solve_with_priors`): 1 kept, 0 rejected; and per world frame the priors made observable,
    /// its map -> frame alignment (frame id, yaw about z, then translation).
    pub prior_weights: Vec<f64>,
    pub world_alignment: Vec<(u32, f64, Vec3)>,
    /// Per landmark (`LandmarkSighting::landmark`), where the solve put it (and its rotation, identity for a landmark
    /// no sighting gave one); per sighting, 1 kept, 0 rejected.
    pub landmarks: Vec<Vec3>,
    pub landmark_rotations: Vec<Mat3>,
    pub sighting_weights: Vec<f64>,
    /// Half the sum of squared whitened residuals of everything kept.
    pub error: f64,
    /// The newest keyframe's position covariance as seen from each keyframe (its offset in that keyframe's frame, in
    /// world axes, m²), from the solved graph; empty if the graph could not be factored.
    pub relative_position_covariance: Vec<Mat3>,
    /// Variance-component evidence on the odometry error estimate from the solved graph (`odometry_error_estimate_evidence`): per source
    /// (xy, z, yaw, roll/pitch, then the kept closures' rotation and translation), the sum of its residuals squared and whitened, and its
    /// redundancy; None if the graph could not be factored.
    pub odometry_error_estimate_evidence: Option<[(f64, f64); 6]>,
}

/// Solve for the pose of every keyframe marked `free`, from the starting estimate `rotations` /
/// `positions`; the rest are held where they are (at least one must be, to pin the map).
pub fn solve(
    rotations: &[Mat3],
    positions: &[Vec3],
    free: &[bool],
    odometry: &[Edge],
    closures: &[Edge],
    config: &SolveConfig,
) -> Solution {
    solve_core(
        rotations,
        positions,
        free,
        odometry,
        closures,
        &[],
        &[],
        config,
    )
}

/// A position prior placed in the solve frame: keyframe `keyframe`'s sensor at `lever` (body frame) should be at
/// `position`, with information (inverse covariance) `information`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Unary {
    pub(crate) keyframe: usize,
    pub(crate) lever: Vec3,
    pub(crate) position: Vec3,
    pub(crate) information: Mat3,
}

fn unary_residual(u: &Unary, state: &State) -> Vec3 {
    let sensor = mat3::add(
        &state.position[u.keyframe],
        &mat3::mat_vec(&state.rotation[u.keyframe], &u.lever),
    );
    mat3::sub(&sensor, &u.position)
}

fn unary_whitened(u: &Unary, state: &State) -> f64 {
    let r = unary_residual(u, state);
    let ir = mat3::mat_vec(&u.information, &r);
    r[0] * ir[0] + r[1] * ir[1] + r[2] * ir[2]
}

pub(crate) fn solve_core(
    rotations: &[Mat3],
    positions: &[Vec3],
    free: &[bool],
    odometry: &[Edge],
    closures: &[Edge],
    unary: &[Unary],
    sightings: &[LandmarkSighting],
    config: &SolveConfig,
) -> Solution {
    // each landmark starts where its first sighting puts it from the starting estimate
    let landmark_count = sightings.iter().map(|s| s.landmark + 1).max().unwrap_or(0);
    let mut landmark: Vec<Option<(Mat3, Vec3)>> = vec![None; landmark_count];
    for s in sightings {
        landmark[s.landmark].get_or_insert_with(|| {
            (
                mat3::mat_mul(&rotations[s.keyframe], &s.rotation),
                mat3::add(
                    &positions[s.keyframe],
                    &mat3::mat_vec(&rotations[s.keyframe], &s.position),
                ),
            )
        });
    }
    let sightings: Vec<Sighting> = sightings.iter().map(Sighting::new).collect();
    // a landmark is a pose once any sighting knows its rotation, else a point
    let mut landmark_dims = vec![3; landmark_count];
    for s in sightings.iter().filter(|s| s.knows_rotation()) {
        landmark_dims[s.landmark] = 6;
    }
    let (landmark_rotation, landmark): (Vec<Mat3>, Vec<Vec3>) = landmark
        .into_iter()
        .zip(&landmark_dims)
        .map(|(l, &dims)| match l {
            Some((r, p)) if dims == 6 => (r, p),
            Some((_, p)) => (mat3::identity(), p),
            None => (mat3::identity(), [0.0; 3]),
        })
        .unzip();
    let mut state = State {
        rotation: rotations.to_vec(),
        position: positions.to_vec(),
        landmark,
        landmark_rotation,
    };
    let sightings = &sightings[..];
    let scaled: Vec<Edge> = closures
        .iter()
        .map(|e| Edge {
            variances: e.variances.map(|v| v * config.gnc_variance_scale),
            ..*e
        })
        .collect();
    let pattern = Pattern::new(
        free,
        odometry.iter().chain(closures),
        unary,
        sightings,
        &landmark_dims,
    );
    // GNC with truncated least squares (Yang et al. 2020): start convex (every closure in), then tighten
    // until each closure is clearly in or out. Odometry is always in.
    let threshold = chi2_inverse_6dof(config.gnc_inlier_probability);
    let residual =
        |state: &State| -> Vec<f64> { scaled.iter().map(|e| whitened_squared(e, state)).collect() };
    // GNC's starting mu comes from the residuals at the starting estimate, before any solve (as GTSAM's
    // GncOptimizer does): the worse the start, the more gently it tightens
    let worst = residual(&state).into_iter().fold(0.0, f64::max);
    let mut weights = vec![1.0; closures.len()];
    // position priors: GNC-weighted the same way, against the 3-DoF cutoff
    let unary_threshold = crate::loop_overlap::chi2_inverse_3dof(config.gnc_inlier_probability);
    let unary_residuals =
        |state: &State| -> Vec<f64> { unary.iter().map(|u| unary_whitened(u, state)).collect() };
    let unary_worst = unary_residuals(&state).into_iter().fold(0.0, f64::max);
    let mut unary_weights = vec![1.0; unary.len()];
    // landmark sightings: the same, each against the cutoff for its known dimensions (a sighting matched to the wrong
    // landmark is an outlier); residuals kept as fractions of that cutoff
    let sighting_thresholds: Vec<f64> = sightings
        .iter()
        .map(|s| landmarks::chi2_inverse(config.gnc_inlier_probability, s.known.max(1)))
        .collect();
    let sighting_residuals = |state: &State| -> Vec<f64> {
        sightings
            .iter()
            .zip(&sighting_thresholds)
            .map(|(s, t)| s.whitened(state) / t)
            .collect()
    };
    let sighting_worst = sighting_residuals(&state).into_iter().fold(0.0, f64::max);
    let mut sighting_weights = vec![1.0; sightings.len()];
    let mut held = vec![false; closures.len()];
    for (h, &c) in held.iter_mut().zip(&config.committed) {
        *h |= c;
    }
    levenberg_marquardt(
        &mut state,
        &pattern,
        odometry,
        &scaled,
        &weights,
        unary,
        &unary_weights,
        sightings,
        &sighting_weights,
        1e-5,
    );
    let mut r2 = residual(&state);
    let mut u2 = unary_residuals(&state);
    let mut s2 = sighting_residuals(&state);
    if worst > threshold || unary_worst > unary_threshold || sighting_worst > 1.0 {
        // the gentlest start any factor kind that exceeds its own cutoff needs (a kind under its cutoff sets none)
        let start = |worst: f64, threshold: f64| {
            (worst > threshold).then(|| threshold / (2.0 * worst - threshold))
        };
        let mut mu = [
            start(worst, threshold),
            start(unary_worst, unary_threshold),
            start(sighting_worst, 1.0),
        ]
        .into_iter()
        .flatten()
        .fold(f64::INFINITY, f64::min)
        .max(f64::EPSILON);
        let tls = |r: f64, threshold: f64, mu: f64| {
            let (upper, lower) = ((mu + 1.0) / mu * threshold, mu / (mu + 1.0) * threshold);
            if r >= upper {
                0.0
            } else if r <= lower {
                1.0
            } else {
                ((threshold * mu * (mu + 1.0) / r).sqrt() - mu).clamp(0.0, 1.0)
            }
        };
        let mut previous_cost = f64::INFINITY;
        for _ in 0..100 {
            for ((w, &r), &held) in weights.iter_mut().zip(&r2).zip(&held) {
                if held {
                    continue;
                }
                *w = tls(r, threshold, mu);
            }
            for (w, &r) in unary_weights.iter_mut().zip(&u2) {
                *w = tls(r, unary_threshold, mu);
            }
            for (w, &r) in sighting_weights.iter_mut().zip(&s2) {
                *w = tls(r, 1.0, mu);
            }
            levenberg_marquardt(
                &mut state,
                &pattern,
                odometry,
                &scaled,
                &weights,
                unary,
                &unary_weights,
                sightings,
                &sighting_weights,
                1e-5,
            );
            r2 = residual(&state);
            u2 = unary_residuals(&state);
            s2 = sighting_residuals(&state);
            // stop like GTSAM's GncOptimizer: weights settled, or the weighted cost stopped changing
            let weighted_cost = cost(
                &state,
                odometry,
                &scaled,
                &weights,
                unary,
                &unary_weights,
                sightings,
                &sighting_weights,
            );
            if weights
                .iter()
                .chain(&unary_weights)
                .chain(&sighting_weights)
                .all(|&w| !(1e-4..=1.0 - 1e-4).contains(&w))
                || (previous_cost - weighted_cost).abs() <= 1e-5 * weighted_cost
            {
                break;
            }
            previous_cost = weighted_cost;
            mu *= 1.4;
        }
    }
    let kept: Vec<f64> = weights
        .iter()
        .map(|&w| if w >= 0.5 { 1.0 } else { 0.0 })
        .collect();
    let unary_kept: Vec<f64> = unary_weights
        .iter()
        .map(|&w| if w >= 0.5 { 1.0 } else { 0.0 })
        .collect();
    let sighting_kept: Vec<f64> = sighting_weights
        .iter()
        .map(|&w| if w >= 0.5 { 1.0 } else { 0.0 })
        .collect();

    // the kept closures at their own variances, Huber-weighted
    let mut huber = kept.clone();
    for _ in 0..3 {
        levenberg_marquardt(
            &mut state,
            &pattern,
            odometry,
            closures,
            &huber,
            unary,
            &unary_kept,
            sightings,
            &sighting_kept,
            1e-5,
        );
        for (h, (e, &k)) in huber.iter_mut().zip(closures.iter().zip(&kept)) {
            let r = whitened_squared(e, &state).sqrt();
            *h = if config.huber_k > 0.0 && r > config.huber_k {
                k * config.huber_k / r
            } else {
                k
            };
        }
    }
    let error = 0.5
        * cost(
            &state,
            odometry,
            closures,
            &huber,
            unary,
            &unary_kept,
            sightings,
            &sighting_kept,
        );
    let relative_position_covariance = relative_position_covariances(
        &state,
        &pattern,
        odometry,
        closures,
        &huber,
        unary,
        &unary_kept,
        sightings,
        &sighting_kept,
        positions.len() - 1,
    )
    .unwrap_or_default();
    // only the kept closures' mutually consistent core (the largest set that agree pairwise around their cycles, PCM)
    // speaks for the noise: a kept closure the others contradict would otherwise read as odometry error estimate and widen it
    let evidence_weights: Vec<f64> = if config.evidence_core {
        let kept_closures: Vec<(usize, Edge)> = closures
            .iter()
            .zip(&huber)
            .enumerate()
            .filter(|(_, (_, &w))| w > 0.0)
            .map(|(i, (e, _))| (i, *e))
            .collect();
        let edges: Vec<Edge> = kept_closures.iter().map(|&(_, e)| e).collect();
        let consistent = consistent_closures(&edges, odometry, config);
        let mut weights = vec![0.0; closures.len()];
        for ((i, _), held) in kept_closures.iter().zip(consistent) {
            if held {
                weights[*i] = huber[*i];
            }
        }
        weights
    } else {
        huber.clone()
    };
    // the evidence reads the solve of what it weighs (the odometry at its noise, the core's closures), and only at
    // that solve's minimum: residuals a solve stopped short of removing are no one's noise
    let mut evidence_state = state.clone();
    let converged = levenberg_marquardt_steps(
        &mut evidence_state,
        &pattern,
        odometry,
        closures,
        &evidence_weights,
        unary,
        &unary_kept,
        sightings,
        &sighting_kept,
        1e-5,
        // a bound on time only: from the solve's state this is near its minimum, but a flat direction can creep
        1000,
        true,
    );
    let odometry_error_estimate_evidence = converged
        .then(|| {
            odometry_error_estimate_evidence(
                &evidence_state,
                &pattern,
                odometry,
                closures,
                &evidence_weights,
                unary,
                &unary_kept,
                sightings,
                &sighting_kept,
            )
        })
        .flatten();
    Solution {
        odometry_error_estimate_evidence,
        rotations: state.rotation,
        positions: state.position,
        closure_weights: kept,
        prior_weights: unary_kept,
        world_alignment: Vec::new(),
        landmarks: state.landmark,
        landmark_rotations: state.landmark_rotation,
        sighting_weights: sighting_kept,
        error,
        relative_position_covariance,
    }
}

/// The noise source (xy, z, yaw, roll/pitch) of each component of an odometry edge's residual [rotation x, y, z,
/// translation x, y, z].
const RESIDUAL_SOURCE: [usize; 6] = [3, 3, 2, 0, 0, 1];

/// Variance-component evidence on the odometry error estimate from the solved graph (Förstner's estimator): for each noise source,
/// how far the solve had to bend its parts of the odometry edges (their residuals squared, whitened by the variances
/// the solve used) and how many independent checks the graph gave it, its redundancy r_c = n_c - tr(N^-1 N_c) (n_c its
/// residual count; N the solve's information matrix, N_c the source's own share of it). A source's variance times
/// sum / r_c is the variance that explains the residuals. The odometry alone determines the graph exactly (a chain),
/// so n_c = tr(N_odo^-1 N_c) and r_c = tr((N_odo^-1 - N^-1) N_c): the difference has rank at most the kept constraints',
/// so Hutchinson's estimator (the mean of z^T (N_odo^-1 - N^-1) N_c z over fixed random +-1 vectors z, two solves each)
/// is accurate to a few percent of r_c, and exactly 0 with nothing kept. None if a matrix cannot be factored.
#[allow(clippy::too_many_arguments)]
fn odometry_error_estimate_evidence(
    state: &State,
    pattern: &Pattern,
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
) -> Option<[(f64, f64); 6]> {
    let size = pattern.size;
    let factor = |cw: &[f64], uw: &[f64], sw: &[f64]| {
        let (mut values, _, diagonal) = normal_equations(
            state, pattern, odometry, closures, cw, unary, uw, sightings, sw,
        );
        values.extend(diagonal.iter().map(|d| 1e-9 * (d + 1e-9)));
        let hessian =
            SparseColMat::new_from_argsort(pattern.symbolic.clone(), &pattern.order, &values)
                .ok()?;
        Llt::try_new_with_symbolic(pattern.llt.clone(), hessian.as_ref(), Side::Lower).ok()
    };
    let full = factor(closure_weights, unary_weights, sighting_weights)?;
    // the odometry alone: everything else at weight 0 (keeping the pattern's fill order)
    let odometry_only = factor(
        &vec![0.0; closures.len()],
        &vec![0.0; unary.len()],
        &vec![0.0; sightings.len()],
    )?;
    let linear: Vec<([f64; 6], [[f64; 6]; 6], [[f64; 6]; 6])> =
        odometry.iter().map(|e| linearize(e, state)).collect();
    let mut evidence = [(0.0, 0.0); 6];
    for (e, (r, _, _)) in odometry.iter().zip(&linear) {
        for k in 0..6 {
            evidence[RESIDUAL_SOURCE[k]].0 += r[k] * r[k] / e.variances[k];
        }
    }
    // a deterministic +-1 sequence (the trace is an expectation over any such sequence)
    let sign = |probe: usize, i: usize| {
        let x = ((probe * 7919 + i * 104729) as f64 * 12.9898).sin() * 43758.5453;
        if x.rem_euclid(1.0) < 0.5 {
            -1.0
        } else {
            1.0
        }
    };
    // r_c = tr(K_c D K_c^T) with K_c the source's whitened odometry rows (N_c = K_c^T K_c) and D = N_odo^-1 - N^-1:
    // symmetric, eigenvalues in [0, 1], rank at most the kept constraints', so +-1 probes over those rows estimate it
    // to about sqrt(2 r_c / probes)
    const PROBES: usize = 32;
    for source in 0..4 {
        let rows: Vec<(usize, usize)> = (0..odometry.len())
            .flat_map(|e| (0..6).map(move |k| (e, k)))
            .filter(|&(_, k)| RESIDUAL_SOURCE[k] == source)
            .collect();
        for probe in 0..PROBES {
            // u = K_c^T w
            let mut u = Mat::<f64>::zeros(size, 1);
            for (index, &(e, k)) in rows.iter().enumerate() {
                let edge = &odometry[e];
                let weight = sign(probe * 4 + source, index) / edge.variances[k].sqrt();
                let (_, jf, jt) = &linear[e];
                for (slot, j) in [(pattern.slot[edge.from], jf), (pattern.slot[edge.to], jt)] {
                    if let Some(slot) = slot {
                        for i in 0..6 {
                            u[(6 * slot + i, 0)] += j[k][i] * weight;
                        }
                    }
                }
            }
            let mut through_odometry = u.clone();
            odometry_only.solve_in_place_with_conj(Conj::No, through_odometry.as_mut());
            let mut through_full = u;
            full.solve_in_place_with_conj(Conj::No, through_full.as_mut());
            // w^T K_c y with y = D u
            let mut estimate = 0.0;
            for (index, &(e, k)) in rows.iter().enumerate() {
                let edge = &odometry[e];
                let weight = sign(probe * 4 + source, index) / edge.variances[k].sqrt();
                let (_, jf, jt) = &linear[e];
                let mut jy = 0.0;
                for (slot, j) in [(pattern.slot[edge.from], jf), (pattern.slot[edge.to], jt)] {
                    if let Some(slot) = slot {
                        for i in 0..6 {
                            let at = 6 * slot + i;
                            jy += j[k][i] * (through_odometry[(at, 0)] - through_full[(at, 0)]);
                        }
                    }
                }
                estimate += weight * jy;
            }
            evidence[source].1 += estimate / PROBES as f64;
        }
    }
    // the kept closures' rotation and translation as two more sources: sum of their whitened residuals squared, and redundancy
    // r = n - tr(K N^-1 K^T) = tr(M), M = I - K N^-1 K^T (K their weighted whitened rows): symmetric, eigenvalues in
    // [0, 1], so the same +-1 probes estimate it to about sqrt(2 r / probes). With it, a closure variance reported too
    // loose shows as closure residuals too small for their redundancy, instead of passing as odometry too good
    let closure_linear: Vec<(usize, [f64; 6], [[f64; 6]; 6], [[f64; 6]; 6])> = closures
        .iter()
        .enumerate()
        .filter(|&(i, _)| closure_weights[i] > 0.0)
        .map(|(i, e)| {
            let (r, jf, jt) = linearize(e, state);
            (i, r, jf, jt)
        })
        .collect();
    for &(i, r, _, _) in &closure_linear {
        let e = &closures[i];
        for k in 0..6 {
            evidence[4 + k / 3].0 += closure_weights[i] * r[k] * r[k] / e.variances[k];
        }
    }
    for group in 0..2 {
        let rows: Vec<(usize, usize)> = (0..closure_linear.len())
            .flat_map(|c| (3 * group..3 * group + 3).map(move |k| (c, k)))
            .collect();
        let row_weight = |c: usize, k: usize| {
            let (i, ..) = closure_linear[c];
            (closure_weights[i] / closures[i].variances[k]).sqrt()
        };
        for probe in 0..PROBES {
            let mut u = Mat::<f64>::zeros(size, 1);
            let mut norm = 0.0;
            for (index, &(c, k)) in rows.iter().enumerate() {
                let (i, _, jf, jt) = &closure_linear[c];
                let edge = &closures[*i];
                let s = sign(probe * 4 + 4 + group, index);
                norm += s * s;
                let weight = s * row_weight(c, k);
                for (slot, j) in [(pattern.slot[edge.from], jf), (pattern.slot[edge.to], jt)] {
                    if let Some(slot) = slot {
                        for m in 0..6 {
                            u[(6 * slot + m, 0)] += j[k][m] * weight;
                        }
                    }
                }
            }
            let mut y = u;
            full.solve_in_place_with_conj(Conj::No, y.as_mut());
            let mut projected = 0.0;
            for (index, &(c, k)) in rows.iter().enumerate() {
                let (i, _, jf, jt) = &closure_linear[c];
                let edge = &closures[*i];
                let weight = sign(probe * 4 + 4 + group, index) * row_weight(c, k);
                let mut jy = 0.0;
                for (slot, j) in [(pattern.slot[edge.from], jf), (pattern.slot[edge.to], jt)] {
                    if let Some(slot) = slot {
                        for m in 0..6 {
                            jy += j[k][m] * y[(6 * slot + m, 0)];
                        }
                    }
                }
                projected += weight * jy;
            }
            evidence[4 + group].1 += (norm - projected) / PROBES as f64;
        }
    }
    Some(evidence)
}

/// The largest set of closures that pairwise agree (PCM, Mangelson 2018): two closures agree when the cycle they close
/// with the raw odometry (A.from through A to A.to, against A.from along the odometry to B.from, through B to B.to,
/// along the odometry to A.to) lands A.to where A says, within the chi-square cutoff of `gnc_inlier_probability` (3
/// DOF). The cycle's covariance is the first-order propagation of every noise on it at the variances its edges carry:
/// both closures' translations, B's rotation swinging the path after it, and each odometry step's translation plus its
/// rotation swinging the path's end about it (a heading error reaches as far as the loop is from where it was made).
/// The set is grown greedily from the best-agreeing closure (an approximate maximum clique); a set of one is no
/// evidence of agreement, so it holds nothing.
pub fn consistent_closures(
    closures: &[Edge],
    odometry: &[Edge],
    config: &SolveConfig,
) -> Vec<bool> {
    let n = closures.len();
    let cutoff = crate::loop_overlap::chi2_inverse_3dof(config.gnc_inlier_probability);
    type Pose = (Mat3, Vec3);
    let compose = |a: &Pose, b: &Pose| -> Pose {
        (
            mat3::mat_mul(&a.0, &b.0),
            mat3::add(&a.1, &mat3::mat_vec(&a.0, &b.1)),
        )
    };
    let invert = |a: &Pose| -> Pose {
        let back = mat3::transpose(&a.0);
        (back, mat3::scale(&mat3::mat_vec(&back, &a.1), -1.0))
    };
    let world = |rotation: &Mat3, variances: &[f64]| -> Mat3 {
        let scaled: Mat3 =
            std::array::from_fn(|i| std::array::from_fn(|j| rotation[i][j] * variances[j]));
        mat3::mat_mul(&scaled, &mat3::transpose(rotation))
    };
    let around = |lever: &Vec3, covariance: &Mat3| -> Mat3 {
        let skew = mat3::skew(lever);
        mat3::mat_mul(&mat3::mat_mul(&skew, covariance), &mat3::transpose(&skew))
    };
    // the raw odometry chain's poses, and prefix sums over its steps (step k from keyframe k to k + 1) of, in the
    // chain's frame, the translation covariance T, the rotation covariance A, A P^T and P A P^T (P = skew of the step's
    // end), so a stretch's covariance at a point q, sum T + [q - p]x A [q - p]x^T, is
    // Q S_A Q^T - Q S_AP - (Q S_AP)^T + S_PAP + S_T
    let keyframes = odometry.iter().map(|e| e.to + 1).max().unwrap_or(0);
    let mut chain: Vec<Pose> = vec![(mat3::identity(), [0.0; 3]); keyframes];
    let mut steps = vec![[[[0.0; 3]; 3]; 4]; keyframes];
    let mut by_from: Vec<Option<&Edge>> = vec![None; keyframes];
    for e in odometry.iter().filter(|e| e.to == e.from + 1) {
        by_from[e.from] = Some(e);
    }
    for k in 1..keyframes {
        let Some(e) = by_from[k - 1] else {
            chain[k] = chain[k - 1];
            steps[k] = steps[k - 1];
            continue;
        };
        chain[k] = compose(&chain[k - 1], &(e.rotation, e.translation));
        let rotation = &chain[k - 1].0;
        let a = world(rotation, &e.variances[..3]);
        let p = mat3::skew(&chain[k].1);
        let ap = mat3::mat_mul(&a, &mat3::transpose(&p));
        let step = [
            world(rotation, &e.variances[3..]),
            a,
            ap,
            mat3::mat_mul(&p, &ap),
        ];
        steps[k] = std::array::from_fn(|m| mat3::add_mat(&steps[k - 1][m], &step[m]));
    }
    let stretch = |a: usize, b: usize, q: &Vec3| -> Mat3 {
        let (low, high) = (a.min(b), a.max(b));
        let part = |m: usize| mat3::sub_mat(&steps[high][m], &steps[low][m]);
        let skew_q = mat3::skew(q);
        let q_ap = mat3::mat_mul(&skew_q, &part(2));
        let q_a_q = mat3::mat_mul(&mat3::mat_mul(&skew_q, &part(1)), &mat3::transpose(&skew_q));
        let lever = mat3::sub_mat(&mat3::sub_mat(&q_a_q, &q_ap), &mat3::transpose(&q_ap));
        mat3::add_mat(&mat3::add_mat(&lever, &part(3)), &part(0))
    };
    let agree = |a: &Edge, b: &Edge| -> bool {
        if [a.from, a.to, b.from, b.to].iter().any(|&k| k >= keyframes) {
            return false;
        }
        let through_a = compose(&chain[a.from], &(a.rotation, a.translation));
        let b_to = compose(&chain[b.from], &(b.rotation, b.translation));
        // the chain after B.to, laid where B puts B.to
        let laid = compose(&b_to, &invert(&chain[b.to]));
        let through_b = compose(&laid, &chain[a.to]);
        let q = through_b.1;
        let after_b = stretch(
            b.to,
            a.to,
            &mat3::mat_vec(&invert(&laid).0, &mat3::sub(&q, &laid.1)),
        );
        let covariance = [
            world(&chain[a.from].0, &a.variances[3..]),
            world(&chain[b.from].0, &b.variances[3..]),
            around(&mat3::sub(&q, &b_to.1), &world(&b_to.0, &b.variances[..3])),
            stretch(a.from, b.from, &q),
            mat3::mat_mul(&mat3::mat_mul(&laid.0, &after_b), &mat3::transpose(&laid.0)),
        ]
        .iter()
        .fold([[0.0; 3]; 3], |sum, m| mat3::add_mat(&sum, m));
        let d = mat3::sub(&through_a.1, &q);
        // inverted at unit trace: the variances are far below mat3::inverse's absolute determinant floor
        let trace = covariance[0][0] + covariance[1][1] + covariance[2][2];
        let unit: Mat3 = std::array::from_fn(|i| std::array::from_fn(|j| covariance[i][j] / trace));
        trace > 0.0
            && mat3::inverse(&unit).is_some_and(|information| {
                let id = mat3::mat_vec(&information, &d);
                (d[0] * id[0] + d[1] * id[1] + d[2] * id[2]) / trace < cutoff
            })
    };
    let pairs: Vec<Vec<bool>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| i == j || agree(&closures[i], &closures[j]))
                .collect()
        })
        .collect();
    let degree: Vec<usize> = pairs
        .iter()
        .map(|row| row.iter().filter(|&&x| x).count())
        .collect();
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&x, &y| degree[y].cmp(&degree[x]));
    let mut members: Vec<usize> = Vec::new();
    for &i in &order {
        if members.iter().all(|&m| pairs[i][m] && pairs[m][i]) {
            members.push(i);
        }
    }
    let mut held = vec![false; n];
    if members.len() > 1 {
        for m in members {
            held[m] = true;
        }
    }
    held
}

/// The covariance of keyframe `tip`'s position as seen from each keyframe (its offset in that keyframe's own frame, in world
/// axes, m²) at `state`, so the map's overall heading uncertainty does not count. Σ, the inverse of the normal equations'
/// matrix, is needed at each keyframe's own 6x6 block, from the Takahashi recursion over its sparse Cholesky factor (the
/// entries the factor's pattern holds, one backward pass), and at its cross blocks with the tip's position, from one solve
/// against the tip's position columns. A held keyframe has no uncertainty of its own.
fn relative_position_covariances(
    state: &State,
    pattern: &Pattern,
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
    tip: usize,
) -> Option<Vec<Mat3>> {
    let size = pattern.size;
    let (mut values, _, diagonal) = normal_equations(
        state,
        pattern,
        odometry,
        closures,
        closure_weights,
        unary,
        unary_weights,
        sightings,
        sighting_weights,
    );
    // the pattern's damping diagonal, at Levenberg-Marquardt's smallest damping
    values.extend(diagonal.iter().map(|d| 1e-9 * (d + 1e-9)));
    let hessian =
        SparseColMat::new_from_argsort(pattern.symbolic.clone(), &pattern.order, &values).ok()?;
    let symbolic = factorize_symbolic_cholesky(
        hessian.symbolic(),
        Side::Lower,
        SymmetricOrdering::Amd,
        CholeskySymbolicParams {
            // the recursion walks the factor column by column
            supernodal_flop_ratio_threshold: SupernodalThreshold::FORCE_SIMPLICIAL,
            ..Default::default()
        },
    )
    .ok()?;
    let mut factor = vec![0.0; symbolic.len_val()];
    let par = Par::Seq;
    let tip_slot = pattern.slot[tip]?;
    // Σ's position columns of the tip: one solve with the tip's three position unit vectors
    let mut tip_columns =
        Mat::<f64>::from_fn(size, 3, |i, c| (i == 6 * tip_slot + 3 + c) as u8 as f64);
    {
        let llt = symbolic
            .factorize_numeric_llt(
                &mut factor,
                hessian.as_ref(),
                Side::Lower,
                Default::default(),
                par,
                MemStack::new(&mut MemBuffer::new(
                    symbolic.factorize_numeric_llt_scratch::<f64>(par, Default::default()),
                )),
                Default::default(),
            )
            .ok()?;
        llt.solve_in_place_with_conj(
            Conj::No,
            tip_columns.as_mut(),
            par,
            MemStack::new(&mut MemBuffer::new(
                llt.solve_in_place_scratch::<f64>(3, par),
            )),
        );
    }
    let SymbolicCholeskyRaw::Simplicial(simplicial) = symbolic.raw() else {
        return None;
    };
    let (col_ptr, row_idx) = (simplicial.col_ptr(), simplicial.row_idx());
    let (_, inverse_order) = symbolic.perm()?.arrays();
    // Σ in the factor's order, at the factor's pattern: column j from the columns after it
    let mut sigma = vec![0.0; factor.len()];
    let at = |sigma: &[f64], row: usize, col: usize| -> f64 {
        let (row, col) = (row.max(col), row.min(col));
        let span = col_ptr[col]..col_ptr[col + 1];
        row_idx[span.clone()]
            .binary_search(&row)
            .map_or(0.0, |k| sigma[span.start + k])
    };
    for j in (0..size).rev() {
        let (first, end) = (col_ptr[j], col_ptr[j + 1]);
        let pivot = factor[first];
        for p in first + 1..end {
            let i = row_idx[p];
            let sum: f64 = (first + 1..end)
                .map(|q| factor[q] * at(&sigma, i, row_idx[q]))
                .sum();
            sigma[p] = -sum / pivot;
        }
        let sum: f64 = (first + 1..end).map(|q| factor[q] * sigma[q]).sum();
        sigma[first] = (1.0 / pivot - sum) / pivot;
    }
    let entry = |slot: usize, u: usize, v: usize| {
        at(
            &sigma,
            inverse_order[6 * slot + u],
            inverse_order[6 * slot + v],
        )
    };
    let tip_block: Mat3 =
        std::array::from_fn(|u| std::array::from_fn(|v| tip_columns[(6 * tip_slot + 3 + u, v)]));
    let mul = |a: &Mat3, b: &Mat3| mat3::mat_mul(a, b);
    let t = mat3::transpose;
    Some(
        pattern
            .slot
            .iter()
            .enumerate()
            .map(|(k, slot)| {
                let Some(s) = *slot else { return tip_block };
                // the tip's offset in keyframe k's own frame, turned back to world axes:
                // δ = δp_tip - δp_k + skew(p_tip - p_k) R_k δθ_k (k's own heading uncertainty does not move the tip
                // relative to k)
                let lever = mul(
                    &mat3::skew(&mat3::sub(&state.position[tip], &state.position[k])),
                    &state.rotation[k],
                );
                let block = |r: usize, c: usize| -> Mat3 {
                    std::array::from_fn(|u| std::array::from_fn(|v| entry(s, r + u, c + v)))
                };
                let with_tip = |r: usize| -> Mat3 {
                    std::array::from_fn(|u| {
                        std::array::from_fn(|v| tip_columns[(6 * s + r + u, v)])
                    })
                };
                let (turn_turn, turn_shift, shift_shift) = (block(0, 0), block(0, 3), block(3, 3));
                let (turn_tip, shift_tip) = (with_tip(0), with_tip(3));
                let swing_tip = mul(&lever, &turn_tip);
                let swing_shift = mul(&lever, &turn_shift);
                let swing = mul(&mul(&lever, &turn_turn), &t(&lever));
                std::array::from_fn(|u| {
                    std::array::from_fn(|v| {
                        tip_block[u][v] + shift_shift[u][v] - shift_tip[u][v] - shift_tip[v][u]
                            + swing[u][v]
                            + swing_tip[u][v]
                            + swing_tip[v][u]
                            - swing_shift[u][v]
                            - swing_shift[v][u]
                    })
                })
            })
            .collect(),
    )
}

#[derive(Clone)]
pub(crate) struct State {
    pub(crate) rotation: Vec<Mat3>,
    pub(crate) position: Vec<Vec3>,
    pub(crate) landmark: Vec<Vec3>,
    pub(crate) landmark_rotation: Vec<Mat3>,
}

impl State {
    /// Keyframe `k` moved by `delta`: rotation about its own axes by delta[0..3], position by delta[3..6].
    fn retract(&self, k: usize, delta: &[f64]) -> (Mat3, Vec3) {
        let rotation = mat3::mat_mul(&self.rotation[k], &exp([delta[0], delta[1], delta[2]]));
        (
            rotation,
            mat3::add(&self.position[k], &[delta[3], delta[4], delta[5]]),
        )
    }
}

/// [rotation error (axis-angle), translation error] of an edge between two poses.
pub(crate) fn edge_residual(e: &Edge, from: (&Mat3, &Vec3), to: (&Mat3, &Vec3)) -> [f64; 6] {
    let predicted = mat3::mat_mul(from.0, &e.rotation);
    let rotation = log(&mat3::mat_mul(&mat3::transpose(&predicted), to.0));
    let local = mat3::mat_vec(&mat3::transpose(from.0), &mat3::sub(to.1, from.1));
    let translation = mat3::sub(&local, &e.translation);
    [
        rotation[0],
        rotation[1],
        rotation[2],
        translation[0],
        translation[1],
        translation[2],
    ]
}

fn whitened_squared(e: &Edge, state: &State) -> f64 {
    let r = edge_residual(
        e,
        (&state.rotation[e.from], &state.position[e.from]),
        (&state.rotation[e.to], &state.position[e.to]),
    );
    (0..6).map(|k| r[k] * r[k] / e.variances[k]).sum()
}

/// The residual and its Jacobians with respect to `from`'s and `to`'s 6-vector perturbations
/// (rotation about the keyframe's own axes, then world position).
fn linearize(e: &Edge, state: &State) -> ([f64; 6], [[f64; 6]; 6], [[f64; 6]; 6]) {
    linearize_between(
        e,
        (&state.rotation[e.from], &state.position[e.from]),
        (&state.rotation[e.to], &state.position[e.to]),
    )
}

/// `linearize` between two given poses.
pub(crate) fn linearize_between(
    e: &Edge,
    (ri, pi): (&Mat3, &Vec3),
    (rj, pj): (&Mat3, &Vec3),
) -> ([f64; 6], [[f64; 6]; 6], [[f64; 6]; 6]) {
    let residual = edge_residual(e, (ri, pi), (rj, pj));
    let ri_t = mat3::transpose(ri);
    let local = mat3::mat_vec(&ri_t, &mat3::sub(pj, pi));
    // rotation error phi = Log(E), E = (Ri Rm)^T Rj; to first order d phi = Jr^-1 (d theta_j - E^T Rm^T d theta_i),
    // with Jr^-1 ~ I + [phi]x / 2
    let phi = [residual[0], residual[1], residual[2]];
    let mut jr_inv = mat3::skew(&phi);
    for row in jr_inv.iter_mut() {
        for v in row.iter_mut() {
            *v *= 0.5;
        }
    }
    for (d, row) in jr_inv.iter_mut().enumerate() {
        row[d] += 1.0;
    }
    let error = mat3::mat_mul(&mat3::transpose(&mat3::mat_mul(ri, &e.rotation)), rj);
    let back = mat3::mat_mul(
        &jr_inv,
        &mat3::mat_mul(&mat3::transpose(&error), &mat3::transpose(&e.rotation)),
    );
    let skew_local = mat3::skew(&local);
    let (mut jf, mut jt) = ([[0.0; 6]; 6], [[0.0; 6]; 6]);
    for r in 0..3 {
        for c in 0..3 {
            jf[r][c] = -back[r][c];
            jt[r][c] = jr_inv[r][c];
            // translation error Ri^T (pj - pi) - t: d/d theta_i = [Ri^T (pj - pi)]x, d/d pi = -Ri^T, d/d pj = Ri^T
            jf[3 + r][c] = skew_local[r][c];
            jf[3 + r][3 + c] = -ri_t[r][c];
            jt[3 + r][3 + c] = ri_t[r][c];
        }
    }
    (residual, jf, jt)
}

fn cost(
    state: &State,
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
) -> f64 {
    sightings
        .iter()
        .zip(sighting_weights)
        .map(|(s, w)| w * s.whitened(state))
        .sum::<f64>()
        + unary
            .iter()
            .zip(unary_weights)
            .map(|(u, w)| w * unary_whitened(u, state))
            .sum::<f64>()
        + odometry
            .iter()
            .map(|e| whitened_squared(e, state))
            .sum::<f64>()
        + closures
            .iter()
            .zip(closure_weights)
            .map(|(e, w)| w * whitened_squared(e, state))
            .sum::<f64>()
}

/// The normal equations' sparsity: a 6x6 block per free keyframe (`slot`, its place among them; held
/// keyframes have none), with every entry's place fixed once, so each iteration only fills values (in
/// `blocks` order) instead of re-sorting.
struct Pattern {
    slot: Vec<Option<usize>>,
    /// Per landmark, where its block starts (after every keyframe's) and its size: 6 for a pose, 3 for a point.
    landmark_at: Vec<usize>,
    landmark_dims: Vec<usize>,
    size: usize,
    symbolic: SymbolicSparseColMat<usize>,
    order: Argsort<usize>,
    llt: SymbolicLlt<usize>,
}

impl Pattern {
    fn new<'a>(
        free: &[bool],
        edges: impl Iterator<Item = &'a Edge>,
        unary: &[Unary],
        sightings: &[Sighting],
        landmark_dims: &[usize],
    ) -> Pattern {
        let mut count = 0;
        let slot: Vec<Option<usize>> = free
            .iter()
            .map(|&f| f.then(|| (count, count += 1).0))
            .collect();
        let mut size = 6 * count.max(1);
        let landmark_at: Vec<usize> = landmark_dims.iter().map(|&d| (size, size += d).0).collect();
        let mut indices: Vec<Pair<usize, usize>> = Vec::new();
        for e in edges {
            for (_, _, a, b) in blocks(&slot, e) {
                indices.extend(block_entries(a, b).map(|(_, _, r, c)| Pair::new(r, c)));
            }
        }
        // position priors after the edges (their keyframe's own block), in `normal_equations` fill order
        for u in unary {
            if let Some(a) = slot[u.keyframe] {
                indices.extend(block_entries(a, a).map(|(_, _, r, c)| Pair::new(r, c)));
            }
        }
        // landmark sightings after those, in `normal_equations` fill order
        for s in sightings {
            indices.extend(
                s.entries(&slot, landmark_at[s.landmark], landmark_dims[s.landmark])
                    .map(|(_, _, r, c)| Pair::new(r, c)),
            );
        }
        // the damping diagonal last, so the matrix is never structurally singular
        indices.extend((0..size).map(|i| Pair::new(i, i)));
        let (symbolic, order) = SymbolicSparseColMat::try_new_from_indices(size, size, &indices)
            .expect("pose graph pattern");
        let llt =
            SymbolicLlt::try_new(symbolic.as_ref(), Side::Lower).expect("pose graph ordering");
        Pattern {
            slot,
            landmark_at,
            landmark_dims: landmark_dims.to_vec(),
            size,
            symbolic,
            order,
            llt,
        }
    }
}

/// The blocks an edge adds to, in fill order: keyframes (a, b) and their slots, slot a >= slot b.
fn blocks<'a>(
    slot: &'a [Option<usize>],
    e: &Edge,
) -> impl Iterator<Item = (usize, usize, usize, usize)> + 'a {
    let (low, high) = if slot[e.from] <= slot[e.to] {
        (e.from, e.to)
    } else {
        (e.to, e.from)
    };
    [(e.from, e.from), (e.to, e.to), (high, low)]
        .into_iter()
        .filter_map(move |(a, b)| Some((a, b, slot[a]?, slot[b]?)))
}

/// Lower-triangle (row, col) indices of the 6x6 block at slots (a, b).
fn block_entries(a: usize, b: usize) -> impl Iterator<Item = (usize, usize, usize, usize)> {
    (0..6)
        .flat_map(move |i| (0..6).map(move |j| (i, j, 6 * a + i, 6 * b + j)))
        .filter(|&(_, _, r, c)| r >= c)
}

/// `levenberg_marquardt_steps` with at most 100 steps, stopping at the first small drop.
#[allow(clippy::too_many_arguments)]
fn levenberg_marquardt(
    state: &mut State,
    pattern: &Pattern,
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
    tolerance: f64,
) -> bool {
    levenberg_marquardt_steps(
        state,
        pattern,
        odometry,
        closures,
        closure_weights,
        unary,
        unary_weights,
        sightings,
        sighting_weights,
        tolerance,
        100,
        false,
    )
}

/// Gauss-Newton steps, damped (and retried) whenever a step would raise the cost; done once a step lowers the cost
/// (whitened squared residuals) by less than `tolerance` of it plus one, the variance of one residual, so a cost near
/// zero is done instead of shrinking forever, or no step lowers it at all where the quadratic model expects no more
/// than that either. With `to_minimum` a small drop counts only from a step as undamped as the first one tried (a
/// heavily damped step drops little anywhere): the noise evidence must be read at the minimum, but the solves stop at
/// the first small drop, as they always have (holding them to the minimum too moved athens' map 2.5 -> 6.2 cm off the
/// ground truth). False if `steps` steps did not get there, or it stalled short of a minimum.
#[allow(clippy::too_many_arguments)]
fn levenberg_marquardt_steps(
    state: &mut State,
    pattern: &Pattern,
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
    tolerance: f64,
    steps: usize,
    to_minimum: bool,
) -> bool {
    let size = pattern.size;
    let mut damping = 1e-6;
    let mut current = cost(
        state,
        odometry,
        closures,
        closure_weights,
        unary,
        unary_weights,
        sightings,
        sighting_weights,
    );
    for _ in 0..steps {
        let (mut values, gradient, diagonal) = normal_equations(
            state,
            pattern,
            odometry,
            closures,
            closure_weights,
            unary,
            unary_weights,
            sightings,
            sighting_weights,
        );
        let filled = values.len();
        values.resize(filled + size, 0.0);
        let (mut stepped, mut accepted, mut predicted) = (false, false, None);
        while damping < 1e12 {
            for (i, d) in diagonal.iter().enumerate() {
                values[filled + i] = damping * (d + 1e-9);
            }
            let hessian =
                SparseColMat::new_from_argsort(pattern.symbolic.clone(), &pattern.order, &values)
                    .expect("pose graph hessian");
            let Ok(llt) =
                Llt::try_new_with_symbolic(pattern.llt.clone(), hessian.as_ref(), Side::Lower)
            else {
                damping *= 10.0;
                continue;
            };
            let mut step = Mat::<f64>::from_fn(size, 1, |i, _| gradient[i]);
            llt.solve_in_place_with_conj(Conj::No, step.as_mut());
            // the cost the quadratic model says the least damped step would remove, g^T step
            predicted
                .get_or_insert_with(|| (0..size).map(|i| gradient[i] * step[(i, 0)]).sum::<f64>());
            let mut trial = State {
                rotation: state.rotation.clone(),
                position: state.position.clone(),
                landmark: state.landmark.clone(),
                landmark_rotation: state.landmark_rotation.clone(),
            };
            for j in 0..trial.landmark.len() {
                let (at, dims) = (pattern.landmark_at[j], pattern.landmark_dims[j]);
                let entry = |i: usize| step[(at + i, 0)];
                if dims == 6 {
                    trial.landmark_rotation[j] = mat3::mat_mul(
                        &trial.landmark_rotation[j],
                        &exp([entry(0), entry(1), entry(2)]),
                    );
                }
                let p = dims - 3;
                trial.landmark[j] =
                    mat3::add(&trial.landmark[j], &[entry(p), entry(p + 1), entry(p + 2)]);
            }
            for (k, slot) in pattern.slot.iter().enumerate() {
                if let Some(slot) = slot {
                    let delta: Vec<f64> = (0..6).map(|i| step[(6 * slot + i, 0)]).collect();
                    (trial.rotation[k], trial.position[k]) = state.retract(k, &delta);
                }
            }
            let trial_cost = cost(
                &trial,
                odometry,
                closures,
                closure_weights,
                unary,
                unary_weights,
                sightings,
                sighting_weights,
            );
            if trial_cost.is_finite() && trial_cost <= current {
                stepped = current - trial_cost > tolerance * (current + 1.0)
                    || (to_minimum && damping > 1e-6);
                accepted = true;
                *state = trial;
                current = trial_cost;
                damping = (damping / 10.0).max(1e-9);
                break;
            }
            damping *= 10.0;
        }
        if !stepped {
            // no step lowering the cost is a minimum only if the model sees nothing left there either
            return accepted || predicted.is_some_and(|p| p <= tolerance * (current + 1.0));
        }
    }
    false
}

/// The Gauss-Newton normal equations at `state`: J^T W J's entries in `Pattern` fill order (damping
/// diagonal not included), -J^T W r, and J^T W J's diagonal.
fn normal_equations(
    state: &State,
    pattern: &Pattern,
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut values: Vec<f64> = Vec::new();
    let mut gradient = vec![0.0; pattern.size];
    let mut diagonal = vec![0.0; pattern.size];
    let weighted = odometry
        .iter()
        .map(|e| (e, 1.0))
        .chain(closures.iter().zip(closure_weights.iter().copied()));
    for (e, w) in weighted {
        let (r, jf, jt) = linearize(e, state);
        let jacobian = |k: usize| if k == e.from { &jf } else { &jt };
        for (a, b, slot_a, slot_b) in blocks(&pattern.slot, e) {
            // the (a, b) block of J^T W J is ja^T W jb
            let (ja, jb) = (jacobian(a), jacobian(b));
            for (i, j, row, col) in block_entries(slot_a, slot_b) {
                let v: f64 = (0..6)
                    .map(|k| ja[k][i] * w / e.variances[k] * jb[k][j])
                    .sum();
                if row == col {
                    diagonal[row] += v;
                }
                values.push(v);
            }
        }
        for (k, jk) in [(e.from, &jf), (e.to, &jt)] {
            if let Some(slot) = pattern.slot[k] {
                for i in 0..6 {
                    gradient[6 * slot + i] -= (0..6)
                        .map(|m| jk[m][i] * w / e.variances[m] * r[m])
                        .sum::<f64>();
                }
            }
        }
    }
    // position priors: residual p_k + R_k lever - z; Jacobian -R_k [lever]x on the rotation perturbation (entries 0..3),
    // identity on the position (entries 3..6)
    for (u, w) in unary.iter().zip(unary_weights) {
        let Some(slot) = pattern.slot[u.keyframe] else {
            continue;
        };
        let r = unary_residual(u, state);
        let lever_skew = mat3::mat_mul(&state.rotation[u.keyframe], &mat3::skew(&u.lever));
        // jacobian[row][col], 3 x 6
        let mut jacobian = [[0.0; 6]; 3];
        for row in 0..3 {
            for col in 0..3 {
                jacobian[row][col] = -lever_skew[row][col];
            }
            jacobian[row][3 + row] = 1.0;
        }
        let info_j: Vec<[f64; 6]> = (0..3)
            .map(|row| {
                let mut out = [0.0; 6];
                for (col, o) in out.iter_mut().enumerate() {
                    *o = (0..3)
                        .map(|m| u.information[row][m] * jacobian[m][col])
                        .sum();
                }
                out
            })
            .collect();
        for (i, j, row, col) in block_entries(slot, slot) {
            let v = w * (0..3).map(|m| jacobian[m][i] * info_j[m][j]).sum::<f64>();
            if row == col {
                diagonal[row] += v;
            }
            values.push(v);
        }
        let ir = mat3::mat_vec(&u.information, &r);
        for i in 0..6 {
            gradient[6 * slot + i] -= w * (0..3).map(|m| jacobian[m][i] * ir[m]).sum::<f64>();
        }
    }
    // landmark sightings: J^T I J over the keyframe's pose and the landmark (`Sighting::entries`)
    for (s, w) in sightings.iter().zip(sighting_weights) {
        let (at, dims) = (
            pattern.landmark_at[s.landmark],
            pattern.landmark_dims[s.landmark],
        );
        let (r, jacobian) = s.linearize(state, dims);
        let info_j: Vec<[f64; 12]> = (0..6)
            .map(|row| {
                std::array::from_fn(|col| {
                    (0..6)
                        .map(|m| s.information[row][m] * jacobian[m][col])
                        .sum()
                })
            })
            .collect();
        for (i, j, row, col) in s.entries(&pattern.slot, at, dims) {
            let v = w * (0..6).map(|m| jacobian[m][i] * info_j[m][j]).sum::<f64>();
            if row == col {
                diagonal[row] += v;
            }
            values.push(v);
        }
        let ir = landmarks::mat_vec6(&s.information, &r);
        let pull = |i: usize| w * (0..6).map(|m| jacobian[m][i] * ir[m]).sum::<f64>();
        if let Some(slot) = pattern.slot[s.keyframe] {
            for i in 0..6 {
                gradient[6 * slot + i] -= pull(i);
            }
        }
        for i in 0..dims {
            gradient[at + i] -= pull(6 + i);
        }
    }
    (values, gradient, diagonal)
}

/// The chi-square value a 6-DoF residual stays under with probability `p` (CDF 1 - e^{-x/2}(1 + x/2 + x²/8)).
fn chi2_inverse_6dof(p: f64) -> f64 {
    crate::loop_overlap::chi2_inverse(p, |x| {
        let half = x / 2.0;
        1.0 - (-half).exp() * (1.0 + half + half * half / 2.0)
    })
}

/// Rotation by axis-angle `w` (Rodrigues).
fn exp(w: Vec3) -> Mat3 {
    let angle = mat3::norm(&w);
    let k = mat3::skew(&w);
    let (a, b) = if angle < 1e-9 {
        (1.0, 0.5)
    } else {
        (angle.sin() / angle, (1.0 - angle.cos()) / (angle * angle))
    };
    let k2 = mat3::mat_mul(&k, &k);
    let mut r = mat3::identity();
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] += a * k[i][j] + b * k2[i][j];
        }
    }
    r
}

/// Axis-angle of rotation `r`.
pub(crate) fn log(r: &Mat3) -> Vec3 {
    let cos = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cos.acos();
    let v = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    let scale = if angle < 1e-9 {
        0.5
    } else {
        angle / (2.0 * angle.sin())
    };
    v.map(|x| x * scale)
}

/// The reported odometry variance of one keyframe step from `from` to `to` (xy, z, yaw, roll/pitch): each source's
/// drift per square root of metre travelled, squared, times the distance, plus its drift per square root of radian
/// turned, squared, times the angle.
pub fn step_variances(config: &Config, from: &KeyPoseWithCloud, to: &KeyPoseWithCloud) -> [f64; 4] {
    let distance = mat3::norm(&mat3::sub(&to.translation_local, &from.translation_local));
    let turn = mat3::angular_distance(&from.rotation_local, &to.rotation_local);
    motion_variances(config, distance, turn)
}

/// The reported odometry variance (xy, z, yaw, roll/pitch) of `distance` m travelled while turning `turn` rad.
pub fn motion_variances(config: &Config, distance: f64, turn: f64) -> [f64; 4] {
    let c = config;
    [
        (
            c.odometry_error_estimate_xy_per_sqrt_m,
            c.odometry_error_estimate_xy_per_sqrt_rad,
        ),
        (
            c.odometry_error_estimate_z_per_sqrt_m,
            c.odometry_error_estimate_z_per_sqrt_rad,
        ),
        (
            c.odometry_error_estimate_yaw_per_sqrt_m,
            c.odometry_error_estimate_yaw_per_sqrt_rad,
        ),
        (
            c.odometry_error_estimate_tilt_per_sqrt_m,
            c.odometry_error_estimate_tilt_per_sqrt_rad,
        ),
    ]
    .map(|(per_m, per_rad)| per_m * per_m * distance + per_rad * per_rad * turn)
}

/// The mean of `step_variances` over the odometry so far (before two keyframes, a step of the keyframe thresholds).
pub(crate) fn mean_step_variances(key_poses: &[KeyPoseWithCloud], config: &Config) -> [f64; 4] {
    if key_poses.len() < 2 {
        return motion_variances(
            config,
            config.keyframe_min_distance_meters,
            config.keyframe_min_rotation_degrees.to_radians(),
        );
    }
    let mut sum = [0.0; 4];
    for pair in key_poses.windows(2) {
        let step = step_variances(config, &pair[0], &pair[1]);
        for c in 0..4 {
            sum[c] += step[c];
        }
    }
    sum.map(|s| s / (key_poses.len() - 1) as f64)
}

/// The odometry as pose-graph edges, each step's reported variances (`step_variances`) times `noise` (xy, z, yaw,
/// roll/pitch ratios).
pub(crate) fn odometry_edges(
    key_poses: &[KeyPoseWithCloud],
    config: &Config,
    noise: [f64; 4],
) -> Vec<Edge> {
    key_poses
        .windows(2)
        .enumerate()
        .map(|(i, pair)| {
            let back = mat3::transpose(&pair[0].rotation_local);
            let step = step_variances(config, &pair[0], &pair[1]);
            let [xy, z, yaw, tilt]: [f64; 4] = std::array::from_fn(|c| step[c] * noise[c]);
            Edge {
                from: i,
                to: i + 1,
                rotation: mat3::mat_mul(&back, &pair[1].rotation_local),
                translation: mat3::mat_vec(
                    &back,
                    &mat3::sub(&pair[1].translation_local, &pair[0].translation_local),
                ),
                variances: [tilt, tilt, yaw, xy, xy, z],
            }
        })
        .collect()
}

/// Committed closures as pose-graph edges; a closure's variance is its ICP fitness on every axis.
pub(crate) fn closure_edges<'a>(closures: impl Iterator<Item = &'a CommittedClosure>) -> Vec<Edge> {
    closures
        .map(|l| Edge {
            from: l.target_id,
            to: l.source_id,
            rotation: l.rotation_offset,
            translation: l.translation_offset,
            variances: [l.score.max(1e-6); 6],
        })
        .collect()
}
