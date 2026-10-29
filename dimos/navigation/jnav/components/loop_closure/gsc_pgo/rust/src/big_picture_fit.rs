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
    /// Pairwise consistency (PCM, Mangelson 2018): the largest set of closures that all agree pairwise around the
    /// cycles they close with the odometry is held in through GNC (`consistent_closures`); 0 odometry variance = off.
    pub pcm_odometry_translation_variance: f64,
    /// When true, GNC also runs with the Geman-McClure surrogate (Black & Rangarajan 1996; Yang et al. 2020), which
    /// starts at least squares rather than at TLS's odometry-dominated mu near 0, keeping a factor whose whitened
    /// residual at its solution is under the chi-square cutoff. Its verdict replaces TLS's when its solution has the
    /// lower truncated-least-squares cost (the objective both minimise: TLS's path missed it), or when the odometry
    /// residuals of its solution reject the odometry's stated noise as too small (`odometry_overtrusted`: the model, not
    /// the path, is wrong, and the kept closures' residuals then teach the noise learner the larger noise).
    pub gnc_overtrust_check: bool,
    /// Per odometry noise source (xy, z, yaw, roll/pitch): how many times larger the most noise the evidence still allows
    /// is than the noise the odometry edges carry (1 when the solve uses the stated noise). The overtrust test asks
    /// whether the residuals reject even that, since a learner's solve noise (its lower bound) is low by design.
    pub overtrust_noise_headroom: [f64; 4],
    /// The odometry's attitude against gravity as absolute factors, always in (`GravityPrior`); empty for none.
    pub gravity: Vec<GravityPrior>,
}

/// A gravity-referenced odometry's tilt as an absolute factor: keyframe `keyframe`'s up direction in its own frame,
/// R_k^T `up` (`up` the solve frame's), should be `body_up`, each axis at `variance` (rad²). Rotation about up is left
/// free, so only roll/pitch are held: an IMU-aided odometry's tilt error is bounded by its accelerometer, not a random
/// walk, so a chain of tilt steps cannot add up to a floor's height.
#[derive(Clone, Copy, Debug)]
pub struct GravityPrior {
    pub keyframe: usize,
    pub up: Vec3,
    pub body_up: Vec3,
    pub variance: f64,
}

/// R_k^T up - body_up.
fn gravity_residual(g: &GravityPrior, state: &State) -> Vec3 {
    mat3::sub(
        &mat3::mat_vec(&mat3::transpose(&state.rotation[g.keyframe]), &g.up),
        &g.body_up,
    )
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
    /// Variance-component evidence on the odometry noise from the solved graph (`odometry_noise_evidence`): per source
    /// (xy, z, yaw, roll/pitch, then the kept closures' rotation and translation), the sum of its residuals squared and whitened, and its
    /// redundancy; None if the graph could not be factored.
    pub odometry_noise_evidence: Option<[(f64, f64); 6]>,
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
        &config.gravity,
    );
    // the noise evidence never sees the gravity factors: they are the raw odometry's own tilt, not a second measurement
    // of it, so as redundancy they would vouch for the odometry's tilt steps with the steps' own data (the learner then
    // collapses the tilt noise towards 0); it reads the odometry and closures alone, re-solved without them
    let evidence_pattern = (!config.gravity.is_empty()).then(|| {
        Pattern::new(
            free,
            odometry.iter().chain(closures),
            unary,
            sightings,
            &landmark_dims,
            &[],
        )
    });
    let evidence_pattern = evidence_pattern.as_ref().unwrap_or(&pattern);

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
    let held = if config.pcm_odometry_translation_variance > 0.0 {
        consistent_closures(&state, &scaled, config)
    } else {
        vec![false; closures.len()]
    };
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
    // the least-squares solve with every factor in: where GNC-GM starts (`gnc_overtrust_check`)
    let least_squares = config.gnc_overtrust_check.then(|| {
        (
            State {
                rotation: state.rotation.clone(),
                position: state.position.clone(),
                landmark: state.landmark.clone(),
                landmark_rotation: state.landmark_rotation.clone(),
            },
            r2.clone(),
            u2.clone(),
            s2.clone(),
        )
    });
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
                &pattern.gravity,
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
    // TLS's convex start (mu near 0) weighs every closure at about cutoff x sqrt(mu) / residual against the always-in
    // odometry, so how far the odometry is trusted decides its first steps: an overtrusted odometry takes the graph back
    // before the closures are heard. GNC-GM starts at least squares instead (Black & Rangarajan 1996; Yang et al. 2020),
    // and its verdict replaces TLS's only where the closures it keeps prove the odometry's noise understated
    // only when TLS voted a closure out: with every one kept, GM has nothing to add
    let least_squares = least_squares.filter(|_| weights.iter().any(|&w| w < 0.5));
    let mut gm_taken = false;
    if let Some((mut gm_state, mut r2, mut u2, mut s2)) = least_squares {
        let max = |r: &[f64]| r.iter().copied().fold(0.0, f64::max);
        // Remark 5: mu0 = 2 r_max^2 / cutoff over the residuals the least-squares solve leaves
        let mut mu = [
            2.0 * max(&r2) / threshold,
            2.0 * max(&u2) / unary_threshold,
            2.0 * max(&s2),
        ]
        .into_iter()
        .fold(1.0, f64::max);
        let gm = |r: f64, threshold: f64, mu: f64| (mu * threshold / (r + mu * threshold)).powi(2);
        let (mut gm_weights, mut gm_unary, mut gm_sightings) = (
            vec![1.0; closures.len()],
            vec![1.0; unary.len()],
            vec![1.0; sightings.len()],
        );
        loop {
            for ((w, &r), &held) in gm_weights.iter_mut().zip(&r2).zip(&held) {
                if !held {
                    *w = gm(r, threshold, mu);
                }
            }
            for (w, &r) in gm_unary.iter_mut().zip(&u2) {
                *w = gm(r, unary_threshold, mu);
            }
            for (w, &r) in gm_sightings.iter_mut().zip(&s2) {
                *w = gm(r, 1.0, mu);
            }
            levenberg_marquardt(
                &mut gm_state,
                &pattern,
                odometry,
                &scaled,
                &gm_weights,
                unary,
                &gm_unary,
                sightings,
                &gm_sightings,
                1e-5,
            );
            r2 = residual(&gm_state);
            u2 = unary_residuals(&gm_state);
            s2 = sighting_residuals(&gm_state);
            if mu <= 1.0 {
                break;
            }
            mu = (mu / 1.4).max(1.0);
        }
        // in or out by the chi-square test at the GM solution, as TLS's weights end
        let classify = |r: f64, threshold: f64| if r < threshold { 1.0 } else { 0.0 };
        for ((w, &r), &held) in gm_weights.iter_mut().zip(&r2).zip(&held) {
            *w = if held { 1.0 } else { classify(r, threshold) };
        }
        for (w, &r) in gm_unary.iter_mut().zip(&u2) {
            *w = classify(r, unary_threshold);
        }
        for (w, &r) in gm_sightings.iter_mut().zip(&s2) {
            *w = classify(r, 1.0);
        }
        // the objective both GNCs minimise, truncated least squares, at each one's verdict: GM's path can find the lower
        // minimum TLS's convex start missed (TLS voting out a consistent group from an odometry-dominated start)
        let truncated_cost = |state: &State| {
            let none = |n: usize| vec![0.0; n];
            cost(
                state,
                &pattern.gravity,
                odometry,
                &scaled,
                &none(closures.len()),
                unary,
                &none(unary.len()),
                sightings,
                &none(sightings.len()),
            ) + residual(state)
                .iter()
                .map(|&r| r.min(threshold))
                .sum::<f64>()
                + unary_residuals(state)
                    .iter()
                    .map(|&r| r.min(unary_threshold))
                    .sum::<f64>()
                + sighting_residuals(state)
                    .iter()
                    .map(|&r| r.min(1.0))
                    .sum::<f64>()
        };
        let lower = truncated_cost(&gm_state) < truncated_cost(&state);
        // GM's verdict is new evidence that the odometry is overtrusted only where TLS's own verdict shows none: where
        // TLS already keeps closures that bend the odometry past its noise (right or look-alike), the noise learner
        // reads that from TLS's residuals, and swapping in GM's larger set would only add closures the evidence did
        // not need (look-alike floors that agree with each other)
        let overtrusted =
            |state: &mut State, weights: &[f64], unary_w: &[f64], sighting_w: &[f64]| {
                // the evidence's own re-solve, without the gravity factors, on a copy: the verdict's state keeps them
                let mut without_gravity;
                let state = if config.gravity.is_empty() {
                    state
                } else {
                    without_gravity = state.clone();
                    &mut without_gravity
                };
                odometry_overtrusted(
                    state,
                    evidence_pattern,
                    odometry,
                    closures,
                    weights,
                    unary,
                    unary_w,
                    sightings,
                    sighting_w,
                    config.gnc_inlier_probability,
                    config.overtrust_noise_headroom,
                )
            };
        let mut tls_state = State {
            rotation: state.rotation.clone(),
            position: state.position.clone(),
            landmark: state.landmark.clone(),
            landmark_rotation: state.landmark_rotation.clone(),
        };
        if lower
            || (overtrusted(&mut gm_state, &gm_weights, &gm_unary, &gm_sightings)
                && !overtrusted(&mut tls_state, &weights, &unary_weights, &sighting_weights))
        {
            (state, weights, unary_weights, sighting_weights) =
                (gm_state, gm_weights, gm_unary, gm_sightings);
            gm_taken = true;
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
            &pattern.gravity,
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
    // GM's verdict was reached without the odometry noise having its say, so only its mutually consistent core (the
    // largest set of kept closures that agree pairwise around their cycles, PCM, at the noise the odometry edges carry)
    // speaks for the noise: a kept closure the others contradict would otherwise read as odometry noise and widen it
    let evidence_weights: Vec<f64> = if gm_taken {
        let kept_closures: Vec<(usize, Edge)> = closures
            .iter()
            .zip(&huber)
            .enumerate()
            .filter(|(_, (_, &w))| w > 0.0)
            .map(|(i, (e, _))| (i, *e))
            .collect();
        let step_variance =
            odometry.iter().map(|e| e.variances[3]).sum::<f64>() / odometry.len().max(1) as f64;
        let pcm = SolveConfig {
            pcm_odometry_translation_variance: step_variance,
            ..config.clone()
        };
        let edges: Vec<Edge> = kept_closures.iter().map(|&(_, e)| e).collect();
        let consistent = consistent_closures(&state, &edges, &pcm);
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
    // without the gravity factors the evidence reads the odometry and closures' own solve
    let mut without_gravity = state.clone();
    if !config.gravity.is_empty() {
        levenberg_marquardt(
            &mut without_gravity,
            evidence_pattern,
            odometry,
            closures,
            &evidence_weights,
            unary,
            &unary_kept,
            sightings,
            &sighting_kept,
            1e-5,
        );
    }
    let odometry_noise_evidence = odometry_noise_evidence(
        &without_gravity,
        evidence_pattern,
        odometry,
        closures,
        &evidence_weights,
        unary,
        &unary_kept,
        sightings,
        &sighting_kept,
    );
    Solution {
        odometry_noise_evidence,
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

/// Variance-component evidence on the odometry noise from the solved graph (Förstner's estimator): for each noise source,
/// how far the solve had to bend its parts of the odometry edges (their residuals squared, whitened by the variances
/// the solve used) and how many independent checks the graph gave it, its redundancy r_c = n_c - tr(N^-1 N_c) (n_c its
/// residual count; N the solve's information matrix, N_c the source's own share of it). A source's variance times
/// sum / r_c is the variance that explains the residuals. The odometry alone determines the graph exactly (a chain),
/// so n_c = tr(N_odo^-1 N_c) and r_c = tr((N_odo^-1 - N^-1) N_c): the difference has rank at most the kept constraints',
/// so Hutchinson's estimator (the mean of z^T (N_odo^-1 - N^-1) N_c z over fixed random +-1 vectors z, two solves each)
/// is accurate to a few percent of r_c, and exactly 0 with nothing kept. None if a matrix cannot be factored.
#[allow(clippy::too_many_arguments)]
fn odometry_noise_evidence(
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

/// Whether the odometry is overtrusted at `state` with these factors in: after a least-squares solve with them at their
/// own variances, the odometry's whitened residuals squared over its redundancy (`odometry_noise_evidence`, its four
/// noise sources pooled, each at its `headroom` x the edges' noise) exceed what that noise allows at `probability`, a chi-square with that many degrees of
/// freedom (Wilson-Hilferty's cube-root normal approximation, as the redundancy is not a whole number).
#[allow(clippy::too_many_arguments)]
fn odometry_overtrusted(
    state: &mut State,
    pattern: &Pattern,
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
    probability: f64,
    headroom: [f64; 4],
) -> bool {
    levenberg_marquardt(
        state,
        pattern,
        odometry,
        closures,
        closure_weights,
        unary,
        unary_weights,
        sightings,
        sighting_weights,
        1e-5,
    );
    let Some(evidence) = odometry_noise_evidence(
        state,
        pattern,
        odometry,
        closures,
        closure_weights,
        unary,
        unary_weights,
        sightings,
        sighting_weights,
    ) else {
        return false;
    };
    // whitened at the largest noise still allowed, each source's sum shrinks by its headroom
    let (sum, redundancy) = evidence[..4]
        .iter()
        .zip(headroom)
        .fold((0.0, 0.0), |(s, r), (&(sc, rc), h)| {
            (s + sc / h.max(1.0), r + rc)
        });
    if !(redundancy > 0.0 && sum.is_finite()) {
        return false;
    }
    let z = landmarks::chi2_inverse(2.0 * probability - 1.0, 1).sqrt();
    let spread = 2.0 / (9.0 * redundancy);
    sum > redundancy * (1.0 - spread + z * spread.sqrt()).powi(3)
}

/// The largest set of closures that pairwise agree (PCM): two closures agree when keyframe A.to's position predicted
/// through closure A equals its position predicted through B and the odometry between their ends, within the
/// chi-square cutoff of `gnc_inlier_probability` (3 DOF), the translation variance being both closures' plus the
/// odometry's over the keyframes between their ends. The set is grown greedily from the best-agreeing closure (an
/// approximate maximum clique); a set of one is no evidence of agreement, so it holds nothing.
fn consistent_closures(state: &State, closures: &[Edge], config: &SolveConfig) -> Vec<bool> {
    let n = closures.len();
    let cutoff = crate::loop_overlap::chi2_inverse_3dof(config.gnc_inlier_probability);
    let at = |pose: usize, offset: &Vec3| {
        mat3::add(
            &state.position[pose],
            &mat3::mat_vec(&state.rotation[pose], offset),
        )
    };
    let agree = |a: &Edge, b: &Edge| -> bool {
        // a.to through a
        let through_a = at(a.from, &a.translation);
        // b.to through b, then a.to through the odometry from b.to (b.to's orientation as b says)
        let b_to = at(b.from, &b.translation);
        let b_to_rotation = mat3::mat_mul(&state.rotation[b.from], &b.rotation);
        let odometry_offset = mat3::mat_vec(
            &mat3::transpose(&state.rotation[b.to]),
            &mat3::sub(&state.position[a.to], &state.position[b.to]),
        );
        let through_b = mat3::add(&b_to, &mat3::mat_vec(&b_to_rotation, &odometry_offset));
        // a.from and b.from are tied by the odometry too: shift b's prediction by their odometry offset error
        let gap = (a.from.abs_diff(b.from) + a.to.abs_diff(b.to)) as f64;
        let variance =
            a.variances[3] + b.variances[3] + gap * config.pcm_odometry_translation_variance;
        let d = mat3::sub(&through_a, &through_b);
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) / variance.max(f64::EPSILON) < cutoff
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
    gravity: &[GravityPrior],
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    unary: &[Unary],
    unary_weights: &[f64],
    sightings: &[Sighting],
    sighting_weights: &[f64],
) -> f64 {
    gravity
        .iter()
        .map(|g| mat3::norm(&gravity_residual(g, state)).powi(2) / g.variance)
        .sum::<f64>()
        + sightings
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
    /// The gravity factors (`SolveConfig::gravity`): never reweighted, so part of the problem's structure.
    gravity: Vec<GravityPrior>,
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
        gravity: &[GravityPrior],
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
        // gravity factors after those (their keyframe's own block)
        for g in gravity {
            if let Some(a) = slot[g.keyframe] {
                indices.extend(block_entries(a, a).map(|(_, _, r, c)| Pair::new(r, c)));
            }
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
            gravity: gravity.to_vec(),
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

/// Gauss-Newton steps, damped (and retried) whenever a step would raise the cost; done once a step lowers
/// the cost by less than `tolerance` of it.
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
) {
    let size = pattern.size;
    let mut damping = 1e-6;
    let mut current = cost(
        state,
        &pattern.gravity,
        odometry,
        closures,
        closure_weights,
        unary,
        unary_weights,
        sightings,
        sighting_weights,
    );
    for _ in 0..100 {
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
        let mut stepped = false;
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
                &pattern.gravity,
                odometry,
                closures,
                closure_weights,
                unary,
                unary_weights,
                sightings,
                sighting_weights,
            );
            if trial_cost.is_finite() && trial_cost <= current {
                stepped = current - trial_cost > tolerance * current;
                *state = trial;
                current = trial_cost;
                damping = (damping / 10.0).max(1e-9);
                break;
            }
            damping *= 10.0;
        }
        if !stepped {
            break;
        }
    }
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
    // gravity factors: residual R_k^T up - body_up; perturbing R_k by exp(d) about its own axes moves it by
    // [R_k^T up]x d, so rotation about up itself (yaw) is untouched
    for g in &pattern.gravity {
        let Some(slot) = pattern.slot[g.keyframe] else {
            continue;
        };
        let r = gravity_residual(g, state);
        let jacobian = mat3::skew(&mat3::mat_vec(
            &mat3::transpose(&state.rotation[g.keyframe]),
            &g.up,
        ));
        let w = 1.0 / g.variance;
        for (i, j, row, col) in block_entries(slot, slot) {
            let v = if i < 3 && j < 3 {
                w * (0..3).map(|m| jacobian[m][i] * jacobian[m][j]).sum::<f64>()
            } else {
                0.0
            };
            if row == col {
                diagonal[row] += v;
            }
            values.push(v);
        }
        for i in 0..3 {
            gradient[6 * slot + i] -= w * (0..3).map(|m| jacobian[m][i] * r[m]).sum::<f64>();
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

/// The odometry between consecutive keyframes, from their dead-reckoned poses, at the configured variances.
/// The reported odometry variance of one keyframe step from `from` to `to` (xy, z, yaw, roll/pitch): the per-keyframe
/// constants, or with `odom_noise_per_motion` the per-metre rates times the distance travelled plus the per-radian rates
/// times the angle turned.
pub(crate) fn step_variances(
    config: &Config,
    from: &KeyPoseWithCloud,
    to: &KeyPoseWithCloud,
) -> [f64; 4] {
    if !config.odom_noise_per_motion {
        return [
            config.odom_trans_xy_var,
            config.odom_trans_z_var,
            config.odom_rot_yaw_var,
            config.odom_rot_roll_pitch_var,
        ];
    }
    let distance = mat3::norm(&mat3::sub(&to.translation_local, &from.translation_local));
    let turn = mat3::angular_distance(&from.rotation_local, &to.rotation_local);
    [
        config.odom_xy_var_per_m * distance + config.odom_xy_var_per_rad * turn,
        config.odom_z_var_per_m * distance + config.odom_z_var_per_rad * turn,
        config.odom_yaw_var_per_m * distance + config.odom_yaw_var_per_rad * turn,
        config.odom_tilt_var_per_m * distance + config.odom_tilt_var_per_rad * turn,
    ]
}

/// The mean of `step_variances` over the odometry so far (the per-keyframe constants before two keyframes).
pub(crate) fn mean_step_variances(key_poses: &[KeyPoseWithCloud], config: &Config) -> [f64; 4] {
    if !config.odom_noise_per_motion || key_poses.len() < 2 {
        return [
            config.odom_trans_xy_var,
            config.odom_trans_z_var,
            config.odom_rot_yaw_var,
            config.odom_rot_roll_pitch_var,
        ];
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

/// Committed closures as pose-graph edges; a closure's variance is its fit's own (`CommittedClosure::variances`), else its ICP fitness, times `scale`
/// (`GscPgo::closure_noise_scale`).
pub(crate) fn closure_edges<'a>(
    closures: impl Iterator<Item = &'a CommittedClosure>,
    // [rotation, translation] factors on ICP fitness
    scale: [f64; 2],
) -> Vec<Edge> {
    closures
        .map(|l| Edge {
            from: l.target_id,
            to: l.source_id,
            rotation: l.rotation_offset,
            translation: l.translation_offset,
            variances: std::array::from_fn(|k| {
                l.variances.map_or(l.score.max(1e-6), |v| v[k]) * scale[k / 3]
            }),
        })
        .collect()
}
