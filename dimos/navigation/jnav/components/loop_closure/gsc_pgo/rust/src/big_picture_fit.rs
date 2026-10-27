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

use std::thread::JoinHandle;

use crate::gsc_pgo::{ClosureEdgeState, CommittedClosure, Config, GscPgo, KeyPoseWithCloud};
use crate::mat3::{self, Mat3, Vec3};

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
    /// Half the sum of squared whitened residuals of everything kept.
    pub error: f64,
    /// The newest keyframe's position covariance as seen from each keyframe (its offset in that keyframe's frame, in
    /// world axes, m²), from the solved graph; empty if the graph could not be factored.
    pub relative_position_covariance: Vec<Mat3>,
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
    solve_core(rotations, positions, free, odometry, closures, &[], config)
}

/// A measured position, in fixed world frame `frame` (GPS after projection to a local east-north-up frame, a total
/// station, motion capture, ...; one id per coordinate system), of a sensor mounted at `lever` in keyframe `keyframe`'s
/// body frame (an antenna's offset from the body origin, from tf), with its covariance (m²).
#[derive(Clone, Copy, Debug)]
pub struct PositionPrior {
    pub keyframe: usize,
    pub frame: u32,
    pub lever: Vec3,
    pub world: Vec3,
    pub covariance: Mat3,
}

/// A position prior placed in the solve frame: keyframe `keyframe`'s sensor at `lever` (body frame) should be at
/// `position`, with information (inverse covariance) `information`.
#[derive(Clone, Copy, Debug)]
struct Unary {
    keyframe: usize,
    lever: Vec3,
    position: Vec3,
    information: Mat3,
}

/// `solve` with absolute position priors, in any number of world frames (each gravity-aligned, each differing from the
/// map by an unknown rotation about the vertical and a translation). A frame is used once its priors pin its heading
/// (horizontal spread more than twice their noise). The frame with the most usable priors is primary: a first fit of
/// it (weighted Procrustes of the prior-free solution onto its priors) moves the graph into it, every keyframe is freed
/// (the priors, not keyframe 0, fix where the map sits) and the priors are unary factors, GNC-weighted like closures
/// (3-DoF chi-square cutoff). Every other frame's primary -> frame alignment is fitted to the primary-only solution and
/// its priors are carried into the primary frame through it for one solve of everything (a two-stage estimate: the
/// alignments are not refined jointly with the graph). The map -> frame alignments are
/// read back at keyframe 0, and the solution is returned in map coordinates.
pub fn solve_with_priors(
    rotations: &[Mat3],
    positions: &[Vec3],
    free: &[bool],
    odometry: &[Edge],
    closures: &[Edge],
    priors: &[PositionPrior],
    config: &SolveConfig,
) -> Solution {
    let mut solution = solve_core(rotations, positions, free, odometry, closures, &[], config);
    let mut frames: Vec<u32> = priors.iter().map(|p| p.frame).collect();
    frames.sort_unstable();
    frames.dedup();
    let of = |frame: u32| -> Vec<usize> {
        (0..priors.len())
            .filter(|&i| priors[i].frame == frame)
            .collect()
    };
    // the sensors' positions in a solution, so a mount offset from the body origin is fitted where it was measured
    let fit = |solution: &Solution, ids: &[usize], weights: &[f64]| {
        let chosen: Vec<PositionPrior> = ids.iter().map(|&i| priors[i]).collect();
        let w: Vec<f64> = ids.iter().map(|&i| weights[i]).collect();
        let sensors: Vec<Vec3> = chosen
            .iter()
            .map(|p| {
                mat3::add(
                    &solution.positions[p.keyframe],
                    &mat3::mat_vec(&solution.rotations[p.keyframe], &p.lever),
                )
            })
            .collect();
        align_priors(&sensors, &chosen, &w)
    };
    let ones = vec![1.0; priors.len()];
    // map -> frame, for every frame its priors make observable
    let first: Vec<(u32, (f64, Vec3))> = frames
        .iter()
        .filter_map(|&f| Some((f, fit(&solution, &of(f), &ones)?)))
        .collect();
    let Some(&(primary, (yaw, translation))) = first.iter().max_by_key(|(f, _)| of(*f).len())
    else {
        solution.prior_weights = vec![0.0; priors.len()];
        return solution;
    };
    let turn = mat3::rot_z(yaw);
    let world_rotations: Vec<Mat3> = solution
        .rotations
        .iter()
        .map(|r| mat3::mat_mul(&turn, r))
        .collect();
    let world_positions: Vec<Vec3> = solution
        .positions
        .iter()
        .map(|p| mat3::add(&mat3::mat_vec(&turn, p), &translation))
        .collect();
    // primary -> frame for the others: compose the first fits
    let mut relative: Vec<(u32, (f64, Vec3))> = first
        .iter()
        .filter(|(f, _)| *f != primary)
        .map(|&(f, (y, t))| {
            let r = y - yaw;
            (
                f,
                (
                    r,
                    mat3::sub(&t, &mat3::mat_vec(&mat3::rot_z(r), &translation)),
                ),
            )
        })
        .collect();
    let all_free = vec![true; free.len()];
    // the priors of the frames in `frames` as unary factors in the primary frame (others through their alignment)
    let unary_of = |frames: &dyn Fn(u32) -> bool,
                    relative: &[(u32, (f64, Vec3))]|
     -> (Vec<usize>, Vec<Unary>) {
        let mut used = Vec::new();
        let mut unary = Vec::new();
        for (i, p) in priors.iter().enumerate() {
            if !frames(p.frame) {
                continue;
            }
            let (position, covariance) = if p.frame == primary {
                (p.world, p.covariance)
            } else if let Some((_, (r, t))) = relative.iter().find(|(f, _)| *f == p.frame) {
                let back = mat3::rot_z(-r);
                (
                    mat3::mat_vec(&back, &mat3::sub(&p.world, t)),
                    mat3::mat_mul(
                        &back,
                        &mat3::mat_mul(&p.covariance, &mat3::transpose(&back)),
                    ),
                )
            } else {
                continue;
            };
            used.push(i);
            unary.push(Unary {
                keyframe: p.keyframe,
                lever: p.lever,
                position,
                information: mat3::inverse(&covariance).unwrap_or([[0.0; 3]; 3]),
            });
        }
        (used, unary)
    };
    let mut weights = vec![0.0; priors.len()];
    // stage 1: the primary frame alone; each other frame's primary -> frame alignment is fitted to that solution, so a
    // frame's own (possibly much tighter) priors cannot hold the graph where a wrong first guess put it
    let (used, unary) = unary_of(&|f| f == primary, &relative);
    let primary_only = solve_core(
        &world_rotations,
        &world_positions,
        &all_free,
        odometry,
        closures,
        &unary,
        config,
    );
    for (&i, &w) in used.iter().zip(&primary_only.prior_weights) {
        weights[i] = w;
    }
    for (f, alignment) in relative.iter_mut() {
        let ids = of(*f);
        if let Some(refit) = fit(&primary_only, &ids, &ones) {
            *alignment = refit;
        }
    }
    // stage 2: every frame together
    let (used, unary) = unary_of(&|_| true, &relative);
    let world = solve_core(
        &primary_only.rotations,
        &primary_only.positions,
        &all_free,
        odometry,
        closures,
        &unary,
        config,
    );
    for (&i, &w) in used.iter().zip(&world.prior_weights) {
        weights[i] = w;
    }
    // map -> primary: keyframe 0's map pose (held in the map) onto its solved pose in the primary frame
    // a turn about the vertical (the frames are gravity-aligned), so it stays exactly a rotation however many solves it
    // passes through
    let turned = mat3::mat_mul(&world.rotations[0], &mat3::transpose(&rotations[0]));
    let align_rotation = mat3::rot_z(turned[1][0].atan2(turned[0][0]));
    let align_translation = mat3::sub(
        &world.positions[0],
        &mat3::mat_vec(&align_rotation, &positions[0]),
    );
    let back = mat3::transpose(&align_rotation);
    let mut solution = world;
    solution.rotations = solution
        .rotations
        .iter()
        .map(|r| mat3::mat_mul(&back, r))
        .collect();
    solution.positions = solution
        .positions
        .iter()
        .map(|p| mat3::mat_vec(&back, &mat3::sub(p, &align_translation)))
        .collect();
    let primary_yaw = align_rotation[1][0].atan2(align_rotation[0][0]);
    solution.world_alignment = std::iter::once((primary, primary_yaw, align_translation))
        .chain(relative.iter().map(|&(f, (r, t))| {
            // map -> frame = (primary -> frame) after (map -> primary)
            (
                f,
                primary_yaw + r,
                mat3::add(&mat3::mat_vec(&mat3::rot_z(r), &align_translation), &t),
            )
        }))
        .collect();
    solution.prior_weights = weights;
    solution
}

/// The map -> world alignment (yaw about z, translation) that best puts the kept priors' keyframes on their world
/// positions, each weighted by its inverse horizontal variance; None until their horizontal spread exceeds twice their
/// typical noise (heading unobservable).
fn align_priors(points: &[Vec3], priors: &[PositionPrior], weights: &[f64]) -> Option<(f64, Vec3)> {
    // `points[i]` is where the current solution has prior i's sensor
    let used: Vec<(Vec3, &PositionPrior, f64)> = priors
        .iter()
        .zip(points)
        .zip(weights)
        .filter(|(_, &w)| w >= 0.5)
        .map(|((p, &point), _)| {
            (
                point,
                p,
                2.0 / (p.covariance[0][0] + p.covariance[1][1]).max(f64::EPSILON),
            )
        })
        .collect();
    if used.len() < 2 {
        return None;
    }
    let total: f64 = used.iter().map(|(_, _, w)| w).sum();
    let mean = |f: &dyn Fn(&Vec3, &PositionPrior) -> Vec3| {
        used.iter().fold([0.0; 3], |m, (point, p, w)| {
            let v = f(point, p);
            [
                m[0] + w * v[0] / total,
                m[1] + w * v[1] / total,
                m[2] + w * v[2] / total,
            ]
        })
    };
    let map_mean = mean(&|point, _| *point);
    let world_mean = mean(&|_, p| p.world);
    let (mut sxx, mut sxy, mut spread, mut noise) = (0.0, 0.0, 0.0, 0.0);
    for (point, p, w) in &used {
        let m = mat3::sub(point, &map_mean);
        let g = mat3::sub(&p.world, &world_mean);
        sxx += w * (m[0] * g[0] + m[1] * g[1]);
        sxy += w * (m[0] * g[1] - m[1] * g[0]);
        spread += w * (g[0] * g[0] + g[1] * g[1]) / total;
        noise += w * (p.covariance[0][0] + p.covariance[1][1]) / total;
    }
    if spread <= 4.0 * noise {
        return None;
    }
    let yaw = sxy.atan2(sxx);
    let rotated = mat3::mat_vec(&mat3::rot_z(yaw), &map_mean);
    Some((yaw, mat3::sub(&world_mean, &rotated)))
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

fn solve_core(
    rotations: &[Mat3],
    positions: &[Vec3],
    free: &[bool],
    odometry: &[Edge],
    closures: &[Edge],
    unary: &[Unary],
    config: &SolveConfig,
) -> Solution {
    let mut state = State {
        rotation: rotations.to_vec(),
        position: positions.to_vec(),
    };
    let scaled: Vec<Edge> = closures
        .iter()
        .map(|e| Edge {
            variances: e.variances.map(|v| v * config.gnc_variance_scale),
            ..*e
        })
        .collect();
    let pattern = Pattern::new(free, odometry.iter().chain(closures), unary);

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
        1e-5,
    );
    let mut r2 = residual(&state);
    let mut u2 = unary_residuals(&state);
    if worst > threshold || unary_worst > unary_threshold {
        // the gentlest start any factor kind that exceeds its own cutoff needs (a kind under its cutoff sets none)
        let start = |worst: f64, threshold: f64| {
            (worst > threshold).then(|| threshold / (2.0 * worst - threshold))
        };
        let mut mu = [start(worst, threshold), start(unary_worst, unary_threshold)]
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
            levenberg_marquardt(
                &mut state,
                &pattern,
                odometry,
                &scaled,
                &weights,
                unary,
                &unary_weights,
                1e-5,
            );
            r2 = residual(&state);
            u2 = unary_residuals(&state);
            // stop like GTSAM's GncOptimizer: weights settled, or the weighted cost stopped changing
            let weighted_cost = cost(&state, odometry, &scaled, &weights, unary, &unary_weights);
            if weights
                .iter()
                .chain(&unary_weights)
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
    let error = 0.5 * cost(&state, odometry, closures, &huber, unary, &unary_kept);
    let relative_position_covariance = relative_position_covariances(
        &state,
        &pattern,
        odometry,
        closures,
        &huber,
        unary,
        &unary_kept,
        positions.len() - 1,
    )
    .unwrap_or_default();
    Solution {
        rotations: state.rotation,
        positions: state.position,
        closure_weights: kept,
        prior_weights: unary_kept,
        world_alignment: Vec::new(),
        error,
        relative_position_covariance,
    }
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

struct State {
    rotation: Vec<Mat3>,
    position: Vec<Vec3>,
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
fn edge_residual(e: &Edge, from: (&Mat3, &Vec3), to: (&Mat3, &Vec3)) -> [f64; 6] {
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
    let (ri, rj) = (&state.rotation[e.from], &state.rotation[e.to]);
    let residual = edge_residual(
        e,
        (ri, &state.position[e.from]),
        (rj, &state.position[e.to]),
    );
    let ri_t = mat3::transpose(ri);
    let local = mat3::mat_vec(
        &ri_t,
        &mat3::sub(&state.position[e.to], &state.position[e.from]),
    );
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
) -> f64 {
    unary
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
    size: usize,
    symbolic: SymbolicSparseColMat<usize>,
    order: Argsort<usize>,
    llt: SymbolicLlt<usize>,
}

impl Pattern {
    fn new<'a>(free: &[bool], edges: impl Iterator<Item = &'a Edge>, unary: &[Unary]) -> Pattern {
        let mut count = 0;
        let slot: Vec<Option<usize>> = free
            .iter()
            .map(|&f| f.then(|| (count, count += 1).0))
            .collect();
        let size = 6 * count.max(1);
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
        // the damping diagonal last, so the matrix is never structurally singular
        indices.extend((0..size).map(|i| Pair::new(i, i)));
        let (symbolic, order) = SymbolicSparseColMat::try_new_from_indices(size, size, &indices)
            .expect("pose graph pattern");
        let llt =
            SymbolicLlt::try_new(symbolic.as_ref(), Side::Lower).expect("pose graph ordering");
        Pattern {
            slot,
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
    tolerance: f64,
) {
    let size = pattern.size;
    let mut damping = 1e-6;
    let mut current = cost(
        state,
        odometry,
        closures,
        closure_weights,
        unary,
        unary_weights,
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
            };
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
fn log(r: &Mat3) -> Vec3 {
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

// ---- the pose graph GSC keeps, re-solved on a background thread whenever closures arrive ----

/// A pose-graph solve running on another thread over the first `keyframes` keyframes (all but keyframe 0 free) and the
/// first `closures_dispatched` committed closures, adopted once it finishes (`GscPgo::update`) or when waited for
/// (`GscPgo::wait_for_solve`).
pub(crate) struct Solving {
    handle: JoinHandle<(Solution, f64)>,
    keyframes: usize,
}

impl GscPgo {
    /// Half the sum of squared whitened residuals of the newest applied solve (how much its
    /// constraints conflict).
    pub fn graph_error(&self) -> f64 {
        self.graph_error
    }
    pub fn committed_closures(&self) -> &[CommittedClosure] {
        &self.committed_closures
    }
    /// Apply the running solve if it has finished, and start one when closures have arrived since the
    /// last. Never blocks. Returns whether keyframes moved.
    pub fn update(&mut self) -> bool {
        let finished = self
            .solving
            .as_ref()
            .is_some_and(|solving| solving.handle.is_finished());
        self.adopt(finished)
    }
    /// `update`, but waiting for the running solve to finish first.
    pub fn wait_for_solve(&mut self) -> bool {
        self.adopt(true)
    }
    /// The running solve's keyframe count and free keyframe count when it started, if one is running.
    pub fn solve_started(&self) -> Option<(usize, usize)> {
        self.solving
            .as_ref()
            .map(|s| (s.keyframes, s.keyframes - 1))
    }
    /// Call `solved` (from the solve's thread) whenever a solve finishes, so a caller can sleep until then.
    pub fn notify_when_solved(&mut self, solved: impl Fn() + Send + Sync + 'static) {
        self.solved = Some(std::sync::Arc::new(solved));
    }
    fn adopt(&mut self, ready: bool) -> bool {
        self.new_closure = false;
        let moved = ready && self.apply_solve();
        if self.solving.is_none() && self.unsolved() {
            self.dispatch_solve();
        }
        moved
    }
    /// Whether a closure or a world position arrived after the newest solve started.
    fn unsolved(&self) -> bool {
        self.committed_closures.len() > self.closures_dispatched
            || self.world_positions.len() > self.world_positions_dispatched
    }
    /// A measured position of a sensor in fixed world frame `frame` at `time`: the sensor sits at `lever` in the body frame
    /// (its mount, from tf), `position` and `covariance` (m²) are in `frame`. It joins the next solve as a prior on the
    /// keyframe nearest in time (`solve_with_priors`).
    pub fn add_world_position(
        &mut self,
        time: f64,
        frame: u32,
        lever: Vec3,
        position: Vec3,
        covariance: Mat3,
    ) {
        self.world_positions
            .push((time, frame, lever, position, covariance));
    }
    /// The newest solve's map -> frame alignments (frame id, yaw about z, translation), one per world frame its
    /// positions made observable.
    pub fn world_alignment(&self) -> &[(u32, f64, Vec3)] {
        &self.world_alignment
    }
    /// Each world position as a prior on the keyframe nearest its time, its covariance grown by how far the body moves in
    /// that time gap (the keyframes' own speed there, squared, in each horizontal axis and up); fixes before the first or
    /// after the last keyframe are left for later.
    fn position_priors(&self) -> Vec<PositionPrior> {
        let n = self.key_poses.len();
        if n < 2 {
            return Vec::new();
        }
        let times: Vec<f64> = self.key_poses.iter().map(|k| k.time).collect();
        self.world_positions
            .iter()
            .filter_map(|&(time, frame, lever, world, covariance)| {
                if time < times[0] || time > times[n - 1] {
                    return None;
                }
                let after = times.partition_point(|&t| t < time).clamp(1, n - 1);
                let keyframe = if time - times[after - 1] <= times[after] - time {
                    after - 1
                } else {
                    after
                };
                let gap = (times[after] - times[after - 1]).max(f64::EPSILON);
                let speed = mat3::norm(&mat3::sub(
                    &self.key_poses[after].translation_local,
                    &self.key_poses[after - 1].translation_local,
                )) / gap;
                let moved = (speed * (time - times[keyframe]).abs()).powi(2);
                let mut grown = covariance;
                for (axis, row) in grown.iter_mut().enumerate() {
                    row[axis] += moved;
                }
                Some(PositionPrior {
                    keyframe,
                    frame,
                    lever,
                    world,
                    covariance: grown,
                })
            })
            .collect()
    }
    /// Wait for the running solve, and solve again until every closure is in. Returns whether keyframes moved.
    pub fn finish(&mut self) -> bool {
        let mut moved = false;
        loop {
            moved |= self.apply_solve();
            if !self.unsolved() {
                return moved;
            }
            self.dispatch_solve();
        }
    }
    /// Start a solve of every committed closure from the odometry (GNC must start from the dead-reckoned chain,
    /// not an estimate earlier closures already bent).
    fn dispatch_solve(&mut self) {
        let n = self.key_poses.len();
        let config = &self.config;
        let closures = closure_edges(self.committed_closures.iter());
        // keyframe 0 pins the map; the rest start from the odometry, riding its correction
        let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
        let pinned = &self.key_poses[0];
        let offset_rotation = mat3::mat_mul(
            &pinned.rotation_global,
            &mat3::transpose(&pinned.rotation_local),
        );
        let offset_translation = mat3::sub(
            &pinned.translation_global,
            &mat3::mat_vec(&offset_rotation, &pinned.translation_local),
        );
        let (mut rotations, mut positions) = (Vec::with_capacity(n), Vec::with_capacity(n));
        rotations.push(pinned.rotation_global);
        positions.push(pinned.translation_global);
        for key_pose in &self.key_poses[1..] {
            rotations.push(mat3::mat_mul(&offset_rotation, &key_pose.rotation_local));
            positions.push(mat3::add(
                &mat3::mat_vec(&offset_rotation, &key_pose.translation_local),
                &offset_translation,
            ));
        }
        // every odometry edge and closure has an end past keyframe 0, so all of them are in
        let odometry = odometry_edges(&self.key_poses, config);
        let priors = self.position_priors();
        let solve_config = SolveConfig {
            gnc_variance_scale: config.closure_gnc_var_scale,
            gnc_inlier_probability: config.closure_gnc_inlier_probability,
            huber_k: config.closure_robust_huber_k,
            pcm_odometry_translation_variance: if config.closure_pcm {
                config.odom_trans_xy_var
            } else {
                0.0
            },
        };
        let solved = self.solved.clone();
        let handle = std::thread::Builder::new()
            .name("gsc-pose-graph".into())
            .spawn(move || {
                let started = std::time::Instant::now();
                let solution = if priors.is_empty() {
                    solve(
                        &rotations,
                        &positions,
                        &free,
                        &odometry,
                        &closures,
                        &solve_config,
                    )
                } else {
                    solve_with_priors(
                        &rotations,
                        &positions,
                        &free,
                        &odometry,
                        &closures,
                        &priors,
                        &solve_config,
                    )
                };
                let seconds = started.elapsed().as_secs_f64();
                if let Some(solved) = solved {
                    solved();
                }
                (solution, seconds)
            })
            .expect("spawn pose graph thread");
        self.closures_dispatched = self.committed_closures.len();
        self.world_positions_dispatched = self.world_positions.len();
        self.solving = Some(Solving {
            handle,
            keyframes: n,
        });
    }
    /// Wait for the running solve and adopt it: its poses, the correction the keyframes since then
    /// ride on, and its closure verdicts. Returns whether there was one.
    fn apply_solve(&mut self) -> bool {
        let Some(solving) = self.solving.take() else {
            return false;
        };
        let (solution, seconds) = solving.handle.join().expect("pose graph solve panicked");
        self.timing.solve_s += seconds;
        if !solution.world_alignment.is_empty() {
            self.world_alignment = solution.world_alignment.clone();
        }
        let n = solving.keyframes;
        self.solved_relative_covariance =
            Some((n - 1, solution.relative_position_covariance.clone()))
                .filter(|(_, covariance)| covariance.len() == n);
        for (key_pose, (rotation, position)) in self.key_poses[..n]
            .iter_mut()
            .zip(solution.rotations.iter().zip(&solution.positions))
        {
            key_pose.rotation_global = *rotation;
            key_pose.translation_global = *position;
        }
        // keyframes made since the solve started ride on its newest keyframe's correction
        let newest = &self.key_poses[n - 1];
        self.rotation_offset = mat3::mat_mul(
            &newest.rotation_global,
            &mat3::transpose(&newest.rotation_local),
        );
        self.translation_offset = mat3::sub(
            &newest.translation_global,
            &mat3::mat_vec(&self.rotation_offset, &newest.translation_local),
        );
        for key_pose in &mut self.key_poses[n..] {
            key_pose.rotation_global =
                mat3::mat_mul(&self.rotation_offset, &key_pose.rotation_local);
            key_pose.translation_global = mat3::add(
                &mat3::mat_vec(&self.rotation_offset, &key_pose.translation_local),
                &self.translation_offset,
            );
        }
        // the solve judged every closure dispatched to it; one committed while it ran was not in it, so it stays unjudged
        for (index, weight) in solution.closure_weights.iter().copied().enumerate() {
            let previous = self.committed_closures[index].gnc_weight;
            self.committed_closures[index].gnc_weight = weight;
            if previous == weight {
                continue;
            }
            let (source, target) = (
                self.committed_closures[index].source_id,
                self.committed_closures[index].target_id,
            );
            let (confidence, yank_z) = (
                self.closure_fit_confidence[index],
                self.closure_yank_z[index],
            );
            if weight > 0.5 {
                self.push_edge(
                    source,
                    target,
                    ClosureEdgeState::Accepted,
                    confidence,
                    yank_z,
                );
                self.push_edge(
                    source,
                    target,
                    ClosureEdgeState::Applied,
                    confidence,
                    yank_z,
                );
            } else {
                self.push_edge(source, target, ClosureEdgeState::Rejected, 0.0, yank_z);
            }
        }
        self.graph_error = solution.error;
        true
    }
}

/// The odometry between consecutive keyframes, from their dead-reckoned poses, at the configured variances.
pub(crate) fn odometry_edges(key_poses: &[KeyPoseWithCloud], config: &Config) -> Vec<Edge> {
    let (tilt, yaw) = (config.odom_rot_roll_pitch_var, config.odom_rot_yaw_var);
    let (xy, z) = (config.odom_trans_xy_var, config.odom_trans_z_var);
    key_poses
        .windows(2)
        .enumerate()
        .map(|(i, pair)| {
            let back = mat3::transpose(&pair[0].rotation_local);
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

/// Committed closures as pose-graph edges; a closure's variance is its ICP fitness.
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
