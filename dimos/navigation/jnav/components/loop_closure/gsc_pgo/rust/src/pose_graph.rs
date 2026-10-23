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

//! A pose graph over keyframe poses: Levenberg-Marquardt on a sparse Cholesky, with graduated non-convexity (truncated least squares) deciding which loops are inliers by
//! how well they agree with the odometry and with each other, then a Huber pass on the kept ones.

use faer::linalg::solvers::SolveCore;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{Argsort, Pair, SparseColMat, SymbolicSparseColMat};
use faer::{Conj, Mat, Side};

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
    /// Loop variances are multiplied by this while GNC decides inliers (looser, so drift alone is not an outlier).
    pub gnc_variance_scale: f64,
    /// Probability that an inlier loop's residual is within GNC's threshold (sets the chi-square cutoff).
    pub gnc_inlier_probability: f64,
    /// Huber threshold on a kept loop's whitened residual; 0 = plain least squares.
    pub huber_k: f64,
}

pub struct Solution {
    pub rotations: Vec<Mat3>,
    pub positions: Vec<Vec3>,
    /// Per loop: 1 kept, 0 rejected (GNC's final weight, rounded at 0.5).
    pub loop_weights: Vec<f64>,
    /// Half the sum of squared whitened residuals of everything kept.
    pub error: f64,
}

/// Solve for every keyframe's pose from the starting estimate `rotations` / `positions`. Keyframe 0 is held fixed.
pub fn solve(
    rotations: &[Mat3],
    positions: &[Vec3],
    odometry: &[Edge],
    loops: &[Edge],
    config: &SolveConfig,
) -> Solution {
    let mut state = State {
        rotation: rotations.to_vec(),
        position: positions.to_vec(),
    };
    let scaled: Vec<Edge> = loops
        .iter()
        .map(|e| Edge {
            variances: e.variances.map(|v| v * config.gnc_variance_scale),
            ..*e
        })
        .collect();
    let pattern = Pattern::new(rotations.len(), odometry.iter().chain(loops));

    // GNC with truncated least squares (Yang et al. 2020): start convex (every loop in), then tighten
    // until each loop is clearly in or out. Odometry is always in.
    let threshold = chi2_inverse_6dof(config.gnc_inlier_probability);
    let residual =
        |state: &State| -> Vec<f64> { scaled.iter().map(|e| whitened_squared(e, state)).collect() };
    // GNC's starting mu comes from the residuals at the starting estimate, before any solve (as GTSAM's
    // GncOptimizer does): the worse the start, the more gently it tightens
    let worst = residual(&state).into_iter().fold(0.0, f64::max);
    let mut weights = vec![1.0; loops.len()];
    levenberg_marquardt(&mut state, &pattern, odometry, &scaled, &weights, 1e-5);
    let mut r2 = residual(&state);
    if worst > threshold {
        let mut mu = threshold / (2.0 * worst - threshold).max(f64::EPSILON);
        let mut previous_cost = f64::INFINITY;
        for _ in 0..100 {
            for (w, &r) in weights.iter_mut().zip(&r2) {
                let (upper, lower) = ((mu + 1.0) / mu * threshold, mu / (mu + 1.0) * threshold);
                *w = if r >= upper {
                    0.0
                } else if r <= lower {
                    1.0
                } else {
                    ((threshold * mu * (mu + 1.0) / r).sqrt() - mu).clamp(0.0, 1.0)
                };
            }
            levenberg_marquardt(&mut state, &pattern, odometry, &scaled, &weights, 1e-5);
            r2 = residual(&state);
            // stop like GTSAM's GncOptimizer: weights settled, or the weighted cost stopped changing
            let weighted_cost = cost(&state, odometry, &scaled, &weights);
            if weights.iter().all(|&w| w < 1e-4 || w > 1.0 - 1e-4)
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

    // the kept loops at their own variances, Huber-weighted
    let mut huber = kept.clone();
    for _ in 0..3 {
        levenberg_marquardt(&mut state, &pattern, odometry, loops, &huber, 1e-5);
        for (h, (e, &k)) in huber.iter_mut().zip(loops.iter().zip(&kept)) {
            let r = whitened_squared(e, &state).sqrt();
            *h = if config.huber_k > 0.0 && r > config.huber_k {
                k * config.huber_k / r
            } else {
                k
            };
        }
    }
    let error = 0.5 * cost(&state, odometry, loops, &huber);
    Solution {
        rotations: state.rotation,
        positions: state.position,
        loop_weights: kept,
        error,
    }
}

/// Each edge's squared whitened residual at the given poses.
pub fn residuals(rotations: &[Mat3], positions: &[Vec3], edges: &[Edge]) -> Vec<f64> {
    let state = State {
        rotation: rotations.to_vec(),
        position: positions.to_vec(),
    };
    edges.iter().map(|e| whitened_squared(e, &state)).collect()
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
    let mut jr_inv = skew(&phi);
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
    let skew_local = skew(&local);
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

fn skew(v: &Vec3) -> Mat3 {
    [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
}

fn cost(state: &State, odometry: &[Edge], loops: &[Edge], loop_weights: &[f64]) -> f64 {
    odometry
        .iter()
        .map(|e| whitened_squared(e, state))
        .sum::<f64>()
        + loops
            .iter()
            .zip(loop_weights)
            .map(|(e, w)| w * whitened_squared(e, state))
            .sum::<f64>()
}

/// The normal equations' sparsity: 6x6 blocks for keyframes 1.. (keyframe 0 is held fixed), with every
/// entry's place fixed once, so each iteration only fills values (in `entries` order) instead of re-sorting.
struct Pattern {
    n: usize,
    symbolic: SymbolicSparseColMat<usize>,
    order: Argsort<usize>,
    llt: SymbolicLlt<usize>,
}

/// The keyframe-block pairs (row >= col, both > 0) an edge between `from` and `to` adds to, in fill order.
fn edge_blocks(from: usize, to: usize) -> impl Iterator<Item = (usize, usize)> {
    [(from, from), (to, to), (from.max(to), from.min(to))]
        .into_iter()
        .filter(|&(a, b)| a > 0 && b > 0)
}

/// Lower-triangle (row, col) indices of the 6x6 block at keyframes (a, b), shifted past the fixed keyframe 0.
fn block_entries(a: usize, b: usize) -> impl Iterator<Item = (usize, usize, usize, usize)> {
    (0..6)
        .flat_map(move |i| (0..6).map(move |j| (i, j, 6 * (a - 1) + i, 6 * (b - 1) + j)))
        .filter(|&(_, _, r, c)| r >= c)
}

impl Pattern {
    fn new<'a>(n: usize, edges: impl Iterator<Item = &'a Edge>) -> Pattern {
        let size = 6 * (n - 1).max(1);
        let mut indices: Vec<Pair<usize, usize>> = Vec::new();
        for e in edges {
            for (a, b) in edge_blocks(e.from, e.to) {
                indices.extend(block_entries(a, b).map(|(_, _, r, c)| Pair::new(r, c)));
            }
        }
        // the damping diagonal last, so the matrix is never structurally singular
        indices.extend((0..size).map(|i| Pair::new(i, i)));
        let (symbolic, order) = SymbolicSparseColMat::try_new_from_indices(size, size, &indices)
            .expect("pose graph pattern");
        let llt =
            SymbolicLlt::try_new(symbolic.as_ref(), Side::Lower).expect("pose graph ordering");
        Pattern {
            n,
            symbolic,
            order,
            llt,
        }
    }
}

/// Gauss-Newton steps, damped (and retried) whenever a step would raise the cost; done once a step lowers
/// the cost by less than `tolerance` of it.
fn levenberg_marquardt(
    state: &mut State,
    pattern: &Pattern,
    odometry: &[Edge],
    loops: &[Edge],
    loop_weights: &[f64],
    tolerance: f64,
) {
    let n = pattern.n;
    if n < 2 {
        return;
    }
    let size = 6 * (n - 1);
    let mut damping = 1e-6;
    let mut current = cost(state, odometry, loops, loop_weights);
    for _ in 0..100 {
        let mut values: Vec<f64> = Vec::new();
        let mut gradient = vec![0.0; size];
        let mut diagonal = vec![0.0; size];
        let weighted = odometry
            .iter()
            .map(|e| (e, 1.0))
            .chain(loops.iter().zip(loop_weights.iter().copied()));
        for (e, w) in weighted {
            let (r, jf, jt) = linearize(e, state);
            let jacobian = |k: usize| if k == e.from { &jf } else { &jt };
            for (a, b) in edge_blocks(e.from, e.to) {
                // the (a, b) block of J^T W J is ja^T W jb
                let (ja, jb) = (jacobian(a), jacobian(b));
                for (i, j, row, col) in block_entries(a, b) {
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
                if k > 0 {
                    for i in 0..6 {
                        gradient[6 * (k - 1) + i] -= (0..6)
                            .map(|m| jk[m][i] * w / e.variances[m] * r[m])
                            .sum::<f64>();
                    }
                }
            }
        }
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
            for k in 1..n {
                let delta: Vec<f64> = (0..6).map(|i| step[(6 * (k - 1) + i, 0)]).collect();
                (trial.rotation[k], trial.position[k]) = state.retract(k, &delta);
            }
            let trial_cost = cost(&trial, odometry, loops, loop_weights);
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

/// The chi-square value a 6-DoF residual stays under with probability `p` (CDF 1 - e^{-x/2}(1 + x/2 + x²/8)).
fn chi2_inverse_6dof(p: f64) -> f64 {
    let (mut lo, mut hi) = (0.0f64, 200.0f64);
    for _ in 0..100 {
        let x = 0.5 * (lo + hi);
        let half = x / 2.0;
        if 1.0 - (-half).exp() * (1.0 + half + half * half / 2.0) < p {
            lo = x;
        } else {
            hi = x;
        }
    }
    0.5 * (lo + hi)
}

/// Rotation by axis-angle `w` (Rodrigues).
fn exp(w: Vec3) -> Mat3 {
    let angle = mat3::norm(&w);
    let k = [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]];
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
