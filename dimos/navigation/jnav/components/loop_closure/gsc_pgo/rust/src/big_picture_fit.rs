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

use faer::linalg::solvers::SolveCore;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{Argsort, Pair, SparseColMat, SymbolicSparseColMat};
use faer::{Conj, Mat, Side};

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
}

pub struct Solution {
    pub rotations: Vec<Mat3>,
    pub positions: Vec<Vec3>,
    /// Per closure: 1 kept, 0 rejected (GNC's final weight, rounded at 0.5).
    pub closure_weights: Vec<f64>,
    /// Half the sum of squared whitened residuals of everything kept.
    pub error: f64,
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
    let pattern = Pattern::new(free, odometry.iter().chain(closures));

    // GNC with truncated least squares (Yang et al. 2020): start convex (every closure in), then tighten
    // until each closure is clearly in or out. Odometry is always in.
    let threshold = chi2_inverse_6dof(config.gnc_inlier_probability);
    let residual =
        |state: &State| -> Vec<f64> { scaled.iter().map(|e| whitened_squared(e, state)).collect() };
    // GNC's starting mu comes from the residuals at the starting estimate, before any solve (as GTSAM's
    // GncOptimizer does): the worse the start, the more gently it tightens
    let worst = residual(&state).into_iter().fold(0.0, f64::max);
    let mut weights = vec![1.0; closures.len()];
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

    // the kept closures at their own variances, Huber-weighted
    let mut huber = kept.clone();
    for _ in 0..3 {
        levenberg_marquardt(&mut state, &pattern, odometry, closures, &huber, 1e-5);
        for (h, (e, &k)) in huber.iter_mut().zip(closures.iter().zip(&kept)) {
            let r = whitened_squared(e, &state).sqrt();
            *h = if config.huber_k > 0.0 && r > config.huber_k {
                k * config.huber_k / r
            } else {
                k
            };
        }
    }
    let error = 0.5 * cost(&state, odometry, closures, &huber);
    Solution {
        rotations: state.rotation,
        positions: state.position,
        closure_weights: kept,
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

fn cost(state: &State, odometry: &[Edge], closures: &[Edge], closure_weights: &[f64]) -> f64 {
    odometry
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
    fn new<'a>(free: &[bool], edges: impl Iterator<Item = &'a Edge>) -> Pattern {
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
    tolerance: f64,
) {
    let size = pattern.size;
    let mut damping = 1e-6;
    let mut current = cost(state, odometry, closures, closure_weights);
    for _ in 0..100 {
        let (mut values, gradient, diagonal) =
            normal_equations(state, pattern, odometry, closures, closure_weights);
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
            let trial_cost = cost(&trial, odometry, closures, closure_weights);
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
    (values, gradient, diagonal)
}

/// How hard the graph resists a proposed closure: the rise in its error if the closure were forced in
/// rigidly, to first order. That is the closure's residual at the current poses measured against the
/// graph's uncertainty between its two keyframes (every path between them, earlier closures
/// included) plus the closure's own: r^T (J Sigma J^T + closure variance)^-1 r. `rotations` /
/// `positions` should be the graph's solution with `closures` (weighted by `closure_weights`).
pub fn strain(
    rotations: &[Mat3],
    positions: &[Vec3],
    odometry: &[Edge],
    closures: &[Edge],
    closure_weights: &[f64],
    candidate: &Edge,
) -> f64 {
    let n = rotations.len();
    if n < 2 {
        return 0.0;
    }
    let state = State {
        rotation: rotations.to_vec(),
        position: positions.to_vec(),
    };
    let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
    let pattern = Pattern::new(&free, odometry.iter().chain(closures));
    let size = pattern.size;
    let (mut values, _, diagonal) =
        normal_equations(&state, &pattern, odometry, closures, closure_weights);
    values.extend(diagonal.iter().map(|d| 1e-9 * (d + 1e-9)));
    let hessian = SparseColMat::new_from_argsort(pattern.symbolic.clone(), &pattern.order, &values)
        .expect("pose graph hessian");
    let Ok(llt) = Llt::try_new_with_symbolic(pattern.llt.clone(), hessian.as_ref(), Side::Lower)
    else {
        return f64::INFINITY;
    };
    // the covariance columns of the closure's two keyframes (keyframe 0 is fixed, so it has none)
    let ends = [candidate.from, candidate.to];
    let mut columns = Mat::<f64>::zeros(size, 12);
    for (end, &k) in ends.iter().enumerate() {
        if let Some(slot) = pattern.slot[k] {
            for i in 0..6 {
                columns[(6 * slot + i, 6 * end + i)] = 1.0;
            }
        }
    }
    llt.solve_in_place_with_conj(Conj::No, columns.as_mut());
    let mut joint = [[0.0; 12]; 12];
    for (end_a, &a) in ends.iter().enumerate() {
        let Some(slot) = pattern.slot[a] else {
            continue;
        };
        for end_b in 0..2 {
            for i in 0..6 {
                for j in 0..6 {
                    joint[6 * end_a + i][6 * end_b + j] = columns[(6 * slot + i, 6 * end_b + j)];
                }
            }
        }
    }
    let (r, jf, jt) = linearize(candidate, &state);
    let jacobian = |k: usize, c: usize| if c < 6 { jf[k][c] } else { jt[k][c - 6] };
    let mut total = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            total[i][j] = (0..12)
                .map(|a| {
                    (0..12)
                        .map(|b| jacobian(i, a) * joint[a][b] * jacobian(j, b))
                        .sum::<f64>()
                })
                .sum::<f64>();
        }
        total[i][i] += candidate.variances[i];
    }
    let solved = solve6(total, r);
    (0..6).map(|i| r[i] * solved[i]).sum()
}

/// x with a x = b for a 6x6 `a` (Gaussian elimination with partial pivoting).
fn solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> [f64; 6] {
    for col in 0..6 {
        let pivot = (col..6)
            .max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))
            .unwrap();
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..6 {
            let factor = a[row][col] / a[col][col];
            for k in col..6 {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = [0.0; 6];
    for row in (0..6).rev() {
        x[row] = (b[row] - (row + 1..6).map(|k| a[row][k] * x[k]).sum::<f64>()) / a[row][row];
    }
    x
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

// ---- the pose graph GSC keeps, re-solved on a background thread whenever closures arrive ----

/// A pose-graph solve running on another thread over the first `keyframes` keyframes, adopted once
/// the map reaches `due_at` keyframes or, on request, as soon as it finishes (see `GscPgo::update`).
pub(crate) struct Solving {
    handle: JoinHandle<(Solution, f64)>,
    keyframes: usize,
    due_at: usize,
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
    /// Whether a solve is running or closures are waiting for one.
    pub fn solve_pending(&self) -> bool {
        self.solving.is_some() || self.committed_closures.len() > self.closures_dispatched
    }
    /// Apply a finished solve, and start one when closures have arrived since the last. A solve is
    /// adopted once the map reaches its `due_at` keyframes (waiting for it if needed), or, with
    /// `now`, as soon as it has finished. Returns whether keyframes moved.
    pub fn update(&mut self, now: bool) -> bool {
        self.new_closure = false;
        let keyframes = self.key_poses.len();
        let ready = self.solving.as_ref().is_some_and(|solving| {
            keyframes >= solving.due_at || (now && solving.handle.is_finished())
        });
        let moved = ready && self.apply_solve();
        if self.solving.is_none() && self.committed_closures.len() > self.closures_dispatched {
            self.dispatch_solve();
        }
        moved
    }
    /// Wait for the running solve, and solve again until every closure is in. Returns whether keyframes moved.
    pub fn finish(&mut self) -> bool {
        if let Some((result, _)) = self.pending_search.take() {
            self.finish_closure_search(result);
        }
        let mut moved = false;
        loop {
            moved |= self.apply_solve();
            if self.committed_closures.len() == self.closures_dispatched {
                return moved;
            }
            self.dispatch_solve();
        }
    }
    /// Start a solve of every keyframe so far and every committed closure, from the odometry (GNC must start
    /// from the dead-reckoned chain, not an estimate earlier closures already bent).
    fn dispatch_solve(&mut self) {
        let n = self.key_poses.len();
        let config = &self.config;
        let rotations: Vec<Mat3> = self.key_poses.iter().map(|k| k.rotation_local).collect();
        let positions: Vec<Vec3> = self.key_poses.iter().map(|k| k.translation_local).collect();
        let odometry = odometry_edges(&self.key_poses, config);
        let closures = closure_edges(self.committed_closures.iter());
        // keyframe 0 pins the map
        let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
        let solve_config = SolveConfig {
            gnc_variance_scale: config.closure_gnc_var_scale,
            gnc_inlier_probability: config.closure_gnc_inlier_probability,
            huber_k: config.closure_robust_huber_k,
        };
        let handle = std::thread::Builder::new()
            .name("gsc-pose-graph".into())
            .spawn(move || {
                let started = std::time::Instant::now();
                let solution = solve(
                    &rotations,
                    &positions,
                    &free,
                    &odometry,
                    &closures,
                    &solve_config,
                );
                (solution, started.elapsed().as_secs_f64())
            })
            .expect("spawn pose graph thread");
        let delay = (n as f64 * self.config.closure_gnc_apply_after_fraction).ceil() as usize;
        self.closures_dispatched = self.committed_closures.len();
        self.solving = Some(Solving {
            handle,
            keyframes: n,
            due_at: n + delay.max(1),
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
        let n = solving.keyframes;
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
        for (index, &weight) in solution.closure_weights.iter().enumerate() {
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
