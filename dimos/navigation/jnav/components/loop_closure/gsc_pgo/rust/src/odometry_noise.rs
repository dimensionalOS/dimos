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

//! The odometry's real noise, measured from the closures the solve kept.
//!
//! Each kept closure measures the source keyframe's position in the target's frame; the raw odometry claims another
//! position, and the difference (the innovation) is the odometry's error over that stretch plus the closure's own. The
//! odometry's error is modelled as the per-keyframe noise of four independent sources: xy translation, z translation,
//! yaw and roll/pitch, each step's heading noise swinging the rest of the stretch by its lever arm (the model of
//! `loop_overlap`). Their variances are separate components (Helmert / Förstner variance component estimation), since
//! translation and heading noise grow differently with a stretch's length (linearly, and about cubically) and are
//! misreported by different factors; a single scale on the reported noise cannot fit both. The reported variances are
//! unit-information priors (Kass & Wasserman 1995), so a few closures barely move them and many outweigh them.

use crate::gsc_pgo::KeyPoseWithCloud;
use crate::mat3::{self, Mat3, Vec3};

/// The noise components in order: xy translation, z translation, yaw, roll/pitch (per keyframe step, m² or rad²).
pub const COMPONENTS: usize = 4;
/// How many axes of a 3-D innovation each component shows in (horizontal 2, vertical 1); one innovation's Fisher
/// information on a component's ln variance is half that, which is the weight of its unit-information prior.
const AXES: [f64; COMPONENTS] = [2.0, 1.0, 2.0, 1.0];

/// One kept closure's evidence on the odometry noise.
#[derive(Clone, Copy, Debug)]
pub struct Innovation {
    /// The closure's measurement minus the raw odometry: the source's position in the target's frame (m).
    pub v: Vec3,
    /// The innovation's covariance per unit variance of each component, in the target's frame.
    pub pieces: [Mat3; COMPONENTS],
    /// The closure's ICP fitness: its modelled translation variance per axis (m²) before the closure scale.
    pub fitness: f64,
}

/// A closure's innovation against the raw odometry from keyframe `target` to `source`, given the source's measured
/// position in the target's frame.
pub fn innovation(
    key_poses: &[KeyPoseWithCloud],
    target: usize,
    source: usize,
    measured: &Vec3,
    fitness: f64,
) -> Innovation {
    let (from, to) = (target.min(source), target.max(source));
    let rotation = |k: usize| key_poses[k].rotation_local;
    let position = |k: usize| key_poses[k].translation_local;
    let to_target = mat3::transpose(&rotation(target));
    let raw = mat3::mat_vec(&to_target, &mat3::sub(&position(source), &position(target)));
    let outer = |m: &Mat3, column: usize| -> Mat3 {
        std::array::from_fn(|i| std::array::from_fn(|j| m[i][column] * m[j][column]))
    };
    let mut pieces = [[[0.0; 3]; 3]; COMPONENTS];
    for k in from + 1..=to {
        // step k-1 -> k: translation noise along k-1's axes, heading noise about k's axes swinging the rest of the way
        // to the far end
        let along = mat3::mat_mul(&to_target, &rotation(k - 1));
        let far = if source > target { source } else { target };
        let lever = mat3::skew(&mat3::mat_vec(
            &to_target,
            &mat3::sub(&position(far), &position(k)),
        ));
        let swing = mat3::mat_mul(&lever, &mat3::mat_mul(&to_target, &rotation(k)));
        let add = |piece: &mut Mat3, m: Mat3| *piece = mat3::add_mat(piece, &m);
        add(
            &mut pieces[0],
            mat3::add_mat(&outer(&along, 0), &outer(&along, 1)),
        );
        add(&mut pieces[1], outer(&along, 2));
        add(&mut pieces[2], outer(&swing, 2));
        add(
            &mut pieces[3],
            mat3::add_mat(&outer(&swing, 0), &outer(&swing, 1)),
        );
    }
    Innovation {
        v: mat3::sub(measured, &raw),
        pieces,
        fitness,
    }
}

/// The ln ratios to the reported values of the four odometry variances, and the ln closure scale (each 0 unless
/// fitted), most probable given the innovations: each is zero-mean Gaussian with covariance
/// S = sum_c reported_c e^(r_c) pieces_c + e^(k) fitness I, so this minimizes
/// sum(ln det S + v^T S^-1 v) + sum_c (axes_c / 2) r_c^2 + 1.5 k^2. One ln at a time: a grid (numerical range only)
/// finds its basin and golden-section search refines it, alternated until they settle. None without an innovation.
pub fn fit(
    innovations: &[Innovation],
    reported: [f64; COMPONENTS],
    fit_odometry: bool,
    fit_closure: bool,
) -> Option<([f64; COMPONENTS], f64)> {
    if innovations.is_empty() {
        return None;
    }
    let cost = |odometry: &[f64; COMPONENTS], closure: f64| -> f64 {
        let variances: [f64; COMPONENTS] = std::array::from_fn(|c| reported[c] * odometry[c].exp());
        let k = closure.exp();
        innovations
            .iter()
            .map(|n| {
                let covariance: Mat3 = std::array::from_fn(|i| {
                    std::array::from_fn(|j| {
                        (0..COMPONENTS)
                            .map(|c| variances[c] * n.pieces[c][i][j])
                            .sum::<f64>()
                            + if i == j { k * n.fitness } else { 0.0 }
                    })
                });
                let Some(inverse) = mat3::inverse(&covariance) else {
                    return f64::INFINITY;
                };
                let iv = mat3::mat_vec(&inverse, &n.v);
                mat3::det(&covariance).ln() + n.v[0] * iv[0] + n.v[1] * iv[1] + n.v[2] * iv[2]
            })
            .sum::<f64>()
            + (0..COMPONENTS)
                .map(|c| AXES[c] / 2.0 * odometry[c] * odometry[c])
                .sum::<f64>()
            + 1.5 * closure * closure
    };
    let minimize = |f: &dyn Fn(f64) -> f64, start: f64| -> f64 {
        let (mut best, mut best_cost) = (start, f(start));
        for step in -60..=60 {
            let t = step as f64 * 0.5;
            let c = f(t);
            if c < best_cost {
                (best, best_cost) = (t, c);
            }
        }
        let (mut low, mut high) = (best - 0.5, best + 0.5);
        let golden = (5f64.sqrt() - 1.0) / 2.0;
        for _ in 0..60 {
            let (a, b) = (high - golden * (high - low), low + golden * (high - low));
            if f(a) < f(b) {
                high = b;
            } else {
                low = a;
            }
        }
        (low + high) / 2.0
    };
    let (mut odometry, mut closure) = ([0.0; COMPONENTS], 0.0);
    for _ in 0..30 {
        let previous = (odometry, closure);
        if fit_odometry {
            for c in 0..COMPONENTS {
                odometry[c] = minimize(
                    &|t| {
                        let mut trial = odometry;
                        trial[c] = t;
                        cost(&trial, closure)
                    },
                    odometry[c],
                );
            }
        }
        if fit_closure {
            closure = minimize(&|t| cost(&odometry, t), closure);
        }
        let moved = (0..COMPONENTS)
            .map(|c| (odometry[c] - previous.0[c]).abs())
            .fold((closure - previous.1).abs(), f64::max);
        if moved < 1e-6 {
            break;
        }
    }
    Some((odometry, closure))
}

/// The standard error of a component's fitted ln variance after `count` innovations (and the prior's one).
pub fn standard_error(component: usize, count: usize) -> f64 {
    (1.0 / (AXES[component] / 2.0 * (count + 1) as f64)).sqrt()
}
