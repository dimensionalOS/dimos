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
//! misreported by different factors; a single scale on the reported noise cannot fit both. The reported noise is the
//! prior: held to by its measured spread across recordings (`odom_noise_prior_ln_sd_*`), or else by one closure's worth
//! of information (a unit-information prior, Kass & Wasserman 1995), so a few closures barely move it.

use crate::gsc_pgo::{Config, KeyPoseWithCloud};
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
    /// The innovation's covariance at the reported noise of each component (each step's `step_variances`), in the
    /// target's frame: the component's ratio to the reported noise scales it.
    pub pieces: [Mat3; COMPONENTS],
    /// The closure's ICP fitness: its modelled translation variance per axis (m²).
    pub fitness: f64,
}

/// A closure's innovation against the raw odometry from keyframe `target` to `source`, given the source's measured
/// position in the target's frame.
pub fn innovation(
    key_poses: &[KeyPoseWithCloud],
    config: &Config,
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
        let step = crate::big_picture_fit::step_variances(config, &key_poses[k - 1], &key_poses[k]);
        let add = |piece: &mut Mat3, m: Mat3, variance: f64| {
            *piece = mat3::add_mat(piece, &m.map(|row| row.map(|v| v * variance)))
        };
        add(
            &mut pieces[0],
            mat3::add_mat(&outer(&along, 0), &outer(&along, 1)),
            step[0],
        );
        add(&mut pieces[1], outer(&along, 2), step[1]);
        add(&mut pieces[2], outer(&swing, 2), step[2]);
        add(
            &mut pieces[3],
            mat3::add_mat(&outer(&swing, 0), &outer(&swing, 1)),
            step[3],
        );
    }
    Innovation {
        v: mat3::sub(measured, &raw),
        pieces,
        fitness,
    }
}

/// The ln ratios to the reported values of the four odometry variances and the share of the closures that are right,
/// most probable given the innovations. A right closure's innovation is
/// zero-mean Gaussian with covariance S = sum_c e^(r_c) pieces_c + fitness I; a wrong one (a look-alike
/// place) points anywhere a scan could have been matched to, uniform with density `outlier_density`. Every committed
/// closure counts, weighed by that mixture rather than by the solve's verdict, which was itself reached with the noise
/// being estimated (low noise rejects the right closures that would raise it). This minimizes
/// -2 sum ln(w N(v; 0, S) + (1 - w) outlier_density) + sum_c r_c^2 / sd_c^2 over the lns and the share w
/// (uniform prior). One at a time: a grid (numerical range only) finds its basin and golden-section search refines it,
/// alternated until they settle. None without an innovation.
pub fn fit(
    innovations: &[Innovation],
    prior_sd: [f64; COMPONENTS],
    outlier_density: f64,
) -> Option<([f64; COMPONENTS], f64)> {
    if innovations.is_empty() {
        return None;
    }
    let ln_outlier = outlier_density.ln();
    let ln_two_pi = (2.0 * std::f64::consts::PI).ln();
    // the share of right closures as a logit
    let cost = |odometry: &[f64; COMPONENTS], logit: f64| -> f64 {
        let variances: [f64; COMPONENTS] = std::array::from_fn(|c| odometry[c].exp());
        let share = 1.0 / (1.0 + (-logit).exp());
        let (ln_right, ln_wrong) = (share.ln(), (1.0 - share).ln());
        innovations
            .iter()
            .map(|n| {
                let covariance: Mat3 = std::array::from_fn(|i| {
                    std::array::from_fn(|j| {
                        (0..COMPONENTS)
                            .map(|c| variances[c] * n.pieces[c][i][j])
                            .sum::<f64>()
                            + if i == j { n.fitness } else { 0.0 }
                    })
                });
                let Some(inverse) = mat3::inverse(&covariance) else {
                    return f64::INFINITY;
                };
                let iv = mat3::mat_vec(&inverse, &n.v);
                let quadratic = n.v[0] * iv[0] + n.v[1] * iv[1] + n.v[2] * iv[2];
                let ln_gaussian =
                    -0.5 * (quadratic + mat3::det(&covariance).ln() + 3.0 * ln_two_pi);
                let (a, b) = (ln_right + ln_gaussian, ln_wrong + ln_outlier);
                let high = a.max(b);
                -2.0 * (high + ((a - high).exp() + (b - high).exp()).ln())
            })
            .sum::<f64>()
            + (0..COMPONENTS)
                .map(|c| odometry[c] * odometry[c] * prior_weight(c, prior_sd[c]))
                .sum::<f64>()
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
    let (mut odometry, mut logit) = ([0.0; COMPONENTS], 0.0);
    for _ in 0..30 {
        let previous = (odometry, logit);
        for c in 0..COMPONENTS {
            odometry[c] = minimize(
                &|t| {
                    let mut trial = odometry;
                    trial[c] = t;
                    cost(&trial, logit)
                },
                odometry[c],
            );
        }
        logit = minimize(&|t| cost(&odometry, t), logit);
        let moved = (0..COMPONENTS)
            .map(|c| (odometry[c] - previous.0[c]).abs())
            .fold((logit - previous.1).abs(), f64::max);
        if moved < 1e-6 {
            break;
        }
    }
    Some((odometry, 1.0 / (1.0 + (-logit).exp())))
}

/// A noise source's ln ratio to its reported variance from the solved graph's residuals: they were whitened at
/// e^(dispatched), and sum ~ e^(u - dispatched) chi-square(redundancy) for the true ln ratio u, so this minimizes
/// redundancy (u - dispatched) + sum e^(dispatched - u) + prior_weight u^2 (-2 ln of likelihood times prior). Its
/// standard error is about sqrt(2 / redundancy).
pub fn residual_ln_ratio(
    sum: f64,
    redundancy: f64,
    dispatched: f64,
    component: usize,
    prior_sd: f64,
) -> f64 {
    residual_ln_ratio_weighted(
        sum,
        redundancy,
        dispatched,
        prior_weight(component, prior_sd),
    )
}

/// `residual_ln_ratio` with the prior's weight given.
fn residual_ln_ratio_weighted(sum: f64, redundancy: f64, dispatched: f64, weight: f64) -> f64 {
    let f = |u: f64| redundancy * (u - dispatched) + sum * (dispatched - u).exp() + weight * u * u;
    // Newton on a convex function, from the likelihood's own maximum
    let mut u = dispatched + (sum / redundancy).max(f64::MIN_POSITIVE).ln();
    for _ in 0..50 {
        let e = sum * (dispatched - u).exp();
        let (gradient, curvature) = (redundancy - e + 2.0 * weight * u, e + 2.0 * weight);
        let next = u - gradient / curvature;
        if (next - u).abs() < 1e-10 {
            break;
        }
        u = next;
    }
    if f(u).is_finite() {
        u
    } else {
        dispatched
    }
}

/// The posterior standard deviation of `residual_ln_ratio`'s result `u` (from the curvature of the same function).
pub fn residual_ln_ratio_sd(
    sum: f64,
    dispatched: f64,
    u: f64,
    component: usize,
    prior_sd: f64,
) -> f64 {
    let curvature = sum * (dispatched - u).exp() + 2.0 * prior_weight(component, prior_sd);
    (2.0 / curvature).sqrt()
}

/// A component's prior weight on its ln ratio squared, in -2 ln units: 1 / sd^2 for a measured spread across
/// recordings, else one innovation's information (axes / 2, a unit-information prior).
fn prior_weight(component: usize, sd: f64) -> f64 {
    if sd > 0.0 {
        1.0 / (sd * sd)
    } else {
        AXES[component] / 2.0
    }
}

/// The standard error of a component's fitted ln variance after `count` innovations (and the prior's one).
pub fn standard_error(component: usize, count: usize) -> f64 {
    (1.0 / (AXES[component] / 2.0 * (count + 1) as f64)).sqrt()
}
