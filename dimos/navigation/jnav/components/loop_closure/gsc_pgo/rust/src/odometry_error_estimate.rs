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

//! The odometry's real noise, measured per source (xy, z, yaw, tilt) from the solved graph's residuals (Helmert /
//! Förstner variance components), the reported noise as the prior (`learned_odometry_error_estimate_prior_ln_sd_*`).

/// The noise components in order: xy translation, z translation, yaw, roll/pitch (per keyframe step, m² or rad²).
pub const COMPONENTS: usize = 4;
/// Axes of a 3-D innovation each component shows in; half of it is a component's unit-information prior weight.
const AXES: [f64; COMPONENTS] = [2.0, 1.0, 2.0, 1.0];

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
