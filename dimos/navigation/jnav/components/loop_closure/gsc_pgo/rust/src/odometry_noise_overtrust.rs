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

//! Overtrust: an odometry reported 100x more accurate than it is, closed by laps of a square. The solve at that noise
//! rejects the right closures, and once it keeps none the residuals give no evidence and the learner stays where it
//! is. With `closure_gnc_overtrust_check`, the solve keeps them and their residuals raise the learned noise; a true or
//! undertrusted (reported less accurate) odometry keeps them too.

use crate::gsc_pgo::{Config, GscPgo, KeyPoseWithCloud};
use crate::mat3::{self, Vec3};

fn uniform(k: usize, salt: f64) -> f64 {
    ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453)
        .rem_euclid(1.0)
        .max(1e-12)
}

fn normal(k: usize, salt: f64) -> f64 {
    (-2.0 * uniform(k, salt).ln()).sqrt() * (std::f64::consts::TAU * uniform(k, salt + 0.5)).cos()
}

/// The true per-keyframe noise (xy, z, yaw, roll/pitch).
const TRUE_NOISE: [f64; 4] = [4e-5, 1e-8, 2e-4, 1e-8];

/// Runs the learner (with the solve's overtrust check on or off) over laps of a 20 m square (a keyframe a metre), reported noise `TRUE_NOISE` x `reported_over_true`,
/// committing a closure every 4th keyframe after the first lap (one in ten a look-alike place several metres off) and
/// solving after each. Returns the share of right closures kept and the learned yaw variance over the true one.
fn run(reported_over_true: f64, overtrust_check: bool) -> (f64, f64) {
    let config = Config {
        // a keyframe a metre: the per-keyframe noise is the per-metre drift squared
        odom_xy_drift_per_sqrt_m: (TRUE_NOISE[0] * reported_over_true).sqrt(),
        odom_z_drift_per_sqrt_m: (TRUE_NOISE[1] * reported_over_true).sqrt(),
        odom_yaw_drift_per_sqrt_m: (TRUE_NOISE[2] * reported_over_true).sqrt(),
        odom_tilt_drift_per_sqrt_m: (TRUE_NOISE[3] * reported_over_true).sqrt(),
        odom_xy_drift_per_sqrt_rad: 0.0,
        odom_z_drift_per_sqrt_rad: 0.0,
        odom_yaw_drift_per_sqrt_rad: 0.0,
        odom_tilt_drift_per_sqrt_rad: 0.0,
        odometry_noise_from_closures: true,
        closure_uninformative_noise_upper_bound: true,
        closure_noise_from_residuals: true,
        odometry_noise_solve_lower_bound: true,
        closure_gnc_overtrust_check: overtrust_check,
        odom_noise_prior_ln_sd_xy: 1.72,
        odom_noise_prior_ln_sd_z: 2.05,
        odom_noise_prior_ln_sd_yaw: 1.15,
        odom_noise_prior_ln_sd_tilt: 0.96,
        ..Config::default()
    };
    let n = 400;
    let (mut truth, mut odometry) = (vec![(0.0_f64, [0.0; 3])], vec![(0.0_f64, [0.0; 3])]);
    for k in 1..n {
        let turn = if k % 20 == 0 {
            std::f64::consts::FRAC_PI_2
        } else {
            0.0
        };
        let (yaw, p) = truth[k - 1];
        truth.push((yaw + turn, mat3::add(&p, &[yaw.cos(), yaw.sin(), 0.0])));
        let (yaw, p) = odometry[k - 1];
        let step = [
            1.0 + TRUE_NOISE[0].sqrt() * normal(k, 2.0),
            TRUE_NOISE[0].sqrt() * normal(k, 3.0),
            0.0,
        ];
        odometry.push((
            yaw + turn + TRUE_NOISE[2].sqrt() * normal(k, 1.0),
            mat3::add(&p, &mat3::mat_vec(&mat3::rot_z(yaw), &step)),
        ));
    }
    let mut pgo = GscPgo::new(config);
    pgo.key_poses = odometry
        .iter()
        .map(|&(yaw, p)| KeyPoseWithCloud {
            rotation_local: mat3::rot_z(yaw),
            translation_local: p,
            rotation_global: mat3::rot_z(yaw),
            translation_global: p,
            time: 0.0,
            body_cloud: None,
        })
        .collect();
    pgo.keyframe_reach = (20.0, 1);
    let fitness = 0.05_f64.powi(2);
    let mut right = Vec::new();
    for (c, source) in (80..n).step_by(4).enumerate() {
        let target = source % 80;
        let ((yaw_t, p_t), (yaw_s, p_s)) = (truth[target], truth[source]);
        let mut measured: Vec3 = mat3::mat_vec(
            &mat3::transpose(&mat3::rot_z(yaw_t)),
            &mat3::sub(&p_s, &p_t),
        );
        let wrong = c % 10 == 9;
        for (axis, m) in measured.iter_mut().enumerate() {
            *m += fitness.sqrt() * normal(c * 3 + axis, 4.0);
            if wrong && axis < 2 {
                *m += 4.0 + 4.0 * uniform(c, 5.0 + axis as f64);
            }
        }
        right.push(!wrong);
        pgo.closure_fit_confidence.push(1.0);
        pgo.closure_yank_stdevs.push(0.0);
        pgo.commit_closure(
            source,
            target,
            mat3::rot_z(yaw_s - yaw_t),
            measured,
            fitness,
        );
        pgo.finish();
    }
    let kept = pgo
        .committed_closures
        .iter()
        .zip(&right)
        .filter(|(c, &r)| r && c.gnc_weight > 0.5)
        .count();
    let rights = right.iter().filter(|&&r| r).count();
    (
        kept as f64 / rights as f64,
        pgo.odometry_noise()[2] * reported_over_true,
    )
}

/// One design for both: the solve's overtrust check keeps the right closures that an overtrusted odometry would vote
/// out, or that GNC-TLS's odometry-led start loses once the learned noise runs low, and the noise learner, reading the
/// residuals of the closures the solve kept, raises the odometry noise from them, so later gating and solves use the
/// learned noise (no innovation mixture needed). Overtrusted, true and undertrusted starts all keep the right closures;
/// a true start stays within e of the truth, and a 100x wrong one moves towards it without passing it (the stated drift
/// counts as `odom_error_stiffness` loops of evidence, so these few laps move it only part of the way).
#[test]
fn the_overtrust_check_keeps_the_right_closures_and_teaches_the_learner() {
    for reported_over_true in [0.01, 1.0, 100.0] {
        let (kept_off, _) = run(reported_over_true, false);
        let (kept, yaw) = run(reported_over_true, true);
        eprintln!(
            "reported x{reported_over_true}: kept {kept_off:.2} -> {kept:.2}, yaw x{yaw:.3} of true"
        );
        assert!(
            kept > 0.9,
            "x{reported_over_true}: the right closures are kept ({kept})"
        );
        let (start, learned) = (reported_over_true.ln(), yaw.ln());
        assert!(
            if start == 0.0 {
                learned.abs() < 1.0
            } else {
                learned.abs() < start.abs() && learned.signum() == start.signum()
            },
            "x{reported_over_true}: yaw variance x{yaw} of true"
        );
    }
}

/// A learner that no committed closure has fed yet solves at the stated noise, as no learner would (a lower bound from
/// the prior's spread alone took an overtrusted odometry's solve lower still and starved the learner it waited on), and
/// gates at the prior's upper bound, admitting what might be news.
#[test]
fn an_unfed_learner_solves_at_the_stated_noise_and_gates_wide() {
    let pgo = GscPgo::new(Config {
        odometry_noise_from_closures: true,
        closure_uninformative_noise_upper_bound: true,
        odometry_noise_solve_lower_bound: true,
        odom_noise_prior_ln_sd_xy: 1.72,
        odom_noise_prior_ln_sd_z: 2.05,
        odom_noise_prior_ln_sd_yaw: 1.15,
        odom_noise_prior_ln_sd_tilt: 0.96,
        ..Config::default()
    });
    assert_eq!(pgo.odometry_noise_for_solve(), [1.0; 4]);
    assert!(pgo.odometry_noise_for_gate().iter().all(|&g| g > 1.0));
}
