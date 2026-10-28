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

//! An odometry reported 100x too confident, closed by laps of a square: the solve at that noise rejects every right
//! closure, so the residuals never see the odometry is noisier and the learner stays stuck where it started. With
//! `odometry_noise_innovation_check`, the closures' innovations against the raw odometry raise it, and the solve keeps
//! them.

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

/// Runs the learner over laps of a 20 m square (a keyframe a metre), reported noise `TRUE_NOISE` x `reported_over_true`,
/// committing a closure every 4th keyframe after the first lap (one in ten a look-alike place several metres off) and
/// solving after each. Returns the share of right closures kept and the learned yaw variance over the true one.
fn run(reported_over_true: f64, innovation_check: bool) -> (f64, f64) {
    let config = Config {
        odom_trans_xy_var: TRUE_NOISE[0] * reported_over_true,
        odom_trans_z_var: TRUE_NOISE[1] * reported_over_true,
        odom_rot_yaw_var: TRUE_NOISE[2] * reported_over_true,
        odom_rot_roll_pitch_var: TRUE_NOISE[3] * reported_over_true,
        odom_noise_per_motion: false,
        odometry_noise_from_closures: true,
        closure_uninformative_noise_upper_bound: true,
        closure_noise_from_residuals: true,
        odometry_noise_solve_lower_bound: true,
        odometry_noise_innovation_check: innovation_check,
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
        pgo.closure_yank_z.push(0.0);
        pgo.commit_closure(
            source,
            target,
            mat3::rot_z(yaw_s - yaw_t),
            measured,
            fitness,
        );
        pgo.finish();
        if std::env::var("TRACE").is_ok() {
            let kept = pgo
                .committed_closures
                .iter()
                .filter(|c| c.gnc_weight > 0.5)
                .count();
            eprintln!(
                "x{reported_over_true} {innovation_check} c{c} kept {kept}/{} noise {:?} solve {:?} mix {:?} from_mix {}",
                pgo.committed_closures.len(),
                pgo.odometry_noise().map(|v| (v * reported_over_true * 1000.0).round() / 1000.0),
                pgo.odometry_noise_for_solve().map(|v| (v * reported_over_true * 1000.0).round() / 1000.0),
                pgo.odometry_noise_mixture.map(|(m, _)| m.odometry.map(|v| (v.exp() * reported_over_true * 1000.0).round() / 1000.0)),
                pgo.odometry_noise_from_mixture
            );
        }
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

#[test]
fn overconfident_odometry_is_corrected_by_the_closures_it_rejects() {
    let (stuck_kept, stuck_yaw) = run(0.01, false);
    eprintln!(
        "without the check: {stuck_kept:.2} of right closures kept, yaw x{stuck_yaw:.3} of true"
    );
    assert!(
        stuck_kept < 0.1,
        "the overconfident solve rejects the right closures ({stuck_kept})"
    );
    assert!(
        stuck_yaw < 0.1,
        "and the residuals cannot raise the noise ({stuck_yaw})"
    );
    let (kept, yaw) = run(0.01, true);
    eprintln!("with the check: {kept:.2} of right closures kept, yaw x{yaw:.3} of true");
    assert!(kept > 0.9, "the right closures are kept ({kept})");
    assert!(yaw.ln().abs() < 1.0, "yaw variance x{yaw} of true");
}

#[test]
fn the_check_leaves_a_well_stated_or_underconfident_odometry_alone() {
    for reported_over_true in [1.0, 100.0] {
        let (kept_off, yaw_off) = run(reported_over_true, false);
        let (kept, yaw) = run(reported_over_true, true);
        eprintln!(
            "reported x{reported_over_true}: kept {kept_off:.2} -> {kept:.2}, yaw x{yaw_off:.3} -> x{yaw:.3} of true"
        );
        assert!(kept >= kept_off - 0.05, "kept {kept_off} -> {kept}");
        assert!(yaw.ln().abs() < 1.0, "yaw variance x{yaw} of true");
    }
}
