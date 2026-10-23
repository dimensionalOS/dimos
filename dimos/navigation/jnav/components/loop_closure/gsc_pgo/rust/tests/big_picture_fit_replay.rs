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

//! Re-solve a pgo-eval run's final graph (its keyframes' odometry + every committed closure) with big_picture_fit::solve,
//! pose graph, and write the keyframe positions next to the run as pose_graph.bin (f64 x, y, z per keyframe).
//! GSC_RUN=<run dir> cargo test --release --test pose_graph_replay -- --ignored --nocapture

use dimos_gsc_pgo::big_picture_fit::{residuals, solve, Edge, SolveConfig};
use dimos_gsc_pgo::mat3;

#[test]
#[ignore]
fn replay_a_runs_final_graph() {
    let run = std::path::PathBuf::from(std::env::var("GSC_RUN").expect("GSC_RUN=<run dir>"));
    let json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(run.join("a.json")).unwrap()).unwrap();
    let config = &json["config"];
    let num = |k: &str| config[k].as_f64().unwrap();
    let raw: Vec<f64> = std::fs::read(run.join("a.kf"))
        .unwrap()
        .chunks_exact(8)
        .map(|b| f64::from_le_bytes(b.try_into().unwrap()))
        .collect();
    let (mut rotations, mut positions, mut corrected) = (Vec::new(), Vec::new(), Vec::new());
    for k in raw.chunks_exact(15) {
        positions.push([k[1], k[2], k[3]]);
        rotations.push(mat3::mat_from_quat(&[k[7], k[4], k[5], k[6]]));
        corrected.push([k[8], k[9], k[10]]);
    }
    let free: Vec<bool> = (0..rotations.len()).map(|k| k > 0).collect();
    let odometry: Vec<Edge> = (1..rotations.len())
        .map(|i| {
            let back = mat3::transpose(&rotations[i - 1]);
            let (rp, yaw, xy, z) = (
                num("odom_rot_roll_pitch_var"),
                num("odom_rot_yaw_var"),
                num("odom_trans_xy_var"),
                num("odom_trans_z_var"),
            );
            Edge {
                from: i - 1,
                to: i,
                rotation: mat3::mat_mul(&back, &rotations[i]),
                translation: mat3::mat_vec(&back, &mat3::sub(&positions[i], &positions[i - 1])),
                variances: [rp, rp, yaw, xy, xy, z],
            }
        })
        .collect();
    let floor = num("closure_translation_variance_floor");
    let committed = json["committed_closure_edges"].as_array().unwrap();
    let closures: Vec<Edge> = committed
        .iter()
        .map(|e| {
            let rotation: Vec<Vec<f64>> = serde_json::from_value(e["rotation"].clone()).unwrap();
            let translation: Vec<f64> = serde_json::from_value(e["translation"].clone()).unwrap();
            let score = e["score"].as_f64().unwrap().max(1e-6);
            let t = score.max(floor);
            Edge {
                from: e["target_id"].as_u64().unwrap() as usize,
                to: e["source_id"].as_u64().unwrap() as usize,
                rotation: [0, 1, 2].map(|r| [rotation[r][0], rotation[r][1], rotation[r][2]]),
                translation: [translation[0], translation[1], translation[2]],
                variances: [score, score, score, t, t, t],
            }
        })
        .collect();
    let solve_config = SolveConfig {
        gnc_variance_scale: num("closure_gnc_var_scale"),
        gnc_inlier_probability: num("closure_gnc_inlier_probability"),
        huber_k: if config["closure_robust_kernel"].as_bool().unwrap() {
            num("closure_robust_huber_k")
        } else {
            0.0
        },
    };
    let gtsam_kept: Vec<bool> = committed
        .iter()
        .map(|e| e["gnc_weight"].as_f64().unwrap() >= 0.5)
        .collect();
    if std::env::var("GSC_CHECK").is_ok() {
        // the closures' residuals (GNC-scaled) at GTSAM's own final poses, split by GTSAM's GNC verdict
        let corrected_rotations: Vec<_> = raw
            .chunks_exact(15)
            .map(|k| mat3::mat_from_quat(&[k[14], k[11], k[12], k[13]]))
            .collect();
        let scaled: Vec<Edge> = closures
            .iter()
            .map(|e| Edge {
                variances: e.variances.map(|v| v * solve_config.gnc_variance_scale),
                ..*e
            })
            .collect();
        let r2 = residuals(&corrected_rotations, &corrected, &scaled);
        for want in [true, false] {
            let mut v: Vec<f64> = r2
                .iter()
                .zip(&gtsam_kept)
                .filter(|(_, &k)| k == want)
                .map(|(r, _)| *r)
                .collect();
            v.sort_by(f64::total_cmp);
            if !v.is_empty() {
                println!(
                    "GTSAM {}: {} closures, r2 median {:.2} p90 {:.2} max {:.2}",
                    if want { "kept" } else { "rejected" },
                    v.len(),
                    v[v.len() / 2],
                    v[v.len() * 9 / 10],
                    v[v.len() - 1]
                );
            }
        }
    }
    let started = std::time::Instant::now();
    let solution = if std::env::var("GSC_ONLY_GTSAM_KEPT").is_ok() {
        // only the closures GTSAM's GNC kept, with GNC effectively off: isolates the solver from the inlier decisions
        let kept: Vec<Edge> = closures
            .iter()
            .zip(&gtsam_kept)
            .filter(|(_, &k)| k)
            .map(|(e, _)| *e)
            .collect();
        let mut s = solve(
            &rotations,
            &positions,
            &free,
            &odometry,
            &kept,
            &SolveConfig {
                gnc_inlier_probability: 1.0,
                ..solve_config.clone()
            },
        );
        s.closure_weights = gtsam_kept
            .iter()
            .map(|&k| if k { 1.0 } else { 0.0 })
            .collect();
        s
    } else {
        solve(
            &rotations,
            &positions,
            &free,
            &odometry,
            &closures,
            &solve_config,
        )
    };
    let agree = solution
        .closure_weights
        .iter()
        .zip(&gtsam_kept)
        .filter(|(w, g)| (**w >= 0.5) == **g)
        .count();
    let moved: Vec<f64> = solution
        .positions
        .iter()
        .zip(&corrected)
        .map(|(a, b)| mat3::norm(&mat3::sub(a, b)))
        .collect();
    let mut sorted = moved.clone();
    sorted.sort_by(f64::total_cmp);
    println!(
        "{} keyframes, {} closures ({} kept, {} agree with GTSAM's GNC), solved in {:.2} s; vs GTSAM: median {:.3} m, max {:.3} m",
        rotations.len(),
        closures.len(),
        solution.closure_weights.iter().filter(|&&w| w >= 0.5).count(),
        agree,
        started.elapsed().as_secs_f64(),
        sorted[sorted.len() / 2],
        sorted[sorted.len() - 1],
    );
    let bytes: Vec<u8> = solution
        .positions
        .iter()
        .flatten()
        .flat_map(|v| v.to_le_bytes())
        .collect();
    std::fs::write(run.join("pose_graph.bin"), bytes).unwrap();
}
