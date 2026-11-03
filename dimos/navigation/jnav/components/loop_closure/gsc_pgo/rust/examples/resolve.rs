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

//! Re-solve a finished pgo-eval run's pose graph offline: its raw keyframe poses (`a.kf`) and committed closures
//! (`a.json`) through `big_picture_fit::solve` at a chosen odometry error estimate, so a solve-side change
//! can be judged in seconds instead of a replay. Writes the solved keyframes in a.kf's format and prints the kept count.
//! usage: resolve <run dir> <out.bin> [odometry error estimate factor, 1] [config overlay json, e.g. '{"closure_gnc_var_scale":4}']

use dimos_gsc_pgo::big_picture_fit::{solve, Edge, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let run = std::path::Path::new(&args[1]);
    let noise: f64 = args.get(3).map_or(1.0, |s| s.parse().unwrap());
    let json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(run.join("a.json")).unwrap()).unwrap();
    let mut config_json = json["config"]["pgo"].clone();
    if config_json.is_null() {
        config_json = json["config"].clone();
    }
    if let Some(overlay) = args.get(4) {
        let overlay: serde_json::Value = serde_json::from_str(overlay).unwrap();
        for (k, v) in overlay.as_object().unwrap() {
            config_json[k] = v.clone();
        }
    }
    let config: dimos_gsc_pgo::gsc_pgo::Config = serde_json::from_value(config_json).unwrap();
    // a.kf: ts, raw (x y z qx qy qz qw), corrected (same), f64 each
    let bytes = std::fs::read(run.join("a.kf")).unwrap();
    let values: Vec<f64> = bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|c| f64::from_le_bytes(*c))
        .collect();
    let poses: Vec<(Mat3, [f64; 3])> = values
        .as_chunks::<15>()
        .0
        .iter()
        .map(|r| {
            (
                mat3::mat_from_quat(&[r[7], r[4], r[5], r[6]]),
                [r[1], r[2], r[3]],
            )
        })
        .collect();
    let n = poses.len();
    let odometry: Vec<Edge> = poses
        .windows(2)
        .enumerate()
        .map(|(i, p)| {
            let back = mat3::transpose(&p[0].0);
            let step = dimos_gsc_pgo::big_picture_fit::motion_variances(
                &config,
                mat3::norm(&mat3::sub(&p[1].1, &p[0].1)),
                mat3::angular_distance(&p[0].0, &p[1].0),
            );
            let [xy, z, yaw, tilt] = step.map(|v| v * noise);
            Edge {
                from: i,
                to: i + 1,
                rotation: mat3::mat_mul(&back, &p[1].0),
                translation: mat3::mat_vec(&back, &mat3::sub(&p[1].1, &p[0].1)),
                variances: [tilt, tilt, yaw, xy, xy, z],
            }
        })
        .collect();
    let closures: Vec<Edge> = json["committed_closure_edges"]
        .as_array()
        .unwrap()
        .iter()
        .map(|e| {
            let rotation: Mat3 = serde_json::from_value(e["rotation"].clone()).unwrap();
            let translation: [f64; 3] = serde_json::from_value(e["translation"].clone()).unwrap();
            let score = e["score"].as_f64().unwrap().max(1e-6);
            Edge {
                from: e["target_id"].as_u64().unwrap() as usize,
                to: e["source_id"].as_u64().unwrap() as usize,
                rotation,
                translation,
                variances: [score; 6],
            }
        })
        .collect();
    let solve_config = SolveConfig {
        gnc_variance_scale: config.closure_gnc_var_scale,
        gnc_inlier_probability: config.closure_gnc_inlier_probability,
        huber_k: config.closure_robust_huber_k,
        evidence_core: false,
        learned_evidence: true,
        seed_closure_weights: Vec::new(),
        seed_start: Vec::new(),
    };
    let rotations: Vec<Mat3> = poses.iter().map(|p| p.0).collect();
    let positions: Vec<[f64; 3]> = poses.iter().map(|p| p.1).collect();
    let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
    let solution = solve(
        &rotations,
        &positions,
        &free,
        &odometry,
        &closures,
        &solve_config,
    );
    let kept = solution
        .closure_weights
        .iter()
        .filter(|&&w| w > 0.5)
        .count();
    println!("kept {kept} / {}", closures.len());
    // the same records as a.kf (ts, raw x y z qx qy qz qw, corrected likewise), the corrected pose the solve's
    let mut out = Vec::new();
    for (k, record) in values.as_chunks::<15>().0.iter().enumerate() {
        let q = mat3::quat_from_mat(&solution.rotations[k]);
        let corrected = [
            solution.positions[k][0],
            solution.positions[k][1],
            solution.positions[k][2],
            q[1],
            q[2],
            q[3],
            q[0],
        ];
        for v in record[..8].iter().chain(&corrected) {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    std::fs::write(&args[2], out).unwrap();
}
