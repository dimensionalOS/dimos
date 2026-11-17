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

//! Re-solves a pgo-eval run's logged solves (solve.jsonl: keyframes, closures) from the odometry, as a solve whose seed
//! does not fit starts, and prints GNC's verdicts, its final cost and how far the result sits from the raw odometry chain
//! (an all-rejected solve's optimum is that chain, cost 0); a JSON line per solve.
//! usage: final_cost_check <run dir> [first keyframes, 0] [last keyframes, all]

use dimos_gsc_pgo::big_picture_fit::{motion_variances, solve, Edge, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

fn main() {
    // the solve's trace (GNC stop, challenge costs) is only kept when GSC_SOLVE_LOG is set
    if std::env::var_os("GSC_SOLVE_LOG").is_none() {
        std::env::set_var("GSC_SOLVE_LOG", "/dev/null");
    }
    let args: Vec<String> = std::env::args().collect();
    let run = std::path::Path::new(&args[1]);
    let first: usize = args.get(2).map_or(0, |s| s.parse().unwrap());
    let last: usize = args.get(3).map_or(usize::MAX, |s| s.parse().unwrap());
    let json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(run.join("a.json")).unwrap()).unwrap();
    let mut config_json = json["config"]["pgo"].clone();
    if config_json.is_null() {
        config_json = json["config"].clone();
    }
    let config: dimos_gsc_pgo::gsc_pgo::Config = serde_json::from_value(config_json).unwrap();
    let bytes = std::fs::read(run.join("a.kf")).unwrap();
    let values: Vec<f64> = bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|c| f64::from_le_bytes(*c))
        .collect();
    let poses: Vec<(Mat3, Vec3)> = values
        .as_chunks::<15>()
        .0
        .iter()
        .map(|r| (mat3::mat_from_quat(&[r[7], r[4], r[5], r[6]]), [r[1], r[2], r[3]]))
        .collect();
    let odometry: Vec<Edge> = poses
        .windows(2)
        .enumerate()
        .map(|(i, p)| {
            let back = mat3::transpose(&p[0].0);
            let [xy, z, yaw, tilt] = motion_variances(
                &config,
                mat3::norm(&mat3::sub(&p[1].1, &p[0].1)),
                mat3::angular_distance(&p[0].0, &p[1].0),
            );
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
        .map(|e| Edge {
            from: e["target_id"].as_u64().unwrap() as usize,
            to: e["source_id"].as_u64().unwrap() as usize,
            rotation: serde_json::from_value(e["rotation"].clone()).unwrap(),
            translation: serde_json::from_value(e["translation"].clone()).unwrap(),
            variances: [e["score"].as_f64().unwrap().max(1e-6); 6],
        })
        .collect();
    // each logged solve once: (keyframes, closures) as dispatched
    let mut problems: Vec<(usize, usize)> = Vec::new();
    for line in std::fs::read_to_string(run.join("solve.jsonl")).unwrap().lines() {
        let row: serde_json::Value = serde_json::from_str(line).unwrap();
        let problem = (
            row["dispatch_keyframes"].as_u64().unwrap() as usize,
            row["closures"].as_u64().unwrap() as usize,
        );
        if problems.last() != Some(&problem) && (first..=last).contains(&problem.0) {
            problems.push(problem);
        }
    }
    for &(n, count) in &problems {
        let n = n.min(poses.len());
        let edges: Vec<Edge> = closures[..count.min(closures.len())]
            .iter()
            .filter(|e| e.from.max(e.to) < n)
            .copied()
            .collect();
        let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
        let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = poses[..n].iter().copied().unzip();
        let solve_config = SolveConfig {
            gnc_variance_scale: config.closure_gnc_var_scale,
            gnc_inlier_probability: config.closure_gnc_inlier_probability,
            huber_k: config.closure_robust_huber_k,
            evidence_core: false,
            learned_evidence: false,
            seed_closure_weights: Vec::new(),
            incumbent_closure_weights: Vec::new(),
            seed_start: Vec::new(),
            planar: config.planar_pose_graph,
            landmark_warm_start_xy_variance_scale: config.landmark_warm_start_xy_variance_scale,
            landmark_warm_start_xy_variance_ladder: config.landmark_warm_start_xy_variance_ladder,
        };
        let started = std::time::Instant::now();
        let solution = solve(&rotations, &positions, &free, &odometry[..n - 1], &edges, &solve_config);
        let seconds = started.elapsed().as_secs_f64();
        let trace = solution.trace.as_ref().unwrap();
        // the raw chain re-anchored at keyframe 0 (pinned), against which an all-rejected solve should land
        let from_raw = (0..n)
            .map(|k| mat3::norm(&mat3::sub(&solution.positions[k], &positions[k])))
            .fold(0.0, f64::max);
        println!(
            "{{\"keyframes\":{n},\"closures\":{},\"kept\":{},\"gnc_iterations\":{},\"stop\":\"{}\",\"converged\":{},\"final_cost\":{:?},\"from_raw_m\":{from_raw:.2},\"s\":{seconds:.2}}}",
            edges.len(),
            solution.closure_weights.iter().filter(|&&w| w > 0.5).count(),
            trace.gnc_iterations,
            trace.gnc_stop,
            solution.converged,
            trace.final_cost,
        );
    }
}
