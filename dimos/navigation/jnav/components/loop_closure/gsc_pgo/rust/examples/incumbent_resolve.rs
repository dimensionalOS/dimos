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

//! Replays a pgo-eval run's solves every `step` closures, unseeded vs seeded (poses / poses+verdicts); JSON line per solve.
//! usage: seeded_resolve <run dir> [step, 1]
//! Replays a pgo-eval run's logged solves (solve.jsonl: keyframes, closures) as two chains, each seeded from its own
//! previous solve: GNC alone, and GNC with the incumbent challenge; a JSON line per solve, then each chain's kept-0 count.
//! usage: incumbent_resolve <run dir> [first keyframes, 0] [last keyframes, all]

use dimos_gsc_pgo::big_picture_fit::{motion_variances, solve, Edge, Solution, SolveConfig};
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
    let kept = |s: &Solution| s.closure_weights.iter().filter(|&&w| w > 0.5).count();
    let mut previous: [Option<(usize, Solution)>; 2] = [None, None];
    let mut kept_zero_after_kept = [0usize; 2];
    for &(n, count) in &problems {
        let n = n.min(poses.len());
        let edges: Vec<Edge> = closures[..count.min(closures.len())]
            .iter()
            .filter(|e| e.from.max(e.to) < n)
            .copied()
            .collect();
        let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
        let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = poses[..n].iter().copied().unzip();
        let mut line = format!("{{\"keyframes\":{n},\"closures\":{}", edges.len());
        for (way, slot) in previous.iter_mut().enumerate() {
            let (mut seed_start, mut incumbent) = (Vec::new(), Vec::new());
            if let Some((before, solution)) = slot.as_ref() {
                // previously solved keyframes where it put them, newer ones riding its newest correction
                let last = before - 1;
                let ride_rotation =
                    mat3::mat_mul(&solution.rotations[last], &mat3::transpose(&poses[last].0));
                let ride_translation =
                    mat3::sub(&solution.positions[last], &mat3::mat_vec(&ride_rotation, &poses[last].1));
                seed_start = (0..n)
                    .map(|k| {
                        if k < *before {
                            (solution.rotations[k], solution.positions[k])
                        } else {
                            (
                                mat3::mat_mul(&ride_rotation, &poses[k].0),
                                mat3::add(&mat3::mat_vec(&ride_rotation, &poses[k].1), &ride_translation),
                            )
                        }
                    })
                    .collect();
                if way == 1 {
                    incumbent = (0..edges.len())
                        .map(|i| solution.closure_weights.get(i).copied().unwrap_or(f64::NAN))
                        .collect();
                }
            }
            let solve_config = SolveConfig {
                gnc_variance_scale: config.closure_gnc_var_scale,
                gnc_inlier_probability: config.closure_gnc_inlier_probability,
                huber_k: config.closure_robust_huber_k,
                evidence_core: false,
                learned_evidence: false,
                seed_closure_weights: Vec::new(),
                incumbent_closure_weights: incumbent,
                seed_start,
                planar: config.planar_pose_graph,
                landmark_warm_start_xy_variance_scale: config.landmark_warm_start_xy_variance_scale,
                landmark_warm_start_xy_variance_ladder: config.landmark_warm_start_xy_variance_ladder,
            };
            let started = std::time::Instant::now();
            let solution = solve(&rotations, &positions, &free, &odometry[..n - 1], &edges, &solve_config);
            let seconds = started.elapsed().as_secs_f64();
            let trace = solution.trace.as_ref().unwrap();
            // how far this solve moved the previous solve's keyframes
            let moved = slot.as_ref().map_or(0.0, |(before, p)| {
                (0..*before)
                    .map(|k| mat3::norm(&mat3::sub(&solution.positions[k], &p.positions[k])))
                    .fold(0.0, f64::max)
            });
            let was = slot.as_ref().map_or(0, |(_, p)| kept(p));
            if was > 0 && kept(&solution) == 0 {
                kept_zero_after_kept[way] += 1;
            }
            line += &format!(
                ",\"{}\":{{\"kept\":{},\"was\":{was},\"stop\":\"{}\",\"challenge\":[{:.1},{:.1}],\"moved_m\":{moved:.1},\"s\":{seconds:.2}}}",
                ["gnc", "challenge"][way],
                kept(&solution),
                trace.gnc_stop,
                trace.challenge_cost[0],
                trace.challenge_cost[1],
            );
            *slot = Some((n, solution));
        }
        println!("{line}}}");
    }
    println!("SUMMARY kept-0 after a solve that kept some: gnc {}, challenge {}", kept_zero_after_kept[0], kept_zero_after_kept[1]);
}
