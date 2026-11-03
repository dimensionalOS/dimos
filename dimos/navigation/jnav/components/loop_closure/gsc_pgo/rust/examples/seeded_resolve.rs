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

//! Replay a finished pgo-eval run's solves offline, each from scratch or seeded from the one before, so the seeded
//! solve (`solve_seed_from_accepted`, `solve_seed_gnc_verdicts`) can be judged in seconds: its raw keyframe poses
//! (`a.kf`) and committed closures (`a.json`, in commit order) are solved after every `step` closures, over the
//! keyframes up to the newest closure's source, three ways: from the odometry, seeded with the previous solve's poses
//! (the keyframes since riding its newest correction; taken only where they already fit), and with those poses plus
//! its closure verdicts. Prints one JSON line
//! per solve (seconds, GNC iterations, kept closures, largest position gap to the unseeded solve) and a summary.
//! usage: seeded_resolve <run dir> [step, 1]

use dimos_gsc_pgo::big_picture_fit::{motion_variances, solve, Edge, Solution, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

struct Chain {
    previous: Option<(usize, Solution)>,
    seeded: usize,
    seconds: Vec<f64>,
    iterations: Vec<usize>,
}

fn main() {
    // the solve's trace (GNC iterations) is only kept with a solve log set; solve never writes it
    if std::env::var_os("GSC_SOLVE_LOG").is_none() {
        std::env::set_var("GSC_SOLVE_LOG", "/dev/null");
    }
    let args: Vec<String> = std::env::args().collect();
    let run = std::path::Path::new(&args[1]);
    let step: usize = args.get(2).map_or(1, |s| s.parse().unwrap());
    let json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(run.join("a.json")).unwrap()).unwrap();
    let mut config_json = json["config"]["pgo"].clone();
    if config_json.is_null() {
        config_json = json["config"].clone();
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
    let mut chains: Vec<Chain> = (0..3)
        .map(|_| Chain {
            previous: None,
            seeded: 0,
            seconds: Vec::new(),
            iterations: Vec::new(),
        })
        .collect();
    let (mut kept_differs, mut worst_gap, mut final_gap) = ([0usize; 3], [0.0f64; 3], [0.0f64; 3]);
    let mut ends: Vec<usize> = (step..=closures.len()).step_by(step).collect();
    if ends.last() != Some(&closures.len()) {
        ends.push(closures.len());
    }
    for &count in &ends {
        let edges = &closures[..count];
        let n = edges.iter().map(|e| e.from.max(e.to) + 1).max().unwrap_or(1).max(2);
        let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
        let mut solutions: Vec<Solution> = Vec::new();
        for (way, chain) in chains.iter_mut().enumerate() {
            let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = poses[..n].iter().copied().unzip();
            let (mut seed_start, mut seed_closure_weights) = (Vec::new(), Vec::new());
            if way > 0 {
                if let Some((before, previous)) = &chain.previous {
                    // the keyframes the previous solve placed where it put them, the rest riding its newest correction
                    let last = before - 1;
                    let ride_rotation =
                        mat3::mat_mul(&previous.rotations[last], &mat3::transpose(&poses[last].0));
                    let ride_translation = mat3::sub(
                        &previous.positions[last],
                        &mat3::mat_vec(&ride_rotation, &poses[last].1),
                    );
                    seed_start = (0..n)
                        .map(|k| {
                            if k < *before {
                                (previous.rotations[k], previous.positions[k])
                            } else {
                                (
                                    mat3::mat_mul(&ride_rotation, &poses[k].0),
                                    mat3::add(&mat3::mat_vec(&ride_rotation, &poses[k].1), &ride_translation),
                                )
                            }
                        })
                        .collect();
                    if way == 2 {
                        seed_closure_weights = (0..count)
                            .map(|i| previous.closure_weights.get(i).copied().unwrap_or(f64::NAN))
                            .collect();
                    }
                }
            }
            let solve_config = SolveConfig {
                gnc_variance_scale: config.closure_gnc_var_scale,
                gnc_inlier_probability: config.closure_gnc_inlier_probability,
                huber_k: config.closure_robust_huber_k,
                evidence_core: false,
                learned_evidence: false,
                seed_closure_weights,
                seed_start,
            };
            let started = std::time::Instant::now();
            let solution = solve(
                &rotations,
                &positions,
                &free,
                &odometry[..n - 1],
                edges,
                &solve_config,
            );
            chain.seconds.push(started.elapsed().as_secs_f64());
            chain
                .iterations
                .push(solution.trace.as_ref().map_or(0, |t| t.gnc_iterations));
            solutions.push(solution);
        }
        let gaps: Vec<f64> = solutions
            .iter()
            .map(|s| {
                (0..n)
                    .map(|k| mat3::norm(&mat3::sub(&s.positions[k], &solutions[0].positions[k])))
                    .fold(0.0, f64::max)
            })
            .collect();
        for way in 0..3 {
            if solutions[way].closure_weights != solutions[0].closure_weights {
                kept_differs[way] += 1;
            }
            worst_gap[way] = worst_gap[way].max(gaps[way]);
            final_gap[way] = gaps[way];
        }
        let kept = |s: &Solution| s.closure_weights.iter().filter(|&&w| w > 0.5).count();
        println!(
            "{{\"closures\":{count},\"keyframes\":{n},\"seconds\":[{:.4},{:.4},{:.4}],\"gnc_iterations\":[{},{},{}],\"kept\":[{},{},{}],\"gap_m\":[{:.3},{:.3},{:.3}],\"error\":[{:.3},{:.3},{:.3}],\"converged\":[{},{},{}],\"seeded\":[{},{},{}]}}",
            chains[0].seconds.last().unwrap(),
            chains[1].seconds.last().unwrap(),
            chains[2].seconds.last().unwrap(),
            chains[0].iterations.last().unwrap(),
            chains[1].iterations.last().unwrap(),
            chains[2].iterations.last().unwrap(),
            kept(&solutions[0]),
            kept(&solutions[1]),
            kept(&solutions[2]),
            gaps[0],
            gaps[1],
            gaps[2],
            solutions[0].error,
            solutions[1].error,
            solutions[2].error,
            solutions[0].converged,
            solutions[1].converged,
            solutions[2].converged,
            solutions[0].seeded,
            solutions[1].seeded,
            solutions[2].seeded,
        );
        for (chain, solution) in chains.iter_mut().zip(solutions) {
            chain.seeded += solution.seeded as usize;
            chain.previous = Some((n, solution));
        }
    }
    let median = |v: &[f64]| {
        let mut v = v.to_vec();
        v.sort_by(f64::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(0.0)
    };
    for (way, name) in ["odometry", "poses", "poses+verdicts"].iter().enumerate() {
        let chain = &chains[way];
        let iterations: Vec<f64> = chain.iterations.iter().map(|&i| i as f64).collect();
        println!(
            "SUMMARY {name}: solves {} (seeded {}) total {:.2} s median {:.4} s max {:.3} s, GNC iterations median {} max {}, kept differs from odometry-start in {} solves, largest gap {:.3} m, final gap {:.3} m",
            chain.seconds.len(),
            chain.seeded,
            chain.seconds.iter().sum::<f64>(),
            median(&chain.seconds),
            chain.seconds.iter().copied().fold(0.0, f64::max),
            median(&iterations),
            iterations.iter().copied().fold(0.0, f64::max),
            kept_differs[way],
            worst_gap[way],
            final_gap[way],
        );
    }
}
