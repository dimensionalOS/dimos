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

//! Re-solves a pgo-eval run's keyframes (raw odometry) with a `--landmark-sightings` CSV and its closures, as one final solve;
//! writes `<out>/a.kf` (corrected = this solve) for score_gt.py. A fast stand-in for a full inputs-only replay.
//! usage: landmark_resolve <run dir> <sightings csv> <out dir> [config overlay JSON] [--no-closures]

use dimos_gsc_pgo::big_picture_fit::{motion_variances, Edge, LandmarkSighting, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::world_positions::solve_with_priors;

fn main() {
    std::env::set_var("GSC_SOLVE_LOG", "/dev/null");
    let args: Vec<String> = std::env::args().collect();
    let (run, csv, out) = (
        std::path::Path::new(&args[1]),
        std::path::Path::new(&args[2]),
        std::path::Path::new(&args[3]),
    );
    let json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(run.join("a.json")).unwrap()).unwrap();
    let mut config_json = json["config"]["pgo"].clone();
    if let Some(overlay) = args.get(4).filter(|a| a.starts_with('{')) {
        let overlay: serde_json::Value = serde_json::from_str(overlay).unwrap();
        for (k, v) in overlay.as_object().unwrap() {
            config_json[k] = v.clone();
        }
    }
    let use_closures = !args.iter().any(|a| a == "--no-closures");
    let config: dimos_gsc_pgo::gsc_pgo::Config = serde_json::from_value(config_json).unwrap();
    let bytes = std::fs::read(run.join("a.kf")).unwrap();
    let values: Vec<f64> = bytes.as_chunks::<8>().0.iter().map(|c| f64::from_le_bytes(*c)).collect();
    let rows: Vec<[f64; 15]> = values.as_chunks::<15>().0.to_vec();
    let times: Vec<f64> = rows.iter().map(|r| r[0]).collect();
    let poses: Vec<(Mat3, Vec3)> = rows
        .iter()
        .map(|r| (mat3::mat_from_quat(&[r[7], r[4], r[5], r[6]]), [r[1], r[2], r[3]]))
        .collect();
    let n = poses.len();
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
    let closures: Vec<Edge> = if use_closures {
        json["committed_closure_edges"]
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
            .collect()
    } else {
        Vec::new()
    };
    // sightings on their nearest keyframes, variance grown by the gap's motion (as `GscPgo::sightings`)
    let mut ids: Vec<u64> = Vec::new();
    let mut sightings = Vec::new();
    // a sighting id's newest row (16 values: publish time, sighting id, then a 14-value row) replaces its earlier ones
    let mut csv_rows: Vec<Vec<f64>> = Vec::new();
    let mut row_of_sighting = std::collections::HashMap::new();
    for line in std::fs::read_to_string(csv).unwrap().lines().filter(|l| !l.trim().is_empty() && !l.starts_with('#')) {
        let v: Vec<f64> = line.split(',').map(|x| x.trim().parse().unwrap()).collect();
        if v.len() != 16 {
            csv_rows.push(v);
            continue;
        }
        let index = *row_of_sighting.entry(v[1] as u64).or_insert_with(|| {
            csv_rows.push(Vec::new());
            csv_rows.len() - 1
        });
        csv_rows[index] = v[2..].to_vec();
    }
    for v in csv_rows {
        let time = v[0];
        if time < times[0] || time > times[n - 1] {
            continue;
        }
        let after = times.partition_point(|&t| t < time).clamp(1, n - 1);
        let keyframe = if time - times[after - 1] <= times[after] - time { after - 1 } else { after };
        let speed = mat3::norm(&mat3::sub(&poses[after].1, &poses[after - 1].1))
            / (times[after] - times[after - 1]).max(f64::EPSILON);
        let moved = (speed * (time - times[keyframe]).abs()).powi(2);
        let id = v[1] as u64;
        let landmark = ids.iter().position(|&i| i == id).unwrap_or_else(|| {
            ids.push(id);
            ids.len() - 1
        });
        let mut covariance: Mat3 = std::array::from_fn(|i| std::array::from_fn(|j| v[5 + 3 * i + j]));
        for (axis, row) in covariance.iter_mut().enumerate() {
            row[axis] += moved;
        }
        sightings.push(LandmarkSighting::point(keyframe, landmark, [v[2], v[3], v[4]], covariance));
    }
    let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = poses.iter().copied().unzip();
    let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
    let started = std::time::Instant::now();
    let solution = solve_with_priors(
        &rotations,
        &positions,
        &free,
        &odometry,
        &closures,
        &[],
        &sightings,
        &SolveConfig {
            gnc_variance_scale: config.closure_gnc_var_scale,
            gnc_inlier_probability: config.closure_gnc_inlier_probability,
            huber_k: config.closure_robust_huber_k,
            evidence_core: false,
            learned_evidence: false,
            seed_closure_weights: Vec::new(),
            seed_start: Vec::new(),
        },
    );
    let kept = solution.sighting_weights.iter().filter(|&&w| w > 0.5).count();
    eprintln!(
        "landmark_resolve: {n} keyframes, {} closures ({} kept), {} sightings ({kept} kept), converged {}, {:.2}s",
        closures.len(),
        solution.closure_weights.iter().filter(|&&w| w > 0.5).count(),
        sightings.len(),
        solution.converged,
        started.elapsed().as_secs_f64()
    );
    std::fs::create_dir_all(out).unwrap();
    let mut bytes = Vec::with_capacity(n * 120);
    for (k, row) in rows.iter().enumerate() {
        let q = mat3::quat_from_mat(&solution.rotations[k]);
        let p = solution.positions[k];
        let corrected = [p[0], p[1], p[2], q[1], q[2], q[3], q[0]];
        for v in row[..8].iter().chain(&corrected) {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    std::fs::write(out.join("a.kf"), bytes).unwrap();
    std::fs::write(out.join("a.kf.closures"), []).unwrap();
}
