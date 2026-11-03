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

//! Re-drives a pgo-eval run's graph through `GscPgo` without its clouds: its keyframes (raw odometry, each followed by
//! the scans that came before the next) and its committed closures, each committed at the scan its search landed at, with
//! the deterministic solve schedule. A fast stand-in for a full replay to compare graph settings (e.g. `graph_simplify`)
//! on the same closures; writes `<out>/a.kf` for score_gt.py.
//! usage: simplify_redrive <run dir> <out dir> [config overlay JSON]

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (run, out) = (std::path::Path::new(&args[1]), std::path::Path::new(&args[2]));
    let json: serde_json::Value = serde_json::from_slice(&std::fs::read(run.join("a.json")).unwrap()).unwrap();
    let mut config_json = json["config"]["pgo"].clone();
    if config_json.is_null() {
        config_json = json["config"].clone();
    }
    // cloud-less: a keyframe is made as soon as it is due, and the stated drift stays as the run ended with it
    config_json["auto_aggregate_cloud_size"] = 0.into();
    config_json["odometry_error_estimate_from_lidar_range"] = false.into();
    if let Some(overlay) = args.get(3).filter(|a| a.starts_with('{')) {
        let overlay: serde_json::Value = serde_json::from_str(overlay).unwrap();
        for (k, v) in overlay.as_object().unwrap() {
            config_json[k] = v.clone();
        }
    }
    let config: Config = serde_json::from_value(config_json).unwrap();
    let bytes = std::fs::read(run.join("a.kf")).unwrap();
    let values: Vec<f64> = bytes.as_chunks::<8>().0.iter().map(|c| f64::from_le_bytes(*c)).collect();
    let rows: Vec<[f64; 15]> = values.as_chunks::<15>().0.to_vec();
    // the accepted searches in landing order, paired with the committed closures (same order)
    let accepted: Vec<&serde_json::Value> = json["closure_diagnostics"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|d| d["outcome"] == "accepted")
        .collect();
    let committed = json["committed_closure_edges"].as_array().unwrap();
    assert_eq!(accepted.len(), committed.len(), "accepted searches vs committed closures");
    let scans = json["scans_processed"].as_u64().unwrap() as usize;
    let per_keyframe = scans as f64 / rows.len() as f64;
    let started = std::time::Instant::now();
    let mut pgo = GscPgo::new(config);
    pgo.set_wait_for_due_jobs(true);
    let (mut next_closure, mut scan) = (0usize, 0usize);
    for (k, row) in rows.iter().enumerate() {
        let rotation = mat3::mat_from_quat(&[row[7], row[4], row[5], row[6]]);
        let mut pose = PoseWithTime::new(rotation, [row[1], row[2], row[3]]);
        pose.set_time(row[0] as i32, (row[0].fract() * 1e9) as u32);
        let until = ((((k + 1) as f64) * per_keyframe).round() as usize).max(scan + 1);
        let mut first = true;
        while scan < until {
            let made = pgo.add_key_pose(&CloudWithPose { cloud: None, pose: pose.clone() });
            assert!(!first || made, "keyframe {k} was not due");
            first = false;
            scan += 1;
            while next_closure < accepted.len()
                && accepted[next_closure]["scan_index_finished"].as_u64().unwrap() as usize <= scan
                && committed[next_closure]["source_id"].as_u64().unwrap() as usize <= k
            {
                let e = &committed[next_closure];
                pgo.commit_closure(
                    e["source_id"].as_u64().unwrap() as usize,
                    e["target_id"].as_u64().unwrap() as usize,
                    serde_json::from_value(e["rotation"].clone()).unwrap(),
                    serde_json::from_value(e["translation"].clone()).unwrap(),
                    e["score"].as_f64().unwrap(),
                );
                next_closure += 1;
            }
            pgo.wait_for_due_solve();
        }
    }
    assert_eq!(next_closure, committed.len(), "closures left uncommitted");
    pgo.finish();
    std::fs::create_dir_all(out).unwrap();
    let mut kf = Vec::new();
    for (k, row) in rows.iter().enumerate() {
        let (rotation, position) = pgo.key_poses().global(k);
        let [qw, qx, qy, qz] = mat3::quat_from_mat(&rotation);
        for v in row[..8].iter().chain(&[position[0], position[1], position[2], qx, qy, qz, qw]) {
            kf.extend_from_slice(&v.to_le_bytes());
        }
    }
    std::fs::write(out.join("a.kf"), kf).unwrap();
    std::fs::write(out.join("a.kf.closures"), []).unwrap();
    let stats = pgo.simplify_stats();
    println!(
        "{}",
        serde_json::json!({
            "keyframes": rows.len(),
            "simplify": stats,
            "closures": committed.len(),
            "kept_closures": pgo.committed_closures().iter().filter(|c| c.gnc_weight > 0.5).count(),
            "solves": pgo.solve_records().len(),
            "solve_s": pgo.solve_records().iter().map(|r| r.2).sum::<f64>(),
            "wall_s": started.elapsed().as_secs_f64(),
        })
    );
}
