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

//! Replays a pgo-eval run's keyframes + committed closures (no search), solving every N keyframes; writes a.kf, prints kept/noise.
//! usage: relearn <run dir> <out a.kf> <config overlay json> [every keyframes, 20] [start keyframe, 0]

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3::{self, Mat3};

/// A committed closure from a.json: source, target, rotation, translation, score.
type Closure = (usize, usize, Mat3, [f64; 3], f64);

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let run = std::path::Path::new(&args[1]);
    let every: usize = args.get(4).map_or(20, |s| s.parse().unwrap());
    let start: usize = args.get(5).map_or(0, |s| s.parse().unwrap());
    let json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(run.join("a.json")).unwrap()).unwrap();
    let mut config_json = json["config"]["pgo"].clone();
    if config_json.is_null() {
        config_json = json["config"].clone();
    }
    let overlay: serde_json::Value = serde_json::from_str(&args[3]).unwrap();
    for (k, v) in overlay.as_object().unwrap() {
        config_json[k] = v.clone();
    }
    let config: Config = serde_json::from_value(config_json).unwrap();
    let bytes = std::fs::read(run.join("a.kf")).unwrap();
    let values: Vec<f64> = bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|c| f64::from_le_bytes(*c))
        .collect();
    let records: Vec<&[f64]> = values.as_chunks::<15>().0.iter().map(|r| r.as_slice()).collect();
    let mut closures: Vec<Closure> = json["committed_closure_edges"]
        .as_array()
        .unwrap()
        .iter()
        .map(|e| {
            (
                e["source_id"].as_u64().unwrap() as usize,
                e["target_id"].as_u64().unwrap() as usize,
                serde_json::from_value(e["rotation"].clone()).unwrap(),
                serde_json::from_value(e["translation"].clone()).unwrap(),
                e["score"].as_f64().unwrap(),
            )
        })
        .collect();
    closures.sort_by_key(|c| c.0);
    let mut pgo = GscPgo::new(config);
    let mut next = 0;
    for (k, r) in records.iter().enumerate() {
        let mut pose = PoseWithTime::new(
            mat3::mat_from_quat(&[r[7], r[4], r[5], r[6]]),
            [r[1], r[2], r[3]],
        );
        pose.second = r[0];
        assert!(
            pgo.add_key_pose(&CloudWithPose { cloud: None, pose }),
            "keyframe {k} was not taken"
        );
        while next < closures.len() && closures[next].0 <= k {
            let (s, t, rotation, translation, score) = closures[next];
            pgo.commit_closure(s, t, rotation, translation, score);
            next += 1;
        }
        if k + 1 >= start && ((k + 1) % every == 0 || k + 1 == start) {
            pgo.wait_for_solve();
        }
    }
    pgo.finish();
    let kept = pgo
        .committed_closures()
        .iter()
        .filter(|c| c.gnc_weight > 0.5)
        .count();
    println!(
        "kept {kept} / {} noise {:?}",
        pgo.committed_closures().len(),
        pgo.odometry_error_estimate()
            .map(|v| (v * 1000.0).round() / 1000.0)
    );
    let mut out = Vec::new();
    for (k, record) in records.iter().enumerate() {
        let key = &pgo.key_poses().get(k);
        let q = mat3::quat_from_mat(&key.rotation_global);
        let corrected = [
            key.translation_global[0],
            key.translation_global[1],
            key.translation_global[2],
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
