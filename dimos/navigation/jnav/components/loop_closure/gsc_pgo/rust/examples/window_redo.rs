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

//! Solves `GSC_WINDOW_DUMP` window jobs again (diagnostics): per dump, a JSON line with its window keyframes, their solved
//! poses and the worst closure residual over GNC's cutoff.
//! usage: window_redo <config json> <pull: x<scale> scales each, =<variance> sets all> <dump.json>...

use dimos_gsc_pgo::big_picture_fit::edge_residual;
use dimos_gsc_pgo::fast_path::{solve_window, window_problem, WindowGraph};
use dimos_gsc_pgo::gsc_pgo::Config;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let config: Config = serde_json::from_str(&std::fs::read_to_string(&args[1]).expect("config")).expect("config json");
    let (uniform, value) = (args[2].starts_with('='), args[2][1..].parse::<f64>().expect("pull"));
    for path in &args[3..] {
        let dump: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).expect("dump")).expect("dump json");
        let mut graph: WindowGraph = serde_json::from_value(dump["graph"].clone()).expect("graph");
        // diagnostics: WINDOW_ODOMETRY_SCALE="<tilt>,<yaw>,<translation>" scales every odometry step's variances
        if let Ok(scale) = std::env::var("WINDOW_ODOMETRY_SCALE") {
            let s: Vec<f64> = scale.split(',').map(|x| x.parse().expect("scale")).collect();
            for e in &mut graph.odometry {
                for k in 0..6 {
                    e.variances[k] *= if k < 2 { s[0] } else if k == 2 { s[1] } else { s[2] };
                }
            }
        }
        for pull in &mut graph.pull {
            *pull = pull.map(|v| if uniform { value } else { v * value });
        }
        let started = std::time::Instant::now();
        let fast = solve_window(&graph, &config);
        let count = graph.pull.len();
        // the solution's whitened cost by kind: odometry rotation / translation, the taper's pulls, each closure
        let (_, _, _, edges) = window_problem(&graph, &config);
        let (rotations, positions) = (&fast.rotations, &fast.positions);
        let whitened = |e: &dimos_gsc_pgo::big_picture_fit::Edge| {
            let r = edge_residual(e, (&rotations[e.from], &positions[e.from]), (&rotations[e.to], &positions[e.to]));
            let w: Vec<f64> = (0..6).map(|k| r[k] * r[k] / e.variances[k]).collect();
            (w[0] + w[1] + w[2], w[3] + w[4] + w[5])
        };
        let steps = graph.odometry.len();
        let (mut odometry_rotation, mut odometry_translation, mut pull) = (0.0, 0.0, 0.0);
        let mut worst_steps: Vec<(f64, usize, usize)> = Vec::new();
        for (i, e) in edges.iter().enumerate() {
            let (rot, tr) = whitened(e);
            if i < steps {
                odometry_rotation += rot;
                odometry_translation += tr;
                worst_steps.push((rot + tr, graph.keyframes[e.from], graph.keyframes[e.to]));
            } else {
                pull += rot + tr;
            }
        }
        worst_steps.sort_by(|a, b| b.0.total_cmp(&a.0));
        worst_steps.truncate(5);
        let closures: Vec<(f64, f64)> = graph.closures.iter().map(|e| whitened(e)).collect();
        let residuals: Vec<[f64; 6]> = graph
            .closures
            .iter()
            .map(|e| edge_residual(e, (&rotations[e.from], &positions[e.from]), (&rotations[e.to], &positions[e.to])))
            .collect();
        let line = serde_json::json!({
            "cost": {"odometry_rotation": odometry_rotation, "odometry_translation": odometry_translation, "pull": pull,
                     "closures": closures, "residuals": residuals, "closure_variance": graph.closures.iter().map(|e| e.variances[3]).collect::<Vec<_>>(), "worst_steps": worst_steps},
            "dump": path, "scan": dump["scan"], "time": dump["time"], "keyframes": &graph.keyframes[..count], "positions": &fast.positions[..count], "rotations": &fast.rotations[..count],
            "worst_over_cutoff": fast.over_cutoff.iter().copied().fold(0.0, f64::max), "converged": fast.converged,
            "seconds": started.elapsed().as_secs_f64(),
        });
        println!("{line}");
    }
}
