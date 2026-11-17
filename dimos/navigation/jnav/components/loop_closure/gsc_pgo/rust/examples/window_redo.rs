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

use dimos_gsc_pgo::fast_path::{solve_window, WindowGraph};
use dimos_gsc_pgo::gsc_pgo::Config;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let config: Config = serde_json::from_str(&std::fs::read_to_string(&args[1]).expect("config")).expect("config json");
    let (uniform, value) = (args[2].starts_with('='), args[2][1..].parse::<f64>().expect("pull"));
    for path in &args[3..] {
        let dump: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).expect("dump")).expect("dump json");
        let mut graph: WindowGraph = serde_json::from_value(dump["graph"].clone()).expect("graph");
        for pull in &mut graph.pull {
            *pull = pull.map(|v| if uniform { value } else { v * value });
        }
        let started = std::time::Instant::now();
        let fast = solve_window(&graph, &config);
        let count = graph.pull.len();
        let line = serde_json::json!({
            "dump": path, "scan": dump["scan"], "time": dump["time"], "keyframes": &graph.keyframes[..count], "positions": &fast.positions[..count], "rotations": &fast.rotations[..count],
            "worst_over_cutoff": fast.over_cutoff.iter().copied().fold(0.0, f64::max), "converged": fast.converged,
            "seconds": started.elapsed().as_secs_f64(),
        });
        println!("{line}");
    }
}
