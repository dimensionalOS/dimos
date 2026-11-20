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

//! Every runner's frame and targets match the Python reference (onnxruntime) step for step.

mod common;

use m20_controller::runner::Runner;
use m20_controller::types::RobotState;
use serde_json::Value;

fn arr<const N: usize>(v: &Value) -> [f64; N] {
    let mut a = [0.0; N];
    for (i, x) in v.as_array().unwrap().iter().enumerate() {
        a[i] = x.as_f64().unwrap();
    }
    a
}

fn vecf(v: &Value) -> Vec<f64> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}

#[test]
fn runners_match_python_reference() {
    let Some(dir) = common::policy_dir() else {
        return;
    };
    let golden = dir.join("golden.json");
    if !golden.exists() {
        eprintln!("skipping: run python -m dimos.simulation.m20_policy_sim.golden first");
        return;
    }
    let cfg = common::config();
    let golden: Value = serde_json::from_str(&std::fs::read_to_string(golden).unwrap()).unwrap();
    let dt = cfg.timing.policy_decimation as f64 / cfg.timing.control_hz;
    for (name, spec) in &cfg.runners {
        let mut runner = Runner::new(name, spec, &cfg.robot, &dir, dt).unwrap();
        let (mut worst_frame, mut worst_q, mut worst_w) = (0.0f64, 0.0f64, 0.0f64);
        for step in golden[name].as_array().unwrap() {
            let s = RobotState {
                quat: arr(&step["quat"]),
                gyro: arr(&step["gyro"]),
                accel: arr(&step["accel"]),
                q: arr(&step["q"]),
                dq: arr(&step["dq"]),
                height_map: step["height_map"]
                    .as_array()
                    .map(|_| vecf(&step["height_map"])),
                ..Default::default()
            };
            let axes: [f64; 3] = arr(&step["axes"]);
            let t = runner.step(&s, &axes, &Default::default()).unwrap();
            for (a, b) in runner.last_frame.iter().zip(vecf(&step["frame"])) {
                worst_frame = worst_frame.max((*a as f64 - b).abs());
            }
            for (a, b) in t.q.iter().zip(vecf(&step["target_q"])) {
                worst_q = worst_q.max((a - b).abs());
            }
            for (a, b) in t.wheel_velocity.iter().zip(vecf(&step["wheel_velocity"])) {
                worst_w = worst_w.max((a - b).abs());
            }
        }
        println!("{name}: frame {worst_frame:.2e} q {worst_q:.2e} wheel {worst_w:.2e}");
        assert!(worst_frame < 1e-4, "{name} frame differs by {worst_frame}");
        assert!(worst_q < 1e-3, "{name} leg targets differ by {worst_q}");
        assert!(worst_w < 1e-2, "{name} wheel targets differ by {worst_w}");
    }
}
