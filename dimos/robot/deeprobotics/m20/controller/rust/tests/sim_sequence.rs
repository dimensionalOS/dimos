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

//! The controller against the MuJoCo sim (server.py) in real time, threaded policy and all: lie -> stand-up ->
//! rl_control -> walk -> stop -> lie-down, without falling. Needs the extracted policies and a Python with mujoco
//! and onnxruntime (M20_SIM_PYTHON, default ~/repos/dimos/.venv/bin/python); skips otherwise.

mod common;

use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::AtomicBool;

use m20_controller::commands::{ScriptStep, ScriptedCommands};
use m20_controller::hardware::mujoco::{MujocoSim, Truth};
use m20_controller::policy::{RunnerSet, ThreadedPolicy};
use m20_controller::rt::{self, Pacing};
use m20_controller::state_machine::{Controller, State};
use m20_controller::types::{Target, UserCommand};

struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../../../..")
        .canonicalize()
        .unwrap()
}

fn start_server(port: u16) -> Option<Server> {
    let python = std::env::var("M20_SIM_PYTHON")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(std::env::var("HOME").unwrap_or_default())
                .join("repos/dimos/.venv/bin/python")
        });
    if !python.exists() {
        eprintln!(
            "skipping: no sim python at {} (set M20_SIM_PYTHON)",
            python.display()
        );
        return None;
    }
    let mut child = Command::new(python)
        .args([
            "-m",
            "dimos.simulation.m20_policy_sim.server",
            "--port",
            &port.to_string(),
            "--once",
        ])
        .env("PYTHONPATH", repo_root())
        .stdout(Stdio::piped())
        .spawn()
        .expect("spawn sim server");
    let mut line = String::new();
    BufReader::new(child.stdout.take().unwrap())
        .read_line(&mut line)
        .unwrap();
    assert!(line.contains("listening"), "sim server said {line:?}");
    Some(Server(child))
}

fn step(at: f64, target: Target, gait: u32, axes: [f64; 3]) -> ScriptStep {
    ScriptStep {
        at,
        command: UserCommand {
            target,
            gait,
            axes,
            ..Default::default()
        },
    }
}

struct Sample {
    t: f64,
    state: State,
    runner: Option<String>,
    truth: Truth,
}

/// Run the script in real time; returns a 50 Hz trace of ground truth.
fn run_script(
    port: u16,
    script: Vec<ScriptStep>,
    seconds: f64,
) -> Option<(Vec<Sample>, Controller, rt::LoopStats)> {
    let dir = common::policy_dir()?;
    let _server = start_server(port)?;
    let cfg = common::config();
    let mut sim = MujocoSim::connect(&format!("127.0.0.1:{port}")).unwrap();
    sim.reset(&serde_json::json!({ "pose": "lie" })).unwrap();
    let mut policy = ThreadedPolicy::spawn(RunnerSet::load(&cfg, &dir).unwrap(), || {});
    let mut commands = ScriptedCommands::new(script);
    let mut controller = Controller::new(cfg.clone());
    let stop = AtomicBool::new(false);
    let mut trace = Vec::new();
    let stats = rt::run(
        &cfg,
        &mut sim,
        &mut controller,
        &mut policy,
        &mut commands,
        Pacing::Realtime,
        &stop,
        |hw, tick| {
            let c = tick.controller;
            if tick.stats.ticks % 20 == 0 {
                let truth = hw.truth().unwrap();
                trace.push(Sample {
                    t: c.time(),
                    state: c.state,
                    runner: c.current_runner().map(String::from),
                    truth,
                });
            }
            if c.time() >= seconds {
                stop.store(true, std::sync::atomic::Ordering::Relaxed);
            }
        },
    )
    .unwrap();
    Some((trace, controller, stats))
}

fn mean_vx(trace: &[Sample], t0: f64, t1: f64) -> f64 {
    let v: Vec<f64> = trace
        .iter()
        .filter(|s| s.t >= t0 && s.t < t1)
        .map(|s| s.truth.velocity[0])
        .collect();
    v.iter().sum::<f64>() / v.len() as f64
}

fn check(
    trace: &[Sample],
    controller: &Controller,
    stats: &rt::LoopStats,
    walk: (f64, f64),
    speed: f64,
    runner: &str,
) {
    let states: Vec<State> = controller.transitions.iter().map(|t| t.2).collect();
    println!("transitions {:?}", controller.transitions);
    println!("counters {:?} loop {:?}", controller.counters, stats);
    // the trailing joint damping is the shutdown sequence after the test stops the loop
    assert_eq!(
        states[..3],
        [State::StandUp, State::RlControl, State::LieDown],
        "transitions"
    );
    for s in trace.iter().filter(|s| s.state == State::RlControl) {
        assert!(
            !s.truth.trunk_contact && s.truth.tilt < 1.0,
            "fell at {:.2} s: {:?}",
            s.t,
            s.truth
        );
    }
    let walking: Vec<&Sample> = trace
        .iter()
        .filter(|s| s.t >= walk.0 && s.t < walk.1)
        .collect();
    assert!(
        walking.iter().all(|s| s.runner.as_deref() == Some(runner)),
        "walked on the wrong runner"
    );
    let vx = mean_vx(trace, (walk.0 + walk.1) / 2.0, walk.1);
    println!("{runner}: commanded {speed} m/s, walked {vx:.3} m/s");
    assert!((vx - speed).abs() < 0.15, "vx {vx:.3} vs {speed}");
    let end = trace.last().unwrap();
    assert!(
        end.truth.position[2] < 0.2,
        "not lying at the end: z {:.3}",
        end.truth.position[2]
    );
    // the Mac and the Python sim are not real-time: a tick that lands > 1 ms late counts as an overrun; the policy
    // must still keep its 50 Hz in controller time
    let overrun = stats.overruns as f64 / stats.ticks as f64;
    println!(
        "overruns {:.2}% (max tick {:.1} ms)",
        overrun * 100.0,
        stats.max_tick_s * 1e3
    );
    assert!(overrun < 0.2, "overruns {:.2}%", overrun * 100.0);
    let rl = &controller.transitions;
    let rl_s = rl[2].0 - rl[1].0;
    let expected = rl_s * controller.config().timing.control_hz
        / controller.config().timing.policy_decimation as f64;
    let steps = controller.counters.policy_steps as f64;
    println!("policy steps {steps} of {expected:.0} at 50 Hz");
    assert!(
        steps > 0.95 * expected,
        "policy fell behind: {steps} of {expected:.0}"
    );
}

/// NaviCharge entry (1.5 s finetune_stand_15kg stand-in), then NaviFlat walking at 0.5 m/s.
#[test]
fn stand_walk_lie_navi_flat() {
    let script = vec![
        step(0.0, Target::Passive, 0x3001, [0.0; 3]),
        step(2.0, Target::RlControl, 0x3001, [0.0; 3]),
        step(8.5, Target::RlControl, 0x3002, [0.0; 3]),
        step(10.0, Target::RlControl, 0x3002, [0.25, 0.0, 0.0]),
        step(15.0, Target::RlControl, 0x3002, [0.0; 3]),
        step(17.0, Target::LieDown, 0x3002, [0.0; 3]),
    ];
    let Some((trace, controller, stats)) = run_script(43931, script, 24.0) else {
        return;
    };
    check(&trace, &controller, &stats, (10.5, 15.0), 0.5, "navi_flat");
}

/// The default path: BasicFlat entry runs the stand-in for 3.5 s, then BasicFlat walks.
#[test]
fn stand_walk_lie_basic_flat() {
    let script = vec![
        step(0.0, Target::Passive, 0x1001, [0.0; 3]),
        step(2.0, Target::RlControl, 0x1001, [0.0; 3]),
        step(10.5, Target::RlControl, 0x1001, [0.25, 0.0, 0.0]),
        step(15.5, Target::RlControl, 0x1001, [0.0; 3]),
        step(17.5, Target::LieDown, 0x1001, [0.0; 3]),
    ];
    let Some((trace, controller, stats)) = run_script(43932, script, 24.5) else {
        return;
    };
    check(&trace, &controller, &stats, (11.0, 15.5), 0.5, "basic_flat");
}
