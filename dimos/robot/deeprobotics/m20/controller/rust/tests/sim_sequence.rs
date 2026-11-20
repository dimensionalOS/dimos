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

//! The controller against the MuJoCo sim (server.py), threaded policy and all: lie -> stand-up -> rl_control -> walk
//! -> stop -> lie-down, without falling. The sim steps once per exchange and the script runs on controller time
//! (ticks), so the walk tests are paced by step count with the policy worker held to one tick of latency: the same
//! run on an idle or a loaded machine. The real-time variant (wall-clock pacing, the worker free-running, overruns
//! counted) is `--ignored`, for an idle machine. Needs the extracted policies and a Python with mujoco and
//! onnxruntime (M20_SIM_PYTHON, default ~/repos/dimos/.venv/bin/python); skips otherwise.

mod common;

use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::AtomicBool;

use m20_controller::commands::{ScriptStep, ScriptedCommands};
use m20_controller::hardware::mujoco::{MujocoSim, Truth};
use m20_controller::policy::{
    PolicyRequest, PolicyResult, PolicyService, RunnerSet, ThreadedPolicy,
};
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

/// Starts the sim on a free port; returns it and the port it took.
fn start_server() -> Option<(Server, u16)> {
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
            "0",
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
    let port = line.trim().rsplit(':').next().and_then(|p| p.parse().ok());
    let port = port.unwrap_or_else(|| panic!("no port in {line:?}"));
    Some((Server(child), port))
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

/// The threaded policy with its worker's answer always collected on the next tick: the robot's worker answers within
/// a tick or two, and here a loaded machine cannot stretch that, so no offer is refused and none is late.
struct Lockstep {
    inner: ThreadedPolicy,
    pending: bool,
}

impl PolicyService for Lockstep {
    /// Never refused: the last answer was collected, so the worker is idle and only its lock can be briefly held
    /// (between storing a result and waiting for the next request), which the real loop would retry a tick later.
    fn offer(&mut self, request: PolicyRequest) -> bool {
        while !self.inner.offer(request.clone()) {
            std::thread::yield_now();
        }
        self.pending = true;
        true
    }
    fn take_result(&mut self) -> Option<PolicyResult> {
        if !std::mem::take(&mut self.pending) {
            return None;
        }
        // a hang guard only (a dead worker), far above any inference: it never decides a result
        let give_up = std::time::Instant::now() + std::time::Duration::from_secs(60);
        loop {
            if let Some(result) = self.inner.take_result() {
                return Some(result);
            }
            assert!(
                std::time::Instant::now() < give_up,
                "policy worker never answered"
            );
            std::thread::yield_now();
        }
    }
    /// The worker's lock misses the offer retried past; none reaches the controller.
    fn dropped(&self) -> u64 {
        0
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    /// step-count pacing, the worker held to one tick (deterministic)
    Lockstep,
    /// wall-clock pacing, the worker free-running (as on the robot)
    Realtime,
}

/// One finished run: the trace, the controller after it, the loop's stats and the offers the policy dropped.
struct Run {
    trace: Vec<Sample>,
    controller: Controller,
    stats: rt::LoopStats,
    dropped: u64,
}

struct Sample {
    t: f64,
    state: State,
    runner: Option<String>,
    truth: Truth,
}

/// Run the script; returns a 50 Hz (controller time) trace of ground truth.
fn run_script(mode: Mode, script: Vec<ScriptStep>, seconds: f64) -> Option<Run> {
    let dir = common::policy_dir()?;
    let (_server, port) = start_server()?;
    let cfg = common::config();
    let mut sim = MujocoSim::connect(&format!("127.0.0.1:{port}")).unwrap();
    sim.reset(&serde_json::json!({ "pose": "lie" })).unwrap();
    let threaded = ThreadedPolicy::spawn(RunnerSet::load(&cfg, &dir).unwrap(), || {}).unwrap();
    let (mut lockstep, mut free);
    let (policy, pacing): (&mut dyn PolicyService, Pacing) = match mode {
        Mode::Lockstep => {
            lockstep = Lockstep {
                inner: threaded,
                pending: false,
            };
            (&mut lockstep, Pacing::Free)
        }
        Mode::Realtime => {
            free = threaded;
            (&mut free, Pacing::Realtime)
        }
    };
    let mut commands = ScriptedCommands::new(script);
    let mut controller = Controller::new(cfg.clone());
    let stop = AtomicBool::new(false);
    let mut trace = Vec::new();
    let stats = rt::run(
        &cfg,
        &mut sim,
        &mut controller,
        policy,
        &mut commands,
        pacing,
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
    let dropped = policy.dropped();
    Some(Run {
        trace,
        controller,
        stats,
        dropped,
    })
}

fn mean_vx(trace: &[Sample], t0: f64, t1: f64) -> f64 {
    let v: Vec<f64> = trace
        .iter()
        .filter(|s| s.t >= t0 && s.t < t1)
        .map(|s| s.truth.velocity[0])
        .collect();
    v.iter().sum::<f64>() / v.len() as f64
}

fn check(mode: Mode, run: &Run, walk: (f64, f64), speed: f64, runner: &str) {
    let Run {
        trace,
        controller,
        stats,
        dropped,
    } = run;
    let dropped = *dropped;
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
    let rl = &controller.transitions;
    let rl_s = rl[2].0 - rl[1].0;
    let expected = rl_s * controller.config().timing.control_hz
        / controller.config().timing.policy_decimation as f64;
    let steps = controller.counters.policy_steps as f64;
    println!("policy steps {steps} of {expected:.0} at 50 Hz, {dropped} offers dropped");
    match mode {
        // every offer taken and answered: a policy step every policy_decimation ticks, to the tick
        Mode::Lockstep => {
            assert_eq!(
                controller.counters.policy_offers_refused, 0,
                "offers refused"
            );
            assert!(
                (steps - expected).abs() <= 1.0,
                "policy steps {steps} of {expected:.0}"
            );
        }
        // the Mac and the Python sim are not real-time: a tick that lands > 1 ms late counts as an overrun; the
        // policy must still keep its 50 Hz in controller time
        Mode::Realtime => {
            let overrun = stats.overruns as f64 / stats.ticks as f64;
            println!(
                "overruns {:.2}% (max tick {:.1} ms)",
                overrun * 100.0,
                stats.max_tick_s * 1e3
            );
            assert!(overrun < 0.2, "overruns {:.2}%", overrun * 100.0);
            assert!(
                steps > 0.95 * expected,
                "policy fell behind: {steps} of {expected:.0}"
            );
        }
    }
}

fn navi_flat(mode: Mode) {
    let script = vec![
        step(0.0, Target::Passive, 0x3001, [0.0; 3]),
        step(2.0, Target::RlControl, 0x3001, [0.0; 3]),
        step(8.5, Target::RlControl, 0x3002, [0.0; 3]),
        step(10.0, Target::RlControl, 0x3002, [0.25, 0.0, 0.0]),
        step(15.0, Target::RlControl, 0x3002, [0.0; 3]),
        step(17.0, Target::LieDown, 0x3002, [0.0; 3]),
    ];
    let Some(run) = run_script(mode, script, 24.0) else {
        return;
    };
    check(mode, &run, (10.5, 15.0), 0.5, "navi_flat");
}

fn basic_flat(mode: Mode) {
    let script = vec![
        step(0.0, Target::Passive, 0x1001, [0.0; 3]),
        step(2.0, Target::RlControl, 0x1001, [0.0; 3]),
        step(12.5, Target::RlControl, 0x1001, [0.25, 0.0, 0.0]),
        step(17.5, Target::RlControl, 0x1001, [0.0; 3]),
        step(19.5, Target::LieDown, 0x1001, [0.0; 3]),
    ];
    let Some(run) = run_script(mode, script, 26.5) else {
        return;
    };
    check(mode, &run, (13.0, 17.5), 0.5, "basic_flat");
}

/// NaviCharge entry (1.5 s finetune_stand_15kg stand-in), then NaviFlat walking at 0.5 m/s.
#[test]
fn stand_walk_lie_navi_flat() {
    navi_flat(Mode::Lockstep);
}

/// The default path: BasicFlat entry runs the finetune_stand_15kg stand-in (6.14 s, as on the robot), then walks.
#[test]
fn stand_walk_lie_basic_flat() {
    basic_flat(Mode::Lockstep);
}

/// Both walks in wall-clock time with the free-running worker; needs an idle machine.
#[test]
#[ignore = "real time: needs an idle machine (cargo test --test sim_sequence -- --ignored)"]
fn realtime_stand_walk_lie() {
    navi_flat(Mode::Realtime);
    basic_flat(Mode::Realtime);
}
