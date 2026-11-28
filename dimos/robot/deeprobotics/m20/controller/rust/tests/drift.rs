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

//! Zero-command drift (ROBUSTNESS.md "Drift") against the MuJoCo sim: standing at a zero command the robot stays put
//! with zero_command on and creeps with rl_deploy's behavior; a shove, a stop from walking and a start from rest; the
//! same on 10-15 deg slopes, across and facing up, in every rolling gait; extre standing on the bridge.
//! Needs the extracted policies and a sim Python (M20_SIM_PYTHON); skips otherwise.

mod common;

use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};

use m20_controller::config::Config;
use m20_controller::hardware::mujoco::{MujocoSim, Truth};
use m20_controller::hardware::Hardware;
use m20_controller::policy::{InlinePolicy, RunnerSet};
use m20_controller::state_machine::{Controller, State};
use m20_controller::types::{Target, UserCommand};

struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn start_sim() -> Option<(Server, MujocoSim)> {
    let python = std::env::var("M20_SIM_PYTHON")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(std::env::var("HOME").unwrap_or_default())
                .join("repos/dimos/.venv/bin/python")
        });
    if !python.exists() {
        eprintln!("skipping: no sim python at {}", python.display());
        return None;
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../../../..")
        .canonicalize()
        .unwrap();
    let mut child = Command::new(python)
        .args([
            "-m",
            "dimos.simulation.m20_policy_sim.server",
            "--port",
            "0",
            "--once",
        ])
        .env("PYTHONPATH", root)
        .stdout(Stdio::piped())
        .spawn()
        .expect("spawn sim server");
    let mut line = String::new();
    BufReader::new(child.stdout.take().unwrap())
        .read_line(&mut line)
        .unwrap();
    let port = line.trim().rsplit(':').next().unwrap().to_string();
    let sim = MujocoSim::connect(&format!("127.0.0.1:{port}")).unwrap();
    Some((Server(child), sim))
}

/// A run's events: the operator's normalized axes from a time, and shoves (vx m/s) at a time.
struct Plan {
    gait: u32,
    axes: Vec<(f64, [f64; 3])>,
    pushes: Vec<(f64, f64)>,
    seconds: f64,
    /// server.py's terrain (flat when empty)
    terrain: serde_json::Value,
}

/// Standing on navi_charge (forced rl_control), `plan.gait` from 1 s; the truth at 50 Hz (controller time, truth).
fn run(cfg: &Config, plan: &Plan) -> Option<Vec<(f64, Truth)>> {
    let dir = common::policy_dir()?;
    let (_server, mut sim) = start_sim()?;
    let pose = cfg.runners["navi_charge"].default_pose.clone();
    let mut state = sim
        .reset(&serde_json::json!({"pose": pose, "terrain": plan.terrain}))
        .unwrap();
    let mut policy = InlinePolicy::new(RunnerSet::load(cfg, &dir).unwrap());
    let mut c = Controller::new(cfg.clone());
    let mut cmd = UserCommand {
        target: Target::RlControl,
        gait: 0x3001,
        ..Default::default()
    };
    c.force(State::RlControl, &state, &cmd, false);
    let mut trace = Vec::new();
    let mut ticks = 0u64;
    while c.time() < plan.seconds {
        let t = c.time();
        if t >= 1.0 && cmd.gait != plan.gait {
            cmd.gait = plan.gait;
            cmd.gait_seq += 1;
        }
        if let Some((_, a)) = plan.axes.iter().rev().find(|(at, _)| *at <= t) {
            cmd.axes = *a;
        }
        // the shove on the tick nearest its time
        let half_tick = cfg.control_dt() / 2.0;
        if let Some((_, v)) = plan
            .pushes
            .iter()
            .find(|(at, _)| (*at - t).abs() < half_tick)
        {
            sim.push([*v, 0.0]).unwrap();
        }
        let out = c.tick(&state, &cmd, &mut policy);
        state = sim.exchange(&out).unwrap();
        ticks += 1;
        if ticks.is_multiple_of(cfg.timing.policy_decimation) {
            trace.push((t, sim.truth().unwrap()));
        }
    }
    assert_eq!(c.state, State::RlControl);
    // zero_command's stand_runners end standing on the bridge
    let bridge = cfg
        .states
        .rl_control
        .safe_switch
        .as_ref()
        .map(|ss| ss.bridge.runner.as_str());
    let runner = c.current_runner();
    assert!(
        runner == Some(cfg.runner_for_gait(plan.gait)) || runner == bridge,
        "ended on {runner:?}"
    );
    Some(trace)
}

/// The distance moved over [t0, t1] (m) and the mean |vx| (m/s) over [t1 - 10, t1].
fn drift(trace: &[(f64, Truth)], t0: f64, t1: f64) -> (f64, f64) {
    // the sample at t, or the last one (the run ends a tick short of its end)
    let at = |t: f64| {
        &trace
            .iter()
            .find(|(s, _)| *s >= t)
            .unwrap_or(trace.last().unwrap())
            .1
    };
    let (a, b) = (at(t0).position, at(t1).position);
    let dx = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
    let v: Vec<f64> = trace
        .iter()
        .filter(|(s, _)| *s >= t1 - 10.0 && *s < t1)
        .map(|(_, tr)| tr.velocity[0].abs())
        .collect();
    (dx, v.iter().sum::<f64>() / v.len() as f64)
}

fn hold(gait: u32) -> Plan {
    Plan {
        gait,
        axes: vec![],
        pushes: vec![],
        seconds: 20.0,
        terrain: serde_json::json!({}),
    }
}

/// 15 s of zero command on the gaits that drifted most: under 2 cm, under 5 mm/s.
#[test]
fn a_zero_command_holds_still() {
    let cfg = common::config();
    for gait in [0x3003, 0x3005, 0x1004] {
        let Some(trace) = run(&cfg, &hold(gait)) else {
            return;
        };
        let (dx, vx) = drift(&trace, 5.0, 20.0);
        println!("{gait:#x}: moved {dx:.4} m, |vx| {vx:.4} m/s");
        assert!(dx < 0.02, "{gait:#x} moved {dx:.3} m");
        assert!(vx < 0.005, "{gait:#x} |vx| {vx:.4}");
    }
}

/// rl_deploy's behavior: the same hold creeps (the bias this cancels).
#[test]
fn without_it_the_robot_creeps() {
    let cfg = common::config().rl_deploy();
    let Some(trace) = run(&cfg, &hold(0x3005)) else {
        return;
    };
    let (dx, _) = drift(&trace, 5.0, 20.0);
    println!("rl_deploy 0x3005: moved {dx:.3} m");
    assert!(dx > 0.3, "moved {dx:.3} m");
}

/// A 0.2 m/s shove while holding: it rolls a few cm and stops.
#[test]
fn a_shove_rolls_a_little_and_stops() {
    let cfg = common::config();
    let mut plan = hold(0x3003);
    plan.pushes = vec![(8.0, 0.2)];
    let Some(trace) = run(&cfg, &plan) else {
        return;
    };
    let (dx, vx) = drift(&trace, 7.9, 20.0);
    println!("shoved: moved {dx:.3} m, |vx| {vx:.4} m/s");
    assert!(dx < 0.1, "moved {dx:.3} m");
    let (late, _) = drift(&trace, 12.0, 20.0);
    assert!(late < 0.02, "still rolling: {late:.3} m");
}

/// Walking at 0.5 m/s, then stopped: put once it has stopped.
#[test]
fn a_stop_from_walking_holds() {
    let cfg = common::config();
    let mut plan = hold(0x3003);
    plan.axes = vec![(3.0, [0.25, 0.0, 0.0]), (8.0, [0.0; 3])];
    plan.seconds = 22.0;
    let Some(trace) = run(&cfg, &plan) else {
        return;
    };
    let (dx, vx) = drift(&trace, 12.0, 22.0);
    println!("after the stop: moved {dx:.4} m, |vx| {vx:.4} m/s");
    assert!(dx < 0.02 && vx < 0.005);
}

/// From rest a command starts the robot as rl_deploy's behavior does: no lurch back, no delay.
#[test]
fn a_command_from_rest_starts_at_once() {
    let mut plan = hold(0x3003);
    plan.axes = vec![(10.0, [0.25, 0.0, 0.0])];
    plan.seconds = 12.0;
    let ours = common::config();
    let (Some(a), Some(b)) = (run(&ours, &plan), run(&ours.clone().rl_deploy(), &plan)) else {
        return;
    };
    let vx = |tr: &[(f64, Truth)], t: f64| tr.iter().find(|(s, _)| *s >= t).unwrap().1.velocity[0];
    let min_after = a
        .iter()
        .filter(|(s, _)| *s >= 10.0)
        .map(|(_, tr)| tr.velocity[0])
        .fold(f64::INFINITY, f64::min);
    for t in [10.3, 10.6, 11.0, 11.9] {
        println!(
            "t {t}: ours {:+.3} rl_deploy {:+.3} m/s",
            vx(&a, t),
            vx(&b, t)
        );
    }
    assert!(min_after > -0.02, "lurched back {min_after:.3} m/s");
    for t in [10.6, 11.0, 11.9] {
        assert!(
            (vx(&a, t) - vx(&b, t)).abs() < 0.08,
            "at {t}: {:.3} vs {:.3}",
            vx(&a, t),
            vx(&b, t)
        );
    }
}

/// A zero-command hold on a slope (noslip contact: MuJoCo's soft contact creeps a held wheel cm on its own).
fn on_slope(gait: u32, deg: f64, yaw: f64) -> Plan {
    let mut plan = hold(gait);
    plan.terrain = serde_json::json!({
        "kind": "slope", "slope_deg": deg, "slope_yaw_deg": yaw, "noslip_iterations": 10
    });
    plan
}

/// The gaits that roll on their wheels (navi_charge's stepping phase and extre: ROBUSTNESS.md "Drift").
const ROLLING: [u32; 7] = [0x1001, 0x1002, 0x3003, 0x1004, 0x3002, 0x3005, 0x5001];

/// 15 s of zero command across a 10 / 15 deg slope (either side) and facing up 15 deg: under 2 cm in every gait.
#[test]
fn a_zero_command_holds_on_a_slope() {
    let cfg = common::config();
    let mut worst = Vec::new();
    for (deg, yaw) in [(10.0, 90.0), (15.0, 90.0), (15.0, -90.0), (15.0, 0.0)] {
        for gait in ROLLING {
            let Some(trace) = run(&cfg, &on_slope(gait, deg, yaw)) else {
                return;
            };
            let (dx, _) = drift(&trace, 5.0, 20.0);
            println!("slope {deg} yaw {yaw} {gait:#x}: moved {dx:.4} m");
            if dx >= 0.02 {
                worst.push(format!("slope {deg} yaw {yaw} {gait:#x}: {dx:.3} m"));
            }
        }
    }
    assert!(worst.is_empty(), "slid: {worst:?}");
}

/// Without the hold the same side-slope stand rolls (what the hold stops).
#[test]
fn without_the_hold_a_side_slope_rolls() {
    let mut cfg = common::config();
    cfg.states.rl_control.zero_command.hold_kp = 0.0;
    let Some(trace) = run(&cfg, &on_slope(0x3003, 15.0, 90.0)) else {
        return;
    };
    let (dx, _) = drift(&trace, 5.0, 20.0);
    println!("no hold, 0x3003 across 15 deg: moved {dx:.3} m");
    assert!(dx > 0.05, "moved {dx:.3} m");
}

/// From rest on a side slope a command starts the robot at once: the hold lets go on the command's first tick.
#[test]
fn a_command_on_a_slope_starts_at_once() {
    let mut plan = on_slope(0x3003, 15.0, 90.0);
    plan.axes = vec![(10.0, [0.25, 0.0, 0.0])];
    plan.seconds = 12.0;
    let ours = common::config();
    let mut no_hold = ours.clone();
    no_hold.states.rl_control.zero_command.hold_kp = 0.0;
    let (Some(a), Some(b)) = (run(&ours, &plan), run(&no_hold, &plan)) else {
        return;
    };
    let vx = |tr: &[(f64, Truth)], t: f64| tr.iter().find(|(s, _)| *s >= t).unwrap().1.velocity[0];
    for t in [10.6, 11.0, 11.9] {
        println!(
            "t {t}: hold {:+.3} no hold {:+.3} m/s",
            vx(&a, t),
            vx(&b, t)
        );
        assert!(
            (vx(&a, t) - vx(&b, t)).abs() < 0.08,
            "at {t}: {:.3} vs {:.3}",
            vx(&a, t),
            vx(&b, t)
        );
    }
}

/// Extre (each gait): 15 s of zero command on flat moves under 2 cm (it stands on the bridge once still).
#[test]
fn extre_holds_still() {
    let cfg = common::config();
    let mut worst = Vec::new();
    for gait in [0x2001, 0x2002, 0x2003, 0x2004] {
        let Some(trace) = run(&cfg, &hold(gait)) else {
            return;
        };
        let (dx, _) = drift(&trace, 5.0, 20.0);
        println!("{gait:#x}: moved {dx:.4} m");
        if dx >= 0.02 {
            worst.push(format!("{gait:#x}: {dx:.3} m"));
        }
    }
    assert!(worst.is_empty(), "rolled: {worst:?}");
}

/// rl_deploy's behavior: extre's zero command rolls (what the stand stops).
#[test]
fn without_it_extre_rolls() {
    let cfg = common::config().rl_deploy();
    let Some(trace) = run(&cfg, &hold(0x2001)) else {
        return;
    };
    let (dx, _) = drift(&trace, 5.0, 20.0);
    println!("rl_deploy 0x2001: moved {dx:.3} m");
    assert!(dx > 0.2, "moved {dx:.3} m");
}

/// Extre across and up a 15 deg slope, and stopped from 0.75 m/s: still within 2 cm once standing.
#[test]
fn extre_holds_on_a_slope_and_after_a_stop() {
    let cfg = common::config();
    let mut worst = Vec::new();
    for yaw in [90.0, -90.0, 0.0] {
        let Some(trace) = run(&cfg, &on_slope(0x2001, 15.0, yaw)) else {
            return;
        };
        let (dx, _) = drift(&trace, 5.0, 20.0);
        println!("extre slope 15 yaw {yaw}: moved {dx:.4} m");
        if dx >= 0.02 {
            worst.push(format!("slope yaw {yaw}: {dx:.3} m"));
        }
    }
    let mut plan = hold(0x2001);
    plan.axes = vec![(3.0, [0.25, 0.0, 0.0]), (8.0, [0.0; 3])];
    plan.seconds = 25.0;
    let Some(trace) = run(&cfg, &plan) else {
        return;
    };
    // the command hands the stand back to extre: walking within 1 s (its 0.74 s warm-up), no lurch back
    let vx = |t: f64| trace.iter().find(|(s, _)| *s >= t).unwrap().1.velocity[0];
    let back = trace
        .iter()
        .filter(|(s, _)| (3.0..8.0).contains(s))
        .map(|(_, tr)| tr.velocity[0])
        .fold(f64::INFINITY, f64::min);
    println!(
        "extre from the stand: {:+.3} m/s at 4 s, min {back:+.3}",
        vx(4.0)
    );
    if vx(4.0) < 0.3 || back < -0.1 {
        worst.push(format!("start: {:.3} m/s at 4 s, min {back:.3}", vx(4.0)));
    }
    let (stop, _) = drift(&trace, 8.0, 10.0);
    let (dx, _) = drift(&trace, 10.0, 25.0);
    println!("extre stopped: {stop:.3} m to a stop, then {dx:.4} m");
    if stop >= 0.3 || dx >= 0.02 {
        worst.push(format!("stop: {stop:.3} m, then {dx:.3} m"));
    }
    assert!(worst.is_empty(), "rolled: {worst:?}");
}
