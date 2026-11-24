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

//! Open-loop replay of a recorded rl_deploy session (inputs prepared by dimos.simulation.m20_policy_sim.replay):
//! state segments through the Controller forced into the recorded state, policy steps through the runners at the
//! ticks rl_deploy's targets changed. Writes out.bin: per row 65 f32 (kp, q, kd, dq x 16, enable), NaN where idle.
//!
//!   cargo run --release --example replay -- DIR [--config m20_controller.toml] [--policies DIR]

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use serde::Deserialize;

use m20_controller::config::{Config, JOINTS};
use m20_controller::policy::{PolicyRequest, RunnerSet};
use m20_controller::state_machine::{Controller, State};
use m20_controller::types::{JointCommand, RobotState, Target, UserCommand};

const TICK: usize = 43;
const OUT: usize = 65;

#[derive(Deserialize)]
struct Segment {
    state: String,
    start: usize,
    end: usize,
}

#[derive(Deserialize)]
struct Step {
    row: usize,
    state_row: i64,
    runner: String,
    reset: bool,
    activate: bool,
    axes: [f64; 3],
    params: BTreeMap<String, f64>,
    #[serde(default)]
    last_action: Option<Vec<f64>>,
}

#[derive(Deserialize)]
struct Plan {
    segments: Vec<Segment>,
    steps: Vec<Step>,
}

struct Ticks(Vec<f64>);

impl Ticks {
    fn rows(&self) -> usize {
        self.0.len() / TICK
    }
    fn stamp(&self, row: usize) -> f64 {
        self.0[row * TICK]
    }
    fn state(&self, row: i64) -> RobotState {
        let v = &self.0[row.max(0) as usize * TICK..][..TICK];
        let mut s = RobotState {
            stamp: v[0],
            ..Default::default()
        };
        s.quat.copy_from_slice(&v[1..5]);
        s.gyro.copy_from_slice(&v[5..8]);
        s.accel.copy_from_slice(&v[8..11]);
        s.q.copy_from_slice(&v[11..27]);
        s.dq.copy_from_slice(&v[27..43]);
        s
    }
    fn missing_before(&self, row: usize) -> usize {
        (((self.stamp(row) - self.stamp(row - 1)) * 1000.0).round() as i64 - 1).max(0) as usize
    }
}

fn store(out: &mut [f32], row: usize, c: &JointCommand) {
    let o = &mut out[row * OUT..][..OUT];
    for i in 0..JOINTS {
        o[i] = c.kp[i] as f32;
        o[16 + i] = c.q[i] as f32;
        o[32 + i] = c.kd[i] as f32;
        o[48 + i] = c.dq[i] as f32;
    }
    o[64] = if c.enable { 1.0 } else { 0.0 };
}

fn state_of(name: &str) -> Result<(State, Target)> {
    Ok(match name {
        "passive" => (State::Passive, Target::Passive),
        "stand_up" => (State::StandUp, Target::Stand),
        "lie_down" => (State::LieDown, Target::LieDown),
        "joint_damping" => (State::JointDamping, Target::JointDamping),
        "rl_control" => (State::RlControl, Target::RlControl),
        "start_damping" => (State::StartDamping, Target::Passive),
        other => bail!("unknown state {other}"),
    })
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(
        args.next()
            .context("usage: replay DIR [--config F] [--policies D]")?,
    );
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut config = manifest.join("m20_controller.toml");
    let mut policies = PathBuf::from(std::env::var("M20_POLICY_DIR").unwrap_or_else(|_| {
        format!(
            "{}/.cache/dimos/m20_policies",
            std::env::var("HOME").unwrap_or_default()
        )
    }));
    while let Some(flag) = args.next() {
        let value = args
            .next()
            .with_context(|| format!("{flag} needs a value"))?;
        match flag.as_str() {
            "--config" => config = value.into(),
            "--policies" => policies = value.into(),
            other => bail!("unknown argument {other}"),
        }
    }
    let mut cfg = Config::load(&config)?;
    // the states on control ticks: ticks.bin stamps are wall-clock seconds, not rl_deploy's seconds since its start
    cfg.timing.f32_state_clock = false;
    let bytes = std::fs::read(dir.join("ticks.bin")).context("ticks.bin")?;
    let ticks = Ticks(
        bytes
            .as_chunks::<8>()
            .0
            .iter()
            .map(|c| f64::from_le_bytes(*c))
            .collect(),
    );
    let plan: Plan = serde_json::from_str(&std::fs::read_to_string(dir.join("plan.json"))?)?;
    let n = ticks.rows();
    let mut out = vec![f32::NAN; n * OUT];

    // state segments: the Controller forced into the recorded state, fed row k - 1's state for row k
    let mut controller = Controller::new(cfg.clone());
    // the recorded states are what rl_deploy did; its faults are not ours to raise here
    controller.set_fault_checks(false);
    let mut no_policy = m20_controller::policy::InlinePolicy::new(RunnerSet::empty());
    for seg in plan.segments.iter() {
        let (state, target) = state_of(&seg.state)?;
        let cmd = UserCommand {
            target,
            ..Default::default()
        };
        if state == State::RlControl {
            let enable = JointCommand {
                enable: true,
                ..Default::default()
            };
            for row in seg.start..(seg.start + cfg.states.enable.ticks as usize).min(seg.end) {
                store(&mut out, row, &enable);
            }
            continue;
        }
        controller.force(
            state,
            &ticks.state(seg.start as i64 - 1),
            &cmd,
            seg.start > 0,
        );
        controller.set_time(1e9);
        for row in seg.start..seg.end {
            if row > seg.start {
                for _ in 0..ticks.missing_before(row) {
                    controller.tick(&ticks.state(row as i64 - 1), &cmd, &mut no_policy);
                    controller.hold_state(state);
                }
            }
            let c = controller.tick(&ticks.state(row as i64 - 1), &cmd, &mut no_policy);
            controller.hold_state(state);
            store(&mut out, row, &c);
        }
    }

    // policy steps: the runners at the recorded update rows, gains and joint mapping as rl_control applies them
    let mut runners = RunnerSet::load(&cfg, &policies)?;
    for (k, step) in plan.steps.iter().enumerate() {
        let req = PolicyRequest {
            runner: step.runner.clone(),
            reset: step.reset,
            entry: false,
            switch: false,
            activate: step.activate,
            state: ticks.state(step.state_row),
            axes: step.axes,
            params: step.params.clone(),
            seq: k as u64,
        };
        if let Some(a) = &step.last_action {
            let p = &mut runners.runner_mut(&step.runner).context("runner")?.pipeline;
            if p.collected >= p.spec.warmup_steps {
                p.last_action.copy_from_slice(&a[..JOINTS]);
            }
        }
        let result = runners.serve(&req);
        let targets = result.targets.map_err(anyhow::Error::msg)?;
        let spec = &cfg.runners[&step.runner];
        let kp = m20_controller::config::tile(&spec.kp, JOINTS)?;
        let kd = m20_controller::config::tile(&spec.kd, JOINTS)?;
        let mut c = JointCommand::default();
        for i in 0..JOINTS {
            let scale = if i % 4 == 3 { 1.0 } else { result.gain_scale };
            c.kp[i] = if i % 4 == 3 { 0.0 } else { kp[i] * scale };
            c.kd[i] = kd[i] * scale;
            c.q[i] = if i % 4 == 3 { 0.0 } else { targets.q[i] };
        }
        for (w, i) in (3..JOINTS).step_by(4).enumerate() {
            c.dq[i] = targets.wheel_velocity[w];
        }
        let end = plan
            .steps
            .get(k + 1)
            .map_or(step.row + cfg.timing.policy_decimation as usize, |s| s.row);
        let seg_end = plan
            .segments
            .iter()
            .find(|g| g.start <= step.row && step.row < g.end)
            .map_or(n, |g| g.end);
        for row in step.row..end.min(seg_end) {
            store(&mut out, row, &c);
        }
    }
    let mut f = std::fs::File::create(dir.join("out.bin"))?;
    let raw: Vec<u8> = out.iter().flat_map(|v| v.to_le_bytes()).collect();
    f.write_all(&raw)?;
    eprintln!("replayed {n} rows, {} policy steps", plan.steps.len());
    Ok(())
}
