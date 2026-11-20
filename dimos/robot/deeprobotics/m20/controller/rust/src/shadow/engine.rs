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

//! The shadow's decision loop, streaming: cycles (rl_deploy's sent command + the joint state before it) are held
//! for lookahead_s, then our command for each is computed exactly as the replay does: non-policy states through the
//! Controller forced into rl_deploy's state at its enable frames, policy steps through the runners at the cycles
//! rl_deploy's targets changed (and every 20 cycles between), from the same state, IMU and command.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use anyhow::Result;

use super::config::ShadowConfig;
use super::journal::Event;
use super::log::{
    Logger, Record, FLAG_IMU, FLAG_OURS, FLAG_OURS_ENABLE, FLAG_POLICY_STEP, FLAG_RL_ENABLE,
};
use super::sniff::{Imu, Steer};
use crate::config::{tile, Config, JOINTS};
use crate::frames::SentCommand;
use crate::policy::{InlinePolicy, PolicyRequest, RunnerSet};
use crate::state_machine::{Controller, State};
use crate::types::{JointCommand, RobotState, Target, UserCommand};

pub const STATES: [&str; 5] = [
    "passive",
    "stand_up",
    "rl_control",
    "lie_down",
    "joint_damping",
];
/// how much input history is kept (s)
const KEEP_S: f64 = 30.0;

pub struct Cycle {
    pub t_ns: u64,
    /// the joint state rl_deploy had when it sent this (the last reply before it), and when it arrived
    pub rx: Option<(f64, RobotState)>,
    pub rl: SentCommand,
}

fn state_of(name: &str) -> Option<(State, Target)> {
    Some(match name {
        "passive" => (State::Passive, Target::Passive),
        "stand_up" => (State::StandUp, Target::Stand),
        "lie_down" => (State::LieDown, Target::LieDown),
        "joint_damping" => (State::JointDamping, Target::JointDamping),
        "rl_control" => (State::RlControl, Target::RlControl),
        _ => return None,
    })
}

fn last_before<T: Clone>(v: &VecDeque<(f64, T)>, t: f64) -> Option<(f64, T)> {
    v.iter().rev().find(|(et, _)| *et <= t).cloned()
}

#[derive(Default)]
pub struct Counters {
    pub cycles: u64,
    pub ours: u64,
    pub policy_steps: u64,
    pub no_imu: u64,
    pub dropped: u64,
}

pub struct Engine {
    cfg: Config,
    scfg: ShadowConfig,
    controller: Controller,
    no_policy: InlinePolicy,
    runners: RunnerSet,
    runner_names: Vec<String>,
    imu: VecDeque<Imu>,
    steer: VecDeque<Steer>,
    states: VecDeque<(f64, String)>,
    gaits: VecDeque<(f64, u32)>,
    targets: VecDeque<(f64, u32)>,
    heights: VecDeque<(f64, f64)>,
    pending: VecDeque<Cycle>,
    last_rx: Option<(f64, RobotState)>,
    last_tx_t: f64,
    // processing state
    prev_t: Option<f64>,
    prev_rl_enable: bool,
    prev_rl_legs: Option<[f64; 12]>,
    state: Option<String>,
    seg_cycle: u64,
    since_step: u64,
    held: Option<JointCommand>,
    held_axes: [f64; 3],
    runner: Option<String>,
    used: BTreeSet<String>,
    prev_gait: u32,
    stand_in_start: f64,
    pose: BTreeMap<u32, BTreeMap<String, f64>>,
    logger: Logger,
    pub counters: Counters,
}

impl Engine {
    pub fn new(cfg: Config, scfg: ShadowConfig, runners: RunnerSet, logger: Logger) -> Engine {
        let mut controller = Controller::new(cfg.clone());
        controller.set_fault_checks(false);
        let runner_names = cfg.runners.keys().cloned().collect();
        Engine {
            controller,
            cfg,
            scfg,
            no_policy: InlinePolicy::new(RunnerSet::empty()),
            runners,
            runner_names,
            imu: VecDeque::new(),
            steer: VecDeque::new(),
            states: VecDeque::new(),
            gaits: VecDeque::new(),
            targets: VecDeque::new(),
            heights: VecDeque::new(),
            pending: VecDeque::new(),
            last_rx: None,
            last_tx_t: 0.0,
            prev_t: None,
            prev_rl_enable: false,
            prev_rl_legs: None,
            state: None,
            seg_cycle: 0,
            since_step: 0,
            held: None,
            held_axes: [0.0; 3],
            runner: None,
            used: BTreeSet::new(),
            prev_gait: 0,
            stand_in_start: 0.0,
            pose: BTreeMap::new(),
            logger,
            counters: Counters::default(),
        }
    }

    pub fn runner_names(cfg: &Config) -> Vec<String> {
        cfg.runners.keys().cloned().collect()
    }

    pub fn imu(&mut self, s: Imu) {
        self.imu.push_back(s);
        while self.imu.front().is_some_and(|f| s.t - f.t > KEEP_S) {
            self.imu.pop_front();
        }
    }

    pub fn steer(&mut self, s: Steer) {
        self.steer.push_back(s);
        while self.steer.front().is_some_and(|f| s.t - f.t > KEEP_S) {
            self.steer.pop_front();
        }
    }

    pub fn event(&mut self, t: f64, e: Event) {
        match e {
            Event::State(s) => self.states.push_back((t, s)),
            Event::Gait(g) => self.gaits.push_back((t, g)),
            Event::Target(g) => self.targets.push_back((t, g)),
            Event::Height(h) => self.heights.push_back((t, h)),
        }
    }

    /// A joint-state reply arrived (state in joint space); it is stamped with the command it answered, the clock
    /// the replay reads the IMU on.
    pub fn rx(&mut self, s: RobotState) {
        self.last_rx = Some((self.last_tx_t, s));
    }

    /// rl_deploy sent a joint command.
    pub fn tx(&mut self, t_ns: u64, rl: SentCommand) {
        if self.pending.len() >= self.scfg.max_pending_cycles {
            self.pending.pop_front();
            self.counters.dropped += 1;
        }
        self.pending.push_back(Cycle {
            t_ns,
            rx: self.last_rx.clone(),
            rl,
        });
        self.last_tx_t = t_ns as f64 * 1e-9;
    }

    /// Process every cycle older than lookahead_s before `now` (all of them when flushing).
    pub fn drain(&mut self, now: f64, flush: bool) -> Result<()> {
        while let Some(c) = self.pending.front() {
            let t = c.t_ns as f64 * 1e-9;
            if !flush && now - t < self.scfg.lookahead_s {
                break;
            }
            self.process()?;
        }
        Ok(())
    }

    fn with_imu(&self, rx: &(f64, RobotState)) -> Option<RobotState> {
        let (t, s) = rx;
        let imu = self.imu.iter().rev().find(|i| i.t <= *t)?;
        let mut s = s.clone();
        let [r, p, y] = imu.rpy_deg.map(|d| (d as f32).to_radians() as f64);
        let (cr, sr, cp, sp, cy, sy) = (
            (r / 2.0).cos(),
            (r / 2.0).sin(),
            (p / 2.0).cos(),
            (p / 2.0).sin(),
            (y / 2.0).cos(),
            (y / 2.0).sin(),
        );
        s.quat = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ];
        s.gyro = imu.gyro;
        s.accel = imu.accel;
        s.stamp = *t;
        Some(s)
    }

    /// Whether rl_deploy's leg targets change within the next n pending cycles (after the front).
    fn change_ahead(&self, base: &[f64; 12], n: usize) -> bool {
        self.pending
            .iter()
            .skip(1)
            .take(n)
            .any(|c| legs(&c.rl.command) != *base)
    }

    fn process(&mut self) -> Result<()> {
        let c = self.pending.pop_front().unwrap();
        let t = c.t_ns as f64 * 1e-9;
        let rl_enable = c.rl.word == self.scfg.enable_word;
        let rl_legs = legs(&c.rl.command);
        let mut missing = self
            .prev_t
            .map_or(0, |p| (((t - p) * 1000.0).round() as i64 - 1).max(0) as u64);
        self.counters.cycles += 1;
        let mut flags = if rl_enable { FLAG_RL_ENABLE } else { 0 };
        // a new state segment starts at rl_deploy's first enable frame
        if rl_enable && !self.prev_rl_enable {
            self.state = last_before(&self.states, t + 0.01).map(|(_, s)| s);
            self.seg_cycle = 0;
            missing = 0;
            if let (Some(name), Some(rx)) = (self.state.clone(), c.rx.as_ref()) {
                if let Some((st, target)) = state_of(&name) {
                    if st != State::RlControl {
                        let cmd = UserCommand {
                            target,
                            ..Default::default()
                        };
                        self.controller.force(st, &rx.1, &cmd, true);
                        self.controller.set_time(1e9);
                    }
                }
            }
        } else if self.state.is_none() && self.prev_t.is_none() {
            // started mid-state: take the journal's last word for it; with none, all-zero gains are idle (passive)
            let idle =
                c.rl.command
                    .kp
                    .iter()
                    .chain(&c.rl.command.kd)
                    .all(|&g| g == 0.0);
            self.state = last_before(&self.states, t)
                .map(|(_, s)| s)
                .or_else(|| idle.then(|| "passive".to_string()));
            if let (Some(name), Some(rx)) = (self.state.clone(), c.rx.as_ref()) {
                if let Some((st, target)) = state_of(&name) {
                    if st != State::RlControl {
                        let cmd = UserCommand {
                            target,
                            ..Default::default()
                        };
                        self.controller.force(st, &rx.1, &cmd, false);
                        self.controller.set_time(1e9);
                    }
                }
            }
        }
        let mut ours: Option<JointCommand> = None;
        let mut state_code = 255u8;
        let mut runner_code = 255u8;
        let mut gait = 0u32;
        let input = c.rx.as_ref().and_then(|rx| self.with_imu(rx));
        if input.is_some() {
            flags |= FLAG_IMU;
        } else {
            self.counters.no_imu += 1;
        }
        if let Some(name) = self.state.clone() {
            state_code = STATES
                .iter()
                .position(|s| *s == name)
                .map_or(255, |i| i as u8);
            match state_of(&name) {
                Some((State::RlControl, _)) => {
                    if self.seg_cycle < self.cfg.states.enable.ticks {
                        ours = Some(JointCommand {
                            enable: true,
                            ..Default::default()
                        });
                    } else {
                        let first =
                            self.held.is_none() || self.seg_cycle == self.cfg.states.enable.ticks;
                        let changed = self.prev_rl_legs.is_some_and(|p| p != rl_legs);
                        let period = self.cfg.timing.policy_decimation;
                        let cadence = self.since_step + 1 >= period
                            && !changed
                            && !self.change_ahead(&rl_legs, (period / 2) as usize);
                        if first || changed || cadence {
                            if let Some(s) = input.as_ref() {
                                if let Some(cmd) = self.policy_step(t, s)? {
                                    self.held = Some(cmd);
                                    flags |= FLAG_POLICY_STEP;
                                    self.counters.policy_steps += 1;
                                }
                            }
                            self.since_step = 0;
                        } else {
                            self.since_step += 1;
                        }
                        ours = self.held;
                    }
                    gait = self.prev_gait;
                    if let Some(r) = &self.runner {
                        runner_code = self
                            .runner_names
                            .iter()
                            .position(|n| n == r)
                            .map_or(255, |i| i as u8);
                    }
                }
                Some((st, target)) => {
                    if let Some(rx) = c.rx.as_ref() {
                        let cmd = UserCommand {
                            target,
                            ..Default::default()
                        };
                        for _ in 0..missing.min(50) {
                            self.controller.tick(&rx.1, &cmd, &mut self.no_policy);
                            self.controller.hold_state(st);
                        }
                        let out = self.controller.tick(&rx.1, &cmd, &mut self.no_policy);
                        self.controller.hold_state(st);
                        ours = Some(out);
                    }
                }
                None => {}
            }
        }
        if let Some(o) = &ours {
            flags |= FLAG_OURS;
            if o.enable {
                flags |= FLAG_OURS_ENABLE;
            }
            self.counters.ours += 1;
        }
        let blank = RobotState::default();
        let rec = Record {
            t_ns: c.t_ns,
            flags,
            rl_word: c.rl.word,
            state: state_code,
            runner: runner_code,
            gait,
            input: input
                .as_ref()
                .or(c.rx.as_ref().map(|r| &r.1))
                .unwrap_or(&blank),
            axes: self.held_axes,
            ours: ours.as_ref(),
            rl: &c.rl.command,
        };
        self.logger.write(&rec)?;
        self.prev_t = Some(t);
        self.prev_rl_enable = rl_enable;
        self.prev_rl_legs = Some(rl_legs);
        self.seg_cycle += 1;
        Ok(())
    }

    /// One policy step at time t from state s, exactly as replay.build_plan / run_python do; None before the
    /// journal has named a gait.
    fn policy_step(&mut self, t: f64, s: &RobotState) -> Result<Option<JointCommand>> {
        let Some((_, gait)) = last_before(&self.gaits, t) else {
            return Ok(None);
        };
        let runner = self.cfg.runner_for_gait(gait).to_string();
        let mut params: BTreeMap<String, f64> = BTreeMap::new();
        if gait == self.scfg.stand_in_gait {
            if self.prev_gait != gait {
                self.stand_in_start = t;
            }
            // the stand-in hands over to the gait the operator asked for
            if let Some((_, target)) = last_before(&self.targets, t) {
                if let Some(si) = self
                    .cfg
                    .states
                    .rl_control
                    .stand_in
                    .iter()
                    .find(|si| si.gait == target)
                {
                    let p = if t - self.stand_in_start < si.first_s {
                        &si.params
                    } else {
                        &si.settle_params
                    };
                    params.extend(p.iter().map(|(k, v)| (k.clone(), *v)));
                }
            }
        }
        self.prev_gait = gait;
        let steer = self.steer.iter().rev().find(|st| st.t <= t).copied();
        let fresh = steer.filter(|st| t - st.t <= self.scfg.command_timeout_s);
        let spec = &self.cfg.runners[&runner];
        let mut axes = [0.0; 3];
        if let Some(st) = fresh {
            let v = [st.axes[0], st.axes[1], st.axes[2]];
            axes = match (st.metric, spec.real_steer_divisor) {
                (true, Some(d)) => [0, 1, 2].map(|i| if d[i] > 0.0 { v[i] / d[i] } else { 0.0 }),
                (true, None) => v,
                (false, _) => v,
            };
        }
        let pose_inputs: Vec<_> = self
            .scfg
            .pose_inputs
            .iter()
            .filter(|p| p.gait == gait)
            .cloned()
            .collect();
        if !pose_inputs.is_empty() {
            if let Some(st) = fresh.filter(|st| !st.metric) {
                let names = ["x", "y", "yaw", "z", "roll", "pitch"];
                let entry = self.pose.entry(gait).or_default();
                for p in &pose_inputs {
                    if let Some(i) = names.iter().position(|n| *n == p.axis) {
                        entry.insert(p.param.clone(), p.scale * st.axes[i]);
                    }
                }
            }
            if let Some(p) = self.pose.get(&gait) {
                params.extend(p.iter().map(|(k, v)| (k.clone(), *v)));
            }
            axes = [0.0; 3];
        }
        if gait == self.scfg.finetune_height_gait {
            params.insert(
                "height".into(),
                last_before(&self.heights, t).map_or(0.0, |(_, h)| h),
            );
        }
        let reset = !self.used.contains(&runner);
        let activate = self.runner.as_deref() != Some(runner.as_str());
        self.used.insert(runner.clone());
        self.runner = Some(runner.clone());
        self.held_axes = axes;
        let req = PolicyRequest {
            runner: runner.clone(),
            reset,
            activate,
            state: s.clone(),
            axes,
            params,
            seq: 0,
        };
        let result = self.runners.serve(&req);
        let targets = result.targets.map_err(anyhow::Error::msg)?;
        let kp = tile(&spec.kp, JOINTS)?;
        let kd = tile(&spec.kd, JOINTS)?;
        let mut c = JointCommand::default();
        for i in 0..JOINTS {
            let wheel = i % 4 == 3;
            let scale = if wheel { 1.0 } else { result.gain_scale };
            c.kp[i] = if wheel { 0.0 } else { kp[i] * scale };
            c.kd[i] = kd[i] * scale;
            c.q[i] = if wheel { 0.0 } else { targets.q[i] };
        }
        for (w, i) in (3..JOINTS).step_by(4).enumerate() {
            c.dq[i] = targets.wheel_velocity[w];
        }
        Ok(Some(c))
    }

    pub fn flush(&mut self) -> Result<()> {
        self.logger.flush()
    }

    pub fn pending(&self) -> usize {
        self.pending.len()
    }
}

fn legs(c: &JointCommand) -> [f64; 12] {
    let mut out = [0.0; 12];
    for (k, i) in (0..JOINTS).filter(|i| i % 4 != 3).enumerate() {
        out[k] = c.q[i];
    }
    out
}
