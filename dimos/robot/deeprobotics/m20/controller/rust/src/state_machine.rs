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

//! rl_deploy's state machine (decoded from v1.5.8; SPEC.md): passive (idle) -> stand_up -> rl_control (runner by gait
//! code, after a stand-in policy on some gaits) -> lie_down, and joint_damping on request or on a fault (stale state,
//! tilt, joint beyond its range, a policy error). Mirrors dimos/simulation/m20_policy_sim/controller.py. One `tick`
//! per 1 ms control step.

use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::config::{tile, Config, Robot, StandIn, JOINTS};
use crate::policy::{PolicyRequest, PolicyService};
use crate::runner::Targets;
use crate::types::{roll_pitch, JointCommand, RobotState, Target, UserCommand};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Passive,
    StandUp,
    RlControl,
    LieDown,
    JointDamping,
}

/// Hermite cubic from (p0, v0) at 0 to (p1, v1) at T (rl_deploy's GetCubicSplinePos/Vel); holds p1 past T.
pub fn cubic(p0: f64, v0: f64, p1: f64, v1: f64, t: f64, total: f64) -> (f64, f64) {
    if t >= total {
        return (p1, v1);
    }
    let a2 = (3.0 * (p1 - p0) - (2.0 * v0 + v1) * total) / (total * total);
    let a3 = (2.0 * (p0 - p1) + (v0 + v1) * total) / (total * total * total);
    (
        p0 + v0 * t + a2 * t * t + a3 * t * t * t,
        v0 + 2.0 * a2 * t + 3.0 * a3 * t * t,
    )
}

/// The pose with each wheel axle `h` below its hip (two-link IK), hind legs mirrored, clamped to the joint range.
pub fn leg_pose_for_height(robot: &Robot, h: f64, hipx: f64) -> [f64; JOINTS] {
    let (l1, l2) = (robot.thigh_length, robot.shank_length);
    let h = h.abs().min(0.999 * (l1 + l2));
    let hipy = -((l1 * l1 + h * h - l2 * l2) / (2.0 * h * l1))
        .clamp(-1.0, 1.0)
        .acos();
    let knee = PI
        - ((l1 * l1 + l2 * l2 - h * h) / (2.0 * l1 * l2))
            .clamp(-1.0, 1.0)
            .acos();
    let pose = [
        hipx, hipy, knee, 0.0, -hipx, hipy, knee, 0.0, hipx, -hipy, -knee, 0.0, -hipx, -hipy,
        -knee, 0.0,
    ];
    let mut out = [0.0; JOINTS];
    for i in 0..JOINTS {
        out[i] = pose[i].clamp(robot.joint_lower[i], robot.joint_upper[i]);
    }
    out
}

fn arr16(v: Vec<f64>) -> [f64; JOINTS] {
    let mut a = [0.0; JOINTS];
    a.copy_from_slice(&v[..JOINTS]);
    a
}

fn gains(kp: &[f64], kd: &[f64]) -> ([f64; JOINTS], [f64; JOINTS]) {
    (
        arr16(tile(kp, JOINTS).expect("validated")),
        arr16(tile(kd, JOINTS).expect("validated")),
    )
}

#[derive(Debug, Default, Clone)]
pub struct Counters {
    pub ticks: u64,
    pub policy_steps: u64,
    pub policy_offers_refused: u64,
    pub stale_ticks: u64,
    pub last_inference_s: f64,
    pub max_inference_s: f64,
}

struct ActiveStandIn {
    spec: StandIn,
    duration: f64,
    elapsed: f64,
}

impl ActiveStandIn {
    fn params(&self) -> &BTreeMap<String, f64> {
        if self.elapsed < self.duration {
            &self.spec.params
        } else {
            &self.spec.settle_params
        }
    }
    fn done(&self) -> bool {
        self.elapsed >= self.duration + self.spec.settle_s
    }
}

pub struct Controller {
    cfg: Config,
    dt: f64,
    pub state: State,
    state_time: f64,
    time: f64,
    enter_q: [f64; JOINTS],
    enter_dq: [f64; JOINTS],
    pub fault: Option<String>,
    blocked_target: Option<Target>,
    last_stamp: Option<f64>,
    stale_run: u64,
    rl_entries: u64,
    stand_in: Option<ActiveStandIn>,
    gait: u32,
    entry_raw_gait: u32,
    runner: Option<String>,
    runner_gains: BTreeMap<String, ([f64; JOINTS], [f64; JOINTS])>,
    targets: Option<Targets>,
    gain_scale: f64,
    reset_pending: bool,
    since_offer: u64,
    seq: u64,
    pub transitions: Vec<(f64, State, State)>,
    pub counters: Counters,
}

impl Controller {
    pub fn new(cfg: Config) -> Controller {
        let runner_gains = cfg
            .runners
            .iter()
            .map(|(n, r)| (n.clone(), gains(&r.kp, &r.kd)))
            .collect();
        Controller {
            dt: cfg.control_dt(),
            cfg,
            state: State::Passive,
            state_time: 0.0,
            time: 0.0,
            enter_q: [0.0; JOINTS],
            enter_dq: [0.0; JOINTS],
            fault: None,
            blocked_target: None,
            last_stamp: None,
            stale_run: 0,
            rl_entries: 0,
            stand_in: None,
            gait: 0,
            entry_raw_gait: 0,
            runner: None,
            runner_gains,
            targets: None,
            gain_scale: 1.0,
            reset_pending: false,
            since_offer: 0,
            seq: 0,
            transitions: Vec::new(),
            counters: Counters::default(),
        }
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn current_runner(&self) -> Option<&str> {
        self.runner.as_deref()
    }

    pub fn last_targets(&self) -> Option<&Targets> {
        self.targets.as_ref()
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    /// The hardware missed a reply; a run of them longer than state_timeout_s is a fault.
    pub fn note_stale(&mut self) {
        self.counters.stale_ticks += 1;
        self.stale_run += 1;
    }

    pub fn note_fresh(&mut self) {
        self.stale_run = 0;
    }

    /// Recomputed every tick like rl_deploy's SafeController (a fault clears when its cause does).
    fn check_faults(&self, s: &RobotState) -> Option<String> {
        let safety = &self.cfg.safety;
        if self.time < safety.startup_grace_s {
            return None;
        }
        if self.stale_run as f64 * self.dt > safety.state_timeout_s {
            return Some(format!("no hardware state for {} ticks", self.stale_run));
        }
        if let Some(last) = self.last_stamp {
            if s.stamp - last > safety.state_timeout_s {
                return Some(format!("hardware state stale for {:.3} s", s.stamp - last));
            }
        }
        let r = &self.cfg.robot;
        for i in 0..JOINTS {
            if s.q[i] < r.joint_lower[i] - safety.joint_limit_margin_rad
                || s.q[i] > r.joint_upper[i] + safety.joint_limit_margin_rad
            {
                return Some(format!(
                    "joint {} at {:.3} rad, beyond its range",
                    r.joint_names[i], s.q[i]
                ));
            }
        }
        let (roll, pitch) = roll_pitch(&s.quat);
        let (roll, pitch) = (roll.abs().to_degrees(), pitch.abs().to_degrees());
        let st = &self.cfg.states;
        let tilted = match self.state {
            State::Passive => roll.max(pitch) > st.passive.max_tilt_deg,
            State::StandUp => roll.max(pitch) > st.stand_up.max_tilt_deg,
            State::RlControl => match self.runner.as_ref().and_then(|n| self.cfg.runners.get(n)) {
                Some(spec) if spec.tilt_check => {
                    roll > st.rl_control.max_roll_deg
                        || pitch > spec.max_pitch_deg.unwrap_or(st.rl_control.max_pitch_deg)
                }
                _ => false,
            },
            _ => false,
        };
        tilted.then(|| format!("tilt roll {roll:.1} pitch {pitch:.1} deg"))
    }

    /// After a fault the operator's standing request is ignored until they send a different one.
    fn effective_target(&mut self, cmd: &UserCommand) -> Target {
        if let Some(blocked) = self.blocked_target {
            if cmd.target == blocked {
                return Target::Passive;
            }
            self.blocked_target = None;
        }
        cmd.target
    }

    fn next_state(&self, target: Target) -> State {
        let st = &self.cfg.states;
        let t = self.state_time;
        if self.state != State::JointDamping
            && (target == Target::JointDamping || self.fault.is_some())
        {
            return State::JointDamping;
        }
        match self.state {
            State::Passive => {
                let ready = self.time >= st.passive.min_s_before_stand;
                if ready && matches!(target, Target::Stand | Target::RlControl) {
                    State::StandUp
                } else {
                    State::Passive
                }
            }
            State::StandUp => match target {
                Target::RlControl if t > 2.0 * st.stand_up.phase_s => State::RlControl,
                Target::LieDown | Target::Passive if t > 2.0 * st.stand_up.phase_s => {
                    State::LieDown
                }
                _ => State::StandUp,
            },
            State::RlControl => match target {
                Target::LieDown | Target::Passive if self.stand_in.is_none() => State::LieDown,
                _ => State::RlControl,
            },
            State::LieDown => match target {
                Target::Stand | Target::RlControl if t > 2.0 * st.lie_down.phase_s => {
                    State::StandUp
                }
                _ => State::LieDown,
            },
            State::JointDamping => {
                if self.fault.is_none()
                    && (t >= st.joint_damping.min_s || target != Target::JointDamping)
                {
                    State::Passive
                } else {
                    State::JointDamping
                }
            }
        }
    }

    fn enter(&mut self, next: State, s: &RobotState, cmd: &UserCommand) {
        self.transitions.push((self.time, self.state, next));
        if next == State::JointDamping && self.fault.is_some() {
            self.blocked_target = Some(cmd.target);
        }
        self.state = next;
        self.state_time = 0.0;
        self.enter_q = s.q;
        self.enter_dq = s.dq;
        if next == State::RlControl {
            let rl = &self.cfg.states.rl_control;
            self.entry_raw_gait = cmd.gait;
            self.gait = if rl.entry_gaits.contains(&cmd.gait) {
                cmd.gait
            } else {
                rl.entry_fallback_gait
            };
            self.runner = None;
            self.targets = None;
            self.start_stand_in(self.gait, true);
            self.rl_entries += 1;
        }
    }

    fn start_stand_in(&mut self, gait: u32, on_entry: bool) {
        self.stand_in = self
            .cfg
            .states
            .rl_control
            .stand_in
            .iter()
            .find(|si| si.gait == gait && (on_entry || si.on_switch))
            .map(|si| ActiveStandIn {
                spec: si.clone(),
                duration: if self.rl_entries == 0 {
                    si.first_s
                } else {
                    si.later_s
                },
                elapsed: 0.0,
            });
    }

    pub fn tick(
        &mut self,
        s: &RobotState,
        cmd: &UserCommand,
        policy: &mut dyn PolicyService,
    ) -> JointCommand {
        let fault = self.check_faults(s);
        if fault.is_some()
            || self
                .fault
                .as_deref()
                .is_some_and(|f| f.starts_with("policy"))
        {
            self.fault = fault.or(self.fault.take());
        } else {
            self.fault = None;
        }
        self.last_stamp = Some(s.stamp);
        let target = self.effective_target(cmd);
        let next = self.next_state(target);
        if next != self.state {
            self.enter(next, s, cmd);
        }
        let st = self.cfg.states.clone();
        let out = match self.state {
            State::Passive => self.constant(&st.passive.kp, &st.passive.kd),
            State::JointDamping => self.constant(&st.joint_damping.kp, &st.joint_damping.kd),
            State::StandUp => self.stand_up(),
            State::LieDown => self.lie_down(),
            State::RlControl => self.rl_control(s, cmd, policy),
        };
        self.state_time += self.dt;
        self.time += self.dt;
        self.counters.ticks += 1;
        out
    }

    fn constant(&self, kp: &[f64], kd: &[f64]) -> JointCommand {
        let (kp, kd) = gains(kp, kd);
        JointCommand {
            kp,
            kd,
            ..Default::default()
        }
    }

    fn stand_up(&self) -> JointCommand {
        let p = &self.cfg.states.stand_up;
        let (kp, kd) = gains(&p.kp, &p.kd);
        let mut c = JointCommand {
            kp,
            kd,
            ..Default::default()
        };
        let (t, total) = (self.state_time, p.phase_s);
        if t <= total {
            let goal = leg_pose_for_height(&self.cfg.robot, p.crouch_height, p.hipx);
            for (i, &target) in goal.iter().enumerate() {
                (c.q[i], c.dq[i]) = cubic(self.enter_q[i], self.enter_dq[i], target, 0.0, t, total);
            }
        } else {
            let (h, _) = cubic(p.crouch_height, 0.0, p.stand_height, 0.0, t - total, total);
            let (h_next, _) = cubic(
                p.crouch_height,
                0.0,
                p.stand_height,
                0.0,
                t - total + self.dt,
                total,
            );
            c.q = leg_pose_for_height(&self.cfg.robot, h, p.hipx);
            let next = leg_pose_for_height(&self.cfg.robot, h_next, p.hipx);
            for (dq, (&n, &q)) in c.dq.iter_mut().zip(next.iter().zip(&c.q)) {
                *dq = (n - q) / self.dt;
            }
        }
        for i in (3..JOINTS).step_by(4) {
            c.q[i] = 0.0;
            c.dq[i] = 0.0;
        }
        c
    }

    fn lie_down(&self) -> JointCommand {
        let p = &self.cfg.states.lie_down;
        let (kp, kd) = gains(&p.kp, &p.kd);
        let (t, total) = (self.state_time, p.phase_s);
        if t > 2.0 * total {
            return JointCommand::default();
        }
        if t > total {
            return JointCommand {
                kd,
                ..Default::default()
            };
        }
        let goal = leg_pose_for_height(&self.cfg.robot, p.lie_height, p.hipx);
        let mut c = JointCommand {
            kp,
            kd,
            ..Default::default()
        };
        for (i, &target) in goal.iter().enumerate().filter(|(i, _)| i % 4 != 3) {
            (c.q[i], c.dq[i]) = cubic(self.enter_q[i], self.enter_dq[i], target, 0.0, t, total);
        }
        c
    }

    fn rl_control(
        &mut self,
        s: &RobotState,
        cmd: &UserCommand,
        policy: &mut dyn PolicyService,
    ) -> JointCommand {
        let (wanted, params) = match &mut self.stand_in {
            Some(si) => {
                let out = (si.spec.runner.clone(), si.params().clone());
                si.elapsed += self.dt;
                if si.done() {
                    self.stand_in = None;
                }
                out
            }
            None => {
                // a gait change after entry (rl_deploy's gait-change marker) is followed as is
                let gait = if cmd.gait != self.entry_raw_gait {
                    cmd.gait
                } else {
                    self.gait
                };
                if gait != self.gait {
                    self.gait = gait;
                    self.start_stand_in(gait, false);
                }
                let name = cmd
                    .runner
                    .clone()
                    .unwrap_or_else(|| self.cfg.runner_for_gait(self.gait).to_string());
                (name, cmd.params.clone())
            }
        };
        if self.runner.as_deref() != Some(wanted.as_str()) {
            self.runner = Some(wanted.clone());
            self.reset_pending = true;
            self.since_offer = self.cfg.timing.policy_decimation;
            self.gain_scale = 1.0;
        }
        if let Some(result) = policy.take_result() {
            if result.runner == wanted {
                match result.targets {
                    Ok(t) => {
                        self.targets = Some(t);
                        self.gain_scale = result.gain_scale;
                        self.counters.policy_steps += 1;
                        self.counters.last_inference_s = result.inference_s;
                        self.counters.max_inference_s =
                            self.counters.max_inference_s.max(result.inference_s);
                    }
                    Err(e) => self.fault = Some(format!("policy {wanted}: {e}")),
                }
            }
        }
        // offer every policy_decimation ticks; a refused offer (busy worker) is retried next tick
        self.since_offer += 1;
        if self.since_offer >= self.cfg.timing.policy_decimation {
            self.seq += 1;
            let req = PolicyRequest {
                runner: wanted.clone(),
                reset: self.reset_pending,
                state: s.clone(),
                axes: cmd.axes,
                params,
                seq: self.seq,
            };
            if policy.offer(req) {
                self.since_offer = 0;
                self.reset_pending = false;
            } else {
                self.counters.policy_offers_refused += 1;
            }
        }
        let (mut kp, mut kd) = self.runner_gains[&wanted];
        // legs only: on the robot NaviCharge's x1.3 start-up gains left the wheels' kd at 0.6
        for i in (0..JOINTS).filter(|i| i % 4 != 3) {
            kp[i] *= self.gain_scale;
            kd[i] *= self.gain_scale;
        }
        let mut c = JointCommand {
            kp,
            kd,
            ..Default::default()
        };
        match &self.targets {
            // after a gait switch the previous runner's targets bridge until the new runner's first step lands
            Some(t) => {
                c.q = t.q;
                for (k, i) in (3..JOINTS).step_by(4).enumerate() {
                    c.dq[i] = t.wheel_velocity[k];
                }
            }
            // until the first policy step lands: hold the pose rl_control started from
            None => c.q = self.enter_q,
        }
        for i in (3..JOINTS).step_by(4) {
            c.kp[i] = 0.0;
            c.q[i] = 0.0;
        }
        c
    }
}
