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
//! joint beyond its range, a policy error). Mirrors dimos/simulation/m20_policy_sim/controller.py. One `tick`
//! per 1 ms control step.

use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::config::{tile, Config, Robot, StandIn, JOINTS};
use crate::policy::{PolicyRequest, PolicyService};
use crate::runner::Targets;
use crate::types::{JointCommand, RobotState, Target, UserCommand};

/// The config's name of a state (states.enable.on_enter).
pub fn state_name(state: State) -> &'static str {
    match state {
        State::Passive => "passive",
        State::StandUp => "stand_up",
        State::RlControl => "rl_control",
        State::LieDown => "lie_down",
        State::JointDamping => "joint_damping",
        State::StartDamping => "start_damping",
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Passive,
    StandUp,
    RlControl,
    LieDown,
    JointDamping,
    /// rl_deploy's state after a (re)start, until the operator stands it up
    StartDamping,
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
    // front-leg IK clamped to the front leg's ik range (rl_deploy ControlParameters), then mirrored
    let hipy = (-((l1 * l1 + h * h - l2 * l2) / (2.0 * h * l1))
        .clamp(-1.0, 1.0)
        .acos())
    .clamp(robot.ik_lower[1], robot.ik_upper[1]);
    let knee = (PI
        - ((l1 * l1 + l2 * l2 - h * h) / (2.0 * l1 * l2))
            .clamp(-1.0, 1.0)
            .acos())
    .clamp(robot.ik_lower[2], robot.ik_upper[2]);
    [
        hipx, hipy, knee, 0.0, -hipx, hipy, knee, 0.0, hipx, -hipy, -knee, 0.0, -hipx, -hipy,
        -knee, 0.0,
    ]
}

/// rl_deploy's cubic (GetCubicSplinePos, and StandUpState / LieDownState::Run's copies of it; aarch64 disassembly):
/// the coefficients' numerators in f32 with its fused multiply-adds, each coefficient rounded to f32, the polynomial
/// in f64, the position and velocity rounded to f32. Holds p1 (velocity 0) from `total` on.
pub fn vendor_cubic(p0: f32, v0: f32, p1: f32, t: f32, total: f32) -> (f32, f32) {
    if t >= total {
        return (p1, 0.0);
    }
    let a3n = p0.mul_add(2.0, total.mul_add(v0, -(p1 + p1)));
    let a2n = (-p0).mul_add(3.0, (-(v0 + v0)).mul_add(total, p1 * 3.0));
    let td = total as f64;
    // pow(x, 3.0) of an f32 x: x * x is exact, so this rounds once, as glibc's pow does
    let a2 = (a2n as f64 / (td * td)) as f32 as f64;
    let a3 = (a3n as f64 / (td * td * td)) as f32 as f64;
    let t2 = t as f64 * t as f64;
    let q = a3.mul_add(t2 * t as f64, a2 * t2);
    let q = (((v0 * t) as f64 + q) + p0 as f64) as f32;
    let dq = (a3 * 3.0).mul_add(t2, (a2 + a2) * t as f64);
    (q, (v0 as f64 + dq) as f32)
}

/// rl_deploy's leg IK for a hip-to-wheel height (StandUpState's, in f32 with its FMAs and f64 acos): hipy, knee,
/// each clamped to the front leg's range. `hh`: h x h as it has it (LieDownState folds 0.03 x 0.03 at compile time).
fn vendor_leg_ik(robot: &Robot, h: f32, hh: Option<f32>) -> (f32, f32) {
    let (l1, l2) = (robot.thigh_length as f32, robot.shank_length as f32);
    let clamp = |q: f32, k: usize| {
        let (lo, hi) = (robot.ik_lower[k] as f32, robot.ik_upper[k] as f32);
        if q > hi {
            hi
        } else if q < lo {
            lo
        } else {
            q
        }
    };
    if h.abs() >= l1 + l2 {
        // rl_deploy: "error height input", 0
        return (0.0, 0.0);
    }
    let (hipy_num, knee_num) = match hh {
        None => (
            (-l2).mul_add(l2, l1.mul_add(l1, h * h)),
            (-h).mul_add(h, l1.mul_add(l1, l2 * l2)),
        ),
        Some(hh) => (
            (-l2).mul_add(l2, l1.mul_add(l1, hh)),
            l1.mul_add(l1, l2 * l2) - hh,
        ),
    };
    let hipy = -((hipy_num as f64 / ((h as f64 + h as f64) * l1 as f64)).acos() as f32);
    let knee = (PI - ((knee_num / ((l1 + l1) * l2)) as f64).acos()) as f32;
    (clamp(hipy, 1), clamp(knee, 2))
}

/// The 16 joint targets of an IK leg pose (hipy, knee) with hipx, mirrored as rl_deploy mirrors them.
fn vendor_pose(hipx: f32, hipy: f32, knee: f32) -> [f32; JOINTS] {
    [
        hipx, hipy, knee, 0.0, -hipx, hipy, knee, 0.0, hipx, -hipy, -knee, 0.0, -hipx, -hipy,
        -knee, 0.0,
    ]
}

/// rl_deploy's stand-up / lie-down phase 1 (t <= total): each joint's cubic from the entry (q, dq) to `goal`; the
/// wheels' velocity 0.
fn vendor_phase_one(
    goal: &[f32; JOINTS],
    q0: &[f64; JOINTS],
    dq0: &[f64; JOINTS],
    t: f32,
    total: f32,
    c: &mut JointCommand,
) {
    for i in 0..JOINTS {
        let (q, dq) = vendor_cubic(q0[i] as f32, dq0[i] as f32, goal[i], t, total);
        c.q[i] = q as f64;
        c.dq[i] = if i % 4 == 3 { 0.0 } else { dq as f64 };
    }
}

/// Gains tiled to every joint (Config::validate checks they tile); a list that does not tile gives zero gains.
fn tiled(v: &[f64]) -> [f64; JOINTS] {
    tile(v, JOINTS)
        .ok()
        .and_then(|t| t.try_into().ok())
        .unwrap_or([0.0; JOINTS])
}

fn gains(kp: &[f64], kd: &[f64]) -> ([f64; JOINTS], [f64; JOINTS]) {
    (tiled(kp), tiled(kd))
}

/// The first non-finite number in a state the faults are judged on, by name.
fn non_finite_state(s: &RobotState) -> Option<&'static str> {
    let fields: [(&str, &[f64]); 5] = [
        ("joint position", &s.q),
        ("joint velocity", &s.dq),
        ("orientation", &s.quat),
        ("gyro", &s.gyro),
        ("stamp", std::slice::from_ref(&s.stamp)),
    ];
    fields
        .iter()
        .find(|(_, v)| v.iter().any(|x| !x.is_finite()))
        .map(|(name, _)| *name)
}

fn command_is_finite(c: &JointCommand) -> bool {
    [&c.kp, &c.q, &c.kd, &c.dq, &c.tau]
        .iter()
        .all(|a| a.iter().all(|x| x.is_finite()))
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
    /// the hardware stamp now and at the state's entry (f32_state_clock)
    stamp: f64,
    enter_stamp: f64,
    enter_q: [f64; JOINTS],
    enter_dq: [f64; JOINTS],
    pub fault: Option<String>,
    blocked_target: Option<Target>,
    last_stamp: Option<f64>,
    stale_run: u64,
    enable_left: u64,
    fault_checks: bool,
    stand_ins_run: std::collections::BTreeSet<u32>,
    rl_entries: u64,
    stand_in: Option<ActiveStandIn>,
    /// rl_deploy's accelerometer mean over the last 20 rl_control ticks (kept across rl_control entries)
    accel_average: crate::types::AccelAverage,
    gait: u32,
    entry_raw_gait: u32,
    runner: Option<String>,
    runner_gains: BTreeMap<String, ([f64; JOINTS], [f64; JOINTS])>,
    targets: Option<Targets>,
    /// the runner whose step the targets (and gains) are from
    targets_runner: Option<String>,
    gain_scale: f64,
    /// rl_control was just entered: the next step runs rl_deploy's entry OnEnter first
    entry_pending: bool,
    /// a runner switch the next step runs OnExit / OnEnter for first
    switch_pending: bool,
    activate_pending: bool,
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
            stamp: 0.0,
            enter_stamp: 0.0,
            enter_q: [0.0; JOINTS],
            enter_dq: [0.0; JOINTS],
            fault: None,
            blocked_target: None,
            last_stamp: None,
            stale_run: 0,
            enable_left: 0,
            fault_checks: true,
            stand_ins_run: Default::default(),
            rl_entries: 0,
            stand_in: None,
            gait: 0,
            entry_raw_gait: 0,
            accel_average: Default::default(),
            runner: None,
            runner_gains,
            targets: None,
            targets_runner: None,
            gain_scale: 1.0,
            entry_pending: false,
            switch_pending: false,
            activate_pending: false,
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

    /// The gait code walking is on (the last one commanded).
    pub fn gait(&self) -> u32 {
        self.gait
    }

    /// A stand-in policy is running (rl_control's entry or a gait switch's), before the gait's own runner.
    pub fn in_stand_in(&self) -> bool {
        self.stand_in.is_some()
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
        if !self.fault_checks {
            return None;
        }
        let safety = &self.cfg.safety;
        if self.time < safety.startup_grace_s {
            return None;
        }
        // NaN compares false against every limit below, so a non-finite reading is a fault of its own
        if let Some(name) = non_finite_state(s) {
            return Some(format!("non-finite {name} in the hardware state"));
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
        None
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
        let t = match self.state {
            State::StandUp | State::LieDown => self.spline_time(),
            _ => self.state_time,
        };
        if self.state != State::JointDamping
            && (target == Target::JointDamping || self.fault.is_some())
        {
            return State::JointDamping;
        }
        match self.state {
            State::Passive | State::StartDamping => {
                let ready = self.time >= st.passive.min_s_before_stand;
                if ready && matches!(target, Target::Stand | Target::RlControl) {
                    State::StandUp
                } else {
                    self.state
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
        self.enter_stamp = s.stamp;
        self.enter_q = s.q;
        self.enter_dq = s.dq;
        let enable = &self.cfg.states.enable;
        self.enable_left = if enable.on_enter.iter().any(|n| n == state_name(next)) {
            enable.ticks
        } else {
            0
        };
        if next == State::RlControl {
            let rl = &self.cfg.states.rl_control;
            self.entry_raw_gait = cmd.gait;
            self.gait = if rl.entry_gaits.contains(&cmd.gait) {
                cmd.gait
            } else {
                rl.entry_fallback_gait
            };
            self.runner = None;
            self.entry_pending = true;
            self.targets = None;
            self.targets_runner = None;
            self.start_stand_in(self.gait, true);
            self.rl_entries += 1;
        }
    }

    fn start_stand_in(&mut self, gait: u32, on_entry: bool) {
        let spec = self
            .cfg
            .states
            .rl_control
            .stand_in
            .iter()
            .find(|si| si.gait == gait && if on_entry { si.on_entry } else { si.on_switch })
            .cloned();
        // first_s the first time this stand-in runs, later_s after
        self.stand_in = spec.map(|si| {
            let first = self.stand_ins_run.insert(si.gait);
            ActiveStandIn {
                duration: if first { si.first_s } else { si.later_s },
                spec: si,
                elapsed: 0.0,
            }
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
        self.stamp = s.stamp;
        let target = self.effective_target(cmd);
        let next = self.next_state(target);
        if next != self.state {
            self.enter(next, s, cmd);
        }
        let st = self.cfg.states.clone();
        let out = match self.state {
            State::Passive => self.constant(&st.passive.kp, &st.passive.kd),
            State::StartDamping => self.constant(&st.start_damping.kp, &st.start_damping.kd),
            State::JointDamping => self.constant(&st.joint_damping.kp, &st.joint_damping.kd),
            State::StandUp => self.stand_up(),
            State::LieDown => self.lie_down(),
            State::RlControl => self.rl_control(s, cmd, policy),
        };
        // a NaN / inf target or gain (a policy output, a bad state through a spline) never reaches the motors
        let out = if self.fault_checks && !command_is_finite(&out) {
            self.fault = Some("policy: non-finite joint command".into());
            self.constant(&st.joint_damping.kp, &st.joint_damping.kd)
        } else {
            out
        };
        let out = if self.enable_left > 0 {
            self.enable_left -= 1;
            JointCommand {
                enable: true,
                ..Default::default()
            }
        } else {
            out
        };
        self.state_time += self.dt;
        self.time += self.dt;
        self.counters.ticks += 1;
        out
    }

    /// Jump to `state` now (replays of recorded sessions): no stand-in policy, optional enable frames.
    pub fn force(&mut self, state: State, s: &RobotState, cmd: &UserCommand, enable: bool) {
        self.enter(state, s, cmd);
        self.stand_in = None;
        if !enable {
            self.enable_left = 0;
        }
    }

    /// Hold `state` regardless of the transition rules (replays compare transitions separately).
    pub fn hold_state(&mut self, state: State) {
        self.state = state;
    }

    /// Turn fault detection off (replays of recorded sessions) or back on.
    pub fn set_fault_checks(&mut self, on: bool) {
        self.fault_checks = on;
    }

    /// Place the controller's clock (replays start past the start-up gates).
    pub fn set_time(&mut self, time: f64) {
        self.time = time;
    }

    /// Seconds in the state for the stand-up / lie-down splines and limits: rl_deploy's f32(stamp) - f32(entry
    /// stamp) with f32_state_clock, else control ticks.
    fn spline_time(&self) -> f64 {
        if self.cfg.timing.f32_state_clock {
            (self.stamp as f32 - self.enter_stamp as f32) as f64
        } else {
            self.state_time
        }
    }

    fn constant(&self, kp: &[f64], kd: &[f64]) -> JointCommand {
        let (kp, kd) = gains(kp, kd);
        JointCommand {
            kp,
            kd,
            ..Default::default()
        }
    }

    /// StandUpState::Run as rl_deploy computes it (f32_state_clock): its f32 clock, cubic, IK and 1 ms difference.
    fn vendor_stand_up(&self, mut c: JointCommand) -> JointCommand {
        let p = &self.cfg.states.stand_up;
        let robot = &self.cfg.robot;
        let (t, total) = (self.spline_time() as f32, p.phase_s as f32);
        let hipx = p.hipx as f32;
        if t <= total {
            let (hipy, knee) = vendor_leg_ik(robot, p.crouch_height as f32, None);
            let goal = vendor_pose(hipx, hipy, knee);
            vendor_phase_one(&goal, &self.enter_q, &self.enter_dq, t, total, &mut c);
            return c;
        }
        let (h0, h1) = (p.crouch_height as f32, p.stand_height as f32);
        let tt = t - total;
        let h = if tt < total {
            vendor_cubic(h0, 0.0, h1, tt, total).0
        } else {
            h1
        };
        // the height 1 ms on (0.001 as an f32)
        let step = 0.001f32;
        let h_next = if tt < total && tt + step < total {
            vendor_cubic(h0, 0.0, h1, tt + step, total).0
        } else {
            h1
        };
        let (hipy, knee) = vendor_leg_ik(robot, h, None);
        let (hipy_next, knee_next) = vendor_leg_ik(robot, h_next, None);
        let q = vendor_pose(hipx, hipy, knee);
        let (dhipy, dknee) = ((hipy_next - hipy) / step, (knee_next - knee) / step);
        for i in 0..JOINTS {
            c.q[i] = q[i] as f64;
            // hipx and the wheels: 0 (not mirrored); hind hipy / knee negated
            c.dq[i] = match (i % 4, i >= 8) {
                (1, false) => dhipy as f64,
                (2, false) => dknee as f64,
                (1, true) => -dhipy as f64,
                (2, true) => -dknee as f64,
                _ => 0.0,
            };
        }
        c
    }

    fn stand_up(&self) -> JointCommand {
        let p = &self.cfg.states.stand_up;
        let (kp, kd) = gains(&p.kp, &p.kd);
        let mut c = JointCommand {
            kp,
            kd,
            ..Default::default()
        };
        if self.cfg.timing.f32_state_clock {
            return self.vendor_stand_up(c);
        }
        let (t, total) = (self.spline_time(), p.phase_s);
        if t <= total {
            let goal = leg_pose_for_height(&self.cfg.robot, p.crouch_height, p.hipx);
            for (i, &target) in goal.iter().enumerate() {
                (c.q[i], c.dq[i]) = cubic(self.enter_q[i], self.enter_dq[i], target, 0.0, t, total);
            }
            // the wheels' position targets follow the spline to 0 too (kp 0); their velocity targets stay 0
            for i in (3..JOINTS).step_by(4) {
                c.dq[i] = 0.0;
            }
            return c;
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
        let (t, total) = (self.spline_time(), p.phase_s);
        if t > 2.0 * total {
            return JointCommand::default();
        }
        if t > total {
            return JointCommand {
                kd,
                ..Default::default()
            };
        }
        let mut c = JointCommand {
            kp,
            kd,
            ..Default::default()
        };
        if self.cfg.timing.f32_state_clock {
            // LieDownState::Run: its goal's h x h folded to an f32 at compile time
            let h = p.lie_height as f32;
            let (hipy, knee) = vendor_leg_ik(&self.cfg.robot, h, Some(h * h));
            let goal = vendor_pose(p.hipx as f32, hipy, knee);
            vendor_phase_one(
                &goal,
                &self.enter_q,
                &self.enter_dq,
                t as f32,
                total as f32,
                &mut c,
            );
            return c;
        }
        let goal = leg_pose_for_height(&self.cfg.robot, p.lie_height, p.hipx);
        for (i, &target) in goal.iter().enumerate() {
            (c.q[i], c.dq[i]) = cubic(self.enter_q[i], self.enter_dq[i], target, 0.0, t, total);
            if i % 4 == 3 {
                c.dq[i] = 0.0;
            }
        }
        c
    }

    /// The normalized axes: cmd.axes, or a metric cmd.velocity over the runner's real_steer_divisor.
    fn axes_for(&self, runner: &str, cmd: &UserCommand) -> [f64; 3] {
        let divisor = self
            .cfg
            .runners
            .get(runner)
            .and_then(|r| r.real_steer_divisor);
        match (cmd.velocity, divisor) {
            (Some(v), Some(d)) => {
                let mut a = [0.0; 3];
                for i in 0..3 {
                    a[i] = if d[i] > 0.0 { v[i] / d[i] } else { 0.0 };
                }
                a
            }
            _ => cmd.axes,
        }
    }

    fn rl_control(
        &mut self,
        s: &RobotState,
        cmd: &UserCommand,
        policy: &mut dyn PolicyService,
    ) -> JointCommand {
        let (wanted, params) = match &mut self.stand_in {
            Some(si) => {
                // a stand-in runs as rl_deploy's gait 0xf001
                let mut params = si.params().clone();
                params.insert("gait".into(), 0xf001 as f64);
                let out = (si.spec.runner.clone(), params);
                si.elapsed += self.dt;
                if si.done() {
                    self.stand_in = None;
                }
                out
            }
            None => {
                // a gait change after entry (rl_deploy's gait-change marker) is followed as is
                // (a change of the commanded gait since the last one: switching back to the entry gait counts)
                let gait = if cmd.gait != self.entry_raw_gait {
                    self.entry_raw_gait = cmd.gait;
                    cmd.gait
                } else {
                    self.gait
                };
                if gait != self.gait {
                    self.gait = gait;
                    self.start_stand_in(gait, false);
                }
                // a forced runner the config does not have is ignored (the gait's runner runs)
                let name = cmd
                    .runner
                    .clone()
                    .filter(|r| self.cfg.runners.contains_key(r))
                    .unwrap_or_else(|| self.cfg.runner_for_gait(self.gait).to_string());
                let mut params = cmd.params.clone();
                params.insert("gait".into(), self.gait as f64);
                (name, params)
            }
        };
        if self.runner.as_deref() != Some(wanted.as_str()) {
            // rl_deploy's policy thread keeps its 50 Hz cadence through a switch (robot, 2026-11-24); a fresh
            // rl_control steps at once
            if self.runner.is_none() {
                self.since_offer = self.cfg.timing.policy_decimation;
                self.gain_scale = 1.0;
            }
            // rl_deploy hands the last runner's action over to the new one (OnExit / OnEnter) and starts its history
            // and command slew over
            self.switch_pending = true;
            self.runner = Some(wanted.clone());
            self.activate_pending = true;
        }
        if let Some(result) = policy.take_result() {
            if result.runner == wanted {
                match result.targets {
                    Ok(t) => {
                        self.targets = Some(t);
                        self.targets_runner = Some(result.runner.clone());
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
        let accel = self.accel_average.push(s.accel);
        // offer every policy_decimation ticks; a refused offer (busy worker) is retried next tick
        self.since_offer += 1;
        if self.since_offer >= self.cfg.timing.policy_decimation {
            self.seq += 1;
            let req = PolicyRequest {
                runner: wanted.clone(),
                reset: false,
                entry: self.entry_pending,
                switch: self.switch_pending,
                activate: self.activate_pending,
                state: RobotState { accel, ..s.clone() },
                axes: self.axes_for(&wanted, cmd),
                params,
                seq: self.seq,
            };
            if policy.offer(req) {
                self.since_offer = 0;
                self.entry_pending = false;
                self.switch_pending = false;
                self.activate_pending = false;
            } else {
                self.counters.policy_offers_refused += 1;
            }
        }
        // the previous runner's gains (and scale) hold until the new runner's first step lands, as its targets do
        let gains_of = self.targets_runner.as_ref().unwrap_or(&wanted);
        let (mut kp, mut kd) = self
            .runner_gains
            .get(gains_of)
            .copied()
            .unwrap_or(([0.0; JOINTS], [0.0; JOINTS]));
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy::{InlinePolicy, PolicyResult, RunnerSet};

    fn config() -> Config {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        Config::load(&path).unwrap()
    }

    fn standing_state(cfg: &Config) -> RobotState {
        let mut s = RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        for i in 0..JOINTS {
            s.q[i] = (cfg.robot.joint_lower[i] + cfg.robot.joint_upper[i]) / 2.0;
        }
        s
    }

    /// Answers every offer with these targets.
    struct Fixed(Targets, Option<PolicyResult>);

    impl PolicyService for Fixed {
        fn offer(&mut self, request: PolicyRequest) -> bool {
            self.1 = Some(PolicyResult {
                runner: request.runner,
                seq: request.seq,
                targets: Ok(self.0.clone()),
                gain_scale: 1.0,
                inference_s: 0.0,
            });
            true
        }
        fn take_result(&mut self) -> Option<PolicyResult> {
            self.1.take()
        }
        fn dropped(&self) -> u64 {
            0
        }
    }

    #[test]
    fn unknown_forced_runner_does_not_panic() {
        let cfg = config();
        let s = standing_state(&cfg);
        let mut c = Controller::new(cfg);
        let cmd = UserCommand {
            target: Target::RlControl,
            runner: Some("no_such_runner".into()),
            ..Default::default()
        };
        c.set_time(10.0);
        c.force(State::RlControl, &s, &cmd, false);
        let mut policy = InlinePolicy::new(RunnerSet::empty());
        for _ in 0..50 {
            c.tick(&s, &cmd, &mut policy);
        }
        // the gait's runner ran (and, with no networks loaded, failed into a policy fault)
        assert_eq!(c.state, State::JointDamping);
        assert!(c.fault.as_deref().is_some_and(|f| f.starts_with("policy")));
    }

    #[test]
    fn non_finite_state_is_a_fault() {
        let cfg = config();
        let mut s = standing_state(&cfg);
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let mut policy = InlinePolicy::new(RunnerSet::empty());
        c.tick(&s, &UserCommand::default(), &mut policy);
        assert!(c.fault.is_none());
        s.q[5] = f64::NAN;
        c.tick(&s, &UserCommand::default(), &mut policy);
        assert!(c.fault.as_deref().is_some_and(|f| f.contains("non-finite")));
        c.tick(&s, &UserCommand::default(), &mut policy);
        assert_eq!(c.state, State::JointDamping);
    }

    #[test]
    fn non_finite_policy_target_never_goes_out() {
        let cfg = config();
        let s = standing_state(&cfg);
        let damping = gains(&cfg.states.joint_damping.kp, &cfg.states.joint_damping.kd);
        let mut c = Controller::new(cfg);
        let cmd = UserCommand {
            target: Target::RlControl,
            ..Default::default()
        };
        c.set_time(10.0);
        c.force(State::RlControl, &s, &cmd, false);
        let mut q = [0.0; JOINTS];
        q[1] = f64::NAN;
        let mut policy = Fixed(
            Targets {
                q,
                wheel_velocity: [0.0; 4],
                action: [0.0; JOINTS],
                velocity_estimate: None,
            },
            None,
        );
        let mut sent = Vec::new();
        for _ in 0..100 {
            sent.push(c.tick(&s, &cmd, &mut policy));
        }
        assert!(sent.iter().all(command_is_finite));
        assert!(sent.iter().any(|o| (o.kp, o.kd) == damping));
        assert_eq!(c.state, State::JointDamping);
    }
}
