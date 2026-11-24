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

//! One rl_deploy policy runner: observation assembly, ONNX inference (tract), action -> joint targets.

use std::collections::{BTreeMap, VecDeque};
use std::f64::consts::PI;
use std::path::Path;

use anyhow::{anyhow, bail, Context, Result};
use tract_onnx::prelude::*;

use crate::config::{
    scale_or_one, tile, ActionLayout, CommandEntry, CommandSource, GaitClockSpec, JointSet,
    LegMode, ObsBlock, Robot, RunnerKind, RunnerSpec, JOINTS,
};
use crate::types::{projected_gravity, RobotState};

type Model = std::sync::Arc<TypedRunnableModel>;

/// A policy step's output in robot order.
#[derive(Debug, Clone)]
pub struct Targets {
    /// leg position targets (wheel entries unused)
    pub q: [f64; JOINTS],
    pub wheel_velocity: [f64; 4],
    /// clipped network output: next step's last action
    pub action: [f64; JOINTS],
    pub velocity_estimate: Option<[f64; 3]>,
}

/// rl_deploy's policy target clamp ranges (PolicyRunnerBase::limit_goal_pos_vel), f32, robot order: hipx widened by
/// policy_hipx_widen of its width at each end (f32 of the f64 sum, one FMA: its aarch64 code), the rest as configured;
/// the wheel entries limit the velocity targets.
pub fn joint_limits(robot: &Robot) -> ([f32; JOINTS], [f32; JOINTS]) {
    let mut lo = [0.0; JOINTS];
    let mut hi = [0.0; JOINTS];
    let w = robot.policy_hipx_widen;
    for i in 0..JOINTS {
        let (l, h) = (
            robot.policy_limit_lower[i] as f32 as f64,
            robot.policy_limit_upper[i] as f32 as f64,
        );
        if i % 4 == 0 {
            lo[i] = l.mul_add(1.0 + w, -(h * w)) as f32;
            hi[i] = h.mul_add(1.0 + w, -(l * w)) as f32;
        } else {
            lo[i] = l as f32;
            hi[i] = h as f32;
        }
    }
    (lo, hi)
}

/// rl_deploy's clamp of one target: above the range -> the upper end, else below it -> the lower end.
fn limit(v: f32, lo: f32, hi: f32) -> f32 {
    if hi < v {
        hi
    } else if v < lo {
        lo
    } else {
        v
    }
}

/// rl_deploy's last_leg_action_mid / last_wheel_action (policy_runner_base.cpp globals, zero at start): what a runner's
/// OnExit leaves for the next runner's OnEnter to build its last action from (the legs in per-leg order).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Handover {
    pub legs: [f32; 12],
    pub wheels: [f32; 4],
}

pub fn load_model(path: &Path) -> Result<Model> {
    let model = tract_onnx::onnx()
        .model_for_path(path)
        .with_context(|| format!("loading {}", path.display()))?;
    let typed = model
        .into_typed()
        .with_context(|| format!("typing {}", path.display()))?;
    // tract's declutter mis-shapes the transformer runners' command Tile; those run the typed graph as is
    match typed.clone().into_optimized() {
        Ok(optimized) => optimized.into_runnable(),
        Err(_) => typed.into_runnable(),
    }
    .with_context(|| format!("planning {}", path.display()))
}

/// The stand / StepMove runners' gait clock, in rl_deploy's f32 (get_gait_phase / update_gait_freq): a counter per
/// policy step; the frequency is re-picked from the command when a full period wraps.
#[derive(Debug, Clone)]
pub struct GaitClock {
    spec: GaitClockSpec,
    pub frequency: f32,
    half: f32,
    omega: f32,
    pub counter: f32,
}

impl GaitClock {
    pub fn new(spec: &GaitClockSpec) -> GaitClock {
        let f = spec.frequency as f32;
        GaitClock {
            spec: spec.clone(),
            frequency: f,
            half: (1.0 / f) * 0.5,
            omega: (f as f64 * PI * 2.0) as f32,
            counter: 0.0,
        }
    }

    /// (half period, sign, time left in the half period): sign of sinf(t x 2 pi f), fmodf(t, half) above 1e-6
    pub fn fields(&self, dt: f64) -> (f32, f32, f32) {
        let t = self.counter * dt as f32;
        let sign = if ((t * self.omega) as f64).sin() < 0.0 {
            -1.0
        } else {
            1.0
        };
        let r = t % self.half;
        (
            self.half,
            sign,
            if r as f64 > 1e-6 { self.half - r } else { 0.0 },
        )
    }

    /// update_gait_freq (after the command is built): a wrap of the period picks the frequency from (vx, vy, wz)
    pub fn update(&mut self, dt: f64, command: &[f64; 3]) {
        let dt = dt as f32;
        let period = 1.0 / self.frequency;
        let next = ((self.counter + 1.0) * dt) % period;
        if next < (self.counter * dt) % period {
            let g = &self.spec;
            let (vx, vy, wz) = (command[0] as f32, command[1] as f32, command[2] as f32);
            let f = (vx / g.vx_gain as f32 + 1.0).max(vy.abs() / g.vy_gain as f32 + 1.0);
            let f = ((wz.abs() / g.wz_gain as f32 + 1.0) * f)
                .clamp(g.min_frequency as f32, g.max_frequency as f32);
            if f != self.frequency {
                self.half = (1.0 / f) * 0.5;
                self.omega = (f as f64 * PI * 2.0) as f32;
                self.counter = ((1.0 / f) * (next / period)) / dt;
                self.frequency = f;
            }
        }
    }

    /// The step's end
    pub fn tick(&mut self) {
        self.counter += 1.0;
    }
}

/// SixAxisStand's mode input (cal_command, 0x2b5 / 0x2d4 / 0x2d8): -mode_byte, but 0 while armed and upright, the
/// arming counted over 100 upright steps; on gait 0xf001 it re-arms whenever it is off.
#[derive(Debug, Clone, Copy, Default)]
pub struct StandMode {
    armed: bool,
    count: i32,
    pub mode: i32,
}

impl StandMode {
    pub fn step(&mut self, mode_byte: bool, gait: u32, gravity_z: f32) {
        self.mode = -(mode_byte as i32);
        let upright = (gravity_z as f64) < -0.9;
        if gait == 0xf001 && !self.armed {
            self.armed = true;
            self.count = 0;
            self.mode = 0;
        }
        if upright && self.armed {
            self.mode = 0;
            self.count += 1;
            if self.count > 99 {
                self.armed = false;
                self.count = 0;
                self.mode = -1;
            }
        }
    }
}

/// The observation / history / action bookkeeping of one runner, without the network.
pub struct Pipeline {
    pub name: String,
    pub spec: RunnerSpec,
    pub default: [f64; JOINTS],
    pub kp: [f64; JOINTS],
    pub kd: [f64; JOINTS],
    pub limits: ([f32; JOINTS], [f32; JOINTS]),
    pub policy_dt: f64,
    pub params: BTreeMap<String, f64>,
    pub last_action: [f64; JOINTS],
    pub history: VecDeque<Vec<f32>>,
    pub phase: f64,
    pub command: [f64; 3],
    /// rl_deploy's slew-limited command in normalized axes (limit_cmd_vel's state), before the command scale
    pub command_state: [f32; 3],
    pub clock: Option<GaitClock>,
    pub stand_mode: StandMode,
    /// the last frame's projected gravity z (stand_mode reads it)
    gravity_z: f32,
    /// clamp the targets as rl_deploy's limit_goal_pos_vel does
    pub clamp_targets: bool,
    /// policy steps since the last reset
    pub steps: u64,
    /// policy steps since the last switch to this runner
    pub active_steps: u64,
    /// steps since the runner's OnEnter (rl_deploy's 0x48), warm-up ones included
    pub collected: u64,
}

fn arr16(what: &str, v: &[f64]) -> Result<[f64; JOINTS]> {
    v.try_into()
        .map_err(|_| anyhow!("{what} has {} values, needs {JOINTS}", v.len()))
}

impl Pipeline {
    pub fn new(name: &str, spec: &RunnerSpec, robot: &Robot, policy_dt: f64) -> Result<Pipeline> {
        Ok(Pipeline {
            name: name.to_string(),
            spec: spec.clone(),
            default: arr16("default_pose", &spec.default_pose)?,
            kp: arr16("kp", &tile(&spec.kp, JOINTS)?)?,
            kd: arr16("kd", &tile(&spec.kd, JOINTS)?)?,
            limits: joint_limits(robot),
            policy_dt,
            params: spec.command_params.clone(),
            last_action: [0.0; JOINTS],
            history: VecDeque::new(),
            phase: 0.0,
            command: [0.0; 3],
            command_state: [0.0; 3],
            clock: spec.gait_clock.as_ref().map(GaitClock::new),
            stand_mode: StandMode::default(),
            gravity_z: 0.0,
            clamp_targets: robot.policy_target_clamp,
            steps: 0,
            active_steps: 0,
            collected: 0,
        })
    }

    /// The runner becomes the active one (a gait switch): its start-up gains count from here.
    pub fn activate(&mut self) {
        self.active_steps = 0;
    }

    /// rl_deploy scales kp / kd for a runner's first steps after each switch to it (NaviCharge x1.3 for 29).
    pub fn gain_scale(&self) -> f64 {
        match self.spec.start_gain_scale {
            Some(k) if self.active_steps < self.spec.start_gain_steps => k,
            _ => 1.0,
        }
    }

    /// rl_deploy's OnExit when a gait switch leaves this runner: its last action, as leg positions and wheel velocities,
    /// into the handover. The base class reads the last action in per-leg order whatever the runner's layout and
    /// maps it back with default + scale x a (FMA); the absolute runners' (Extre, FinetuneStand, SixAxisStand,
    /// StepMove) copy it.
    pub fn on_exit(&self, h: &mut Handover) {
        let act = &self.spec.action;
        let (scale, wheel_scale) = (act.leg_scale[0] as f32, act.wheel_scale as f32);
        for (k, &i) in JointSet::LegsPerLeg.indices().iter().enumerate() {
            let a = self.last_action[i] as f32;
            h.legs[k] = if act.leg_mode == LegMode::Absolute {
                a
            } else {
                a.mul_add(scale, self.default[i] as f32)
            };
        }
        for (w, &i) in JointSet::Wheels.indices().iter().enumerate() {
            h.wheels[w] = self.last_action[i] as f32 * wheel_scale;
        }
    }

    /// rl_deploy's OnEnter when this runner becomes the active one (a gait switch, or rl_control's entry): the handover
    /// becomes its last action (the base class first maps the legs to actions in place, (q - default) / scale, and
    /// lays them out per leg when `per_leg`, else by type; the absolute runners lay positions out per leg), the wheels
    /// divided by its wheel scale in place; its history, command slew and step counters start from zero, its warm-up
    /// and gait clock over. The phase carries on.
    pub fn on_enter(&mut self, h: &mut Handover, per_leg: bool) {
        let act = &self.spec.action;
        let absolute = act.leg_mode == LegMode::Absolute;
        let (scale, wheel_scale) = (act.leg_scale[0] as f32, act.wheel_scale as f32);
        if !absolute {
            for (k, &i) in JointSet::LegsPerLeg.indices().iter().enumerate() {
                h.legs[k] = (h.legs[k] - self.default[i] as f32) / scale;
            }
        }
        for w in &mut h.wheels {
            *w /= wheel_scale;
        }
        let mut a = [0.0f32; JOINTS];
        if absolute || per_leg {
            for leg in 0..4 {
                a[4 * leg..4 * leg + 3].copy_from_slice(&h.legs[3 * leg..3 * leg + 3]);
                a[4 * leg + 3] = h.wheels[leg];
            }
        } else {
            for k in 0..12 {
                a[k] = h.legs[3 * (k % 4) + k / 4];
            }
            a[12..].copy_from_slice(&h.wheels);
        }
        self.last_action = a.map(|x| x as f64);
        self.history.clear();
        self.command = [0.0; 3];
        self.command_state = [0.0; 3];
        if let Some(clock) = &mut self.clock {
            clock.counter = 0.0;
        }
        self.stand_mode.armed = false;
        self.steps = 0;
        self.active_steps = 0;
        self.collected = 0;
    }

    pub fn reset(&mut self) {
        self.last_action = [0.0; JOINTS];
        self.history.clear();
        self.phase = 0.0;
        self.command = [0.0; 3];
        self.command_state = [0.0; 3];
        self.params = self.spec.command_params.clone();
        self.clock = self.spec.gait_clock.as_ref().map(GaitClock::new);
        self.stand_mode = StandMode::default();
        self.steps = 0;
        self.active_steps = 0;
        self.collected = 0;
    }

    /// Normalized axes [-1, 1] -> physical (vx, vy, wz) as rl_deploy's PolicyRunnerBase::limit_cmd_vel does it, in f32:
    /// the normalized command moves toward the axes by at most command_slew per policy step (state += clamp(axes -
    /// state)), then is scaled by command_max.
    pub fn update_command(&mut self, axes: &[f64; 3]) {
        let deadband = self.spec.axis_deadband.unwrap_or([0.0; 3]);
        let mut axes = *axes;
        for i in 0..3 {
            axes[i] = axes[i].clamp(-1.0, 1.0);
            if axes[i].abs() < deadband[i] {
                axes[i] = 0.0;
            }
        }
        let slew = self.spec.command_slew;
        for i in 0..3 {
            let a = axes[i] as f32;
            self.command_state[i] = match slew {
                Some(limit) => {
                    let limit = limit[i] as f32;
                    let mut d = a - self.command_state[i];
                    if limit < d.abs() {
                        d = limit.copysign(d);
                    }
                    d + self.command_state[i]
                }
                None => a,
            };
        }
        let mut target = [0.0; 3];
        for i in 0..3 {
            target[i] = (self.command_state[i] * self.spec.command_max[i] as f32) as f64;
        }
        if let Some(w) = &self.spec.wz_speed_limit {
            let w_max = (w.a / (target[0] + w.b) - w.c).min(w.d / (1.0 + w.e * target[1].abs()));
            target[2] = self.command_state[2] as f64 * w_max.clamp(w.lo, w.hi);
        }
        if let (Some(clamp), Some(above)) = (self.spec.wz_clamp, self.spec.wz_clamp_above_vx) {
            if target[0] > above {
                target[2] = target[2].clamp(-clamp, clamp);
            }
        }
        self.command = target;
    }

    /// One observation frame, in rl_deploy's f32 arithmetic: each value as f32, then times its scale in f32 (joint
    /// positions minus the f32 default pose first); gravity from the IMU's own angles when the state has them.
    /// The projected gravity the runners observe: from the IMU's own angles as rl_deploy computes it when the state
    /// has them, else from the quaternion.
    pub fn gravity(s: &RobotState) -> [f32; 3] {
        match s.imu_rpy_deg {
            Some(rpy) => crate::types::vendor_projected_gravity(rpy),
            None => projected_gravity(&s.quat).map(|g| g as f32),
        }
    }

    pub fn frame(&self, s: &RobotState) -> Result<Vec<f32>> {
        let f32s = |v: &[f64]| v.iter().map(|x| *x as f32).collect::<Vec<f32>>();
        let mut f: Vec<f32> = Vec::with_capacity(self.spec.frame);
        for block in &self.spec.obs {
            let (values, scale): (Vec<f32>, &[f64]) = match block {
                ObsBlock::Gyro { scale } => (f32s(&s.gyro), scale),
                ObsBlock::Gravity { scale } => (Self::gravity(s).to_vec(), scale),
                ObsBlock::Accel {
                    scale,
                    divisor: Some(d),
                } => {
                    let d = *d as f32;
                    (s.accel.iter().map(|a| *a as f32 / d).collect(), scale)
                }
                ObsBlock::Accel { scale, .. } => (f32s(&s.accel), scale),
                ObsBlock::Command { scale } => (f32s(&self.command), scale),
                ObsBlock::Constant { values, name } => {
                    match name.as_ref().and_then(|n| self.params.get(n)) {
                        Some(v) => (vec![*v as f32], &[]),
                        None => (f32s(values), &[]),
                    }
                }
                ObsBlock::Phase { .. } => {
                    // RLControl's phase is f32; its sin / cos in f64 (sin(phase x 2 pi)), stored as f32
                    let x = self.phase as f32 as f64 * (2.0 * PI);
                    (vec![x.sin() as f32, x.cos() as f32], &[])
                }
                ObsBlock::JointPos {
                    joints,
                    relative,
                    scale,
                } => (
                    joints
                        .indices()
                        .iter()
                        .map(|&i| {
                            let q = s.q[i] as f32;
                            if *relative {
                                q - self.default[i] as f32
                            } else {
                                q
                            }
                        })
                        .collect(),
                    scale,
                ),
                ObsBlock::JointVel { joints, scale } => (
                    joints.indices().iter().map(|&i| s.dq[i] as f32).collect(),
                    scale,
                ),
                ObsBlock::LastAction { scale } => (f32s(&self.last_action), scale),
                ObsBlock::HeightMap { rows, cols, scale } => {
                    let n = rows * cols;
                    let hm = match &s.height_map {
                        Some(h) if h.len() == n => f32s(h),
                        Some(h) => bail!(
                            "{}: height map has {} values, needs {n}",
                            self.name,
                            h.len()
                        ),
                        None => vec![0.0; n],
                    };
                    (hm, scale)
                }
            };
            if scale.is_empty() {
                f.extend(values);
            } else {
                let sc = scale_or_one(scale, values.len())?;
                f.extend(values.iter().zip(&sc).map(|(v, k)| v * *k as f32));
            }
        }
        if f.len() != self.spec.frame {
            bail!(
                "{}: frame is {}, config says {}",
                self.name,
                f.len(),
                self.spec.frame
            );
        }
        Ok(f)
    }

    /// The gait phase advances before it is observed (rl_deploy: the first frame after a reset sees one step).
    pub fn advance_phase(&mut self) {
        for block in &self.spec.obs {
            if let ObsBlock::Phase { frequency } = block {
                // rl_deploy: phase = f32(f64(phase) + dt x f64(f32 frequency)) - f32(trunc of that sum)
                let sum = self.phase as f32 as f64 + self.policy_dt * (*frequency as f32) as f64;
                self.phase = ((sum as f32) - (sum.trunc() as f32)) as f64;
            }
        }
    }

    /// A fresh sequence runner only collects frames for its first warmup_steps, holding the default pose (its last
    /// action is that pose, as the absolute-target networks would have output it).
    pub fn warmup(&mut self, frame: Vec<f32>) -> Result<Targets> {
        self.history.push_back(frame);
        while self.history.len() > self.spec.history {
            self.history.pop_front();
        }
        // rl_deploy builds the command every step (its gait clock and stand mode run on)
        self.sequence_command()?;
        let act = &self.spec.action;
        let mut a = [0.0; JOINTS];
        if act.leg_mode == LegMode::Absolute {
            let scale = tile(&act.leg_scale, 12)?;
            match act.layout {
                ActionLayout::PerLeg => {
                    for (k, &i) in JointSet::LegsPerLeg.indices().iter().enumerate() {
                        a[i] = self.default[i] / scale[k];
                    }
                }
                ActionLayout::LegsByTypeThenWheels => {
                    for (k, &i) in JointSet::LegsByType.indices().iter().enumerate() {
                        a[k] = self.default[i] / scale[k];
                    }
                }
            }
        }
        self.last_action = a;
        self.collected += 1;
        Ok(Targets {
            q: self.default,
            wheel_velocity: [0.0; 4],
            action: a,
            velocity_estimate: None,
        })
    }

    /// Push the frame into the history and build the network inputs (name order: state, command, uncertain).
    pub fn network_input(&mut self, frame: Vec<f32>) -> Result<Vec<Tensor>> {
        let n = self.spec.history;
        // rl_deploy's history starts as zeros (its constructor's, and OnEnter's)
        if self.history.is_empty() {
            self.history
                .extend(std::iter::repeat_n(vec![0.0; frame.len()], n));
        }
        self.history.pop_front();
        self.history.push_back(frame.clone());
        let width = self.spec.frame;
        match self.spec.kind {
            RunnerKind::Frames => {
                let mut x = frame;
                for h in &self.history {
                    x.extend_from_slice(h);
                }
                let len = x.len();
                Ok(vec![
                    tract_ndarray::Array2::from_shape_vec((1, len), x)?.into()
                ])
            }
            RunnerKind::Sequence => {
                let x: Vec<f32> = self.history.iter().flatten().copied().collect();
                let state: Tensor = tract_ndarray::Array3::from_shape_vec((1, n, width), x)?.into();
                let cmd = self.sequence_command()?;
                let len = cmd.len();
                let command: Tensor = tract_ndarray::Array2::from_shape_vec((1, len), cmd)?.into();
                let uncertain: Tensor = tract_ndarray::Array2::<f32>::zeros((1, 1)).into();
                Ok(vec![state, command, uncertain])
            }
        }
    }

    /// The sequence runners' command input, in rl_deploy's f32 (cal_command); the gait clock and stand mode advance.
    fn sequence_command(&mut self) -> Result<Vec<f32>> {
        let param = |name: &str, default: f64| *self.params.get(name).unwrap_or(&default);
        let (byte, gait) = (
            param("mode_byte", 0.0) != 0.0,
            param("gait", 0xf001 as f64) as u32,
        );
        self.stand_mode.step(byte, gait, self.gravity_z);
        let mut cmd = Vec::with_capacity(self.spec.command_vector.len());
        for entry in &self.spec.command_vector {
            cmd.push(match entry {
                CommandEntry::Value(v) => *v as f32,
                CommandEntry::Source(src) => self.command_source(src)?,
            });
        }
        let (command, dt) = (self.command, self.policy_dt);
        if let Some(clock) = &mut self.clock {
            clock.update(dt, &command);
            clock.tick();
        }
        Ok(cmd)
    }

    fn command_source(&self, e: &CommandSource) -> Result<f32> {
        let param = |name: &str| *self.params.get(name).unwrap_or(&0.0) as f32;
        // rl_deploy's f32 radians of a degree constant
        let rad = |deg: f64| ((deg as f32 / 180.0) as f64 * PI) as f32;
        let (scale, offset) = (e.scale as f32, e.offset as f32);
        Ok(match e.src.as_str() {
            "vx" => self.command[0] as f32,
            "vy" => self.command[1] as f32,
            "wz" => self.command[2] as f32,
            "param" => {
                let name = e
                    .name
                    .as_deref()
                    .ok_or_else(|| anyhow!("param entry needs a name"))?;
                match e.scale_deg {
                    Some(deg) => rad(deg) * param(name),
                    None => param(name).mul_add(scale, offset),
                }
            }
            "stand_mode" => self.stand_mode.mode as f32,
            "gait" => {
                let clock = self
                    .clock
                    .as_ref()
                    .ok_or_else(|| anyhow!("{}: gait entry needs gait_clock", self.name))?;
                let (half, sign, rem) = clock.fields(self.policy_dt);
                let v = match e.field.as_deref() {
                    Some("half_period") => half,
                    Some("sign") => sign,
                    Some("remaining") => rem,
                    other => bail!("unknown gait field {other:?}"),
                };
                v.mul_add(scale, offset)
            }
            "foot" => {
                let f = self
                    .spec
                    .feet
                    .as_ref()
                    .ok_or_else(|| anyhow!("{}: foot entry needs feet", self.name))?;
                let leg = e.leg.min(3);
                let [nx, ny] = f.nominal[leg].map(|v| v as f32);
                let (x, y) = if param("feet_fixed") != 0.0 {
                    (nx, f.fixed_y[leg] as f32)
                } else {
                    // Rz(yaw) nominal + (dx, dy), its FMAs
                    let (sin, cos) = crate::dock::libm::sincosf(rad(f.yaw_deg) * param("yaw"));
                    (
                        param("dx").mul_add(f.dx_scale as f32, (-sin).mul_add(ny, cos * nx)),
                        param("dy").mul_add(f.dy_scale as f32, cos.mul_add(ny, sin * nx)),
                    )
                };
                if e.axis.as_deref() == Some("y") {
                    y
                } else {
                    x
                }
            }
            other => bail!("unknown command source {other}"),
        })
    }

    /// Network output -> clipped action (kept as last action), phase advance, joint targets.
    pub fn apply_output(&mut self, raw: &[f32], vel: Option<&[f32]>) -> Result<Targets> {
        if raw.len() != JOINTS {
            bail!("{}: network output has {} values", self.name, raw.len());
        }
        let act = &self.spec.action;
        let mut a = [0.0; JOINTS];
        for i in 0..JOINTS {
            a[i] = (raw[i] as f64).clamp(-act.clip, act.clip);
        }
        self.last_action = a;
        self.steps += 1;
        self.active_steps += 1;
        self.collected += 1;
        let (leg_idx, leg_a, wheel_a): (&[usize], Vec<f64>, Vec<f64>) = match act.layout {
            ActionLayout::PerLeg => {
                let idx = JointSet::LegsPerLeg.indices();
                (
                    idx,
                    idx.iter().map(|&i| a[i]).collect(),
                    JointSet::Wheels.indices().iter().map(|&i| a[i]).collect(),
                )
            }
            ActionLayout::LegsByTypeThenWheels => (
                JointSet::LegsByType.indices(),
                a[..12].to_vec(),
                a[12..].to_vec(),
            ),
        };
        let scale = tile(&act.leg_scale, 12)?;
        let mut q = self.default;
        for (k, &ri) in leg_idx.iter().enumerate() {
            let base = if act.leg_mode == LegMode::Offset {
                self.default[ri]
            } else {
                0.0
            };
            // rl_deploy: default + scale x action in f32, then limit_goal_pos_vel
            let t = base as f32 + scale[k] as f32 * leg_a[k] as f32;
            q[ri] = if self.clamp_targets {
                limit(t, self.limits.0[ri], self.limits.1[ri])
            } else {
                t
            } as f64;
        }
        let mut wheel = [0.0; 4];
        for (k, &i) in JointSet::Wheels.indices().iter().enumerate() {
            let v = act.wheel_scale as f32 * wheel_a[k] as f32;
            wheel[k] = if self.clamp_targets {
                limit(v, self.limits.0[i], self.limits.1[i])
            } else {
                v
            } as f64;
        }
        let velocity_estimate = match vel {
            Some(v) if v.len() == 3 => Some([v[0] as f64, v[1] as f64, v[2] as f64]),
            _ => None,
        };
        Ok(Targets {
            q,
            wheel_velocity: wheel,
            action: a,
            velocity_estimate,
        })
    }
}

/// The network: tract (pure Rust), or ONNX Runtime loaded from a library (rl_deploy's own numerics).
enum Backend {
    Tract(Model),
    Ort(crate::ort::Session),
}

/// The ONNX Runtime library to infer with: $M20_ONNXRUNTIME (empty: none), else the config's [inference] onnxruntime
/// when that file is there; none = tract.
pub fn onnxruntime_library(configured: &str) -> Option<String> {
    match std::env::var("M20_ONNXRUNTIME") {
        Ok(p) => (!p.is_empty()).then_some(p),
        Err(_) => (!configured.is_empty() && Path::new(configured).exists())
            .then(|| configured.to_string()),
    }
}

pub struct Runner {
    pub pipeline: Pipeline,
    model: Backend,
    /// the last network frame, for tests and logging
    pub last_frame: Vec<f32>,
    /// the last network inputs (data, shape), kept when `keep_inputs` (offline comparisons)
    pub last_inputs: Vec<(Vec<f32>, Vec<i64>)>,
    pub keep_inputs: bool,
}

impl Runner {
    pub fn new(
        name: &str,
        spec: &RunnerSpec,
        robot: &Robot,
        policy_dir: &Path,
        policy_dt: f64,
    ) -> Result<Runner> {
        Self::with_inference(name, spec, robot, policy_dir, policy_dt, None)
    }

    /// [`Self::new`], inferring through ONNX Runtime loaded from `onnxruntime` when given.
    pub fn with_inference(
        name: &str,
        spec: &RunnerSpec,
        robot: &Robot,
        policy_dir: &Path,
        policy_dt: f64,
        onnxruntime: Option<&str>,
    ) -> Result<Runner> {
        let path = policy_dir.join(&spec.policy);
        let model = match onnxruntime {
            Some(lib) => Backend::Ort(crate::ort::Session::new(lib, &path)?),
            None => Backend::Tract(load_model(&path)?),
        };
        Ok(Runner {
            pipeline: Pipeline::new(name, spec, robot, policy_dt)?,
            model,
            last_frame: Vec::new(),
            last_inputs: Vec::new(),
            keep_inputs: false,
        })
    }

    pub fn reset(&mut self) {
        self.pipeline.reset();
    }

    pub fn infer(&mut self, inputs: Vec<Tensor>) -> Result<(Vec<f32>, Option<Vec<f32>>)> {
        if self.keep_inputs {
            self.last_inputs = inputs
                .iter()
                .map(|t| {
                    let shape = t.shape().iter().map(|d| *d as i64).collect();
                    Ok((
                        t.to_plain_array_view::<f32>()?.iter().copied().collect(),
                        shape,
                    ))
                })
                .collect::<Result<_>>()?;
        }
        let model = match &self.model {
            Backend::Tract(m) => m,
            Backend::Ort(session) => {
                let mut ins = inputs
                    .iter()
                    .map(|t| {
                        let shape = t.shape().iter().map(|d| *d as i64).collect();
                        Ok((
                            t.to_plain_array_view::<f32>()?.iter().copied().collect(),
                            shape,
                        ))
                    })
                    .collect::<Result<Vec<(Vec<f32>, Vec<i64>)>>>()?;
                let mut out = session.run(&mut ins)?.into_iter();
                let action = out.next().ok_or_else(|| anyhow!("no output"))?;
                return Ok((action, out.next()));
            }
        };
        let out = model.run(inputs.into_iter().map(|t| t.into()).collect())?;
        let first = out.first().ok_or_else(|| anyhow!("no output"))?;
        let action: Vec<f32> = first
            .to_plain_array_view::<f32>()?
            .iter()
            .copied()
            .collect();
        let vel = match out.get(1) {
            Some(v) => Some(v.to_plain_array_view::<f32>()?.iter().copied().collect()),
            None => None,
        };
        Ok((action, vel))
    }

    pub fn step(
        &mut self,
        s: &RobotState,
        axes: &[f64; 3],
        params: &BTreeMap<String, f64>,
    ) -> Result<Targets> {
        self.pipeline
            .params
            .extend(params.iter().map(|(k, v)| (k.clone(), *v)));
        self.pipeline.update_command(axes);
        self.pipeline.gravity_z = Pipeline::gravity(s)[2];
        let spec = &self.pipeline.spec;
        // SixAxisStand holds only while rl_control's entry stand-in runs (UserCommand 0x1c)
        let gate = spec
            .warmup_param
            .as_ref()
            .is_none_or(|p| self.pipeline.params.get(p).copied().unwrap_or(0.0) != 0.0);
        let warming = gate && self.pipeline.collected < spec.warmup_steps;
        if !warming {
            self.pipeline.advance_phase();
        }
        let frame = self.pipeline.frame(s)?;
        self.last_frame.clone_from(&frame);
        if warming {
            return self.pipeline.warmup(frame);
        }
        let inputs = self.pipeline.network_input(frame)?;
        let (action, vel) = self.infer(inputs)?;
        self.pipeline.apply_output(&action, vel.as_deref())
    }
}
