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

/// Policy target clamp: the joint range, hipx narrowed to mid +- fraction * half range.
pub fn joint_limits(robot: &Robot) -> ([f64; JOINTS], [f64; JOINTS]) {
    let mut lo = [0.0; JOINTS];
    let mut hi = [0.0; JOINTS];
    for i in 0..JOINTS {
        let (l, h) = (robot.joint_lower[i], robot.joint_upper[i]);
        if i % 4 == 0 {
            let (mid, half) = ((l + h) / 2.0, (h - l) / 2.0);
            lo[i] = mid - robot.hipx_limit_fraction * half;
            hi[i] = mid + robot.hipx_limit_fraction * half;
        } else {
            lo[i] = l;
            hi[i] = h;
        }
    }
    (lo, hi)
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

/// The stand / StepMove runners' gait clock: a counter per policy step, frequency re-picked each half period.
#[derive(Debug, Clone)]
pub struct GaitClock {
    spec: GaitClockSpec,
    pub frequency: f64,
    pub counter: f64,
}

impl GaitClock {
    pub fn new(spec: &GaitClockSpec) -> GaitClock {
        GaitClock {
            spec: spec.clone(),
            frequency: spec.frequency,
            counter: 0.0,
        }
    }

    /// (half period, sign, time left in the half period)
    pub fn fields(&self, dt: f64) -> (f64, f64, f64) {
        let t = self.counter * dt;
        let half = 0.5 / self.frequency;
        let sign = if (2.0 * PI * self.frequency * t).sin() >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let r = t % half;
        (half, sign, if r > 1e-6 { half - r } else { 0.0 })
    }

    pub fn advance(&mut self, dt: f64, command: &[f64; 3]) {
        let half = 0.5 / self.frequency;
        let before = (self.counter * dt / half + 1e-9).floor();
        self.counter += 1.0;
        if (self.counter * dt / half + 1e-9).floor() != before {
            let g = &self.spec;
            let f = (1.0 + command[2].abs() / g.wz_gain)
                * (1.0 + command[0] / g.vx_gain).max(1.0 + command[1].abs() / g.vy_gain);
            let f = f.clamp(g.min_frequency, g.max_frequency);
            self.counter *= self.frequency / f;
            self.frequency = f;
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
    pub limits: ([f64; JOINTS], [f64; JOINTS]),
    pub wheel_limit: f64,
    pub policy_dt: f64,
    pub params: BTreeMap<String, f64>,
    pub last_action: [f64; JOINTS],
    pub history: VecDeque<Vec<f32>>,
    pub phase: f64,
    pub command: [f64; 3],
    pub clock: Option<GaitClock>,
    /// policy steps since the last reset
    pub steps: u64,
}

fn arr16(v: &[f64]) -> [f64; JOINTS] {
    let mut a = [0.0; JOINTS];
    a.copy_from_slice(&v[..JOINTS]);
    a
}

impl Pipeline {
    pub fn new(name: &str, spec: &RunnerSpec, robot: &Robot, policy_dt: f64) -> Result<Pipeline> {
        Ok(Pipeline {
            name: name.to_string(),
            spec: spec.clone(),
            default: arr16(&spec.default_pose),
            kp: arr16(&tile(&spec.kp, JOINTS)?),
            kd: arr16(&tile(&spec.kd, JOINTS)?),
            limits: joint_limits(robot),
            wheel_limit: robot.wheel_velocity_limit,
            policy_dt,
            params: spec.command_params.clone(),
            last_action: [0.0; JOINTS],
            history: VecDeque::new(),
            phase: 0.0,
            command: [0.0; 3],
            clock: spec.gait_clock.as_ref().map(GaitClock::new),
            steps: 0,
        })
    }

    /// rl_deploy scales kp / kd for a runner's first steps (NaviCharge x1.3 for 29 steps).
    pub fn gain_scale(&self) -> f64 {
        match self.spec.start_gain_scale {
            Some(k) if self.steps < self.spec.start_gain_steps => k,
            _ => 1.0,
        }
    }

    pub fn reset(&mut self) {
        self.last_action = [0.0; JOINTS];
        self.history.clear();
        self.phase = 0.0;
        self.command = [0.0; 3];
        self.params = self.spec.command_params.clone();
        self.clock = self.spec.gait_clock.as_ref().map(GaitClock::new);
        self.steps = 0;
    }

    /// Normalized axes [-1, 1] -> physical (vx, vy, wz), slew-limited per policy step where configured.
    pub fn update_command(&mut self, axes: &[f64; 3]) {
        let deadband = self.spec.axis_deadband.unwrap_or([0.0; 3]);
        let mut axes = *axes;
        for i in 0..3 {
            axes[i] = axes[i].clamp(-1.0, 1.0);
            if axes[i].abs() < deadband[i] {
                axes[i] = 0.0;
            }
        }
        let mut target = [0.0; 3];
        for i in 0..3 {
            target[i] = axes[i] * self.spec.command_max[i];
        }
        if let Some(w) = &self.spec.wz_speed_limit {
            let w_max = (w.a / (target[0] + w.b) - w.c).min(w.d / (1.0 + w.e * target[1].abs()));
            target[2] = axes[2] * w_max.clamp(w.lo, w.hi);
        }
        if let (Some(clamp), Some(above)) = (self.spec.wz_clamp, self.spec.wz_clamp_above_vx) {
            if target[0] > above {
                target[2] = target[2].clamp(-clamp, clamp);
            }
        }
        let slew = self.spec.command_slew.unwrap_or([0.0; 3]);
        for i in 0..3 {
            self.command[i] = if slew[i] > 0.0 {
                self.command[i] + (target[i] - self.command[i]).clamp(-slew[i], slew[i])
            } else {
                target[i]
            };
        }
    }

    pub fn frame(&self, s: &RobotState) -> Result<Vec<f32>> {
        let mut f: Vec<f64> = Vec::with_capacity(self.spec.frame);
        for block in &self.spec.obs {
            let (values, scale): (Vec<f64>, &[f64]) = match block {
                ObsBlock::Gyro { scale } => (s.gyro.to_vec(), scale),
                ObsBlock::Gravity { scale } => (projected_gravity(&s.quat).to_vec(), scale),
                ObsBlock::Accel { scale } => (s.accel.to_vec(), scale),
                ObsBlock::Command { scale } => (self.command.to_vec(), scale),
                ObsBlock::Constant { values, name } => {
                    match name.as_ref().and_then(|n| self.params.get(n)) {
                        Some(v) => (vec![*v], &[]),
                        None => (values.clone(), &[]),
                    }
                }
                ObsBlock::Phase { .. } => (
                    vec![(2.0 * PI * self.phase).sin(), (2.0 * PI * self.phase).cos()],
                    &[],
                ),
                ObsBlock::JointPos {
                    joints,
                    relative,
                    scale,
                } => (
                    joints
                        .indices()
                        .iter()
                        .map(|&i| s.q[i] - if *relative { self.default[i] } else { 0.0 })
                        .collect(),
                    scale,
                ),
                ObsBlock::JointVel { joints, scale } => {
                    (joints.indices().iter().map(|&i| s.dq[i]).collect(), scale)
                }
                ObsBlock::LastAction { scale } => (self.last_action.to_vec(), scale),
                ObsBlock::HeightMap { rows, cols, scale } => {
                    let n = rows * cols;
                    let hm = match &s.height_map {
                        Some(h) if h.len() == n => h.clone(),
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
            let sc = scale_or_one(scale, values.len())?;
            f.extend(values.iter().zip(&sc).map(|(v, k)| v * k));
        }
        if f.len() != self.spec.frame {
            bail!(
                "{}: frame is {}, config says {}",
                self.name,
                f.len(),
                self.spec.frame
            );
        }
        Ok(f.into_iter().map(|v| v as f32).collect())
    }

    /// Push the frame into the history and build the network inputs (name order: state, command, uncertain).
    pub fn network_input(&mut self, frame: Vec<f32>) -> Result<Vec<Tensor>> {
        let n = self.spec.history;
        if self.history.is_empty() {
            self.history
                .extend(std::iter::repeat(frame.clone()).take(n));
        } else {
            self.history.pop_front();
            self.history.push_back(frame.clone());
        }
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
                let mut cmd = Vec::with_capacity(self.spec.command_vector.len());
                for entry in &self.spec.command_vector {
                    let v = match entry {
                        CommandEntry::Value(v) => *v,
                        CommandEntry::Source(src) => self.command_source(src)?,
                    };
                    cmd.push(v as f32);
                }
                let command_now = self.command;
                let dt = self.policy_dt;
                if let Some(clock) = &mut self.clock {
                    clock.advance(dt, &command_now);
                }
                let len = cmd.len();
                let command: Tensor = tract_ndarray::Array2::from_shape_vec((1, len), cmd)?.into();
                let uncertain: Tensor = tract_ndarray::Array2::<f32>::zeros((1, 1)).into();
                Ok(vec![state, command, uncertain])
            }
        }
    }

    fn command_source(&self, e: &CommandSource) -> Result<f64> {
        let param = |name: &str| *self.params.get(name).unwrap_or(&0.0);
        Ok(match e.src.as_str() {
            "vx" => self.command[0],
            "vy" => self.command[1],
            "wz" => self.command[2],
            "param" => {
                let name = e
                    .name
                    .as_deref()
                    .ok_or_else(|| anyhow!("param entry needs a name"))?;
                param(name) * e.scale + e.offset
            }
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
                v * e.scale + e.offset
            }
            "foot" => {
                let f = self
                    .spec
                    .feet
                    .as_ref()
                    .ok_or_else(|| anyhow!("{}: foot entry needs feet", self.name))?;
                let th = f.yaw_scale * param("yaw");
                let [nx, ny] = f.nominal[e.leg.min(3)];
                let x = th.cos() * nx - th.sin() * ny + f.dx_scale * param("dx");
                let y = th.sin() * nx + th.cos() * ny + f.dy_scale * param("dy");
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
        for block in &self.spec.obs {
            if let ObsBlock::Phase { frequency } = block {
                self.phase = (self.phase + self.policy_dt * frequency).rem_euclid(1.0);
            }
        }
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
            q[ri] = (base + scale[k] * leg_a[k]).clamp(self.limits.0[ri], self.limits.1[ri]);
        }
        let mut wheel = [0.0; 4];
        for k in 0..4 {
            wheel[k] = (act.wheel_scale * wheel_a[k]).clamp(-self.wheel_limit, self.wheel_limit);
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

pub struct Runner {
    pub pipeline: Pipeline,
    model: Model,
    /// the last network frame, for tests and logging
    pub last_frame: Vec<f32>,
}

impl Runner {
    pub fn new(
        name: &str,
        spec: &RunnerSpec,
        robot: &Robot,
        policy_dir: &Path,
        policy_dt: f64,
    ) -> Result<Runner> {
        let model = load_model(&policy_dir.join(&spec.policy))?;
        Ok(Runner {
            pipeline: Pipeline::new(name, spec, robot, policy_dt)?,
            model,
            last_frame: Vec::new(),
        })
    }

    pub fn reset(&mut self) {
        self.pipeline.reset();
    }

    pub fn infer(&self, inputs: Vec<Tensor>) -> Result<(Vec<f32>, Option<Vec<f32>>)> {
        let out = self
            .model
            .run(inputs.into_iter().map(|t| t.into()).collect())?;
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
        let frame = self.pipeline.frame(s)?;
        self.last_frame.clone_from(&frame);
        let inputs = self.pipeline.network_input(frame)?;
        let (action, vel) = self.infer(inputs)?;
        self.pipeline.apply_output(&action, vel.as_deref())
    }
}
