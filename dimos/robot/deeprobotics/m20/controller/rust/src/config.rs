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

//! m20_controller.toml, shared with the Python reference sim (dimos/simulation/m20_policy_sim).

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{bail, Context, Result};
use serde::Deserialize;

pub const JOINTS: usize = 16;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub robot: Robot,
    pub timing: Timing,
    pub safety: Safety,
    pub states: States,
    pub runners: BTreeMap<String, RunnerSpec>,
    #[serde(default)]
    pub realtime: Realtime,
    #[serde(default)]
    pub ecan: Ecan,
    #[serde(default)]
    pub command_input: CommandInput,
    /// docking: the reflector-post detector and charge_manager's dock / undock sequence
    pub dock: crate::dock::config::DockConfig,
    #[serde(default)]
    pub inference: Inference,
    /// basic_server's numbers the controller's basic_server layer uses
    pub basic_server: BasicServerConfig,
    /// rl_deploy's metric velocity command to normalized axes (DdsInterface::HandlerVel)
    pub real_steer: RealSteer,
    /// the runners safety.default_command_slew gave a slew (rl_deploy's have none)
    #[serde(skip)]
    pub slew_defaulted: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RealSteerGaits {
    pub gaits: Vec<u32>,
    pub divisor: [f64; 3],
}

/// DdsInterface::HandlerVel: /REAL_STEER (m/s, m/s, rad/s) over a divisor chosen by the commanded gait.
#[derive(Debug, Clone, Deserialize)]
pub struct RealSteer {
    pub by_gait: Vec<RealSteerGaits>,
    pub default: [f64; 3],
    pub rescale_gaits: Vec<u32>,
    pub rescale_to: f64,
    pub rescale_from: f64,
}

impl RealSteer {
    /// The normalized axes rl_deploy stores for velocity `v` received while `gait` is commanded (f32 as it).
    pub fn axes(&self, gait: u32, v: [f64; 3]) -> [f64; 3] {
        let d = self
            .by_gait
            .iter()
            .find(|g| g.gaits.contains(&gait))
            .map_or(self.default, |g| g.divisor);
        let (mut x, mut y) = (v[0] as f32, v[1] as f32);
        if self.rescale_gaits.contains(&gait) {
            let (ax, ay) = ((x.abs()) as f64, (y.abs()) as f64);
            if ax < self.rescale_to
                && ay < self.rescale_to
                && (ax >= self.rescale_from || ay >= self.rescale_from)
            {
                let m = if y.abs() <= x.abs() { ax } else { ay };
                let k = (self.rescale_to / m) as f32;
                x *= k;
                y *= k;
            }
        }
        let div = |a: f32, d: f64| (a as f64 / d) as f32 as f64;
        [div(x, d[0]), div(y, d[1]), div(v[2] as f32, d[2])]
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct BasicServerConfig {
    /// SetMotionState(1)'s wait for the stand (s)
    pub stand_wait_s: f64,
    pub align: AlignConfig,
}

/// aligningBodyPosture's fall-recovery checks.
#[derive(Debug, Clone, Deserialize)]
pub struct AlignConfig {
    pub checks: u32,
    pub check_period_s: f64,
    pub accel_z: f64,
    pub height_m: f64,
}

/// How the policies are evaluated.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct Inference {
    /// the ONNX Runtime library (libonnxruntime.so) to run them with, as rl_deploy does (its numerics to the bit),
    /// when it is there (rl_deploy's own on AOS); missing or empty: tract. $M20_ONNXRUNTIME overrides.
    #[serde(default)]
    pub onnxruntime: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Robot {
    pub joint_names: Vec<String>,
    pub joint_lower: Vec<f64>,
    pub joint_upper: Vec<f64>,
    /// rl_deploy's PolicyRunnerBase::limit_goal_pos_vel ranges (f32, per joint, robot order): hip / knee position
    /// targets and wheel velocity targets
    pub policy_limit_lower: Vec<f64>,
    pub policy_limit_upper: Vec<f64>,
    /// limit_goal_pos_vel widens the hipx range by this fraction of its width at each end
    pub policy_hipx_widen: f64,
    /// clamp every policy target as limit_goal_pos_vel does (rl_deploy does, after every step); off: unclamped
    #[serde(default = "yes")]
    pub policy_target_clamp: bool,
    /// the sim's actuator clip only: the motor drivers compute the PD torque, rl_deploy sends no torque limit
    pub torque_limit: Vec<f64>,
    pub thigh_length: f64,
    pub shank_length: f64,
    /// the hip's lateral offset (ControlParameters +8), for MotionInfo's height
    pub hip_offset: f64,
    /// what GetBodyHeight adds to the shank (the wheel radius)
    pub motion_height_wheel: f64,
    /// front-leg (hipx, hipy, knee) range the stand-up / lie-down IK clamps to before mirroring
    pub ik_lower: [f64; 3],
    pub ik_upper: [f64; 3],
}

#[derive(Debug, Clone, Deserialize)]
pub struct Timing {
    pub control_hz: f64,
    pub policy_decimation: u64,
    /// stand-up / lie-down time as rl_deploy keeps it: the hardware stamp (s since the bus opened) cast to f32, now
    /// minus entry (its splines step at the f32 resolution of that stamp); false: control ticks
    #[serde(default)]
    pub f32_state_clock: bool,
    /// true: that f32 is rl_deploy's seconds since start, so the splines coarsen with uptime (1/16 s steps after 6 days,
    /// its stand-up / lie-down stutter); false: seconds since the state's entry, the same arithmetic without the steps
    #[serde(default)]
    pub f32_state_clock_from_start: bool,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Safety {
    pub command_timeout_s: f64,
    /// joint-damping commands sent after a stop request before the process exits
    pub shutdown_damping_ticks: u64,
    /// at most this long lying a standing robot down on shutdown, before the damping ticks
    pub shutdown_lie_down_max_s: f64,
    /// ours: a non-finite axis, velocity or param is 0 and axes are clipped to [-1, 1] before anything reads them
    #[serde(default)]
    pub finite_commands: bool,
    /// ours: the command slew (normalized, per policy step) of runners rl_deploy gives none
    #[serde(default)]
    pub default_command_slew: Option<[f64; 3]>,
    /// ours: every runner's scaled command (m/s, m/s, rad/s) clamped to this
    #[serde(default)]
    pub command_limit: Option<[f64; 3]>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Gains {
    pub kp: Vec<f64>,
    pub kd: Vec<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Passive {
    pub kp: Vec<f64>,
    pub kd: Vec<f64>,
    pub min_s_before_stand: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct JointDamping {
    pub kp: Vec<f64>,
    pub kd: Vec<f64>,
    pub min_s: f64,
    /// ours: a fault the motors can still act on lies the robot down first (stopping it, as a lie-down), then damps
    #[serde(default)]
    pub lie_down_first: bool,
    /// ours: the operator's joint damping (motion 2, rl_deploy's soft e-stop) lies down first too
    #[serde(default)]
    pub lie_down_first_on_command: bool,
}

#[derive(Debug, Clone, Deserialize)]
pub struct StandUp {
    pub kp: Vec<f64>,
    pub kd: Vec<f64>,
    pub crouch_height: f64,
    pub stand_height: f64,
    pub phase_s: f64,
    pub hipx: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LieDown {
    pub kp: Vec<f64>,
    pub kd: Vec<f64>,
    pub lie_height: f64,
    pub phase_s: f64,
    pub hipx: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct StandIn {
    pub gait: u32,
    pub runner: String,
    pub on_entry: bool,
    pub on_switch: bool,
    pub first_s: f64,
    pub later_s: f64,
    pub params: BTreeMap<String, f64>,
    pub settle_s: f64,
    pub settle_params: BTreeMap<String, f64>,
}

/// RLControlState's rest on a hot standstill (basic_flat).
#[derive(Debug, Clone, Deserialize)]
pub struct Overheat {
    pub rest_c: f64,
    pub abort_motor_c: f64,
    pub abort_driver_c: f64,
    pub still_ticks: u32,
    pub still_body_speed: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RlControl {
    pub entry_gaits: Vec<u32>,
    pub entry_fallback_gait: u32,
    pub fallback_runner: String,
    /// a gait switch waits until |vx| + |vy| + |wz| (normalized) is below this
    pub switch_below_speed: f64,
    pub overheat: Overheat,
    #[serde(default)]
    pub stand_in: Vec<StandIn>,
    /// ours, not rl_deploy's: what keeps a gait switch calm (absent: rl_deploy's switch)
    #[serde(default)]
    pub safe_switch: Option<SafeSwitch>,
    /// ours: what rl_control does with the wheels while the command is zero
    #[serde(default)]
    pub zero_command: ZeroCommand,
}

/// Ours (ROBUSTNESS.md "Drift"): the wheels at a zero operator and ramped command.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct ZeroCommand {
    /// the 4 wheel velocity targets' mean subtracted (the policies' common-mode bias)
    pub cancel_wheel_drift: bool,
    /// the wheels held at their positions there with this kp (0: no hold)
    pub hold_kp: f64,
}

/// Ours (ROBUSTNESS.md): bridged switches, a still body before a switch or lie-down, a target blend.
#[derive(Debug, Clone, Deserialize)]
pub struct SafeSwitch {
    /// (from runner, to runner) switches that run `bridge` first; "*" matches any runner
    pub bridge_pairs: Vec<[String; 2]>,
    /// the stand policy a bridged switch runs (its gait field unused)
    pub bridge: StandIn,
    /// after a runner switch the leg targets move from the last command to the new runner's over this long (s)
    pub blend_s: f64,
    /// a gait switch (and, with lie_down_when_still, a lie-down) waits for the body to be still, not only the command
    pub still: Still,
    pub lie_down_when_still: bool,
    /// a lie-down also waits for the legs to be still (rad/s); stepping runners stand on the bridge first
    pub lie_leg_rad_s: f64,
    /// a switch or lie-down also goes ahead once the wheels stop slowing under this, the command at zero (m/s)
    pub settle_below_m_s: f64,
    /// slowing by less than this (m/s^2, the wheels' mean over a policy step) is not slowing
    pub settle_decel_m_s2: f64,
    pub lie_bridge_from: Vec<String>,
    /// a gait written during a stand-in is kept and switched to after it (rl_deploy: lost)
    pub keep_gait_during_stand_in: bool,
}

/// Params that share one range (a pose: each alone may reach its end, together they share it).
#[derive(Debug, Clone, Deserialize)]
pub struct ParamBudget {
    pub names: Vec<String>,
    pub max: f64,
}

/// The body is still: the wheels' mean rim speed and the body rate under these.
#[derive(Debug, Clone, Deserialize)]
pub struct Still {
    pub wheel_radius_m: f64,
    pub wheel_m_s: f64,
    pub gyro_rad_s: f64,
}

impl SafeSwitch {
    /// Whether a switch from runner `from` to `to` is bridged.
    pub fn bridged(&self, from: &str, to: &str) -> bool {
        let m = |p: &str, r: &str| p == "*" || p == r;
        from != to
            && self
                .bridge_pairs
                .iter()
                .any(|[f, t]| m(f, from) && m(t, to))
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct Enable {
    pub ticks: u64,
    pub on_enter: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct States {
    pub enable: Enable,
    pub passive: Passive,
    pub start_damping: Gains,
    pub joint_damping: JointDamping,
    pub stand_up: StandUp,
    pub lie_down: LieDown,
    pub rl_control: RlControl,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunnerKind {
    Frames,
    Sequence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointSet {
    LegsByType,
    LegsPerLeg,
    Wheels,
    AllPerLeg,
}

impl JointSet {
    pub fn indices(self) -> &'static [usize] {
        match self {
            JointSet::LegsByType => &[0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14],
            JointSet::LegsPerLeg => &[0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14],
            JointSet::Wheels => &[3, 7, 11, 15],
            JointSet::AllPerLeg => &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ObsBlock {
    Gyro {
        #[serde(default)]
        scale: Vec<f64>,
    },
    Gravity {
        #[serde(default)]
        scale: Vec<f64>,
    },
    Accel {
        #[serde(default)]
        scale: Vec<f64>,
        /// divided by this in f32 instead of scaled (the stand runners' accel / 9.81)
        #[serde(default)]
        divisor: Option<f64>,
    },
    Command {
        #[serde(default)]
        scale: Vec<f64>,
    },
    Constant {
        values: Vec<f64>,
        #[serde(default)]
        name: Option<String>,
    },
    Phase {
        frequency: f64,
    },
    JointPos {
        joints: JointSet,
        #[serde(default = "yes")]
        relative: bool,
        #[serde(default)]
        scale: Vec<f64>,
    },
    JointVel {
        joints: JointSet,
        #[serde(default)]
        scale: Vec<f64>,
    },
    LastAction {
        #[serde(default)]
        scale: Vec<f64>,
    },
    HeightMap {
        rows: usize,
        cols: usize,
        #[serde(default)]
        scale: Vec<f64>,
    },
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionLayout {
    LegsByTypeThenWheels,
    PerLeg,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LegMode {
    Offset,
    Absolute,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Action {
    pub layout: ActionLayout,
    pub clip: f64,
    pub leg_mode: LegMode,
    pub leg_scale: Vec<f64>,
    pub wheel_scale: f64,
}

/// One `command` input of a sequence runner: a constant, or a source with an affine map.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum CommandEntry {
    Value(f64),
    Source(CommandSource),
}

#[derive(Debug, Clone, Deserialize)]
pub struct CommandSource {
    /// vx | vy | wz | param | gait | foot | stand_mode
    pub src: String,
    #[serde(default)]
    pub name: Option<String>,
    /// gait: half_period | sign | remaining
    #[serde(default)]
    pub field: Option<String>,
    #[serde(default = "one")]
    pub scale: f64,
    #[serde(default)]
    pub offset: f64,
    /// param: times this angle as rl_deploy's f32(f64(f32(deg / 180)) x pi), instead of scale
    #[serde(default)]
    pub scale_deg: Option<f64>,
    #[serde(default)]
    pub leg: usize,
    #[serde(default)]
    pub axis: Option<String>,
}

fn one() -> f64 {
    1.0
}

#[derive(Debug, Clone, Deserialize)]
pub struct GaitClockSpec {
    pub frequency: f64,
    pub min_frequency: f64,
    pub max_frequency: f64,
    pub wz_gain: f64,
    pub vx_gain: f64,
    pub vy_gain: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Feet {
    pub nominal: [[f64; 2]; 4],
    /// the yaw param's full-scale angle (deg)
    pub yaw_deg: f64,
    pub dx_scale: f64,
    pub dy_scale: f64,
    /// the feet's y while the feet_fixed param is set (rl_deploy's rl_control stand-in), x nominal and no shift
    pub fixed_y: [f64; 4],
}

#[derive(Debug, Clone, Deserialize)]
pub struct WzSpeedLimit {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
    pub e: f64,
    pub lo: f64,
    pub hi: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RunnerSpec {
    pub gaits: Vec<u32>,
    pub policy: String,
    pub kind: RunnerKind,
    pub history: usize,
    pub frame: usize,
    pub default_pose: Vec<f64>,
    pub kp: Vec<f64>,
    pub kd: Vec<f64>,
    pub command_max: [f64; 3],
    #[serde(default)]
    pub command_slew: Option<[f64; 3]>,
    /// ours: the operator's params this runner reads, clamped to [lo, hi]
    #[serde(default)]
    pub param_limits: BTreeMap<String, [f64; 2]>,
    /// ours: these params scaled down together so their absolute values sum to at most `max`
    #[serde(default)]
    pub param_budget: Option<ParamBudget>,
    /// ours: |vx|, |vy|, |wz| of the scaled command clamped to these (safety.command_limit fills it)
    #[serde(default)]
    pub command_limit: Option<[f64; 3]>,
    #[serde(default)]
    pub wz_clamp: Option<f64>,
    #[serde(default)]
    pub wz_clamp_above_vx: Option<f64>,
    #[serde(default)]
    pub axis_deadband: Option<[f64; 3]>,
    #[serde(default)]
    pub wz_speed_limit: Option<WzSpeedLimit>,
    #[serde(default)]
    pub gait_clock: Option<GaitClockSpec>,
    #[serde(default)]
    pub feet: Option<Feet>,
    #[serde(default)]
    pub start_gain_scale: Option<f64>,
    #[serde(default)]
    pub start_gain_steps: u64,
    #[serde(default)]
    pub warmup_steps: u64,
    /// warm up only while this param is set (rl_deploy's SixAxisStand: UserCommand 0x1c, rl_control's entry stand-in)
    #[serde(default)]
    pub warmup_param: Option<String>,
    #[serde(default)]
    pub command_vector: Vec<CommandEntry>,
    #[serde(default)]
    pub command_params: BTreeMap<String, f64>,
    pub obs: Vec<ObsBlock>,
    pub action: Action,
}

/// Real-time placement of the 1 kHz loop and the policy thread (Linux; ignored elsewhere).
#[derive(Debug, Clone, Deserialize)]
pub struct Realtime {
    #[serde(default)]
    pub control_cpu: Option<usize>,
    #[serde(default)]
    pub policy_cpu: Option<usize>,
    #[serde(default)]
    pub control_fifo_priority: Option<i32>,
    #[serde(default)]
    pub policy_fifo_priority: Option<i32>,
    #[serde(default)]
    pub lock_memory: bool,
    /// off Linux only (dev runs): sleep this much short of each deadline, then yield to it (macOS oversleeps ~1 ms)
    #[serde(default = "default_sleep_slack_s")]
    pub sleep_slack_s: f64,
}

fn default_sleep_slack_s() -> f64 {
    0.0015
}

impl Default for Realtime {
    fn default() -> Self {
        Realtime {
            control_cpu: None,
            policy_cpu: None,
            control_fifo_priority: None,
            policy_fifo_priority: None,
            lock_memory: false,
            sleep_slack_s: default_sleep_slack_s(),
        }
    }
}

/// The eth1 motor bus (see the ECAN backend); defaults are the values seen on the robot.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct Ecan {
    pub interface: String,
    pub ethertype: u16,
    pub destination_mac: [u8; 6],
    pub board_id: u8,
    pub send_flags: u32,
    pub recv_flags: u32,
    pub reply_timeout_s: f64,
    pub control_word: u16,
    /// control word of the enable frames (SetJointEnable)
    pub enable_control_word: u16,
    pub kp_scale: f64,
    pub kd_scale: f64,
    // controller joint angle = direction * (motor angle - offset), hips and knees wrapped into (-pi, pi]; defaults are
    // the vendor calibration (m20-ecan `Calibration::m20`), checked on the robot through a stand-up
    pub position_offset: Vec<f64>,
    pub direction: Vec<f64>,
    // send the BMS charge-enable request (0xb3) when m20-battery's rule asks, in place of the next joint commands
    // (rl_deploy's way); off by default
    pub charge_enable: bool,
    // backend "bus" only: let commands carrying kp / kd onto the motor bus. Off (the default) every such command goes
    // out as rl_deploy's sitting command (all gains zero), so the controller cannot move the robot. Never turn it on
    // on the robot without an operator's go.
    pub allow_gains: bool,
    // backend "bus" only: stream every frame sent and received here, in the ecan_tap.so shim's format, so the dimos
    // controller module (tap mode) publishes joint state and batteries as it does beside rl_deploy; empty = off
    pub tap_socket: String,
}

impl Default for Ecan {
    fn default() -> Self {
        Ecan {
            interface: "eth1".into(),
            ethertype: 0x4452,
            destination_mac: [0x22, 0x33, 0x44, 0x55, 0x66, 0x77],
            board_id: 0x79,
            send_flags: 0x8000_0000,
            recv_flags: 0x8000_0000,
            reply_timeout_s: 0.0008,
            control_word: 4,
            enable_control_word: 2,
            kp_scale: 10.0,
            kd_scale: 10.0,
            position_offset: vendor_calibration().0,
            direction: vendor_calibration().1,
            charge_enable: false,
            allow_gains: false,
            tap_socket: "/var/opt/robot/data/ecan_capture/live.sock".into(),
        }
    }
}

use crate::frames::vendor_calibration;

/// Where operator commands arrive: JSON lines over UDP.
#[derive(Debug, Clone, Deserialize)]
pub struct CommandInput {
    pub udp_bind: String,
}

impl Default for CommandInput {
    fn default() -> Self {
        CommandInput {
            udp_bind: "127.0.0.1:43920".into(),
        }
    }
}

/// A 1-element array broadcast, or a shorter array repeated, to length n.
pub fn tile(values: &[f64], n: usize) -> Result<Vec<f64>> {
    if values.is_empty() || !n.is_multiple_of(values.len()) {
        bail!("cannot tile {} values to {}", values.len(), n);
    }
    Ok((0..n).map(|i| values[i % values.len()]).collect())
}

/// `scale` of an obs block, tiled to the block size (empty = 1).
pub fn scale_or_one(values: &[f64], n: usize) -> Result<Vec<f64>> {
    if values.is_empty() {
        Ok(vec![1.0; n])
    } else {
        tile(values, n)
    }
}

impl Config {
    pub fn load(path: &Path) -> Result<Config> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let mut cfg: Config =
            toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
        if let Some(limit) = cfg.safety.command_limit {
            for r in cfg.runners.values_mut() {
                r.command_limit.get_or_insert(limit);
            }
        }
        if let Some(slew) = cfg.safety.default_command_slew {
            for (name, r) in cfg
                .runners
                .iter_mut()
                .filter(|(_, r)| r.command_slew.is_none())
            {
                r.command_slew = Some(slew);
                cfg.slew_defaulted.push(name.clone());
            }
        }
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn validate(&self) -> Result<()> {
        let positive = [
            ("timing.control_hz", self.timing.control_hz),
            ("robot.thigh_length", self.robot.thigh_length),
            ("robot.shank_length", self.robot.shank_length),
        ];
        for (name, v) in positive {
            if !(v.is_finite() && v > 0.0) {
                bail!("{name} must be a positive number, is {v}");
            }
        }
        if self.timing.policy_decimation == 0 {
            bail!("timing.policy_decimation must be at least 1");
        }
        let durations = [
            ("safety.command_timeout_s", self.safety.command_timeout_s),
            ("ecan.reply_timeout_s", self.ecan.reply_timeout_s),
            ("realtime.sleep_slack_s", self.realtime.sleep_slack_s),
            ("states.stand_up.phase_s", self.states.stand_up.phase_s),
            ("states.lie_down.phase_s", self.states.lie_down.phase_s),
        ];
        for (name, v) in durations {
            if !(v.is_finite() && v >= 0.0) {
                bail!("{name} must be a non-negative number of seconds, is {v}");
            }
        }
        let st = &self.states;
        let state_gains = [
            ("passive", &st.passive.kp, &st.passive.kd),
            ("start_damping", &st.start_damping.kp, &st.start_damping.kd),
            ("joint_damping", &st.joint_damping.kp, &st.joint_damping.kd),
            ("stand_up", &st.stand_up.kp, &st.stand_up.kd),
            ("lie_down", &st.lie_down.kp, &st.lie_down.kd),
        ];
        for (name, kp, kd) in state_gains {
            tile(kp, JOINTS).with_context(|| format!("states.{name}.kp"))?;
            tile(kd, JOINTS).with_context(|| format!("states.{name}.kd"))?;
        }
        tile(&self.ecan.position_offset, JOINTS).context("ecan.position_offset")?;
        let direction = tile(&self.ecan.direction, JOINTS).context("ecan.direction")?;
        if direction.iter().any(|d| d.abs() != 1.0) {
            bail!("ecan.direction entries must be 1 or -1");
        }
        let lens = [
            ("joint_names", self.robot.joint_names.len()),
            ("joint_lower", self.robot.joint_lower.len()),
            ("policy_limit_lower", self.robot.policy_limit_lower.len()),
            ("policy_limit_upper", self.robot.policy_limit_upper.len()),
            ("joint_upper", self.robot.joint_upper.len()),
        ];
        for (name, len) in lens {
            if len != JOINTS {
                bail!("robot.{name} needs {JOINTS} entries");
            }
        }
        tile(&self.robot.torque_limit, JOINTS).context("robot.torque_limit")?;
        if !self
            .runners
            .contains_key(&self.states.rl_control.fallback_runner)
        {
            bail!("states.rl_control.fallback_runner names no runner");
        }
        for si in &self.states.rl_control.stand_in {
            if !self.runners.contains_key(&si.runner) {
                bail!(
                    "stand_in for gait {:#x} names no runner {}",
                    si.gait,
                    si.runner
                );
            }
        }
        if let Some(ss) = &self.states.rl_control.safe_switch {
            if !self.runners.contains_key(&ss.bridge.runner) {
                bail!("safe_switch.bridge names no runner {}", ss.bridge.runner);
            }
        }
        for (name, r) in &self.runners {
            if r.default_pose.len() != JOINTS {
                bail!("runner {name}: default_pose needs {JOINTS} entries");
            }
            tile(&r.kp, JOINTS).with_context(|| format!("runner {name} kp"))?;
            tile(&r.kd, JOINTS).with_context(|| format!("runner {name} kd"))?;
            tile(&r.action.leg_scale, 12)
                .with_context(|| format!("runner {name} action.leg_scale"))?;
            if let Some(g) = &r.gait_clock {
                let gains = [g.frequency, g.wz_gain, g.vx_gain, g.vy_gain];
                if gains.iter().any(|v| !(v.is_finite() && *v > 0.0)) {
                    bail!("runner {name}: gait_clock frequency and gains must be positive");
                }
            }
            let size: usize = r.obs.iter().map(block_size).sum();
            if size != r.frame {
                bail!(
                    "runner {name}: obs blocks add up to {size}, frame is {}",
                    r.frame
                );
            }
        }
        self.dock.validate()?;
        for gait in [
            self.dock.sequence.approach_gait,
            self.dock.sequence.finetune_gait,
            self.dock.sequence.basic_flat_gait,
            self.dock.sequence.default_restore_gait,
        ] {
            if !self.runners.values().any(|r| r.gaits.contains(&gait)) {
                bail!("dock.sequence gait {gait:#x} has no runner");
            }
        }
        Ok(())
    }

    /// The runner a gait code selects, or the fallback runner.
    pub fn runner_for_gait(&self, gait: u32) -> &str {
        self.runners
            .iter()
            .find(|(_, r)| r.gaits.contains(&gait))
            .map(|(n, _)| n.as_str())
            .unwrap_or(&self.states.rl_control.fallback_runner)
    }

    /// This config with every difference of ours from rl_deploy (ROBUSTNESS.md) off: replays of its recordings.
    pub fn rl_deploy(mut self) -> Config {
        self.states.rl_control.safe_switch = None;
        self.states.rl_control.zero_command = ZeroCommand::default();
        self.states.joint_damping.lie_down_first = false;
        self.states.joint_damping.lie_down_first_on_command = false;
        self.safety.finite_commands = false;
        self.safety.default_command_slew = None;
        for name in std::mem::take(&mut self.slew_defaulted) {
            if let Some(r) = self.runners.get_mut(&name) {
                r.command_slew = None;
            }
        }
        self.safety.command_limit = None;
        for r in self.runners.values_mut() {
            r.param_limits.clear();
            r.command_limit = None;
            r.param_budget = None;
        }
        self
    }

    pub fn control_dt(&self) -> f64 {
        1.0 / self.timing.control_hz
    }
}

pub fn block_size(b: &ObsBlock) -> usize {
    match b {
        ObsBlock::Gyro { .. }
        | ObsBlock::Gravity { .. }
        | ObsBlock::Accel { .. }
        | ObsBlock::Command { .. } => 3,
        ObsBlock::Constant { values, .. } => values.len(),
        ObsBlock::Phase { .. } => 2,
        ObsBlock::JointPos { joints, .. } | ObsBlock::JointVel { joints, .. } => {
            joints.indices().len()
        }
        ObsBlock::LastAction { .. } => JOINTS,
        ObsBlock::HeightMap { rows, cols, .. } => rows * cols,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        Config::load(&path).unwrap()
    }

    #[test]
    fn rejects_what_would_panic_later() {
        let mut c = config();
        c.timing.control_hz = 0.0;
        assert!(c.validate().is_err());
        let mut c = config();
        c.states.stand_up.kp = vec![1.0; 3];
        assert!(c.validate().is_err());
        let mut c = config();
        c.ecan.direction = vec![0.0];
        assert!(c.validate().is_err());
        let mut c = config();
        c.ecan.reply_timeout_s = -1.0;
        assert!(c.validate().is_err());
        assert!(config().validate().is_ok());
    }
}
