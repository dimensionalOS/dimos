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
use crate::types::{roll_pitch, JointCommand, RobotState, Target, UserCommand};

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

/// rl_deploy's basic flat gait (its entry fallback and the overheat rest's).
pub const GAIT_BASIC_FLAT: u32 = 0x1001;

/// rl_deploy's MotionInfo height (GetBodyHeight 0x170cc0, f32 like it): each leg's wheel contact (FK with the hip
/// offset, the shank extended by the wheel radius) below its hip, rotated by the IMU roll and pitch (yaw zeroed), its z
/// accumulated in f64 with weight 0.25.
pub fn motion_info_height(robot: &Robot, q: &[f64; JOINTS], roll: f32, pitch: f32) -> f32 {
    let (f8, l1, l2) = (
        robot.hip_offset as f32,
        robot.thigh_length as f32,
        robot.shank_length as f32,
    );
    let shank = l2 as f64 + robot.motion_height_wheel;
    // quaternion z(0) * y(pitch) * x(roll), as its three sincosf halves compose
    let (sy, cy) = (pitch * 0.5).sin_cos();
    let (sx, cx) = (roll * 0.5).sin_cos();
    let (w, x, y, z) = (cy * cx, cy * sx, sy * cx, -(sy * sx));
    // the rotation matrix's third row
    let r = [
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
    ];
    let mut h = 0.0f64;
    for leg in 0..4 {
        let (hx, hy, kn) = (
            q[4 * leg] as f32,
            q[4 * leg + 1] as f32,
            q[4 * leg + 2] as f32,
        );
        let (s1, c1) = ((hy as f64).sin(), (hy as f64).cos());
        let (s12, c12) = ((kn as f64 + hy as f64).sin(), (kn as f64 + hy as f64).cos());
        let (sh, ch) = ((hx as f64).sin(), (hx as f64).cos());
        let c = c12 * shank + c1 * l1 as f64;
        let px = -((s12 * shank + s1 * l1 as f64) as f32);
        let py = (c * sh + f8 as f64 * ch) as f32;
        let pz = -((f8 as f64 * sh - ch * c) as f32);
        h += (r[0] * px + r[1] * py + r[2] * pz) as f64 * 0.25;
        h = h as f32 as f64;
    }
    h as f32
}

/// The IMU's roll and pitch as rl_deploy has them (f32 rad from yesense_node's degrees, else from the orientation).
pub fn imu_roll_pitch(s: &RobotState) -> (f32, f32) {
    match s.imu_rpy_deg {
        Some(rpy) => {
            let rad = |d: f32| ((d / 180.0) as f64 * std::f64::consts::PI) as f32;
            (rad(rpy[0]), rad(rpy[1]))
        }
        None => {
            let (r, p) = roll_pitch(&s.quat);
            (r as f32, p as f32)
        }
    }
}

/// The gait that enters RL control straight from Idle (basic_server's aligningBodyPosture, SixAxisStand).
pub const ALIGN_GAIT: u32 = 0xf003;

/// safety.finite_commands (ours): non-finite axes / velocity -> 0, axes clipped, non-finite params dropped.
pub fn finite_command(cmd: &UserCommand) -> UserCommand {
    let zero = |v: f64| if v.is_finite() { v } else { 0.0 };
    let mut c = cmd.clone();
    c.axes = c.axes.map(|a| zero(a).clamp(-1.0, 1.0));
    c.velocity = c.velocity.map(|v| v.map(zero));
    c.params.retain(|_, v| v.is_finite());
    c
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

/// SafeController::Run's tilt limits for a UserCommand target and gait (f32 rad angles, compared as f64, strictly).
#[allow(clippy::approx_constant)] // the vendor's rounded limits (0.7854, 1.5708), not pi / 4 and pi / 2
pub fn tilt_fault(target: Target, gait: u32, roll: f32, pitch: f32) -> Option<String> {
    let (roll, pitch) = (roll.abs() as f64, pitch.abs() as f64);
    let (max_roll, max_pitch) = match target {
        Target::Passive => (0.349066, 0.349066),
        Target::Stand => (0.7854, 0.7854),
        Target::RlControl => {
            if gait & 0xff00 == 0x2000 || gait == 0xf003 {
                return None;
            }
            (0.7854, if gait == 0x1002 { 1.5708 } else { 1.134464 })
        }
        _ => return None,
    };
    (roll > max_roll || pitch > max_pitch).then(|| {
        format!(
            "tilt roll {:.1} pitch {:.1} deg (limits {:.0} / {:.0})",
            roll.to_degrees(),
            pitch.to_degrees(),
            f64::to_degrees(max_roll),
            f64::to_degrees(max_pitch)
        )
    })
}

/// rl_deploy's driver faults (SafeController::Run): every tick, any driver status word other than 1 (none when the
/// source reports no status words); every 1000 ticks (after the first 1000), the temperatures, latched per joint:
/// a motor at 120 deg C or more until it is below 100, a driver at 100 or more until it is below 80; the fault
/// holds while any is latched.
#[derive(Debug, Clone, Default)]
pub struct DriverWatch {
    ticks: u32,
    motor_hot: u32,
    driver_hot: u32,
}

impl DriverWatch {
    pub fn check(&mut self, s: &RobotState, names: &[String]) -> Option<String> {
        let d = s.drivers.as_ref()?;
        self.ticks += 1;
        if self.ticks > 999 {
            self.ticks = 0;
            for i in 0..JOINTS {
                let (motor, driver) = (d.motor_temperature_c[i], d.driver_temperature_c[i]);
                if motor >= 120.0 {
                    self.motor_hot |= 1 << i;
                } else if motor < 100.0 {
                    self.motor_hot &= !(1 << i);
                }
                if driver >= 100.0 {
                    self.driver_hot |= 1 << i;
                } else if driver < 80.0 {
                    self.driver_hot &= !(1 << i);
                }
            }
        }
        let first = |bits: u32| {
            (0..JOINTS)
                .find(|i| bits & (1 << i) != 0)
                .map(|i| names[i].as_str())
        };
        if let Some(i) = (0..JOINTS).find(|&i| d.status_word[i] != 1) {
            Some(format!(
                "joint {} driver status word {:#06x}",
                names[i], d.status_word[i]
            ))
        } else if let Some(n) = first(self.motor_hot) {
            Some(format!("joint {n} motor overheated"))
        } else {
            first(self.driver_hot).map(|n| format!("joint {n} driver overheated"))
        }
    }
}

/// rl_deploy's joint-data checks (SafeController::IsJointDataNormal, 0x1773a8, on every 1 ms SafeController tick):
/// per joint, a run of 31 readings with a NaN in its position, velocity or torque; a run of 51 readings identical to
/// the one before (a frozen motor; all 16 joints); a run of 31 hip / knee positions more than 0.1 + 0.14 rad past
/// the range (ControlParameters' front-leg range, mirrored per leg as it builds it; f32 limits, f64 sums). A count
/// restarts when its condition stops, and the fault with it.
#[derive(Debug, Clone)]
pub struct JointWatch {
    previous: Option<[[f32; 3]; JOINTS]>,
    nan: [u32; JOINTS],
    frozen: [u32; JOINTS],
    out_of_range: [u32; JOINTS],
    lower: [f32; JOINTS],
    upper: [f32; JOINTS],
}

impl JointWatch {
    pub fn new(robot: &crate::config::Robot) -> JointWatch {
        let (lo, hi) = (
            robot.ik_lower.map(|v| v as f32),
            robot.ik_upper.map(|v| v as f32),
        );
        let mut lower = [0.0f32; JOINTS];
        let mut upper = [0.0f32; JOINTS];
        // FL as given, FR hipx mirrored, HL hipy / knee mirrored, HR both (the vectors IsJointDataNormal builds)
        for leg in 0..4 {
            let (mirror_hipx, mirror_rest) = (leg % 2 == 1, leg >= 2);
            for j in 0..3 {
                let mirror = if j == 0 { mirror_hipx } else { mirror_rest };
                let (l, h) = if mirror {
                    (-hi[j], -lo[j])
                } else {
                    (lo[j], hi[j])
                };
                lower[4 * leg + j] = l;
                upper[4 * leg + j] = h;
            }
        }
        JointWatch {
            previous: None,
            nan: [0; JOINTS],
            frozen: [0; JOINTS],
            out_of_range: [0; JOINTS],
            lower,
            upper,
        }
    }

    /// One reading; the fault it raises, if any.
    pub fn check(&mut self, s: &RobotState, names: &[String]) -> Option<String> {
        let now: [[f32; 3]; JOINTS] =
            std::array::from_fn(|i| [s.q[i] as f32, s.dq[i] as f32, s.tau[i] as f32]);
        let previous = *self.previous.get_or_insert(now);
        let mut fault = None;
        for i in 0..JOINTS {
            let v = now[i];
            self.nan[i] = if v.iter().any(|x| x.is_nan()) {
                self.nan[i] + 1
            } else {
                0
            };
            // f32 ==, as rl_deploy compares (a NaN is never equal)
            self.frozen[i] = if (0..3).all(|k| v[k] == previous[i][k]) {
                self.frozen[i] + 1
            } else {
                0
            };
            if i % 4 != 3 {
                let q = v[0] as f64;
                let past =
                    self.upper[i] as f64 + 0.1 + 0.14 < q || q < self.lower[i] as f64 - 0.1 - 0.14;
                self.out_of_range[i] = if past { self.out_of_range[i] + 1 } else { 0 };
            }
            if fault.is_none() {
                fault = if self.frozen[i] >= 51 {
                    // fixed text while it lasts (the status logs every change of the fault)
                    Some(format!("joint {} frozen (51 identical readings)", names[i]))
                } else if self.nan[i] > 30 {
                    Some(format!("joint {} NaN (31 readings)", names[i]))
                } else if self.out_of_range[i] > 30 {
                    Some(format!("joint {} beyond its range (31 readings)", names[i]))
                } else {
                    None
                };
            }
        }
        self.previous = Some(now);
        fault
    }
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
    /// safe_switch's bridge (ours): gait writes during it are kept and a lie-down is not refused
    bridge: bool,
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
    joint_watch: JointWatch,
    driver_watch: DriverWatch,
    enable_left: u64,
    fault_checks: bool,
    stand_ins_run: std::collections::BTreeSet<u32>,
    rl_entries: u64,
    stand_in: Option<ActiveStandIn>,
    /// rl_deploy's accelerometer mean over the last 20 rl_control ticks (kept across rl_control entries)
    accel_average: crate::types::AccelAverage,
    gait: u32,
    /// rl_deploy's UserCommand gait slot (UC+0x10): a gait write lands in it, a stand-in overwrites it
    target_gait: u32,
    /// the last gait write seen (value, gait_seq): a change of either is a new write
    seen_gait: (u32, u64),
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
    /// RLControlState's rest on a hot standstill (0x144): 0 none, 1 resting (the stand-in), 3 rested until moving
    rest: u8,
    rest_ticks: u32,
    /// a joint got too hot during a rest (0x15d): no rest until moving
    rest_hot: bool,
    /// the last tick's accelerometer and joint positions (basic_server's fall-recovery check reads them)
    pub last_accel: [f64; 3],
    pub last_q: [f64; JOINTS],
    /// the last tick's IMU roll and pitch (rad, f32 as rl_deploy's GetImuRpy)
    pub last_roll_pitch: [f32; 2],
    /// safe_switch's blend (ours): the leg targets it starts from and how far in it is (s)
    blend_from: Option<([f64; JOINTS], f64)>,
    /// rl_control's last leg position command
    last_leg_q: [f64; JOINTS],
    /// a lie-down asked from rl_control may go ahead (safe_switch: the body is still)
    lie_down_ready: bool,
}

impl Controller {
    pub fn new(cfg: Config) -> Controller {
        let robot = cfg.robot.clone();
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
            joint_watch: JointWatch::new(&robot),
            driver_watch: DriverWatch::default(),
            enable_left: 0,
            fault_checks: true,
            stand_ins_run: Default::default(),
            rl_entries: 0,
            stand_in: None,
            gait: 0,
            target_gait: 0,
            seen_gait: (0, 0),
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
            rest: 0,
            rest_ticks: 0,
            rest_hot: false,
            last_accel: [0.0; 3],
            last_q: [0.0; JOINTS],
            last_roll_pitch: [0.0; 2],
            blend_from: None,
            last_leg_q: [0.0; JOINTS],
            lie_down_ready: true,
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

    /// The hardware missed a reply (counted; rl_deploy raises no fault for it).
    pub fn note_stale(&mut self) {
        self.counters.stale_ticks += 1;
        self.stale_run += 1;
    }

    pub fn note_fresh(&mut self) {
        self.stale_run = 0;
    }

    /// Recomputed every tick like rl_deploy's SafeController (a fault clears when its cause does).
    fn check_faults(&mut self, s: &RobotState, cmd: &UserCommand) -> Option<String> {
        // rl_deploy's SafeController faults on the driver status words and temperatures, the joint data and the tilt
        // only (IMU staleness and NaN set status bits it publishes, not the fault byte), from its first tick
        let drivers = self.driver_watch.check(s, &self.cfg.robot.joint_names);
        let joints = self.joint_watch.check(s, &self.cfg.robot.joint_names);
        if !self.fault_checks {
            return None;
        }
        drivers.or(joints).or_else(|| self.tilt_fault(s, cmd))
    }

    /// rl_deploy's tilt fault (SafeController::Run, 0x183980, every 1 ms, no debounce): |roll| or |pitch| of the IMU's
    /// own angles (f32 rad, as HandlerIMU converts them; else from the orientation) above a limit that depends on the
    /// target state and gait in its UserCommand: idle (0) 0.349066 rad, stand-up (1) 0.7854, rl_control (0x11) roll
    /// 0.7854 and pitch 1.134464 (1.5708 on 0x1002), none on 0x2xxx (Extre) and 0xf003, none in any other state
    /// (joint damping, lie-down). Its target is the operator's, except that joint damping replaces it (rl_deploy's
    /// states write 2 on a fault) and rl_control's gait is the one rl_control runs (stand-ins 0xf001).
    fn tilt_fault(&self, s: &RobotState, cmd: &UserCommand) -> Option<String> {
        let (roll, pitch) = imu_roll_pitch(s);
        let target = if self.state == State::JointDamping {
            Target::JointDamping
        } else {
            self.blocked_target
                .filter(|b| *b == cmd.target)
                .map_or(cmd.target, |_| Target::Passive)
        };
        let gait = if self.state == State::RlControl {
            if self.stand_in.is_some() {
                0xf001
            } else {
                self.gait
            }
        } else {
            cmd.gait
        };
        tilt_fault(target, gait, roll, pitch)
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

    /// rl_deploy's GetNextStateName per state (fsm.c 0x164898..0x165600).
    fn next_state(&self, target: Target, gait: u32) -> State {
        let st = &self.cfg.states;
        let t = match self.state {
            State::StandUp | State::LieDown => self.spline_time(),
            _ => self.state_time,
        };
        // a fault sends every state but Idle to JointDamping (rl_deploy's IdleState stays, its target set to 0)
        if self.state != State::JointDamping
            && (target == Target::JointDamping
                || (self.fault.is_some() && self.state != State::Passive))
        {
            return State::JointDamping;
        }
        match self.state {
            // IdleState: RL control at once on 0xf003 (aligning the body posture), a stand 2 s after start-up
            State::Passive | State::StartDamping => {
                let ready = self.time >= st.passive.min_s_before_stand && self.fault.is_none();
                if self.fault.is_none() && target == Target::RlControl && gait == ALIGN_GAIT {
                    State::RlControl
                } else if ready && target == Target::Stand {
                    State::StandUp
                } else {
                    self.state
                }
            }
            // StandUpState: after both phases, RL control or a lie-down; any other target keeps standing
            State::StandUp => match target {
                Target::RlControl if t > 2.0 * st.stand_up.phase_s => State::RlControl,
                Target::LieDown if t > 2.0 * st.stand_up.phase_s => State::LieDown,
                _ => State::StandUp,
            },
            // RLControlState: only a lie-down leaves it (not during a stand-in)
            State::RlControl => match target {
                Target::LieDown
                    if self.stand_in.as_ref().is_none_or(|si| si.bridge) && self.lie_down_ready =>
                {
                    State::LieDown
                }
                _ => State::RlControl,
            },
            // LieDownState: a stand after both phases
            State::LieDown => match target {
                Target::Stand if t > 2.0 * st.lie_down.phase_s => State::StandUp,
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
            self.seen_gait = (cmd.gait, cmd.gait_seq);
            self.gait = if rl.entry_gaits.contains(&cmd.gait) {
                cmd.gait
            } else {
                rl.entry_fallback_gait
            };
            self.runner = None;
            self.blend_from = None;
            self.entry_pending = true;
            self.targets = None;
            self.targets_runner = None;
            self.target_gait = self.gait;
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
                bridge: false,
            }
        });
    }

    /// safe_switch (ours): a bridged switch from runner `from` runs the bridge stand policy first.
    fn start_bridge(&mut self, from: Option<&str>) {
        let Some(ss) = &self.cfg.states.rl_control.safe_switch else {
            return;
        };
        let to = self.cfg.runner_for_gait(self.gait);
        let Some(from) = from else {
            return;
        };
        if from != ss.bridge.runner && ss.bridged(from, to) {
            self.stand_in = Some(ActiveStandIn {
                spec: ss.bridge.clone(),
                duration: ss.bridge.first_s,
                elapsed: 0.0,
                bridge: true,
            });
        }
    }

    pub fn tick(
        &mut self,
        s: &RobotState,
        cmd: &UserCommand,
        policy: &mut dyn PolicyService,
    ) -> JointCommand {
        let finite;
        let cmd = if self.cfg.safety.finite_commands {
            finite = finite_command(cmd);
            &finite
        } else {
            cmd
        };
        let fault = self.check_faults(s, cmd);
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
        self.last_accel = s.accel;
        self.last_q = s.q;
        self.last_roll_pitch = imu_roll_pitch(s).into();
        let target = self.effective_target(cmd);
        // safe_switch (ours): a lie-down from rl_control stops the robot first and lies once it is still
        let lie_wait = self.state == State::RlControl
            && target == Target::LieDown
            && self
                .cfg
                .states
                .rl_control
                .safe_switch
                .as_ref()
                .is_some_and(|ss| ss.lie_down_when_still);
        self.lie_down_ready = !lie_wait || self.still_enough(s);
        let stopped;
        let cmd = if lie_wait {
            let mut c = cmd.clone();
            c.axes = [0.0; 3];
            c.velocity = None;
            stopped = c;
            &stopped
        } else {
            cmd
        };
        let next = self.next_state(target, cmd.gait);
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

    /// MotionInfo's height for the last tick (rl_deploy's GetBodyHeight).
    pub fn motion_info_height(&self) -> f32 {
        let [roll, pitch] = self.last_roll_pitch;
        motion_info_height(&self.cfg.robot, &self.last_q, roll, pitch)
    }

    /// Lying down is over (the lie-down's spline and damping are done), or the robot is not held at all.
    pub fn at_rest(&self) -> bool {
        match self.state {
            State::LieDown => self.spline_time() > 2.0 * self.cfg.states.lie_down.phase_s,
            State::StandUp | State::RlControl => false,
            _ => true,
        }
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

    /// Seconds in the state for the stand-up / lie-down splines and limits: with f32_state_clock an f32, rl_deploy's
    /// f32(stamp) - f32(entry stamp) (f32_state_clock_from_start) or f32(stamp - entry stamp); else control ticks.
    fn spline_time(&self) -> f64 {
        if self.cfg.timing.f32_state_clock && self.cfg.timing.f32_state_clock_from_start {
            (self.stamp as f32 - self.enter_stamp as f32) as f64
        } else if self.cfg.timing.f32_state_clock {
            (self.stamp - self.enter_stamp) as f32 as f64
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
        for (i, &qi) in q.iter().enumerate() {
            c.q[i] = qi as f64;
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

    /// The normalized axes: cmd.axes, or a metric cmd.velocity as HandlerVel normalizes it for the commanded gait.
    fn axes_for(&self, _runner: &str, cmd: &UserCommand) -> [f64; 3] {
        match cmd.velocity {
            Some(v) => self.cfg.real_steer.axes(cmd.gait, v),
            None => cmd.axes,
        }
    }

    /// RLControlState::Run on basic_flat (0x179d30..): a hot standstill rests on the 0xf001 stand-in once per stop.
    fn overheat_rest(&mut self, s: &RobotState, cmd: &UserCommand) {
        let o = self.cfg.states.rl_control.overheat.clone();
        let Some(d) = s.drivers.as_ref() else {
            return;
        };
        let resting = self.rest == 1 && self.stand_in.is_some();
        if self.rest == 1 && self.stand_in.is_none() {
            self.rest = 3;
        }
        if self.gait != GAIT_BASIC_FLAT && !resting {
            return;
        }
        let runner = self.runner.clone().unwrap_or_default();
        let speed: f64 = self.axes_for(&runner, cmd).iter().map(|a| a.abs()).sum();
        let motor = d
            .motor_temperature_c
            .iter()
            .fold(f32::MIN, |m, t| m.max(*t)) as f64;
        let driver = d
            .driver_temperature_c
            .iter()
            .fold(f32::MIN, |m, t| m.max(*t)) as f64;
        if speed >= self.cfg.states.rl_control.switch_below_speed {
            self.rest = 0;
            self.rest_ticks = 0;
            self.rest_hot = false;
        } else if resting
            && !self.rest_hot
            && (motor >= o.abort_motor_c || driver >= o.abort_driver_c)
        {
            self.rest = 0;
            self.rest_ticks = 0;
            self.rest_hot = true;
            self.stand_in = None;
            self.gait = 0xf001;
            self.target_gait = 0xf001;
            return;
        }
        let body = self
            .targets
            .as_ref()
            .and_then(|t| t.velocity_estimate)
            .map_or(0.0, |v| v[0].abs());
        if speed < self.cfg.states.rl_control.switch_below_speed
            && body < o.still_body_speed
            && self.rest == 0
        {
            self.rest_ticks += 1;
            if self.rest_ticks >= o.still_ticks && (motor > o.rest_c || driver > o.rest_c) {
                self.rest = 1;
                self.rest_ticks = 0;
                self.start_stand_in(GAIT_BASIC_FLAT, true);
            }
            // its counter wraps back to half past 9999
            if self.rest_ticks > 10 * o.still_ticks - 1 {
                self.rest_ticks = 5 * o.still_ticks;
            }
        }
    }

    /// safe_switch (ours): the body is still enough for a gait switch or a lie-down (always without safe_switch).
    fn still_enough(&self, s: &RobotState) -> bool {
        let Some(ss) = &self.cfg.states.rl_control.safe_switch else {
            return true;
        };
        let st = &ss.still;
        // the wheels' mean rim speed: the body rolling (step_move's stepping in place spins them either way)
        let rim = (3..JOINTS).step_by(4).map(|i| s.dq[i]).sum::<f64>() / 4.0 * st.wheel_radius_m;
        let wheels = rim.abs() < st.wheel_m_s;
        let rate = s.gyro.iter().map(|g| g * g).sum::<f64>().sqrt() < st.gyro_rad_s;
        wheels && rate
    }

    fn rl_control(
        &mut self,
        s: &RobotState,
        cmd: &UserCommand,
        policy: &mut dyn PolicyService,
    ) -> JointCommand {
        let write = (cmd.gait, cmd.gait_seq) != self.seen_gait;
        self.seen_gait = (cmd.gait, cmd.gait_seq);
        let keep = self
            .cfg
            .states
            .rl_control
            .safe_switch
            .as_ref()
            .is_some_and(|ss| ss.keep_gait_during_stand_in);
        // ours: a gait written during a bridge (or, with keep_gait_during_stand_in, any stand-in) waits for its end
        if write && self.stand_in.as_ref().is_some_and(|si| si.bridge || keep) {
            self.target_gait = cmd.gait;
        }
        self.overheat_rest(s, cmd);
        let (wanted, params) = match &mut self.stand_in {
            Some(si) => {
                // runs as gait 0xf001, written over the gait slot: a gait sent meanwhile is lost (update_policy_ptr)
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
                if write {
                    self.target_gait = cmd.gait;
                }
                // update_policy_ptr switches only while |vx| + |vy| + |wz| < 0.01 (normalized); else the gait waits
                let runner = self.runner.clone().unwrap_or_default();
                let speed: f64 = self.axes_for(&runner, cmd).iter().map(|a| a.abs()).sum();
                if self.target_gait != self.gait
                    && speed < self.cfg.states.rl_control.switch_below_speed
                    && self.still_enough(s)
                {
                    self.gait = self.target_gait;
                    self.start_stand_in(self.gait, false);
                    if self.stand_in.is_none() {
                        let from = self.runner.clone();
                        self.start_bridge(from.as_deref());
                    }
                }
                // a forced runner the config does not have is ignored (the gait's runner runs)
                let name = cmd
                    .runner
                    .clone()
                    .filter(|r| self.cfg.runners.contains_key(r))
                    .unwrap_or_else(|| self.cfg.runner_for_gait(self.gait).to_string());
                let mut params = cmd.params.clone();
                params.insert("gait".into(), self.gait as f64);
                match &mut self.stand_in {
                    // a bridge started this tick runs from this tick (ours; no tick of the gait's runner first)
                    Some(si) if si.bridge => {
                        let mut params = si.params().clone();
                        params.insert("gait".into(), 0xf001 as f64);
                        si.elapsed += self.dt;
                        (si.spec.runner.clone(), params)
                    }
                    _ => (name, params),
                }
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
                        // safe_switch (ours): a new runner's first targets blend in from the last command
                        let blend = self
                            .cfg
                            .states
                            .rl_control
                            .safe_switch
                            .as_ref()
                            .map_or(0.0, |ss| ss.blend_s);
                        if blend > 0.0
                            && self
                                .targets_runner
                                .as_ref()
                                .is_some_and(|r| *r != result.runner)
                        {
                            self.blend_from = Some((self.last_leg_q, 0.0));
                        }
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
                if let Some((from, elapsed)) = &mut self.blend_from {
                    let total = self
                        .cfg
                        .states
                        .rl_control
                        .safe_switch
                        .as_ref()
                        .map_or(0.0, |ss| ss.blend_s);
                    let k = if total > 0.0 {
                        (*elapsed / total).min(1.0)
                    } else {
                        1.0
                    };
                    for i in (0..JOINTS).filter(|i| i % 4 != 3) {
                        c.q[i] = from[i] + (c.q[i] - from[i]) * k;
                    }
                    *elapsed += self.dt;
                    if k >= 1.0 {
                        self.blend_from = None;
                    }
                }
            }
            // until the first policy step lands: hold the pose rl_control started from
            None => c.q = self.enter_q,
        }
        for i in (3..JOINTS).step_by(4) {
            c.kp[i] = 0.0;
            c.q[i] = 0.0;
        }
        self.last_leg_q = c.q;
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy::{InlinePolicy, RunnerSet};

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

    /// Answers every request with the pose held (no networks), so rl_control runs without policies.
    struct HoldPolicy(Option<PolicyRequest>);

    impl PolicyService for HoldPolicy {
        fn offer(&mut self, request: PolicyRequest) -> bool {
            self.0 = Some(request);
            true
        }
        fn take_result(&mut self) -> Option<crate::policy::PolicyResult> {
            self.0.take().map(|r| crate::policy::PolicyResult {
                runner: r.runner,
                seq: r.seq,
                targets: Ok(Targets {
                    q: r.state.q,
                    wheel_velocity: [0.0; 4],
                    action: [0.0; JOINTS],
                    velocity_estimate: None,
                }),
                gain_scale: 1.0,
                inference_s: 0.0,
            })
        }
        fn dropped(&self) -> u64 {
            0
        }
    }

    /// rl_control entered on 0x1001, run `ticks` with `cmd`, varying the joints so no frozen-motor fault.
    fn run(
        c: &mut Controller,
        s: &mut RobotState,
        cmd: &UserCommand,
        ticks: usize,
        p: &mut HoldPolicy,
    ) {
        for _ in 0..ticks {
            s.stamp += 0.001;
            s.tau = [s.stamp; JOINTS];
            c.tick(s, cmd, p);
        }
    }

    /// The longest run of ticks a stand-up and a lie-down hold one target, the bus clock starting at `uptime_s`.
    fn longest_hold(from_start: bool, uptime_s: f64) -> usize {
        let mut cfg = config();
        cfg.timing.f32_state_clock_from_start = from_start;
        let mut s = standing_state(&cfg);
        s.stamp = uptime_s;
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let mut p = HoldPolicy(None);
        let mut longest = 0;
        for (state, target, ticks) in [
            (State::StandUp, Target::Stand, 2900),
            (State::LieDown, Target::LieDown, 1900),
        ] {
            let cmd = UserCommand {
                target,
                ..Default::default()
            };
            c.force(state, &s, &cmd, false);
            let (mut last, mut run) = (None, 0);
            for k in 0..ticks {
                s.stamp += 0.001;
                // a torque that changes each tick (no frozen-motor fault)
                s.tau = [k as f64 * 1e-3; JOINTS];
                let q = c.tick(&s, &cmd, &mut p).q;
                assert_eq!(c.state, state, "{:?}", c.fault);
                run = if Some(q) == last { run + 1 } else { 1 };
                longest = longest.max(run);
                last = Some(q);
            }
        }
        longest
    }

    #[test]
    fn the_splines_do_not_coarsen_with_uptime() {
        // rl_deploy up 11.2 days (2026-11-17): its f32 seconds-since-start step every 1/16 s
        let uptime = 11.2 * 86400.0;
        assert!(
            longest_hold(true, uptime) >= 60,
            "the vendor clock's 62.5 ms steps"
        );
        // ours counts from the state's entry: a new target every tick, as fresh
        assert_eq!(longest_hold(false, uptime), longest_hold(false, 0.0));
        assert!(longest_hold(false, uptime) <= 2);
    }

    #[test]
    fn a_gait_sent_during_the_stand_in_is_lost_and_a_resend_switches() {
        let mut cfg = config();
        cfg.states.rl_control.safe_switch = None;
        let mut s = standing_state(&cfg);
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let mut p = HoldPolicy(None);
        let mut cmd = UserCommand {
            target: Target::RlControl,
            gait: 0x1001,
            ..Default::default()
        };
        c.force(State::RlControl, &s, &cmd, false);
        c.stand_in = None;
        c.start_stand_in(0x1001, true);
        run(&mut c, &mut s, &cmd, 100, &mut p);
        assert!(c.in_stand_in());
        // M20Connection's first 0x3003, while the stand-in runs
        cmd.gait = 0x3003;
        cmd.gait_seq = 1;
        run(&mut c, &mut s, &cmd, 4000, &mut p);
        assert!(!c.in_stand_in());
        assert_eq!(c.gait(), 0x1001, "lost: basic_flat after the stand-in");
        assert_eq!(c.current_runner(), Some("basic_flat"));
        // its resend of the same code switches
        cmd.gait_seq = 2;
        run(&mut c, &mut s, &cmd, 100, &mut p);
        assert_eq!(c.gait(), 0x3003);
        assert_eq!(c.current_runner(), Some("basic_stair"));
    }

    #[test]
    fn a_gait_switch_waits_until_the_command_is_still() {
        let cfg = config();
        let mut s = standing_state(&cfg);
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let mut p = HoldPolicy(None);
        let mut cmd = UserCommand {
            target: Target::RlControl,
            gait: 0x3003,
            axes: [0.3, 0.0, 0.0],
            ..Default::default()
        };
        c.force(State::RlControl, &s, &cmd, false);
        c.stand_in = None;
        c.gait = 0x3003;
        c.target_gait = 0x3003;
        run(&mut c, &mut s, &cmd, 100, &mut p);
        cmd.gait = 0x3002;
        cmd.gait_seq = 1;
        run(&mut c, &mut s, &cmd, 500, &mut p);
        assert_eq!(c.gait(), 0x3003, "moving: the switch waits");
        cmd.axes = [0.008, 0.0, -0.004];
        run(&mut c, &mut s, &cmd, 1, &mut p);
        assert_eq!(c.gait(), 0x3003, "0.012: still moving");
        cmd.axes = [0.005, 0.0, -0.004];
        run(&mut c, &mut s, &cmd, 1, &mut p);
        assert_eq!(c.gait(), 0x3002);
    }

    #[test]
    fn a_hot_standstill_on_basic_flat_rests_once_until_moving() {
        let cfg = config();
        let mut s = standing_state(&cfg);
        let mut d = crate::types::Drivers {
            status_word: [1; JOINTS],
            motor_temperature_c: [40.0; JOINTS],
            driver_temperature_c: [40.0; JOINTS],
        };
        s.drivers = Some(d);
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let mut p = HoldPolicy(None);
        let still = UserCommand {
            target: Target::RlControl,
            gait: GAIT_BASIC_FLAT,
            ..Default::default()
        };
        c.force(State::RlControl, &s, &still, false);
        c.stand_ins_run.insert(GAIT_BASIC_FLAT);
        run(&mut c, &mut s, &still, 2000, &mut p);
        assert!(!c.in_stand_in(), "cool: no rest");
        d.motor_temperature_c[5] = 75.0;
        s.drivers = Some(d);
        // the still count already passed 1000 while cool: the first hot tick rests
        run(&mut c, &mut s, &still, 1, &mut p);
        assert!(c.in_stand_in());
        // 1.5 s + 0.5 s, then basic_flat, and no second rest while still
        run(&mut c, &mut s, &still, 2100, &mut p);
        assert!(!c.in_stand_in());
        assert_eq!(c.current_runner(), Some("basic_flat"));
        run(&mut c, &mut s, &still, 3000, &mut p);
        assert!(!c.in_stand_in(), "rested once per stop");
        // moving re-arms it
        let mut walk = still.clone();
        walk.axes = [0.3, 0.0, 0.0];
        run(&mut c, &mut s, &walk, 10, &mut p);
        run(&mut c, &mut s, &still, 1001, &mut p);
        assert!(c.in_stand_in());
        // a motor at 100 during the rest ends it on 0xf001
        d.motor_temperature_c[5] = 100.0;
        s.drivers = Some(d);
        run(&mut c, &mut s, &still, 1, &mut p);
        assert!(!c.in_stand_in());
        assert_eq!(c.gait(), 0xf001);
    }

    /// The vendor's MotionInfo height on the robot (m20tool status, 11-24): 0.46 m walking on 0x3003, 0.10 m lying
    /// docked; ours from basic_stair's measured stance and the lie-down pose.
    #[test]
    fn motion_info_height_is_get_body_height_s() {
        let cfg = config();
        // basic_stair's stance on the 11-20 capture: hip 0.40-0.41 m above the axle (mean leg FK of the measured joints)
        let stair = leg_pose_for_height(&cfg.robot, 0.405, -0.1);
        let standing = motion_info_height(&cfg.robot, &stair, 0.0, 0.0);
        // the robot read 0.46 walking at 0.69 m/s; standing on the 11-20 capture's basic_stair joints this formula is 0.48
        assert!((standing - 0.47).abs() < 0.015, "standing {standing}");
        let lying = leg_pose_for_height(&cfg.robot, cfg.states.lie_down.lie_height, 0.0);
        let lying = motion_info_height(&cfg.robot, &lying, 0.0, 0.0);
        assert!((lying - 0.10).abs() < 0.02, "lying {lying}");
        // a body pitched 20 deg forward: the contacts' height changes, ours from the joints alone would not
        let pitched = motion_info_height(&cfg.robot, &stair, 0.0, 20f32.to_radians());
        assert!((pitched - standing).abs() > 0.01);
    }

    /// rl_deploy's GetNextStateName (fsm.c): which targets each state takes.
    #[test]
    fn transitions_are_rl_deploy_s() {
        let cfg = config();
        let mut s = standing_state(&cfg);
        let mut p = HoldPolicy(None);
        let cmd = |target, gait| UserCommand {
            target,
            gait,
            ..Default::default()
        };
        // Idle: RL control does not stand the robot; with 0xf003 it enters RL control at once
        let mut c = Controller::new(cfg.clone());
        c.set_time(10.0);
        run(&mut c, &mut s, &cmd(Target::RlControl, 0x1001), 10, &mut p);
        assert_eq!(c.state, State::Passive);
        run(
            &mut c,
            &mut s,
            &cmd(Target::RlControl, ALIGN_GAIT),
            1,
            &mut p,
        );
        assert_eq!(c.state, State::RlControl);
        // RL control: a passive target changes nothing, a lie-down lies down
        run(
            &mut c,
            &mut s,
            &cmd(Target::Passive, ALIGN_GAIT),
            100,
            &mut p,
        );
        assert_eq!(c.state, State::RlControl);
        run(&mut c, &mut s, &cmd(Target::LieDown, ALIGN_GAIT), 1, &mut p);
        assert_eq!(c.state, State::LieDown);
        // lying: only a stand stands it up (after 4 s)
        run(
            &mut c,
            &mut s,
            &cmd(Target::RlControl, 0x1001),
            5000,
            &mut p,
        );
        assert_eq!(c.state, State::LieDown);
        run(&mut c, &mut s, &cmd(Target::Stand, 0x1001), 1, &mut p);
        assert_eq!(c.state, State::StandUp);
        // standing up: passive keeps standing
        run(&mut c, &mut s, &cmd(Target::Passive, 0x1001), 4000, &mut p);
        assert_eq!(c.state, State::StandUp);
    }

    #[test]
    fn joint_checks_count_like_rl_deploy() {
        let cfg = config();
        let names = cfg.robot.joint_names.clone();
        let base = standing_state(&cfg);
        let mut w = JointWatch::new(&cfg.robot);
        let vary = |k: usize| {
            let mut s = base.clone();
            s.dq[0] = k as f64 * 1e-3;
            s.dq[3] = k as f64 * 1e-3;
            for i in 0..JOINTS {
                s.tau[i] = (k % 7) as f64 * 0.01 + i as f64;
                s.dq[i] += (k % 5) as f64 * 1e-4;
            }
            s
        };
        // hipx FL range [-0.4363, 0.6109]: 0.6109 + 0.24 + 0.001 is past it; 30 readings pass, the 31st faults
        for k in 0..31 {
            let mut s = vary(k);
            s.q[0] = 0.6109 + 0.241;
            let f = w.check(&s, &names);
            assert_eq!(f.is_some(), k == 30, "reading {k}: {f:?}");
        }
        assert!(
            w.check(&vary(40), &names).is_none(),
            "back in range clears it"
        );
        // just inside the margin never faults
        for k in 0..40 {
            let mut s = vary(k + 100);
            s.q[0] = 0.6109 + 0.239;
            assert!(w.check(&s, &names).is_none());
        }
        // NaN: the 31st in a row
        for k in 0..31 {
            let mut s = vary(k + 200);
            s.tau[6] = f64::NAN;
            let f = w.check(&s, &names);
            assert_eq!(f.is_some(), k == 30, "NaN reading {k}: {f:?}");
        }
        // frozen: the 51st identical reading after the first
        let s = vary(301);
        assert!(w.check(&s, &names).is_none());
        for k in 1..=51 {
            let f = w.check(&s, &names);
            assert_eq!(f.is_some(), k == 51, "frozen reading {k}: {f:?}");
        }
    }

    #[test]
    fn driver_faults_are_rl_deploy_s() {
        let cfg = config();
        let names = cfg.robot.joint_names.clone();
        let mut w = DriverWatch::default();
        let mut s = standing_state(&cfg);
        assert!(w.check(&s, &names).is_none(), "no driver data, no fault");
        let mut d = crate::types::Drivers {
            status_word: [1; JOINTS],
            motor_temperature_c: [40.0; JOINTS],
            driver_temperature_c: [40.0; JOINTS],
        };
        s.drivers = Some(d);
        assert!(w.check(&s, &names).is_none());
        d.status_word[5] = 0x21;
        s.drivers = Some(d);
        assert!(w
            .check(&s, &names)
            .is_some_and(|f| f.contains("status word")));
        d.status_word[5] = 1;
        // a motor at 120: the fault comes at the next 1000-tick check and holds until it is below 100
        d.motor_temperature_c[2] = 120.0;
        s.drivers = Some(d);
        let first = (0..1000).position(|_| w.check(&s, &names).is_some());
        assert!(first.is_some_and(|k| k < 1000));
        d.motor_temperature_c[2] = 100.0;
        s.drivers = Some(d);
        assert!(
            (0..2000).all(|_| w.check(&s, &names).is_some()),
            "100..120 stays latched"
        );
        d.motor_temperature_c[2] = 99.9;
        s.drivers = Some(d);
        assert!((0..1000).any(|_| w.check(&s, &names).is_none()));
        // drivers: 100 latches, below 80 clears
        d.driver_temperature_c[7] = 100.0;
        s.drivers = Some(d);
        assert!((0..1000).any(|_| w
            .check(&s, &names)
            .is_some_and(|f| f.contains("driver overheated"))));
    }

    #[test]
    fn tilt_limits_are_rl_deploy_s() {
        let deg = |d: f64| d.to_radians() as f32;
        assert!(tilt_fault(Target::Passive, 0, deg(19.9), 0.0).is_none());
        assert!(tilt_fault(Target::Passive, 0, 0.0, deg(20.1)).is_some());
        assert!(tilt_fault(Target::Stand, 0, deg(44.9), deg(44.9)).is_none());
        assert!(tilt_fault(Target::Stand, 0, deg(45.1), 0.0).is_some());
        assert!(tilt_fault(Target::RlControl, 0x1001, deg(44.9), deg(64.9)).is_none());
        assert!(tilt_fault(Target::RlControl, 0x1001, 0.0, deg(65.1)).is_some());
        assert!(tilt_fault(Target::RlControl, 0x1002, 0.0, deg(89.9)).is_none());
        assert!(tilt_fault(Target::RlControl, 0x1002, deg(45.1), 0.0).is_some());
        assert!(tilt_fault(Target::RlControl, 0x2001, deg(80.0), deg(80.0)).is_none());
        assert!(tilt_fault(Target::RlControl, 0xf003, deg(80.0), deg(80.0)).is_none());
        assert!(tilt_fault(Target::LieDown, 0, deg(80.0), deg(80.0)).is_none());
        assert!(tilt_fault(Target::JointDamping, 0, deg(80.0), deg(80.0)).is_none());
    }

    #[test]
    fn non_finite_commands_are_zero_and_axes_clipped() {
        let mut cmd = UserCommand {
            axes: [f64::NAN, 5.0, f64::NEG_INFINITY],
            velocity: Some([f64::INFINITY, 0.3, f64::NAN]),
            ..Default::default()
        };
        cmd.params.insert("height".into(), f64::NAN);
        cmd.params.insert("roll".into(), 0.5);
        let c = finite_command(&cmd);
        assert_eq!(c.axes, [0.0, 1.0, 0.0]);
        assert_eq!(c.velocity, Some([0.0, 0.3, 0.0]));
        assert!(!c.params.contains_key("height"));
        assert_eq!(c.params["roll"], 0.5);
    }

    #[test]
    fn safe_switch_keeps_a_gait_sent_during_the_stand_in() {
        let cfg = config();
        let mut s = standing_state(&cfg);
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let mut p = HoldPolicy(None);
        let mut cmd = UserCommand {
            target: Target::RlControl,
            gait: 0x1001,
            ..Default::default()
        };
        c.force(State::RlControl, &s, &cmd, false);
        c.stand_in = None;
        c.start_stand_in(0x1001, true);
        run(&mut c, &mut s, &cmd, 100, &mut p);
        cmd.gait = 0x3003;
        cmd.gait_seq += 1;
        run(&mut c, &mut s, &cmd, 4000, &mut p);
        assert!(!c.in_stand_in());
        assert_eq!(c.current_runner(), Some("basic_stair"));
    }

    #[test]
    fn safe_switch_bridges_the_listed_pairs_and_waits_for_a_still_body() {
        let cfg = config();
        let ss = cfg
            .states
            .rl_control
            .safe_switch
            .clone()
            .expect("safe_switch on");
        assert!(ss.bridged("basic_flat", "navi_flat"));
        assert!(ss.bridged("navi_flat", "basic_backknee"));
        assert!(ss.bridged("basic_backknee", "extre"));
        assert!(!ss.bridged("navi_flat", "basic_flat"));
        assert!(!ss.bridged("basic_stair", "navi_flat"));
        // a lie-down while rolling: the command is zeroed and the lie waits for the wheels to stop
        let mut s = standing_state(&cfg);
        let mut c = Controller::new(cfg.clone());
        c.set_time(10.0);
        let mut p = HoldPolicy(None);
        let mut cmd = UserCommand {
            target: Target::RlControl,
            gait: 0x3002,
            ..Default::default()
        };
        c.force(State::RlControl, &s, &cmd, false);
        c.stand_in = None;
        run(&mut c, &mut s, &cmd, 100, &mut p);
        let rolling = ss.still.wheel_m_s / ss.still.wheel_radius_m * 2.0;
        for i in (3..JOINTS).step_by(4) {
            s.dq[i] = rolling;
        }
        cmd.target = Target::LieDown;
        run(&mut c, &mut s, &cmd, 100, &mut p);
        assert_eq!(c.state, State::RlControl, "still rolling");
        // wheels spinning against each other are not rolling (step_move stepping in place)
        for (k, i) in (3..JOINTS).step_by(4).enumerate() {
            s.dq[i] = if k % 2 == 0 { rolling } else { -rolling };
        }
        run(&mut c, &mut s, &cmd, 1, &mut p);
        assert_eq!(c.state, State::LieDown);
        // a bridged switch runs the stand policy from its first tick, then the gait's runner
        let mut c = Controller::new(cfg.clone());
        c.set_time(10.0);
        let mut cmd = UserCommand {
            target: Target::RlControl,
            gait: 0x1001,
            ..Default::default()
        };
        c.force(State::RlControl, &s, &cmd, false);
        c.stand_in = None;
        run(&mut c, &mut s, &cmd, 100, &mut p);
        assert_eq!(c.current_runner(), Some("basic_flat"));
        cmd.gait = 0x3002;
        cmd.gait_seq += 1;
        run(&mut c, &mut s, &cmd, 1, &mut p);
        assert_eq!(c.current_runner(), Some(ss.bridge.runner.as_str()));
        run(
            &mut c,
            &mut s,
            &cmd,
            (ss.bridge.first_s * 1000.0) as usize + 1,
            &mut p,
        );
        assert_eq!(c.current_runner(), Some("navi_flat"));
    }
}
