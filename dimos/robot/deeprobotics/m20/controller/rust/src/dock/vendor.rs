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

//! DeepRobotics charge_manager v4.0.30 and reflective_column_node (NOS /opt/robot/share/charge_manager/bin) as their
//! decompiled code does it:
//!
//! - [`ChargeManagerToml`]: the vendor's own config file, read as `ChargeManager::GetParamValue` reads it, and
//!   [`ChargeParams`], the numbers it derives from it (f32, in its units).
//! - [`ChargeCore`]: `goToPileAction` / `exitPileAction`'s work per dock pose, in f32 with its fused multiply-adds, so
//!   its steer matches the vendor's logged `sendX/sendY/sendYaw/sendZ` to the float. Everything that blocks in the
//!   vendor (gait switches, sleeps, lying down, the charge check) is the caller's ([`super::docking`]).
//! - [`ReflectorFilter`]: what reflective_column_node does to its per-sweep pose before publishing it (range gate,
//!   15-sweep start filter, 0.8 m jump gate).

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};

use super::config::{FilterConfig, SequenceConfig};

/// The vendor's file on NOS.
pub const CHARGE_MANAGER_TOML: &str = "/opt/robot/share/charge_manager/config/charge_manager.toml";

/// charge_manager.toml as charge_manager reads it: every key it reads, in the type it reads it as (`toml::find_or<int>`
/// for the mm / deg / "/1000" ones). A missing key is refused (the vendor falls back to a compiled-in default).
#[derive(Debug, Clone, Deserialize)]
#[allow(non_snake_case)]
pub struct ChargeManagerToml {
    pub adjust_posture_times: i32,
    pub first_detection_current_timeout: i32,
    pub tag_receive_timeout: i32,
    pub delta_x: i32,
    pub delta_y: i32,
    pub delta_yaw: i32,
    pub ext_delta_x: i32,
    pub ext_delta_y: i32,
    pub ext_delta_yaw: i32,
    pub ext_min_speed_x: i32,
    pub ext_min_speed_y: i32,
    pub ext_min_speed_yaw: i32,
    pub min_speed_x: i32,
    pub min_speed_y: i32,
    pub min_speed_yaw: i32,
    pub pre_point_x: f32,
    pub pre_exitPile_point_x: f32,
    pub back_way: String,
    pub exit_pile_distance_x: f32,
    pub NoChargePlug_way: String,
    pub RetryMethod_InPlace: bool,
    pub current_detection: bool,
    pub mos_detection: bool,
    pub all_battery_charge: bool,
    pub DistanceCompensationX: f32,
    pub ObstacleAvoidDis: f32,
    pub ObstacleAvoid_enable: bool,
    // ObstacleDetection::LoadParametersFromToml (doubles; lidar_z_min is read but replaced per cloud)
    pub mOD_hz: f64,
    pub lidar_z_max: f64,
    pub lidar_x_min: f64,
    pub lidar_y_min: f64,
    pub lidar_x_max: f64,
    pub lidar_y_max: f64,
    pub lidar_yaw: f64,
    pub charge_pile_h: f64,
    pub obstacle_count_method: String,
    pub cluster_count_distance: f64,
    pub cluster_points_count_min: i32,
    pub grid_size: f64,
    pub grid_occu_num: i32,
}

impl ChargeManagerToml {
    pub fn parse(text: &str) -> Result<ChargeManagerToml> {
        let t: ChargeManagerToml = toml::from_str(text).context("parsing charge_manager.toml")?;
        // Start compares NoChargePlug_way with these two; anything else leaves a failed dock in goToPile for good
        if !matches!(t.NoChargePlug_way.as_str(), "goBack" | "sitDown") {
            bail!(
                "charge_manager.toml NoChargePlug_way must be goBack or sitDown, is {:?}",
                t.NoChargePlug_way
            );
        }
        // ProcessLatestCloud falls back to cluster on any other name; only grid is implemented here
        if t.obstacle_count_method != "grid" {
            bail!(
                "charge_manager.toml obstacle_count_method {:?}: only grid (the vendor's setting) is implemented",
                t.obstacle_count_method
            );
        }
        if !(t.mos_detection || t.current_detection) {
            bail!("charge_manager.toml: mos_detection or current_detection must be on (else nothing confirms a charge)");
        }
        if t.current_detection {
            bail!("charge_manager.toml current_detection: the pack current is not read here (the vendor's file has it off)");
        }
        if t.tag_receive_timeout <= 0
            || t.first_detection_current_timeout < 0
            || t.adjust_posture_times < 0
        {
            bail!("charge_manager.toml: tag_receive_timeout > 0, first_detection_current_timeout and adjust_posture_times >= 0");
        }
        if !(t.grid_size > 0.0) || t.grid_occu_num < 1 {
            bail!("charge_manager.toml: grid_size > 0 and grid_occu_num >= 1");
        }
        Ok(t)
    }

    pub fn load(path: &str) -> Result<(ChargeManagerToml, String)> {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {path}"))?;
        Ok((ChargeManagerToml::parse(&text)?, text))
    }
}

/// `(float)int / 1000.0`, as GetParamValue / goToPileAction scale the integer keys.
fn per(v: i32, d: f64) -> f32 {
    (v as f32 as f64 / d) as f32
}

/// What charge_manager derives from its file (f32 where it keeps f32).
#[derive(Debug, Clone, PartialEq)]
pub struct ChargeParams {
    /// adjust_posture_times: tries after the first
    pub retries: i32,
    /// first_detection_current_timeout (ms after the power-on)
    pub charge_confirm_ms: u64,
    /// tag_receive_timeout (ms): FindTarget's wait for a first pose
    pub find_target_ms: u64,
    /// `(tag_receive_timeout * 1000) / 5` 200 ms waits without a pose before the detector is restarted
    pub restart_after_waits: u32,
    /// delta_x, delta_y (mm -> m), delta_yaw (/100 -> rad)
    pub delta: [f32; 3],
    /// ext_delta_x, ext_delta_y (mm -> m), ext_delta_yaw (/100 -> rad)
    pub ext_delta: [f32; 3],
    /// min_speed_*, ext_min_speed_* (/1000, normalized steer)
    pub min_speed: [f32; 3],
    pub ext_min_speed: [f32; 3],
    pub pre_point_x: f32,
    pub pre_exit_point_x: f32,
    pub exit_pile_distance_x: f32,
    /// back_way = "fixed": undock to exit_pile_distance_x (else "auto": back to where docking started)
    pub back_fixed: bool,
    /// NoChargePlug_way = "goBack" (else "sitDown")
    pub no_charge_go_back: bool,
    pub retry_in_place: bool,
    pub all_battery_charge: bool,
    /// DistanceCompensationX (cm -> m)
    pub compensation_x: f32,
    pub obstacle_avoid: bool,
    pub obstacle_avoid_dis: f32,
}

impl ChargeParams {
    pub fn from_toml(t: &ChargeManagerToml) -> ChargeParams {
        ChargeParams {
            retries: t.adjust_posture_times,
            charge_confirm_ms: (t.first_detection_current_timeout as i64 * 1000).max(0) as u64,
            find_target_ms: t.tag_receive_timeout as u64 * 1000,
            restart_after_waits: (t.tag_receive_timeout as u32).wrapping_mul(1000) / 5,
            delta: [
                per(t.delta_x, 1000.0),
                per(t.delta_y, 1000.0),
                per(t.delta_yaw, 100.0),
            ],
            ext_delta: [
                per(t.ext_delta_x, 1000.0),
                per(t.ext_delta_y, 1000.0),
                per(t.ext_delta_yaw, 100.0),
            ],
            min_speed: [
                per(t.min_speed_x, 1000.0),
                per(t.min_speed_y, 1000.0),
                per(t.min_speed_yaw, 1000.0),
            ],
            ext_min_speed: [
                per(t.ext_min_speed_x, 1000.0),
                per(t.ext_min_speed_y, 1000.0),
                per(t.ext_min_speed_yaw, 1000.0),
            ],
            pre_point_x: t.pre_point_x,
            pre_exit_point_x: t.pre_exitPile_point_x,
            exit_pile_distance_x: t.exit_pile_distance_x,
            back_fixed: t.back_way == "fixed",
            no_charge_go_back: t.NoChargePlug_way == "goBack",
            retry_in_place: t.RetryMethod_InPlace,
            all_battery_charge: t.all_battery_charge,
            compensation_x: (t.DistanceCompensationX as f64 / 100.0) as f32,
            obstacle_avoid: t.ObstacleAvoid_enable,
            obstacle_avoid_dis: t.ObstacleAvoidDis,
        }
    }
}

/// VelocityController: the incremental PID, its f32 fused multiply-adds in the binary's order.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    u: f32,
    e1: f32,
    e2: f32,
}

impl Pid {
    pub fn new(k: [f32; 3]) -> Pid {
        Pid {
            kp: k[0],
            ki: k[1],
            kd: k[2],
            ..Default::default()
        }
    }

    pub fn reset(&mut self) {
        self.u = 0.0;
        self.e1 = 0.0;
        self.e2 = 0.0;
    }

    pub fn control(&mut self, e: f32) -> f32 {
        let (e1, e2) = (self.e1, self.e2);
        self.e2 = e1;
        self.e1 = e;
        let p = self.kp.mul_add(e - e1, self.u);
        let i = self.ki.mul_add(e, p);
        self.u = self.kd.mul_add(e2 + (-e1).mul_add(2.0, e), i);
        self.u
    }
}

/// RobotCommander::limitSpeed: clamp to +-max, and anything closer to 0 than min out to +-min (0 goes to +min).
pub fn limit_speed(v: f32, max: f32, min: f32) -> f32 {
    if v >= max {
        max
    } else if v < -max {
        -max
    } else if v.abs() < min {
        if v < 0.0 {
            -min
        } else {
            min
        }
    } else {
        v
    }
}

/// The reflective_column_node pose charge_manager keeps (x, y as published, yaw from the quaternion), as f32.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetPose {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

/// Which way the obstacle check looks (ObstacleDirection: 0 front, 1 back).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Direction {
    Front,
    Back,
}

/// A steer charge_manager writes (drdds Steer on /CHARGE_CMD: normalized x, y, yaw, and z, FinetuneStand's height).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Steer {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub z: f32,
}

/// Why a dock or undock ended (charge_manager's error codes).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum Failure {
    /// 0x1008: not on NaviCharge / FinetuneStand in rl_control
    NotNavigating,
    /// 0x1005: the approach past its limit (then BasicFlat)
    ApproachTimeout,
    /// 0x1009 / 0x100a: stopped for an obstacle past its limit (then BasicFlat)
    ObstacleTimeout,
    /// 0x1006: the undock past its limit (then BasicFlat), or a non-finite steer
    ExitTimeout,
    BadSteer,
}

/// Something one pose decided besides the steer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    /// "已到达进桩预备点!": the obstacle check looks ahead now
    StagingReached,
    /// "已到达精调点!" (x, y, yaw then): stopped, FinetuneStand next
    FineReached([f32; 3]),
    /// "已到达桩上 或桩上微调阶段超时(4s)" (x, y, yaw then): (0, 0, 0, sit_z) sent, lie down after sit_hold_ms
    FinetuneDone([f32; 3]),
    /// "检测到障碍物": stopped for an obstacle this far (m)
    Obstacle(f32),
    /// "已到达退桩预备点!"
    ExitStagingReached,
    /// "已到达指定位置!": undocked
    ExitReached,
    Failed(Failure),
}

/// One pose's commands: a steer to send and / or stopMove, and what it decided.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Outcome {
    pub stop: bool,
    pub steer: Option<Steer>,
    pub event: Option<Event>,
    /// the first poses are taken: the start point (mInit: x out from docked, y, yaw)
    pub start_point: Option<[f32; 3]>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Approach {
    /// uVar20: 1 to the staging point, 2 to the fine point, 3 FinetuneStand
    phase: u8,
    /// the start point is chosen (CVar21)
    started: bool,
    samples: u32,
    /// the candidate start point (fVar31, fVar30, fVar27)
    best: [f32; 3],
    /// local_4a0: FinetuneStand's z counter
    k: u32,
    /// local_4b0: the approach limit counts from here (ms), reset when FinetuneStand starts
    t0_ms: u64,
    /// local_4c0: since when stopped for an obstacle (0 = not)
    obstacle_since_ms: u64,
    /// local_49c: tries left
    retries_left: i32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Exit {
    staged: bool,
    t0_ms: u64,
    obstacle_since_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    Idle,
    Dock(Approach),
    Exit(Exit),
}

/// What happens after a try finds no charge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoCharge {
    /// stand (StandUp2WideWalk), retry_settle_ms, then approach again
    Retry,
    /// tries used up (0x1007)
    Exhausted,
}

/// What a 200 ms wait without a pose did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Wait {
    pub stop: bool,
    /// restart the detector and FindTarget again (restart_after_waits in a row)
    pub restart: bool,
}

/// goToPileAction / exitPileAction's state between poses.
#[derive(Debug, Clone)]
pub struct ChargeCore {
    pub params: ChargeParams,
    seq: SequenceConfig,
    pid: [Pid; 3],
    mode: Mode,
    /// mInit (0x2c0..0x2c8): kept across docks, the undock's goal with back_way = auto
    start_point: [f32; 3],
    /// waits without a pose in a row (local_4cc / uVar19)
    waits: u32,
    /// the obstacle check's direction
    pub direction: Direction,
}

impl ChargeCore {
    pub fn new(params: ChargeParams, seq: &SequenceConfig) -> ChargeCore {
        ChargeCore {
            params,
            pid: [
                Pid::new(seq.pid_x),
                Pid::new(seq.pid_y),
                Pid::new(seq.pid_yaw),
            ],
            seq: seq.clone(),
            mode: Mode::Idle,
            start_point: [0.0; 3],
            waits: 0,
            direction: Direction::Front,
        }
    }

    pub fn start_point(&self) -> [f32; 3] {
        self.start_point
    }

    /// Keep a start point across a config reload.
    pub fn set_start_point(&mut self, p: [f32; 3]) {
        self.start_point = p;
    }

    fn reset_pids(&mut self) {
        for p in &mut self.pid {
            p.reset();
        }
    }

    /// goToPileAction's start (before WideWalk): the approach limit counts from `now_ms`.
    pub fn begin_dock(&mut self, now_ms: u64) {
        self.reset_pids();
        self.waits = 0;
        self.direction = Direction::Front;
        self.mode = Mode::Dock(Approach {
            phase: 1,
            started: false,
            samples: 0,
            best: [100.0; 3],
            k: 2,
            t0_ms: now_ms,
            obstacle_since_ms: 0,
            retries_left: self.params.retries,
        });
    }

    /// setFineTuneStandMode returned (FinetuneStand is up): its limits count from `now_ms`.
    pub fn finetune_started(&mut self, now_ms: u64) {
        if let Mode::Dock(a) = &mut self.mode {
            a.t0_ms = now_ms;
        }
    }

    /// No charge after lying down (or the lie-down failed).
    pub fn no_charge(&mut self) -> NoCharge {
        let Mode::Dock(a) = &mut self.mode else {
            return NoCharge::Exhausted;
        };
        if a.retries_left < 1 {
            return NoCharge::Exhausted;
        }
        a.retries_left -= 1;
        a.k = 20;
        if self.params.retry_in_place {
            a.phase = 2;
        } else {
            a.phase = 1;
            self.direction = Direction::Back;
        }
        self.waits = 0;
        NoCharge::Retry
    }

    /// The tries left (local_49c).
    pub fn retries_left(&self) -> i32 {
        match self.mode {
            Mode::Dock(a) => a.retries_left,
            _ => self.params.retries,
        }
    }

    /// exitPileAction once stood up: the goal (mInit), the limit from `now_ms`; returns the back-off steer it writes
    /// five times before its loop.
    pub fn begin_exit(&mut self, now_ms: u64) -> Steer {
        self.reset_pids();
        if self.start_point == [0.0; 3] || self.params.back_fixed {
            self.start_point[0] = self.params.exit_pile_distance_x;
        }
        self.waits = 0;
        self.direction = Direction::Back;
        self.mode = Mode::Exit(Exit {
            staged: false,
            t0_ms: now_ms,
            obstacle_since_ms: 0,
        });
        Steer {
            x: self.seq.back_off_x,
            ..Default::default()
        }
    }

    pub fn end(&mut self) {
        self.mode = Mode::Idle;
    }

    /// A 200 ms wait passed without a pose.
    pub fn wait_timed_out(&mut self) -> Wait {
        let finetune = matches!(self.mode, Mode::Dock(a) if a.phase == 3);
        self.waits += 1;
        let restart = self.waits >= self.params.restart_after_waits;
        Wait {
            stop: !finetune,
            restart,
        }
    }

    /// The detector restarted and FindTarget saw the dock again.
    pub fn restarted(&mut self) {
        self.waits = 0;
    }

    /// One dock pose. `navigating`: on NaviCharge / FinetuneStand in rl_control; `obstacle`: the obstacle check's
    /// nearest occupied cell (m) when it found one.
    pub fn on_pose(
        &mut self,
        now_ms: u64,
        p: TargetPose,
        navigating: bool,
        obstacle: Option<f32>,
    ) -> Outcome {
        match self.mode {
            Mode::Idle => Outcome::default(),
            Mode::Dock(_) => self.dock_pose(now_ms, p, navigating, obstacle),
            Mode::Exit(_) => self.exit_pose(now_ms, p, navigating, obstacle),
        }
    }

    fn fail(stop: bool, why: Failure) -> Outcome {
        Outcome {
            stop,
            event: Some(Event::Failed(why)),
            ..Default::default()
        }
    }

    fn blocked(&self, obstacle: Option<f32>) -> Option<f32> {
        // `ObstacleAvoidDis <= distance` is clear
        obstacle.filter(|d| !(self.params.obstacle_avoid_dis <= *d))
    }

    fn dock_pose(
        &mut self,
        now: u64,
        p: TargetPose,
        navigating: bool,
        obstacle: Option<f32>,
    ) -> Outcome {
        let Mode::Dock(mut a) = self.mode else {
            return Outcome::default();
        };
        let out = self.dock_step(&mut a, now, p, navigating, obstacle);
        if let Mode::Dock(m) = &mut self.mode {
            *m = a;
        }
        out
    }

    fn dock_step(
        &mut self,
        a: &mut Approach,
        now: u64,
        p: TargetPose,
        navigating: bool,
        obstacle: Option<f32>,
    ) -> Outcome {
        if !navigating {
            return Self::fail(true, Failure::NotNavigating);
        }
        let s = self.seq.clone();
        let c = &self.params;
        let x = c.compensation_x - p.x;
        let y = -p.y;
        let yaw = p.yaw;
        let mut out = Outcome::default();
        if !a.started {
            if !(0.0 < x) {
                return out;
            }
            if a.samples <= s.start_samples {
                a.samples += 1;
                if y.abs() < a.best[1].abs() {
                    a.best = [-p.x, y, yaw];
                }
                return out;
            }
            a.started = true;
            self.start_point = a.best;
            out.start_point = Some(a.best);
        }
        let d = [c.delta[0] as f64, c.delta[1] as f64, c.delta[2] as f64];
        if a.phase == 3 {
            let z = (a.k as f64 * s.finetune_z_step) as f32;
            a.k += 1;
            if a.k > s.finetune_z_last - 1 {
                let inside = x.abs() < c.delta[0] && y.abs() < c.delta[1] && yaw.abs() < c.delta[2];
                if (inside && a.t0_ms + s.finetune_min_ms < now)
                    || a.t0_ms + s.finetune_max_ms < now
                {
                    out.steer = Some(Steer {
                        z: s.sit_z,
                        ..Default::default()
                    });
                    out.event = Some(Event::FinetuneDone([x, y, yaw]));
                    return out;
                }
                a.k = s.finetune_z_last;
            }
            self.waits = 0;
            out.steer = Some(Steer { x, y, yaw, z });
            return out;
        }
        if let Some(dist) = self.blocked(obstacle) {
            out.stop = true;
            if a.obstacle_since_ms == 0 {
                self.waits = 0;
                a.obstacle_since_ms = now;
                out.event = Some(Event::Obstacle(dist));
                return out;
            }
            if now - a.obstacle_since_ms <= s.obstacle_limit_ms {
                self.waits = 0;
                out.event = Some(Event::Obstacle(dist));
                return out;
            }
            out.event = Some(Event::Failed(Failure::ObstacleTimeout));
            return out;
        }
        if a.t0_ms + s.approach_limit_ms < now {
            return Self::fail(true, Failure::ApproachTimeout);
        }
        if a.phase == 2 {
            if d[0] + s.fine_margin[0] <= x.abs() as f64
                || d[1] + s.fine_margin[1] <= y.abs() as f64
                || d[2] + s.fine_margin[2] <= yaw.abs() as f64
            {
                let ux = self.pid[0].control(x);
                let uy = self.pid[1].control(y);
                let uyaw = -self.pid[2].control(yaw);
                out.steer = Some(Steer {
                    x: limit_speed(ux, s.fine_max[0], c.min_speed[0]),
                    y: limit_speed(uy, s.fine_max[1], c.min_speed[1]),
                    yaw: limit_speed(uyaw, s.fine_max[2], c.min_speed[2]),
                    z: 0.0,
                });
                a.k = 2;
            } else {
                out.stop = true;
                out.event = Some(Event::FineReached([x, y, yaw]));
                a.phase = 3;
                a.k = 2;
                a.t0_ms = now;
            }
        } else {
            let ex = x - c.pre_point_x;
            if d[0] + s.staging_margin[0] <= ex.abs() as f64
                || d[1] + s.staging_margin[1] <= y.abs() as f64
                || d[2] + s.staging_margin[2] <= yaw.abs() as f64
            {
                let ux = self.pid[0].control(ex);
                let uy = self.pid[1].control(y);
                let uyaw = -self.pid[2].control(yaw);
                out.steer = Some(Steer {
                    x: limit_speed(ux, s.staging_max[0], s.staging_min[0]),
                    y: limit_speed(uy, s.staging_max[1], s.staging_min[1]),
                    yaw: limit_speed(uyaw, s.staging_max[2], c.min_speed[2]),
                    z: 0.0,
                });
            } else {
                out.event = Some(Event::StagingReached);
                self.pid[0].reset();
                self.direction = Direction::Front;
                a.phase = 2;
            }
        }
        a.obstacle_since_ms = 0;
        self.waits = 0;
        out
    }

    fn exit_pose(
        &mut self,
        now: u64,
        p: TargetPose,
        navigating: bool,
        obstacle: Option<f32>,
    ) -> Outcome {
        let Mode::Exit(mut e) = self.mode else {
            return Outcome::default();
        };
        let out = self.exit_step(&mut e, now, p, navigating, obstacle);
        if let Mode::Exit(m) = &mut self.mode {
            *m = e;
        }
        out
    }

    fn exit_step(
        &mut self,
        e: &mut Exit,
        now: u64,
        p: TargetPose,
        navigating: bool,
        obstacle: Option<f32>,
    ) -> Outcome {
        let s = self.seq.clone();
        let mut out = Outcome::default();
        if e.t0_ms + s.exit_limit_ms < now {
            return Self::fail(true, Failure::ExitTimeout);
        }
        if let Some(dist) = self.blocked(obstacle) {
            out.stop = true;
            if e.obstacle_since_ms == 0 {
                self.waits = 0;
                e.obstacle_since_ms = now;
                out.event = Some(Event::Obstacle(dist));
                return out;
            }
            if now - e.obstacle_since_ms <= s.obstacle_limit_ms {
                self.waits = 0;
                out.event = Some(Event::Obstacle(dist));
                return out;
            }
            out.event = Some(Event::Failed(Failure::ObstacleTimeout));
            return out;
        }
        if !navigating {
            return Self::fail(true, Failure::NotNavigating);
        }
        let c = &self.params;
        let x = c.compensation_x - p.x;
        let y = -p.y;
        let yaw = p.yaw;
        let (ex, ey, eyaw);
        if !e.staged {
            let goal = if self.start_point[0] <= c.pre_exit_point_x {
                c.pre_exit_point_x
            } else {
                self.start_point[0]
            };
            ex = x - goal;
            ey = y;
            eyaw = yaw;
            if (c.pre_exit_point_x as f64) - s.exit_staging_margin < x as f64 {
                out.stop = true;
                out.event = Some(Event::ExitStagingReached);
                e.staged = true;
            }
        } else {
            ex = x - self.start_point[0];
            ey = y - self.start_point[1];
            eyaw = yaw - self.start_point[2];
            if ex.abs() < c.ext_delta[0] && ey.abs() < c.ext_delta[1] && eyaw.abs() < c.ext_delta[2]
            {
                out.stop = true;
                out.event = Some(Event::ExitReached);
                return out;
            }
        }
        let ux = self.pid[0].control(ex);
        let uy = self.pid[1].control(ey);
        let u = self.pid[2].control(eyaw);
        if ux.is_nan() || uy.is_nan() || u.is_nan() {
            return Self::fail(out.stop, Failure::BadSteer);
        }
        out.steer = Some(Steer {
            x: limit_speed(ux, s.exit_max[0], c.ext_min_speed[0]),
            y: limit_speed(uy, s.exit_max[1], c.ext_min_speed[1]),
            yaw: limit_speed(-u, s.exit_max[2], c.ext_min_speed[2]),
            z: 0.0,
        });
        e.obstacle_since_ms = 0;
        self.waits = 0;
        out
    }
}

/// reflective_column_node's output stage: a sweep's lidar position (x, y in its post frame, m) and yaw kept only inside
/// its range box; after a (re)start the first `start_sweeps` are held back and the next publishes the last of them
/// within 3 sigma of their mean; then a sweep farther than `max_jump_m` from the one before is dropped.
#[derive(Debug, Clone)]
pub struct ReflectorFilter {
    cfg: FilterConfig,
    samples: Vec<[f64; 2]>,
    started: bool,
    last: [f64; 2],
}

impl ReflectorFilter {
    pub fn new(cfg: &FilterConfig) -> ReflectorFilter {
        ReflectorFilter {
            cfg: cfg.clone(),
            samples: Vec::with_capacity(cfg.start_sweeps),
            started: false,
            last: [0.0; 2],
        }
    }

    /// The start fix is out (sweeps are published or jump-dropped from here).
    pub fn started(&self) -> bool {
        self.started
    }

    /// The service's start command: clear the position queue, filter again.
    pub fn restart(&mut self) {
        self.samples.clear();
        self.started = false;
    }

    /// One sweep's (x, y, yaw) -> the published (x, y, yaw), if any.
    pub fn sweep(&mut self, x: f64, y: f64, yaw: f64) -> Option<[f64; 3]> {
        let c = &self.cfg;
        if x < c.x_range[0] || c.x_range[1] < x || c.max_y < y || y < -c.max_y {
            return None;
        }
        let mut cur = [x, y];
        if !self.started {
            if self.samples.len() < c.start_sweeps {
                self.samples.push(cur);
                return None;
            }
            cur = self.start_fix();
            self.last = cur;
            self.started = true;
        }
        let jump = (cur[0] - self.last[0]).hypot(cur[1] - self.last[1]);
        self.last = cur;
        if c.max_jump_m < jump {
            return None;
        }
        Some([cur[0], cur[1], yaw])
    }

    /// FilterPositionData: mean and population sigma of x and y; the last sample within 3 sigma of both (else the last).
    fn start_fix(&self) -> [f64; 2] {
        let n = self.samples.len() as f64;
        let mean = |k: usize| self.samples.iter().map(|s| s[k]).sum::<f64>() / n;
        let (mx, my) = (mean(0), mean(1));
        let var =
            |k: usize, m: f64| self.samples.iter().map(|s| (s[k] - m).powi(2)).sum::<f64>() / n;
        let (sx, sy) = (var(0, mx).sqrt(), var(1, my).sqrt());
        let k = self.cfg.start_sigmas;
        self.samples
            .iter()
            .rev()
            .find(|s| (s[0] - mx).abs() < sx * k && (s[1] - my).abs() < sy * k)
            .or(self.samples.last())
            .copied()
            .unwrap_or([0.0; 2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_is_p_plus_d_from_a_reset_and_keeps_its_last_error() {
        let mut p = Pid::new([1.2, 0.0, 0.2]);
        // the vendor's first fine-point steer of 2026-11-21 13:27: x 0.599033 then 0.570704 -> 0.679179 logged
        p.control(0.599033);
        assert!((p.control(0.570704) - 0.679179).abs() < 1e-6);
        p.reset();
        assert!((p.control(0.5) - 0.7).abs() < 1e-6);
    }

    #[test]
    fn limit_speed_is_the_binarys() {
        assert_eq!(limit_speed(0.9, 0.7, 0.18), 0.7);
        assert_eq!(limit_speed(-0.9, 0.7, 0.18), -0.7);
        assert_eq!(limit_speed(0.01, 0.8, 0.25), 0.25);
        assert_eq!(limit_speed(-0.01, 0.8, 0.25), -0.25);
        assert_eq!(limit_speed(0.0, 0.8, 0.25), 0.25);
        assert_eq!(limit_speed(0.5, 0.7, 0.18), 0.5);
    }

    fn filter_cfg() -> FilterConfig {
        FilterConfig {
            x_range: [-4.0, -0.05],
            max_y: 1.5,
            pose_offset_x: 0.7599999904632568,
            start_sweeps: 15,
            start_sigmas: 3.0,
            max_jump_m: 0.8,
        }
    }

    #[test]
    fn the_filter_holds_back_fifteen_then_publishes_and_drops_jumps() {
        let mut f = ReflectorFilter::new(&filter_cfg());
        for k in 0..15 {
            assert_eq!(f.sweep(-1.0 - k as f64 * 1e-3, 0.0, 0.1), None);
        }
        // the 16th publishes the last of the 15 within 3 sigma, with its own yaw
        assert_eq!(f.sweep(-2.0, 0.0, 0.2), Some([-1.014, 0.0, 0.2]));
        assert!(f.sweep(-1.02, 0.0, 0.2).is_some());
        // a jump is dropped, and so is the sweep after it (measured from the dropped one)
        assert_eq!(f.sweep(-2.5, 0.0, 0.2), None);
        assert_eq!(f.sweep(-1.03, 0.0, 0.2), None);
        assert!(f.sweep(-1.04, 0.0, 0.2).is_some());
        // out of the range box: not even counted
        assert_eq!(f.sweep(-0.01, 0.0, 0.2), None);
        assert_eq!(f.sweep(-1.0, 1.6, 0.2), None);
        f.restart();
        assert_eq!(f.sweep(-1.0, 0.0, 0.2), None);
    }
}
