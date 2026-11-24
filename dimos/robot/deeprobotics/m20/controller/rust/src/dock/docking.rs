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

//! charge_manager's dock / undock sequence stepped once per control tick ([`Docking`]), and the command source that
//! puts it in front of the operator's ([`DockingCommands`]). The per-pose decisions are [`super::vendor::ChargeCore`];
//! this is what blocks in the vendor, as phases:
//!
//! Dock (DoChargeTask + goToPileAction): the detector restarts and FindTarget waits for its first pose; the obstacle
//! check opens looking ahead (obstacle_open_ms); NaviCharge, held gait_settle_ms (WideWalk); the loop: the start point,
//! the staging point, the fine point, every pose steering; FinetuneStand (setFineTuneStandMode); its loop of pose
//! inputs; (0, 0, 0, sit_z) for sit_hold_ms; lie down; three charge-enables charge_sleep_ms apart (power_on_ms), then
//! first_detection_current_timeout for every pack's charge MOS. Charging: a charge-enable every charge_wait_ms +
//! charge_sleep_ms (an undock asked for in the sleep starts when it ends). No charge: stand on NaviCharge,
//! retry_settle_ms, approach again (adjust_posture_times); used up, NoChargePlug_way: goBack stands, restarts the
//! detector and undocks, sitDown stays lying.
//!
//! Undock (DoOverChargeTask + exitPileAction, only from charging or after a failure): charge-enable 0, the detector
//! restarts and FindTarget waits (lying); the obstacle check opens looking back; stand (StandUp2WideWalk: stand,
//! stand_rl_control_ms, rl_control and NaviCharge); the back-off kick; back off to the exit staging point and on to
//! where docking started (back_way); the motion state docking started from, restore_ms, its gait, charge-enable 0.
//!
//! What charge_manager writes to rl_deploy (motion state, gait, steer, charge-enable) is written here as [`Write`]s at
//! the same points, and the controller's command follows from them.
//!
//! In every loop a pose missing for pose_wait_ms stops the robot, and enough in a row restart the detector (a FindTarget
//! that fails lies the robot down). A steer holds until the next or steer_timeout_ms (rl_deploy). Every time is
//! counted in control ticks.
//!
//! Ours on top: refuses unless standing in rl_control with no fault and the stand-in over; a controller fault lies the
//! robot down; an operator motion command ends it.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use serde::Serialize;

use super::config::SequenceConfig;
use super::vendor::{
    ChargeCore, ChargeManagerToml, ChargeParams, Direction, Event, Failure, NoCharge, Outcome,
    ReflectorFilter, Steer, TargetPose,
};
use super::{ChargeReading, DockPose};
use crate::commands::CommandSource;
use crate::config::Config;
use crate::state_machine::{Controller, State};
use crate::types::{Target, UserCommand};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    Dock,
    Undock,
    Cancel,
}

impl Request {
    pub fn parse(s: &str) -> Option<Request> {
        match s.trim() {
            "dock" => Some(Request::Dock),
            "undock" => Some(Request::Undock),
            "cancel" => Some(Request::Cancel),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum Phase {
    Idle,
    /// the detector restarted: waiting for its first pose (FindTarget)
    FindTarget,
    /// ObstacleDetection::Open's obstacle_open_ms sleep before what FindTarget leads to
    OpenObstacle,
    /// charge_manager waiting on its detector process's service (detector_service): the replies still to come
    DetectorCall,
    /// switching to NaviCharge, then held gait_settle_ms (WideWalk)
    EnterGait,
    /// goToPileAction's loop: the start point, the staging point, the fine point
    Approach,
    /// switching to FinetuneStand (setFineTuneStandMode)
    EnterFinetune,
    Finetune,
    /// (0, 0, 0, sit_z) held sit_hold_ms
    SitHold,
    LyingDown,
    /// power_on_ms, then up to first_detection_current_timeout for the charge
    ConfirmCharge,
    /// no charge: standing on NaviCharge again, then retry_settle_ms
    StandForRetry,
    /// lying on the dock (charging, or waiting for it after NoChargePlug_way = sitDown), until undock
    Charging,
    /// the detector restarting mid-loop (FindTarget again)
    Restart,
    /// undocking (or giving up): StandUp2WideWalk
    StandUp,
    BackOff,
    /// undocked: restore_ms, then the gait docking started from
    Restore,
}

/// What the sequence needs from the controller each tick.
#[derive(Debug, Clone, Default)]
pub struct View {
    pub state: Option<State>,
    pub fault: bool,
    pub runner: Option<String>,
    pub stand_in: bool,
    /// the gait walking is on
    pub gait: u32,
    /// control ticks the controller has been in `state`
    pub state_ticks: u64,
}

impl View {
    /// Refresh from the controller (no allocation unless the runner changed).
    pub fn update(&mut self, c: &Controller) {
        if self.state == Some(c.state) {
            self.state_ticks += 1;
        } else {
            self.state = Some(c.state);
            self.state_ticks = 0;
        }
        self.fault = c.fault.is_some();
        self.stand_in = c.in_stand_in();
        self.gait = c.gait();
        if self.runner.as_deref() != c.current_runner() {
            self.runner = c.current_runner().map(String::from);
        }
    }

    fn is(&self, state: State) -> bool {
        self.state == Some(state)
    }

    fn on(&self, runner: &str) -> bool {
        !self.stand_in && self.runner.as_deref() == Some(runner)
    }

    fn lying(&self) -> bool {
        self.is(State::LieDown) || self.is(State::Passive)
    }
}

/// The command the sequence sends this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Output {
    pub target: Target,
    pub gait: u32,
    pub axes: [f64; 3],
    /// FinetuneStand's pose inputs (dx, dy, yaw, height) while finetuning
    pub pose: Option<[f64; 4]>,
    /// charge_manager's /BATTERY_CHARGE_ENABLE written this tick (rl_deploy: 1 sends the board 3 charge-enable frames)
    pub charge_enable: Option<i32>,
}

/// The command parameters that carry a charge-enable write to the bus: its value and its count.
pub const CHARGE_ENABLE_PARAM: &str = "charge_enable";
pub const CHARGE_ENABLE_SEQ_PARAM: &str = "charge_enable_seq";

/// rl_deploy's MotionState codes as charge_manager writes them.
pub const MOTION_STAND: i32 = 1;
pub const MOTION_LIE_DOWN: i32 = 4;
pub const MOTION_RL_CONTROL: i32 = 17;

/// What charge_manager writes to rl_deploy (DDS /MOTION_STATE, /GAIT, /CHARGE_CMD, /BATTERY_CHARGE_ENABLE), written by
/// this sequence where and in the order charge_manager writes it; the commands it sends the controller follow from
/// them. [`Docking::record_writes`] keeps them for comparing with a recording.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Write {
    MotionState(i32),
    Gait(u32),
    Steer(Steer),
    ChargeEnable(i32),
}

fn motion_code(t: Target) -> Option<i32> {
    match t {
        Target::Stand => Some(MOTION_STAND),
        Target::LieDown => Some(MOTION_LIE_DOWN),
        Target::RlControl => Some(MOTION_RL_CONTROL),
        Target::Passive | Target::JointDamping => None,
    }
}

fn motion_target(code: i32) -> Option<Target> {
    match code {
        MOTION_STAND => Some(Target::Stand),
        MOTION_LIE_DOWN => Some(Target::LieDown),
        MOTION_RL_CONTROL => Some(Target::RlControl),
        _ => None,
    }
}

/// rl_deploy's MotionState code for a controller state (what charge_manager reads from /MOTION_INFO).
fn state_code(s: Option<State>) -> i32 {
    match s {
        Some(State::StandUp) => MOTION_STAND,
        Some(State::LieDown) => MOTION_LIE_DOWN,
        Some(State::RlControl) => MOTION_RL_CONTROL,
        _ => 0,
    }
}

/// The sequence's state as published (dock_state).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Status {
    pub phase: Phase,
    /// docking or undocking (the sequence drives the controller)
    pub active: bool,
    /// tries after the first that found no charge
    pub retries: u32,
    /// the last sequence's outcome ("" while none ended yet)
    pub result: String,
    pub error: bool,
    pub charging: bool,
    pub packs_present: u8,
    pub packs_charge_mos_on: u8,
    /// charge_manager's (x m, y m, yaw rad) at the fine point
    pub fine_point: Option<[f32; 3]>,
    /// where docking started (mInit: x m out from docked, y m, yaw rad), the undock's goal with back_way = auto
    pub start_point: Option<[f32; 3]>,
    /// the obstacle check is open, looking this way (the detector module runs it)
    pub obstacle: Option<Direction>,
    /// stopped for an obstacle this far (m)
    pub obstacle_stop: Option<f32>,
    /// charge_manager.toml is read
    pub vendor_config: bool,
}

/// What charge_manager does once its detector process's service calls return.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AfterCalls {
    /// OpenMarkerProcess (close, open): FindTarget
    FindTarget(Next),
    /// the undock's end (close, CloseMarkProcess's close): Close24vPower, idle
    Undocked,
    /// charging confirmed (CloseMarkProcess): the charging loop
    Charging,
}

/// What a FindTarget / stand-up leads to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Next {
    Dock,
    Undock,
    BackOff,
    /// stood up after giving up: restart the detector, then back off
    FindTarget,
}

/// One pose's charge_manager inputs from our detector's pose.
fn target_pose(p: &DockPose, docked_x: f64) -> (f64, f64, f64) {
    let [out, y, yaw] = p.robot_in_dock();
    (docked_x - out, -y, yaw)
}

pub struct Docking {
    seq: SequenceConfig,
    hz: f64,
    charge_enabled: bool,
    approach_runner: String,
    finetune_runner: String,
    core: Option<ChargeCore>,
    filter: ReflectorFilter,
    pose_offset_x: f64,
    phase: Phase,
    ticks_in_phase: u64,
    /// control ticks since start
    tick: u64,
    next: Next,
    resume: Phase,
    giving_up: bool,
    /// the last sequence failed (charge_manager's error state: an undock is allowed)
    failed: bool,
    /// the phase's runner is up (ticks_in_phase counts from then)
    on_runner: bool,
    pose_seq: Option<u64>,
    /// the pose the detector's output stage published this tick
    published: Option<TargetPose>,
    /// when the loop's 200 ms pose wait started (us of control time): a pose, or the last missed wait's end plus
    /// charge_manager's own time logging and stopping
    woke_at_us: u64,
    /// the last steer and the tick it went out
    steer: Option<(Steer, u64)>,
    target: Target,
    gait: u32,
    /// the gait to restore after an undock (mInitRobotGaitStatus_)
    restore_gait: u32,
    charge: Option<ChargeReading>,
    /// the charge counts from readings after this one (the vendor clears its flag at the dock's start)
    charge_seq_at_start: u64,
    obstacle: Option<(Option<f32>, u64)>,
    obstacle_open: bool,
    obstacle_seq_at_open: u64,
    status: Status,
    /// what was written, kept when recording
    record: Option<Vec<(u64, Write)>>,
    charge_enable_now: Option<i32>,
    /// charge-enables written so far
    pub charge_enables: u64,
    /// the motion state to restore after an undock (mInitRobotStatus_)
    restore_state: i32,
    /// an undock asked for while charging, taken at the charging loop's next wake
    undock_pending: bool,
    /// the charging loop: the tick of its next charge-enable write, and the end of the sleep after the last one
    charge_next_write: u64,
    charge_sleep_until: u64,
    /// StandUp2WideWalk: the tick it writes rl_control and the gait (2 s after stand), once
    stand_at: Option<u64>,
    /// detector service replies still to come, and what follows them
    calls_pending: u32,
    calls_then: Option<AfterCalls>,
}

impl Docking {
    /// `charge_enabled`: the bus backend runs m20-battery's charge-enable rule (without it nothing can charge).
    /// `vendor`: charge_manager.toml, once read (docking refuses until then).
    pub fn new(cfg: &Config, charge_enabled: bool, vendor: Option<&ChargeManagerToml>) -> Docking {
        let seq = cfg.dock.sequence;
        let runner = |gait: u32| cfg.runner_for_gait(gait).to_string();
        let mut d = Docking {
            approach_runner: runner(seq.approach_gait),
            finetune_runner: runner(seq.finetune_gait),
            hz: cfg.timing.control_hz,
            charge_enabled,
            core: None,
            filter: ReflectorFilter::new(&cfg.dock.filter),
            pose_offset_x: cfg.dock.filter.pose_offset_x,
            phase: Phase::Idle,
            ticks_in_phase: 0,
            tick: 0,
            next: Next::Dock,
            resume: Phase::Approach,
            giving_up: false,
            failed: false,
            on_runner: false,
            pose_seq: None,
            published: None,
            woke_at_us: 0,
            steer: None,
            target: Target::Passive,
            gait: seq.approach_gait,
            restore_gait: seq.default_restore_gait,
            charge: None,
            charge_seq_at_start: 0,
            obstacle: None,
            obstacle_open: false,
            obstacle_seq_at_open: 0,
            record: None,
            charge_enable_now: None,
            charge_enables: 0,
            restore_state: MOTION_RL_CONTROL,
            undock_pending: false,
            charge_next_write: 0,
            charge_sleep_until: 0,
            stand_at: None,
            calls_pending: 0,
            calls_then: None,
            status: Status {
                phase: Phase::Idle,
                active: false,
                retries: 0,
                result: String::new(),
                error: false,
                charging: false,
                packs_present: 0,
                packs_charge_mos_on: 0,
                fine_point: None,
                start_point: None,
                obstacle: None,
                obstacle_stop: None,
                vendor_config: false,
            },
            seq,
        };
        if let Some(t) = vendor {
            d.set_vendor(t);
        }
        d
    }

    /// Take charge_manager.toml (only while idle).
    pub fn set_vendor(&mut self, t: &ChargeManagerToml) -> bool {
        if self.active() {
            return false;
        }
        let start = self.core.as_ref().map(|c| c.start_point());
        let mut core = ChargeCore::new(ChargeParams::from_toml(t), &self.seq);
        if let Some(p) = start {
            core.set_start_point(p);
        }
        self.core = Some(core);
        self.status.vendor_config = true;
        true
    }

    pub fn has_vendor(&self) -> bool {
        self.core.is_some()
    }

    /// Keep every [`Write`] with its tick (from now on), for [`Self::take_writes`].
    pub fn record_writes(&mut self) {
        self.record.get_or_insert_with(Vec::new);
    }

    pub fn take_writes(&mut self) -> Vec<(u64, Write)> {
        self.record.as_mut().map(std::mem::take).unwrap_or_default()
    }

    /// Control ticks since start (the tick [`Self::advance`] last ran).
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// Charging on the dock as charge_manager was when a recording began: lying, charging, the undock's goal
    /// `start_point` and the (motion state, gait) to restore after it from the dock before, and the charging loop's
    /// last charge-enable written this tick (it sleeps charge_sleep_ms, waits charge_wait_ms, writes again).
    pub fn resume_charging(&mut self, start_point: [f32; 3], restore: (i32, u32)) {
        if let Some(c) = self.core.as_mut() {
            c.set_start_point(start_point);
        }
        self.status.start_point = Some(start_point);
        (self.restore_state, self.restore_gait) = restore;
        self.target = Target::LieDown;
        self.status.charging = true;
        self.set_phase(Phase::Charging);
        self.charge_sleep_until = self.tick + self.ticks(self.seq.charge_sleep_ms);
        self.charge_next_write = self.charge_sleep_until + self.ticks(self.seq.charge_wait_ms);
    }

    /// One of charge_manager's writes, and what it asks the controller.
    fn write(&mut self, w: Write) {
        match w {
            Write::MotionState(code) => {
                if let Some(t) = motion_target(code) {
                    self.target = t;
                }
            }
            Write::Gait(g) => self.gait = g,
            Write::Steer(s) => self.steer = Some((s, self.tick)),
            Write::ChargeEnable(v) => {
                self.charge_enable_now = Some(v);
                self.charge_enables += 1;
            }
        }
        if let Some(r) = self.record.as_mut() {
            r.push((self.tick, w));
        }
    }

    /// The target, written as charge_manager's MotionState when it has one.
    fn set_target(&mut self, t: Target) {
        match motion_code(t) {
            Some(code) if t != self.target => self.write(Write::MotionState(code)),
            _ => self.target = t,
        }
    }

    fn set_gait(&mut self, g: u32) {
        if g != self.gait {
            self.write(Write::Gait(g));
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn status(&self) -> &Status {
        &self.status
    }

    /// The sequence drives the controller (returns a command every tick).
    pub fn active(&self) -> bool {
        self.phase != Phase::Idle
    }

    fn ms(&self, ticks: u64) -> u64 {
        (ticks as f64 * 1000.0 / self.hz).round() as u64
    }

    fn now_ms(&self) -> u64 {
        self.ms(self.tick)
    }

    fn now_us(&self) -> u64 {
        (self.tick as f64 * 1e6 / self.hz).round() as u64
    }

    /// A reply from the detector's service (detector_service: what charge_manager waits on before going on).
    pub fn detector_replied(&mut self) {
        if self.phase == Phase::DetectorCall && self.calls_pending > 0 {
            self.calls_pending -= 1;
        }
    }

    /// charge_manager calls its detector process `n` times (DDS service, waiting for each reply), then `then`; without
    /// detector_service (ours is no process to call) it goes on at once.
    fn call_detector(&mut self, n: u32, then: AfterCalls) -> Option<Output> {
        if !self.seq.detector_service {
            return self.after_calls(then);
        }
        self.calls_pending = n;
        self.calls_then = Some(then);
        self.set_phase(Phase::DetectorCall);
        Some(self.output(None))
    }

    fn after_calls(&mut self, then: AfterCalls) -> Option<Output> {
        match then {
            AfterCalls::FindTarget(next) => self.find_target(next),
            AfterCalls::Undocked => {
                self.write(Write::ChargeEnable(0));
                let tries = self.status.retries + 1;
                return if self.giving_up {
                    self.finish(
                        Target::RlControl,
                        format!("no charge after {tries} tries; backed off"),
                        true,
                    )
                } else {
                    self.finish(Target::RlControl, "undocked", false)
                };
            }
            AfterCalls::Charging => {
                self.set_phase(Phase::Charging);
                self.charge_sleep_until = self.tick;
                self.charge_next_write = self.tick + self.ticks(self.seq.charge_wait_ms);
            }
        }
        Some(self.output(None))
    }

    fn ticks(&self, ms: u64) -> u64 {
        (ms as f64 * self.hz / 1000.0).round() as u64
    }

    fn phase_ms(&self) -> u64 {
        self.ms(self.ticks_in_phase)
    }

    fn set_phase(&mut self, phase: Phase) {
        self.phase = phase;
        self.ticks_in_phase = 0;
        self.on_runner = false;
    }

    fn params(&self) -> Option<&ChargeParams> {
        self.core.as_ref().map(|c| &c.params)
    }

    fn charged(&self, r: &ChargeReading) -> bool {
        let all = self.params().map_or(true, |p| p.all_battery_charge);
        r.packs_present > 0
            && if all {
                r.packs_charge_mos_on >= r.packs_present
            } else {
                r.packs_charge_mos_on > 0
            }
    }

    /// Open the obstacle check (when charge_manager.toml has it on); true when it was closed (Open then sleeps).
    fn open_obstacle(&mut self) -> bool {
        let avoid = self.params().is_some_and(|p| p.obstacle_avoid);
        let opened = avoid && !self.obstacle_open;
        self.obstacle_open = avoid;
        self.obstacle_seq_at_open = self.obstacle.map_or(0, |o| o.1);
        self.status.obstacle_stop = None;
        opened
    }

    /// The obstacle check's newest reading since it opened.
    fn obstacle_reading(&self) -> Option<f32> {
        if !self.obstacle_open {
            return None;
        }
        self.obstacle
            .filter(|o| o.1 > self.obstacle_seq_at_open)
            .and_then(|o| o.0)
    }

    /// End the sequence with `result`; `target` is what the controller is left with.
    fn finish(&mut self, target: Target, result: impl Into<String>, error: bool) -> Option<Output> {
        self.set_target(target);
        self.steer = None;
        self.stand_at = None;
        self.undock_pending = false;
        self.obstacle_open = false;
        if let Some(c) = self.core.as_mut() {
            c.end();
        }
        self.set_phase(Phase::Idle);
        self.status.result = result.into();
        self.status.error = error;
        self.failed = error;
        self.giving_up = false;
        Some(self.output(None))
    }

    /// Fail standing on BasicFlat (NarrowWalk).
    fn finish_basic_flat(&mut self, result: impl Into<String>) -> Option<Output> {
        // NarrowWalk
        self.write(Write::Gait(self.seq.basic_flat_gait));
        self.finish(Target::RlControl, result, true)
    }

    /// The steer as normalized axes, zeroed steer_timeout_ms after it went out.
    fn axes(&self) -> [f64; 3] {
        match self.steer {
            Some((s, at)) if self.tick - at <= self.ticks(self.seq.steer_timeout_ms) => {
                [s.x as f64, s.y as f64, s.yaw as f64]
            }
            _ => [0.0; 3],
        }
    }

    /// FinetuneStand's pose inputs from the steer (rl_deploy): dx, dy, yaw scaled, height = max(-z, 0).
    fn pose_inputs(&self) -> [f64; 4] {
        let s = self.steer.map(|s| s.0).unwrap_or_default();
        let k = self.seq.finetune_pose_scale;
        [
            k[0] * s.x as f64,
            k[1] * s.y as f64,
            k[2] * s.yaw as f64,
            (-(s.z as f64)).max(0.0),
        ]
    }

    fn output(&self, pose: Option<[f64; 4]>) -> Output {
        let axes = if pose.is_some() {
            [0.0; 3]
        } else {
            self.axes()
        };
        Output {
            target: self.target,
            gait: self.gait,
            axes,
            pose,
            charge_enable: self.charge_enable_now,
        }
    }

    fn send(&mut self, steer: Steer) {
        self.write(Write::Steer(steer));
    }

    /// Ask for a dock, an undock or a stop. Returns why it was refused, if it was.
    pub fn request(&mut self, request: Request, view: &View) -> Result<(), String> {
        match request {
            Request::Cancel => {
                if self.active() {
                    let lying = matches!(
                        self.phase,
                        Phase::SitHold | Phase::LyingDown | Phase::ConfirmCharge | Phase::Charging
                    );
                    self.finish(
                        if lying { Target::LieDown } else { self.target },
                        "cancelled",
                        false,
                    );
                }
                Ok(())
            }
            Request::Dock => {
                if self.active() {
                    return Err(format!("busy ({:?})", self.phase));
                }
                if self.core.is_none() {
                    return Err("charge_manager.toml is not read yet".into());
                }
                if !self.charge_enabled {
                    return Err(
                        "charge_enable is off: nothing would send the charge-enable request".into(),
                    );
                }
                if view.fault || !view.is(State::RlControl) || view.stand_in {
                    return Err(format!(
                        "dock needs the robot standing in rl_control with no fault (state {:?}{}{})",
                        view.state,
                        if view.fault { ", fault" } else { "" },
                        if view.stand_in { ", stand-in running" } else { "" }
                    ));
                }
                // goToPileAction records the motion state and gait to restore after the undock
                self.restore_gait = view.gait;
                self.restore_state = state_code(view.state);
                self.status.fine_point = None;
                self.status.result.clear();
                self.status.error = false;
                self.failed = false;
                self.status.retries = 0;
                self.charge_seq_at_start = self.charge.map_or(0, |r| r.seq);
                self.target = Target::RlControl;
                self.gait = view.gait;
                // OpenMarkerProcess: close and open the detector, then FindTarget
                self.call_detector(2, AfterCalls::FindTarget(Next::Dock));
                Ok(())
            }
            Request::Undock => {
                if self.core.is_none() {
                    return Err("charge_manager.toml is not read yet".into());
                }
                let charging_now = self.charge.is_some_and(|r| self.charged(&r));
                let allowed = self.phase == Phase::Charging
                    || (!self.active() && (self.failed || charging_now));
                if !allowed {
                    return Err(format!(
                        "charge_manager undocks only while charging or after a failed sequence ({:?})",
                        self.phase
                    ));
                }
                if view.fault {
                    return Err("undock needs no controller fault".into());
                }
                self.status.result.clear();
                self.status.error = false;
                self.failed = false;
                self.giving_up = false;
                // charge_manager's main loop takes it at its next wake (charging: once the loop's sleep ends)
                self.undock_pending = true;
                Ok(())
            }
        }
    }

    /// Restart the detector and wait for its first pose.
    fn find_target(&mut self, next: Next) {
        self.filter.restart();
        // a pose published before the restart is not the detector's first
        self.published = None;
        self.next = next;
        self.steer = None;
        self.set_phase(Phase::FindTarget);
    }

    /// StandUp2WideWalk: unless in rl_control, stand (MotionState 1), 2 s later rl_control and NaviCharge, then wait
    /// for NaviCharge; in rl_control already it changes nothing.
    fn stand_up(&mut self, next: Next, view: &View) {
        self.stand_at = None;
        if !view.is(State::RlControl) {
            self.write(Write::MotionState(MOTION_STAND));
            self.stand_at = Some(self.tick + self.ticks(self.seq.stand_rl_control_ms));
        }
        self.next = next;
        self.steer = None;
        if self.stand_at.is_some() {
            self.set_phase(Phase::StandUp);
        } else {
            self.stood();
        }
    }

    /// StandUp2WideWalk returned: what the stand-up was for.
    fn stood(&mut self) {
        if self.next == Next::FindTarget {
            self.find_target(Next::BackOff);
        } else {
            self.begin_back_off();
        }
    }

    /// FindTarget found the dock (and the obstacle check is open): what it was for.
    fn found_target(&mut self, view: &View) {
        match self.next {
            Next::Dock => {
                let now = self.now_ms();
                if let Some(c) = self.core.as_mut() {
                    c.begin_dock(now);
                }
                // WideWalk
                if view.gait != self.seq.approach_gait {
                    self.write(Write::Gait(self.seq.approach_gait));
                }
                self.set_phase(Phase::EnterGait);
            }
            Next::Undock => self.stand_up(Next::BackOff, view),
            Next::BackOff | Next::FindTarget => self.begin_back_off(),
        }
    }

    /// The undock starts (DoOverChargeTask): Close24vPower, then the detector restarts.
    fn start_undock(&mut self) {
        self.undock_pending = false;
        self.status.charging = false;
        self.write(Write::ChargeEnable(0));
        // OpenMarkerProcess: close and open the detector, then FindTarget
        self.call_detector(2, AfterCalls::FindTarget(Next::Undock));
    }

    /// One control tick: the newest dock pose (with its sweep count) and battery reading, the controller as last seen.
    pub fn step(
        &mut self,
        view: &View,
        pose: Option<(DockPose, u64)>,
        charge: Option<ChargeReading>,
    ) -> Option<Output> {
        self.ingest(pose, charge, None);
        self.advance(view)
    }

    /// Take this tick's newest dock pose (with its sweep count), battery reading and obstacle reading (nearest
    /// occupied cell, m, or none, with its count).
    pub fn ingest(
        &mut self,
        pose: Option<(DockPose, u64)>,
        charge: Option<ChargeReading>,
        obstacle: Option<(Option<f32>, u64)>,
    ) {
        self.published = None;
        if let Some((p, seq)) = pose {
            if self.pose_seq != Some(seq) {
                self.pose_seq = Some(seq);
                let (x, y, yaw) = target_pose(&p, self.seq.docked_x_m);
                if let Some([lx, ly, lyaw]) = self.filter.sweep(x - self.pose_offset_x, y, yaw) {
                    self.published = Some(TargetPose {
                        x: (lx + self.pose_offset_x) as f32,
                        y: ly as f32,
                        yaw: lyaw as f32,
                    });
                }
            }
        }
        if let Some(r) = charge {
            self.charge = Some(r);
            self.status.charging = self.charged(&r) && self.phase == Phase::Charging;
            self.status.packs_present = r.packs_present;
            self.status.packs_charge_mos_on = r.packs_charge_mos_on;
        }
        if let Some(o) = obstacle {
            self.obstacle = Some(o);
        }
    }

    /// A loop's pose handling: the core's steer / stop, or a missed wait.
    fn loop_tick(&mut self, view: &View) -> Option<Outcome> {
        let now = self.now_ms();
        let Some(p) = self.published else {
            let deadline = self.woke_at_us + self.seq.pose_wait_ms * 1000;
            if self.now_us() >= deadline {
                // the next wait starts once charge_manager has logged the miss and stopped the robot
                self.woke_at_us = deadline + self.seq.pose_wait_overhead_us;
                let core = self.core.as_mut()?;
                let w = core.wait_timed_out();
                if w.stop {
                    self.send(Steer::default());
                }
                if w.restart {
                    self.resume = self.phase;
                    self.filter.restart();
                    self.set_phase(Phase::Restart);
                }
            }
            return None;
        };
        self.woke_at_us = self.now_us();
        let navigating = view.is(State::RlControl)
            && (view.on(&self.approach_runner) || view.on(&self.finetune_runner));
        let obstacle = self.obstacle_reading();
        let out = self.core.as_mut()?.on_pose(now, p, navigating, obstacle);
        if out.stop {
            self.send(Steer::default());
        }
        if let Some(s) = out.steer {
            self.send(s);
        }
        self.status.obstacle_stop = match out.event {
            Some(Event::Obstacle(d)) => Some(d),
            _ => None,
        };
        Some(out)
    }

    fn failed(&mut self, why: Failure) -> Option<Output> {
        match why {
            Failure::NotNavigating => self.finish(
                self.target,
                "not on NaviCharge / FinetuneStand in rl_control",
                true,
            ),
            Failure::ApproachTimeout => self.finish_basic_flat("not at the dock in time"),
            Failure::ObstacleTimeout => self.finish_basic_flat("stopped for an obstacle too long"),
            Failure::ExitTimeout => self.finish_basic_flat("not undocked in time"),
            Failure::BadSteer => self.finish(self.target, "non-finite steer", true),
        }
    }

    /// Advance one control tick on what `ingest` took, the controller as last seen.
    pub fn advance(&mut self, view: &View) -> Option<Output> {
        self.tick += 1;
        self.charge_enable_now = None;
        if self.undock_pending
            && (self.phase != Phase::Charging || self.tick >= self.charge_sleep_until)
        {
            self.start_undock();
        }
        self.status.phase = self.phase;
        self.status.active = self.active();
        self.status.obstacle = if self.obstacle_open {
            self.core.as_ref().map(|c| c.direction)
        } else {
            None
        };
        if !self.active() {
            return None;
        }
        self.ticks_in_phase += 1;
        if view.fault {
            return self.finish(Target::LieDown, "controller fault", true);
        }
        let limit = (self.seq.transition_limit_s * self.hz).round() as u64;
        match self.phase {
            Phase::Idle => None,
            Phase::FindTarget => {
                if self.published.is_some() {
                    // ObstacleDetection::Open sleeps obstacle_open_ms when it opens the check
                    if self.open_obstacle() {
                        self.set_phase(Phase::OpenObstacle);
                    } else {
                        self.found_target(view);
                    }
                } else if self.phase_ms() > self.params()?.find_target_ms {
                    let target = self.target;
                    return self.finish(target, "the dock is not in sight", true);
                }
                Some(self.output(None))
            }
            Phase::DetectorCall => {
                if self.calls_pending == 0 {
                    if let Some(then) = self.calls_then.take() {
                        return self.after_calls(then);
                    }
                }
                Some(self.output(None))
            }
            Phase::OpenObstacle => {
                if self.phase_ms() >= self.seq.obstacle_open_ms {
                    self.found_target(view);
                }
                Some(self.output(None))
            }
            Phase::EnterGait | Phase::StandForRetry => {
                if !view.is(State::RlControl) && self.phase == Phase::EnterGait {
                    return self.finish(
                        Target::RlControl,
                        format!("left rl_control ({:?})", view.state),
                        true,
                    );
                }
                if view.is(State::RlControl) && view.on(&self.approach_runner) {
                    if !self.on_runner {
                        self.on_runner = true;
                        self.ticks_in_phase = 0;
                    } else {
                        let settle = if self.phase == Phase::EnterGait {
                            self.seq.gait_settle_ms
                        } else {
                            self.seq.retry_settle_ms
                        };
                        if self.phase_ms() >= settle {
                            if self.phase == Phase::StandForRetry {
                                // the back-off steer, at once overwritten by the stale finetune struct (0, 0, 0, sit_z)
                                self.send(Steer {
                                    z: self.seq.sit_z,
                                    ..Default::default()
                                });
                            }
                            self.woke_at_us = self.now_us();
                            self.set_phase(Phase::Approach);
                        }
                    }
                } else if self.ticks_in_phase > limit {
                    return self.finish(
                        Target::RlControl,
                        format!("no {}", self.approach_runner),
                        true,
                    );
                }
                Some(self.output(None))
            }
            Phase::Approach => {
                if let Some(out) = self.loop_tick(view) {
                    if let Some(p) = out.start_point {
                        self.status.start_point = Some(p);
                    }
                    match out.event {
                        Some(Event::FineReached(p)) => {
                            self.status.fine_point = Some(p);
                            // setFineTuneStandMode
                            self.write(Write::Gait(self.seq.finetune_gait));
                            self.set_phase(Phase::EnterFinetune);
                            return Some(self.output(Some([0.0; 4])));
                        }
                        Some(Event::Failed(why)) => return self.failed(why),
                        _ => {}
                    }
                }
                Some(self.output(None))
            }
            Phase::EnterFinetune => {
                let up = view.on(&self.finetune_runner);
                if up || self.phase_ms() >= self.seq.finetune_gait_wait_ms {
                    let now = self.now_ms();
                    self.core.as_mut()?.finetune_started(now);
                    self.woke_at_us = self.now_us();
                    self.set_phase(Phase::Finetune);
                }
                Some(self.output(Some([0.0; 4])))
            }
            Phase::Finetune => {
                if let Some(out) = self.loop_tick(view) {
                    match out.event {
                        Some(Event::FinetuneDone(_)) => self.set_phase(Phase::SitHold),
                        Some(Event::Failed(why)) => return self.failed(why),
                        _ => {}
                    }
                }
                if self.phase == Phase::Restart {
                    return Some(self.output(None));
                }
                Some(self.output(Some(self.pose_inputs())))
            }
            Phase::SitHold => {
                if self.phase_ms() >= self.seq.sit_hold_ms {
                    // SitDown
                    if !view.is(State::LieDown) {
                        self.write(Write::MotionState(MOTION_LIE_DOWN));
                    }
                    self.set_phase(Phase::LyingDown);
                    return Some(self.output(None));
                }
                Some(self.output(Some(self.pose_inputs())))
            }
            Phase::LyingDown => {
                if view.is(State::LieDown) {
                    // Open12vPower(3): charge-enable, sleep, three times
                    self.write(Write::ChargeEnable(1));
                    self.set_phase(Phase::ConfirmCharge);
                } else if self.ticks_in_phase > limit {
                    return self.no_charge(view);
                }
                Some(self.output(None))
            }
            Phase::ConfirmCharge => {
                if !view.lying() {
                    return self.finish(
                        self.target,
                        format!("left lie_down ({:?})", view.state),
                        true,
                    );
                }
                let p = self.params()?.clone();
                let elapsed = self.phase_ms();
                if elapsed < self.seq.power_on_ms && elapsed % self.seq.charge_sleep_ms == 0 {
                    self.write(Write::ChargeEnable(1));
                }
                if elapsed >= self.seq.power_on_ms {
                    let fresh = self.charge.filter(|r| r.seq > self.charge_seq_at_start);
                    if fresh.is_some_and(|r| self.charged(&r)) {
                        self.obstacle_open = false;
                        if let Some(c) = self.core.as_mut() {
                            c.end();
                        }
                        self.status.charging = true;
                        self.status.result = "docked, charging".into();
                        self.status.error = false;
                        // CloseMarkProcess, then the charging loop
                        return self.call_detector(1, AfterCalls::Charging);
                    } else if elapsed - self.seq.power_on_ms > p.charge_confirm_ms {
                        return self.no_charge(view);
                    }
                }
                Some(self.output(None))
            }
            Phase::Charging => {
                if !view.lying() {
                    // the vendor cancels a charge the robot was stood up from
                    return self.finish(self.target, "stood up while on the dock", false);
                }
                // charge_manager's charging loop: wait charge_wait_ms, Open12vPower(1) (write, sleep charge_sleep_ms)
                if self.status.charging && self.tick >= self.charge_next_write {
                    self.write(Write::ChargeEnable(1));
                    self.charge_sleep_until = self.tick + self.ticks(self.seq.charge_sleep_ms);
                    self.charge_next_write =
                        self.charge_sleep_until + self.ticks(self.seq.charge_wait_ms);
                }
                Some(self.output(None))
            }
            Phase::Restart => {
                if self.published.is_some() {
                    self.core.as_mut()?.restarted();
                    self.woke_at_us = self.now_us();
                    let resume = self.resume;
                    self.set_phase(resume);
                    self.on_runner = true;
                } else if self.phase_ms() > self.params()?.find_target_ms {
                    return self.finish(Target::LieDown, "lost the dock", true);
                }
                Some(self.output(None))
            }
            Phase::StandUp => {
                if let Some(at) = self.stand_at {
                    if self.tick >= at {
                        self.stand_at = None;
                        self.write(Write::MotionState(MOTION_RL_CONTROL));
                        self.write(Write::Gait(self.seq.approach_gait));
                    }
                } else if view.is(State::RlControl) && view.on(&self.approach_runner) {
                    self.stood();
                } else if self.ticks_in_phase > limit {
                    return self.finish(
                        self.target,
                        format!("did not stand ({:?})", view.state),
                        true,
                    );
                }
                Some(self.output(None))
            }
            Phase::BackOff => {
                if let Some(out) = self.loop_tick(view) {
                    match out.event {
                        Some(Event::ExitReached) => {
                            self.obstacle_open = false;
                            if let Some(c) = self.core.as_mut() {
                                c.end();
                            }
                            // restoreInitRobotStatus
                            self.write(Write::MotionState(self.restore_state));
                            self.set_phase(Phase::Restore);
                        }
                        Some(Event::Failed(why)) => return self.failed(why),
                        _ => {}
                    }
                }
                Some(self.output(None))
            }
            Phase::Restore => {
                if self.phase_ms() >= self.seq.restore_ms {
                    // restoreInitRobotGaitStatus; the detector closed (exitPileAction's end, then CloseMarkProcess in
                    // its main loop), Close24vPower
                    self.write(Write::Gait(self.restore_gait));
                    return self.call_detector(2, AfterCalls::Undocked);
                }
                Some(self.output(None))
            }
        }
    }

    fn begin_back_off(&mut self) {
        let now = self.now_ms();
        if let Some(core) = self.core.as_mut() {
            let kick = core.begin_exit(now);
            for _ in 0..self.seq.back_off_writes {
                self.send(kick);
            }
        }
        self.woke_at_us = self.now_us();
        self.set_phase(Phase::BackOff);
    }

    /// No charge after lying down: retry, or (tries used up) NoChargePlug_way.
    fn no_charge(&mut self, view: &View) -> Option<Output> {
        let core = self.core.as_mut()?;
        match core.no_charge() {
            NoCharge::Retry => {
                self.status.retries += 1;
                self.set_target(Target::RlControl);
                self.set_gait(self.seq.approach_gait);
                self.steer = None;
                self.set_phase(Phase::StandForRetry);
            }
            NoCharge::Exhausted => {
                core.end();
                self.obstacle_open = false;
                if self.params()?.no_charge_go_back {
                    self.giving_up = true;
                    self.stand_up(Next::FindTarget, view);
                } else {
                    // sitDown: lie on the dock and wait for a charge
                    self.status.result = "no charge; lying on the dock".into();
                    self.status.error = true;
                    self.set_phase(Phase::Charging);
                }
            }
        }
        Some(self.output(None))
    }
}

/// Where the module's handlers leave the dock's inputs for the 1 kHz loop (read with try_lock: never waited on).
#[derive(Default)]
pub struct DockFeed {
    /// the newest dock pose and its sweep count
    pub pose: Mutex<Option<(DockPose, u64)>>,
    pub charge: Mutex<Option<ChargeReading>>,
    pub request: Mutex<Option<Request>>,
    /// the obstacle check's newest reading (nearest occupied cell m, or none) and its count
    pub obstacle: Mutex<Option<(Option<f32>, u64)>>,
    /// charge_manager.toml, once read (taken when docking is idle)
    pub vendor: Mutex<Option<ChargeManagerToml>>,
}

/// The docking sequence in front of the operator's command source. While it drives, the operator's velocity-only
/// commands are ignored (navigation keeps sending zeros) and a motion command (new motion_seq) cancels it and goes
/// through. Afterwards the target and gait it left are kept under the operator's velocity commands until their next
/// motion command, so a cmd_vel never re-applies an older stand request to a robot that docking laid down.
pub struct DockingCommands<C: CommandSource> {
    inner: C,
    pub docking: Docking,
    feed: Arc<DockFeed>,
    view: View,
    operator: UserCommand,
    held: Option<(Target, u32)>,
    on_status: Box<dyn FnMut(&Status) + Send>,
    last_status: Option<Status>,
    /// why the last request was refused
    pub last_refusal: Option<String>,
}

impl<C: CommandSource> DockingCommands<C> {
    pub fn new(
        inner: C,
        docking: Docking,
        feed: Arc<DockFeed>,
        on_status: Box<dyn FnMut(&Status) + Send>,
    ) -> Self {
        DockingCommands {
            inner,
            docking,
            feed,
            view: View::default(),
            operator: UserCommand::default(),
            held: None,
            on_status,
            last_status: None,
            last_refusal: None,
        }
    }

    fn command(&self, out: &Output) -> UserCommand {
        let mut params = BTreeMap::new();
        if let Some([dx, dy, yaw, height]) = out.pose {
            params.insert("dx".to_string(), dx);
            params.insert("dy".to_string(), dy);
            params.insert("yaw".to_string(), yaw);
            params.insert("height".to_string(), height);
        }
        // charge_manager's /BATTERY_CHARGE_ENABLE this tick, for the bus (1: rl_deploy's 3 charge-enable frames), with
        // its count so a command repeated after the sequence ends is not taken again
        if let Some(v) = out.charge_enable {
            params.insert(CHARGE_ENABLE_PARAM.to_string(), v as f64);
            params.insert(
                CHARGE_ENABLE_SEQ_PARAM.to_string(),
                self.docking.charge_enables as f64,
            );
        }
        UserCommand {
            target: out.target,
            gait: out.gait,
            axes: out.axes,
            velocity: None,
            params,
            runner: None,
            motion_seq: self.operator.motion_seq,
        }
    }
}

impl<C: CommandSource> CommandSource for DockingCommands<C> {
    fn poll(&mut self, now: f64) -> Option<UserCommand> {
        let mut passthrough = None;
        if let Some(c) = self.inner.poll(now) {
            let motion = c.motion_seq != self.operator.motion_seq;
            self.operator = c.clone();
            if motion {
                self.held = None;
                if self.docking.active() {
                    self.docking.finish(
                        Target::Passive,
                        "cancelled by an operator motion command",
                        false,
                    );
                }
                passthrough = Some(c);
            } else if !self.docking.active() {
                let mut c = c;
                if let Some((target, gait)) = self.held {
                    c.target = target;
                    c.gait = gait;
                }
                passthrough = Some(c);
            }
        }
        if !self.docking.active() {
            if let Some(t) = self.feed.vendor.try_lock().ok().and_then(|mut v| v.take()) {
                self.docking.set_vendor(&t);
            }
        }
        let pose = self.feed.pose.try_lock().ok().and_then(|p| *p);
        let charge = self.feed.charge.try_lock().ok().and_then(|c| *c);
        let obstacle = self.feed.obstacle.try_lock().ok().and_then(|o| *o);
        self.docking.ingest(pose, charge, obstacle);
        let request = self.feed.request.try_lock().ok().and_then(|mut r| r.take());
        if let Some(request) = request {
            if let Err(why) = self.docking.request(request, &self.view) {
                self.docking.status.result = format!("{request:?} refused: {why}");
                self.docking.status.error = true;
                self.last_refusal = Some(why);
            }
        }
        let was_active = self.docking.active();
        let out = self.docking.advance(&self.view);
        if self.last_status.as_ref() != Some(self.docking.status()) {
            (self.on_status)(self.docking.status());
            self.last_status = Some(self.docking.status().clone());
        }
        match out {
            Some(out) => {
                if !self.docking.active() {
                    self.held = Some((out.target, out.gait));
                }
                Some(self.command(&out))
            }
            None if was_active => None,
            None => passthrough,
        }
    }

    fn observe(&mut self, controller: &Controller) {
        self.view.update(controller);
    }

    fn expires(&self) -> bool {
        !self.docking.active() && self.inner.expires()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn cfg() -> Config {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        Config::load(&path).unwrap()
    }

    fn vendor() -> ChargeManagerToml {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/charge_manager.toml");
        ChargeManagerToml::load(path.to_str().unwrap()).unwrap().0
    }

    fn standing(runner: &str) -> View {
        View {
            state: Some(State::RlControl),
            fault: false,
            runner: Some(runner.into()),
            stand_in: false,
            gait: 0x3003,
            state_ticks: 10_000,
        }
    }

    fn facing(d: f64) -> DockPose {
        DockPose {
            x: d,
            y: 0.05,
            yaw: PI,
        }
    }

    /// Sweeps every 100 ticks until `phase` (or `ticks` pass).
    fn run_until(
        k: &mut Docking,
        view: &View,
        d: f64,
        seq: &mut u64,
        phase: Phase,
        ticks: u64,
    ) -> Option<Output> {
        let mut last = None;
        for t in 0..ticks {
            if t % 100 == 0 {
                *seq += 1;
            }
            last = k.step(view, Some((facing(d), *seq)), None).or(last);
            if k.phase() == phase {
                break;
            }
        }
        last
    }

    #[test]
    fn refuses_without_the_vendor_config_charge_enable_or_standing() {
        let cfg = cfg();
        let v = vendor();
        assert!(Docking::new(&cfg, true, None)
            .request(Request::Dock, &standing("navi_charge"))
            .is_err());
        assert!(Docking::new(&cfg, false, Some(&v))
            .request(Request::Dock, &standing("navi_charge"))
            .is_err());
        let mut lying = standing("navi_charge");
        lying.state = Some(State::LieDown);
        assert!(Docking::new(&cfg, true, Some(&v))
            .request(Request::Dock, &lying)
            .is_err());
        let mut k = Docking::new(&cfg, true, Some(&v));
        assert!(k.request(Request::Dock, &standing("navi_charge")).is_ok());
        assert_eq!(k.phase(), Phase::FindTarget);
        // undock only while charging or after a failure
        assert!(Docking::new(&cfg, true, Some(&v))
            .request(Request::Undock, &lying)
            .is_err());
    }

    #[test]
    fn find_target_waits_for_the_detectors_sixteenth_sweep_and_fails_without() {
        let cfg = cfg();
        let v = vendor();
        let mut k = Docking::new(&cfg, true, Some(&v));
        let view = standing("navi_charge");
        k.request(Request::Dock, &view).unwrap();
        let mut seq = 0;
        // 15 sweeps held back: still finding at 1.5 s, through on the 16th
        run_until(&mut k, &view, 2.0, &mut seq, Phase::EnterGait, 1550);
        assert_eq!(k.phase(), Phase::EnterGait);
        assert_eq!(seq, 16);
        let mut k = Docking::new(&cfg, true, Some(&v));
        k.request(Request::Dock, &view).unwrap();
        for _ in 0..2001 {
            k.step(&view, None, None);
        }
        assert_eq!(k.phase(), Phase::Idle);
        assert!(k.status().error && k.status().result.contains("not in sight"));
    }

    #[test]
    fn a_missing_pose_stops_the_robot_and_resumes() {
        let cfg = cfg();
        let v = vendor();
        let mut k = Docking::new(&cfg, true, Some(&v));
        let view = standing("navi_charge");
        k.request(Request::Dock, &view).unwrap();
        let mut seq = 0;
        run_until(&mut k, &view, 2.0, &mut seq, Phase::Approach, 10_000);
        // steering toward the staging point after the six start samples
        let out = run_until(&mut k, &view, 2.0, &mut seq, Phase::Idle, 1000).unwrap();
        assert!(out.axes[0] > 0.0, "{:?}", out.axes);
        // no pose for 150 ms: rl_deploy zeroes the steer; 200 ms: charge_manager stops; still docking
        let mut out = None;
        for _ in 0..250 {
            out = k.step(&view, Some((facing(2.0), seq)), None);
        }
        assert_eq!(out.unwrap().axes, [0.0; 3]);
        assert_eq!(k.phase(), Phase::Approach);
    }

    #[test]
    fn a_fault_or_leaving_rl_control_ends_it() {
        let cfg = cfg();
        let v = vendor();
        let mut k = Docking::new(&cfg, true, Some(&v));
        let view = standing("navi_charge");
        k.request(Request::Dock, &view).unwrap();
        let mut seq = 0;
        run_until(&mut k, &view, 2.0, &mut seq, Phase::Approach, 10_000);
        let mut v2 = view.clone();
        v2.fault = true;
        let out = k.step(&v2, Some((facing(2.0), seq)), None).unwrap();
        assert_eq!((out.target, out.axes), (Target::LieDown, [0.0; 3]));
        assert!(!k.active() && k.status().error);
        // a pose while not on NaviCharge in rl_control: charge_manager's 0x1008
        let mut k = Docking::new(&cfg, true, Some(&v));
        k.request(Request::Dock, &view).unwrap();
        let mut seq = 0;
        run_until(&mut k, &view, 2.0, &mut seq, Phase::Approach, 10_000);
        let mut v3 = view.clone();
        v3.state = Some(State::JointDamping);
        for t in 0..200u64 {
            if t % 100 == 0 {
                seq += 1;
            }
            k.step(&v3, Some((facing(2.0), seq)), None);
        }
        assert!(!k.active() && k.status().error);
    }

    #[test]
    fn an_obstacle_stops_the_approach_and_ends_it_on_basic_flat_after_ten_seconds() {
        let cfg = cfg();
        let v = vendor();
        let mut k = Docking::new(&cfg, true, Some(&v));
        let view = standing("navi_charge");
        k.request(Request::Dock, &view).unwrap();
        let mut seq = 0;
        run_until(&mut k, &view, 2.0, &mut seq, Phase::Approach, 10_000);
        assert_eq!(k.status().obstacle, Some(Direction::Front));
        let mut last = None;
        for t in 0..11_000u64 {
            if t % 100 == 0 {
                seq += 1;
            }
            k.ingest(Some((facing(2.0), seq)), None, Some((Some(0.15), seq)));
            last = k.advance(&view).or(last);
            if t == 1000 {
                assert_eq!(k.status().obstacle_stop, Some(0.15));
                assert_eq!(last.as_ref().unwrap().axes, [0.0; 3]);
            }
        }
        let last = last.unwrap();
        assert_eq!(k.phase(), Phase::Idle);
        assert_eq!((last.target, last.gait), (Target::RlControl, 0x1001));
        assert!(k.status().error);
        // a cell at 0.335 (the posts) is no obstacle
        let mut k = Docking::new(&cfg, true, Some(&v));
        k.request(Request::Dock, &view).unwrap();
        let mut seq = 0;
        run_until(&mut k, &view, 2.0, &mut seq, Phase::Approach, 10_000);
        for t in 0..2000u64 {
            if t % 100 == 0 {
                seq += 1;
            }
            k.ingest(Some((facing(2.0), seq)), None, Some((Some(0.335), seq)));
            k.advance(&view);
        }
        assert_eq!(k.status().obstacle_stop, None);
    }

    /// An inner source that replays a list, one per poll.
    struct Replay(Vec<UserCommand>);
    impl CommandSource for Replay {
        fn poll(&mut self, _now: f64) -> Option<UserCommand> {
            if self.0.is_empty() {
                None
            } else {
                Some(self.0.remove(0))
            }
        }
    }

    #[test]
    fn velocity_is_ignored_while_docking_and_a_motion_command_cancels_it() {
        let cfg = cfg();
        let feed = Arc::new(DockFeed::default());
        let walk = UserCommand {
            target: Target::RlControl,
            gait: 0x3003,
            velocity: Some([0.5, 0.0, 0.0]),
            motion_seq: 3,
            ..Default::default()
        };
        let lie = UserCommand {
            target: Target::LieDown,
            motion_seq: 4,
            ..walk.clone()
        };
        let mut c = DockingCommands::new(
            Replay(vec![walk.clone()]),
            Docking::new(&cfg, true, None),
            feed.clone(),
            Box::new(|_| {}),
        );
        c.view = standing("navi_charge");
        // the operator's walk passes while idle; charge_manager.toml arrives through the feed
        assert_eq!(c.poll(0.0).map(|u| u.velocity), Some(walk.velocity));
        *feed.vendor.lock().unwrap() = Some(vendor());
        c.poll(0.0);
        assert!(c.docking.has_vendor());
        *feed.request.lock().unwrap() = Some(Request::Dock);
        let out = c.poll(0.0).unwrap();
        assert_eq!(out.velocity, None);
        // navigation's velocity keeps arriving: ignored
        c.inner.0.push(walk.clone());
        let out = c.poll(0.0).unwrap();
        assert_eq!(out.velocity, None);
        assert!(c.docking.active() && !c.expires());
        // the operator lies it down: docking ends, the command goes through
        c.inner.0.push(lie.clone());
        let out = c.poll(0.0).unwrap();
        assert_eq!(out.target, Target::LieDown);
        assert!(!c.docking.active());
    }
}
