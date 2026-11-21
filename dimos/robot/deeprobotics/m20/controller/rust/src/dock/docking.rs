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

//! charge_manager's dock / undock sequence, stepped once per control tick ([`Docking`]), and the command source that
//! puts it in front of the operator's ([`DockingCommands`]).
//!
//! Dock (from standing in rl_control, the dock in sight and near): NaviCharge (0x3001); stand gait_settle_s; steer to
//! the staging point staging_x_m out from the docked point, then to the fine point (docked_x_m out from the posts),
//! each reached when settle_sweeps sweeps taken standing still agree it is within tolerance; FinetuneStand (0x3004),
//! whose pose inputs can shift the body onto the remaining error (finetune_adjust); lie down; once the legs are passive m20-battery's rule (in the bus
//! backend) sends the charge-enable request, and every pack's charge MOS must come on within charge_confirm_s. No
//! charge: stand, back to the staging point and approach again (charge_manager's NoChargePlug_way = goBack), `retries`
//! times, then back off as an undock does. Undock: stand on NaviCharge, back to exit_distance_m beyond the docked
//! point, BasicFlat.
//!
//! Steer is charge_manager's: normalized axes, per axis 0 inside the tolerance, else gain x error clamped to
//! [min_axes, max_axes]. Between sweeps the error is predicted from what the steer covered since the last one.
//!
//! Safety: refuses unless standing in rl_control with no fault and the stand-in over; a controller fault, the state
//! leaving the one a phase needs, or an operator motion command ends it; no dock pose for dock_lost_s while driving
//! lies the robot down (charge_manager's reaction to a target timeout) or, backing off, stops it. Every limit is a
//! count of control ticks or sweeps; none reads a clock.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use serde::Serialize;

use super::config::SequenceConfig;
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
    /// docking: switching to the approach gait, then standing gait_settle_s
    EnterGait,
    ToStaging,
    ToFine,
    Finetune,
    LyingDown,
    ConfirmCharge,
    /// no charge: standing up again to retry from the staging point
    StandForRetry,
    /// lying on the dock with every pack charging (held until undock or an operator motion command)
    Charging,
    /// undocking (or giving up): standing on the approach gait
    StandUp,
    BackOff,
    ExitGait,
}

/// What the sequence needs from the controller each tick.
#[derive(Debug, Clone, Default)]
pub struct View {
    pub state: Option<State>,
    pub fault: bool,
    pub runner: Option<String>,
    pub stand_in: bool,
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
}

/// The command the sequence sends this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Output {
    pub target: Target,
    pub gait: u32,
    pub axes: [f64; 3],
    /// FinetuneStand's pose inputs (dx, dy, yaw) while finetuning
    pub pose: Option<[f64; 3]>,
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
    /// the robot's offset from the fine point in the dock frame (x m out, y m left, yaw deg; small-angle) as the
    /// settle sweeps measured it
    pub fine_point: Option<[f64; 3]>,
}

#[derive(Default)]
struct Settle {
    sweeps: u32,
    sum: [f64; 3],
}

pub struct Docking {
    cfg: SequenceConfig,
    hz: f64,
    charge_enabled: bool,
    approach_runner: String,
    finetune_runner: String,
    exit_runner: String,
    command_max: [f64; 3],
    lie_ticks: u64,
    phase: Phase,
    ticks_in_phase: u64,
    approach_ticks: u64,
    retries: u32,
    giving_up: bool,
    pose: Option<DockPose>,
    pose_seq: Option<u64>,
    ticks_since_pose: u64,
    new_sweep: bool,
    settle: Option<Settle>,
    /// the phase's runner is up (ticks_in_phase counts from then)
    on_runner: bool,
    /// finetune: sweeps still to skip while the last adjustment takes effect
    finetune_wait: u32,
    axes: [f64; 3],
    offset: [f64; 3],
    target: Target,
    gait: u32,
    charge: Option<ChargeReading>,
    charge_seq_at_lie: u64,
    status: Status,
}

fn sign(v: f64) -> f64 {
    if v < 0.0 {
        -1.0
    } else {
        1.0
    }
}

impl Docking {
    /// `charge_enabled`: the bus backend runs m20-battery's charge-enable rule (without it nothing can charge).
    pub fn new(cfg: &Config, charge_enabled: bool) -> Docking {
        let seq = cfg.dock.sequence.clone();
        let runner = |gait: u32| cfg.runner_for_gait(gait).to_string();
        let approach_runner = runner(seq.approach_gait);
        let command_max = cfg
            .runners
            .get(&approach_runner)
            .map(|r| r.command_max)
            .unwrap_or([0.0; 3]);
        let hz = cfg.timing.control_hz;
        Docking {
            finetune_runner: runner(seq.finetune_gait),
            exit_runner: runner(seq.exit_gait),
            approach_runner,
            command_max,
            // lie-down's spline, then its damping phase: the legs are passive after both
            lie_ticks: (2.0 * cfg.states.lie_down.phase_s * hz).ceil() as u64,
            hz,
            charge_enabled,
            phase: Phase::Idle,
            ticks_in_phase: 0,
            approach_ticks: 0,
            retries: 0,
            giving_up: false,
            pose: None,
            pose_seq: None,
            ticks_since_pose: u64::MAX / 2,
            new_sweep: false,
            settle: None,
            on_runner: false,
            finetune_wait: 0,
            axes: [0.0; 3],
            offset: [0.0; 3],
            target: Target::Passive,
            gait: seq.approach_gait,
            charge: None,
            charge_seq_at_lie: 0,
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
            },
            cfg: seq,
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

    fn ticks(&self, seconds: f64) -> u64 {
        (seconds * self.hz).round() as u64
    }

    fn set_phase(&mut self, phase: Phase) {
        self.phase = phase;
        self.ticks_in_phase = 0;
        self.settle = None;
        self.on_runner = false;
        self.finetune_wait = 0;
        self.axes = [0.0; 3];
    }

    /// End the sequence with `result`; `target` is what the controller is left with.
    fn finish(&mut self, target: Target, result: impl Into<String>, error: bool) -> Option<Output> {
        self.target = target;
        self.axes = [0.0; 3];
        self.set_phase(Phase::Idle);
        self.status.result = result.into();
        self.status.error = error;
        self.giving_up = false;
        Some(self.output(None))
    }

    fn output(&self, pose: Option<[f64; 3]>) -> Output {
        Output {
            target: self.target,
            gait: self.gait,
            axes: self.axes,
            pose,
        }
    }

    fn charged(&self, r: &ChargeReading) -> bool {
        r.packs_present > 0
            && if self.cfg.all_packs_charge {
                r.packs_charge_mos_on >= r.packs_present
            } else {
                r.packs_charge_mos_on > 0
            }
    }

    /// Ask for a dock, an undock or a stop. Returns why it was refused, if it was.
    pub fn request(&mut self, request: Request, view: &View) -> Result<(), String> {
        match request {
            Request::Cancel => {
                if self.active() {
                    let lying = matches!(
                        self.phase,
                        Phase::LyingDown | Phase::ConfirmCharge | Phase::Charging
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
                let fresh = self.ticks_since_pose <= self.ticks(self.cfg.dock_lost_s);
                let Some(pose) = self.pose.filter(|_| fresh) else {
                    return Err("the dock is not in sight".into());
                };
                let distance = pose.x.hypot(pose.y);
                if distance > self.cfg.max_start_distance_m {
                    return Err(format!(
                        "the dock is {distance:.2} m away (at most {:.2})",
                        self.cfg.max_start_distance_m
                    ));
                }
                self.retries = 0;
                self.approach_ticks = 0;
                self.status.fine_point = None;
                self.status.result.clear();
                self.status.error = false;
                self.target = Target::RlControl;
                self.gait = self.cfg.approach_gait;
                self.set_phase(Phase::EnterGait);
                Ok(())
            }
            Request::Undock => {
                if self.active() && self.phase != Phase::Charging {
                    return Err(format!("busy ({:?})", self.phase));
                }
                let standable = [State::LieDown, State::Passive, State::RlControl]
                    .iter()
                    .any(|s| view.is(*s));
                if view.fault || !standable {
                    return Err(format!(
                        "undock needs the robot lying, passive or in rl_control with no fault (state {:?})",
                        view.state
                    ));
                }
                self.status.result.clear();
                self.status.error = false;
                self.giving_up = false;
                self.begin_stand_up();
                Ok(())
            }
        }
    }

    fn begin_stand_up(&mut self) {
        self.target = Target::RlControl;
        self.gait = self.cfg.approach_gait;
        self.set_phase(Phase::StandUp);
    }

    /// Steer toward `goal_x` out along the dock's axis; true once settle sweeps taken standing still agree it is
    /// within `tol` (m, m, deg).
    fn drive(&mut self, goal_x: f64, tol: [f64; 3], min_axes: [f64; 3]) -> bool {
        let tol = [tol[0], tol[1], tol[2].to_radians()];
        let Some(pose) = self.pose else {
            self.axes = [0.0; 3];
            return false;
        };
        let e = pose.error_to(goal_x);
        if let Some(s) = self.settle.as_mut() {
            self.axes = [0.0; 3];
            if self.new_sweep {
                s.sweeps += 1;
                // the first sweep after stopping is still moving: averaged from the second on
                if s.sweeps > 1 {
                    for (sum, e) in s.sum.iter_mut().zip(e) {
                        *sum += e;
                    }
                }
                if s.sweeps > self.cfg.settle_sweeps {
                    let n = (s.sweeps - 1) as f64;
                    let mean = [s.sum[0] / n, s.sum[1] / n, s.sum[2] / n];
                    self.settle = None;
                    if (0..3).all(|k| mean[k].abs() <= tol[k]) {
                        if self.phase == Phase::ToFine {
                            self.status.fine_point =
                                Some([mean[0], mean[1], -mean[2].to_degrees()]);
                        }
                        return true;
                    }
                }
            }
            return false;
        }
        // predicted error: what the steer covered since the sweep is already behind the robot
        let age = self.ticks_since_pose as f64 / self.hz;
        let mut axes = [0.0; 3];
        let mut inside = true;
        for k in 0..3 {
            let predicted = e[k] - self.axes[k] * self.command_max[k] * age;
            // driven into half the tolerance, judged on all of it: drift while settling stays inside
            if predicted.abs() <= tol[k] / 2.0 || self.command_max[k] <= 0.0 {
                continue;
            }
            inside = false;
            let wanted = self.cfg.gain[k] * predicted.abs() / self.command_max[k];
            axes[k] = sign(predicted) * wanted.clamp(min_axes[k], self.cfg.max_axes[k]);
        }
        if inside {
            self.settle = Some(Settle::default());
            self.axes = [0.0; 3];
        } else {
            self.axes = axes;
        }
        false
    }

    /// One control tick: the newest dock pose (with its sweep count) and battery reading, the controller as last seen.
    pub fn step(
        &mut self,
        view: &View,
        pose: Option<(DockPose, u64)>,
        charge: Option<ChargeReading>,
    ) -> Option<Output> {
        self.ingest(pose, charge);
        self.advance(view)
    }

    /// Take this tick's newest dock pose (with its sweep count) and battery reading.
    pub fn ingest(&mut self, pose: Option<(DockPose, u64)>, charge: Option<ChargeReading>) {
        self.new_sweep = false;
        match pose {
            Some((p, seq)) if self.pose_seq != Some(seq) => {
                self.pose = Some(p);
                self.pose_seq = Some(seq);
                self.ticks_since_pose = 0;
                self.new_sweep = true;
            }
            _ => self.ticks_since_pose = self.ticks_since_pose.saturating_add(1),
        }
        if let Some(r) = charge {
            self.charge = Some(r);
            let charging = self.charged(&r);
            self.status.charging =
                charging && matches!(self.phase, Phase::ConfirmCharge | Phase::Charging);
            self.status.packs_present = r.packs_present;
            self.status.packs_charge_mos_on = r.packs_charge_mos_on;
        }
    }

    /// Advance one control tick on what `ingest` took, the controller as last seen.
    pub fn advance(&mut self, view: &View) -> Option<Output> {
        self.status.phase = self.phase;
        self.status.active = self.active();
        self.status.retries = self.retries;
        if !self.active() {
            return None;
        }
        self.ticks_in_phase += 1;
        if view.fault {
            return self.finish(Target::LieDown, "controller fault", true);
        }
        let lost = self.ticks_since_pose > self.ticks(self.cfg.dock_lost_s);
        let transition_limit = self.ticks(self.cfg.transition_limit_s);
        let settle = self.ticks(self.cfg.gait_settle_s);
        let s = self.cfg.clone();
        match self.phase {
            Phase::Idle => None,
            Phase::EnterGait | Phase::StandForRetry | Phase::StandUp => {
                if !view.is(State::RlControl)
                    && !view.is(State::StandUp)
                    && self.phase == Phase::EnterGait
                {
                    return self.finish(
                        Target::RlControl,
                        format!("left rl_control ({:?})", view.state),
                        true,
                    );
                }
                if view.is(State::RlControl) && view.on(&self.approach_runner) {
                    // standing on the approach runner: hold still gait_settle_s, then go
                    if !self.on_runner {
                        self.on_runner = true;
                        self.ticks_in_phase = 0;
                    } else if self.ticks_in_phase >= settle {
                        self.approach_ticks = 0;
                        let next = if self.phase == Phase::StandUp {
                            Phase::BackOff
                        } else {
                            Phase::ToStaging
                        };
                        self.set_phase(next);
                    }
                } else if self.ticks_in_phase > transition_limit {
                    return self.finish(
                        Target::RlControl,
                        format!(
                            "no {} after {:.0} s",
                            self.approach_runner, s.transition_limit_s
                        ),
                        true,
                    );
                }
                Some(self.output(None))
            }
            Phase::ToStaging | Phase::ToFine => {
                if !view.is(State::RlControl) {
                    return self.finish(
                        self.target,
                        format!("left rl_control ({:?})", view.state),
                        true,
                    );
                }
                if lost {
                    return self.finish(Target::LieDown, "lost the dock while approaching", true);
                }
                self.approach_ticks += 1;
                if self.approach_ticks > self.ticks(s.approach_limit_s) {
                    return self.finish(
                        Target::RlControl,
                        format!("not at the dock after {:.0} s", s.approach_limit_s),
                        true,
                    );
                }
                let reached = if self.phase == Phase::ToStaging {
                    self.drive(
                        s.docked_x_m + s.staging_x_m,
                        s.staging_tolerance,
                        s.min_axes,
                    )
                } else {
                    self.drive(
                        s.docked_x_m - s.distance_compensation_x_m,
                        s.tolerance,
                        s.min_axes,
                    )
                };
                if reached {
                    if self.phase == Phase::ToStaging {
                        self.set_phase(Phase::ToFine);
                    } else {
                        self.offset = [0.0; 3];
                        self.gait = s.finetune_gait;
                        self.set_phase(Phase::Finetune);
                    }
                }
                Some(self.output(None))
            }
            Phase::Finetune => {
                if !view.is(State::RlControl) {
                    return self.finish(
                        self.target,
                        format!("left rl_control ({:?})", view.state),
                        true,
                    );
                }
                if lost {
                    return self.finish(Target::LieDown, "lost the dock while finetuning", true);
                }
                let params = |o: [f64; 3]| -> [f64; 3] {
                    std::array::from_fn(|k| (o[k] / s.finetune_body_per_param[k]).clamp(-1.0, 1.0))
                };
                if !self.on_runner {
                    if !view.on(&self.finetune_runner) {
                        if self.ticks_in_phase > transition_limit {
                            return self.finish(
                                Target::RlControl,
                                format!("no {}", self.finetune_runner),
                                true,
                            );
                        }
                        return Some(self.output(Some([0.0; 3])));
                    }
                    self.on_runner = true;
                    self.ticks_in_phase = 0;
                }
                // on the finetune runner: gait_settle_s standing, then adjusting until finetune_limit_s
                let limit = if s.finetune_adjust {
                    settle + self.ticks(s.finetune_limit_s)
                } else {
                    settle
                };
                if self.ticks_in_phase > limit {
                    self.target = Target::LieDown;
                    self.set_phase(Phase::LyingDown);
                    return Some(self.output(None));
                }
                if s.finetune_adjust && self.ticks_in_phase > settle && self.new_sweep {
                    if let Some(pose) = self.pose {
                        let e = pose.error_to(s.docked_x_m - s.distance_compensation_x_m);
                        let tol = [s.tolerance[0], s.tolerance[1], s.tolerance[2].to_radians()];
                        self.finetune_sweep(e, tol);
                    }
                }
                if self.phase == Phase::LyingDown {
                    return Some(self.output(None));
                }
                Some(self.output(Some(params(self.offset))))
            }
            Phase::LyingDown => {
                if view.is(State::LieDown) && view.state_ticks >= self.lie_ticks {
                    self.charge_seq_at_lie = self.charge.map_or(0, |r| r.seq);
                    self.set_phase(Phase::ConfirmCharge);
                } else if !view.is(State::LieDown) && self.ticks_in_phase > transition_limit {
                    return self.finish(
                        Target::LieDown,
                        format!("did not lie down ({:?})", view.state),
                        true,
                    );
                }
                Some(self.output(None))
            }
            Phase::ConfirmCharge => {
                if !view.is(State::LieDown) {
                    return self.finish(
                        self.target,
                        format!("left lie_down ({:?})", view.state),
                        true,
                    );
                }
                let fresh = self.charge.filter(|r| r.seq > self.charge_seq_at_lie);
                if fresh.is_some_and(|r| self.charged(&r)) {
                    self.status.charging = true;
                    self.status.result = "docked, charging".into();
                    self.status.error = false;
                    self.set_phase(Phase::Charging);
                } else if self.ticks_in_phase > self.ticks(s.charge_confirm_s) {
                    if self.retries < s.retries {
                        self.retries += 1;
                        self.target = Target::RlControl;
                        self.gait = s.approach_gait;
                        self.set_phase(Phase::StandForRetry);
                    } else {
                        // give up: back off as an undock does, then report
                        self.giving_up = true;
                        self.begin_stand_up();
                    }
                }
                Some(self.output(None))
            }
            Phase::Charging => {
                if !view.is(State::LieDown) && !view.is(State::Passive) {
                    return self.finish(
                        self.target,
                        format!("left lie_down while charging ({:?})", view.state),
                        true,
                    );
                }
                Some(self.output(None))
            }
            Phase::BackOff => {
                if !view.is(State::RlControl) {
                    return self.finish(
                        self.target,
                        format!("left rl_control ({:?})", view.state),
                        true,
                    );
                }
                if lost {
                    // too close for the lidar to see the posts at first: back straight out until it does
                    if self.ticks_in_phase <= self.ticks(s.dock_lost_s) {
                        self.axes = [-s.exit_min_axes[0], 0.0, 0.0];
                        return Some(self.output(None));
                    }
                    return self.finish(Target::RlControl, "lost the dock while backing off", true);
                }
                if self.drive(
                    s.docked_x_m + s.exit_distance_m,
                    s.exit_tolerance,
                    s.exit_min_axes,
                ) {
                    self.gait = s.exit_gait;
                    self.set_phase(Phase::ExitGait);
                }
                Some(self.output(None))
            }
            Phase::ExitGait => {
                if view.is(State::RlControl) && view.on(&self.exit_runner) {
                    let (result, error) = if self.giving_up {
                        (
                            format!("no charge after {} tries; backed off", self.retries + 1),
                            true,
                        )
                    } else {
                        ("undocked".to_string(), false)
                    };
                    return self.finish(Target::RlControl, result, error);
                }
                if self.ticks_in_phase > transition_limit {
                    return self.finish(
                        Target::RlControl,
                        format!("no {}", self.exit_runner),
                        true,
                    );
                }
                Some(self.output(None))
            }
        }
    }

    /// One finetune adjustment from a sweep's error: inside the tolerance lie down, else shift the body onto it and
    /// let settle_sweeps sweeps pass before judging again.
    fn finetune_sweep(&mut self, e: [f64; 3], tol: [f64; 3]) {
        if self.finetune_wait > 0 {
            self.finetune_wait -= 1;
            return;
        }
        self.finetune_wait = self.cfg.settle_sweeps;
        if (0..3).all(|k| e[k].abs() <= tol[k]) {
            self.target = Target::LieDown;
            self.set_phase(Phase::LyingDown);
            return;
        }
        for k in 0..3 {
            if e[k].abs() > tol[k] {
                self.offset[k] += e[k];
            }
        }
    }
}

/// Where the module's handlers leave the dock's inputs for the 1 kHz loop (read with try_lock: never waited on).
#[derive(Default)]
pub struct DockFeed {
    /// the newest dock pose and its sweep count
    pub pose: Mutex<Option<(DockPose, u64)>>,
    pub charge: Mutex<Option<ChargeReading>>,
    pub request: Mutex<Option<Request>>,
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
        if let Some([dx, dy, yaw]) = out.pose {
            params.insert("dx".to_string(), dx);
            params.insert("dy".to_string(), dy);
            params.insert("yaw".to_string(), yaw);
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
        let pose = self.feed.pose.try_lock().ok().and_then(|p| *p);
        let charge = self.feed.charge.try_lock().ok().and_then(|c| *c);
        self.docking.ingest(pose, charge);
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

    fn standing(runner: &str) -> View {
        View {
            state: Some(State::RlControl),
            fault: false,
            runner: Some(runner.into()),
            stand_in: false,
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

    /// A docking that has seen the dock `d` m ahead.
    fn seen(cfg: &Config, d: f64) -> Docking {
        let mut k = Docking::new(cfg, true);
        k.step(&standing("navi_charge"), Some((facing(d), 1)), None);
        k
    }

    #[test]
    fn refuses_unless_standing_in_rl_control_with_the_dock_near() {
        let cfg = cfg();
        let mut k = seen(&cfg, 2.0);
        let mut lying = standing("navi_charge");
        lying.state = Some(State::LieDown);
        assert!(k.request(Request::Dock, &lying).is_err());
        let mut faulted = standing("navi_charge");
        faulted.fault = true;
        assert!(k.request(Request::Dock, &faulted).is_err());
        let mut stand_in = standing("six_axis_stand");
        stand_in.stand_in = true;
        assert!(k.request(Request::Dock, &stand_in).is_err());
        assert!(seen(&cfg, 5.0)
            .request(Request::Dock, &standing("navi_charge"))
            .is_err());
        // never saw it
        assert!(Docking::new(&cfg, true)
            .request(Request::Dock, &standing("navi_charge"))
            .is_err());
        // seen too long ago
        let mut stale = seen(&cfg, 2.0);
        for _ in 0..=stale.ticks(cfg.dock.sequence.dock_lost_s) {
            stale.step(&standing("navi_charge"), Some((facing(2.0), 1)), None);
        }
        assert!(stale
            .request(Request::Dock, &standing("navi_charge"))
            .is_err());
        // no charge-enable rule: nothing would charge
        let mut off = Docking::new(&cfg, false);
        off.step(&standing("navi_charge"), Some((facing(2.0), 1)), None);
        assert!(off
            .request(Request::Dock, &standing("navi_charge"))
            .is_err());
        assert!(k.request(Request::Dock, &standing("navi_charge")).is_ok());
        assert_eq!(k.phase(), Phase::EnterGait);
    }

    /// Runs the sequence to the approach with the dock at `d` (fresh sweeps every 100 ticks).
    fn approaching(cfg: &Config, d: f64) -> (Docking, u64) {
        let mut k = seen(cfg, d);
        let view = standing("navi_charge");
        k.request(Request::Dock, &view).unwrap();
        let mut seq = 1;
        for t in 0..3000u64 {
            if t % 100 == 0 {
                seq += 1;
            }
            let out = k.step(&view, Some((facing(d), seq)), None).unwrap();
            assert_eq!((out.target, out.gait), (Target::RlControl, 0x3001));
        }
        assert_eq!(k.phase(), Phase::ToStaging);
        (k, seq)
    }

    #[test]
    fn steers_toward_the_staging_point_at_least_the_minimum() {
        let cfg = cfg();
        let (mut k, seq) = approaching(&cfg, 2.0);
        let off = DockPose {
            x: 2.8,
            y: 0.2,
            yaw: PI,
        };
        let out = k
            .step(&standing("navi_charge"), Some((off, seq + 1)), None)
            .unwrap();
        let s = &cfg.dock.sequence;
        // ~1 m to the staging point: forward, capped; 0.2 m to the left: at least the minimum lateral
        assert!((out.axes[0] - s.max_axes[0]).abs() < 1e-9, "{:?}", out.axes);
        assert!(out.axes[1] >= s.min_axes[1] - 1e-9, "{:?}", out.axes);
        assert_eq!(out.axes[2], 0.0);
    }

    #[test]
    fn losing_the_dock_while_approaching_lies_down() {
        let cfg = cfg();
        let (mut k, seq) = approaching(&cfg, 2.0);
        let mut last = None;
        for _ in 0..=k.ticks(cfg.dock.sequence.dock_lost_s) + 1 {
            if let Some(out) = k.step(&standing("navi_charge"), Some((facing(2.0), seq)), None) {
                last = Some(out);
            }
        }
        assert_eq!(k.phase(), Phase::Idle);
        assert_eq!(
            last.map(|o| (o.target, o.axes)),
            Some((Target::LieDown, [0.0; 3]))
        );
        assert!(k.status().error && k.status().result.contains("lost"));
    }

    #[test]
    fn a_fault_or_leaving_rl_control_ends_it() {
        let cfg = cfg();
        let (mut k, seq) = approaching(&cfg, 2.0);
        let mut v = standing("navi_charge");
        v.fault = true;
        let out = k.step(&v, Some((facing(2.0), seq)), None).unwrap();
        assert_eq!((out.target, out.axes), (Target::LieDown, [0.0; 3]));
        assert!(!k.active() && k.status().error);
        let (mut k, seq) = approaching(&cfg, 2.0);
        let mut v = standing("navi_charge");
        v.state = Some(State::JointDamping);
        k.step(&v, Some((facing(2.0), seq)), None);
        assert!(!k.active() && k.status().error);
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
            Docking::new(&cfg, true),
            feed.clone(),
            Box::new(|_| {}),
        );
        c.view = standing("navi_charge");
        // the operator's walk passes while idle
        assert_eq!(c.poll(0.0).map(|u| u.velocity), Some(walk.velocity));
        *feed.pose.lock().unwrap() = Some((facing(2.0), 1));
        c.poll(0.0);
        *feed.request.lock().unwrap() = Some(Request::Dock);
        let out = c.poll(0.0).unwrap();
        assert_eq!((out.gait, out.velocity), (0x3001, None));
        // navigation's velocity keeps arriving: ignored
        c.inner.0.push(walk.clone());
        let out = c.poll(0.0).unwrap();
        assert_eq!((out.gait, out.velocity), (0x3001, None));
        assert!(c.docking.active() && !c.expires());
        // the operator lies it down: docking ends, the command goes through
        c.inner.0.push(lie.clone());
        let out = c.poll(0.0).unwrap();
        assert_eq!(out.target, Target::LieDown);
        assert!(!c.docking.active());
    }

    #[test]
    fn after_docking_a_velocity_command_keeps_the_target_docking_left() {
        let cfg = cfg();
        let feed = Arc::new(DockFeed::default());
        let stand = UserCommand {
            target: Target::RlControl,
            gait: 0x3003,
            motion_seq: 2,
            ..Default::default()
        };
        let mut c = DockingCommands::new(
            Replay(vec![stand.clone()]),
            Docking::new(&cfg, true),
            feed.clone(),
            Box::new(|_| {}),
        );
        c.view = standing("navi_charge");
        c.poll(0.0);
        *feed.pose.lock().unwrap() = Some((facing(2.0), 1));
        *feed.request.lock().unwrap() = Some(Request::Dock);
        c.poll(0.0);
        // lost dock: docking lies the robot down and ends
        for _ in 0..=c.docking.ticks(cfg.dock.sequence.dock_lost_s) + 3001 {
            c.poll(0.0);
        }
        assert!(!c.docking.active());
        // a cmd_vel (same motion_seq) must not stand it back up
        c.inner.0.push(UserCommand {
            velocity: Some([0.2, 0.0, 0.0]),
            ..stand
        });
        let out = c.poll(0.0).unwrap();
        assert_eq!(out.target, Target::LieDown);
    }
}
