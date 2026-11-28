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

//! The controller's status as one record (m20tool status's view of rl_deploy), published as JSON and logged on change.

use serde::Serialize;

use crate::config::{Config, JOINTS};
use crate::dock::docking::Status as DockStatus;
use crate::dock::obstacle;
use crate::rt::Tick;
use crate::state_machine::State;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ControllerStatus {
    /// the state machine's state (Passive, StandUp, RlControl, LieDown, JointDamping, StartDamping)
    pub state: String,
    pub fault: Option<String>,
    /// in rl_control without a fault (cmd_vel is followed)
    pub ready: bool,
    /// the operator's (or docking's) target state
    pub target: String,
    /// the gait code commanded (hex)
    pub commanded_gait: String,
    /// the gait code rl_control runs (hex; the commanded one outside rl_control)
    pub gait: String,
    /// the runner whose policy runs (none outside rl_control)
    pub runner: Option<String>,
    /// rl_control's stand-in policy (on entry or a gait switch) is running before the gait's own runner
    pub stand_in: bool,
    /// the commanded velocity (m/s, m/s, rad/s) after the command timeout, or the normalized axes when none
    pub velocity: [f64; 3],
    /// base_link above the wheels' contact from the measured joints (m; leg FK, mean of the legs)
    pub height_incl_wheel_m: f64,
    /// each leg's hip above its wheel axle from the measured joints (m; FL, FR, HL, HR)
    pub leg_hip_to_axle_m: [f64; 4],
    /// the same from the commanded joint targets (m); none while no leg holds a position (kp 0)
    pub commanded_height_incl_wheel_m: Option<f64>,
    /// basic_server's usage mode (0 regular, 1 navigation, 2 assist)
    pub usage_mode: i32,
    pub dock_phase: Option<String>,
    /// charge_manager's state for it (0 idle, 1 docking, 2 charging, 3 undocking, 5 on the dock without a charge)
    pub charger_state: Option<i32>,
    /// rl_deploy's MotionInfo height (GetBodyHeight: hip above the wheel contact, tilt-compensated)
    pub motion_info_height_m: f64,
    /// the newest IMU sample's age at this tick (ms; none before the first): what the policy step reads
    pub imu_age_ms: Option<f64>,
    /// /HES_STATUS: the ECAN board's hard-stop byte (0 clear, 1 pressed; none before the first 0x8a frame)
    pub hard_stop: Option<u8>,
    /// the 1 kHz loop's overruns so far and its slowest tick (ms)
    pub overruns: u64,
    /// the 1 kHz loop's reads skipped so far because a receiver held the lock (rt::try_read)
    pub lock_skips: u64,
    pub max_tick_ms: f64,
    pub dock_active: bool,
    pub charging: bool,
    /// rl_deploy's battery level (normal, warning under 21 %, error under 6 % or at 54 V); none before a report
    pub battery: Option<crate::battery::BatteryLevel>,
    /// the app's joint zeroing (ResetJointsZero): running, how many ended, the last one's code
    pub joint_zero: crate::joint_zero::Report,
}

/// Each leg's hip-to-axle depth from its joints, as obstacle::body_height averages it.
pub fn leg_heights(cfg: &Config, q: &[f64; JOINTS]) -> [f64; 4] {
    let (l1, l2) = (cfg.robot.thigh_length, cfg.robot.shank_length);
    std::array::from_fn(|leg| {
        let (hipx, hipy, knee) = (q[4 * leg], q[4 * leg + 1], q[4 * leg + 2]);
        (l1 * hipy.cos() + l2 * (hipy + knee).cos()) * hipx.cos()
    })
}

impl ControllerStatus {
    pub fn of(
        cfg: &Config,
        tick: &Tick,
        dock: Option<&DockStatus>,
        usage_mode: i32,
    ) -> ControllerStatus {
        let c = tick.controller;
        let wheel = cfg.dock.obstacle.wheel_radius_m;
        let holds = (0..JOINTS).any(|i| i % 4 != 3 && tick.command.kp[i] > 0.0);
        let round = |h: f64| (h * 1e4).round() / 1e4;
        let gait = if c.state == State::RlControl {
            c.gait()
        } else {
            tick.user.gait
        };
        ControllerStatus {
            state: format!("{:?}", c.state),
            fault: c.fault.clone(),
            ready: c.state == State::RlControl && c.fault.is_none(),
            target: format!("{:?}", tick.user.target),
            commanded_gait: format!("{:#06x}", tick.user.gait),
            gait: format!("{gait:#06x}"),
            runner: (c.state == State::RlControl)
                .then(|| c.current_runner().map(str::to_string))
                .flatten(),
            stand_in: c.in_stand_in(),
            velocity: tick.user.velocity.unwrap_or(tick.user.axes),
            height_incl_wheel_m: round(obstacle::body_height(&cfg.robot, &tick.state.q, wheel)),
            leg_hip_to_axle_m: leg_heights(cfg, &tick.state.q).map(round),
            commanded_height_incl_wheel_m: holds
                .then(|| round(obstacle::body_height(&cfg.robot, &tick.command.q, wheel))),
            usage_mode,
            dock_phase: dock.map(|d| format!("{:?}", d.phase)),
            charger_state: dock.map(crate::basic_server::charger_state_of),
            motion_info_height_m: round(c.motion_info_height() as f64),
            imu_age_ms: None,
            hard_stop: None,
            overruns: tick.stats.overruns,
            lock_skips: crate::rt::LOCK_SKIPS.load(std::sync::atomic::Ordering::Relaxed),
            max_tick_ms: round(tick.stats.max_tick_s * 1e3),
            dock_active: dock.is_some_and(|d| d.active),
            charging: dock.is_some_and(|d| d.charging),
            battery: tick.state.battery,
            joint_zero: Default::default(),
        }
    }

    /// The fields whose change is logged at once (all but the velocity and the heights).
    fn key(&self) -> impl PartialEq + '_ {
        (
            &self.state,
            &self.fault,
            &self.target,
            &self.commanded_gait,
            &self.gait,
            &self.runner,
            self.stand_in,
            (
                &self.dock_phase,
                self.dock_active,
                self.charging,
                self.usage_mode,
            ),
            self.hard_stop,
            self.battery,
            self.joint_zero,
        )
    }
}

/// Logs a status on any change but velocity, or a commanded height `height_step_m` from the last logged.
pub struct StatusLog {
    height_step_m: f64,
    last: Option<ControllerStatus>,
}

impl StatusLog {
    pub fn new(height_step_m: f64) -> StatusLog {
        StatusLog {
            height_step_m,
            last: None,
        }
    }

    /// Whether `now` is to be logged (it becomes the one later statuses are compared to).
    pub fn changed(&mut self, now: &ControllerStatus) -> bool {
        let log = match &self.last {
            None => true,
            Some(last) => {
                let height_moved = match (
                    last.commanded_height_incl_wheel_m,
                    now.commanded_height_incl_wheel_m,
                ) {
                    (Some(a), Some(b)) => (a - b).abs() >= self.height_step_m,
                    (a, b) => a.is_some() != b.is_some(),
                };
                last.key() != now.key() || height_moved
            }
        };
        if log {
            self.last = Some(now.clone());
        }
        log
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy::{InlinePolicy, RunnerSet};
    use crate::rt::LoopStats;
    use crate::state_machine::Controller;
    use crate::types::{RobotState, Target, UserCommand};

    fn config() -> Config {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        Config::load(&path).unwrap()
    }

    /// Stand-up from lying: every status the 1 kHz loop would build, and the ones logged.
    #[test]
    fn stand_up_logs_each_state_and_the_rising_commanded_height() {
        let cfg = config();
        let mut s = RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        // lying: the lie-down pose
        s.q = crate::state_machine::leg_pose_for_height(&cfg.robot, 0.03, 0.0);
        let mut c = Controller::new(cfg.clone());
        c.set_time(10.0);
        let cmd = UserCommand {
            target: Target::Stand,
            gait: 0x1001,
            ..Default::default()
        };
        let mut policy = InlinePolicy::new(RunnerSet::empty());
        let mut log = StatusLog::new(0.01);
        let stats = LoopStats::default();
        let mut logged = Vec::new();
        for _ in 0..3500 {
            let command = c.tick(&s, &cmd, &mut policy);
            s.stamp += 0.001;
            // the joints follow the command (and move, so no frozen-joint fault)
            if command.kp[1] > 0.0 {
                s.q = command.q;
            }
            s.tau = [s.stamp; JOINTS];
            let tick = Tick {
                state: &s,
                command: &command,
                user: &cmd,
                controller: &c,
                stats: &stats,
            };
            let now = ControllerStatus::of(&cfg, &tick, None, 0);
            if log.changed(&now) {
                logged.push(now);
            }
        }
        assert_eq!(logged[0].state, "StandUp");
        assert_eq!(logged[0].gait, "0x1001");
        assert_eq!(logged[0].target, "Stand");
        let heights: Vec<f64> = logged
            .iter()
            .filter_map(|l| l.commanded_height_incl_wheel_m)
            .collect();
        // the IK height 0.38 plus the wheel radius, reached at the end
        let last = *heights.last().unwrap();
        let expected = cfg.states.stand_up.stand_height + cfg.dock.obstacle.wheel_radius_m;
        assert!((last - expected).abs() < 0.011, "{last} vs {expected}");
        assert!(
            heights.len() > 10,
            "the rising height is logged: {heights:?}"
        );
        assert!(logged.len() < 80, "not every tick: {}", logged.len());
        let json = serde_json::to_string(&logged[0]).unwrap();
        assert!(json.contains("\"commanded_gait\":\"0x1001\""), "{json}");
    }

    #[test]
    fn a_gait_or_runner_change_alone_is_logged() {
        let base = ControllerStatus {
            state: "RlControl".into(),
            fault: None,
            ready: true,
            target: "RlControl".into(),
            commanded_gait: "0x1001".into(),
            gait: "0x1001".into(),
            runner: Some("basic_flat".into()),
            stand_in: false,
            velocity: [0.0; 3],
            height_incl_wheel_m: 0.45,
            leg_hip_to_axle_m: [0.38; 4],
            commanded_height_incl_wheel_m: Some(0.45),
            usage_mode: 0,
            dock_phase: None,
            charger_state: None,
            motion_info_height_m: Default::default(),
            imu_age_ms: None,
            hard_stop: None,
            overruns: 0,
            lock_skips: 0,
            max_tick_ms: Default::default(),
            dock_active: false,
            charging: false,
            battery: None,
            joint_zero: Default::default(),
        };
        let mut log = StatusLog::new(0.01);
        assert!(log.changed(&base));
        let mut moving = base.clone();
        moving.velocity = [0.5, 0.0, 0.0];
        moving.commanded_height_incl_wheel_m = Some(0.455);
        assert!(
            !log.changed(&moving),
            "velocity and a small height change are not"
        );
        let mut switched = base.clone();
        switched.commanded_gait = "0x3002".into();
        assert!(log.changed(&switched));
        let mut runner = switched.clone();
        runner.runner = Some("navi_flat".into());
        assert!(log.changed(&runner));
        let mut lower = runner.clone();
        lower.commanded_height_incl_wheel_m = Some(0.43);
        assert!(log.changed(&lower));
    }
}
