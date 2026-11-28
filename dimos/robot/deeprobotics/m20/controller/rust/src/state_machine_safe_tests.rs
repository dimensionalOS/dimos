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

//! The state machine's differences from rl_deploy (ROBUSTNESS.md): safe_switch, finite_commands, lie-down-first damping.

use super::tests::{config, run, standing_state, HoldPolicy};
use super::*;

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
    // faster than settle_below: only stillness lies it down
    let rolling = ss.settle_below_m_s / ss.still.wheel_radius_m * 2.0;
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

/// rl_control standing still on 0x3002 (no stand-in), the policy holding the pose.
fn standing_in_rl(cfg: Config) -> (Controller, RobotState, HoldPolicy, UserCommand) {
    let mut s = standing_state(&cfg);
    let mut c = Controller::new(cfg);
    c.set_time(10.0);
    let mut p = HoldPolicy(None);
    let cmd = UserCommand {
        target: Target::RlControl,
        gait: 0x3002,
        ..Default::default()
    };
    c.force(State::RlControl, &s, &cmd, false);
    c.stand_in = None;
    run(&mut c, &mut s, &cmd, 100, &mut p);
    (c, s, p, cmd)
}

#[test]
fn joint_damping_lies_down_first_unless_the_drivers_fault() {
    let cfg = config();
    let lie_ticks = (2.0 * cfg.states.lie_down.phase_s * 1000.0) as usize;
    // the operator's damping: lie down, then damp
    let (mut c, mut s, mut p, mut cmd) = standing_in_rl(cfg.clone());
    cmd.target = Target::JointDamping;
    run(&mut c, &mut s, &cmd, 1, &mut p);
    assert_eq!(c.state, State::LieDown);
    run(&mut c, &mut s, &cmd, lie_ticks / 2 + 2, &mut p);
    assert_eq!(c.state, State::JointDamping);
    // rl_deploy's: damping at once
    let (mut c, mut s, mut p, mut cmd) = standing_in_rl(cfg.clone().rl_deploy());
    cmd.target = Target::JointDamping;
    run(&mut c, &mut s, &cmd, 1, &mut p);
    assert_eq!(c.state, State::JointDamping);
    // a driver status-word fault: the motors cannot hold, damping at once
    let (mut c, mut s, mut p, cmd) = standing_in_rl(cfg.clone());
    s.drivers = Some(crate::types::Drivers {
        status_word: [1; JOINTS],
        motor_temperature_c: [40.0; JOINTS],
        driver_temperature_c: [40.0; JOINTS],
    });
    if let Some(d) = s.drivers.as_mut() {
        d.status_word[2] = 0x21;
    }
    run(&mut c, &mut s, &cmd, 1, &mut p);
    assert_eq!(c.state, State::JointDamping);
    // a joint beyond its range (the motors still act): lie down first
    let (mut c, mut s, mut p, cmd) = standing_in_rl(cfg);
    s.q[1] = c.cfg.robot.joint_upper[1] + 1.0;
    run(&mut c, &mut s, &cmd, 40, &mut p);
    assert!(c.fault.is_some());
    assert_eq!(c.state, State::LieDown);
}

/// A policy that takes every request and never answers (a stalled thread).
struct Silent;

impl PolicyService for Silent {
    fn offer(&mut self, _: PolicyRequest) -> bool {
        true
    }
    fn take_result(&mut self) -> Option<crate::policy::PolicyResult> {
        None
    }
    fn dropped(&self) -> u64 {
        0
    }
}

#[test]
fn a_stalled_imu_or_policy_lies_down_at_once() {
    let cfg = config();
    let timeout = cfg.safety.imu_timeout_s.expect("imu_timeout_s on");
    let roll = |s: &mut RobotState| {
        for i in (3..JOINTS).step_by(4) {
            s.dq[i] = 20.0;
        }
    };
    // a stale IMU while rolling: no stop first, the lie-down starts at once
    let (mut c, mut s, mut p, cmd) = standing_in_rl(cfg.clone());
    roll(&mut s);
    s.imu_age_s = Some(2.0 * timeout);
    run(&mut c, &mut s, &cmd, 1, &mut p);
    assert!(c.fault.as_deref().is_some_and(|f| f.contains("imu")));
    assert_eq!(c.state, State::LieDown);
    // sensor_fault_lies_down off: damping at once
    let mut damp = cfg.clone();
    damp.states.joint_damping.sensor_fault_lies_down = false;
    let (mut c, mut s, mut p, cmd) = standing_in_rl(damp);
    s.imu_age_s = Some(2.0 * timeout);
    run(&mut c, &mut s, &cmd, 1, &mut p);
    assert_eq!(c.state, State::JointDamping);
    // rl_deploy's: no IMU fault
    let (mut c, mut s, mut p, cmd) = standing_in_rl(cfg.clone().rl_deploy());
    s.imu_age_s = Some(2.0 * timeout);
    run(&mut c, &mut s, &cmd, 100, &mut p);
    assert_eq!(c.state, State::RlControl);
    // a policy that stops answering: a policy fault after policy_missed_steps periods, then the lie-down
    let n = cfg
        .safety
        .policy_missed_steps
        .expect("policy_missed_steps on");
    let period = cfg.timing.policy_decimation as usize;
    let (mut c, mut s, _, cmd) = standing_in_rl(cfg);
    let mut silent = Silent;
    // the last result landed up to a period before
    for _ in 0..((n as usize - 1) * period) {
        s.stamp += 0.001;
        s.tau = [s.stamp; JOINTS];
        c.tick(&s, &cmd, &mut silent);
    }
    assert_eq!(c.state, State::RlControl, "within its periods");
    for _ in 0..(2 * period) {
        s.stamp += 0.001;
        s.tau = [s.stamp; JOINTS];
        c.tick(&s, &cmd, &mut silent);
    }
    assert!(c
        .fault
        .as_deref()
        .is_some_and(|f| f.starts_with("policy stalled")));
    assert_eq!(c.state, State::LieDown);
}

#[test]
fn a_joint_data_fault_lies_down_without_the_stop() {
    let cfg = config();
    let (mut c, mut s, mut p, cmd) = standing_in_rl(cfg);
    // rolling (no stop would end) with a hipy past its range
    for i in (3..JOINTS).step_by(4) {
        s.dq[i] = 20.0;
    }
    s.q[1] = c.cfg.robot.joint_upper[1] + 1.0;
    run(&mut c, &mut s, &cmd, 40, &mut p);
    assert!(c.fault.as_deref().is_some_and(|f| f.contains("range")));
    assert_eq!(c.state, State::LieDown);
}

#[test]
fn a_battery_at_its_error_level_lies_down_and_refuses_a_stand() {
    use crate::battery::BatteryLevel;
    let cfg = config();
    // standing: a stop, then the lie-down
    let (mut c, mut s, mut p, cmd) = standing_in_rl(cfg.clone());
    s.battery = Some(BatteryLevel::Warning);
    run(&mut c, &mut s, &cmd, 100, &mut p);
    assert_eq!(c.state, State::RlControl, "a warning only reports");
    s.battery = Some(BatteryLevel::Error);
    run(&mut c, &mut s, &cmd, 1, &mut p);
    assert_eq!(c.state, State::LieDown);
    // lying: a stand is refused
    let stand = UserCommand {
        target: Target::Stand,
        ..Default::default()
    };
    let lie_ticks = (2.0 * cfg.states.lie_down.phase_s * 1000.0) as usize + 10;
    run(&mut c, &mut s, &stand, lie_ticks, &mut p);
    assert_eq!(c.state, State::LieDown);
    // rl_deploy's: it only reports
    let (mut c, mut s, mut p, cmd) = standing_in_rl(cfg.rl_deploy());
    s.battery = Some(BatteryLevel::Error);
    run(&mut c, &mut s, &cmd, 100, &mut p);
    assert_eq!(c.state, State::RlControl);
}
