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

//! The state machine's rl_deploy behavior, rule by rule.

use super::*;
use crate::policy::{InlinePolicy, RunnerSet};

pub(super) fn config() -> Config {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
    Config::load(&path).unwrap()
}

pub(super) fn standing_state(cfg: &Config) -> RobotState {
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
    // the gait's runner ran (and, with no networks loaded, failed into a policy fault): lie down, then damp
    assert_eq!(c.state, State::LieDown);
    assert!(c.fault.as_deref().is_some_and(|f| f.starts_with("policy")));
    let lie_ticks = (2.0 * c.cfg.states.lie_down.phase_s * 1000.0) as usize;
    let mut s = s;
    for _ in 0..lie_ticks {
        s.stamp += 0.001;
        c.tick(&s, &cmd, &mut policy);
    }
    assert_eq!(c.state, State::JointDamping);
}

/// Answers every request with the pose held (no networks), so rl_control runs without policies.
pub(super) struct HoldPolicy(pub(super) Option<PolicyRequest>);

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
                command_zero: true,
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
pub(super) fn run(
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
    let cfg = config().rl_deploy();
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
