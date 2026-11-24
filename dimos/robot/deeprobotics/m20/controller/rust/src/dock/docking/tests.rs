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

use super::*;
use std::f64::consts::PI;

fn cfg() -> Config {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
    Config::load(&path).unwrap()
}

fn vendor() -> ChargeManagerToml {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/charge_manager.toml");
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
fn a_fault_alone_changes_nothing_but_leaving_rl_control_ends_it() {
    let cfg = cfg();
    let v = vendor();
    let mut k = Docking::new(&cfg, true, Some(&v));
    let view = standing("navi_charge");
    k.request(Request::Dock, &view).unwrap();
    let mut seq = 0;
    run_until(&mut k, &view, 2.0, &mut seq, Phase::Approach, 10_000);
    // charge_manager has no fault input: a fault flag alone changes nothing
    let mut v2 = view.clone();
    v2.fault = true;
    k.step(&v2, Some((facing(2.0), seq)), None).unwrap();
    assert_eq!(k.phase(), Phase::Approach);
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

fn lying() -> View {
    let mut v = standing("navi_charge");
    v.state = Some(State::LieDown);
    v
}

/// The robot `tx`, `ty` (m) off the docked point, facing the dock.
fn docked(tx: f64, ty: f64) -> DockPose {
    DockPose {
        x: cfg().dock.sequence.docked_x_m - tx,
        y: -ty,
        yaw: PI,
    }
}

/// charge_manager's start-up on the dock: a sweep and a battery report every 100 ms
struct Boot {
    k: Docking,
    tick: u64,
    seq: u64,
    enables: Vec<u64>,
}

impl Boot {
    fn new() -> Boot {
        Boot {
            k: Docking::new(&cfg(), true, Some(&vendor())),
            tick: 0,
            seq: 0,
            enables: vec![],
        }
    }

    /// Run to `until_ms`; pose `pose`, charge MOS on at/after `charge_at_ms`.
    fn run(&mut self, until_ms: u64, pose: Option<DockPose>, charge_at_ms: u64) {
        self.run_as(&lying(), until_ms, pose, charge_at_ms, u64::MAX);
    }

    /// Run to `until_ms` as `view`; the charge MOS on from `charge_at_ms` until `charge_off_ms`.
    fn run_as(
        &mut self,
        view: &View,
        until_ms: u64,
        pose: Option<DockPose>,
        charge_at_ms: u64,
        charge_off_ms: u64,
    ) {
        while self.tick < until_ms {
            let mut feed_pose = None;
            let mut feed_charge = None;
            if self.tick % 100 == 0 {
                self.seq += 1;
                feed_pose = pose.map(|p| (p, self.seq));
                let on = if self.tick >= charge_at_ms && self.tick < charge_off_ms {
                    2
                } else {
                    0
                };
                feed_charge = Some(ChargeReading {
                    seq: self.seq,
                    packs_present: 2,
                    packs_charge_mos_on: on,
                    charger_connected: on > 0,
                });
            }
            self.k.step(view, feed_pose, feed_charge);
            if self.k.charge_enable_now == Some(1) {
                self.enables.push(self.tick);
            }
            self.tick += 1;
        }
    }
}

#[test]
fn start_up_lying_on_the_dock_charging_takes_the_charge_and_undocks() {
    let mut b = Boot::new();
    // PowerOnSelfTest: Open12vPower(3) (1 at 0, 2, 4 s), DetectionCharge from 7 s: "Detection charge success..."
    b.run(7001, None, 0);
    // (the harness's ms 0 is the sequence's tick 1)
    assert_eq!(b.enables, vec![0, 1999, 3999]);
    assert_eq!(b.k.status().result, "Detection charge success...");
    assert_eq!(b.k.phase(), Phase::Idle);
    // the idle loop's first wake 3 s on: 空闲状态直接进入充电状态, state:0 -> 2 (as 2026-11-04 02:50:06 -> 02:50:09)
    b.run(9_999, None, 0);
    assert_eq!(b.k.phase(), Phase::Idle);
    b.run(10_000, None, 0);
    assert_eq!(b.k.phase(), Phase::Charging);
    assert_eq!(b.k.status().result, "空闲状态直接进入充电状态");
    assert!(b.k.status().charging);
    // command 1: the charging loop writes Open12vPower(1) after charge_wait_ms
    b.run(20_000, None, 0);
    assert!(b.enables.len() > 3, "{:?}", b.enables);
    assert!(b.k.request(Request::Undock, &lying()).is_ok());
    b.run(30_000, Some(docked(0.0, 0.0)), 0);
    assert_ne!(b.k.phase(), Phase::Charging);
}

#[test]
fn start_up_on_the_dock_confirms_a_late_charge_and_takes_it_with_command_minus_1() {
    let mut b = Boot::new();
    // no charge by 12 s: OpenMarkerProcess, FindTarget (16 sweeps), 0.001 m off: DetectionCharge(5) tries
    b.run(16_000, Some(docked(0.001, 0.001)), 15_000);
    assert_eq!(b.k.phase(), Phase::Charging);
    assert_eq!(b.k.status().result, "Init charge status is [charging]");
    // its command stays -1: no charging-loop writes
    let n = b.enables.len();
    b.run(30_000, Some(docked(0.001, 0.001)), 15_000);
    assert_eq!(b.enables.len(), n);
    assert!(b.k.request(Request::Undock, &lying()).is_ok());
}

#[test]
fn start_up_on_the_dock_without_a_charge_is_error_nochargeplug_and_undocks() {
    let mut b = Boot::new();
    // 12 s, the pose at ~13.6 s, three 5 s tries: [error] by ~29 s
    b.run(30_000, Some(docked(0.001, 0.001)), u64::MAX);
    assert_eq!(b.k.phase(), Phase::Idle);
    assert!(b.k.status().error);
    assert_eq!(
        b.k.status().result,
        "Init charge status is [error] because of [NOCHARGEPLUG], target x is 0.001000"
    );
    // DoExcepHandlingProcess: Open12vPower(3) back to back (a write every 2 s)
    let before = b.enables.len();
    b.run(42_000, Some(docked(0.001, 0.001)), u64::MAX);
    assert_eq!(b.enables.len() - before, 6, "{:?}", b.enables);
    // undock is allowed from the error
    let mut u = Boot::new();
    u.run(30_000, Some(docked(0.001, 0.001)), u64::MAX);
    assert!(u.k.request(Request::Undock, &lying()).is_ok());
    // a charge that comes: [error] -> [charging] at the end of an Open12vPower(3)
    b.run(60_000, Some(docked(0.001, 0.001)), 45_000);
    assert_eq!(b.k.phase(), Phase::Charging);
    assert_eq!(
        b.k.status().result,
        "My status is [error], begin to transtion to [charging]"
    );
}

#[test]
fn start_up_away_from_the_dock_is_idle() {
    let mut b = Boot::new();
    // FindTarget's tag_receive_timeout (2 s) after the 12 s detect
    b.run(14_100, None, u64::MAX);
    assert_eq!(b.k.phase(), Phase::Idle);
    assert!(!b.k.status().error);
    assert_eq!(
        b.k.status().result,
        "Init charge status is [idle] because did not see the charging pile."
    );
    assert!(b.k.request(Request::Undock, &lying()).is_err());
    // no charge: it stays idle
    b.run(60_000, None, u64::MAX);
    assert_eq!(b.k.phase(), Phase::Idle);
}

/// The vendor logs' silent idle -> charging episodes: "a bit far", then the idle loop's 3 s wake takes a later charge
#[test]
fn start_up_a_bit_far_then_the_idle_loop_takes_a_later_charge() {
    // 11-20 22:40:50.736 -> state:0 -> 2 42.0 s on; 11-21 06:31:23.825 -> 219.0 s on
    for (x, y, wake_s) in [(0.004372, -0.003019, 42u64), (0.000889, 0.006985, 219)] {
        let mut b = Boot::new();
        let pose = Some(docked(x, y));
        let mut ended = None;
        while ended.is_none() {
            b.run(b.tick + 1, pose, u64::MAX);
            if b.k.status().result.contains("a bit far") {
                ended = Some(b.tick);
            }
            assert!(b.tick < 20_000, "{}", b.k.status().result);
        }
        let ended = ended.unwrap();
        let r = b.k.status().result.clone();
        assert!(
            r.starts_with("Init charge status is [idle] because of the distance from the charging pile is a bit far"),
            "{r}"
        );
        // the charge comes within the last wake period
        let charge_at = ended + (wake_s - 2) * 1000;
        b.run(ended + wake_s * 1000 - 1, pose, charge_at);
        assert_eq!(b.k.phase(), Phase::Idle, "{x} {y}");
        b.run(ended + wake_s * 1000 + 1, pose, charge_at);
        assert_eq!(b.k.phase(), Phase::Charging, "{x} {y}");
        assert_eq!(b.k.status().result, "空闲状态直接进入充电状态");
        // then the logged undock from charging (06:35:09) is allowed
        assert!(b.k.request(Request::Undock, &lying()).is_ok());
    }
}

#[test]
fn charging_with_command_minus_1_waits_for_a_command_even_standing() {
    let mut b = Boot::new();
    b.run(16_000, Some(docked(0.001, 0.001)), 15_000);
    assert_eq!(b.k.status().result, "Init charge status is [charging]");
    // charge_manager's state 2 acts only on command 1: standing up or losing the charge changes nothing
    b.run_as(&standing("navi_charge"), 20_000, None, 15_000, 18_000);
    assert_eq!(b.k.phase(), Phase::Charging);
    assert!(b.k.request(Request::Undock, &lying()).is_ok());
}

#[test]
fn charge_lost_while_charging_is_error_nochargeplug_until_it_returns() {
    let mut b = Boot::new();
    // Detection charge success, the idle loop's wake: [charging] with command 1
    b.run(10_000, None, 0);
    assert_eq!(b.k.phase(), Phase::Charging);
    // the charge drops at 20 s: the loop's next pass (after Open12vPower(1)'s sleep) is [error] 0x1007
    let view = lying();
    b.run_as(&view, 20_000, None, 0, 20_000);
    assert_eq!(b.k.phase(), Phase::Charging);
    let lost = b.enables.len();
    b.run_as(&view, 27_000, None, 0, 20_000);
    assert_eq!(b.k.phase(), Phase::Idle);
    assert!(b.k.status().error);
    assert_eq!(
        b.k.status().result,
        "My status is [charging], begin transtion status to [error] error code:0x1007"
    );
    // DoExcepHandlingProcess: Open12vPower(3) back to back
    b.run_as(&view, 39_000, None, 0, 20_000);
    assert!(b.enables.len() - lost >= 6, "{:?}", &b.enables[lost..]);
    // no charge: it stays in the error, and undock is allowed from it
    b.run_as(&view, 60_000, None, 0, 20_000);
    assert_eq!(b.k.phase(), Phase::Idle);
    assert!(b.k.status().error);
    let mut c = Boot::new();
    c.run(10_000, None, 0);
    c.run_as(&view, 30_000, None, 0, 20_000);
    assert!(c.k.request(Request::Undock, &view).is_ok());
}

#[test]
fn the_error_state_returns_charging_when_the_charge_comes_and_idles_on_cancel() {
    let view = lying();
    let mut b = Boot::new();
    b.run(10_000, None, 0);
    b.run_as(&view, 30_000, None, 0, 20_000);
    assert!(b.k.status().error);
    // the charge back from 31 s: [error] -> [charging] at the end of an Open12vPower(3)
    b.run_as(&view, 45_000, None, 31_000, u64::MAX);
    assert_eq!(b.k.phase(), Phase::Charging);
    assert_eq!(
        b.k.status().result,
        "My status is [error], begin to transtion to [charging]"
    );
    // command 2 in the error state: idle, and no undock from idle
    let mut c = Boot::new();
    c.run(10_000, None, 0);
    c.run_as(&view, 30_000, None, 0, 20_000);
    c.k.request(Request::Cancel, &view).unwrap();
    assert!(!c.k.status().error);
    assert_eq!(
        c.k.status().result,
        "My status is [error], begin to transtion to [idle]"
    );
    assert!(c.k.request(Request::Undock, &view).is_err());
    let n = c.enables.len();
    c.run_as(&view, 45_000, None, 0, 20_000);
    assert_eq!(c.enables.len(), n);
}

#[test]
fn a_controller_fault_while_charging_keeps_the_charge() {
    // 2026-11-25 04:04:26: a fault (JointDamping, state 2) knocked ours off Charging; charge_manager keeps it
    let mut b = Boot::new();
    b.run(10_000, None, 0);
    assert_eq!(b.k.phase(), Phase::Charging);
    let mut damping = lying();
    damping.state = Some(State::JointDamping);
    damping.fault = true;
    b.run_as(&damping, 30_000, None, 0, u64::MAX);
    assert_eq!(b.k.phase(), Phase::Charging);
    assert!(b.k.status().charging && !b.k.status().error);
    assert!(b.k.request(Request::Undock, &damping).is_ok());
}

#[test]
fn an_undock_in_soft_e_stop_writes_nothing_and_idles() {
    // DoOverChargeTask (0x203908): motion state 2 -> 0x1008, no write; the idle loop then takes the charge again
    let mut b = Boot::new();
    b.run(10_000, None, 0);
    assert_eq!(b.k.phase(), Phase::Charging);
    let mut damping = lying();
    damping.state = Some(State::JointDamping);
    b.k.record_writes();
    b.k.request(Request::Undock, &damping).unwrap();
    b.run_as(&damping, 16_000, None, 0, u64::MAX);
    let writes = b.k.take_writes();
    assert!(
        writes
            .iter()
            .all(|(_, w)| matches!(w, Write::ChargeEnable(1))),
        "{writes:?}"
    );
    assert_eq!(b.k.phase(), Phase::Charging);
}

#[test]
fn an_undock_no_dock_this_run_set_restores_robot_commander_s_initial_gait() {
    // mInitRobotGaitStatus_: 0x1001 (ctor 0x1f5990) until goToPileAction (0x180bc0) sets it; written after the undock
    let mut b = Boot::new();
    b.run(10_000, None, 0);
    assert_eq!(b.k.phase(), Phase::Charging);
    b.k.record_writes();
    b.k.request(Request::Undock, &lying()).unwrap();
    // the undock's own stand-up and back-off as charge_manager writes them, on a robot that follows
    let mut view = lying();
    let mut seq = b.seq;
    let mut gaits = vec![];
    // the robot backs straight out at 0.3 m/s once the back-off starts
    let mut out_m = 0.0;
    for t in 0..30_000u64 {
        if t % 100 == 0 {
            seq += 1;
        }
        if b.k.phase() == Phase::BackOff {
            out_m += 0.0003;
        }
        let out = b.k.step(&view, Some((docked(-out_m, 0.0), seq)), None);
        if let Some(o) = &out {
            view.state = Some(match o.target {
                Target::Stand => State::StandUp,
                Target::RlControl => State::RlControl,
                Target::LieDown => State::LieDown,
                _ => State::Passive,
            });
            view.gait = o.gait;
            view.runner = Some(cfg().runner_for_gait(o.gait).to_string());
        }
        if b.k.phase() == Phase::Idle {
            break;
        }
    }
    let writes = b.k.take_writes();
    for (_, w) in &writes {
        if let Write::Gait(g) = w {
            gaits.push(*g);
        }
    }
    assert_eq!(b.k.status().result, "undocked");
    assert_eq!(gaits.last(), Some(&0x1001), "{gaits:#x?}");
    // the undock's Close24vPower first, then rl_control restored (mInitRobotStatus_ ours 17)
    assert!(
        matches!(writes.first(), Some((_, Write::ChargeEnable(0)))),
        "{writes:?}"
    );
    let restored = writes.iter().rev().find_map(|(_, w)| match w {
        Write::MotionState(m) => Some(*m),
        _ => None,
    });
    assert_eq!(restored, Some(MOTION_RL_CONTROL));
    // no start point this run: back off to exit_pile_distance_x (1.3 m) from the docked point
    let goal = vendor().exit_pile_distance_x as f64;
    assert!(
        (out_m - goal).abs() < 0.1,
        "backed off {out_m:.3} m, goal {goal} m"
    );
}
