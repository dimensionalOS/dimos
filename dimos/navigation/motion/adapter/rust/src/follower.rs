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

//! `trajectory_follower`: the motion controller as a robot-side module.
//!
//! A port of `adapter/follower.py`, which is the specification. The law is a
//! pure pose+path -> twist function in `dimos_motion2_tc`, parity-locked to its
//! python twin; this module owns the subscriptions, the control clock, goal
//! arrival and the deadman. The pose is the `path.frame_id -> base_frame` edge
//! on tf, read per tick (`tf_pose.rs`), so the law controls in the frame the
//! plan is expressed in.
//!
//! NO MAP. The room the planner priced arrives in the path's own timestamps
//! (`stamps::decode_ceilings`), and the law reads it from there; the follower
//! never sees the local map.
//!
//! THE DEADMAN. `max_path_age_s`, measured from ARRIVAL, zeroes the twist
//! once the held path is that old: it guards a planner that stopped speaking,
//! alive-and-failing included. The planner's own hold stub covers a stale
//! map; this covers the planner itself. The same age on the pose's tf stamp
//! zeroes it too: a plan with no live pose under it is not driven.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dimos_module::{native_config, warn_throttled, Input, Module, Output, Tf};
use dimos_motion2_target::planner::Emb;
use dimos_motion2_tc::laws::hinted::update as hinted_update;
use lcm_msgs::geometry_msgs::Twist;
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::std_msgs::Bool;
// serde derives come in through #[native_config]
use tracing::info;

use crate::emb;
use crate::msg;
use crate::tf_pose::PoseWatch;

/// Mirrors `TrajectoryFollowerConfig` (adapter/follower.py).
#[native_config]
#[derive(Clone)]
pub struct Config {
    #[validate(range(exclusive_min = 0.0))]
    pub control_frequency: f64,
    /// Planar distance that counts as arrival (m).
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f64,
    /// The body, as `embodiment/base.py` records it: the planner's own value.
    pub embodiment: Emb,
    /// The pose is the `path.frame_id -> base_frame` edge on tf, read each
    /// tick. Ticks wait until it resolves, and once its stamp stops advancing
    /// for `max_path_age_s` it counts as missing again.
    pub base_frame: String,
    /// A commanded speed at or under this is standing still, whatever the
    /// reason. Classification for the stall log only; it commands nothing.
    #[validate(range(min = 0.0))]
    pub idle_speed: f64,
    /// Zero the twist once the held path is this old, measured from ARRIVAL.
    /// Sized to the replan gate: plans arrive per map (~1 Hz, gaps to ~1.3 s),
    /// so anything under ~2 s stutters the walk on healthy cadence.
    #[validate(range(exclusive_min = 0.0))]
    pub max_path_age_s: f64,
}

/// Arrival edge detector: fires once per goal, then holds until it moves. A
/// port of `follower.GoalLatch`.
#[derive(Debug, Clone)]
pub struct GoalLatch {
    tolerance: f64,
    goal: Option<(f64, f64)>,
    reached: bool,
}

impl GoalLatch {
    pub fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            goal: None,
            reached: false,
        }
    }

    pub fn reached(&self) -> bool {
        self.reached
    }

    /// Moves under the arrival tolerance are the SAME goal: replans snap the
    /// path end to the search grid, and re-chasing that is jitter.
    pub fn set_goal(&mut self, xy: (f64, f64)) {
        if self.goal.is_none_or(|g| dist(xy, g) > self.tolerance) {
            self.goal = Some(xy);
            self.reached = false;
        }
    }

    /// True exactly once: the tick this position first reaches the goal.
    pub fn arrive(&mut self, xy: (f64, f64)) -> bool {
        let Some(goal) = self.goal else {
            return false;
        };
        if self.reached || dist(xy, goal) >= self.tolerance {
            return false;
        }
        self.reached = true;
        true
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// The goal a path carries, or `None` when it carries none.
///
/// A plan ends at the goal, so its last pose is the target -- but a
/// SINGLE-POSE path is the planner's refusal, and reading that as an arrival
/// target would latch `goal_reached` at the robot's own feet.
pub fn goal_of(path: &Path) -> Option<(f64, f64)> {
    if path.poses.len() < 2 {
        return None;
    }
    let last = path.poses.last()?;
    Some((last.pose.position.x, last.pose.position.y))
}

/// Everything the handlers record and the worker reads.
#[derive(Default)]
struct Shared {
    path: Option<Arc<Path>>,
    /// ARRIVAL, not `msg.ts`: what this guards is how long since the planner
    /// was last heard from.
    path_at: Option<Instant>,
    /// Goals in the order the paths carrying them arrived, drained per tick.
    /// A queue rather than "the latest path's goal" so the latch sees the same
    /// `set_goal` SEQUENCE the python's subscription callback sees, even when
    /// two plans land inside one control period.
    goals: Vec<(f64, f64)>,
}

#[derive(Module)]
#[module(name = "trajectory_follower", setup = spawn_worker, teardown = stop_worker)]
pub struct TrajectoryFollower {
    #[input(decode = Path::decode, handler = on_path)]
    path: Input<Path>,

    #[input(decode = Bool::decode, handler = on_stop_movement)]
    stop_movement: Input<Bool>,

    #[output(encode = Twist::encode)]
    nav_cmd_vel: Output<Twist>,

    #[output(encode = Bool::encode)]
    goal_reached: Output<Bool>,

    #[config]
    config: Config,

    #[tf]
    tf: Tf,

    shared: Arc<Mutex<Shared>>,
    worker: Option<tokio::task::JoinHandle<()>>,
}

impl TrajectoryFollower {
    async fn spawn_worker(&mut self) {
        let worker = Worker {
            shared: Arc::clone(&self.shared),
            config: self.config.clone(),
            nav_cmd_vel: self.nav_cmd_vel.clone(),
            goal_reached: self.goal_reached.clone(),
            tf: self.tf.clone(),
        };
        self.worker = Some(tokio::spawn(worker.run()));
    }

    /// Stop the loop first, then say stop: a twist published while the worker
    /// could still tick would be raced by its next command.
    async fn stop_worker(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
        msg::publish(&self.nav_cmd_vel, &msg::twist(0.0, 0.0, 0.0)).await;
    }

    async fn on_path(&mut self, path: Path) {
        let goal = goal_of(&path);
        let mut s = self.shared.lock().expect("shared mutex");
        s.path = Some(Arc::new(path));
        s.path_at = Some(Instant::now());
        if let Some(goal) = goal {
            s.goals.push(goal);
        }
    }

    /// Preemption: drop the plan and stop. The zero goes out here rather than
    /// waiting for the next tick, because a stop request that takes a control
    /// period to land is not a stop request.
    async fn on_stop_movement(&mut self, msg: Bool) {
        if !msg.data {
            return;
        }
        {
            let mut s = self.shared.lock().expect("shared mutex");
            s.path = None;
            s.path_at = None;
        }
        msg::publish(&self.nav_cmd_vel, &msg::twist(0.0, 0.0, 0.0)).await;
    }
}

/// What one control tick should command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Tick {
    /// Not running yet: no pose, or no plan (which is also where a
    /// `stop_movement` leaves us).
    Idle,
    /// The planner has gone quiet. The deadman.
    Stale { age_s: f64 },
    /// The tick the robot first reaches the goal: stop and latch.
    Arrived,
    /// The goal is reached and latched; hold until a new one.
    Holding,
    /// Run the law.
    Drive,
}

/// The tick branch, lifted out of the loop so it can be tested with no
/// transport and no clock. `latch` is advanced here because arrival is an
/// EDGE and asking twice would lose it.
pub fn decide(
    pose: Option<(f64, f64, f64)>,
    path_age_s: Option<f64>,
    max_path_age_s: f64,
    latch: &mut GoalLatch,
) -> Tick {
    let (Some(pose), Some(age)) = (pose, path_age_s) else {
        return Tick::Idle;
    };
    // The deadman outranks arrival: a goal reached against a plan nobody is
    // refreshing is a coincidence, not an arrival.
    if age > max_path_age_s {
        return Tick::Stale { age_s: age };
    }
    if latch.arrive((pose.0, pose.1)) {
        return Tick::Arrived;
    }
    if latch.reached() {
        return Tick::Holding;
    }
    Tick::Drive
}

/// Edge trigger, so a dead planner warns once rather than `control_frequency`
/// times a second for as long as it stays dead.
#[derive(Default)]
struct Gate {
    on: bool,
}

impl Gate {
    fn enter(&mut self) -> bool {
        !std::mem::replace(&mut self.on, true)
    }

    fn recover(&mut self) -> bool {
        std::mem::replace(&mut self.on, false)
    }
}

struct Worker {
    shared: Arc<Mutex<Shared>>,
    config: Config,
    nav_cmd_vel: Output<Twist>,
    goal_reached: Output<Bool>,
    tf: Tf,
}

struct Snapshot {
    path: Option<Arc<Path>>,
    age_s: Option<f64>,
    goals: Vec<(f64, f64)>,
}

impl Worker {
    async fn run(self) {
        let mut ticker =
            tokio::time::interval(Duration::from_secs_f64(1.0 / self.config.control_frequency));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        let hinted = emb::hinted_params(&self.config.embodiment);
        let mut latch = GoalLatch::new(self.config.goal_tolerance);
        let mut stale = Gate::default();
        let mut watch = PoseWatch::new(self.config.max_path_age_s);

        loop {
            ticker.tick().await;
            let now = Instant::now();
            let snap = self.snapshot(now);
            for goal in &snap.goals {
                latch.set_goal(*goal);
            }

            // the pose is read here, on the tick, off tf in the frame the plan
            // is expressed in -- so there is none to read before a plan
            let pose = snap.path.as_ref().and_then(|path| {
                watch.get(
                    &self.tf,
                    &path.header.frame_id,
                    &self.config.base_frame,
                    now,
                )
            });
            let pose = pose.map(|p| (p.state[0], p.state[1], p.state[2]));
            match decide(pose, snap.age_s, self.config.max_path_age_s, &mut latch) {
                Tick::Idle => {
                    if snap.path.is_some() {
                        // a plan with no live pose under it: the deadman on the pose
                        self.publish_twist(0.0, 0.0, 0.0).await;
                    }
                    warn_throttled!(
                        Duration::from_secs(3),
                        path = snap.path.is_some(),
                        pose = pose.is_some(),
                        "not driving: a local plan or its pose on tf is missing",
                    )
                }
                Tick::Stale { age_s } => {
                    if stale.enter() {
                        tracing::warn!(
                            age_s,
                            max_path_age_s = self.config.max_path_age_s,
                            "path is stale, zeroing the twist"
                        );
                    }
                    self.publish_twist(0.0, 0.0, 0.0).await;
                }
                Tick::Arrived => {
                    self.publish_twist(0.0, 0.0, 0.0).await;
                    msg::publish(&self.goal_reached, &Bool { data: true }).await;
                    info!("Goal reached");
                }
                Tick::Holding => {
                    self.publish_twist(0.0, 0.0, 0.0).await;
                    warn_throttled!(
                        Duration::from_secs(3),
                        "not driving: waiting for a new goal, the last one is reached and latched",
                    );
                }
                Tick::Drive => {
                    if stale.recover() {
                        info!("path is live again, resuming");
                    }
                    let pose = pose.expect("Drive implies a pose");
                    let path = snap.path.clone().expect("Drive implies a path");
                    let states = msg::path_states(&path);
                    let ts = msg::path_stamps(&path);
                    // the law decodes the dialect itself; there is no
                    // clearance array to hand it
                    let (vx, vy, wz) = hinted_update(pose, &states, None, Some(&ts), &hinted);
                    self.publish_twist(vx, vy, wz).await;
                    self.report(vx, vy, wz, states.len());
                }
            }
        }
    }

    fn snapshot(&self, now: Instant) -> Snapshot {
        let mut s = self.shared.lock().expect("shared mutex");
        Snapshot {
            path: s.path.clone(),
            age_s: s.path_at.map(|t| now.duration_since(t).as_secs_f64()),
            goals: std::mem::take(&mut s.goals),
        }
    }

    async fn publish_twist(&self, vx: f64, vy: f64, wz: f64) {
        msg::publish(&self.nav_cmd_vel, &msg::twist(vx, vy, wz)).await;
    }

    /// Standing still with a plan in hand is the ambiguous case, and the two
    /// causes want opposite fixes: a one-pose plan is the PLANNER refusing
    /// (look upstream -- map, clearance, goal), while a real plan the law
    /// answers with ~zero is the FOLLOWER's own governor or gait envelope.
    fn report(&self, vx: f64, vy: f64, wz: f64, waypoints: usize) {
        let speed = vx.hypot(vy);
        if speed > self.config.idle_speed || wz.abs() > self.config.idle_speed {
            return;
        }
        if waypoints < 2 {
            warn_throttled!(
                Duration::from_secs(3),
                "not moving: the planner published a single-pose stub, i.e. no safe route",
            );
        } else {
            warn_throttled!(
                Duration::from_secs(3),
                waypoints,
                "not moving: the law commands ~0 on a real plan; suspect the speed \
                 governor or the gait envelope",
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner;

    fn latch() -> GoalLatch {
        GoalLatch::new(0.2)
    }

    // GoalLatch -- the cases in adapter/test_follower.py

    #[test]
    fn goal_latch_fires_once_then_holds() {
        let mut l = latch();
        l.set_goal((1.0, 0.0));
        assert!(!l.arrive((0.0, 0.0)));
        assert!(l.arrive((0.95, 0.0)));
        assert!(l.reached());
        assert!(!l.arrive((0.95, 0.0)));
    }

    #[test]
    fn goal_latch_ignores_sub_tolerance_goal_moves() {
        let mut l = latch();
        l.set_goal((1.0, 0.0));
        assert!(l.arrive((1.0, 0.0)));
        l.set_goal((1.05, 0.0)); // a replan's grid snap: the same goal
        assert!(l.reached());
        l.set_goal((3.0, 0.0)); // a new task
        assert!(!l.reached());
    }

    #[test]
    fn goal_latch_without_a_goal_never_arrives() {
        assert!(!latch().arrive((0.0, 0.0)));
    }

    // the goal a path carries

    fn path_of(states: &[msg::State]) -> Path {
        msg::build_path(states, &[], 0.0, "odom", 0.0)
    }

    #[test]
    fn a_single_pose_stub_is_a_refusal_not_an_arrival_target() {
        assert_eq!(goal_of(&path_of(&[[1.0, 2.0, 0.0]])), None);
        assert_eq!(goal_of(&path_of(&[])), None);
        assert_eq!(
            goal_of(&path_of(&[[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]])),
            Some((1.0, 2.0))
        );
    }

    #[test]
    fn a_hold_stub_never_latches_the_goal_at_the_robots_own_feet() {
        let stub = planner::hold_stub((5.0, 5.0, 0.0), "odom", 0.0, 0.0);
        assert_eq!(goal_of(&stub), None);
        let mut l = latch();
        if let Some(goal) = goal_of(&stub) {
            l.set_goal(goal);
        }
        assert!(!l.arrive((5.0, 5.0)));
    }

    // the tick branch

    #[test]
    fn no_pose_or_no_path_is_idle() {
        assert_eq!(decide(None, Some(0.0), 1.0, &mut latch()), Tick::Idle);
        assert_eq!(
            decide(Some((0.0, 0.0, 0.0)), None, 1.0, &mut latch()),
            Tick::Idle
        );
    }

    #[test]
    fn a_stale_path_zeroes_the_twist() {
        let mut l = latch();
        l.set_goal((10.0, 0.0));
        assert_eq!(
            decide(Some((0.0, 0.0, 0.0)), Some(1.5), 1.0, &mut l),
            Tick::Stale { age_s: 1.5 }
        );
        // and the boundary is exclusive, so a path exactly at the limit drives
        assert_eq!(
            decide(Some((0.0, 0.0, 0.0)), Some(1.0), 1.0, &mut l),
            Tick::Drive
        );
    }

    #[test]
    fn a_stale_path_outranks_an_arrival() {
        // standing on the goal of a plan nobody is refreshing is a
        // coincidence: the deadman fires and the latch stays unarmed, so a
        // live plan later still gets its arrival edge
        let mut l = latch();
        l.set_goal((0.0, 0.0));
        assert_eq!(
            decide(Some((0.0, 0.0, 0.0)), Some(9.0), 1.0, &mut l),
            Tick::Stale { age_s: 9.0 }
        );
        assert!(!l.reached());
        assert_eq!(
            decide(Some((0.0, 0.0, 0.0)), Some(0.1), 1.0, &mut l),
            Tick::Arrived
        );
    }

    #[test]
    fn arrival_fires_once_and_then_holds() {
        let mut l = latch();
        l.set_goal((1.0, 0.0));
        let at_goal = Some((1.0, 0.0, 0.0));
        assert_eq!(decide(at_goal, Some(0.0), 1.0, &mut l), Tick::Arrived);
        assert_eq!(decide(at_goal, Some(0.0), 1.0, &mut l), Tick::Holding);
        // and it keeps holding even after the robot drifts off the goal
        assert_eq!(
            decide(Some((0.5, 0.0, 0.0)), Some(0.0), 1.0, &mut l),
            Tick::Holding
        );
    }

    // the law's parameters off the body

    fn fixture() -> Emb {
        Emb::fixture()
    }

    #[test]
    fn the_params_land_every_field_in_the_law() {
        let p = emb::hinted_params(&fixture());
        assert_eq!(p.base.lookahead, 0.35);
        // the law drives inside the governor's band
        assert_eq!(p.base.min_speed, fixture().min_speed);
        assert_eq!(p.base.max_speed, fixture().max_speed);
        assert_eq!(p.base.speed_lookahead, 2.0);
        // python spells it walk_slip_ramp, the law spells it slip_ramp
        assert_eq!(p.slip_ramp, 0.08);
        assert_eq!(p.walk_gain, 0.964);
    }

    #[test]
    fn an_unstamped_path_leaves_the_law_ungoverned_rather_than_creeping() {
        // a producer that does not speak the dialect must not be read as a
        // tight corridor; None is the honest answer and the law cruises
        let states: Vec<msg::State> = (0..5).map(|k| [k as f64 * 0.2, 0.0, 0.0]).collect();
        let band = emb::hinted_params(&Emb::fixture()).base;
        assert!(dimos_motion2_tc::stamps::decode_ceilings(&[0.0; 5], &states, &band).is_none());
    }

    #[test]
    fn a_veto_stub_commands_zero() {
        let stub = planner::hold_stub((1.0, 2.0, 0.5), "odom", 0.0, 0.0);
        let states = msg::path_states(&stub);
        let ts = msg::path_stamps(&stub);
        assert_eq!(
            hinted_update(
                (1.0, 2.0, 0.5),
                &states,
                None,
                Some(&ts),
                &emb::hinted_params(&fixture())
            ),
            (0.0, 0.0, 0.0)
        );
    }
}
