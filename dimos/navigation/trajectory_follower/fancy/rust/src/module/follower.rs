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

//! `trajectory_follower`: the motion controller as a robot-side module, a port of `adapter/follower.py`.
//! The law is a pure pose+path -> twist function; this owns subscriptions, the control clock, arrival and the deadman.
//! No map: the room the planner priced arrives in the path's own timestamps (`stamps::decode_ceilings`).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::laws::hinted::update as hinted_update;
use dimos_local_planner::planner::Emb;
use dimos_module::{native_config, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::Twist;
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::std_msgs::Bool;
// serde derives come in through #[native_config]
use tracing::info;

use crate::emb;
use dimos_local_planner::module::msg;
use dimos_local_planner::module::tf_pose::PoseWatch;

/// Mirrors `TrajectoryFollowerConfig` (adapter/follower.py).
#[native_config]
#[derive(Clone)]
pub struct Config {
    #[validate(range(exclusive_min = 0.0))]
    pub control_frequency: f64,
    /// Planar distance that counts as arrival (m).
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f64,
    /// The planner's own body (`embodiment/base.py`).
    pub embodiment: Emb,
    /// The pose is the `path.frame_id -> base_frame` edge on tf, read each tick.
    pub base_frame: String,
    /// Deadman: zero the twist once the held path is this old, measured from arrival.
    /// Must clear the replan cadence (one plan per map, gaps to ~1.3 s).
    #[validate(range(exclusive_min = 0.0))]
    pub max_path_age_s: f64,
}

/// Arrival edge detector: fires once per goal, then holds until it moves (`follower.GoalLatch`).
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

    /// Moves under the tolerance are the same goal: replans snap the path end to the grid.
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

/// The goal a path carries: its last pose, or `None` for a single-pose refusal stub.
pub fn goal_of(path: &Path) -> Option<(f64, f64)> {
    if path.poses.len() < 2 {
        return None;
    }
    let last = path.poses.last()?;
    Some((last.pose.position.x, last.pose.position.y))
}

#[derive(Default)]
struct Shared {
    path: Option<Arc<Path>>,
    /// Arrival, not `msg.ts`: this measures how long since the planner was last heard.
    path_at: Option<Instant>,
    /// Drained per tick in arrival order, so the latch sees the same `set_goal` sequence the python does.
    goals: Vec<(f64, f64)>,
}

#[derive(Module)]
#[module(name = "trajectory_follower", setup = spawn_worker, teardown = stop_worker)]
pub struct TrajectoryFollower {
    #[input(decode = Path::decode, handler = on_path)]
    path: Input<Path>,

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

    /// Abort the loop before the zero twist, or the next tick could race it.
    async fn stop_worker(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
        msg::publish(&self.nav_cmd_vel, &msg::twist(0.0, 0.0, 0.0)).await;
    }

    /// An empty path is a stop: halt now, not a control period later.
    async fn on_path(&mut self, path: Path) {
        if path.poses.is_empty() {
            {
                let mut s = self.shared.lock().expect("shared mutex");
                s.path = None;
                s.path_at = None;
            }
            msg::publish(&self.nav_cmd_vel, &msg::twist(0.0, 0.0, 0.0)).await;
            return;
        }
        let goal = goal_of(&path);
        let mut s = self.shared.lock().expect("shared mutex");
        s.path = Some(Arc::new(path));
        s.path_at = Some(Instant::now());
        if let Some(goal) = goal {
            s.goals.push(goal);
        }
    }
}

/// What one control tick should command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Tick {
    /// No pose or no plan.
    Idle,
    /// The deadman: the planner has gone quiet.
    Stale { age_s: f64 },
    /// First tick at the goal: stop and latch.
    Arrived,
    /// Latched; hold until a new goal.
    Holding,
    /// Run the law.
    Drive,
}

/// The tick branch, clock-free for tests. Advances `latch` itself: arrival is an edge, asking twice loses it.
pub fn decide(
    pose: Option<(f64, f64, f64)>,
    path_age_s: Option<f64>,
    max_path_age_s: f64,
    latch: &mut GoalLatch,
) -> Tick {
    let (Some(pose), Some(age)) = (pose, path_age_s) else {
        return Tick::Idle;
    };
    // the deadman outranks arrival: a goal reached against an unrefreshed plan is a coincidence
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

/// Edge trigger: a dead planner warns once, not every tick.
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

            // read per tick in the plan's own frame: no plan, no pose
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
                }
                Tick::Drive => {
                    if stale.recover() {
                        info!("path is live again, resuming");
                    }
                    let pose = pose.expect("Drive implies a pose");
                    let path = snap.path.clone().expect("Drive implies a path");
                    let states = msg::path_states(&path);
                    let ts = msg::path_stamps(&path);
                    // no clearance array: the law decodes the stamps itself
                    let (vx, vy, wz) = hinted_update(pose, &states, None, Some(&ts), &hinted);
                    self.publish_twist(vx, vy, wz).await;
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_hold_stub_is_a_stop_to_every_law() {
        // the refusal is one pose, under the laws' `path.len() < 2` veto
        let held =
            dimos_local_planner::module::planner::hold_stub((1.5, -2.0, 0.0), "odom", 0.0, 0.0);
        let states = msg::path_states(&held);
        let cfg = emb::hinted_params(&Emb::fixture());
        assert_eq!(
            crate::laws::hinted::update((1.5, -2.0, 0.0), &states, None, None, &cfg),
            (0.0, 0.0, 0.0)
        );
    }
    use dimos_local_planner::module::planner;

    fn latch() -> GoalLatch {
        GoalLatch::new(0.2)
    }

    // GoalLatch: the cases in adapter/test_follower.py

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
        // exclusive boundary: exactly at the limit still drives
        assert_eq!(
            decide(Some((0.0, 0.0, 0.0)), Some(1.0), 1.0, &mut l),
            Tick::Drive
        );
    }

    #[test]
    fn a_stale_path_outranks_an_arrival() {
        // the latch stays unarmed under the deadman, so a live plan later still gets its edge
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
        // still holding after drifting off the goal
        assert_eq!(
            decide(Some((0.5, 0.0, 0.0)), Some(0.0), 1.0, &mut l),
            Tick::Holding
        );
    }

    fn fixture() -> Emb {
        Emb::fixture()
    }

    #[test]
    fn the_params_land_every_field_in_the_law() {
        let p = emb::hinted_params(&fixture());
        assert_eq!(p.base.lookahead, 0.35);
        assert_eq!(p.base.min_speed, fixture().min_speed);
        assert_eq!(p.base.max_speed, fixture().max_speed);
        assert_eq!(p.base.speed_lookahead, 2.0);
        // python spells it walk_slip_ramp, the law spells it slip_ramp
        assert_eq!(p.slip_ramp, 0.08);
        assert_eq!(p.walk_gain, 0.964);
    }

    #[test]
    fn an_unstamped_path_leaves_the_law_ungoverned_rather_than_creeping() {
        // an unstamped path is None, not a tight corridor
        let states: Vec<msg::State> = (0..5).map(|k| [k as f64 * 0.2, 0.0, 0.0]).collect();
        let band = emb::hinted_params(&Emb::fixture()).base;
        assert!(crate::stamps::decode_ceilings(&[0.0; 5], &states, &band).is_none());
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
