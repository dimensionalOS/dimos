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

//! The SE(2) local planner as a robot-side module; `adapter/planner.py` is the spec. Cloud from
//! `local_map`, pose from tf (`world_frame -> base_frame`), goal a carrot `goal_lookahead_m`
//! along `planner_path`. A refusal or a stale map publishes a one-pose hold stub.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::planner::{plan_explored, Emb, COMMIT_MARGIN};
use crate::{clearance, stamps};
use dimos_module::{native_config, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::PointCloud2;
use tracing::{debug, info, warn};
use validator::ValidationError;

use crate::module::emb;
use crate::module::msg;
use crate::module::obstacles::{self, ObstacleModel};
use crate::module::tf_pose::PoseWatch;

/// Mirrors `LocalPlannerConfig` (adapter/planner.py); the python `planner` registry
/// field does not cross, this module is the rust target planner.
#[native_config]
#[derive(Clone)]
#[validate(schema(function = "validate_obstacle_model"))]
pub struct Config {
    /// The python module's own value (`embodiment/base.py`), so both halves plan the same robot.
    pub embodiment: Emb,
    /// Growth of every planning box per side (m); negative shrinks.
    pub body_dilate_m: f64,
    /// Price multiplier per metre over lattice cells with no floor return; <= 1 turns it off.
    pub unseen_cost: f64,
    /// Plan discretisation (m); the python reads `planners/base.py RESOLUTION`.
    #[validate(range(exclusive_min = 0.0))]
    pub resolution: f64,
    /// The local map's voxel pitch (m); every lattice pitch scales off it.
    #[validate(range(exclusive_min = 0.0))]
    pub pointcloud_resolution: f64,
    #[validate(range(exclusive_min = 0.0))]
    pub replan_hz: f64,
    /// Carrot arc along the global route.
    #[validate(range(exclusive_min = 0.0))]
    pub goal_lookahead_m: f64,
    /// The pose is the `world_frame -> base_frame` tf edge; it counts as missing
    /// again once its stamp stalls for `max_map_age_s`.
    pub world_frame: String,
    pub base_frame: String,
    /// Plan only on a new local map or a moved carrot.
    pub replan_on_change: bool,
    /// Carrot movement worth re-solving for; MLS republishes at ~1 Hz with the
    /// waypoints moving and the carrot not.
    #[validate(range(min = 0.0))]
    pub replan_carrot_m: f64,
    /// A carrot jump beyond this drops the held route.
    #[validate(range(min = 0.0))]
    pub reset_carrot_m: f64,
    /// What counts as an obstacle (`obstacles.rs`).
    pub obstacle_model: String,
    /// Hold once the local map is this old, measured from arrival.
    #[validate(range(exclusive_min = 0.0))]
    pub max_map_age_s: f64,
}

fn validate_obstacle_model(config: &Config) -> Result<(), ValidationError> {
    if !obstacles::MODELS.contains(&config.obstacle_model.as_str()) {
        return Err(ValidationError::new(
            "obstacle_model must be one of: body_band",
        ));
    }
    Ok(())
}

/// Newest of each input; a replan is a fresh look at the world, never a queue.
#[derive(Default)]
struct Shared {
    cloud: Option<Arc<PointCloud2>>,
    /// Bumped per arrival so the worker can cache the extracted points.
    cloud_seq: u64,
    /// Arrival time, not `msg.ts`: the mapper's clock is not the robot's.
    cloud_at: Option<Instant>,
    global_xy: Option<Vec<[f64; 2]>>,
}

#[derive(Module)]
#[module(name = "local_planner", setup = spawn_worker, teardown = stop_worker)]
pub struct LocalPlanner {
    #[input(decode = PointCloud2::decode, handler = on_local_map)]
    local_map: Input<PointCloud2>,

    #[input(decode = Path::decode, handler = on_planner_path)]
    planner_path: Input<Path>,

    #[output(encode = Path::encode)]
    path: Output<Path>,

    #[config]
    config: Config,

    #[tf]
    tf: Tf,

    shared: Arc<Mutex<Shared>>,
    worker: Option<tokio::task::JoinHandle<()>>,
}

impl LocalPlanner {
    async fn spawn_worker(&mut self) {
        let emb = emb::dilated(self.config.embodiment.clone(), self.config.body_dilate_m);
        let model = obstacles::load(&self.config.obstacle_model, &emb)
            .expect("validated obstacle model name");
        let worker = Worker {
            shared: Arc::clone(&self.shared),
            config: self.config.clone(),
            emb,
            model,
            path: self.path.clone(),
            tf: self.tf.clone(),
        };
        self.worker = Some(tokio::spawn(worker.run()));
    }

    async fn stop_worker(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
    }

    // Handlers only record; the replan is the slow part and lives on the worker.

    async fn on_local_map(&mut self, msg: PointCloud2) {
        let mut s = self.shared.lock().expect("shared mutex");
        s.cloud = Some(Arc::new(msg));
        s.cloud_seq = s.cloud_seq.wrapping_add(1);
        s.cloud_at = Some(Instant::now());
    }

    /// MLS emits an empty path when it finds no route; the next tick clears what was published.
    async fn on_planner_path(&mut self, msg: Path) {
        let xy: Vec<[f64; 2]> = msg
            .poses
            .iter()
            .map(|p| [p.pose.position.x, p.pose.position.y])
            .collect();
        let mut s = self.shared.lock().expect("shared mutex");
        s.global_xy = (!xy.is_empty()).then_some(xy);
    }
}

/// `lookahead` metres of arc from the waypoint closest to the robot, clamped to
/// the route's end; port of `planner.carrot_along`.
pub fn carrot_along(route: &[[f64; 2]], robot: (f64, f64), lookahead: f64) -> Option<(f64, f64)> {
    let last = route.last()?;
    // np.argmin keeps the first minimum on a tie
    let mut i = 0usize;
    let mut best = f64::INFINITY;
    for (k, p) in route.iter().enumerate() {
        let d = (p[0] - robot.0).hypot(p[1] - robot.1);
        if d < best {
            best = d;
            i = k;
        }
    }
    let mut remaining = lookahead;
    for j in i..route.len().saturating_sub(1) {
        let (dx, dy) = (route[j + 1][0] - route[j][0], route[j + 1][1] - route[j][1]);
        let seg = dx.hypot(dy);
        if seg >= remaining {
            let f = remaining / seg;
            return Some((route[j][0] + f * dx, route[j][1] + f * dy));
        }
        remaining -= seg;
    }
    Some((last[0], last[1]))
}

/// Has an input the plan depends on moved? The route enters only through the
/// carrot, so that is what is compared.
pub fn replan_due(
    gate: bool,
    planned: Option<(u64, (f64, f64))>,
    cloud_seq: u64,
    carrot: (f64, f64),
    carrot_m: f64,
) -> bool {
    if !gate {
        return true;
    }
    match planned {
        None => true,
        Some((seq, was)) => {
            seq != cloud_seq || (was.0 - carrot.0).hypot(was.1 - carrot.1) > carrot_m
        }
    }
}

/// One tick's branch of the replan loop, the python `_plan_loop`'s three-way,
/// lifted out so it tests without a transport.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Tick {
    /// The map has gone quiet: refuse.
    Hold { age_s: f64 },
    /// The route is gone while a plan is out: publish an empty path, forget the plan.
    Clear,
    /// Everything the search needs has arrived.
    Plan,
    /// Something has never arrived. Nothing to publish yet.
    Wait,
}

pub fn decide(
    has_pose: bool,
    cloud_age_s: Option<f64>,
    has_route: bool,
    published: bool,
    max_map_age_s: f64,
) -> Tick {
    match (has_pose, cloud_age_s) {
        // Stale outranks everything and needs no route: a frozen map at cruise
        // speed is the failure this guards.
        (true, Some(age)) if age > max_map_age_s => Tick::Hold { age_s: age },
        // A clear needs neither pose nor map; it fires once because it turns `published` off.
        _ if !has_route && published => Tick::Clear,
        (true, Some(_)) if has_route => Tick::Plan,
        _ => Tick::Wait,
    }
}

/// The planner's refusal: one pose at the robot, which every law reads as hold.
pub fn hold_stub(pose: (f64, f64, f64), frame_id: &str, ts: f64, ground_z: f64) -> Path {
    msg::build_path(&[[pose.0, pose.1, pose.2]], &[ts], ts, frame_id, ground_z)
}

/// Edge trigger so a dead map warns once per spell, not `replan_hz` times a second.
#[derive(Default)]
pub struct StaleGate {
    stale: bool,
}

impl StaleGate {
    /// True on the first tick of a stale spell.
    pub fn enter(&mut self) -> bool {
        !std::mem::replace(&mut self.stale, true)
    }

    /// True on the first tick after one ends.
    pub fn recover(&mut self) -> bool {
        std::mem::replace(&mut self.stale, false)
    }
}

/// One search plus the precision annotation: `planner.py::_plan_once` then
/// `planner.py::annotate`. Free so it runs with no transport.
#[allow(clippy::too_many_arguments)]
pub fn plan_once(
    config: &Config,
    emb: &Emb,
    model: &dyn ObstacleModel,
    points: &[[f32; 3]],
    pose: (f64, f64, f64),
    goal: (f64, f64),
    ground_z: f64,
    incumbent: Option<&[[f64; 3]]>,
) -> Path {
    let started = Instant::now();
    let t0 = msg::now_secs();
    // The model decides which returns are obstacles; the search only sees xy.
    let hard = obstacles::hard_points(model, points, ground_z);
    let ground = obstacles::ground_points(points, ground_z);
    let points: &[[f32; 3]] = &hard;
    let cloud: Vec<[f64; 2]> = points.iter().map(|p| [p[0] as f64, p[1] as f64]).collect();
    let states = match plan_explored(
        &cloud,
        &ground,
        config.unseen_cost,
        pose,
        goal,
        emb,
        config.resolution,
        config.pointcloud_resolution,
        incumbent,
        COMMIT_MARGIN,
    ) {
        Some(s) if !s.is_empty() => s,
        _ => {
            debug!(
                plan_ms = ms_since(started),
                "planner refused; publishing a hold stub"
            );
            return hold_stub(pose, &config.world_frame, t0, ground_z);
        }
    };
    // Room is measured off the same points the search saw, or the stamps price a different world.
    let xy: Vec<[f64; 2]> = states.iter().map(|s| [s[0], s[1]]).collect();
    let room = clearance::path_clearance(&xy, points, emb::half_width(emb));
    let ts = stamps::encode_precision(&states, &room, t0, &emb::governor(emb));
    debug!(
        waypoints = states.len(),
        plan_ms = ms_since(started),
        "path planned"
    );
    msg::build_path(&states, &ts, t0, &config.world_frame, ground_z)
}

/// Owns the replan cadence and the publishing, off the dispatch loop.
struct Worker {
    shared: Arc<Mutex<Shared>>,
    config: Config,
    emb: Emb,
    model: Box<dyn ObstacleModel>,
    path: Output<Path>,
    tf: Tf,
}

/// The handlers' snapshot, taken once per tick under one lock.
struct Snapshot {
    cloud: Option<Arc<PointCloud2>>,
    cloud_seq: u64,
    age_s: Option<f64>,
    route: Option<Vec<[f64; 2]>>,
}

impl Worker {
    async fn run(self) {
        let mut ticker =
            tokio::time::interval(Duration::from_secs_f64(1.0 / self.config.replan_hz));
        // A tick lost to an overrunning replan is gone; a burst afterwards makes the next late.
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        let mut gate = StaleGate::default();
        let mut watch = PoseWatch::new(self.config.max_map_age_s);
        let mut points: Option<(u64, Arc<Vec<[f32; 3]>>)> = None;
        // The (cloud, carrot) the published plan was made from.
        let mut planned: Option<(u64, (f64, f64))> = None;
        // The published plan; the search keeps it unless a fresh one earns the switch.
        let mut incumbent: Option<Vec<[f64; 3]>> = None;
        let mut published = false;

        loop {
            ticker.tick().await;
            let now = Instant::now();
            let snap = self.snapshot(now);
            let pose = watch.get(
                &self.tf,
                &self.config.world_frame,
                &self.config.base_frame,
                now,
            );
            // the ground is `Emb::base_height` under the body, not read off the scene
            let ground_z = pose.map(|p| p.z - self.config.embodiment.base_height);
            let pose = pose.map(|p| (p.state[0], p.state[1], p.state[2]));
            match decide(
                pose.is_some(),
                snap.age_s,
                snap.route.is_some(),
                published,
                self.config.max_map_age_s,
            ) {
                Tick::Hold { age_s } => {
                    if gate.enter() {
                        warn!(
                            age_s = age_s,
                            max_map_age_s = self.config.max_map_age_s,
                            "local_map is stale, holding"
                        );
                    }
                    // A hold is not change-gated: it is about the clock. Forget the plan
                    // and the incumbent, a route held across a dead link is unvalidated.
                    planned = None;
                    incumbent = None;
                    let pose = pose.expect("Hold implies a pose");
                    let held = hold_stub(
                        pose,
                        &self.config.world_frame,
                        msg::now_secs(),
                        ground_z.unwrap_or(0.0),
                    );
                    published = true;
                    msg::publish(&self.path, &held).await;
                }
                Tick::Clear => {
                    info!("planner_path is empty, clearing the local plan");
                    planned = None;
                    incumbent = None;
                    published = false;
                    let empty =
                        msg::build_path(&[], &[], msg::now_secs(), &self.config.world_frame, 0.0);
                    msg::publish(&self.path, &empty).await;
                }
                Tick::Plan => {
                    if gate.recover() {
                        info!("local_map is live again, resuming planning");
                    }
                    let pose = pose.expect("Plan implies a pose");
                    let route = snap.route.expect("Plan implies a route");
                    let Some(goal) =
                        carrot_along(&route, (pose.0, pose.1), self.config.goal_lookahead_m)
                    else {
                        continue; // an empty route is stored as None, so unreachable
                    };
                    if !replan_due(
                        self.config.replan_on_change,
                        planned,
                        snap.cloud_seq,
                        goal,
                        self.config.replan_carrot_m,
                    ) {
                        continue;
                    }
                    // A carrot that jumped is a different task; the held route is the old one's.
                    if planned.is_some_and(|(_, was)| {
                        (was.0 - goal.0).hypot(was.1 - goal.1) > self.config.reset_carrot_m
                    }) {
                        incumbent = None;
                    }
                    let cloud = snap.cloud.expect("Plan implies a cloud");
                    // The search is synchronous; block_in_place keeps it off the async
                    // workers, hence the module's 2 threads.
                    let produced = tokio::task::block_in_place(|| {
                        let pts = self.points(&cloud, snap.cloud_seq, &mut points)?;
                        Some(plan_once(
                            &self.config,
                            &self.emb,
                            self.model.as_ref(),
                            &pts,
                            pose,
                            goal,
                            ground_z.unwrap_or(0.0),
                            incumbent.as_deref(),
                        ))
                    });
                    if let Some(produced) = produced {
                        planned = Some((snap.cloud_seq, goal));
                        incumbent = Some(msg::path_states(&produced));
                        published = true;
                        msg::publish(&self.path, &produced).await;
                    }
                }
                Tick::Wait => {}
            }
        }
    }

    fn snapshot(&self, now: Instant) -> Snapshot {
        let s = self.shared.lock().expect("shared mutex");
        Snapshot {
            cloud: s.cloud.clone(),
            cloud_seq: s.cloud_seq,
            age_s: s.cloud_at.map(|t| now.duration_since(t).as_secs_f64()),
            route: s.global_xy.clone(),
        }
    }

    /// The latest cloud as xyz points, extracted once per arrival.
    fn points(
        &self,
        cloud: &PointCloud2,
        seq: u64,
        cache: &mut Option<(u64, Arc<Vec<[f32; 3]>>)>,
    ) -> Option<Arc<Vec<[f32; 3]>>> {
        if let Some((cached_seq, pts)) = cache {
            if *cached_seq == seq {
                return Some(Arc::clone(pts));
            }
        }
        match msg::extract_xyz(cloud) {
            Ok(pts) => {
                let pts = Arc::new(pts);
                *cache = Some((seq, Arc::clone(&pts)));
                Some(pts)
            }
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "could not read local_map, skipped a replan",
                );
                None
            }
        }
    }
}

fn ms_since(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

#[cfg(test)]
mod tests {
    use super::*;

    fn route(points: &[(f64, f64)]) -> Vec<[f64; 2]> {
        points.iter().map(|&(x, y)| [x, y]).collect()
    }

    // carrot_along: the cases in adapter/test_planner.py

    #[test]
    fn carrot_walks_arc_from_the_closest_waypoint() {
        let r = route(&[(0.0, 0.0), (2.0, 0.0), (2.0, 4.0)]);
        // closest to (2.1, 0.5) is (2, 0); 1.5 m of arc up the second leg
        assert_eq!(carrot_along(&r, (2.1, 0.5), 1.5), Some((2.0, 1.5)));
    }

    #[test]
    fn carrot_interpolates_within_a_segment() {
        let r = route(&[(0.0, 0.0), (10.0, 0.0)]);
        assert_eq!(carrot_along(&r, (0.0, 0.0), 5.0), Some((5.0, 0.0)));
    }

    #[test]
    fn carrot_clamps_to_the_route_end() {
        let r = route(&[(0.0, 0.0), (1.0, 0.0)]);
        assert_eq!(carrot_along(&r, (0.9, 0.0), 5.0), Some((1.0, 0.0)));
    }

    #[test]
    fn carrot_on_a_single_waypoint_is_that_waypoint() {
        assert_eq!(
            carrot_along(&route(&[(3.0, 4.0)]), (0.0, 0.0), 5.0),
            Some((3.0, 4.0))
        );
    }

    #[test]
    fn carrot_never_walks_backwards_past_the_robot() {
        let r = route(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]);
        assert_eq!(carrot_along(&r, (2.0, 0.0), 1.0), Some((2.0, 0.0)));
    }

    #[test]
    fn carrot_of_an_empty_route_is_nothing() {
        assert_eq!(carrot_along(&[], (0.0, 0.0), 1.0), None);
    }

    #[test]
    fn a_zero_length_leg_is_stepped_over_rather_than_divided_by() {
        let r = route(&[(0.0, 0.0), (0.0, 0.0), (4.0, 0.0)]);
        let got = carrot_along(&r, (0.0, 0.0), 1.0).expect("a carrot");
        assert!(got.0.is_finite() && got.1.is_finite(), "{got:?}");
        assert_eq!(got, (1.0, 0.0));
    }

    #[test]
    fn a_stale_map_holds_even_with_everything_else_present() {
        assert_eq!(
            decide(true, Some(7.0), true, false, 5.0),
            Tick::Hold { age_s: 7.0 }
        );
        assert_eq!(
            decide(true, Some(7.0), false, false, 5.0),
            Tick::Hold { age_s: 7.0 }
        );
        // and outranks a clear: both stop the robot, the hold says why
        assert_eq!(
            decide(true, Some(7.0), false, true, 5.0),
            Tick::Hold { age_s: 7.0 }
        );
    }

    #[test]
    fn a_route_that_vanished_clears_once_whatever_else_is_missing() {
        assert_eq!(decide(true, Some(0.2), false, true, 5.0), Tick::Clear);
        assert_eq!(decide(false, None, false, true, 5.0), Tick::Clear);
        // once cleared, nothing is published, so nothing to clear
        assert_eq!(decide(true, Some(0.2), false, false, 5.0), Tick::Wait);
    }

    #[test]
    fn a_live_map_with_a_pose_and_a_route_plans() {
        assert_eq!(decide(true, Some(0.2), true, false, 5.0), Tick::Plan);
    }

    #[test]
    fn a_missing_input_waits_rather_than_holding() {
        // no pose is "not running yet", not "the map died"
        assert_eq!(decide(false, Some(9.0), true, false, 5.0), Tick::Wait);
        assert_eq!(decide(true, None, true, false, 5.0), Tick::Wait);
        assert_eq!(decide(true, Some(0.2), false, false, 5.0), Tick::Wait);
    }

    #[test]
    fn the_age_boundary_is_exclusive() {
        assert_eq!(decide(true, Some(5.0), true, false, 5.0), Tick::Plan);
    }

    #[test]
    fn a_hold_stub_is_one_pose_at_the_robot() {
        let held = hold_stub((1.5, -2.0, std::f64::consts::FRAC_PI_2), "odom", 7.0, 0.0);
        assert_eq!(held.header.frame_id, "odom");
        assert_eq!(held.poses.len(), 1);
        assert_eq!(held.poses[0].pose.position.x, 1.5);
        assert_eq!(held.poses[0].pose.position.y, -2.0);
        let yaw = msg::yaw_of(&held.poses[0].pose.orientation);
        assert!((yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn the_stale_gate_fires_once_per_spell() {
        let mut gate = StaleGate::default();
        assert!(gate.enter());
        assert!(!gate.enter());
        assert!(!gate.enter());
        assert!(gate.recover());
        assert!(!gate.recover()); // a live map does not announce itself
        assert!(gate.enter()); // and a second spell warns again
    }

    fn config() -> Config {
        Config {
            embodiment: Emb::fixture(),
            body_dilate_m: 0.0,
            unseen_cost: 1.0,
            resolution: 0.1,
            pointcloud_resolution: 0.08,
            replan_hz: 5.0,
            goal_lookahead_m: 5.0,
            world_frame: "odom".into(),
            base_frame: "base_link".into(),
            replan_on_change: true,
            replan_carrot_m: 0.2,
            reset_carrot_m: 1.0,
            obstacle_model: "body_band".into(),
            max_map_age_s: 5.0,
        }
    }

    fn fixture() -> Emb {
        Emb::fixture()
    }

    fn model(cfg: &Config) -> Box<dyn ObstacleModel> {
        obstacles::load(&cfg.obstacle_model, &cfg.embodiment).expect("known model")
    }

    #[test]
    fn a_planned_path_carries_monotone_stamps() {
        let cfg = config();
        let produced = plan_once(
            &cfg,
            &fixture(),
            model(&cfg).as_ref(),
            &[],
            (0.0, 0.0, 0.0),
            (3.0, 0.0),
            0.0,
            None,
        );
        assert!(produced.poses.len() > 1, "expected a real plan, got a stub");
        assert_eq!(produced.header.frame_id, "odom");
        let ts = msg::path_stamps(&produced);
        for k in 1..ts.len() {
            assert!(ts[k] >= ts[k - 1], "stamp {k} went backwards: {ts:?}");
        }
        assert!(
            ts[ts.len() - 1] > ts[0],
            "a moving plan must advance the clock"
        );
        // and the profile reads back out, which is what the follower does
        let states = msg::path_states(&produced);
        let e = Emb::fixture();
        let params = emb::base_params(&e, [e.min_speed, e.max_speed]);
        assert!(stamps::decode_ceilings(&ts, &states, &params).is_some());
    }

    #[test]
    fn a_tighter_world_stamps_a_slower_plan() {
        let cfg = config();
        let gap = |half: f32| {
            let mut pts = Vec::new();
            let z = obstacles::LOW as f32 + 0.05;
            let mut t = -2.0f32;
            while t <= 2.0 {
                pts.push([1.5, half + t.max(0.0), z]);
                pts.push([1.5, -half - t.max(0.0), z]);
                t += 0.02;
            }
            pts
        };
        let span = |p: &Path| {
            let ts = msg::path_stamps(p);
            ts[ts.len() - 1] - ts[0]
        };
        let m = model(&cfg);
        let roomy = plan_once(
            &cfg,
            &fixture(),
            m.as_ref(),
            &gap(1.4),
            (0.0, 0.0, 0.0),
            (3.0, 0.0),
            0.0,
            None,
        );
        let tight = plan_once(
            &cfg,
            &fixture(),
            m.as_ref(),
            &gap(0.45),
            (0.0, 0.0, 0.0),
            (3.0, 0.0),
            0.0,
            None,
        );
        assert!(roomy.poses.len() > 1 && tight.poses.len() > 1);
        assert!(
            span(&tight) > span(&roomy),
            "tight {:.3}s vs roomy {:.3}s",
            span(&tight),
            span(&roomy)
        );
    }

    #[test]
    fn a_refused_search_publishes_a_stub_at_the_current_pose() {
        // sealed box, the pure crate's own refusal fixture
        let mut walls = Vec::new();
        let z = obstacles::LOW as f32 + 0.05;
        let mut t = -1.0f32;
        while t <= 1.0 {
            walls.push([-1.0, t, z]);
            walls.push([1.0, t, z]);
            walls.push([t, -1.0, z]);
            walls.push([t, 1.0, z]);
            t += 0.02;
        }
        let cfg = config();
        let produced = plan_once(
            &cfg,
            &fixture(),
            model(&cfg).as_ref(),
            &walls,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            0.0,
            None,
        );
        assert_eq!(produced.poses.len(), 1, "a refusal is a one-pose stub");
        assert_eq!(produced.poses[0].pose.position.x, 0.0);
        assert_eq!(produced.poses[0].pose.position.y, 0.0);
    }

    #[test]
    fn a_wall_over_the_body_is_not_a_wall() {
        let mut walls = Vec::new();
        let mut t = -1.0f32;
        while t <= 1.0 {
            walls.push([-1.0, t, 0.2]);
            walls.push([1.0, t, 0.2]);
            walls.push([t, -1.0, 0.2]);
            walls.push([t, 1.0, 0.2]);
            t += 0.02;
        }
        let lifted: Vec<[f32; 3]> = walls.iter().map(|p| [p[0], p[1], p[2] + 2.0]).collect();
        let cfg = config();
        let produced = plan_once(
            &cfg,
            &fixture(),
            model(&cfg).as_ref(),
            &lifted,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            0.0,
            None,
        );
        assert!(produced.poses.len() > 1, "an overhead wall is not a wall");
    }

    /// A sealed box on a floor at `ground_z`, so the map's z origin is base height, not the ground.
    fn room_on_a_floor(ground_z: f32) -> Vec<[f32; 3]> {
        let mut pts = Vec::new();
        let mut t = -2.0f32;
        while t <= 2.0 {
            let mut u = -2.0f32;
            while u <= 2.0 {
                pts.push([t, u, ground_z]);
                u += 0.08;
            }
            // walls 0.10..0.30 m above the ground: under the absolute 0.05..0.45 band at -0.28
            for k in 1..=3 {
                let z = ground_z + 0.1 * k as f32;
                pts.push([-1.0, t, z]);
                pts.push([1.0, t, z]);
                pts.push([t, -1.0, z]);
                pts.push([t, 1.0, z]);
            }
            t += 0.02;
        }
        pts
    }

    #[test]
    fn the_body_band_reads_the_walls_off_the_body_reference() {
        let room = room_on_a_floor(-0.28);
        // under an absolute band the walls are invisible; off the body reference the room is sealed
        let cfg = Config {
            obstacle_model: "body_band".into(),
            ..config()
        };
        let seen = plan_once(
            &cfg,
            &fixture(),
            model(&cfg).as_ref(),
            &room,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            -0.28,
            None,
        );
        assert_eq!(seen.poses.len(), 1, "the walls are still invisible");
    }

    /// Twin of `adapter/test_planner.py::test_a_tall_body_plans_around_what_the_old_band_cut_off`:
    /// a 0.55 m wall is inside a 0.60 m body's band and outside the absolute 0.05..0.45 one.
    #[test]
    fn a_tall_body_plans_around_what_the_old_band_cut_off() {
        let mut wall: Vec<[f32; 3]> = Vec::new();
        let mut y = -1.0f32;
        while y <= 1.0 {
            wall.push([1.5, y, 0.55]);
            y += 0.05;
        }
        let detour = |m: &dyn ObstacleModel| -> f64 {
            let out = plan_once(
                &config(),
                &fixture(),
                m,
                &wall,
                (0.0, 0.0, 0.0),
                (3.0, 0.0),
                0.0,
                None,
            );
            if out.poses.len() < 2 {
                return f64::INFINITY; // a refusal is the strongest "it saw the wall"
            }
            out.poses
                .iter()
                .map(|q| q.pose.position.y.abs())
                .fold(0.0f64, f64::max)
        };
        let tall = Emb {
            height: 0.60,
            ..Emb::fixture()
        };
        let tall_model = obstacles::load("body_band", &tall).expect("known model");
        assert!(
            detour(tall_model.as_ref()) > 0.8,
            "the tall body drove through its own obstacle"
        );
        // control: the same wall is over a go2's belly, so it is not a wall
        let cfg = config();
        let fixture_model = model(&cfg);
        assert!(obstacles::hard_points(fixture_model.as_ref(), &wall, 0.0).is_empty());
        assert!(detour(fixture_model.as_ref()) < 0.2);
    }

    #[test]
    fn the_body_band_drops_the_ground_slab_rather_than_walling_the_robot_in() {
        // quantisation puts the ground's own layer just inside a naive band
        let cfg = Config {
            obstacle_model: "body_band".into(),
            ..config()
        };
        let mut slab = Vec::new();
        let mut t = -2.0f32;
        while t <= 2.0 {
            let mut u = -2.0f32;
            while u <= 2.0 {
                for k in 0..3 {
                    slab.push([t, u, -0.28 + 0.04 * k as f32]);
                }
                u += 0.08;
            }
            t += 0.02;
        }
        let produced = plan_once(
            &cfg,
            &fixture(),
            model(&cfg).as_ref(),
            &slab,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            -0.28,
            None,
        );
        assert!(produced.poses.len() > 1, "the ground read as a wall");
    }

    const CARROT_M: f64 = 0.2;

    #[test]
    fn a_tick_with_nothing_new_does_not_replan() {
        assert!(!replan_due(
            true,
            Some((7, (2.0, 0.0))),
            7,
            (2.0, 0.0),
            CARROT_M
        ));
    }

    #[test]
    fn a_new_map_or_a_moved_carrot_replans() {
        assert!(replan_due(
            true,
            Some((7, (2.0, 0.0))),
            8,
            (2.0, 0.0),
            CARROT_M
        ));
        assert!(replan_due(
            true,
            Some((7, (2.0, 0.0))),
            7,
            (2.3, 0.0),
            CARROT_M
        ));
    }

    #[test]
    fn a_republished_route_moves_the_carrot_by_nothing_and_is_not_a_replan() {
        assert!(!replan_due(
            true,
            Some((7, (2.0, 0.0))),
            7,
            (2.02, -0.01),
            CARROT_M
        ));
    }

    #[test]
    fn the_first_tick_replans_because_nothing_was_planned_yet() {
        assert!(replan_due(true, None, 0, (0.0, 0.0), CARROT_M));
    }

    #[test]
    fn an_ungated_planner_replans_on_every_tick() {
        assert!(replan_due(
            false,
            Some((7, (2.0, 0.0))),
            7,
            (2.0, 0.0),
            CARROT_M
        ));
    }
}
