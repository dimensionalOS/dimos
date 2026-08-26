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

//! `motion_planner`: the SE(2) local planner as a robot-side module.
//!
//! A port of `adapter/planner.py`, which is the specification. The raycaster's
//! `local_map` is the cloud, leveled body odometry is the pose, and the goal is
//! a carrot -- `goal_lookahead_m` of arc along the MLS global route
//! (`planner_path`), clamped to its end. A spawned worker ticks on a fixed
//! cadence but replans only when an input that matters has changed, and
//! publishes the result as a stamped nav Path.
//!
//! A REFUSAL COMES OUT AS THE PLANNER MADE IT: a single-pose stub, which the
//! follower reads as "hold". That is also the shape the staleness guard
//! publishes, because a map that has gone quiet and a search that found no
//! route are the same statement to whatever is downstream -- there is no safe
//! way forward from here.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dimos_module::{native_config, warn_throttled, Input, Module, Output, Tf};
use dimos_motion2_target::planner::{plan, Emb, COMMIT_MARGIN};
use dimos_motion2_tc::{clearance, stamps};
use lcm_msgs::nav_msgs::{Odometry, Path};
use lcm_msgs::sensor_msgs::PointCloud2;
use tracing::{debug, info, warn};
use validator::ValidationError;

use crate::emb;
use crate::msg;
use crate::obstacles::{self, ObstacleModel};
use crate::tf_pose::OdomBasePose;

/// Mirrors `MotionPlannerConfig` (adapter/planner.py). The python `planner`
/// registry field deliberately does not cross: this module IS the rust target
/// planner, and a wrapper that wants a different one is not this module.
#[native_config]
#[derive(Clone)]
#[validate(schema(function = "validate_obstacle_model"))]
pub struct Config {
    /// The body, as `embodiment/base.py` records it -- the python module's
    /// own value, so the two halves cannot plan for different robots.
    pub embodiment: Emb,
    /// Every planning box grown by this much PER SIDE; negative shrinks it,
    /// which is how a deployment asks for a tighter plan than the measured
    /// legs. A gap admits a route when it is `box_width + 2 * precision` wide.
    pub body_dilate_m: f64,
    /// Plan discretisation (m). The python takes it from
    /// `planners/base.py RESOLUTION`; here it crosses explicitly.
    #[validate(range(exclusive_min = 0.0))]
    pub resolution: f64,
    #[validate(range(exclusive_min = 0.0))]
    pub replan_hz: f64,
    /// Carrot arc along the global route.
    #[validate(range(exclusive_min = 0.0))]
    pub goal_lookahead_m: f64,
    pub world_frame: String,
    /// Odometry is stamped at the SENSOR, so the pose it carries is the
    /// lidar's, not the robot's. tf resolves it into the body; messages are
    /// dropped until the mount leg arrives.
    pub base_frame: String,
    /// Plan when an input that MATTERS changed -- a new local map, or a carrot
    /// that moved -- rather than on every tick of the clock. The planner ticks
    /// at 5 Hz over a 1 Hz map, so four ticks in five re-solve an unchanged
    /// world; the follower tracks the published path as the robot moves and
    /// needs no republish to do it.
    pub replan_on_change: bool,
    /// How far the carrot has to move to be worth re-solving for. The route it
    /// rides on is republished at ~1 Hz with its head trimmed to the robot and
    /// its tail re-solved, so the waypoints move every time and the carrot does
    /// not -- gating on the array would dedup nothing.
    ///
    #[validate(range(min = 0.0))]
    pub replan_carrot_m: f64,
    /// A carrot that jumped this far is a different task, and the route this
    /// module is holding on to is about the old one -- so it is dropped and the
    /// next search starts from nothing. Republish noise moves the carrot ~0 m; a
    /// real reroute moved it 4.6 m in the door recording. The python twin of
    /// this field used to stop at the module boundary, back when the search
    /// itself was stateless; the incumbent is an input now, so it crosses.
    #[validate(range(min = 0.0))]
    pub reset_carrot_m: f64,
    /// What counts as an obstacle (`obstacles.rs`). "body_band" reads the cloud
    /// against the surface the feet stand on, which the embodiment knows the
    /// base's height above.
    pub obstacle_model: String,
    /// Hold once the local map is this old, measured from ARRIVAL.
    #[validate(range(exclusive_min = 0.0))]
    pub max_map_age_s: f64,
    /// Rate cap for mirroring the plan onto the viewer stream. 0 disables it.
    #[validate(range(min = 0.0))]
    pub viz_publish_hz: f64,
}

fn validate_obstacle_model(config: &Config) -> Result<(), ValidationError> {
    if !obstacles::MODELS.contains(&config.obstacle_model.as_str()) {
        return Err(ValidationError::new(
            "obstacle_model must be one of: body_band",
        ));
    }
    Ok(())
}

/// Everything the handlers record and the worker reads. Only the newest of
/// each is kept: a replan is a fresh look at the world, never a queue.
#[derive(Default)]
struct Shared {
    cloud: Option<Arc<PointCloud2>>,
    /// Bumped per arrival so the worker can cache the extracted points.
    cloud_seq: u64,
    /// ARRIVAL, not `msg.ts`: the mapper's clock is not the robot's, and what
    /// this guards is how long since the mapper was last heard from.
    cloud_at: Option<Instant>,
    pose: Option<(f64, f64, f64)>,
    /// Where the surface under the robot is, off the BODY rather than off the
    /// scene: the base rides `Emb::base_height` above it.
    ground_z: Option<f64>,
    global_xy: Option<Vec<[f64; 2]>>,
}

#[derive(Module)]
#[module(name = "motion_planner", setup = spawn_worker, teardown = stop_worker)]
pub struct MotionPlanner {
    #[input(decode = PointCloud2::decode, handler = on_local_map)]
    local_map: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[input(decode = Path::decode, handler = on_planner_path)]
    planner_path: Input<Path>,

    #[output(encode = Path::encode)]
    path: Output<Path>,

    /// The same plan, for the viewer's body boxes, at its own rate.
    #[output(encode = Path::encode)]
    plan_body: Output<Path>,

    #[config]
    config: Config,

    #[tf]
    tf: Tf,

    /// Built on the first odometry message: `Tf` is only handed over at build
    /// time, and the base frame is config the constructor does not see.
    base_pose: Option<OdomBasePose>,
    shared: Arc<Mutex<Shared>>,
    worker: Option<tokio::task::JoinHandle<()>>,
}

impl MotionPlanner {
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
            plan_body: self.plan_body.clone(),
        };
        self.worker = Some(tokio::spawn(worker.run()));
    }

    async fn stop_worker(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
    }

    // Handlers do nothing but record. Anything slower would back up every
    // other input of this module, and the replan is the slow part.

    async fn on_local_map(&mut self, msg: PointCloud2) {
        let mut s = self.shared.lock().expect("shared mutex");
        s.cloud = Some(Arc::new(msg));
        s.cloud_seq = s.cloud_seq.wrapping_add(1);
        s.cloud_at = Some(Instant::now());
    }

    async fn on_odometry(&mut self, msg: Odometry) {
        let resolver = self
            .base_pose
            .get_or_insert_with(|| OdomBasePose::new(self.tf.clone(), &self.config.base_frame));
        let Some(iso) = resolver.resolve_iso(&msg) else {
            return;
        };
        let pose = crate::tf_pose::state_of(&iso);
        let base_height = self.config.embodiment.base_height;
        let mut s = self.shared.lock().expect("shared mutex");
        s.pose = Some((pose[0], pose[1], pose[2]));
        s.ground_z = Some(iso.translation.z - base_height);
    }

    /// MLS emits an empty path when it finds no route: no carrot, so hold the
    /// last local plan rather than chase a stale one.
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

/// `lookahead` metres of arc along the route from the waypoint closest to the
/// robot, clamped to the route's end. A port of `planner.carrot_along`.
pub fn carrot_along(route: &[[f64; 2]], robot: (f64, f64), lookahead: f64) -> Option<(f64, f64)> {
    let last = route.last()?;
    // np.argmin keeps the FIRST minimum on a tie
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

/// Has an input the plan depends on moved since the plan was made?
///
/// The plan consumes the global route through exactly one quantity -- the
/// carrot -- so that is what the gate compares. Keying on the waypoint array
/// instead never dedups anything: MLS trims the route head to the robot on
/// every ~1 Hz republish and re-solves with tail wobble, so the array moves
/// every time while the carrot does not move at all.
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

/// What one tick of the replan loop should do. The python `_plan_loop`'s
/// three-way branch, lifted out of the loop so it can be tested without a
/// transport.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Tick {
    /// The map has gone quiet. Refuse, and say so once.
    Hold { age_s: f64 },
    /// Everything the search needs has arrived.
    Plan,
    /// Something has never arrived. Nothing to publish yet.
    Wait,
}

pub fn decide(
    has_pose: bool,
    cloud_age_s: Option<f64>,
    has_route: bool,
    max_map_age_s: f64,
) -> Tick {
    match (has_pose, cloud_age_s) {
        // The stale branch comes FIRST and needs no route: a frozen map at
        // cruise speed is the failure this guards, and waiting for a global
        // route before refusing would leave the follower on the last plan.
        (true, Some(age)) if age > max_map_age_s => Tick::Hold { age_s: age },
        (true, Some(_)) if has_route => Tick::Plan,
        _ => Tick::Wait,
    }
}

/// The planner's refusal: one pose at where the robot is, which every law
/// reads as "there is nothing to follow, hold position".
pub fn hold_stub(pose: (f64, f64, f64), frame_id: &str, ts: f64, ground_z: f64) -> Path {
    msg::build_path(&[[pose.0, pose.1, pose.2]], &[ts], ts, frame_id, ground_z)
}

/// Edge trigger for the stale spell, so a dead map warns once rather than
/// `replan_hz` times a second for as long as it stays dead.
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

/// Rate cap for a stream that is watched rather than consumed.
pub struct RateCap {
    hz: f64,
    last: Option<Instant>,
}

impl RateCap {
    pub fn new(hz: f64) -> Self {
        Self { hz, last: None }
    }

    pub fn due(&mut self, now: Instant) -> bool {
        if self.hz <= 0.0 {
            return false;
        }
        let period = Duration::from_secs_f64(1.0 / self.hz);
        if self.last.is_none_or(|t| now.duration_since(t) >= period) {
            self.last = Some(now);
            return true;
        }
        false
    }
}

/// One search plus the precision annotation: `planner.py::_plan_once` and
/// `planner.py::annotate`, in that order.
///
/// Free rather than a method so it can be exercised with no transport, which
/// is the whole reason the async shell above stays as thin as it is.
// The argument list IS the module's own state, laid out: everything the search
// reads, named at the call. A struct would only move the same names one
// indirection away from it.
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
    // The search gets the obstacles, as xy: which returns are obstacles was
    // decided here, by the model, and the search has no z to decide it again
    // with. The follower's room hint is measured off the very same points, so
    // the governor and the stamped profile cannot be pricing different worlds.
    let hard = obstacles::hard_points(model, points, ground_z);
    let points: &[[f32; 3]] = &hard;
    let cloud: Vec<[f64; 2]> = points.iter().map(|p| [p[0] as f64, p[1] as f64]).collect();
    let states = match plan(
        &cloud,
        pose,
        goal,
        emb,
        config.resolution,
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
    // The room hint the follower's governor reads back out of the stamps has
    // to be the same quantity, measured off the same cloud, or the plan is
    // priced for a world the follower does not see.
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
    plan_body: Output<Path>,
}

/// The handlers' snapshot, taken once per tick under one lock.
struct Snapshot {
    cloud: Option<Arc<PointCloud2>>,
    cloud_seq: u64,
    age_s: Option<f64>,
    pose: Option<(f64, f64, f64)>,
    ground_z: Option<f64>,
    route: Option<Vec<[f64; 2]>>,
}

impl Worker {
    async fn run(self) {
        let mut ticker =
            tokio::time::interval(Duration::from_secs_f64(1.0 / self.config.replan_hz));
        // A tick missed because a replan overran is a tick that is gone; firing
        // the backlog immediately afterwards would only make the next one late
        // too.
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        let mut gate = StaleGate::default();
        let mut viz = RateCap::new(self.config.viz_publish_hz);
        let mut points: Option<(u64, Arc<Vec<[f32; 3]>>)> = None;
        // The (cloud, carrot) the published plan was made from.
        let mut planned: Option<(u64, (f64, f64))> = None;
        // ...and the plan itself. The search prefers the route it already
        // published unless a fresh one earns the switch, and this module is
        // where that memory lives: the shell owns it, the planner judges it.
        let mut incumbent: Option<Vec<[f64; 3]>> = None;

        loop {
            ticker.tick().await;
            let now = Instant::now();
            let snap = self.snapshot(now);
            match decide(
                snap.pose.is_some(),
                snap.age_s,
                snap.route.is_some(),
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
                    // A hold is not gated: it is a statement about the CLOCK,
                    // and nothing arriving is exactly the case it fires on.
                    // Forget what was planned so the first live tick plans, and
                    // what was published with it: a route held across a dead
                    // link is a route nothing has re-validated.
                    planned = None;
                    incumbent = None;
                    let pose = snap.pose.expect("Hold implies a pose");
                    let held = hold_stub(
                        pose,
                        &self.config.world_frame,
                        msg::now_secs(),
                        snap.ground_z.unwrap_or(0.0),
                    );
                    self.publish(&held, now, &mut viz).await;
                }
                Tick::Plan => {
                    if gate.recover() {
                        info!("local_map is live again, resuming planning");
                    }
                    let pose = snap.pose.expect("Plan implies a pose");
                    let route = snap.route.expect("Plan implies a route");
                    // the carrot is the whole of the route the plan consumes,
                    // so it is taken every tick and the gate reads it
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
                    // A carrot that jumped is a different task, and the route
                    // being held is about the old one.
                    if planned.is_some_and(|(_, was)| {
                        (was.0 - goal.0).hypot(was.1 - goal.1) > self.config.reset_carrot_m
                    }) {
                        incumbent = None;
                    }
                    let cloud = snap.cloud.expect("Plan implies a cloud");
                    // The search is the expensive call in this process, and it
                    // is synchronous. block_in_place keeps it off the runtime's
                    // async workers, which is why the module asks for 2 threads.
                    let produced = tokio::task::block_in_place(|| {
                        let pts = self.points(&cloud, snap.cloud_seq, &mut points)?;
                        Some(plan_once(
                            &self.config,
                            &self.emb,
                            self.model.as_ref(),
                            &pts,
                            pose,
                            goal,
                            snap.ground_z.unwrap_or(0.0),
                            incumbent.as_deref(),
                        ))
                    });
                    if let Some(produced) = produced {
                        planned = Some((snap.cloud_seq, goal));
                        incumbent = Some(msg::path_states(&produced));
                        self.publish(&produced, now, &mut viz).await;
                    }
                }
                Tick::Wait => warn_throttled!(
                    Duration::from_secs(3),
                    local_map = snap.cloud.is_some(),
                    odometry = snap.pose.is_some(),
                    planner_path = snap.route.is_some(),
                    "nothing planned: an input the planner needs has not arrived",
                ),
            }
        }
    }

    fn snapshot(&self, now: Instant) -> Snapshot {
        let s = self.shared.lock().expect("shared mutex");
        Snapshot {
            cloud: s.cloud.clone(),
            cloud_seq: s.cloud_seq,
            age_s: s.cloud_at.map(|t| now.duration_since(t).as_secs_f64()),
            pose: s.pose,
            ground_z: s.ground_z,
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

    /// The plan on `path`, and a mirror on `plan_body` at the viz rate.
    ///
    /// A refusal is drawn too: an empty viewport looks like a crashed module,
    /// and a held stub is a decision the operator has to be able to see.
    async fn publish(&self, plan: &Path, now: Instant, viz: &mut RateCap) {
        msg::publish(&self.path, plan).await;
        if viz.due(now) {
            msg::publish(&self.plan_body, plan).await;
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

    // carrot_along -- the cases in adapter/test_planner.py

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
        // the arc starts at the nearest waypoint, so a robot at the far end
        // gets the end, not a point behind it
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

    // the tick branch

    #[test]
    fn a_stale_map_holds_even_with_everything_else_present() {
        assert_eq!(
            decide(true, Some(7.0), true, 5.0),
            Tick::Hold { age_s: 7.0 }
        );
        assert_eq!(
            decide(true, Some(7.0), false, 5.0),
            Tick::Hold { age_s: 7.0 }
        );
    }

    #[test]
    fn a_live_map_with_a_pose_and_a_route_plans() {
        assert_eq!(decide(true, Some(0.2), true, 5.0), Tick::Plan);
    }

    #[test]
    fn a_missing_input_waits_rather_than_holding() {
        // no pose is "not running yet", not "the map died" -- publishing a
        // hold stub would need a pose to put it at anyway
        assert_eq!(decide(false, Some(9.0), true, 5.0), Tick::Wait);
        assert_eq!(decide(true, None, true, 5.0), Tick::Wait);
        // an empty MLS route: hold the last plan, do not chase a stale one
        assert_eq!(decide(true, Some(0.2), false, 5.0), Tick::Wait);
    }

    #[test]
    fn the_age_boundary_is_exclusive() {
        assert_eq!(decide(true, Some(5.0), true, 5.0), Tick::Plan);
    }

    // the refusal shape

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
    fn a_hold_stub_is_a_stop_to_every_law() {
        // one pose is under the laws' `path.len() < 2` veto, which is the whole
        // contract the refusal shape rests on
        let held = hold_stub((1.5, -2.0, 0.0), "odom", 0.0, 0.0);
        let states = msg::path_states(&held);
        let cfg = emb::hinted_params(&Emb::go2());
        assert_eq!(
            dimos_motion2_tc::laws::hinted::update((1.5, -2.0, 0.0), &states, None, &cfg),
            (0.0, 0.0, 0.0)
        );
    }

    // the log gates

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

    #[test]
    fn the_rate_cap_passes_the_first_call_then_holds() {
        let mut cap = RateCap::new(2.0);
        let t0 = Instant::now();
        assert!(cap.due(t0));
        assert!(!cap.due(t0 + Duration::from_millis(100)));
        assert!(cap.due(t0 + Duration::from_millis(600)));
    }

    #[test]
    fn a_zero_rate_cap_is_off() {
        let mut cap = RateCap::new(0.0);
        let t0 = Instant::now();
        assert!(!cap.due(t0));
        assert!(!cap.due(t0 + Duration::from_secs(60)));
    }

    // the annotation, end to end

    fn config() -> Config {
        Config {
            embodiment: Emb::go2(),
            body_dilate_m: 0.0,
            resolution: 0.1,
            replan_hz: 5.0,
            goal_lookahead_m: 5.0,
            world_frame: "odom".into(),
            base_frame: "base_link".into(),
            replan_on_change: true,
            replan_carrot_m: 0.2,
            reset_carrot_m: 1.0,
            obstacle_model: "body_band".into(),
            max_map_age_s: 5.0,
            viz_publish_hz: 2.0,
        }
    }

    fn go2() -> Emb {
        Emb::go2()
    }

    /// The model a config names, for the plan_once calls below.
    fn model(cfg: &Config) -> Box<dyn ObstacleModel> {
        obstacles::load(&cfg.obstacle_model, &cfg.embodiment).expect("known model")
    }

    #[test]
    fn a_planned_path_carries_monotone_stamps() {
        // open floor: the search runs and the profile prices every segment
        let cfg = config();
        let produced = plan_once(
            &cfg,
            &go2(),
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
        let params = emb::blind_params(&Emb::go2()).base;
        assert!(stamps::decode_ceilings(&ts, &states, &params).is_some());
    }

    #[test]
    fn a_tighter_world_stamps_a_slower_plan() {
        // the annotation is the point of the module: the same route through a
        // narrower gap has to come out priced slower, or the follower's
        // governor is reading nothing
        let cfg = config();
        let gap = |half: f32| {
            let mut pts = Vec::new();
            let mut t = -2.0f32;
            while t <= 2.0 {
                pts.push([1.5, half + t.max(0.0), 0.2]);
                pts.push([1.5, -half - t.max(0.0), 0.2]);
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
            &go2(),
            m.as_ref(),
            &gap(1.4),
            (0.0, 0.0, 0.0),
            (3.0, 0.0),
            0.0,
            None,
        );
        let tight = plan_once(
            &cfg,
            &go2(),
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
        let mut t = -1.0f32;
        while t <= 1.0 {
            walls.push([-1.0, t, 0.2]);
            walls.push([1.0, t, 0.2]);
            walls.push([t, -1.0, 0.2]);
            walls.push([t, 1.0, 0.2]);
            t += 0.02;
        }
        let cfg = config();
        let produced = plan_once(
            &cfg,
            &go2(),
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
        // the band has a ceiling: the same wall, lifted over the belly, stops
        // being an obstacle at all
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
            &go2(),
            model(&cfg).as_ref(),
            &lifted,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            0.0,
            None,
        );
        assert!(produced.poses.len() > 1, "an overhead wall is not a wall");
    }

    // the obstacle model

    /// A sealed box on a ground surface at `ground_z`, the whole thing sunk so
    /// the map's z origin is base height rather than the ground -- the
    /// recording's case.
    fn room_on_a_floor(ground_z: f32) -> Vec<[f32; 3]> {
        let mut pts = Vec::new();
        let mut t = -2.0f32;
        while t <= 2.0 {
            let mut u = -2.0f32;
            while u <= 2.0 {
                pts.push([t, u, ground_z]); // the ground slab
                u += 0.08;
            }
            // walls, standing 0.10..0.30 m ABOVE that ground -- entirely under
            // the absolute 0.05..0.45 band once the ground is at -0.28
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
        // the walls sit under an absolute band; off the body's own reference
        // the same room is a sealed box
        let cfg = Config {
            obstacle_model: "body_band".into(),
            ..config()
        };
        let seen = plan_once(
            &cfg,
            &go2(),
            model(&cfg).as_ref(),
            &room,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            -0.28,
            None,
        );
        assert_eq!(seen.poses.len(), 1, "the walls are still invisible");
    }

    /// The latent bug the planar search contract closes -- twin of
    /// `adapter/test_planner.py::test_a_tall_body_plans_around_what_the_old_band_cut_off`.
    ///
    /// A 0.55 m wall is inside a 0.60 m body's band and outside the absolute
    /// 0.05..0.45 one. While the search re-sliced that band on its way in, it
    /// dropped the very points the model had correctly kept, and the body
    /// drove straight through them.
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
                &go2(),
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
            ..Emb::go2()
        };
        let tall_model = obstacles::load("body_band", &tall).expect("known model");
        assert!(
            detour(tall_model.as_ref()) > 0.8,
            "the tall body drove through its own obstacle"
        );
        // and the control: the same wall IS over a go2's belly, so it is not a wall
        let cfg = config();
        let go2_model = model(&cfg);
        assert!(obstacles::hard_points(go2_model.as_ref(), &wall, 0.0).is_empty());
        assert!(detour(go2_model.as_ref()) < 0.2);
    }

    #[test]
    fn the_body_band_drops_the_ground_slab_rather_than_walling_the_robot_in() {
        // the +0.29 counterfactual in the diagnosis: quantisation puts the
        // ground's own layer just inside a naive band, and every tick refuses
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
            &go2(),
            model(&cfg).as_ref(),
            &slab,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            -0.28,
            None,
        );
        assert!(produced.poses.len() > 1, "the ground read as a wall");
    }

    // the replan gate

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
        // MLS trims the route head to the robot and re-solves the tail on
        // every ~1 Hz republish: the waypoints move, the carrot does not
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
