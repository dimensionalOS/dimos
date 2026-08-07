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
//! python twin; this module owns the subscriptions, the control clock, the
//! on-robot clearance annotation, goal arrival and the deadman.
//!
//! CONFIG NAMES A TRACK, NEVER A LAW. `control/tracks.py` is the one map from
//! track to law, and folding in a research generation is a one-line change
//! there. Nothing outside [`Track`] in this file mentions a law by name.
//!
//! THE DEADMAN. On the laptop the link was the deadman: drop it and `cmd_vel`
//! stopped and the bridge watchdog halted the robot. Co-located with the
//! planner that accidental safety is gone, so `max_path_age_s` is its
//! replacement -- measured from ARRIVAL, and it guards a dead planner rather
//! than a dead link. The planner's own hold stub covers a stale map; this
//! covers a planner that stopped speaking at all.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dimos_module::{native_config, warn_throttled, Input, Module, Output, Tf};
use dimos_motion2_tc::geom::Params;
use dimos_motion2_tc::laws::blind::{update as blind_update, BlindParams};
use dimos_motion2_tc::laws::hinted::{HintedParams, Law as HintedLaw};
use dimos_motion2_tc::{clearance, stamps};
use lcm_msgs::geometry_msgs::Twist;
use lcm_msgs::nav_msgs::{Odometry, Path};
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::std_msgs::Bool;
use serde::{Deserialize, Serialize};
use tracing::info;
use validator::{Validate, ValidationError};

use crate::emb;
use crate::floor;
use crate::msg::{self, State};
use crate::tf_pose::OdomBasePose;

/// The input regime the follower runs under. `control/tracks.py` is the source
/// of truth; this is that map, and the only place in this crate that knows
/// which law a track runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Track {
    /// Handed the per-waypoint room recomputed from the robot's own local map.
    Hinted,
    /// Room withheld: the law recovers the required precision from the path's
    /// own timestamps instead.
    Blind,
}

impl Track {
    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "hinted" => Some(Track::Hinted),
            "blind" => Some(Track::Blind),
            _ => None,
        }
    }

    /// Does this track get the clearance array at all?
    pub fn annotate_clearance(self) -> bool {
        matches!(self, Track::Hinted)
    }
}

/// Mirror of the python `ControllerConfig`, flat, and nested inside [`Config`]
/// exactly as pydantic's `model_dump()` nests it.
///
/// Hand-written derives rather than `#[native_config]`: that attribute's checks
/// apply to a module's TOP-level config, and the one-to-one key check at
/// startup does not recurse. What keeps this honest is `deny_unknown_fields`
/// plus every field being required -- serde's missing-field error is the
/// backstop, and it names the field.
#[derive(Debug, Clone, Deserialize, Serialize, Validate)]
#[serde(deny_unknown_fields)]
pub struct ControllerConfig {
    // --- the eleven every law reads (`ControllerConfig.law_params`) ---
    pub lookahead: f64,
    pub max_speed: f64,
    pub max_yaw_rate: f64,
    pub k_pos: f64,
    pub k_yaw: f64,
    pub fan_yaw_per_m: f64,
    pub fan_yaw_done: f64,
    pub min_speed: f64,
    pub speed_clearance: f64,
    pub speed_floor_clearance: f64,
    pub speed_lookahead: f64,
    // --- gait calibration, read by the blind law only ---
    pub walk_gain: f64,
    pub walk_slip: f64,
    pub walk_slip_ramp: f64,
    // --- read by the hinted law only ---
    pub tangent_preview: f64,
    pub escape_clearance: f64,
    pub escape_preview: f64,
    pub escape_speed: f64,
    pub brake_accel: f64,
    pub brake_margin: f64,
}

impl ControllerConfig {
    pub fn base(&self) -> Params {
        Params {
            lookahead: self.lookahead,
            max_speed: self.max_speed,
            max_yaw_rate: self.max_yaw_rate,
            k_pos: self.k_pos,
            k_yaw: self.k_yaw,
            fan_yaw_per_m: self.fan_yaw_per_m,
            fan_yaw_done: self.fan_yaw_done,
            min_speed: self.min_speed,
            speed_clearance: self.speed_clearance,
            speed_floor_clearance: self.speed_floor_clearance,
            speed_lookahead: self.speed_lookahead,
        }
    }

    pub fn hinted(&self) -> HintedParams {
        HintedParams {
            base: self.base(),
            tangent_preview: self.tangent_preview,
            escape_clearance: self.escape_clearance,
            escape_preview: self.escape_preview,
            escape_speed: self.escape_speed,
            brake_accel: self.brake_accel,
            brake_margin: self.brake_margin,
        }
    }

    pub fn blind(&self) -> BlindParams {
        BlindParams {
            base: self.base(),
            walk_gain: self.walk_gain,
            walk_slip: self.walk_slip,
            slip_ramp: self.walk_slip_ramp,
        }
    }
}

/// Mirrors `TrajectoryFollowerConfig` (adapter/follower.py).
#[native_config]
#[derive(Clone)]
#[validate(schema(function = "validate_track_and_embodiment"))]
pub struct Config {
    /// A TRACK name, never a law. See [`Track`].
    pub track: String,
    #[validate(nested)]
    pub controller_config: ControllerConfig,
    #[validate(range(exclusive_min = 0.0))]
    pub control_frequency: f64,
    /// Planar distance that counts as arrival (m).
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f64,
    /// Names the body rather than a number, so `emb.width / 2` is read the same
    /// way the planner read it when it priced the plan.
    pub embodiment: String,
    /// Odometry is stamped at the SENSOR, so the pose it carries is the
    /// lidar's, not the robot's. tf resolves it into the body; messages are
    /// dropped until the mount leg arrives.
    pub base_frame: String,
    /// Anchor the local map to the floor under the robot before the room hint
    /// is measured off it, so the band read here is the band the planner
    /// planned in. The planner carries the same three knobs -- see `floor.rs`.
    pub floor_anchor: bool,
    /// Lidar height above the ground while standing. With it, tf gives the base
    /// height above ground and hence the floor prior; 0 leaves the band where
    /// the map's z origin puts it.
    #[validate(range(min = 0.0))]
    pub lidar_height: f64,
    /// Drop returns within this of the estimated floor before the band is
    /// taken. TWO voxel layers, not one -- see the planner's `ground_margin_m`.
    #[validate(range(min = 0.0))]
    pub ground_margin_m: f64,
    /// A commanded speed at or under this is standing still, whatever the
    /// reason. Classification for the stall log only; it commands nothing.
    #[validate(range(min = 0.0))]
    pub idle_speed: f64,
    /// Zero the twist once the held path is this old, measured from ARRIVAL.
    #[validate(range(exclusive_min = 0.0))]
    pub max_path_age_s: f64,
}

fn validate_track_and_embodiment(config: &Config) -> Result<(), ValidationError> {
    if Track::parse(&config.track).is_none() {
        return Err(ValidationError::new("track must be one of: hinted, blind"));
    }
    if emb::by_tag(&config.embodiment).is_none() {
        return Err(ValidationError::new(
            "embodiment must be one of: go2, go2-payload, slim, diffdrive",
        ));
    }
    Ok(())
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
    pose: Option<(f64, f64, f64)>,
    /// Where tf puts the ground under the base: the floor estimate's bound.
    floor_prior: Option<f64>,
    path: Option<Arc<Path>>,
    /// ARRIVAL, not `msg.ts`: what this guards is how long since the planner
    /// was last heard from.
    path_at: Option<Instant>,
    path_seq: u64,
    cloud: Option<Arc<PointCloud2>>,
    cloud_seq: u64,
    /// Goals in the order the paths carrying them arrived, drained per tick.
    /// A queue rather than "the latest path's goal" so the latch sees the same
    /// `set_goal` SEQUENCE the python's subscription callback sees, even when
    /// two plans land inside one control period.
    goals: Vec<(f64, f64)>,
    /// Bumped by `stop_movement`; the worker resets the law when it changes.
    reset_epoch: u64,
}

#[derive(Module)]
#[module(name = "trajectory_follower", setup = spawn_worker, teardown = stop_worker)]
pub struct TrajectoryFollower {
    #[input(decode = Path::decode, handler = on_path)]
    path: Input<Path>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[input(decode = PointCloud2::decode, handler = on_local_map)]
    local_map: Input<PointCloud2>,

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

    /// Built on the first odometry message: `Tf` is only handed over at build
    /// time, and the base frame is config the constructor does not see.
    base_pose: Option<OdomBasePose>,
    /// How far the base sits above the ground, looked up once off the mount leg.
    base_height: Option<f64>,
    shared: Arc<Mutex<Shared>>,
    worker: Option<tokio::task::JoinHandle<()>>,
}

impl TrajectoryFollower {
    async fn spawn_worker(&mut self) {
        let worker = Worker {
            shared: Arc::clone(&self.shared),
            track: Track::parse(&self.config.track).expect("validated track"),
            half_width: emb::half_width(
                &emb::by_tag(&self.config.embodiment).expect("validated embodiment tag"),
            ),
            config: self.config.clone(),
            nav_cmd_vel: self.nav_cmd_vel.clone(),
            goal_reached: self.goal_reached.clone(),
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
        s.path_seq = s.path_seq.wrapping_add(1);
        if let Some(goal) = goal {
            s.goals.push(goal);
        }
    }

    async fn on_odometry(&mut self, msg: Odometry) {
        let resolver = self
            .base_pose
            .get_or_insert_with(|| OdomBasePose::new(self.tf.clone(), &self.config.base_frame));
        let Some(iso) = resolver.resolve_iso(&msg) else {
            return;
        };
        if self.base_height.is_none() && self.config.lidar_height > 0.0 {
            self.base_height =
                resolver.base_height_above_ground(&msg.child_frame_id, self.config.lidar_height);
        }
        let pose = crate::tf_pose::state_of(&iso);
        let mut s = self.shared.lock().expect("shared mutex");
        s.pose = Some((pose[0], pose[1], pose[2]));
        s.floor_prior = self.base_height.map(|h| iso.translation.z - h);
    }

    async fn on_local_map(&mut self, msg: PointCloud2) {
        let mut s = self.shared.lock().expect("shared mutex");
        s.cloud = Some(Arc::new(msg));
        s.cloud_seq = s.cloud_seq.wrapping_add(1);
    }

    /// Preemption: drop the plan, stop, and forget the law's history. The zero
    /// goes out here rather than waiting for the next tick, because a stop
    /// request that takes a control period to land is not a stop request.
    async fn on_stop_movement(&mut self, msg: Bool) {
        if !msg.data {
            return;
        }
        {
            let mut s = self.shared.lock().expect("shared mutex");
            s.path = None;
            s.path_at = None;
            s.reset_epoch = s.reset_epoch.wrapping_add(1);
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

/// The extracted cloud and the room hint, each rebuilt only when its input
/// changes. `follower.py::_clearance_for` caches on the same key.
#[derive(Default)]
struct Cache {
    /// Keyed by cloud arrival.
    points: Option<(u64, Arc<Vec<[f32; 3]>>)>,
    /// Keyed by the (path, cloud) pair it was measured from.
    room: Option<((u64, u64), Room)>,
}

type Room = Arc<Vec<f64>>;

/// The dialect's own band, which is what the python fallback decodes with.
///
/// `decode_ceilings` takes the CONSUMER's band so a controller recovers the
/// ceiling its own governor would have produced -- but this path is not that.
/// It is `follower.py`'s `decode_ceilings(path)` with no arguments, i.e. the
/// module constants, because the result is about to be bent back through the
/// encoder's inverse. Reading with one band and writing with another would
/// re-price every waypoint.
fn dialect_band() -> Params {
    Params {
        min_speed: stamps::MIN_SPEED,
        max_speed: stamps::MAX_SPEED,
        ..Params::default()
    }
}

/// The per-waypoint room hint off the floor-anchored map: the twin of
/// `follower.py::_clearance_for`'s recompute branch.
///
/// THE BAND HAS TO BE THE PLANNER'S BAND. `path_clearance` slices an absolute
/// 0.05..0.45 m, so measuring it on the raw map on a LIO stack governs the
/// speed by a slab over the robot's head-room while the plan was priced 0.33 m
/// lower. Free rather than a method so it can be exercised with no transport.
pub fn measure_room(
    config: &Config,
    points: &[[f32; 3]],
    pose: (f64, f64, f64),
    floor_prior: Option<f64>,
    states: &[State],
    half_width: f64,
) -> Vec<f64> {
    // Anchored per (path, map) pair like the hint itself: the floor under the
    // robot moves far slower than the pair it is cached with.
    let anchored = floor::anchored_cloud(
        points,
        (pose.0, pose.1),
        floor_prior,
        config.floor_anchor,
        config.ground_margin_m,
    );
    let xy: Vec<[f64; 2]> = states.iter().map(|s| [s[0], s[1]]).collect();
    clearance::path_clearance(&xy, &anchored, half_width)
}

struct Worker {
    shared: Arc<Mutex<Shared>>,
    config: Config,
    track: Track,
    half_width: f64,
    nav_cmd_vel: Output<Twist>,
    goal_reached: Output<Bool>,
}

struct Snapshot {
    pose: Option<(f64, f64, f64)>,
    floor_prior: Option<f64>,
    path: Option<Arc<Path>>,
    path_seq: u64,
    age_s: Option<f64>,
    cloud: Option<Arc<PointCloud2>>,
    cloud_seq: u64,
    goals: Vec<(f64, f64)>,
    reset_epoch: u64,
}

impl Worker {
    async fn run(self) {
        let mut ticker =
            tokio::time::interval(Duration::from_secs_f64(1.0 / self.config.control_frequency));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        let hinted = self.config.controller_config.hinted();
        let blind = self.config.controller_config.blind();
        let mut law = HintedLaw::new();
        let mut latch = GoalLatch::new(self.config.goal_tolerance);
        let mut stale = Gate::default();
        let mut cache = Cache::default();
        let mut epoch = 0u64;
        let started = Instant::now();

        loop {
            ticker.tick().await;
            let now = Instant::now();
            let snap = self.snapshot(now);
            for goal in &snap.goals {
                latch.set_goal(*goal);
            }
            if snap.reset_epoch != epoch {
                epoch = snap.reset_epoch;
                law.reset();
            }

            match decide(
                snap.pose,
                snap.age_s,
                self.config.max_path_age_s,
                &mut latch,
            ) {
                Tick::Idle => warn_throttled!(
                    Duration::from_secs(3),
                    odometry = snap.pose.is_some(),
                    path = snap.path.is_some(),
                    "not driving: odometry or a local plan has not arrived",
                ),
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
                    let pose = snap.pose.expect("Drive implies a pose");
                    let path = snap.path.clone().expect("Drive implies a path");
                    let states = msg::path_states(&path);
                    let ts = msg::path_stamps(&path);
                    let room = tokio::task::block_in_place(|| {
                        self.room(&snap, pose, &states, &ts, &mut cache)
                    });
                    let t = started.elapsed().as_secs_f64();
                    let (vx, vy, wz) = match self.track {
                        Track::Hinted => law.step(
                            pose,
                            &states,
                            room.as_ref().map(|r| r.as_slice()),
                            &hinted,
                            t,
                        ),
                        // the blind law decodes the dialect itself, which is
                        // the whole point of the track
                        Track::Blind => blind_update(pose, &states, None, Some(&ts), &blind),
                    };
                    self.publish_twist(vx, vy, wz).await;
                    self.report(vx, vy, wz, states.len());
                }
            }
        }
    }

    fn snapshot(&self, now: Instant) -> Snapshot {
        let mut s = self.shared.lock().expect("shared mutex");
        Snapshot {
            pose: s.pose,
            floor_prior: s.floor_prior,
            path: s.path.clone(),
            path_seq: s.path_seq,
            age_s: s.path_at.map(|t| now.duration_since(t).as_secs_f64()),
            cloud: s.cloud.clone(),
            cloud_seq: s.cloud_seq,
            goals: std::mem::take(&mut s.goals),
            reset_epoch: s.reset_epoch,
        }
    }

    /// The per-waypoint room hint for this tick, or `None` when the law is to
    /// run blind of it.
    fn room(
        &self,
        snap: &Snapshot,
        pose: (f64, f64, f64),
        states: &[State],
        ts: &[f64],
        cache: &mut Cache,
    ) -> Option<Arc<Vec<f64>>> {
        if !self.track.annotate_clearance() {
            return None;
        }
        let Some(cloud) = snap.cloud.as_ref() else {
            // No local map of our own: fall back to the precision the planner
            // stamped into the path's own timestamps. The hinted law reads
            // clearance and nothing else, so the ceilings are bent back
            // through the encoder's inverse rather than the law being changed.
            let ceilings = stamps::decode_ceilings(ts, states, &dialect_band())?;
            return Some(Arc::new(stamps::ceilings_to_clearance(&ceilings)));
        };
        let key = (snap.path_seq, snap.cloud_seq);
        if let Some((cached, room)) = &cache.room {
            if *cached == key {
                return Some(Arc::clone(room));
            }
        }
        let points = self.points(cloud, snap.cloud_seq, cache)?;
        let room = Arc::new(measure_room(
            &self.config,
            &points,
            pose,
            snap.floor_prior,
            states,
            self.half_width,
        ));
        cache.room = Some((key, Arc::clone(&room)));
        Some(room)
    }

    fn points(
        &self,
        cloud: &PointCloud2,
        seq: u64,
        cache: &mut Cache,
    ) -> Option<Arc<Vec<[f32; 3]>>> {
        if let Some((cached, points)) = &cache.points {
            if *cached == seq {
                return Some(Arc::clone(points));
            }
        }
        // No z offset here: the follower measures the room in the frame the
        // map arrives in, and the planner's calibration has already been
        // applied to the profile it stamped.
        match msg::extract_xyz(cloud, 0.0) {
            Ok(points) => {
                let points = Arc::new(points);
                cache.points = Some((seq, Arc::clone(&points)));
                Some(points)
            }
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "could not read local_map; driving on the path's stamps instead",
                );
                None
            }
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
                track = %self.config.track,
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

    fn path_of(states: &[State]) -> Path {
        msg::build_path(states, &[], 0.0, "odom")
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
        let stub = planner::hold_stub((5.0, 5.0, 0.0), "odom", 0.0);
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

    // track dispatch

    #[test]
    fn a_track_names_a_regime_and_nothing_else_is_a_track() {
        assert_eq!(Track::parse("hinted"), Some(Track::Hinted));
        assert_eq!(Track::parse("blind"), Some(Track::Blind));
        // laws are NOT tracks: naming one here is the mistake this refuses
        assert_eq!(Track::parse("seed"), None);
        assert_eq!(Track::parse("hinted-rs"), None);
        assert_eq!(Track::parse(""), None);
        assert!(Track::Hinted.annotate_clearance());
        assert!(!Track::Blind.annotate_clearance());
    }

    #[test]
    fn a_reset_law_answers_like_a_fresh_one() {
        // the contract `laws/hinted.rs` states, and what makes the
        // stop_movement reset a real reset rather than a hope
        let cfg = controller_config().hinted();
        let states: Vec<State> = (0..20).map(|k| [k as f64 * 0.1, 0.0, 0.0]).collect();
        let mut fresh = HintedLaw::new();
        let mut used = HintedLaw::new();
        for k in 0..5 {
            used.step((0.0, 1.0, 2.0), &states, None, &cfg, k as f64 * 0.02);
        }
        used.reset();
        assert_eq!(
            fresh.step((0.1, 0.0, 0.0), &states, None, &cfg, 0.02),
            used.step((0.1, 0.0, 0.0), &states, None, &cfg, 0.02)
        );
    }

    // the no-cloud clearance fallback

    fn controller_config() -> ControllerConfig {
        ControllerConfig {
            lookahead: 0.35,
            max_speed: 0.5,
            max_yaw_rate: 1.4,
            k_pos: 2.0,
            k_yaw: 2.0,
            fan_yaw_per_m: 3.0,
            fan_yaw_done: 0.25,
            min_speed: 0.2,
            speed_clearance: 0.35,
            speed_floor_clearance: 0.05,
            speed_lookahead: 2.0,
            walk_gain: 0.964,
            walk_slip: 0.132,
            walk_slip_ramp: 0.08,
            tangent_preview: 0.15,
            escape_clearance: 0.10,
            escape_preview: 1.00,
            escape_speed: 0.75,
            brake_accel: 0.8,
            brake_margin: 0.15,
        }
    }

    #[test]
    fn the_params_split_lands_every_field_in_the_right_law() {
        let cfg = controller_config();
        assert_eq!(cfg.base().lookahead, 0.35);
        assert_eq!(cfg.hinted().brake_margin, 0.15);
        assert_eq!(cfg.hinted().base.speed_lookahead, 2.0);
        // python spells it walk_slip_ramp, the law spells it slip_ramp
        assert_eq!(cfg.blind().slip_ramp, 0.08);
        assert_eq!(cfg.blind().walk_gain, 0.964);
    }

    #[test]
    fn a_stamped_path_feeds_the_hinted_governor_with_no_cloud() {
        // the OPEN the brief left: the hinted law takes clearance and no
        // stamps, so a follower with no map of its own has to invert the
        // dialect. A tightly-stamped plan must come out as tight room.
        let states: Vec<State> = (0..12).map(|k| [k as f64 * 0.2, 0.0, 0.0]).collect();
        let tight = stamps::encode_precision(&states, &[stamps::FLOOR_CLEARANCE; 12], 0.0);
        let roomy = stamps::encode_precision(&states, &[10.0; 12], 0.0);

        let band = dialect_band();
        let of = |ts: &[f64]| {
            let ceilings =
                stamps::decode_ceilings(ts, &states, &band).expect("a stamped plan decodes");
            stamps::ceilings_to_clearance(&ceilings)
        };
        let (a, b) = (of(&tight), of(&roomy));
        assert!(a[5] < b[5], "tight {} should be under roomy {}", a[5], b[5]);
        assert!((a[5] - stamps::FLOOR_CLEARANCE).abs() < 1e-12);
        assert!((b[5] - stamps::SPEED_CLEARANCE).abs() < 1e-12);

        // and it actually reaches the law: the tight plan is requested slower.
        // The RAW law, not `step` -- a first tick out of a standing start is
        // the rate limiter's answer, not the governor's, on either plan.
        let cfg = controller_config().hinted();
        let slow = dimos_motion2_tc::laws::hinted::update((0.0, 0.0, 0.0), &states, Some(&a), &cfg);
        let fast = dimos_motion2_tc::laws::hinted::update((0.0, 0.0, 0.0), &states, Some(&b), &cfg);
        assert!(
            slow.0.hypot(slow.1) < fast.0.hypot(fast.1),
            "tight {slow:?} vs roomy {fast:?}"
        );
    }

    #[test]
    fn an_unstamped_path_leaves_the_law_ungoverned_rather_than_creeping() {
        // a producer that does not speak the dialect must not be read as a
        // tight corridor; None is the honest answer and the law cruises
        let states: Vec<State> = (0..5).map(|k| [k as f64 * 0.2, 0.0, 0.0]).collect();
        assert!(stamps::decode_ceilings(&[0.0; 5], &states, &dialect_band()).is_none());
    }

    // the room hint's band -- the cases in adapter/test_follower.py

    fn config() -> Config {
        Config {
            track: "hinted".to_string(),
            controller_config: controller_config(),
            control_frequency: 10.0,
            goal_tolerance: 0.2,
            embodiment: "go2".to_string(),
            base_frame: "base_link".to_string(),
            floor_anchor: true,
            lidar_height: 0.45,
            ground_margin_m: 0.16,
            idle_speed: 0.02,
            max_path_age_s: 1.0,
        }
    }

    fn half_width() -> f64 {
        emb::half_width(&emb::by_tag("go2").expect("go2"))
    }

    /// A floor slab at `floor_z` with a post 0.20..0.30 m above it, at x=1.
    ///
    /// Sunk far enough that the post is entirely UNDER the raw 0.05..0.45 band
    /// -- the recording's case, where the map's z origin is base height.
    fn room_with_a_post(floor_z: f32) -> Vec<[f32; 3]> {
        let mut pts: Vec<[f32; 3]> = (0..400)
            .map(|i| {
                let a = i as f32 / 400.0 * std::f32::consts::TAU;
                [2.0 * a.cos(), 2.0 * a.sin(), floor_z]
            })
            .collect();
        for k in 0..5 {
            pts.push([1.0, 0.0, floor_z + 0.20 + 0.025 * k as f32]);
        }
        pts
    }

    fn waypoints() -> Vec<State> {
        vec![[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]]
    }

    /// The room hint the raw band gives, which is what anchoring has to change
    /// -- and what it must degrade to when it cannot anchor.
    fn raw_room(points: &[[f32; 3]]) -> Vec<f64> {
        let xy: Vec<[f64; 2]> = waypoints().iter().map(|s| [s[0], s[1]]).collect();
        clearance::path_clearance(&xy, points, half_width())
    }

    #[test]
    fn the_room_hint_is_measured_on_the_floor_anchored_map() {
        let room = room_with_a_post(-0.28);
        // unanchored, the post sits under the band and reads as infinite room
        // -- the governor would drive at full speed into what the planner
        // routed round
        let hw = half_width();
        let blind = measure_room(&config(), &room, (0.0, 0.0, 0.0), None, &waypoints(), hw);
        assert!(blind.iter().all(|d| d.is_infinite()), "{blind:?}");
        // anchored off the tf prior: the post is where the planner sees it
        let seen = measure_room(
            &config(),
            &room,
            (0.0, 0.0, 0.0),
            Some(-0.24),
            &waypoints(),
            hw,
        );
        assert!((seen[0] - (1.0 - hw)).abs() < 1e-6, "{seen:?}");
        assert!((seen[1] - (0.5 - hw)).abs() < 1e-6, "{seen:?}");
    }

    #[test]
    fn an_unanchored_room_hint_is_the_raw_band() {
        // no tf prior: the follower degrades to the band as it was, exactly as
        // the planner does, rather than anchoring to whatever the low quantile
        // of the cloud happens to be
        let room = room_with_a_post(-0.28);
        let cfg = Config {
            lidar_height: 0.0,
            ..config()
        };
        let got = measure_room(
            &cfg,
            &room,
            (0.0, 0.0, 0.0),
            None,
            &waypoints(),
            half_width(),
        );
        assert_eq!(got, raw_room(&room));
    }

    #[test]
    fn the_anchor_switch_turns_the_room_hint_back_to_the_raw_band() {
        let room = room_with_a_post(-0.28);
        let cfg = Config {
            floor_anchor: false,
            ..config()
        };
        let got = measure_room(
            &cfg,
            &room,
            (0.0, 0.0, 0.0),
            Some(-0.24),
            &waypoints(),
            half_width(),
        );
        assert_eq!(got, raw_room(&room));
    }

    #[test]
    fn a_room_hint_on_a_floor_already_at_zero_is_the_raw_band() {
        // the referee's sim worlds put the plan poses on the ground; anchoring
        // there only drops the ground the body is standing on, which was never
        // in the band, so the hint the judge hands the controller cannot move
        let room = room_with_a_post(0.0);
        let got = measure_room(
            &config(),
            &room,
            (0.0, 0.0, 0.0),
            Some(0.0),
            &waypoints(),
            half_width(),
        );
        assert_eq!(got, raw_room(&room));
    }

    #[test]
    fn a_veto_stub_commands_zero_on_both_tracks() {
        let stub = planner::hold_stub((1.0, 2.0, 0.5), "odom", 0.0);
        let states = msg::path_states(&stub);
        let ts = msg::path_stamps(&stub);
        let cfg = controller_config();
        assert_eq!(
            HintedLaw::new().step((1.0, 2.0, 0.5), &states, None, &cfg.hinted(), 0.02),
            (0.0, 0.0, 0.0)
        );
        assert_eq!(
            blind_update((1.0, 2.0, 0.5), &states, None, Some(&ts), &cfg.blind()),
            (0.0, 0.0, 0.0)
        );
    }
}
