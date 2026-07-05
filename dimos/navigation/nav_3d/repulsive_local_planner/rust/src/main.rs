// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Native repulsive-field local planner: GENUINE high-rate solves.
//!
//! The Python module re-anchored a cached plan at 60 Hz and only re-SOLVED at
//! ~2-4 Hz (a solve was 200-300 ms of numpy); the stability machinery that grew
//! around that latency (adoption gates, blocked debounce, horizon feedback) is
//! deliberately absent here — a Rust wavefront over the same window is
//! sub-millisecond, so every published path IS a fresh solve against the live
//! costmap. Kept from the measured Python semantics: the internal level-aware
//! costmap (higher resolution), temporal commitment (previous-path bias), the
//! carrot tail with reversal trim, the same-goal alternative-route debounce
//! (the global planner flip-flops in 2-8 s phases), and odometry
//! dead-reckoning between samples.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_repulsive_field::costmap::{self, CostmapConfig, LevelTracker};
use dimos_repulsive_field::solver::{self, SolverConfig};
use dimos_module::{error_throttled, native_config, run, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::{Odometry, Path};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tracing::{debug, warn};

#[native_config]
#[derive(Clone)]
pub struct Config {
    pub world_frame: String,
    pub body_frame: String,
    /// Publish local_path in the robot's base frame (the CMU pure-pursuit
    /// follower consumes a vehicle-frame route and never tf-transforms it).
    pub output_base_frame: bool,
    /// Genuine solve rate (Hz).
    pub solve_hz: f32,
    /// Publish decimation (Hz). Solves stay at solve_hz (fresh commitment
    /// chain); publishing every solve is what hurts — the consumer process's
    /// single LCM intake thread fell ~1.5 s behind at 60 Hz (hl61: the
    /// follower acted on paths published mid-rotation ~1.5 s earlier, a
    /// stale-feedback limit cycle of full-circle spins; the recorder in the
    /// same process stopped keeping up entirely at t=247).
    pub publish_hz: f32,
    /// Stop publishing when odometry is older than this (s): the follower's
    /// cmd watchdog then halts the base instead of chasing a ghost pose.
    pub max_odom_age_s: f32,
    /// Stop publishing when the newest terrain slice is older than this (s):
    /// planning against a frozen costmap silently steered the robot off the
    /// map's edge in hl62. Warn at half this age.
    pub max_costmap_age_s: f32,
    /// Publish the internal costmap's lethal cells as a world-frame cloud on
    /// costmap_cloud for the viewer overlay (0 disables).
    pub costmap_cloud_hz: f32,
    /// Same-goal alternative-route debounce (s) — see the Python module.
    pub route_change_persist_s: f32,
    pub route_reroute_threshold_m: f32,

    // Costmap knobs (see costmap.rs; names mirror the Python HeightCostConfig).
    pub resolution: f32,
    pub can_pass_under: f32,
    pub can_climb: f32,
    pub max_safe_fall: f32,
    pub void_depth_lethal: f32,
    pub slice_below: f32,
    pub slice_above: f32,
    pub half_extent: f32,
    pub level_hysteresis: f32,

    // Solver knobs (see solver.rs; names mirror the Python config).
    pub vehicle_width: f32,
    pub safety_margin: f32,
    pub influence_radius: f32,
    pub clearance_weight: f32,
    pub path_weight: f32,
    pub commitment_weight: f32,
    pub carrot_lookahead: f32,
    pub carrot_lookahead_time_s: f32,
    pub carrot_lookahead_max: f32,
    pub carrot_gap_max: f32,
    pub dijkstra_radius: f32,
    pub horizon: f32,
    pub goal_tolerance: f32,
    pub smoothing_iterations: u32,
    pub face_forward_weight: f32,
    pub tail_reversal_trim_deg: f32,
}

impl Config {
    fn costmap(&self) -> CostmapConfig {
        CostmapConfig {
            resolution: self.resolution,
            can_pass_under: self.can_pass_under,
            can_climb: self.can_climb,
            ignore_noise: 0.05,
            max_safe_fall: self.max_safe_fall,
            void_depth_lethal: self.void_depth_lethal,
            slice_below: self.slice_below,
            slice_above: self.slice_above,
            half_extent: self.half_extent,
            level_hysteresis: self.level_hysteresis,
        }
    }
    fn solver(&self) -> SolverConfig {
        SolverConfig {
            vehicle_width: self.vehicle_width,
            safety_margin: self.safety_margin,
            influence_radius: self.influence_radius,
            clearance_weight: self.clearance_weight,
            path_weight: self.path_weight,
            commitment_weight: self.commitment_weight,
            carrot_lookahead: self.carrot_lookahead,
            carrot_lookahead_time_s: self.carrot_lookahead_time_s,
            carrot_lookahead_max: self.carrot_lookahead_max,
            carrot_gap_max: self.carrot_gap_max,
            dijkstra_radius: self.dijkstra_radius,
            horizon: self.horizon,
            goal_tolerance: self.goal_tolerance,
            smoothing_iterations: self.smoothing_iterations as usize,
            face_forward_weight: self.face_forward_weight,
            tail_reversal_trim_deg: self.tail_reversal_trim_deg,
        }
    }
}

type Shared<T> = Arc<Mutex<T>>;

/// Odometry-derived robot state with an EMA world-frame velocity for
/// dead-reckoning between samples (port of _update_velocity_estimate).
#[derive(Clone, Copy, Default)]
struct RobotState {
    x: f32,
    y: f32,
    z: f32,
    yaw: f32,
    vx: f32,
    vy: f32,
    wz: f32,
}

#[derive(Default)]
struct SharedState {
    robot: Option<(RobotState, Instant)>,
    terrain: Option<Vec<[f32; 3]>>,
    /// The committed global route + the pending same-goal alternative.
    route: Vec<(f32, f32)>,
    pending_route: Option<(Vec<(f32, f32)>, Instant)>,
    tail: Vec<(f32, f32)>,
}

#[derive(Module)]
#[module(setup = spawn_worker, teardown = stop_worker)]
struct RepulsiveField {
    #[input(decode = PointCloud2::decode, handler = on_terrain_map)]
    terrain_map: Input<PointCloud2>,

    // jnav spec: the GlobalPlanner output stream (autoconnect wires by name).
    #[input(decode = Path::decode, handler = on_global_path)]
    global_path: Input<Path>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    // Route markers beyond the current leg goal (MapMemManager.route_tail).
    #[input(decode = Path::decode, handler = on_route_tail)]
    route_tail: Input<Path>,

    #[output(encode = Path::encode)]
    local_path: Output<Path>,

    // Lethal cells of the INTERNAL costmap for the viewer (the legacy
    // CostMapper overlay showed a map the planner never used).
    #[output(encode = PointCloud2::encode)]
    costmap_cloud: Output<PointCloud2>,

    #[config]
    config: Config,

    state: Shared<SharedState>,
    worker: Option<tokio::task::JoinHandle<()>>,
}

impl RepulsiveField {
    async fn spawn_worker(&mut self) {
        let worker = Worker {
            state: Arc::clone(&self.state),
            config: self.config.clone(),
            local_path: self.local_path.clone(),
            costmap_cloud: self.costmap_cloud.clone(),
        };
        self.worker = Some(tokio::spawn(worker.run()));
    }

    async fn stop_worker(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
    }

    async fn on_terrain_map(&mut self, msg: PointCloud2) {
        match extract_xyz(&msg) {
            Ok(points) => {
                self.state.lock().expect("state").terrain = Some(points);
            }
            Err(e) => {
                warn!(error = %e, "terrain_map extract failed; dropped");
            }
        }
    }

    async fn on_global_path(&mut self, msg: Path) {
        let new_route: Vec<(f32, f32)> = msg
            .poses
            .iter()
            .map(|p| (p.pose.position.x as f32, p.pose.position.y as f32))
            .collect();
        if new_route.len() < 2 {
            return;
        }
        let mut state = self.state.lock().expect("state");
        if state.route.len() < 2 {
            state.route = new_route;
            state.pending_route = None;
            return;
        }
        // Same-goal alternative-route debounce (port; measured: the global
        // planner flip-flops between near-equal routes in 2-8 s phases and
        // chasing each flip drove the robot back and forth).
        let old_goal = *state.route.last().unwrap();
        let new_goal = *new_route.last().unwrap();
        let goal_moved =
            (new_goal.0 - old_goal.0).hypot(new_goal.1 - old_goal.1) > self.config.resolution;
        let deviation = route_deviation(&state.route, &new_route);
        if !goal_moved && deviation > self.config.route_reroute_threshold_m {
            let now = Instant::now();
            match &state.pending_route {
                Some((pending, since))
                    if route_deviation(pending, &new_route) < 1.0 =>
                {
                    if now.duration_since(*since).as_secs_f32()
                        < self.config.route_change_persist_s
                    {
                        return; // not stable long enough — keep the committed route
                    }
                }
                _ => {
                    state.pending_route = Some((new_route, now));
                    return; // a NEW alternative: start its persistence clock
                }
            }
        }
        state.pending_route = None;
        state.route = new_route;
    }

    async fn on_odometry(&mut self, msg: Odometry) {
        let p = &msg.pose.pose.position;
        let q = &msg.pose.pose.orientation;
        let yaw = yaw_of_quaternion(q.x as f32, q.y as f32, q.z as f32, q.w as f32);
        let now = Instant::now();
        let mut state = self.state.lock().expect("state");
        let mut next = RobotState {
            x: p.x as f32,
            y: p.y as f32,
            z: p.z as f32,
            yaw,
            ..Default::default()
        };
        if let Some((prev, prev_t)) = state.robot {
            let dt = now.duration_since(prev_t).as_secs_f32();
            if dt > 1e-4 && dt <= 1.0 {
                const ALPHA: f32 = 0.35; // EMA: smooth quantization without lagging accel
                let vx = (next.x - prev.x) / dt;
                let vy = (next.y - prev.y) / dt;
                let wz = wrap_angle(next.yaw - prev.yaw) / dt;
                next.vx = prev.vx + ALPHA * (vx - prev.vx);
                next.vy = prev.vy + ALPHA * (vy - prev.vy);
                next.wz = prev.wz + ALPHA * (wz - prev.wz);
            }
        }
        state.robot = Some((next, now));
    }

    async fn on_route_tail(&mut self, msg: Path) {
        let tail: Vec<(f32, f32)> = msg
            .poses
            .iter()
            .map(|p| (p.pose.position.x as f32, p.pose.position.y as f32))
            .collect();
        self.state.lock().expect("state").tail = tail;
    }
}

struct Worker {
    state: Shared<SharedState>,
    config: Config,
    local_path: Output<Path>,
    costmap_cloud: Output<PointCloud2>,
}

impl Worker {
    async fn run(self) {
        let costmap_cfg = self.config.costmap();
        let solver_cfg = self.config.solver();
        let mut level = LevelTracker::default();
        let mut map: Option<costmap::Costmap> = None;
        let mut terrain_at: Option<Instant> = None;
        let mut prev_path: Option<Vec<(f32, f32)>> = None;
        let period = Duration::from_secs_f32(1.0 / self.config.solve_hz.max(1.0));
        let mut ticker = tokio::time::interval(period);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut solve_count: u64 = 0;
        let mut window_start = Instant::now();
        let publish_period = Duration::from_secs_f32(1.0 / self.config.publish_hz.max(1.0));
        let mut last_publish: Option<Instant> = None;
        let cloud_period = (self.config.costmap_cloud_hz > 0.0)
            .then(|| Duration::from_secs_f32(1.0 / self.config.costmap_cloud_hz));
        let mut last_cloud: Option<Instant> = None;

        loop {
            ticker.tick().await;

            // Snapshot inputs (cheap; heavy work happens outside the lock).
            let (robot_opt, terrain, route, tail) = {
                let mut state = self.state.lock().expect("state");
                (
                    state.robot,
                    state.terrain.take(),
                    state.route.clone(),
                    state.tail.clone(),
                )
            };
            let Some((robot, robot_at)) = robot_opt else {
                dimos_module::warn_throttled!(Duration::from_secs(5), "no odometry yet");
                continue;
            };
            let age = Instant::now().duration_since(robot_at).as_secs_f32();
            if age > self.config.max_odom_age_s {
                // Stale odometry: let the follower's watchdog stop the base. If
                // this persists the input pipe itself is broken — say so.
                dimos_module::warn_throttled!(
                    Duration::from_secs(5),
                    odom_age_s = age,
                    "skipping solves: odometry stale"
                );
                continue;
            }
            if route.len() < 2 {
                dimos_module::warn_throttled!(Duration::from_secs(5), "no global route yet");
                continue;
            }

            // Dead-reckon the pose forward (first-order unicycle).
            let pose = (
                robot.x + robot.vx * age,
                robot.y + robot.vy * age,
                wrap_angle(robot.yaw + robot.wz * age),
            );
            let speed = robot.vx.hypot(robot.vy);

            // Rebuild the costmap when a fresh terrain slice arrived (the slice
            // updates at ~2 Hz; the solve runs every tick regardless). Built
            // INLINE — no block_in_place: at 60-120 Hz that demotes/replaces a
            // runtime worker thread every call, and under a CPU-saturated sim
            // the churn starved this process's LCM recv task until the socket
            // buffer overflowed. Fragmented multi-MB terrain messages (41
            // datagrams each, all-or-nothing) died first while small
            // odometry/route messages survived: hl62 froze on a costmap built
            // 20 s earlier, the robot walked off its 8 m edge, and the carrot
            // collapsed to a 0.22 m stub 1.84 m short of wp3, for 13 minutes,
            // silently. A build is a few ms at ~1.2 Hz; a solve is sub-ms.
            if let Some(points) = terrain {
                let reference = level.update(robot.z, costmap_cfg.level_hysteresis);
                map = Some(costmap::build(
                    &points,
                    (pose.0, pose.1, robot.z),
                    reference,
                    &costmap_cfg,
                ));
                terrain_at = Some(Instant::now());
            }
            let Some(map_ref) = map.as_ref() else {
                dimos_module::warn_throttled!(Duration::from_secs(5), "no costmap yet (no terrain received)");
                continue;
            };
            // Terrain-input death must be LOUD and must PARK the robot (via
            // the follower's cmd watchdog), never silently steer on a frozen
            // world (the hl62 failure above).
            let map_age = terrain_at.map(|at| at.elapsed().as_secs_f32()).unwrap_or(0.0);
            if map_age > self.config.max_costmap_age_s {
                dimos_module::error_throttled!(
                    Duration::from_secs(5),
                    costmap_age_s = map_age,
                    "terrain input dead: costmap frozen, halting publishes"
                );
                continue;
            }
            if map_age > self.config.max_costmap_age_s * 0.5 {
                dimos_module::warn_throttled!(
                    Duration::from_secs(5),
                    costmap_age_s = map_age,
                    "terrain input stale: costmap aging"
                );
            }

            let extension = solver::carrot_extension(&route, &tail, &solver_cfg);
            let plan = solver::plan(
                map_ref,
                &route,
                &extension,
                pose,
                speed,
                prev_path.as_deref(),
                &solver_cfg,
            );
            if plan.poses.len() >= 2 {
                prev_path = Some(plan.poses.iter().map(|p| (p.0, p.1)).collect());
            }

            // Viewer overlay: the internal costmap's lethal cells, so the
            // viewer shows the map the planner ACTUALLY plans on.
            if let Some(period) = cloud_period {
                if last_cloud.is_none_or(|at: Instant| at.elapsed() >= period.mul_f32(0.95)) {
                    last_cloud = Some(Instant::now());
                    let msg = build_costmap_cloud(map_ref, robot.z, &self.config.world_frame);
                    if let Err(e) = self.costmap_cloud.publish(&msg).await {
                        error_throttled!(
                            Duration::from_secs(5),
                            error = %e,
                            "costmap_cloud failed to publish",
                        );
                    }
                }
            }

            // Small epsilon so 60 Hz ticks don't alias a 30 Hz target to 20 Hz.
            let due = last_publish
                .is_none_or(|at| at.elapsed() >= publish_period.mul_f32(0.95));
            if due {
                last_publish = Some(Instant::now());
                let msg = self.build_path_msg(&plan.poses, pose);
                if let Err(e) = self.local_path.publish(&msg).await {
                    error_throttled!(
                        Duration::from_secs(1),
                        error = %e,
                        "local_path failed to publish",
                    );
                }
            }

            solve_count += 1;
            if window_start.elapsed() >= Duration::from_secs(10) {
                debug!(
                    solves_per_s = solve_count as f64 / window_start.elapsed().as_secs_f64(),
                    "solver rate"
                );
                solve_count = 0;
                window_start = Instant::now();
            }
        }
    }

    /// World plan -> Path message, optionally rotated into the base frame at
    /// the given pose (port of _publish_plan).
    fn build_path_msg(&self, poses: &[(f32, f32, f32)], robot: (f32, f32, f32)) -> Path {
        let to_base = self.config.output_base_frame;
        let frame = if to_base {
            &self.config.body_frame
        } else {
            &self.config.world_frame
        };
        let (sin_y, cos_y) = robot.2.sin_cos();
        let stamp = now();
        let mut out: Vec<PoseStamped> = Vec::with_capacity(poses.len());
        for &(x, y, pose_yaw) in poses {
            let (px, py, pyaw) = if to_base {
                let (dx, dy) = (x - robot.0, y - robot.1);
                (
                    cos_y * dx + sin_y * dy,
                    -sin_y * dx + cos_y * dy,
                    wrap_angle(pose_yaw - robot.2),
                )
            } else {
                (x, y, pose_yaw)
            };
            let (sz, cz) = (pyaw * 0.5).sin_cos();
            out.push(PoseStamped {
                header: header(frame, stamp.clone()),
                pose: Pose {
                    position: Point {
                        x: px as f64,
                        y: py as f64,
                        z: 0.0,
                    },
                    orientation: Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: sz as f64,
                        w: cz as f64,
                    },
                },
            });
        }
        Path {
            header: header(frame, stamp),
            poses: out,
        }
    }
}

fn route_deviation(a: &[(f32, f32)], b: &[(f32, f32)]) -> f32 {
    // Max pointwise deviation after arc-length resampling to 24 points.
    let ra = resample(a, 24);
    let rb = resample(b, 24);
    ra.iter()
        .zip(rb.iter())
        .map(|(p, q)| (p.0 - q.0).hypot(p.1 - q.1))
        .fold(0.0, f32::max)
}

fn resample(path: &[(f32, f32)], n: usize) -> Vec<(f32, f32)> {
    if path.is_empty() {
        return vec![(0.0, 0.0); n];
    }
    let mut arc = vec![0.0f32];
    for pair in path.windows(2) {
        let d = (pair[1].0 - pair[0].0).hypot(pair[1].1 - pair[0].1);
        arc.push(arc.last().unwrap() + d);
    }
    let total = *arc.last().unwrap();
    if total < 1e-6 {
        return vec![path[0]; n];
    }
    let mut out = Vec::with_capacity(n);
    let mut j = 0;
    for k in 0..n {
        let target = total * k as f32 / (n - 1) as f32;
        // j must stop at the LAST SEGMENT (len-2): float noise in the cumsum
        // can put the final target a hair past the stored total, and walking
        // j to len-1 indexed path[j+1] out of bounds — a panic on the module's
        // MAIN task that killed the whole handler loop at the first route
        // change (hl59: 43 messages then 18 min of silence).
        while j + 2 < arc.len() && arc[j + 1] < target {
            j += 1;
        }
        let span = (arc[j + 1] - arc[j]).max(1e-9);
        let f = (target - arc[j]) / span;
        let a = path[j];
        let b = path[j + 1];
        out.push((a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f));
    }
    out
}

fn yaw_of_quaternion(x: f32, y: f32, z: f32, w: f32) -> f32 {
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn wrap_angle(a: f32) -> f32 {
    a.sin().atan2(a.cos())
}

/// Lethal cells of the internal costmap as a world-frame xyz cloud, matching
/// the legacy CostMapper overlay the viewer already renders (red points at
/// the robot's storey: odom z is the body centre ~0.6 m above the feet; drop
/// to foot level and lift 0.15 m so cells sit just above the treads).
fn build_costmap_cloud(map: &costmap::Costmap, robot_z: f32, frame: &str) -> PointCloud2 {
    let vis_z = robot_z - 0.45;
    let mut data: Vec<u8> = Vec::new();
    let mut n: i32 = 0;
    for row in 0..map.height {
        for col in 0..map.width {
            if map.cost[row * map.width + col] >= costmap::LETHAL_THRESHOLD {
                let (x, y) = map.cell_center(row, col);
                data.extend_from_slice(&x.to_le_bytes());
                data.extend_from_slice(&y.to_le_bytes());
                data.extend_from_slice(&vis_z.to_le_bytes());
                n += 1;
            }
        }
    }
    let field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    PointCloud2 {
        header: header(frame, now()),
        height: 1,
        width: n,
        fields: vec![field("x", 0), field("y", 4), field("z", 8)],
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * n,
        data,
        is_dense: true,
    }
}

fn now() -> Time {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    Time {
        sec: dur.as_secs().min(i32::MAX as u64) as i32,
        nsec: dur.subsec_nanos() as i32,
    }
}

fn header(frame_id: &str, stamp: Time) -> Header {
    Header {
        seq: 0,
        stamp,
        frame_id: frame_id.into(),
    }
}

#[derive(Debug)]
struct ExtractError(String);
impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Pull xyz f32 triples out of a PointCloud2 (x/y/z float32 fields).
fn extract_xyz(msg: &PointCloud2) -> Result<Vec<[f32; 3]>, ExtractError> {
    let mut ox = None;
    let mut oy = None;
    let mut oz = None;
    for f in &msg.fields {
        match f.name.as_str() {
            "x" => ox = Some(f.offset as usize),
            "y" => oy = Some(f.offset as usize),
            "z" => oz = Some(f.offset as usize),
            _ => {}
        }
    }
    let (ox, oy, oz) = match (ox, oy, oz) {
        (Some(a), Some(b), Some(c)) => (a, b, c),
        _ => return Err(ExtractError("missing x/y/z fields".into())),
    };
    let step = msg.point_step as usize;
    if step == 0 {
        return Err(ExtractError("zero point_step".into()));
    }
    let data = &msg.data;
    let count = data.len() / step;
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let base = i * step;
        let read = |off: usize| -> f32 {
            let b = &data[base + off..base + off + 4];
            f32::from_le_bytes([b[0], b[1], b[2], b[3]])
        };
        let p = [read(ox), read(oy), read(oz)];
        if p[0].is_finite() && p[1].is_finite() && p[2].is_finite() {
            out.push(p);
        }
    }
    Ok(out)
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<RepulsiveField, _>(transport).await;
}
