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

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use crate::mapper::{register, Mapper, Pose};
use crate::region_viz::{pack_cell, RegionSweep};
use crate::voxel_ray_tracer::{
    partition_seed, region_of, Cell, ChunkKey, Config, Cylinder, SeedPartition, SeedRegion,
};
use dimos_module::pointcloud::extract_xyz;
use dimos_module::{
    debug_throttled, error_throttled, warn_throttled, Input, Module, Output, Tf, Transform,
};
use lcm_msgs::geometry_msgs::{Point, Pose as PoseMsg, PoseStamped, Quaternion};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tokio::sync::mpsc;
use tokio::sync::mpsc::error::TryRecvError;
use tokio::task::JoinHandle;
use tracing::{info, warn};

/// Messages queued to the worker in arrival order.
enum Job {
    Lidar(PointCloud2, Instant),
    ClearMask(PointCloud2),
    LoadedMap(PointCloud2),
    /// A loaded map placed in the world and split into tiles, or None when
    /// it could not be placed.
    SeedPrepared(Option<SeedPartition>),
}

/// Messages the handlers can queue ahead of the worker before they wait.
const JOB_QUEUE_CAPACITY: usize = 256;

/// How long one worker pass may spend on seed tiles before it returns to the
/// job queue.
const SEED_PASS_BUDGET: Duration = Duration::from_millis(20);

#[derive(Module)]
#[module(name = "ray_tracing", setup = spawn_worker, teardown = stop_worker)]
pub struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    // World-frame points a sensor knows to be empty. Their voxels are deleted
    // outright, reaching space ray tracing cannot clear.
    #[input(decode = PointCloud2::decode, handler = on_voxel_clear_mask)]
    voxel_clear_mask: Input<PointCloud2>,

    // An externally loaded map cloud. Only the first one seeds the map.
    #[input(decode = PointCloud2::decode, handler = on_loaded_map)]
    loaded_map: Input<PointCloud2>,

    #[tf]
    tf: Tf,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map_fine: Output<PointCloud2>,

    // Cylinder bounds of the local map. Position is the center, orientation holds
    // radius, z_min, z_max. Stamped like local_map so consumers pair them.
    #[output(encode = PoseStamped::encode)]
    region_bounds: Output<PoseStamped>,

    // The map for viewers, one region cell per message with the cell packed in
    // the header seq.
    #[output(encode = PointCloud2::encode)]
    map_regions: Output<PointCloud2>,

    // One region of a seeded map as it lands, gated like local_map. seed_bounds
    // carries its cylinder like region_bounds.
    #[output(encode = PointCloud2::encode)]
    seed_map: Output<PointCloud2>,

    #[output(encode = PoseStamped::encode)]
    seed_bounds: Output<PoseStamped>,

    #[config]
    config: Config,

    jobs: Option<mpsc::Sender<Job>>,
    worker: Option<JoinHandle<()>>,
}

/// Whether transforms have already moved past a cloud's stamp, so the one it needs will never
/// arrive: waiting on it would cap a backed-up queue's drain rate at 1/tf_wait_timeout_s.
fn transform_is_past(stamp: f64, latest_tf: Option<f64>, tolerance: f64) -> bool {
    latest_tf.is_some_and(|latest| latest > stamp + tolerance)
}

/// Whether a cloud is more than `max_age_s` older than the newest transform for its frame (0 keeps all).
/// Aged against the transforms rather than the wall clock, so a replay ages the same way.
fn is_stale(stamp: f64, latest_tf: Option<f64>, max_age_s: f64) -> bool {
    max_age_s > 0.0 && latest_tf.is_some_and(|latest| latest - stamp > max_age_s)
}

impl RayTracingVoxelMap {
    async fn spawn_worker(&mut self) {
        let (tx, rx) = mpsc::channel(JOB_QUEUE_CAPACITY);
        let worker = Worker {
            jobs: rx,
            job_sender: tx.downgrade(),
            tf: self.tf.clone(),
            config: self.config.clone(),
            global_map: self.global_map.clone(),
            local_map: self.local_map.clone(),
            local_map_fine: self.local_map_fine.clone(),
            region_bounds: self.region_bounds.clone(),
            map_regions: self.map_regions.clone(),
            seed_map: self.seed_map.clone(),
            seed_bounds: self.seed_bounds.clone(),
        };
        self.jobs = Some(tx);
        self.worker = Some(tokio::spawn(worker.run()));
    }

    async fn stop_worker(&mut self) {
        self.jobs = None;
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        self.enqueue(Job::Lidar(msg, Instant::now())).await;
    }

    async fn on_voxel_clear_mask(&mut self, msg: PointCloud2) {
        self.enqueue(Job::ClearMask(msg)).await;
    }

    async fn on_loaded_map(&mut self, msg: PointCloud2) {
        self.enqueue(Job::LoadedMap(msg)).await;
    }

    async fn enqueue(&self, job: Job) {
        let Some(jobs) = &self.jobs else {
            return;
        };
        if jobs.send(job).await.is_err() {
            error_throttled!(
                Duration::from_secs(1),
                "Ray tracing worker is gone, dropped a message.",
            );
        }
    }
}

/// A seed load in progress, applied a pass budget of tiles at a time and
/// handed on region by region as each one completes.
struct SeedLoad {
    regions: Vec<SeedRegion>,
    next_region: usize,
    next_tile: usize,
    tile_count: usize,
    tiles_done: usize,
    created: usize,
    started: Instant,
    max_tile_ms: f64,
    sum_tile_ms: f64,
    max_live_wait_ms: f64,
}

impl SeedLoad {
    fn new(part: SeedPartition) -> Self {
        Self {
            tile_count: part.tile_count(),
            regions: part.regions,
            next_region: 0,
            next_tile: 0,
            tiles_done: 0,
            created: 0,
            started: Instant::now(),
            max_tile_ms: 0.0,
            sum_tile_ms: 0.0,
            max_live_wait_ms: 0.0,
        }
    }

    fn mean_tile_ms(&self) -> f64 {
        self.sum_tile_ms / self.tiles_done.max(1) as f64
    }

    /// Apply the next tile. The region it completed, if any.
    fn step(&mut self, mapper: &mut Mapper) -> Option<Cylinder> {
        let region = self.regions.get(self.next_region)?;
        let tile = &region.tiles[self.next_tile];
        let tile_start = Instant::now();
        self.created += tokio::task::block_in_place(|| mapper.seed_tile(tile));
        let tile_ms = tile_start.elapsed().as_secs_f64() * 1e3;
        self.tiles_done += 1;
        self.max_tile_ms = self.max_tile_ms.max(tile_ms);
        self.sum_tile_ms += tile_ms;
        self.next_tile += 1;
        if self.next_tile < region.tiles.len() {
            return None;
        }
        self.next_tile = 0;
        self.next_region += 1;
        Some(region.cylinder)
    }

    fn finished(&self) -> bool {
        self.next_region >= self.regions.len()
    }
}

/// Stage of the one loaded map.
enum SeedState {
    Idle,
    Placing,
    Loading(SeedLoad),
    Done,
}

impl SeedState {
    /// Start placing a loaded map, or refuse it. Only Idle takes a cloud:
    /// reseeding would resurrect voxels live rays carved.
    fn take_cloud(&mut self) -> bool {
        if !matches!(self, SeedState::Idle) {
            return false;
        }
        *self = SeedState::Placing;
        true
    }

    /// Placement returned. A map that could not be placed leaves the stage
    /// Idle, so a later cloud can retry.
    fn placed(&mut self, partition: Option<SeedPartition>) {
        *self = match partition {
            Some(part) => SeedState::Loading(SeedLoad::new(part)),
            None => SeedState::Idle,
        };
    }
}

/// Everything the worker mutates across jobs.
struct State {
    mapper: Mapper,
    // Stamp of the last applied clear mask, so a late one cannot erase voxels
    // a newer mask already accounted for.
    last_clear_mask_stamp: f64,
    // Stamp of the last lidar frame folded in, which is what a full map
    // snapshot is current as of.
    last_frame_stamp: Time,
    seed: SeedState,
    viz: RegionSweep,
}

/// Owns the mapper and does every map mutation and publish off the handle
/// loop. Queued jobs go first, then one pass budget of seed tiles.
struct Worker {
    jobs: mpsc::Receiver<Job>,
    // Handed to the seed placement task so its result re-enters the queue.
    // Weak, so the channel closes once the module drops its sender.
    job_sender: mpsc::WeakSender<Job>,
    tf: Tf,
    config: Config,
    global_map: Output<PointCloud2>,
    local_map: Output<PointCloud2>,
    local_map_fine: Output<PointCloud2>,
    region_bounds: Output<PoseStamped>,
    map_regions: Output<PointCloud2>,
    seed_map: Output<PointCloud2>,
    seed_bounds: Output<PoseStamped>,
}

impl Worker {
    async fn run(mut self) {
        let mut state = State {
            mapper: Mapper::new(self.config.clone()),
            last_clear_mask_stamp: 0.0,
            last_frame_stamp: Time::default(),
            seed: SeedState::Idle,
            viz: RegionSweep::default(),
        };
        loop {
            let loading = matches!(state.seed, SeedState::Loading(_));
            let job = if loading {
                match self.jobs.try_recv() {
                    Ok(job) => Some(job),
                    Err(TryRecvError::Empty) => None,
                    Err(TryRecvError::Disconnected) => return,
                }
            } else {
                match self.jobs.recv().await {
                    Some(job) => Some(job),
                    None => return,
                }
            };
            if let Some(job) = job {
                self.handle(&mut state, job).await;
            }
            self.seed_step(&mut state).await;
            if loading {
                tokio::task::yield_now().await;
            }
        }
    }

    async fn handle(&self, state: &mut State, job: Job) {
        match job {
            Job::Lidar(msg, received) => self.ingest_frame(state, msg, received).await,
            Job::ClearMask(msg) => self.apply_clear_mask(state, msg),
            Job::LoadedMap(msg) => self.place_loaded_map(state, msg).await,
            Job::SeedPrepared(partition) => {
                if let Some(part) = &partition {
                    info!(
                        regions = part.regions.len(),
                        tiles = part.tile_count(),
                        voxels = part.voxels,
                        "Premap load started."
                    );
                    let mapper = &mut state.mapper;
                    tokio::task::block_in_place(|| mapper.reserve_chunks(part.tile_count()));
                }
                state.seed.placed(partition);
            }
        }
    }

    /// Fold one lidar frame into the map and publish whatever is due.
    async fn ingest_frame(&self, state: &mut State, msg: PointCloud2, received: Instant) {
        // Register with the transform nearest the cloud stamp, waiting briefly
        // for one still in flight rather than dropping the cloud.
        let stamp = time_secs(&msg.header.stamp);
        let tolerance = self.config.tf_match_tolerance_s;
        let lookup = self
            .tf
            .lookup(&self.config.world_frame, &msg.header.frame_id)
            .at(stamp)
            .tolerance(tolerance);
        let latest = self
            .tf
            .get_latest(&self.config.world_frame, &msg.header.frame_id)
            .map(|latest| latest.ts);
        if is_stale(stamp, latest, self.config.max_cloud_age_s) {
            warn_throttled!(Duration::from_secs(5), cloud_frame = %msg.header.frame_id, "Skipped a cloud older than max_cloud_age_s: the map is behind and catching up.");
            return;
        }
        let found = if transform_is_past(stamp, latest, tolerance) {
            lookup.get()
        } else {
            lookup
                .within(Duration::from_secs_f64(self.config.tf_wait_timeout_s))
                .await
        };
        let Some(tf_pose) = found else {
            warn!(
                stamp,
                world_frame = %self.config.world_frame,
                cloud_frame = %msg.header.frame_id,
                "No transform within tolerance of the cloud stamp, dropped a cloud.",
            );
            return;
        };
        let points = match extract_xyz(&msg) {
            Ok(p) => p.into_iter().map(|[x, y, z]| (x, y, z)).collect::<Vec<_>>(),
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to get lidar points, dropped a cloud.",
                );
                return;
            }
        };
        if points.is_empty() {
            return;
        }

        let mapper = &mut state.mapper;
        let emit_fine = self.config.emit_fine;
        let process_start = Instant::now();
        let (region, global_points, local_points, fine_points) =
            tokio::task::block_in_place(|| {
                mapper.add_frame(points, tf_to_pose(&tf_pose));
                let region = mapper.local_due().then(|| mapper.take_local_bounds());
                let cylinder = region.map(|c| c.bounds());
                let global_points = mapper.global_due().then(|| mapper.global_points());
                let local_points = cylinder.as_ref().map(|cyl| mapper.local_points(cyl));
                let fine_points = emit_fine
                    .then(|| cylinder.as_ref().and_then(|cyl| mapper.fine_points(cyl)))
                    .flatten();
                (region, global_points, local_points, fine_points)
            });
        let process_ms = process_start.elapsed().as_secs_f64() * 1e3;
        let wait_ms = process_start.duration_since(received).as_secs_f64() * 1e3;
        if let SeedState::Loading(load) = &mut state.seed {
            load.max_live_wait_ms = load.max_live_wait_ms.max(wait_ms);
        }

        let out_frame_id = self.config.world_frame.as_str();
        let stamp = msg.header.stamp;
        state.last_frame_stamp = stamp.clone();

        // Bounds pair with local_map by stamp, so publish them on its cadence.
        if let Some(c) = region {
            let bounds_msg = bounds_to_pose(&c, out_frame_id, stamp.clone());
            publish_bounds(&self.region_bounds, &bounds_msg).await;
        }

        if let Some(points) = global_points {
            let global = points_to_cloud(&points, out_frame_id, stamp.clone());
            publish_cloud(&self.global_map, &global).await;
        }
        if let Some(points) = local_points {
            let local = points_to_cloud(&points, out_frame_id, stamp.clone());
            publish_cloud(&self.local_map, &local).await;
        }
        if let Some(points) = fine_points {
            let fine = points_to_cloud(&points, out_frame_id, stamp.clone());
            publish_cloud(&self.local_map_fine, &fine).await;
        }

        if state.mapper.viz_due() {
            self.publish_map_regions(state, &stamp).await;
        }
        debug_throttled!(
            Duration::from_secs(5),
            process_ms,
            wait_ms,
            "lidar frame processed"
        );
    }

    /// Publish the map region cells due this viz tick, the cell in each
    /// header seq.
    async fn publish_map_regions(&self, state: &mut State, stamp: &Time) {
        let due = tokio::task::block_in_place(|| {
            map_regions_due(
                &mut state.mapper,
                &mut state.viz,
                self.config.voxel_size,
                self.config.region_m,
                self.config.viz_sweep_regions as usize,
            )
        });
        for (cell, points) in due {
            let mut cloud = points_to_cloud(&points, &self.config.world_frame, stamp.clone());
            cloud.header.seq = pack_cell(cell);
            publish_cloud(&self.map_regions, &cloud).await;
        }
    }

    /// Delete the voxels covering a cloud of world-frame points a sensor knows
    /// to be empty.
    ///
    /// Ray tracing only clears what a ray passes through, so a sensor that
    /// occludes itself - a wrist camera staring past its own arm - can never
    /// clear the volume its arm hides. It deposits voxels of itself there and
    /// walls itself in. A publisher that knows those points are free says so
    /// here.
    fn apply_clear_mask(&self, state: &mut State, msg: PointCloud2) {
        let stamp = time_secs(&msg.header.stamp);
        if stamp < state.last_clear_mask_stamp {
            warn_throttled!(
                Duration::from_secs(1),
                stamp,
                last_stamp = state.last_clear_mask_stamp,
                "Out-of-order voxel clear mask dropped",
            );
            return;
        }
        // The keys are metric positions, so the cloud must already be in the
        // world frame. Registering it would need a pose this node has no reason
        // to trust for a mask.
        if msg.header.frame_id != self.config.world_frame {
            warn_throttled!(
                Duration::from_secs(1),
                frame = %msg.header.frame_id,
                expected = %self.config.world_frame,
                "Voxel clear mask is not in the world frame, dropped a mask.",
            );
            return;
        }
        let points = match extract_xyz(&msg) {
            Ok(p) => p.into_iter().map(|[x, y, z]| (x, y, z)).collect::<Vec<_>>(),
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to get clear mask points, dropped a mask.",
                );
                return;
            }
        };
        state.last_clear_mask_stamp = stamp;
        if points.is_empty() {
            return;
        }
        let mapper = &mut state.mapper;
        tokio::task::block_in_place(|| mapper.clear_metric(points));
    }

    /// Place the first loaded map on its own task and tile it off the worker.
    async fn place_loaded_map(&self, state: &mut State, msg: PointCloud2) {
        if !state.seed.take_cloud() {
            return;
        }
        let tf = self.tf.clone();
        let sender = self.job_sender.clone();
        let world_frame = self.config.world_frame.clone();
        let voxel_size = self.config.voxel_size;
        let region_m = self.config.region_m;
        let origin = state.mapper.last_origin();
        tokio::spawn(async move {
            let placed = tf
                .lookup(&world_frame, &msg.header.frame_id)
                .within(LOADED_MAP_TF_WAIT_TIMEOUT)
                .await;
            let partition = match placed {
                Some(t) => {
                    let pose = tf_to_pose(&t);
                    tokio::task::spawn_blocking(move || {
                        prepare_seed(&msg, pose, voxel_size, origin, region_m)
                    })
                    .await
                    .unwrap_or(None)
                }
                None => {
                    warn!(
                        world_frame = %world_frame,
                        cloud_frame = %msg.header.frame_id,
                        "No transform for the loaded map, dropped a cloud.",
                    );
                    None
                }
            };
            if let Some(sender) = sender.upgrade() {
                let _ = sender.send(Job::SeedPrepared(partition)).await;
            }
        });
    }

    /// Apply seed tiles for one pass budget, handing each completed region
    /// on as it lands.
    async fn seed_step(&self, state: &mut State) {
        let SeedState::Loading(load) = &mut state.seed else {
            return;
        };
        let pass_start = Instant::now();
        while !load.finished() {
            let completed = load.step(&mut state.mapper);
            debug_throttled!(
                Duration::from_millis(500),
                tiles_done = load.tiles_done,
                tiles = load.tile_count,
                regions_done = load.next_region,
                regions = load.regions.len(),
                max_tile_ms = load.max_tile_ms,
                mean_tile_ms = load.mean_tile_ms(),
                max_live_wait_ms = load.max_live_wait_ms,
                "Premap load in progress."
            );
            if let Some(cylinder) = completed {
                let seq = load.next_region as i32;
                self.publish_seed_region(&state.mapper, &cylinder, seq, &state.last_frame_stamp)
                    .await;
            }
            if pass_start.elapsed() >= SEED_PASS_BUDGET {
                break;
            }
        }
        let SeedState::Loading(load) = &state.seed else {
            return;
        };
        if !load.finished() {
            return;
        }
        info!(
            num_created = load.created,
            regions = load.regions.len(),
            tiles = load.tile_count,
            load_s = load.started.elapsed().as_secs_f64(),
            max_tile_ms = load.max_tile_ms,
            mean_tile_ms = load.mean_tile_ms(),
            max_live_wait_ms = load.max_live_wait_ms,
            "Premap load finished."
        );
        state.seed = SeedState::Done;
    }

    /// Publish one seeded region as the map now holds it. Cloud and bounds
    /// carry the region number in their header seq, which consumers pair on.
    async fn publish_seed_region(
        &self,
        mapper: &Mapper,
        cylinder: &Cylinder,
        seq: i32,
        stamp: &Time,
    ) {
        let points = tokio::task::block_in_place(|| mapper.local_points(&cylinder.bounds()));
        let (bounds_msg, cloud) =
            seed_region_messages(cylinder, &points, seq, &self.config.world_frame, stamp);
        publish_bounds(&self.seed_bounds, &bounds_msg).await;
        publish_cloud(&self.seed_map, &cloud).await;
    }
}

/// One seeded region as its bounds and cloud messages, both stamped alike
/// with the region number in their header seq.
fn seed_region_messages(
    cylinder: &Cylinder,
    points: &[f32],
    seq: i32,
    frame_id: &str,
    stamp: &Time,
) -> (PoseStamped, PointCloud2) {
    let mut bounds = bounds_to_pose(cylinder, frame_id, stamp.clone());
    bounds.header.seq = seq;
    let mut cloud = points_to_cloud(points, frame_id, stamp.clone());
    cloud.header.seq = seq;
    (bounds, cloud)
}

/// The regions due this viz tick with their points: those whose chunks
/// changed, those that emptied, and `sweep` more from the sweep cursor.
fn map_regions_due(
    mapper: &mut Mapper,
    viz: &mut RegionSweep,
    voxel_size: f32,
    region_m: f32,
    sweep: usize,
) -> Vec<(Cell, Vec<f32>)> {
    let changed: Vec<Cell> = mapper
        .take_changed_chunks()
        .into_iter()
        .map(|chunk| region_of(chunk, voxel_size, region_m))
        .collect();
    let mut present: BTreeMap<Cell, Vec<ChunkKey>> = BTreeMap::new();
    for chunk in mapper.healthy_chunk_keys() {
        present
            .entry(region_of(chunk, voxel_size, region_m))
            .or_default()
            .push(chunk);
    }
    let mapper = &*mapper;
    viz.tick(changed, &present, sweep)
        .into_iter()
        .map(|cell| {
            let points = present
                .get(&cell)
                .map_or_else(Vec::new, |chunks| mapper.chunk_points(chunks));
            (cell, points)
        })
        .collect()
}

/// Register a loaded cloud into the world by `pose` and split it into seed
/// tiles, nearest `origin` first. None when the cloud is unusable.
fn prepare_seed(
    msg: &PointCloud2,
    pose: Pose,
    voxel_size: f32,
    origin: (f32, f32, f32),
    region_m: f32,
) -> Option<SeedPartition> {
    let mut points = match extract_xyz(msg) {
        Ok(p) => p.into_iter().map(|[x, y, z]| (x, y, z)).collect::<Vec<_>>(),
        Err(e) => {
            warn!(error = %e, "Failed to get loaded map points, dropped a cloud.");
            return None;
        }
    };
    if points.is_empty() {
        return None;
    }
    register(&mut points, pose);
    Some(partition_seed(&points, voxel_size, origin, region_m))
}

/// How long a loaded map waits for the transform that places it.
const LOADED_MAP_TF_WAIT_TIMEOUT: Duration = Duration::from_secs(2);

fn time_secs(t: &Time) -> f64 {
    t.sec as f64 + t.nsec as f64 * 1e-9
}

/// A cylinder as the bounds message: position is the center, orientation
/// holds radius, z_min, z_max.
fn bounds_to_pose(c: &Cylinder, frame_id: &str, stamp: Time) -> PoseStamped {
    PoseStamped {
        header: Header {
            seq: 0,
            stamp,
            frame_id: frame_id.to_string(),
        },
        pose: PoseMsg {
            position: Point {
                x: c.cx as f64,
                y: c.cy as f64,
                z: 0.0,
            },
            orientation: Quaternion {
                x: c.radius as f64,
                y: c.z_min as f64,
                z: c.z_max as f64,
                w: 0.0,
            },
        },
    }
}

async fn publish_bounds(out: &Output<PoseStamped>, msg: &PoseStamped) {
    if let Err(e) = out.publish(msg).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "Bounds failed to publish",
        );
    }
}

/// The f32 pose of a transform, for registering clouds.
fn tf_to_pose(t: &Transform) -> Pose {
    let translation = t.translation().cast::<f32>();
    let rotation = t.rotation().cast::<f32>();
    Pose {
        position: (translation.x, translation.y, translation.z),
        orientation: (
            rotation.coords.x,
            rotation.coords.y,
            rotation.coords.z,
            rotation.coords.w,
        ),
    }
}

fn write_point(data: &mut Vec<u8>, n: &mut i32, x: f32, y: f32, z: f32) {
    data.extend_from_slice(&x.to_le_bytes());
    data.extend_from_slice(&y.to_le_bytes());
    data.extend_from_slice(&z.to_le_bytes());
    data.extend_from_slice(&0.0_f32.to_le_bytes());
    *n += 1;
}

fn make_cloud(data: Vec<u8>, n: i32, frame_id: &str, stamp: Time) -> PointCloud2 {
    let make_field = |name: &str, off: i32| PointField {
        name: name.into(),
        offset: off,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    PointCloud2 {
        header: Header {
            seq: 0,
            stamp,
            frame_id: frame_id.into(),
        },
        height: 1,
        width: n,
        fields: vec![
            make_field("x", 0),
            make_field("y", 4),
            make_field("z", 8),
            make_field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n,
        data,
        is_dense: true,
    }
}

/// Pack flat (x, y, z) triples into an LCM cloud message.
fn points_to_cloud(points: &[f32], frame_id: &str, stamp: Time) -> PointCloud2 {
    let mut data = Vec::with_capacity((points.len() / 3) * 16);
    let mut n: i32 = 0;
    for p in points.as_chunks::<3>().0 {
        write_point(&mut data, &mut n, p[0], p[1], p[2]);
    }
    make_cloud(data, n, frame_id, stamp)
}

async fn publish_cloud(out: &Output<PointCloud2>, cloud: &PointCloud2) {
    if let Err(e) = out.publish(cloud).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "Voxel map failed to publish",
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::voxel_ray_tracer::{
        emit_points, metric_voxel_keys, update_map, LocalBounds, VoxelKey, VoxelMap, CHUNK_EDGE,
    };
    use ahash::AHashSet;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    fn test_config() -> Config {
        Config {
            voxel_size: 1.0,
            fine_divisor: 0,
            emit_fine: false,
            max_range: 1000.0,
            ray_subsample: 1,
            shadow_depth: 0.0,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            support_min: 0,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
            world_frame: "world".to_string(),
            tf_match_tolerance_s: 0.1,
            max_cloud_age_s: 0.0,
            tf_wait_timeout_s: 0.05,
            worker_threads: 4,
            region_m: 4.0,
            viz_emit_every: 0,
            viz_sweep_regions: 0,
        }
    }

    /// Build a map whose listed voxels are healthy, through the public API.
    fn map_with_healthy(keys: &[VoxelKey]) -> VoxelMap {
        let mut map = VoxelMap::default();
        let pts: Vec<(f32, f32, f32)> = keys
            .iter()
            .map(|&(x, y, z)| (x as f32 + 0.5, y as f32 + 0.5, z as f32 + 0.5))
            .collect();
        update_map(&mut map, (0.25, 0.25, 0.25), &pts, &test_config());
        map
    }

    fn cloud_points(c: &PointCloud2) -> AHashSet<(u32, u32, u32)> {
        let mut out = AHashSet::new();
        let step = c.point_step as usize;
        for i in 0..c.width as usize {
            let base = i * step;
            let x = f32::from_le_bytes(c.data[base..base + 4].try_into().unwrap());
            let y = f32::from_le_bytes(c.data[base + 4..base + 8].try_into().unwrap());
            let z = f32::from_le_bytes(c.data[base + 8..base + 12].try_into().unwrap());
            out.insert((x.to_bits(), y.to_bits(), z.to_bits()));
        }
        out
    }

    fn voxel_center(kx: i32, ky: i32, kz: i32) -> (u32, u32, u32) {
        (
            (kx as f32 + 0.5).to_bits(),
            (ky as f32 + 0.5).to_bits(),
            (kz as f32 + 0.5).to_bits(),
        )
    }

    /// A prepared seed lands a map-frame cloud in the world through the tf
    /// pose, tiled for the mapper.
    #[test]
    fn prepared_seed_lands_loaded_map_in_world_frame() {
        // Yaw 90 deg at (10, 0, 0): map +x becomes world +y.
        let iso = Isometry3::from_parts(
            Translation3::new(10.0, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        );
        let t = Transform::new("odom", "map", 0.0, iso);
        let cloud = points_to_cloud(&[3.5, 0.0, 0.5], "map", Time::default());

        let part = prepare_seed(&cloud, tf_to_pose(&t), 1.0, (0.0, 0.0, 0.0), 4.0)
            .expect("loaded map must decode");
        assert_eq!(part.voxels, 1);
        assert_eq!(part.regions.len(), 1);
        let mut mapper = Mapper::new(test_config());
        let created: usize = part.tiles().map(|tile| mapper.seed_tile(tile)).sum();
        assert_eq!(created, 1);

        let region = mapper.local_points(&part.regions[0].cylinder.bounds());
        let seeded = points_to_cloud(&region, "odom", Time::default());
        assert!(cloud_points(&seeded).contains(&voxel_center(10, 3, 0)));
    }

    /// Two chunks stacked in z share a region cell, a third far off has its own.
    fn two_region_seed() -> SeedPartition {
        let points = [(1.5, 1.5, 0.5), (1.5, 1.5, 17.5), (40.5, 40.5, 0.5)];
        let part = partition_seed(&points, 1.0, (0.0, 0.0, 0.0), 4.0);
        let tiles: Vec<usize> = part.regions.iter().map(|r| r.tiles.len()).collect();
        assert_eq!(tiles, vec![2, 1]);
        part
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn seed_load_hands_each_region_on_after_its_last_tile() {
        let part = two_region_seed();
        let centers: Vec<(f32, f32)> = part
            .regions
            .iter()
            .map(|r| (r.cylinder.cx, r.cylinder.cy))
            .collect();
        let mut load = SeedLoad::new(part);
        let mut mapper = Mapper::new(test_config());

        assert!(load.step(&mut mapper).is_none());
        assert!(!load.finished());
        let first = load.step(&mut mapper).map(|c| (c.cx, c.cy));
        assert_eq!(first, Some(centers[0]));
        assert!(!load.finished());
        let second = load.step(&mut mapper).map(|c| (c.cx, c.cy));
        assert_eq!(second, Some(centers[1]));
        assert!(load.finished());
        assert_eq!((load.tiles_done, load.tile_count, load.created), (3, 3, 3));
    }

    #[test]
    fn only_an_idle_seed_takes_a_loaded_map() {
        let mut seed = SeedState::Idle;
        assert!(seed.take_cloud());
        assert!(matches!(seed, SeedState::Placing));

        // A second cloud is refused while the first is placing or loading.
        assert!(!seed.take_cloud());
        assert!(matches!(seed, SeedState::Placing));
        seed.placed(Some(two_region_seed()));
        assert!(!seed.take_cloud());
        assert!(matches!(seed, SeedState::Loading(_)));

        let mut done = SeedState::Done;
        assert!(!done.take_cloud());
        assert!(matches!(done, SeedState::Done));
    }

    #[test]
    fn a_placed_seed_loads_and_a_failed_placement_returns_to_idle() {
        let mut seed = SeedState::Placing;
        seed.placed(Some(two_region_seed()));
        let SeedState::Loading(load) = &seed else {
            panic!("a placed seed must be loading");
        };
        assert_eq!((load.regions.len(), load.tile_count), (2, 3));

        let mut seed = SeedState::Placing;
        seed.placed(None);
        assert!(matches!(seed, SeedState::Idle));
        assert!(seed.take_cloud(), "a later cloud retries");
    }

    #[test]
    fn seed_region_messages_pair_on_seq_and_stamp() {
        let part = two_region_seed();
        let stamp = Time { sec: 7, nsec: 9 };
        for (index, region) in part.regions.iter().enumerate() {
            let seq = index as i32 + 1;
            let (bounds, cloud) =
                seed_region_messages(&region.cylinder, &[1.5, 1.5, 0.5], seq, "odom", &stamp);
            assert_eq!((bounds.header.seq, cloud.header.seq), (seq, seq));
            assert_eq!(bounds.header.stamp, stamp);
            assert_eq!(cloud.header.stamp, stamp);
            assert_eq!(bounds.pose.position.x, region.cylinder.cx as f64);
            assert_eq!(cloud.width, 1);
        }
    }

    /// A seeded region is due once with its points, and once more empty after
    /// it is cleared.
    #[test]
    fn map_regions_are_due_on_change_and_once_more_when_emptied() {
        let mut mapper = Mapper::new(test_config());
        let part = partition_seed(
            &[(1.5, 1.5, 0.5), (40.5, 40.5, 0.5)],
            1.0,
            (0.0, 0.0, 0.0),
            4.0,
        );
        for tile in part.tiles() {
            mapper.seed_tile(tile);
        }
        let mut viz = RegionSweep::default();
        let sizes = |due: Vec<(Cell, Vec<f32>)>| -> Vec<(Cell, usize)> {
            due.into_iter().map(|(c, p)| (c, p.len() / 3)).collect()
        };

        // The 4 m grid widens to the chunk edge, so a point's cell is its chunk.
        let far = (40 / CHUNK_EDGE, 40 / CHUNK_EDGE);
        let due = map_regions_due(&mut mapper, &mut viz, 1.0, 4.0, 0);
        assert_eq!(sizes(due), vec![((0, 0), 1), (far, 1)]);
        assert!(map_regions_due(&mut mapper, &mut viz, 1.0, 4.0, 0).is_empty());

        mapper.clear_metric([(1.5, 1.5, 0.5)]);
        let due = map_regions_due(&mut mapper, &mut viz, 1.0, 4.0, 0);
        assert_eq!(sizes(due), vec![((0, 0), 0)]);
        assert!(map_regions_due(&mut mapper, &mut viz, 1.0, 4.0, 0).is_empty());
    }

    #[test]
    fn a_cloud_waits_for_its_transform_only_while_it_could_still_arrive() {
        assert!(!transform_is_past(100.0, None, 0.1));
        assert!(!transform_is_past(100.0, Some(100.05), 0.1));
        assert!(transform_is_past(100.0, Some(100.5), 0.1));
    }

    #[test]
    fn a_cloud_is_stale_by_the_newest_transform_not_the_wall_clock() {
        assert!(!is_stale(100.0, None, 1.0));
        assert!(!is_stale(100.0, Some(100.5), 1.0));
        assert!(is_stale(100.0, Some(101.5), 1.0));
        // 0 keeps every cloud, however old.
        assert!(!is_stale(100.0, Some(500.0), 0.0));
    }

    /// The clear-mask handler names voxels by decoding a cloud and quantizing
    /// it. Both halves have to agree with how returns were quantized on the way
    /// in, or a mask silently clears nothing.
    #[test]
    fn clear_mask_cloud_round_trips_to_the_voxels_it_covers() {
        let map = map_with_healthy(&[(3, -2, 1)]);
        let occupied: Vec<VoxelKey> = map.voxels.keys().collect();
        assert_eq!(occupied, vec![(3, -2, 1)]);

        // A mask cloud naming that voxel's center, encoded and decoded exactly
        // as the port would.
        let cloud = points_to_cloud(&[3.5, -1.5, 1.5], "world", Time::default());
        let Ok(points) = extract_xyz(&cloud) else {
            panic!("clear mask cloud must decode");
        };
        let points: Vec<(f32, f32, f32)> = points.into_iter().map(|[x, y, z]| (x, y, z)).collect();
        let keys: Vec<VoxelKey> = metric_voxel_keys(points, 1.0).collect();

        assert_eq!(keys, occupied);
    }

    #[test]
    fn local_map_includes_voxel_inside_cylinder() {
        let map = map_with_healthy(&[(0, 0, 0)]);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), &live),
            "world",
            Time::default(),
        );
        assert!(cloud_points(&global).contains(&voxel_center(0, 0, 0)));
        assert!(cloud_points(&local).contains(&voxel_center(0, 0, 0)));
    }

    #[test]
    fn local_map_excludes_voxel_outside_radius() {
        let map = map_with_healthy(&[(5, 0, 0)]);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: -10.0,
            z_max: 10.0,
        };
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), &live),
            "world",
            Time::default(),
        );
        assert!(cloud_points(&global).contains(&voxel_center(5, 0, 0)));
        assert!(!cloud_points(&local).contains(&voxel_center(5, 0, 0)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn local_map_excludes_voxel_outside_z_range() {
        let map = map_with_healthy(&[(0, 0, 5)]);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 100.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), &live),
            "world",
            Time::default(),
        );
        assert!(cloud_points(&global).contains(&voxel_center(0, 0, 5)));
        assert!(!cloud_points(&local).contains(&voxel_center(0, 0, 5)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn live_voxels_follow_the_cylinder_in_local_map() {
        let map = VoxelMap::default();
        let mut live: AHashSet<VoxelKey> = AHashSet::new();
        live.insert((1, 0, 0));
        live.insert((10, 10, 10));
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), &live),
            "world",
            Time::default(),
        );
        assert!(cloud_points(&global).contains(&voxel_center(1, 0, 0)));
        assert!(cloud_points(&global).contains(&voxel_center(10, 10, 10)));
        assert!(cloud_points(&local).contains(&voxel_center(1, 0, 0)));
        assert!(!cloud_points(&local).contains(&voxel_center(10, 10, 10)));
    }

    #[test]
    fn local_map_applies_support_min() {
        // The live local cloud must honor support_min, so an isolated healthy
        // voxel is dropped while a dense patch survives. Live voxels bypass it.
        let mut keys: Vec<VoxelKey> = Vec::new();
        for x in 0..3 {
            for y in 0..3 {
                keys.push((x, y, 0));
            }
        }
        keys.push((20, 0, 0));
        let mut map = map_with_healthy(&keys);
        let mut live: AHashSet<VoxelKey> = AHashSet::new();
        live.insert((25, 0, 0));
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 1e6,
            z_min: -10.0,
            z_max: 10.0,
        };
        map.set_support_min(3);
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), &live),
            "world",
            Time::default(),
        );
        let pts = cloud_points(&local);
        assert!(pts.contains(&voxel_center(1, 1, 0)), "dense patch kept");
        assert!(
            !pts.contains(&voxel_center(20, 0, 0)),
            "isolated healthy voxel dropped by support_min"
        );
        assert!(
            pts.contains(&voxel_center(25, 0, 0)),
            "live voxel bypasses support_min"
        );
    }
}
