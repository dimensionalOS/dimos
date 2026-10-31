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

use std::sync::Arc;

use ahash::AHashSet;
use dimos_module::worker_pool;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::voxel_ray_tracer::{
    batch_local_bounds, chunk_points, emit_points, emit_points_fine, emit_points_ungated,
    global_normal_fits, global_normals, seed_points, seed_tile, update_map, ChunkKey, Config,
    Cylinder, FrameHits, LocalBounds, VoxelMap,
};

pub type Point = (f32, f32, f32);

/// Floats per cell `Mapper::surfaces` returns.
pub const SURFACE_CELL_FLOATS: usize = 5;

/// Where the last GPU frame's time went, for logs.
#[derive(Clone, Copy, Debug, Default)]
pub struct GpuFrame {
    /// Encoding and submitting the GPU passes.
    pub integrate_ms: f64,
    /// Waiting for them and reading the report back, then mirroring its
    /// voxels into the CPU map.
    pub report_ms: f64,
    pub mirror_ms: f64,
    pub reported: usize,
}

/// What one `Mapper::surfaces` call recomputed, for logs.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceUpdate {
    /// Changed columns inside the region that it recomputed around.
    pub dirty_columns: usize,
    /// Bricks of the window it recomputed, and of the window around the
    /// whole region it would otherwise have.
    pub bricks: u64,
    pub region_bricks: u64,
    /// Columns it rewrote and the cells it sent for them.
    pub columns: u64,
    pub cells: usize,
    /// Wall time of the extraction (until its plane split is read back) and
    /// of the export and its readback.
    pub extract_ms: f64,
    pub export_ms: f64,
}

/// The MLS planner's surface parameters, which it sends so the GPU extracts
/// the surface it would.
#[derive(Clone, Debug, PartialEq, serde::Deserialize)]
pub struct SurfaceRequest {
    pub robot_height_m: f32,
    pub closing_radius_m: f32,
    pub step_threshold_m: f32,
    pub max_slope_deg: f32,
    /// Read back wall distances too (a second full-window read); without,
    /// they go out infinite.
    #[serde(default)]
    pub wall_distance: bool,
    /// The farthest wall distance the planner uses (its clearance plus wall
    /// buffer): wall distances go out capped there, and a change moves them
    /// at most this far from it. 0 leaves them uncapped.
    #[serde(default)]
    pub wall_reach_m: f32,
    /// Surface more than this far above the sensor is not the planner's.
    #[serde(default = "unbounded")]
    pub max_overhead_m: f32,
}

fn unbounded() -> f32 {
    f32::INFINITY
}

/// Sensor pose in the world frame: position plus (x, y, z, w) orientation.
#[derive(Clone, Copy)]
pub struct Pose {
    pub position: Point,
    pub orientation: (f32, f32, f32, f32),
}

/// The per-frame mapping loop shared by the native module and the Python
/// binding. Callers own transport only.
pub struct Mapper {
    config: Config,
    // The mapper owns its worker pool, so its thread setting cannot collide
    // with other components sharing the process.
    pool: Arc<rayon::ThreadPool>,
    map: VoxelMap,
    live: FrameHits,
    batch_points: Vec<Point>,
    batch_origins: Vec<Point>,
    last_registered: Vec<Point>,
    last_origin: Point,
    frame_count: u32,
    gpu: Option<gpu::GpuMap>,
}

impl Mapper {
    /// The config must already be validated at the caller's boundary.
    pub fn new(config: Config) -> Self {
        let gpu = config.gpu.then(|| gpu::GpuMap::new(&config)).flatten();
        Self {
            gpu,
            pool: worker_pool(config.worker_threads),
            map: VoxelMap::with_support_min(config.support_min),
            config,
            live: FrameHits::default(),
            batch_points: Vec::new(),
            batch_origins: Vec::new(),
            last_registered: Vec::new(),
            last_origin: (0.0, 0.0, 0.0),
            frame_count: 0,
        }
    }

    /// Whether the map updates on the GPU.
    pub fn on_gpu(&self) -> bool {
        self.gpu.is_some()
    }

    /// The GPU's MLS surface where it changed since the last call, around
    /// the changed columns inside `bounds` and between its z_min and z_max,
    /// SURFACE_CELL_FLOATS per row: x, y, z voxel center, plane height, wall
    /// distance. Every rewritten column has a row; one with a NaN z marks a
    /// column left with no surface cell in that z range.
    /// Empty when nothing changed, None on the CPU.
    pub fn surfaces(&mut self, bounds: &Cylinder, request: &SurfaceRequest) -> Option<Vec<f32>> {
        let sensor_z = self.last_origin.2;
        self.gpu.as_mut()?.surfaces(bounds, request, sensor_z)
    }

    /// Time each GPU pass from now on, for profiling; false on the CPU or
    /// without device timestamps.
    pub fn time_gpu_passes(&mut self) -> bool {
        self.gpu.as_mut().is_some_and(|g| g.time_passes())
    }

    /// Each timed GPU pass since the last call, label and milliseconds.
    pub fn take_gpu_pass_times(&mut self) -> Vec<(&'static str, f64)> {
        self.gpu
            .as_mut()
            .map(|g| g.take_pass_times())
            .unwrap_or_default()
    }

    /// Mark every column of `bounds` changed, so the next `surfaces` call
    /// rewrites them all, as a whole-map reference does.
    pub fn mark_surfaces_dirty(&mut self, bounds: &Cylinder) {
        if let Some(gpu) = &mut self.gpu {
            gpu.mark_dirty(bounds);
        }
    }

    /// Where the last frame's time went on the GPU map.
    pub fn gpu_frame(&self) -> GpuFrame {
        self.gpu.as_ref().map(|g| g.frame).unwrap_or_default()
    }

    /// What the last `surfaces` call recomputed.
    pub fn surface_update(&self) -> SurfaceUpdate {
        self.gpu
            .as_ref()
            .map(|g| g.surface_update())
            .unwrap_or_default()
    }

    pub fn config(&self) -> &Config {
        &self.config
    }

    pub fn map(&self) -> &VoxelMap {
        &self.map
    }

    /// Register a sensor-frame cloud into the world by `pose` and fold it into
    /// the map.
    pub fn add_frame(&mut self, mut sensor_points: Vec<Point>, pose: Pose) {
        register(&mut sensor_points, pose);
        self.ingest(sensor_points, pose.position);
    }

    /// Fold an already world-frame cloud into the map, raycasting from `origin`.
    pub fn add_frame_world(&mut self, world_points: Vec<Point>, origin: Point) {
        self.ingest(world_points, origin);
    }

    /// Bulk-seed a world-frame map cloud with no ray tracing, creating only
    /// absent voxels. Returns how many voxels were created.
    pub fn seed_points(&mut self, points: &[Point]) -> usize {
        if let Some(gpu) = &mut self.gpu {
            gpu.seed(points);
        }
        let pool = Arc::clone(&self.pool);
        pool.install(|| seed_points(&mut self.map, points, &self.config))
    }

    /// Make room for `additional` chunks ahead of a tiled seed load.
    pub fn reserve_chunks(&mut self, additional: usize) {
        self.map.reserve_chunks(additional);
    }

    /// Seed one tile of a partitioned world-frame cloud. Returns how many
    /// voxels were created.
    pub fn seed_tile(&mut self, tile: &[Point]) -> usize {
        if let Some(gpu) = &mut self.gpu {
            gpu.seed(tile);
        }
        let pool = Arc::clone(&self.pool);
        pool.install(|| seed_tile(&mut self.map, tile, &self.config))
            .len()
    }

    /// The last frame's sensor origin in the world frame.
    pub fn last_origin(&self) -> Point {
        self.last_origin
    }

    fn ingest(&mut self, points: Vec<Point>, origin: Point) {
        let pool = Arc::clone(&self.pool);
        self.live = match &mut self.gpu {
            Some(gpu) => gpu.update(&mut self.map, origin, &points, &self.config),
            None => pool.install(|| update_map(&mut self.map, origin, &points, &self.config)),
        };

        // The batch only feeds the local region bounds, so skip it when the
        // local cadence is disabled.
        if self.config.emit_every > 0 {
            self.batch_points.extend_from_slice(&points);
            self.batch_origins.push(origin);
        }
        self.last_registered = points;
        self.last_origin = origin;
        self.frame_count += 1;
    }

    /// The last frame's world-frame points, for visualization tools.
    pub fn registered_points(&self) -> &[Point] {
        &self.last_registered
    }

    /// Whether the local map is due this frame.
    pub fn local_due(&self) -> bool {
        emit_due(self.frame_count, self.config.emit_every)
    }

    /// Whether the global map is due this frame.
    pub fn global_due(&self) -> bool {
        emit_due(self.frame_count, self.config.global_emit_every)
    }

    /// Whether the map regions viz is due this frame.
    pub fn viz_due(&self) -> bool {
        emit_due(self.frame_count, self.config.viz_emit_every)
    }

    /// The chunks whose emitted points changed since the last take.
    pub fn take_changed_chunks(&mut self) -> AHashSet<ChunkKey> {
        self.map.take_changed_chunks()
    }

    pub fn healthy_chunk_keys(&self) -> impl Iterator<Item = ChunkKey> + '_ {
        self.map.healthy_chunk_keys()
    }

    /// Support-gated centers of the given chunks' healthy voxels, flat triples.
    pub fn chunk_points(&self, chunks: &[ChunkKey]) -> Vec<f32> {
        let pool = Arc::clone(&self.pool);
        pool.install(|| chunk_points(&self.map, self.config.voxel_size, chunks))
    }

    /// Cylinder over the batched frames, consuming the batch. An empty batch
    /// yields a zero-radius region at the last origin.
    pub fn take_local_bounds(&mut self) -> Cylinder {
        let bounds = if self.batch_origins.is_empty() {
            let (x, y, z) = self.last_origin;
            Cylinder {
                cx: x,
                cy: y,
                radius: 0.0,
                z_min: z,
                z_max: z,
            }
        } else {
            let margin = self.config.shadow_depth + self.config.voxel_size;
            batch_local_bounds(
                &self.batch_points,
                &self.batch_origins,
                self.config.region_percentile,
                margin,
            )
        };
        self.batch_points.clear();
        self.batch_origins.clear();
        bounds
    }

    /// All healthy voxel centers plus this frame's live voxels, flat triples.
    pub fn global_points(&self) -> Vec<f32> {
        self.pool
            .install(|| emit_points_ungated(&self.map, self.config.voxel_size, &self.live.coarse))
    }

    /// Support-gated healthy voxel centers within `bounds`, plus live voxels,
    /// flat triples.
    pub fn local_points(&self, bounds: &LocalBounds) -> Vec<f32> {
        self.pool.install(|| {
            emit_points(
                &self.map,
                self.config.voxel_size,
                Some(bounds),
                &self.live.coarse,
            )
        })
    }

    /// Fine cells within `bounds` under the same gates as `local_points`, or
    /// None when the fine layer is off.
    pub fn fine_points(&self, bounds: &LocalBounds) -> Option<Vec<f32>> {
        let (divisor, _) = self.config.fine_layer()?;
        Some(self.pool.install(|| {
            emit_points_fine(
                &self.map,
                self.config.voxel_size,
                divisor,
                Some(bounds),
                &self.live.fine,
            )
        }))
    }

    /// Positions and normals of every healthy voxel, fitting stale normals for
    /// the output. Visualization only.
    pub fn normals(&self) -> (Vec<f32>, Vec<f32>) {
        self.pool
            .install(|| global_normals(&self.map, self.config.voxel_size))
    }

    /// Positions, normals, and smallest eigenvalues from a fresh whole-map
    /// pooled fit. Visualization only.
    pub fn normal_fits(&self) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
        self.pool
            .install(|| global_normal_fits(&self.map, self.config.voxel_size))
    }

    /// Reset to an empty map, keeping the config.
    pub fn clear(&mut self) {
        if self.gpu.is_some() {
            self.gpu = gpu::GpuMap::new(&self.config);
        }
        self.map.clear();
        self.live = FrameHits::default();
        self.batch_points.clear();
        self.batch_origins.clear();
        self.last_registered.clear();
        self.last_origin = (0.0, 0.0, 0.0);
        self.frame_count = 0;
    }
}

#[cfg(feature = "gpu")]
mod gpu {
    use ahash::AHashSet;
    use dimos_gpu_mls::brick_map::{BrickMap, MapConfig};
    use dimos_gpu_mls::gpu::Gpu;
    use dimos_gpu_mls::surfaces::{SurfaceExtractor, Window};
    use tracing::{info, warn};

    use dimos_gpu_mls::surfaces::SurfaceConfig;

    use super::{GpuFrame, Point, SurfaceRequest, SurfaceUpdate, SURFACE_CELL_FLOATS};
    use crate::voxel_ray_tracer::{Config, Cylinder, FrameHits, VoxelMap};

    /// Headroom a regrown surface window gets over the one that outgrew it.
    const SURFACE_WINDOW_GROWTH: f32 = 1.25;

    /// The GPU map, updated each frame; its report of the voxels each frame
    /// touched is mirrored into the CPU VoxelMap, which every emit reads.
    pub struct GpuMap {
        map: BrickMap,
        max_points: usize,
        /// Lowest and highest voxel z the map has held.
        z_range: Option<(i32, i32)>,
        extractor: Option<SurfaceExtractor>,
        /// Columns whose surface may have moved since it was last extracted:
        /// a voxel there started or stopped being emitted, or the GPU changed
        /// an emitted one materially (created, removed, past a refit milestone).
        dirty: AHashSet<(i32, i32)>,
        /// Columns the last surfaces sent a cell in, which a later rewrite
        /// that leaves them none must clear.
        sent_columns: AHashSet<(i32, i32)>,
        /// Whether a planner asked for surfaces, so changes are tracked;
        /// until then nothing would drain `dirty`.
        tracking: bool,
        surface_update: SurfaceUpdate,
        pub frame: GpuFrame,
    }

    impl GpuMap {
        /// None, with a warning, when there is no hardware adapter or the map
        /// cannot be built on it.
        pub fn new(config: &Config) -> Option<Self> {
            let gpu = match Gpu::new() {
                Ok(gpu) => gpu,
                Err(e) => {
                    warn!(error = %e, "No GPU, the ray tracer runs on the CPU.");
                    return None;
                }
            };
            let adapter = &gpu.adapter_info;
            if adapter.device_type == dimos_gpu_mls::wgpu::DeviceType::Cpu {
                warn!(adapter = %adapter.name, "Only a software GPU, the ray tracer runs on the CPU.");
                return None;
            }
            if config.graze_recent_frames != u32::MAX {
                warn!(
                    graze_recent_frames = config.graze_recent_frames,
                    "The GPU map always spares grazes; graze_recent_frames is ignored."
                );
            }
            let map_config = MapConfig {
                voxel_size: config.voxel_size,
                fine_divisor: config.fine_divisor,
                max_range: config.max_range,
                ray_subsample: config.ray_subsample,
                shadow_depth: config.shadow_depth,
                grace_depth: config.grace_depth,
                min_health: config.min_health,
                max_health: config.max_health,
                graze_cos: config.graze_cos,
                brick_capacity: BrickMap::max_brick_capacity(&gpu),
                voxel_capacity: BrickMap::max_voxel_capacity(&gpu),
                max_points: config.gpu_max_points,
                list_capacity: config.gpu_list_capacity,
            };
            let name = adapter.name.clone();
            let backend = adapter.backend;
            let mut map = match BrickMap::new(gpu, map_config) {
                Ok(map) => map,
                Err(e) => {
                    warn!(error = %e, "GPU map failed, the ray tracer runs on the CPU.");
                    return None;
                }
            };
            map.enable_report();
            info!(adapter = %name, backend = ?backend, "Ray tracer map on the GPU.");
            Some(Self {
                map,
                max_points: config.gpu_max_points as usize,
                z_range: None,
                extractor: None,
                dirty: AHashSet::new(),
                sent_columns: AHashSet::new(),
                tracking: false,
                surface_update: SurfaceUpdate::default(),
                frame: GpuFrame::default(),
            })
        }

        /// Every voxel on the GPU with its health, read back whole.
        #[cfg(test)]
        pub fn voxels(&self) -> Vec<(crate::voxel_ray_tracer::VoxelKey, i32)> {
            self.map.finish();
            self.map
                .snapshot()
                .into_iter()
                .map(|v| (v.key, v.health))
                .collect()
        }

        /// Seed points in uploads of at most max_points.
        pub fn seed(&mut self, points: &[Point]) {
            let inv = 1.0 / self.map.config.voxel_size;
            for &(_, _, z) in points {
                self.widen_z((z * inv).floor() as i32);
            }
            for batch in points.chunks(self.max_points) {
                self.map.seed(batch);
            }
        }

        /// One frame on the GPU, mirrored into `mirror`. Returns the frame's
        /// hits for the live merge.
        pub fn update(
            &mut self,
            mirror: &mut VoxelMap,
            origin: Point,
            points: &[Point],
            config: &Config,
        ) -> FrameHits {
            let strided: Vec<Point>;
            let points = if points.len() > self.max_points {
                let step = points.len().div_ceil(self.max_points);
                warn!(
                    points = points.len(),
                    gpu_max_points = self.max_points,
                    "Frame over gpu_max_points, kept every {step}th point."
                );
                strided = points.iter().step_by(step).copied().collect();
                &strided
            } else {
                points
            };
            let started = std::time::Instant::now();
            self.map.integrate(origin, points);
            let integrated = std::time::Instant::now();
            let report = self.map.read_report();
            let read = std::time::Instant::now();
            if report.overflow != 0 {
                warn!(
                    overflow = report.overflow,
                    "GPU lists overflowed, raise gpu_list_capacity; the map missed voxels this frame."
                );
            }
            // The live hits are the ones the frame changed; an unchanged hit is
            // already at full health, so the map holds it anyway.
            let mut coarse = AHashSet::with_capacity(report.hits);
            for (i, voxel) in report.voxels.iter().enumerate() {
                if mirror.mirror_voxel(voxel.key, voxel.health, voxel.fine, self.tracking) {
                    self.dirty.insert((voxel.key.0, voxel.key.1));
                }
                if voxel.health.is_some() {
                    self.widen_z(voxel.key.2);
                }
                if i < report.hits {
                    coarse.insert(voxel.key);
                }
            }
            // Isolated noise is not emitted, as the CPU path's local map drops
            // it; a removed emitted voxel flipped above.
            if self.tracking {
                self.dirty.extend(
                    report
                        .changed
                        .iter()
                        .filter(|&&k| mirror.is_emitted(k))
                        .map(|&(x, y, _)| (x, y)),
                );
            }
            let ms = |a: std::time::Instant, b: std::time::Instant| (b - a).as_secs_f64() * 1e3;
            self.frame = GpuFrame {
                integrate_ms: ms(started, integrated),
                report_ms: ms(integrated, read),
                mirror_ms: ms(read, std::time::Instant::now()),
                reported: report.voxels.len(),
            };
            let fine = match config.fine_layer() {
                Some((_, fine_size)) if config.emit_fine => {
                    let inv = 1.0 / fine_size;
                    points
                        .iter()
                        .map(|&(x, y, z)| {
                            (
                                (x * inv).floor() as i32,
                                (y * inv).floor() as i32,
                                (z * inv).floor() as i32,
                            )
                        })
                        .collect()
                }
                _ => AHashSet::new(),
            };
            FrameHits { coarse, fine }
        }

        fn widen_z(&mut self, z: i32) {
            let (lo, hi) = self.z_range.get_or_insert((z, z));
            *lo = (*lo).min(z);
            *hi = (*hi).max(z);
        }

        pub fn surface_update(&self) -> SurfaceUpdate {
            self.surface_update
        }

        pub fn mark_dirty(&mut self, bounds: &Cylinder) {
            let inv = 1.0 / self.map.config.voxel_size;
            let cell = |c: f32| (c * inv).floor() as i32;
            for x in cell(bounds.cx - bounds.radius)..=cell(bounds.cx + bounds.radius) {
                for y in cell(bounds.cy - bounds.radius)..=cell(bounds.cy + bounds.radius) {
                    self.dirty.insert((x, y));
                }
            }
        }

        pub fn time_passes(&mut self) -> bool {
            self.map.timer = dimos_gpu_mls::gpu::PassTimer::new(self.map.gpu(), 32);
            self.map.timer.is_some()
        }

        pub fn take_pass_times(&mut self) -> Vec<(&'static str, f64)> {
            let gpu = std::sync::Arc::clone(self.map.gpu());
            self.map
                .timer
                .as_mut()
                .map(|t| t.take(&gpu))
                .unwrap_or_default()
        }

        /// The MLS surface rewritten around the dirty columns inside
        /// `bounds`, between its z_min and z_max, as rows of x, y, z voxel
        /// center, plane height and wall distance (capped at the request's
        /// wall reach); a NaN z marks a rewritten column with no surface.
        pub fn surfaces(
            &mut self,
            bounds: &Cylinder,
            request: &SurfaceRequest,
            sensor_z: f32,
        ) -> Option<Vec<f32>> {
            let config = SurfaceConfig {
                robot_height_m: request.robot_height_m,
                closing_radius_m: request.closing_radius_m,
                step_threshold_m: request.step_threshold_m,
                max_slope_deg: request.max_slope_deg,
                ..SurfaceConfig::mls_defaults()
            };
            self.surface_update = SurfaceUpdate::default();
            // Changes before the first request were not tracked: start whole.
            if !self.tracking {
                self.tracking = true;
                self.mark_dirty(bounds);
            }
            let Some((z_lo, z_hi)) = self.z_range else {
                return Some(Vec::new());
            };
            let voxel_size = self.map.config.voxel_size;
            let inv = 1.0 / voxel_size;
            let cell = |c: f32| (c * inv).floor() as i32;
            let (x0, x1) = (
                cell(bounds.cx - bounds.radius),
                cell(bounds.cx + bounds.radius),
            );
            let (y0, y1) = (
                cell(bounds.cy - bounds.radius),
                cell(bounds.cy + bounds.radius),
            );
            // A changed column moves surfaces within reach of it, and their
            // wall distances within the wall reach of those; each of those
            // reads as far again around it.
            let reach = config.reach_cells(voxel_size);
            let wall_reach = (request.wall_reach_m * inv).ceil() as i32;
            let pad = reach + wall_reach;
            let headroom = config.clearance_cells(voxel_size) + 1;
            let bricks = |w: u32, h: u32, d: i32| {
                w.div_ceil(8) as u64 * h.div_ceil(8) as u64 * (d as u32).div_ceil(8) as u64
            };
            self.surface_update.region_bricks = bricks(
                (x1 - x0 + 1 + 2 * pad) as u32,
                (y1 - y0 + 1 + 2 * pad) as u32,
                z_hi - z_lo + 1 + headroom,
            );
            // The planner's heights, read as far again around as columns are,
            // since a surface climbs at most a cell per column.
            let export_z = (
                cell(bounds.z_min),
                cell(bounds.z_max.min(sensor_z + request.max_overhead_m)),
            );
            let window_z = (
                (export_z.0 - pad).max(z_lo),
                (export_z.1 + pad).min(z_hi) + headroom,
            );
            if export_z.0 > export_z.1 || window_z.0 > window_z.1 {
                return Some(Vec::new());
            }
            let mut changed: Option<(i32, i32, i32, i32)> = None;
            let mut dirty_columns = 0;
            self.dirty.retain(|&(x, y)| {
                let inside = (x0..=x1).contains(&x) && (y0..=y1).contains(&y);
                if inside {
                    dirty_columns += 1;
                    let c = changed.get_or_insert((x, x, y, y));
                    *c = (c.0.min(x), c.1.max(x), c.2.min(y), c.3.max(y));
                }
                !inside
            });
            self.surface_update.dirty_columns = dirty_columns;
            let Some((cx0, cx1, cy0, cy1)) = changed else {
                return Some(Vec::new());
            };
            // The planner knows voxels only inside the region, so writes stay there.
            let (wx0, wx1, wy0, wy1) = (
                (cx0 - pad).max(x0),
                (cx1 + pad).min(x1),
                (cy0 - pad).max(y0),
                (cy1 + pad).min(y1),
            );
            let window = Window {
                min: (wx0 - pad, wy0 - pad, window_z.0),
                size: (
                    (wx1 - wx0 + 1 + 2 * pad) as u32,
                    (wy1 - wy0 + 1 + 2 * pad) as u32,
                    (window_z.1 - window_z.0 + 1) as u32,
                ),
            };
            self.surface_update.bricks = bricks(window.size.0, window.size.1, window.size.2 as i32);
            let reuse = self
                .extractor
                .as_mut()
                .is_some_and(|e| e.config == config && e.retarget(window));
            if !reuse {
                let grow = |n: u32| (n as f32 * SURFACE_WINDOW_GROWTH).ceil() as u32;
                let room = Window {
                    min: window.min,
                    size: (
                        grow(window.size.0),
                        grow(window.size.1),
                        grow(window.size.2),
                    ),
                };
                self.extractor = None;
                let mut extractor = match SurfaceExtractor::new(&self.map, room, config) {
                    Ok(e) => e,
                    Err(e) => {
                        warn!(error = %e, "GPU surface window does not fit, no surfaces this frame.");
                        return None;
                    }
                };
                extractor.retarget(window);
                self.extractor = Some(extractor);
            }
            let extractor = self.extractor.as_mut().expect("set above");
            let started = std::time::Instant::now();
            extractor.extract(&mut self.map);
            let extracted = std::time::Instant::now();
            let rows = extractor.export((wx0, wx1, wy0, wy1), export_z);
            self.surface_update.extract_ms = (extracted - started).as_secs_f64() * 1e3;
            self.surface_update.export_ms = extracted.elapsed().as_secs_f64() * 1e3;
            let half = 0.5 * voxel_size;
            let center = |c: i32| c as f32 * voxel_size + half;
            let mut out = Vec::with_capacity(SURFACE_CELL_FLOATS * rows.len());
            let mut with_cells: AHashSet<(i32, i32)> = AHashSet::with_capacity(rows.len());
            for cell in &rows {
                let (x, y, z) = cell.key;
                let w = match request.wall_distance {
                    true if request.wall_reach_m > 0.0 => cell
                        .wall_m
                        .unwrap_or(f32::INFINITY)
                        .min(request.wall_reach_m),
                    true => cell.wall_m.unwrap_or(f32::INFINITY),
                    false => f32::INFINITY,
                };
                out.extend_from_slice(&[center(x), center(y), center(z), cell.height_m, w]);
                with_cells.insert((x, y));
            }
            self.surface_update.cells = rows.len();
            // A column that held cells and now has none in range is cleared by a
            // NaN-z row; one that never held any needs none.
            let in_write =
                |&(x, y): &(i32, i32)| (wx0..=wx1).contains(&x) && (wy0..=wy1).contains(&y);
            for &(x, y) in self
                .sent_columns
                .iter()
                .filter(|c| in_write(c) && !with_cells.contains(c))
            {
                out.extend_from_slice(&[center(x), center(y), f32::NAN, f32::NAN, f32::NAN]);
            }
            self.sent_columns.retain(|c| !in_write(c));
            self.sent_columns.extend(with_cells);
            let (ww, wh) = ((wx1 - wx0 + 1) as usize, (wy1 - wy0 + 1) as usize);
            self.surface_update.columns = (ww * wh) as u64;
            Some(out)
        }
    }
}

#[cfg(not(feature = "gpu"))]
mod gpu {
    use tracing::warn;

    use super::{Point, SurfaceRequest, SurfaceUpdate};
    use crate::voxel_ray_tracer::{Config, Cylinder, FrameHits, VoxelMap};

    pub enum GpuMap {}

    impl GpuMap {
        pub fn new(_config: &Config) -> Option<Self> {
            warn!("Built without the gpu feature, the ray tracer runs on the CPU.");
            None
        }

        pub fn seed(&mut self, _points: &[Point]) {
            match *self {}
        }

        pub fn surface_update(&self) -> SurfaceUpdate {
            match *self {}
        }

        pub fn mark_dirty(&mut self, _bounds: &Cylinder) {
            match *self {}
        }

        pub fn time_passes(&mut self) -> bool {
            match *self {}
        }

        pub fn take_pass_times(&mut self) -> Vec<(&'static str, f64)> {
            match *self {}
        }

        pub fn surfaces(
            &mut self,
            _bounds: &Cylinder,
            _request: &SurfaceRequest,
            _sensor_z: f32,
        ) -> Option<Vec<f32>> {
            match *self {}
        }

        pub fn update(
            &mut self,
            _mirror: &mut VoxelMap,
            _origin: Point,
            _points: &[Point],
            _config: &Config,
        ) -> FrameHits {
            match *self {}
        }
    }
}

/// Map points into the world frame by the pose, in place.
pub fn register(points: &mut [Point], pose: Pose) {
    let (px, py, pz) = pose.position;
    let (qx, qy, qz, qw) = pose.orientation;
    let translation = Vector3::new(px, py, pz);
    let rot = UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz)).to_rotation_matrix();
    for p in points.iter_mut() {
        let w = rot * Vector3::new(p.0, p.1, p.2) + translation;
        *p = (w.x, w.y, w.z);
    }
}

/// Whether the Nth-frame output fires this frame. Zero disables it.
fn emit_due(frame_count: u32, every: u32) -> bool {
    every != 0 && frame_count.is_multiple_of(every)
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDENTITY: (f32, f32, f32, f32) = (0.0, 0.0, 0.0, 1.0);

    fn config() -> Config {
        Config {
            voxel_size: 1.0,
            fine_divisor: 0,
            emit_fine: false,
            max_range: 100.0,
            ray_subsample: 1,
            shadow_depth: 2.0,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            graze_recent_frames: u32::MAX,
            gpu: false,
            gpu_max_points: 1 << 18,
            gpu_list_capacity: 1 << 20,
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
            viz_cell_m: 4.0,
            viz_emit_every: 0,
            viz_sweep_cells: 0,
        }
    }

    #[test]
    fn add_frame_registers_by_pose() {
        let mut mapper = Mapper::new(config());
        // Yaw 90 deg: sensor +x becomes world +y. Sensor at (10, 0, 0).
        let half = 2.0_f32.sqrt() / 2.0;
        let pose = Pose {
            position: (10.0, 0.0, 0.0),
            orientation: (0.0, 0.0, half, half),
        };
        mapper.add_frame(vec![(3.5, 0.0, 0.5)], pose);
        let world = mapper.registered_points();
        assert!((world[0].0 - 10.0).abs() < 1e-5);
        assert!((world[0].1 - 3.5).abs() < 1e-5);
        let global = mapper.global_points();
        assert_eq!(global, vec![10.5, 3.5, 0.5]);
    }

    #[test]
    fn world_frame_add_skips_registration() {
        let mut mapper = Mapper::new(config());
        mapper.add_frame_world(vec![(5.5, 0.5, 0.5)], (0.0, 0.0, 0.0));
        assert_eq!(mapper.registered_points(), &[(5.5, 0.5, 0.5)]);
        assert_eq!(mapper.global_points(), vec![5.5, 0.5, 0.5]);
    }

    #[test]
    fn identity_pose_is_passthrough() {
        let mut mapper = Mapper::new(config());
        let pose = Pose {
            position: (0.0, 0.0, 0.0),
            orientation: IDENTITY,
        };
        mapper.add_frame(vec![(5.5, 0.5, 0.5)], pose);
        assert_eq!(mapper.registered_points(), &[(5.5, 0.5, 0.5)]);
        assert_eq!(mapper.global_points(), vec![5.5, 0.5, 0.5]);
    }

    #[test]
    fn take_local_bounds_consumes_batch_and_falls_back_to_last_origin() {
        let mut mapper = Mapper::new(config());
        let pose = Pose {
            position: (1.0, 2.0, 3.0),
            orientation: IDENTITY,
        };
        mapper.add_frame(vec![(2.0, 0.5, 0.5)], pose);
        let c = mapper.take_local_bounds();
        assert_eq!((c.cx, c.cy), (1.0, 2.0));
        assert!(c.radius > 0.0);

        // Batch consumed: the next call has nothing and centers on the pose.
        let c = mapper.take_local_bounds();
        assert_eq!((c.cx, c.cy, c.radius), (1.0, 2.0, 0.0));
        assert_eq!((c.z_min, c.z_max), (3.0, 3.0));
    }

    #[test]
    fn cadence_follows_config() {
        let cfg = Config {
            emit_every: 2,
            global_emit_every: 3,
            ..config()
        };
        let mut mapper = Mapper::new(cfg);
        let pose = Pose {
            position: (0.0, 0.0, 0.0),
            orientation: IDENTITY,
        };
        let mut dues = Vec::new();
        for _ in 0..6 {
            mapper.add_frame(vec![(5.5, 0.5, 0.5)], pose);
            dues.push((mapper.local_due(), mapper.global_due()));
        }
        assert_eq!(
            dues,
            vec![
                (false, false),
                (true, false),
                (false, true),
                (true, false),
                (false, false),
                (true, true),
            ]
        );
    }

    #[test]
    fn zero_cadence_disables_emits() {
        let cfg = Config {
            emit_every: 0,
            global_emit_every: 0,
            ..config()
        };
        let mut mapper = Mapper::new(cfg);
        let pose = Pose {
            position: (0.0, 0.0, 0.0),
            orientation: IDENTITY,
        };
        for _ in 0..4 {
            mapper.add_frame(vec![(5.5, 0.5, 0.5)], pose);
            assert_eq!((mapper.local_due(), mapper.global_due()), (false, false));
        }
    }

    #[test]
    fn seed_points_emits_without_batching_regions() {
        let mut mapper = Mapper::new(config());
        assert_eq!(mapper.seed_points(&[(5.5, 0.5, 0.5), (6.5, 0.5, 0.5)]), 2);

        assert_eq!(mapper.global_points().len(), 6);
        let bounds = LocalBounds {
            origin_x: 5.0,
            origin_y: 0.0,
            r_xy_max_sq: 100.0,
            z_min: -5.0,
            z_max: 5.0,
        };
        assert_eq!(mapper.local_points(&bounds).len(), 6);

        let c = mapper.take_local_bounds();
        assert_eq!(c.radius, 0.0, "a seed must not feed the region batch");

        assert_eq!(mapper.seed_points(&[(5.5, 0.5, 0.5)]), 0);
    }

    /// local_points applies the support gate. global_points stays unfiltered.
    #[test]
    fn local_points_apply_support_gate() {
        let cfg = Config {
            support_min: 3,
            ..config()
        };
        let mut mapper = Mapper::new(cfg);
        let mut cloud: Vec<Point> = Vec::new();
        for x in 0..3 {
            for y in 0..3 {
                cloud.push((x as f32 + 0.5, y as f32 + 0.5, 0.5));
            }
        }
        cloud.push((20.5, 20.5, 0.5));
        assert_eq!(mapper.seed_points(&cloud), 10);

        let everywhere = Cylinder {
            cx: 0.0,
            cy: 0.0,
            radius: 100.0,
            z_min: -1.0,
            z_max: 10.0,
        }
        .bounds();
        assert_eq!(
            mapper.local_points(&everywhere).len(),
            27,
            "isolated seed gated out"
        );
        assert_eq!(mapper.global_points().len(), 30);
    }

    #[test]
    fn fine_points_none_when_layer_off() {
        let mut mapper = Mapper::new(config());
        let pose = Pose {
            position: (0.0, 0.0, 0.0),
            orientation: IDENTITY,
        };
        mapper.add_frame(vec![(5.5, 0.5, 0.5)], pose);
        let bounds = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 1e6,
            z_min: -10.0,
            z_max: 10.0,
        };
        assert!(mapper.fine_points(&bounds).is_none());

        let mut fine = Mapper::new(Config {
            fine_divisor: 2,
            ..config()
        });
        fine.add_frame(vec![(5.1, 0.1, 0.1)], pose);
        assert_eq!(fine.fine_points(&bounds).unwrap(), vec![5.25, 0.25, 0.25]);
    }

    /// Fraction of the union two point sets disagree on.
    #[cfg(feature = "gpu")]
    fn disagreement(a: &[f32], b: &[f32]) -> (f64, usize) {
        let set = |p: &[f32]| -> AHashSet<(u32, u32, u32)> {
            p.chunks_exact(3)
                .map(|c| (c[0].to_bits(), c[1].to_bits(), c[2].to_bits()))
                .collect()
        };
        let (a, b) = (set(a), set(b));
        let differ = a.symmetric_difference(&b).count();
        (differ as f64 / a.union(&b).count().max(1) as f64, a.len())
    }

    /// The GPU-backed mapper mirrors the GPU map exactly, and emits what the CPU
    /// mapper does up to the GPU's boundary flips (map_parity allows 1e-3).
    #[cfg(feature = "gpu")]
    #[test]
    fn gpu_mapper_emits_what_the_cpu_mapper_does() {
        use dimos_gpu_mls::synthetic::{Lcg, Scene};
        const EMIT_TOLERANCE: f64 = 1e-3;
        let cfg = Config {
            voxel_size: 0.08,
            fine_divisor: 3,
            max_range: 15.0,
            shadow_depth: 0.1,
            grace_depth: 0.2,
            min_health: -1,
            max_health: 5,
            graze_cos: 0.7,
            support_min: 4,
            ..config()
        };
        let mut cpu = Mapper::new(cfg.clone());
        let mut gpu = Mapper::new(Config { gpu: true, ..cfg });
        if !gpu.on_gpu() {
            eprintln!("no hardware GPU adapter, skipped");
            return;
        }
        let scene = Scene::room(12.0, 2.5);
        let mut rng = Lcg(7);
        for origin in scene.trajectory(24, 0.4) {
            let points = scene.scan(origin, 20_000, &mut rng, 0.01, 20.0);
            cpu.add_frame_world(points.clone(), origin);
            gpu.add_frame_world(points, origin);
            let (cpu_bounds, gpu_bounds) = (cpu.take_local_bounds(), gpu.take_local_bounds());
            assert_eq!(cpu_bounds.radius, gpu_bounds.radius);
            let (local, n) = disagreement(
                &cpu.local_points(&cpu_bounds.bounds()),
                &gpu.local_points(&gpu_bounds.bounds()),
            );
            assert!(
                n > 0 && local <= EMIT_TOLERANCE,
                "local maps differ on {local} of {n}"
            );
        }
        let (global, n) = disagreement(&cpu.global_points(), &gpu.global_points());
        eprintln!("global maps differ on {global} of {n} points");
        assert!(
            global <= EMIT_TOLERANCE,
            "global maps differ on {global} of {n}"
        );

        let on_gpu: AHashSet<_> = gpu.gpu.as_ref().unwrap().voxels().into_iter().collect();
        let mirrored: AHashSet<_> = gpu.map.voxel_healths().collect();
        assert_eq!(on_gpu.len(), mirrored.len());
        assert!(on_gpu == mirrored, "the mirror drifted from the GPU map");
    }
}
