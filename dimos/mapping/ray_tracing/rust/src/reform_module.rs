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

//! The ray tracer that follows a pose-graph optimizer's loop closures: on each
//! `loop_closure_event` it warps its whole voxel map into the corrected frame
//! in place, so the map and the local regions it hands on stay corrected.

use std::time::{Duration, Instant};

use dimos_module::{warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::PoseStamped;
use lcm_msgs::sensor_msgs::PointCloud2;
use tracing::debug;
use validator::ValidationError;

use crate::module::{Job, WorkerHandle, WorkerOutputs};
use crate::reform::{decode_loop_closure_event, decode_pose_graph, ReformParams, ReformState};
use crate::voxel_ray_tracer::{validate_config, Config};

/// The ray tracer's config plus how closures warp it.
#[dimos_module::native_config]
#[derive(Clone)]
#[validate(schema(function = "validate_reform_config"))]
pub struct ReformConfig {
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    #[validate(range(min = 0, max = 4))]
    pub fine_divisor: u32,
    pub emit_fine: bool,
    #[validate(range(min = 0.0))]
    pub max_range: f32,
    #[validate(range(min = 1))]
    pub ray_subsample: u32,
    #[validate(range(min = 0.0))]
    pub shadow_depth: f32,
    #[validate(range(min = 0.0))]
    pub grace_depth: f32,
    pub min_health: i32,
    #[validate(range(min = 1))]
    pub max_health: i32,
    #[validate(range(min = 0.0, max = 1.0))]
    pub graze_cos: f32,
    #[validate(range(min = 0))]
    pub support_min: i32,
    #[validate(range(min = 0))]
    pub emit_every: u32,
    #[validate(range(min = 0))]
    pub global_emit_every: u32,
    #[validate(range(min = 0.0, max = 100.0))]
    pub region_percentile: f32,
    #[validate(length(min = 1))]
    pub world_frame: String,
    #[validate(range(exclusive_min = 0.0))]
    pub tf_match_tolerance_s: f64,
    #[validate(range(min = 0.0))]
    pub max_cloud_age_s: f64,
    #[validate(range(min = 0.0))]
    pub tf_wait_timeout_s: f64,
    #[validate(range(min = 1))]
    pub worker_threads: u32,
    #[validate(range(exclusive_min = 0.0))]
    pub region_m: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub viz_cell_m: f32,
    pub viz_emit_every: u32,
    pub viz_sweep_cells: u32,
    /// Spatial width of the blend between keyframe corrections, m. 0 picks
    /// twice the mean keyframe spacing.
    #[validate(range(min = 0.0))]
    pub reform_blend_sigma_m: f64,
    /// Temporal width of the blend, s, which keeps the passes of a revisited
    /// place apart. 0 picks twice the mean keyframe interval.
    #[validate(range(min = 0.0))]
    pub reform_blend_time_sigma_s: f64,
    /// A keyframe whose correction moves its surroundings less than this
    /// waits for a later closure, so near no-op closures cost nothing.
    #[validate(range(min = 0.0))]
    pub reform_min_shift_m: f64,
    /// A warp blends one keyframe per cell of this size (and of the time
    /// width in time), m; 0 picks the blend width, negative blends every keyframe.
    pub reform_thin_cell_m: f64,
    /// Where to write the healthy voxel centers (little-endian f32 xyz) every 10 s, empty for never.
    pub dump_path: String,
}

fn validate_reform_config(config: &ReformConfig) -> Result<(), ValidationError> {
    validate_config(&config.base())
}

impl ReformConfig {
    pub fn base(&self) -> Config {
        Config {
            voxel_size: self.voxel_size,
            fine_divisor: self.fine_divisor,
            emit_fine: self.emit_fine,
            max_range: self.max_range,
            ray_subsample: self.ray_subsample,
            shadow_depth: self.shadow_depth,
            grace_depth: self.grace_depth,
            min_health: self.min_health,
            max_health: self.max_health,
            graze_cos: self.graze_cos,
            support_min: self.support_min,
            emit_every: self.emit_every,
            global_emit_every: self.global_emit_every,
            region_percentile: self.region_percentile,
            world_frame: self.world_frame.clone(),
            tf_match_tolerance_s: self.tf_match_tolerance_s,
            max_cloud_age_s: self.max_cloud_age_s,
            tf_wait_timeout_s: self.tf_wait_timeout_s,
            worker_threads: self.worker_threads,
            region_m: self.region_m,
            viz_cell_m: self.viz_cell_m,
            viz_emit_every: self.viz_emit_every,
            viz_sweep_cells: self.viz_sweep_cells,
        }
    }

    pub fn reform(&self) -> ReformParams {
        ReformParams {
            blend_sigma_m: self.reform_blend_sigma_m,
            blend_time_sigma_s: self.reform_blend_time_sigma_s,
            min_shift_m: self.reform_min_shift_m,
            thin_cell_m: self.reform_thin_cell_m,
        }
    }
}

#[derive(Module)]
#[module(name = "reform_ray_tracing", setup = spawn_worker, teardown = stop_worker)]
pub struct ReformRayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    // An externally loaded map cloud. Only the first one seeds the map.
    #[input(decode = PointCloud2::decode, handler = on_loaded_map)]
    loaded_map: Input<PointCloud2>,

    // Every keyframe's pose, for the anchors that hold the unmoved parts.
    #[input(decode = PointCloud2::decode, handler = on_pose_graph)]
    pose_graph: Input<PointCloud2>,

    // Per closure, each keyframe it moved: its pre pose and the delta.
    #[input(decode = PointCloud2::decode, handler = on_loop_closure_event)]
    loop_closure_event: Input<PointCloud2>,

    #[tf]
    tf: Tf,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map_fine: Output<PointCloud2>,

    #[output(encode = PoseStamped::encode)]
    region_bounds: Output<PoseStamped>,

    #[output(encode = PointCloud2::encode)]
    map_regions: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    seed_map: Output<PointCloud2>,

    #[output(encode = PoseStamped::encode)]
    seed_bounds: Output<PoseStamped>,

    #[config]
    config: ReformConfig,

    worker: WorkerHandle,
    reform: ReformState,
}

impl ReformRayTracingVoxelMap {
    async fn spawn_worker(&mut self) {
        let outputs = WorkerOutputs {
            global_map: self.global_map.clone(),
            local_map: self.local_map.clone(),
            local_map_fine: self.local_map_fine.clone(),
            region_bounds: self.region_bounds.clone(),
            map_regions: self.map_regions.clone(),
            seed_map: self.seed_map.clone(),
            seed_bounds: self.seed_bounds.clone(),
        };
        let dump_path = (!self.config.dump_path.is_empty()).then(|| self.config.dump_path.clone());
        self.worker = WorkerHandle::spawn(
            self.tf.clone(),
            self.config.base(),
            outputs,
            true,
            dump_path,
        );
    }

    async fn stop_worker(&mut self) {
        self.worker.stop();
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        self.worker.enqueue(Job::Lidar(msg, Instant::now())).await;
    }

    async fn on_loaded_map(&mut self, msg: PointCloud2) {
        self.worker.enqueue(Job::LoadedMap(msg)).await;
    }

    async fn on_pose_graph(&mut self, msg: PointCloud2) {
        match decode_pose_graph(&msg) {
            Ok(records) => self.reform.add_keyframes(&records),
            Err(e) => warn_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Failed to decode pose_graph, dropped it."
            ),
        }
    }

    async fn on_loop_closure_event(&mut self, msg: PointCloud2) {
        let moved = match decode_loop_closure_event(&msg) {
            Ok(moved) => moved,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to decode loop_closure_event, dropped it."
                );
                return;
            }
        };
        match self.reform.closure(&moved, &self.config.reform()) {
            Some(warp) => self.worker.enqueue(Job::Warp(warp)).await,
            None => debug!(
                keyframes = moved.len(),
                "Loop closure moved nothing past reform_min_shift_m, held for the next."
            ),
        }
    }
}
