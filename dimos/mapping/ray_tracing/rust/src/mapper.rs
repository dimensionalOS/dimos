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
    batch_local_bounds, coarse_of_fine, emit_points, emit_points_fine, global_normal_fits,
    metric_voxel_keys, update_map, Config, Cylinder, FrameHits, LocalBounds, VoxelMap,
};

pub type Point = (f32, f32, f32);

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
}

impl Mapper {
    /// The config must already be validated at the caller's boundary.
    pub fn new(config: Config) -> Self {
        Self {
            pool: worker_pool(config.worker_threads),
            config,
            map: VoxelMap::default(),
            live: FrameHits::default(),
            batch_points: Vec::new(),
            batch_origins: Vec::new(),
            last_registered: Vec::new(),
            last_origin: (0.0, 0.0, 0.0),
            frame_count: 0,
        }
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
        let (px, py, pz) = pose.position;
        let (qx, qy, qz, qw) = pose.orientation;
        let translation = Vector3::new(px, py, pz);
        let rot =
            UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz)).to_rotation_matrix();
        for p in sensor_points.iter_mut() {
            let w = rot * Vector3::new(p.0, p.1, p.2) + translation;
            *p = (w.x, w.y, w.z);
        }
        self.ingest(sensor_points, pose.position);
    }

    /// Fold an already world-frame cloud into the map, raycasting from `origin`.
    pub fn add_frame_world(&mut self, world_points: Vec<Point>, origin: Point) {
        self.ingest(world_points, origin);
    }

    fn ingest(&mut self, points: Vec<Point>, origin: Point) {
        let pool = Arc::clone(&self.pool);
        self.live = pool.install(|| update_map(&mut self.map, origin, &points, &self.config));

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
        self.pool.install(|| {
            emit_points(
                &self.map,
                self.config.voxel_size,
                None,
                0,
                &self.live.coarse,
            )
        })
    }

    /// Support-gated healthy voxel centers within `bounds`, plus live voxels,
    /// flat triples.
    pub fn local_points(&self, bounds: &LocalBounds) -> Vec<f32> {
        self.pool.install(|| {
            emit_points(
                &self.map,
                self.config.voxel_size,
                Some(bounds),
                self.config.support_min,
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
                self.config.support_min,
                &self.live.fine,
            )
        }))
    }

    /// Positions, normals, and smallest eigenvalues from a fresh whole-map
    /// pooled fit. Visualization only.
    pub fn normal_fits(&self) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
        self.pool
            .install(|| global_normal_fits(&self.map, self.config.voxel_size))
    }

    /// Delete the voxels covering `points`, world-frame metric positions the
    /// caller knows to be free. Returns how many voxels were removed.
    ///
    /// This frame's live hits are dropped alongside them, so a cleared voxel
    /// cannot come back out of the live overlay before the next frame replaces
    /// it.
    pub fn clear_metric(&mut self, points: impl IntoIterator<Item = Point>) -> usize {
        let keys: Vec<_> = metric_voxel_keys(points, self.config.voxel_size).collect();
        for key in &keys {
            self.live.coarse.remove(key);
        }
        if let Some((divisor, _)) = self.config.fine_layer() {
            let cleared: AHashSet<_> = keys.iter().copied().collect();
            self.live
                .fine
                .retain(|&fine| !cleared.contains(&coarse_of_fine(fine, divisor as i32)));
        }
        self.map.clear_voxels(keys)
    }

    /// Reset to an empty map, keeping the config.
    pub fn clear(&mut self) {
        self.map.clear();
        self.live = FrameHits::default();
        self.batch_points.clear();
        self.batch_origins.clear();
        self.last_registered.clear();
        self.last_origin = (0.0, 0.0, 0.0);
        self.frame_count = 0;
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
            support_min: 0,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
            world_frame: "world".to_string(),
            tf_match_tolerance_s: 0.1,
            worker_threads: 4,
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

    /// The ghost case: a sensor deposits returns off its own body, then tells
    /// the mapper that volume is free. Ray tracing never reaches it because the
    /// body occludes what is behind it, so the mask is the only way out.
    #[test]
    fn clear_metric_erases_voxels_ray_tracing_cannot_reach() {
        let mut mapper = Mapper::new(config());
        let pose = Pose {
            position: (0.0, 0.0, 0.0),
            orientation: IDENTITY,
        };
        mapper.add_frame(vec![(5.5, 0.5, 0.5)], pose);
        assert_eq!(mapper.global_points(), vec![5.5, 0.5, 0.5]);

        assert_eq!(mapper.clear_metric([(5.5, 0.5, 0.5)]), 1);

        assert!(mapper.global_points().is_empty());
        assert_eq!(mapper.clear_metric([(5.5, 0.5, 0.5)]), 0);
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
}
