// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::time::Duration;

use ahash::AHashSet;
use dimos_module::{error_throttled, run, warn_throttled, Input, LcmTransport, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::PointCloud2;

use dimos_voxel_ray_tracing::{
    build_pointclouds, extract_xyz, update_map, world_to_voxel, LocalBounds, VoxelKey, VoxelMap,
    VoxelMapperConfig,
};

#[derive(Module)]
#[module(setup = validate_config)]
struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map: Output<PointCloud2>,

    #[config]
    config: VoxelMapperConfig,

    map: VoxelMap,
    last_origin: Option<(f32, f32, f32)>,
}

impl RayTracingVoxelMap {
    /// Make sure all the configs are valid on setup
    async fn validate_config(&self) {
        if let Err(e) = self.config.validate() {
            panic!("voxel_ray_tracing: {}", e);
        }
    }

    async fn on_odometry(&mut self, msg: Odometry) {
        self.last_origin = Some((
            msg.pose.pose.position.x as f32,
            msg.pose.pose.position.y as f32,
            msg.pose.pose.position.z as f32,
        ));
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        let Some(origin) = self.last_origin else {
            // Need at least one odometry sample before we can raycast.
            return;
        };

        let voxel_size = self.config.voxel_size;

        let points = match extract_xyz(&msg) {
            Ok(p) => p,
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

        let inv = 1.0_f32 / voxel_size;
        let half = voxel_size * 0.5;
        let mut live: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        let mut r_xy_max_sq = 0.0_f32;
        for &(x, y, z) in &points {
            let key = world_to_voxel(x, y, z, inv);
            live.insert(key);
            let cx = key.0 as f32 * voxel_size + half;
            let cy = key.1 as f32 * voxel_size + half;
            let cz = key.2 as f32 * voxel_size + half;
            z_min = z_min.min(cz);
            z_max = z_max.max(cz);
            let dx = cx - origin.0;
            let dy = cy - origin.1;
            r_xy_max_sq = r_xy_max_sq.max(dx * dx + dy * dy);
        }
        let cylinder = LocalBounds {
            origin_x: origin.0,
            origin_y: origin.1,
            r_xy_max_sq,
            z_min,
            z_max,
        };

        update_map(&mut self.map, origin, &points, &self.config);

        let (global_cloud, local_cloud) = build_pointclouds(
            &self.map,
            &live,
            voxel_size,
            &cylinder,
            &msg.header.frame_id,
            msg.header.stamp,
        );
        if let Err(e) = self.global_map.publish(&global_cloud).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Updated global voxel map failed to publish",
            );
        }
        if let Err(e) = self.local_map.publish(&local_cloud).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Updated local voxel map failed to publish",
            );
        }
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<RayTracingVoxelMap, _>(transport)
        .await
        .expect("voxel_ray_tracing run failed");
}
