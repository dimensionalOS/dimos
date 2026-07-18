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

//
// Rust native-module port of `VoxelGridMapper` (dimos/mapping/voxels.py):
// accumulate world-frame lidar clouds into a global voxel map and publish the
// full map every `emit_every` frames.
//
// Ports: lidar (PointCloud2 in) -> global_map (PointCloud2 out).

use std::time::Duration;

use dimos_mappers::cloud::{extract_xyz, points_to_cloud};
use dimos_mappers::voxel_grid::VoxelGrid;
use dimos_module::{
    error_throttled, native_config, run_with_transport, warn_throttled, Input, Module, Output,
};
use lcm_msgs::sensor_msgs::PointCloud2;

#[native_config]
struct Config {
    /// Edge length of one voxel cube (m). Python: VoxelGridMapperConfig.voxel_size.
    #[validate(range(exclusive_min = 0.0))]
    voxel_size: f64,
    /// Clear all existing voxels in an XY column before inserting new ones.
    carve_columns: bool,
    /// Frame the emitted global map is stamped with ("frame_id" is reserved by
    /// the Python-side NativeModuleConfig, hence the name).
    map_frame_id: String,
    /// Publish the accumulated map every Nth frame. Zero disables emission.
    #[validate(range(min = 0))]
    emit_every: u32,
}

#[derive(Module)]
struct VoxelMapper {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

    #[config]
    config: Config,

    grid: VoxelGrid,
    frame_count: u64,
}

impl VoxelMapper {
    async fn on_lidar(&mut self, msg: PointCloud2) {
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

        let voxel_size = self.config.voxel_size as f32;
        self.grid
            .add_frame(&points, voxel_size, self.config.carve_columns);
        self.frame_count += 1;

        let every = self.config.emit_every as u64;
        if every != 0 && self.frame_count.is_multiple_of(every) {
            let centers = self.grid.global_points(voxel_size);
            let cloud = points_to_cloud(&centers, &self.config.map_frame_id, msg.header.stamp);
            if let Err(e) = self.global_map.publish(&cloud).await {
                error_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Global map failed to publish",
                );
            }
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<VoxelMapper>().await;
}
