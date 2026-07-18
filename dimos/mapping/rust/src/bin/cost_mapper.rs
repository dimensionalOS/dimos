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
// Rust native-module port of `CostMapper` (dimos/mapping/costmapper.py):
// turn the accumulated global point cloud into a terrain-slope OccupancyGrid.
//
// Ports: global_map, merged_map (PointCloud2 in) -> global_costmap
// (OccupancyGrid out). Like the Python combine_latest pipeline, the merged map
// wins once one has ever arrived, and computation is (re)triggered by every
// message on either input after the first global_map.

use std::time::Duration;

use dimos_mappers::cloud::extract_xyz;
use dimos_mappers::occupancy::{height_cost_occupancy, CostGrid, HeightCostConfig};
use dimos_module::{
    error_throttled, native_config, run_with_transport, warn_throttled, Input, Module, Output,
};
use lcm_msgs::geometry_msgs::{Point, Pose, Quaternion};
use lcm_msgs::nav_msgs::{MapMetaData, OccupancyGrid};
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::std_msgs::Header;
use validator::ValidationError;

#[native_config]
#[validate(schema(function = "validate_algo"))]
struct Config {
    /// Occupancy algorithm. Only "height_cost" (the unitree-go2 default) is
    /// ported; other OCCUPANCY_ALGOS entries are Python-only for now.
    algo: String,
    #[validate(range(exclusive_min = 0.0))]
    resolution: f64,
    can_pass_under: f64,
    can_climb: f64,
    ignore_noise: f64,
    smoothing: f64,
    /// Zero-cost disc around the world origin for robots that can't see
    /// directly below themselves.
    #[validate(range(min = 0.0))]
    initial_safe_radius_meters: f64,
}

fn validate_algo(cfg: &Config) -> Result<(), ValidationError> {
    if cfg.algo != "height_cost" {
        return Err(ValidationError::new("unsupported_algo"));
    }
    Ok(())
}

#[derive(Module)]
struct CostMapper {
    #[input(decode = PointCloud2::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

    #[input(decode = PointCloud2::decode, handler = on_merged_map)]
    merged_map: Input<PointCloud2>,

    #[output(encode = OccupancyGrid::encode)]
    global_costmap: Output<OccupancyGrid>,

    #[config]
    config: Config,

    latest_merged: Option<PointCloud2>,
    have_global: bool,
}

impl CostMapper {
    async fn on_global_map(&mut self, msg: PointCloud2) {
        self.have_global = true;
        let selected = self.latest_merged.take();
        match selected {
            Some(merged) => {
                self.compute_and_publish(&merged).await;
                self.latest_merged = Some(merged);
            }
            None => self.compute_and_publish(&msg).await,
        }
    }

    async fn on_merged_map(&mut self, msg: PointCloud2) {
        if self.have_global {
            self.compute_and_publish(&msg).await;
        }
        self.latest_merged = Some(msg);
    }

    async fn compute_and_publish(&mut self, cloud: &PointCloud2) {
        let points = match extract_xyz(cloud) {
            Ok(p) => p,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to get map points, dropped a costmap update.",
                );
                return;
            }
        };
        // Python upcasts the float32 cloud to float64 before gridding.
        let points: Vec<(f64, f64, f64)> = points
            .iter()
            .map(|&(x, y, z)| (x as f64, y as f64, z as f64))
            .collect();

        let hc = HeightCostConfig {
            resolution: self.config.resolution,
            can_pass_under: self.config.can_pass_under,
            can_climb: self.config.can_climb,
            ignore_noise: self.config.ignore_noise,
            smoothing: self.config.smoothing,
        };
        let mut grid = height_cost_occupancy(&points, &hc);
        self.apply_initial_safe_radius(&mut grid);

        let msg = OccupancyGrid {
            header: Header {
                seq: 0,
                stamp: cloud.header.stamp.clone(),
                frame_id: cloud.header.frame_id.clone(),
            },
            info: MapMetaData {
                map_load_time: cloud.header.stamp.clone(),
                resolution: self.config.resolution as f32,
                width: grid.width as i32,
                height: grid.height as i32,
                origin: Pose {
                    position: Point {
                        x: grid.origin_x,
                        y: grid.origin_y,
                        z: 0.0,
                    },
                    orientation: Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
            },
            data: grid.data,
        };
        if let Err(e) = self.global_costmap.publish(&msg).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Costmap failed to publish",
            );
        }
    }

    /// Port of `CostMapper._apply_initial_safe_radius`: zero every cell whose
    /// (corner) world position lies within the disc around the world origin,
    /// with half-a-cell tolerance.
    fn apply_initial_safe_radius(&self, grid: &mut CostGrid) {
        let radius = self.config.initial_safe_radius_meters;
        if radius <= 0.0 || grid.data.is_empty() {
            return;
        }
        let res = self.config.resolution;
        let effective = radius + res * 0.5;
        let effective_sq = effective * effective;
        for r in 0..grid.height {
            let wy = r as f64 * res + grid.origin_y;
            for c in 0..grid.width {
                let wx = c as f64 * res + grid.origin_x;
                if wx * wx + wy * wy <= effective_sq {
                    grid.data[r * grid.width + c] = 0;
                }
            }
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<CostMapper>().await;
}
