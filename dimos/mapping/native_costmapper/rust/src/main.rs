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

use std::time::Duration;

use dimos_module::{error_throttled, run_with_transport, warn_throttled, Input, Module, Output};
use dimos_native_costmapper::costmapper::{apply_initial_safe_radius, calculate_costmap, Config};
use lcm_msgs::nav_msgs::OccupancyGrid;
use lcm_msgs::sensor_msgs::PointCloud2;

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

    latest_global_map: Option<PointCloud2>,
    latest_merged_map: Option<PointCloud2>,
}

impl CostMapper {
    async fn on_global_map(&mut self, message: PointCloud2) {
        self.latest_global_map = Some(message);
        let selected = self
            .latest_merged_map
            .as_ref()
            .or(self.latest_global_map.as_ref())
            .expect("global map was just stored");
        self.calculate_and_publish(selected).await;
    }

    async fn on_merged_map(&mut self, message: PointCloud2) {
        self.latest_merged_map = Some(message);
        if self.latest_global_map.is_some() {
            let selected = self
                .latest_merged_map
                .as_ref()
                .expect("merged map was just stored");
            self.calculate_and_publish(selected).await;
        }
    }

    async fn calculate_and_publish(&self, message: &PointCloud2) {
        let mut grid = match calculate_costmap(message, &self.config) {
            Ok(grid) => grid,
            Err(error) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %error,
                    "Cost mapper rejected an input cloud",
                );
                return;
            }
        };
        apply_initial_safe_radius(&mut grid, self.config.initial_safe_radius_meters);
        if let Err(error) = self.global_costmap.publish(&grid).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %error,
                topic = %self.global_costmap.topic,
                "Cost mapper failed to publish",
            );
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<CostMapper>().await;
}
