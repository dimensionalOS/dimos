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

mod msg_convert;

use dim_slam::{
    DimSlamCore, DimSlamCoreConfig, ImuConfig, LidarConfig, ProcessNoise, SourceConfig,
};
use dimos_module::{native_config, run_with_transport, Input, Module, Output, Tf};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{Imu, PointCloud2};
use msg_convert::{
    tf_lookup, to_estimate, to_imu_sample, to_lidar_scan, to_odometry_msg, to_transform,
};

#[native_config]
struct DimSlamLioConfig {
    odom_frame_id: String,
    output_frame_id: String,
    publish_tf: bool,
    /// The first drives the filter between scans and levels it.
    imus: Vec<ImuConfig>,
    lidars: Vec<LidarConfig>,
    /// External sources, each identified by the transform its estimates carry.
    odom_sources: Vec<SourceConfig>,
    process_noise: ProcessNoise,
}

#[derive(Module)]
#[module(setup = init, teardown = report)]
struct DimSlamLio {
    /// Every lidar, told apart by frame_id.
    #[input(decode = PointCloud2::decode)]
    lidar: Input<PointCloud2>,
    /// Every IMU, told apart by frame_id.
    #[input(decode = Imu::decode)]
    imu: Input<Imu>,
    #[input(decode = Odometry::decode)]
    odom_sources: Input<Odometry>,
    #[output(encode = Odometry::encode)]
    odometry: Output<Odometry>,
    #[tf]
    tf: Tf,
    #[config]
    config: DimSlamLioConfig,

    slam: Option<DimSlamCore>,
}

impl DimSlamLio {
    async fn init(&mut self) {
        let slam = DimSlamCore::new(DimSlamCoreConfig {
            odom_frame_id: self.config.odom_frame_id.clone(),
            output_frame_id: self.config.output_frame_id.clone(),
            imus: self.config.imus.clone(),
            lidars: self.config.lidars.clone(),
            odom_sources: self.config.odom_sources.clone(),
            process_noise: self.config.process_noise,
            ..Default::default()
        });
        self.slam = Some(slam.unwrap_or_else(|error| panic!("{error}")));
    }

    async fn handle_lidar(&mut self, msg: PointCloud2) {
        let Some(scan) = to_lidar_scan(&msg) else {
            tracing::warn!(frame = %msg.header.frame_id, "cloud dropped: no little-endian float32 x/y/z");
            return;
        };
        let Self { slam, tf, .. } = self;
        slam.as_mut()
            .expect("setup ran")
            .handle_cloud(&scan, &tf_lookup(tf));
        self.publish().await;
    }

    async fn handle_imu(&mut self, msg: Imu) {
        let Self { slam, tf, .. } = self;
        slam.as_mut()
            .expect("setup ran")
            .handle_imu(&to_imu_sample(&msg), &tf_lookup(tf));
        self.publish().await;
    }

    async fn handle_odom_sources(&mut self, msg: Odometry) {
        let Self { slam, tf, .. } = self;
        slam.as_mut()
            .expect("setup ran")
            .handle_source(&to_estimate(&msg), &tf_lookup(tf));
        self.publish().await;
    }

    async fn publish(&mut self) {
        let Some(estimate) = self.slam.as_mut().expect("setup ran").maybe_publish() else {
            return;
        };
        self.odometry
            .publish(&to_odometry_msg(&estimate))
            .await
            .ok();
        if self.config.publish_tf {
            self.tf.publish(&[to_transform(&estimate)]).await.ok();
        }
    }

    async fn report(&mut self) {
        if let Some(slam) = &self.slam {
            slam.report();
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<DimSlamLio>().await;
}
