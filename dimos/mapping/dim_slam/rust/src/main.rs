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
// cuVSLAM visual odometry enters the fusion filter in-process as a drifting source, so its
// pose never touches the wire.

mod imu_info;
mod msg_convert;

use std::collections::BTreeMap;

use dim_slam::nalgebra::Isometry3;
use dim_slam::{
    CuvslamCore, CuvslamOdometryConfig, FusionCore, OdometryEstimate, OdometryFusionConfig,
    SourceConfig,
};
use dimos_module::{native_config, run_with_transport, warn_throttled, Input, Module, Output, Tf};
use imu_info::ImuInfo;
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{CameraInfo, Image, Imu, PointCloud2};
use msg_convert::{
    to_camera_model, to_estimate, to_image_frame, to_imu_sample, to_isometry, to_noise_model,
    to_odometry_msg, to_point_cloud2, to_transform,
};

#[native_config]
struct DimSlamConfig {
    camera_mode: String,
    camera_frames: Vec<String>,
    rectified: bool,
    use_gpu: bool,
    /// Fused as a drifting source, so it must also appear in source_frames.
    visual_odom_frame: String,
    rig_frame: String,
    covariance_gate_translation_std: f64,
    speed_gate_max_linear: f64,
    speed_gate_max_angular: f64,
    /// cuVSLAM's own inertial mode, separate from the filter's use_imu.
    cuvslam_enable_imu: bool,
    depth_units_per_meter: f64,
    depth_cloud_min_range: f64,
    depth_cloud_max_range: f64,
    depth_cloud_decimation: i64,

    odom_frame: String,
    base_frame: String,
    publish_tf: bool,
    publish_rate: f64,
    replay_buffer_seconds: f64,
    mahalanobis_gate: f64,
    max_position_m: f64,
    use_imu: bool,
    imu_gyro_noise_density: f64,
    imu_gyro_random_walk: f64,
    imu_accel_noise_density: f64,
    imu_accel_random_walk: f64,
    gravity: f64,
    imu_init_samples: i64,
    initial_position_std: f64,
    initial_velocity_std: f64,
    initial_rotation_std: f64,
    initial_bias_std: f64,
    source_frames: Vec<String>,
    source_pose_variances: Vec<f64>,
    source_twist_variances: Vec<f64>,
    constraint_twist_variances: Vec<f64>,
}

/// The flat lists are what native_config can express; the library wants them keyed by frame.
fn fusion_sources(config: &DimSlamConfig) -> BTreeMap<String, SourceConfig> {
    assert_eq!(
        config.source_pose_variances.len(),
        config.source_frames.len() * 6,
        "source_pose_variances needs 6 entries per source frame"
    );
    assert_eq!(
        config.source_twist_variances.len(),
        config.source_frames.len() * 6,
        "source_twist_variances needs 6 entries per source frame"
    );
    config
        .source_frames
        .iter()
        .zip(config.source_pose_variances.as_chunks::<6>().0)
        .zip(config.source_twist_variances.as_chunks::<6>().0)
        .map(|((frame, pose), twist)| {
            (
                frame.clone(),
                SourceConfig {
                    pose_variances: *pose,
                    twist_variances: *twist,
                },
            )
        })
        .collect()
}

fn tf_lookup(tf: &Tf) -> impl Fn(&str, &str) -> Option<Isometry3<f64>> + '_ {
    move |parent: &str, child: &str| {
        tf.get_latest(parent, child)
            .map(|transform| to_isometry(&transform))
    }
}

#[derive(Module)]
#[module(setup = init, teardown = report)]
struct DimSlam {
    #[input(decode = CameraInfo::decode)]
    camera_info: Input<CameraInfo>,
    #[input(decode = Image::decode)]
    image: Input<Image>,
    /// rgbd only; unconnected otherwise.
    #[input(decode = Image::decode)]
    depth_image: Input<Image>,
    /// Only needed when depth has to be reprojected onto the rig camera.
    #[input(decode = CameraInfo::decode)]
    depth_camera_info: Input<CameraInfo>,
    #[input(decode = Imu::decode)]
    imu: Input<Imu>,
    /// cuvslam_enable_imu only: the IMU's noise model and frame.
    #[input(decode = ImuInfo::decode)]
    imu_info: Input<ImuInfo>,
    /// External odometry sources, told apart by header.frame_id.
    #[input(decode = Odometry::decode)]
    sources: Input<Odometry>,
    #[output(encode = Odometry::encode)]
    odometry: Output<Odometry>,
    /// rgbd only: range-gated depth points, in the depth frame.
    #[output(encode = PointCloud2::encode)]
    depth_cloud: Output<PointCloud2>,
    #[tf]
    tf: Tf,
    #[config]
    config: DimSlamConfig,

    vo: Option<CuvslamCore>,
    fusion: Option<FusionCore>,
}

impl DimSlam {
    async fn init(&mut self) {
        self.vo = Some(CuvslamCore::new(CuvslamOdometryConfig {
            camera_mode: self.config.camera_mode.clone(),
            camera_frames: self.config.camera_frames.clone(),
            rectified: self.config.rectified,
            use_gpu: self.config.use_gpu,
            odom_frame: self.config.visual_odom_frame.clone(),
            base_frame: self.config.base_frame.clone(),
            rig_frame: self.config.rig_frame.clone(),
            covariance_gate_translation_std: self.config.covariance_gate_translation_std,
            speed_gate_max_linear: self.config.speed_gate_max_linear,
            speed_gate_max_angular: self.config.speed_gate_max_angular,
            enable_imu: self.config.cuvslam_enable_imu,
            depth_units_per_meter: self.config.depth_units_per_meter,
            depth_cloud_min_range: self.config.depth_cloud_min_range,
            depth_cloud_max_range: self.config.depth_cloud_max_range,
            depth_cloud_decimation: self.config.depth_cloud_decimation,
        }));
        self.fusion = Some(FusionCore::new(OdometryFusionConfig {
            odom_frame: self.config.odom_frame.clone(),
            base_frame: self.config.base_frame.clone(),
            publish_rate: self.config.publish_rate,
            replay_buffer_seconds: self.config.replay_buffer_seconds,
            mahalanobis_gate: self.config.mahalanobis_gate,
            max_position_m: self.config.max_position_m,
            use_imu: self.config.use_imu,
            imu_gyro_noise_density: self.config.imu_gyro_noise_density,
            imu_gyro_random_walk: self.config.imu_gyro_random_walk,
            imu_accel_noise_density: self.config.imu_accel_noise_density,
            imu_accel_random_walk: self.config.imu_accel_random_walk,
            gravity: self.config.gravity,
            imu_init_samples: self.config.imu_init_samples,
            initial_position_std: self.config.initial_position_std,
            initial_velocity_std: self.config.initial_velocity_std,
            initial_rotation_std: self.config.initial_rotation_std,
            initial_bias_std: self.config.initial_bias_std,
            sources: fusion_sources(&self.config),
            constraint_twist_variances: self
                .config
                .constraint_twist_variances
                .as_slice()
                .try_into()
                .expect("constraint_twist_variances needs exactly 6 entries"),
        }));
    }

    async fn handle_camera_info(&mut self, info: CameraInfo) {
        let Self { vo, tf, .. } = self;
        vo.as_mut()
            .expect("setup ran")
            .handle_camera_info(to_camera_model(info), &tf_lookup(tf));
    }

    async fn handle_image(&mut self, img: Image) {
        let Self { vo, tf, .. } = self;
        let tracked = vo
            .as_mut()
            .expect("setup ran")
            .handle_image(to_image_frame(img), &tf_lookup(tf));
        self.fuse(tracked).await;
    }

    async fn handle_depth_image(&mut self, img: Image) {
        let Self { vo, tf, .. } = self;
        let (cloud, tracked) = vo
            .as_mut()
            .expect("setup ran")
            .handle_depth_image(to_image_frame(img), &tf_lookup(tf));
        if let Some(cloud) = cloud {
            self.depth_cloud
                .publish(&to_point_cloud2(&cloud))
                .await
                .ok();
        }
        self.fuse(tracked).await;
    }

    async fn handle_depth_camera_info(&mut self, info: CameraInfo) {
        self.vo
            .as_mut()
            .expect("setup ran")
            .handle_depth_camera_info(to_camera_model(info));
    }

    async fn handle_imu(&mut self, msg: Imu) {
        let sample = to_imu_sample(&msg);
        if self.config.cuvslam_enable_imu {
            self.vo.as_mut().expect("setup ran").handle_imu(&sample);
        }
        if !self.config.use_imu {
            return;
        }
        let Some(base_from_imu) = self
            .tf
            .get_latest(&self.config.base_frame, &msg.header.frame_id)
        else {
            warn_throttled!(
                std::time::Duration::from_secs(10),
                imu_frame = %msg.header.frame_id,
                "imu dropped: tf does not place the IMU in the base frame",
            );
            return;
        };
        self.fusion
            .as_mut()
            .expect("setup ran")
            .handle_imu(&sample, &to_isometry(&base_from_imu));
        self.publish().await;
    }

    async fn handle_imu_info(&mut self, info: ImuInfo) {
        let Self { vo, tf, .. } = self;
        vo.as_mut()
            .expect("setup ran")
            .handle_imu_info(to_noise_model(info), &tf_lookup(tf));
    }

    async fn handle_sources(&mut self, msg: Odometry) {
        self.fusion
            .as_mut()
            .expect("setup ran")
            .handle_source(&to_estimate(&msg));
        self.publish().await;
    }

    async fn fuse(&mut self, tracked: Option<OdometryEstimate>) {
        let Some(visual_odometry) = tracked else {
            return;
        };
        self.fusion
            .as_mut()
            .expect("setup ran")
            .handle_source(&visual_odometry);
        self.publish().await;
    }

    async fn publish(&mut self) {
        let Some(estimate) = self.fusion.as_mut().expect("setup ran").maybe_publish() else {
            return;
        };
        self.odometry
            .publish(&to_odometry_msg(&estimate))
            .await
            .ok();
        if !self.config.publish_tf {
            return;
        }
        self.tf.publish(&[to_transform(&estimate)]).await.ok();
    }

    async fn report(&mut self) {
        if let Some(vo) = &self.vo {
            vo.report();
        }
        if let Some(fusion) = &self.fusion {
            fusion.report();
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<DimSlam>().await;
}
