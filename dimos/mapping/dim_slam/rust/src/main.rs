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

mod msg_convert;

use dim_slam::{
    CameraConfig, CuvslamCore, CuvslamOdometryConfig, FusionCore, ImuConfig, OdometryEstimate,
    OdometryFusionConfig, SourceConfig,
};
use dimos_module::{native_config, run_with_transport, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{CameraInfo, Image, Imu, PointCloud2};
use msg_convert::{
    tf_lookup, to_camera_model, to_estimate, to_image_frame, to_imu_sample, to_isometry,
    to_odometry_msg, to_point_cloud2, to_transform,
};

/// The tracker's own drifting world frame. It never touches the wire, so the name is
/// internal bookkeeping: it only tells the tracker's estimates apart inside the filter.
const VISUAL_ODOM_FRAME_ID: &str = "visual_odom";

#[native_config]
struct DimSlamConfig {
    camera_mode: String,
    /// In cuVSLAM's index order: the rig cameras first (two for stereo, one otherwise), then
    /// any settings-only streams such as an rgbd depth camera. Empty discovers the rig off
    /// camera_info.
    cameras: Vec<CameraConfig>,
    use_gpu: bool,
    rig_frame_id: String,
    map_frame_id: String,
    covariance_gate_translation_std: f64,
    speed_gate_max_linear: f64,
    speed_gate_max_angular: f64,
    max_skew_ms: f64,

    odom_frame_id: String,
    output_frame_id: String,
    publish_tf: bool,
    publish_rate: f64,
    replay_buffer_seconds: f64,
    outlier_rejection_allowed_variance: f64,
    max_position_m: f64,
    /// `frame_id` empty disables the IMU; otherwise it names the frame the samples carry.
    imu: ImuConfig,
    initial_gravity_estimate: f64,
    initial_position_std: f64,
    initial_velocity_std: f64,
    initial_rotation_std: f64,
    initial_bias_std: f64,
    /// Trust in the in-process visual odometry source.
    visual_odom_pose_variances: [f64; 6],
    visual_odom_twist_variances: [f64; 6],
    /// External sources, each identified by the transform its estimates carry.
    odom_sources: Vec<SourceConfig>,
    constraint_twist_variances: [f64; 6],
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
    /// External odometry sources, told apart by the transform each message carries.
    #[input(decode = Odometry::decode)]
    odom_sources: Input<Odometry>,
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
        let vo = CuvslamCore::new(CuvslamOdometryConfig {
            camera_mode: self.config.camera_mode.clone(),
            cameras: self.config.cameras.clone(),
            use_gpu: self.config.use_gpu,
            odom_frame_id: VISUAL_ODOM_FRAME_ID.to_string(),
            output_frame_id: self.config.output_frame_id.clone(),
            rig_frame_id: self.config.rig_frame_id.clone(),
            map_frame_id: self.config.map_frame_id.clone(),
            covariance_gate_translation_std: self.config.covariance_gate_translation_std,
            speed_gate_max_linear: self.config.speed_gate_max_linear,
            speed_gate_max_angular: self.config.speed_gate_max_angular,
            // The fusion filter owns the IMU; cuVSLAM's own inertial mode stays off.
            enable_imu: false,
            max_skew_ms: self.config.max_skew_ms,
        });
        self.vo = Some(vo.unwrap_or_else(|error| panic!("{error}")));
        // The in-process tracker is always a fusion source; it goes first so a no-IMU blend
        // prefers it.
        let mut odom_sources = vec![SourceConfig {
            parent_frame_id: VISUAL_ODOM_FRAME_ID.to_string(),
            child_frame_id: self.config.output_frame_id.clone(),
            pose_variances: self.config.visual_odom_pose_variances,
            twist_variances: self.config.visual_odom_twist_variances,
        }];
        odom_sources.extend(self.config.odom_sources.iter().cloned());
        self.fusion = Some(FusionCore::new(OdometryFusionConfig {
            odom_frame_id: self.config.odom_frame_id.clone(),
            output_frame_id: self.config.output_frame_id.clone(),
            publish_rate: self.config.publish_rate,
            replay_buffer_seconds: self.config.replay_buffer_seconds,
            outlier_rejection_allowed_variance: self.config.outlier_rejection_allowed_variance,
            max_position_m: self.config.max_position_m,
            imu: Some(self.config.imu.clone()).filter(|imu| !imu.frame_id.is_empty()),
            initial_gravity_estimate: self.config.initial_gravity_estimate,
            initial_position_std: self.config.initial_position_std,
            initial_velocity_std: self.config.initial_velocity_std,
            initial_rotation_std: self.config.initial_rotation_std,
            initial_bias_std: self.config.initial_bias_std,
            odom_sources,
            constraint_twist_variances: self.config.constraint_twist_variances,
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
        if self.config.imu.frame_id.is_empty() {
            return;
        }
        let sample = to_imu_sample(&msg);
        let Some(base_from_imu) = self
            .tf
            .get_latest(&self.config.output_frame_id, &msg.header.frame_id)
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

    async fn handle_odom_sources(&mut self, msg: Odometry) {
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
