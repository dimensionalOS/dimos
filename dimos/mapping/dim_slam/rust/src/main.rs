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

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dim_slam::{
    CameraConfig, DimSlamCore, DimSlamCoreConfig, ImuConfig, InitialStds, ProcessNoise,
    SourceConfig, SourceKind,
};
use dimos_module::{native_config, run_with_transport, Input, Module, Output, Tf};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{CameraInfo, Image, Imu, PointCloud2};
use msg_convert::{
    tf_lookup, to_camera_model, to_estimate, to_image_frame, to_imu_sample, to_odometry_msg,
    to_point_cloud2, to_transform,
};
use serde::{Deserialize, Serialize};

/// Per-axis variances. For a pose x/y/z are metres and roll/pitch/yaw radians; for a twist
/// they are the linear (m/s) and angular (rad/s) rates about the same axes.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
struct Covariance {
    x: f64,
    y: f64,
    z: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

impl Covariance {
    fn to_array(&self) -> [f64; 6] {
        [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
    }
}

/// One external odometry source, identified by the transform its estimates carry.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
struct OdomSourceConfig {
    parent_frame_id: String,
    child_frame_id: String,
    kind: SourceKind,
    pose_variances: Covariance,
    twist_variances: Covariance,
    realign_after_seconds: f64,
    still_variance: f64,
    still_speed: f64,
    still_angular_speed: f64,
}

/// One IMU, identified by the frame its samples carry.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
struct ImuSourceConfig {
    frame_id: String,
    gyro_noise_density: f64,
    gyro_random_walk: f64,
    accel_noise_density: f64,
    accel_random_walk: f64,
    init_samples: i64,
    init_gyro_limit: f64,
}

/// Standard deviations seeding the filter covariance at initialization.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
struct InitialStdsConfig {
    position: f64,
    velocity: f64,
    rotation: f64,
    angular_velocity: f64,
    bias: f64,
}

#[native_config]
struct DimSlamConfig {
    camera_mode: String,
    /// In cuVSLAM's index order: the rig cameras first (two for stereo, the whole list for
    /// multisensor, one otherwise), then any settings-only streams such as an rgbd depth
    /// camera. Empty discovers the rig off camera_info; multisensor requires the list.
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
    /// Empty disables inertial propagation; each entry keeps its own noise figures and biases.
    imus: Vec<ImuSourceConfig>,
    initial_gravity_estimate: f64,
    initial_stds: InitialStdsConfig,
    process_noise: ProcessNoise,
    /// Trust in the in-process visual odometry source.
    visual_odom_pose_variances: Covariance,
    /// External sources, each identified by the transform its estimates carry.
    odom_sources: Vec<OdomSourceConfig>,
    planar: bool,
    zero_velocity_variances: Covariance,
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

    slam: Option<Arc<Mutex<Slam>>>,
}

/// The core plus the newest input stamp and when it arrived: the timer runs on the inputs' clock, live or replayed.
struct Slam {
    core: DimSlamCore,
    newest_stamp: Option<(i64, Instant)>,
}

impl Slam {
    fn saw(&mut self, timestamp_ns: i64) {
        if self
            .newest_stamp
            .is_none_or(|(newest, _)| timestamp_ns > newest)
        {
            self.newest_stamp = Some((timestamp_ns, Instant::now()));
        }
    }

    fn data_now_ns(&self) -> Option<i64> {
        let (stamp, arrived) = self.newest_stamp?;
        Some(stamp + arrived.elapsed().as_nanos() as i64)
    }
}

impl DimSlam {
    async fn init(&mut self) {
        let slam = DimSlamCore::new(DimSlamCoreConfig {
            camera_mode: self.config.camera_mode.clone(),
            cameras: self.config.cameras.clone(),
            use_gpu: self.config.use_gpu,
            rig_frame_id: self.config.rig_frame_id.clone(),
            map_frame_id: self.config.map_frame_id.clone(),
            covariance_gate_translation_std: self.config.covariance_gate_translation_std,
            speed_gate_max_linear: self.config.speed_gate_max_linear,
            speed_gate_max_angular: self.config.speed_gate_max_angular,
            max_skew_ms: self.config.max_skew_ms,
            visual_odom_pose_variances: self.config.visual_odom_pose_variances.to_array(),
            odom_frame_id: self.config.odom_frame_id.clone(),
            output_frame_id: self.config.output_frame_id.clone(),
            publish_rate: self.config.publish_rate,
            replay_buffer_seconds: self.config.replay_buffer_seconds,
            outlier_rejection_allowed_variance: self.config.outlier_rejection_allowed_variance,
            max_position_m: self.config.max_position_m,
            imus: self
                .config
                .imus
                .iter()
                .map(|imu| ImuConfig {
                    frame_id: imu.frame_id.clone(),
                    gyro_noise_density: imu.gyro_noise_density,
                    gyro_random_walk: imu.gyro_random_walk,
                    accel_noise_density: imu.accel_noise_density,
                    accel_random_walk: imu.accel_random_walk,
                    init_samples: imu.init_samples,
                    init_gyro_limit: imu.init_gyro_limit,
                })
                .collect(),
            initial_gravity_estimate: self.config.initial_gravity_estimate,
            initial_stds: InitialStds {
                position: self.config.initial_stds.position,
                velocity: self.config.initial_stds.velocity,
                rotation: self.config.initial_stds.rotation,
                angular_velocity: self.config.initial_stds.angular_velocity,
                bias: self.config.initial_stds.bias,
            },
            process_noise: self.config.process_noise,
            odom_sources: self
                .config
                .odom_sources
                .iter()
                .map(|source| SourceConfig {
                    parent_frame_id: source.parent_frame_id.clone(),
                    child_frame_id: source.child_frame_id.clone(),
                    kind: source.kind,
                    pose_variances: source.pose_variances.to_array(),
                    twist_variances: source.twist_variances.to_array(),
                    realign_after_seconds: source.realign_after_seconds,
                    still_variance: source.still_variance,
                    still_speed: source.still_speed,
                    still_angular_speed: source.still_angular_speed,
                })
                .collect(),
            planar: self.config.planar,
            zero_velocity_variances: self.config.zero_velocity_variances.to_array(),
        });
        let slam = Arc::new(Mutex::new(Slam {
            core: slam.unwrap_or_else(|error| panic!("{error}")),
            newest_stamp: None,
        }));
        self.slam = Some(Arc::clone(&slam));
        let (odometry, tf, publish_tf) = (
            self.odometry.clone(),
            self.tf.clone(),
            self.config.publish_tf,
        );
        let mut ticks = tokio::time::interval(Duration::from_secs_f64(
            1.0 / self.config.publish_rate.max(1.0),
        ));
        tokio::spawn(async move {
            loop {
                ticks.tick().await;
                let estimate = {
                    let mut slam = slam.lock().expect("slam lock");
                    let Some(now_ns) = slam.data_now_ns() else {
                        continue;
                    };
                    slam.core.maybe_publish_at(now_ns)
                };
                let Some(estimate) = estimate else { continue };
                odometry.publish(&to_odometry_msg(&estimate)).await.ok();
                if publish_tf {
                    tf.publish(&[to_transform(&estimate)]).await.ok();
                }
            }
        });
    }

    fn slam(&self) -> std::sync::MutexGuard<'_, Slam> {
        self.slam
            .as_ref()
            .expect("setup ran")
            .lock()
            .expect("slam lock")
    }

    async fn handle_camera_info(&mut self, info: CameraInfo) {
        let lookup = tf_lookup(&self.tf);
        self.slam()
            .core
            .handle_camera_info(to_camera_model(info), &lookup);
    }

    async fn handle_image(&mut self, img: Image) {
        let lookup = tf_lookup(&self.tf);
        let frame = to_image_frame(img);
        let mut slam = self.slam();
        slam.saw(frame.timestamp_ns);
        slam.core.handle_image(frame, &lookup);
    }

    async fn handle_depth_image(&mut self, img: Image) {
        let lookup = tf_lookup(&self.tf);
        let frame = to_image_frame(img);
        let cloud = {
            let mut slam = self.slam();
            slam.saw(frame.timestamp_ns);
            slam.core.handle_depth_image(frame, &lookup)
        };
        if let Some(cloud) = cloud {
            self.depth_cloud
                .publish(&to_point_cloud2(&cloud))
                .await
                .ok();
        }
    }

    async fn handle_depth_camera_info(&mut self, info: CameraInfo) {
        self.slam()
            .core
            .handle_depth_camera_info(to_camera_model(info));
    }

    async fn handle_imu(&mut self, msg: Imu) {
        let lookup = tf_lookup(&self.tf);
        let sample = to_imu_sample(&msg);
        let mut slam = self.slam();
        slam.saw(sample.timestamp_ns);
        slam.core.handle_imu(&sample, &lookup);
    }

    async fn handle_odom_sources(&mut self, msg: Odometry) {
        let estimate = to_estimate(&msg);
        let mut slam = self.slam();
        slam.saw(estimate.timestamp_ns);
        slam.core.handle_source(&estimate);
    }

    async fn report(&mut self) {
        if self.slam.is_some() {
            self.slam().core.report();
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<DimSlam>().await;
}
