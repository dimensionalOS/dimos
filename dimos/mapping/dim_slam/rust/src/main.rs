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
    CameraConfig, DeskewConfig, DimSlamCore, DimSlamCoreConfig, ImuConfig, InitialStds,
    LidarConfig, ProcessNoise, SourceConfig, SourceKind,
};
use dimos_module::{native_config, run_with_transport, Input, Module, Output, Tf};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{CameraInfo, Image, Imu, PointCloud2};
use msg_convert::{
    tf_lookup, to_camera_model, to_deskewed_cloud_msg, to_estimate, to_image_frame, to_imu_sample,
    to_lidar_scan, to_odometry_msg, to_point_cloud2, to_transform,
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
    still_variance: f64,
    still_speed: f64,
    still_angular_speed: f64,
}

/// The local map lidar scans are matched against.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct MapConfig {
    spacing: f64,
    cube: f32,
    capacity: usize,
    seed_points: usize,
    grow_min_points: usize,
    /// Neighbors from the full search in its original order; off: a pruned search finding the same neighbors.
    unpruned_neighbor_order: bool,
}

impl MapConfig {
    fn core(&self) -> dim_slam::MapConfig {
        dim_slam::MapConfig {
            spacing: self.spacing,
            cube: self.cube,
            capacity: self.capacity,
            seed_points: self.seed_points,
            grow_min_points: self.grow_min_points,
            point_lio_neighbor_order: self.unpruned_neighbor_order,
        }
    }
}

#[native_config]
struct DimSlamConfig {
    /// Empty runs no cameras: lidar, IMU and odometry sources only.
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
    /// Trust in the in-process visual odometry source.
    visual_odom_pose_variances: Covariance,

    odom_frame_id: String,
    output_frame_id: String,
    publish_tf: bool,
    /// The first drives the filter between scans and levels it.
    imus: Vec<ImuConfig>,
    lidars: Vec<LidarConfig>,
    /// External sources, each identified by the transform its estimates carry.
    odom_sources: Vec<OdomSourceConfig>,
    process_noise: ProcessNoise,
    standard_gravity: f64,
    initial_gravity_estimate: f64,
    initial_stds: InitialStds,
    map: MapConfig,
    outlier_rejection_allowed_variance: f64,
    max_position_m: f64,
    per_dimension_error_variance: Covariance,
    /// Publish every this many poses the filter makes (1 = all), for lidars split into short slices.
    #[validate(range(min = 1))]
    publish_every: u32,
    warn_throttle_s: f64,
    /// Each scan deskewed onto `deskewed_cloud`.
    deskew: DeskewConfig,
    /// Every this many scans, log the handlers' time and the inputs' age on arrival; 0 never.
    stats_every_scans: u64,
    /// CPUs the process runs on (Linux); empty leaves it to the scheduler.
    cpus: Vec<usize>,
}

impl DimSlamConfig {
    fn core(&self) -> DimSlamCoreConfig {
        let defaults = DimSlamCoreConfig::default();
        DimSlamCoreConfig {
            camera_mode: if self.camera_mode.is_empty() {
                defaults.camera_mode
            } else {
                self.camera_mode.clone()
            },
            cameras: self.cameras.clone(),
            use_gpu: self.use_gpu,
            rig_frame_id: self.rig_frame_id.clone(),
            map_frame_id: self.map_frame_id.clone(),
            covariance_gate_translation_std: self.covariance_gate_translation_std,
            speed_gate_max_linear: self.speed_gate_max_linear,
            speed_gate_max_angular: self.speed_gate_max_angular,
            max_skew_ms: self.max_skew_ms,
            visual_odom_pose_variances: self.visual_odom_pose_variances.to_array(),
            odom_frame_id: self.odom_frame_id.clone(),
            output_frame_id: self.output_frame_id.clone(),
            imus: self.imus.clone(),
            lidars: self.lidars.clone(),
            odom_sources: self
                .odom_sources
                .iter()
                .map(|source| SourceConfig {
                    parent_frame_id: source.parent_frame_id.clone(),
                    child_frame_id: source.child_frame_id.clone(),
                    kind: source.kind,
                    pose_variances: source.pose_variances.to_array(),
                    twist_variances: source.twist_variances.to_array(),
                    still_variance: source.still_variance,
                    still_speed: source.still_speed,
                    still_angular_speed: source.still_angular_speed,
                })
                .collect(),
            process_noise: self.process_noise,
            standard_gravity: self.standard_gravity,
            initial_gravity_estimate: self.initial_gravity_estimate,
            initial_stds: self.initial_stds,
            map: self.map.core(),
            outlier_rejection_allowed_variance: self.outlier_rejection_allowed_variance,
            max_position_m: self.max_position_m,
            per_dimension_error_variance: self.per_dimension_error_variance.to_array(),
            publish_every: self.publish_every,
            warn_throttle_s: self.warn_throttle_s,
            deskew: self.deskew.clone(),
        }
    }
}

/// What `stats_every_scans` reports: handler seconds and arrival ages, per input, since the last report.
#[derive(Default)]
struct Stats {
    scans: u64,
    scan_s: f64,
    scan_max_s: f64,
    scan_age_s: f64,
    imus: u64,
    imu_s: f64,
    imu_age_s: f64,
    /// s, each scan's age when it arrived and each published pose's age as it goes out (its stamp is its scan's end).
    scan_ages: Vec<f64>,
    pose_ages: Vec<f64>,
}

/// The `q` quantile of `values` (sorted in place), in ms.
fn quantile_ms(values: &mut [f64], q: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    1e3 * values[((values.len() - 1) as f64 * q).round() as usize]
}

#[derive(Module)]
#[module(setup = init, teardown = report)]
struct DimSlam {
    #[input(decode = CameraInfo::decode)]
    camera_info: Input<CameraInfo>,
    /// Every camera, told apart by frame_id.
    #[input(decode = Image::decode)]
    image: Input<Image>,
    /// Feeds depth_cloud, and is tracked against in rgbd.
    #[input(decode = Image::decode)]
    depth_image: Input<Image>,
    /// Only needed when depth has to be reprojected onto the rig camera.
    #[input(decode = CameraInfo::decode)]
    depth_camera_info: Input<CameraInfo>,
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
    /// Range-gated depth points, in the depth frame.
    #[output(encode = PointCloud2::encode)]
    depth_cloud: Output<PointCloud2>,
    #[output(encode = PointCloud2::encode)]
    deskewed_cloud: Output<PointCloud2>,
    #[tf]
    tf: Tf,
    #[config]
    config: DimSlamConfig,

    slam: Option<DimSlamCore>,
    stats: Stats,
}

impl DimSlam {
    async fn init(&mut self) {
        pin_to(&self.config.cpus);
        let slam = DimSlamCore::new(self.config.core());
        match slam {
            Ok(slam) => self.slam = Some(slam),
            Err(error) => {
                tracing::error!(%error, "dim_slam config refused; every input is dropped")
            }
        }
    }

    async fn handle_camera_info(&mut self, info: CameraInfo) {
        if !self.cameras_on(&info.header.frame_id) {
            return;
        }
        let Self { slam, tf, .. } = self;
        if let Some(slam) = slam.as_mut() {
            slam.handle_camera_info(to_camera_model(info), &tf_lookup(tf));
        }
    }

    async fn handle_image(&mut self, img: Image) {
        if !self.cameras_on(&img.header.frame_id) {
            return;
        }
        let Self { slam, tf, .. } = self;
        if let Some(slam) = slam.as_mut() {
            slam.handle_image(to_image_frame(img), &tf_lookup(tf));
        }
        self.publish().await;
    }

    async fn handle_depth_image(&mut self, img: Image) {
        if !self.cameras_on(&img.header.frame_id) {
            return;
        }
        let Self { slam, tf, .. } = self;
        let Some(slam) = slam.as_mut() else {
            return;
        };
        if let Some(cloud) = slam.handle_depth_image(to_image_frame(img), &tf_lookup(tf)) {
            self.depth_cloud
                .publish(&to_point_cloud2(&cloud))
                .await
                .ok();
        }
        self.publish().await;
    }

    async fn handle_depth_camera_info(&mut self, info: CameraInfo) {
        if !self.cameras_on(&info.header.frame_id) {
            return;
        }
        if let Some(slam) = self.slam.as_mut() {
            slam.handle_depth_camera_info(to_camera_model(info));
        }
    }

    /// With no camera_mode a camera's messages are dropped, with a warning per frame.
    fn cameras_on(&self, frame_id: &str) -> bool {
        if !self.config.camera_mode.is_empty() {
            return true;
        }
        dim_slam::warn_throttled!(
            key = (frame_id,);
            frame = %frame_id,
            "camera message dropped: camera_mode is not set, so no camera is tracked",
        );
        false
    }

    async fn handle_lidar(&mut self, msg: PointCloud2) {
        let Some(scan) = to_lidar_scan(&msg) else {
            tracing::warn!(frame = %msg.header.frame_id, "cloud dropped: no little-endian float32 x/y/z");
            return;
        };
        let (start, age) = (std::time::Instant::now(), age_s(scan.timestamp_ns));
        let Self { slam, tf, .. } = self;
        let Some(slam) = slam.as_mut() else {
            return;
        };
        slam.handle_cloud(&scan, &tf_lookup(tf));
        let took = start.elapsed().as_secs_f64();
        let stats = &mut self.stats;
        stats.scans += 1;
        stats.scan_s += took;
        stats.scan_max_s = stats.scan_max_s.max(took);
        stats.scan_age_s += age;
        stats.scan_ages.push(age);
        if self.config.stats_every_scans > 0 && stats.scans >= self.config.stats_every_scans {
            self.log_stats();
        }
        self.publish().await;
    }

    async fn handle_imu(&mut self, msg: Imu) {
        let sample = to_imu_sample(&msg);
        let (start, age) = (std::time::Instant::now(), age_s(sample.timestamp_ns));
        let Self { slam, tf, .. } = self;
        let Some(slam) = slam.as_mut() else {
            return;
        };
        slam.handle_imu(&sample, &tf_lookup(tf));
        self.stats.imus += 1;
        self.stats.imu_s += start.elapsed().as_secs_f64();
        self.stats.imu_age_s += age;
        self.publish().await;
    }

    async fn handle_odom_sources(&mut self, msg: Odometry) {
        let Self { slam, tf, .. } = self;
        let Some(slam) = slam.as_mut() else {
            return;
        };
        slam.handle_source(&to_estimate(&msg), &tf_lookup(tf));
        self.publish().await;
    }

    async fn publish(&mut self) {
        let Some(slam) = self.slam.as_mut() else {
            return;
        };
        for deskewed in slam.take_deskewed() {
            self.deskewed_cloud
                .publish(&to_deskewed_cloud_msg(&deskewed))
                .await
                .ok();
        }
        let Some(estimate) = slam.maybe_publish() else {
            return;
        };
        self.stats.pose_ages.push(age_s(estimate.timestamp_ns));
        self.odometry
            .publish(&to_odometry_msg(&estimate))
            .await
            .ok();
        if self.config.publish_tf {
            self.tf.publish(&[to_transform(&estimate)]).await.ok();
        }
    }

    fn log_stats(&mut self) {
        let mut stats = std::mem::take(&mut self.stats);
        let (scans, imus) = (stats.scans as f64, stats.imus.max(1) as f64);
        let (scan_ages, pose_ages) = (&mut stats.scan_ages, &mut stats.pose_ages);
        tracing::info!(
            scans = stats.scans,
            scan_ms_mean = 1e3 * stats.scan_s / scans,
            scan_ms_max = 1e3 * stats.scan_max_s,
            scan_age_ms_mean = 1e3 * stats.scan_age_s / scans,
            imus = stats.imus,
            imu_ms_mean = 1e3 * stats.imu_s / imus,
            imu_age_ms_mean = 1e3 * stats.imu_age_s / imus,
            scan_age_ms_p50 = quantile_ms(scan_ages, 0.5),
            scan_age_ms_p95 = quantile_ms(scan_ages, 0.95),
            scan_age_ms_max = quantile_ms(scan_ages, 1.0),
            poses_published = pose_ages.len(),
            pose_age_ms_p50 = quantile_ms(pose_ages, 0.5),
            pose_age_ms_p95 = quantile_ms(pose_ages, 0.95),
            pose_age_ms_max = quantile_ms(pose_ages, 1.0),
            "input stats"
        );
        if let Some(slam) = &self.slam {
            slam.report();
        }
    }

    async fn report(&mut self) {
        if let Some(slam) = &self.slam {
            slam.report();
        }
    }
}

/// Every thread this process starts from here on inherits the set (the runtime's workers too, as they spawn lazily).
#[cfg(target_os = "linux")]
fn pin_to(cpus: &[usize]) {
    if cpus.is_empty() {
        return;
    }
    // SAFETY: a zeroed cpu_set_t is the empty set; CPU_SET writes within it; pid 0 is this thread.
    let result = unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        for &cpu in cpus {
            libc::CPU_SET(cpu, &mut set);
        }
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set)
    };
    if result == 0 {
        tracing::info!(?cpus, "pinned");
    } else {
        tracing::warn!(?cpus, error = %std::io::Error::last_os_error(), "could not pin");
    }
}

#[cfg(not(target_os = "linux"))]
fn pin_to(cpus: &[usize]) {
    if !cpus.is_empty() {
        tracing::warn!(?cpus, "cpus is Linux-only; ignored");
    }
}

/// Seconds from a message's stamp to now on this host's clock (the stamps are the lidar's PTP clock).
fn age_s(stamp_ns: i64) -> f64 {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64());
    now - stamp_ns as f64 / 1e9
}

#[tokio::main]
async fn main() {
    run_with_transport::<DimSlam>().await;
}
