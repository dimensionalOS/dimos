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
    /// Every this many scans, log the handlers' time and the inputs' age on arrival; 0 never.
    stats_every_scans: u64,
    /// CPUs the process runs on (Linux); empty leaves it to the scheduler.
    cpus: Vec<usize>,
    /// Scan thinning before fusion: every `keep_every`th usable point, then one per `scan_cube` m cube.
    keep_every: u32,
    scan_cube: f32,
    /// Publish every this many poses the filter makes (1 = all), for lidars split into short slices.
    #[validate(range(min = 1))]
    publish_every: u64,
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
    stats: Stats,
    poses: u64,
}

impl DimSlamLio {
    async fn init(&mut self) {
        pin_to(&self.config.cpus);
        let slam = DimSlamCore::new(DimSlamCoreConfig {
            odom_frame_id: self.config.odom_frame_id.clone(),
            output_frame_id: self.config.output_frame_id.clone(),
            imus: self.config.imus.clone(),
            lidars: self.config.lidars.clone(),
            odom_sources: self.config.odom_sources.clone(),
            process_noise: self.config.process_noise,
            keep_every: self.config.keep_every,
            scan_cube: self.config.scan_cube,
            ..Default::default()
        });
        self.slam = Some(slam.unwrap_or_else(|error| panic!("{error}")));
    }

    async fn handle_lidar(&mut self, msg: PointCloud2) {
        let Some(scan) = to_lidar_scan(&msg) else {
            tracing::warn!(frame = %msg.header.frame_id, "cloud dropped: no little-endian float32 x/y/z");
            return;
        };
        let (start, age) = (std::time::Instant::now(), age_s(scan.timestamp_ns));
        let Self { slam, tf, .. } = self;
        slam.as_mut()
            .expect("setup ran")
            .handle_cloud(&scan, &tf_lookup(tf));
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
        slam.as_mut()
            .expect("setup ran")
            .handle_imu(&sample, &tf_lookup(tf));
        self.stats.imus += 1;
        self.stats.imu_s += start.elapsed().as_secs_f64();
        self.stats.imu_age_s += age;
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
        self.poses += 1;
        if self.poses % self.config.publish_every != 0 {
            return;
        }
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
    run_with_transport::<DimSlamLio>().await;
}
