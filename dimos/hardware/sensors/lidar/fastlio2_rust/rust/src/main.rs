// LCM-driven FAST-LIO2 native module.
//
// Consumes a livox lidar PointCloud2 + Imu stream, runs the FAST-LIO2 pipeline,
// and publishes Odometry plus the registered (world-frame) scan.

use std::time::Duration;

use dimos_module::{error_throttled, run, warn_throttled, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::{
    Point as GeomPoint, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{Imu, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use nalgebra::UnitQuaternion;
use serde::Deserialize;

use fastlio_rs::commons::{Config as PipelineConfig, IMUData, Point, PointCloud, SyncPackage, V3D};
use fastlio_rs::lidar_processor::LidarProcessor;
use fastlio_rs::map_builder::{BuilderStatus, MapBuilder};

const POINT_STEP: i32 = 16;
const FLOAT32: u8 = PointField::FLOAT32 as u8;

// Verbose pipeline tracing, gated on the `debug` config flag. Off by default.
macro_rules! debug_log {
    ($cond:expr, $($arg:tt)*) => {
        if $cond {
            tracing::info!(target: "fastlio2_rust_native", $($arg)*);
        }
    };
}

#[derive(Debug, Deserialize)]
struct ModuleConfig {
    #[serde(default = "default_frame_id")]
    frame_id: String,
    #[serde(default = "default_child_frame_id")]
    child_frame_id: String,
    #[serde(default)]
    debug: bool,
    // Path to a standard FAST-LIO YAML; the crate parses it into the pipeline
    // Config. Empty -> Config::default().
    #[serde(default)]
    config_path: String,
    // Reject pose updates whose post-update speed exceeds this (m/s). Overrides
    // whatever the YAML sets. Default ~200 mph.
    #[serde(default = "default_max_velocity")]
    max_velocity: f64,
    // FAST-LIO scan/map downsample voxel sizes. None keeps the YAML value.
    #[serde(default)]
    filter_size_surf: Option<f64>,
    #[serde(default)]
    filter_size_map: Option<f64>,
    // Output publish gating: < 0 disabled, 0 every scan, > 0 throttled to N Hz.
    #[serde(default)]
    map_freq: f64,
    #[serde(default)]
    odom_freq: f64,
    // Publish the registered (world-frame) cloud on `registered_scan`:
    // 0 off, > 0 throttled to N Hz.
    #[serde(default)]
    registered_scan_freq: f64,
}

fn default_frame_id() -> String {
    "odom".to_string()
}

fn default_child_frame_id() -> String {
    "base_link".to_string()
}

fn default_max_velocity() -> f64 {
    89.408
}

#[derive(Module)]
#[module(setup = on_start)]
struct FastLio2Rust {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[input(decode = Imu::decode, handler = on_imu)]
    imu: Input<Imu>,

    #[output(encode = Odometry::encode)]
    odometry: Output<Odometry>,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    registered_scan: Output<PointCloud2>,

    #[config]
    config: ModuleConfig,

    builder: Option<MapBuilder>,
    imu_buffer: Vec<IMUData>,
    // Scan time of the last publish on each output, for freq throttling.
    last_map_pub_sec: Option<f64>,
    last_odom_pub_sec: Option<f64>,
    last_registered_pub_sec: Option<f64>,
}

// Whether a stream gated at `freq` Hz should publish at `now`, given its last
// publish time: freq < 0 never, freq == 0 always, freq > 0 throttled to freq Hz.
fn should_publish(freq: f64, now: f64, last: Option<f64>) -> bool {
    if freq < 0.0 {
        false
    } else if freq == 0.0 {
        true
    } else {
        match last {
            Some(last) => now - last >= 1.0 / freq,
            None => true,
        }
    }
}

impl FastLio2Rust {
    async fn on_start(&mut self) {
        let mut pipeline = if self.config.config_path.is_empty() {
            PipelineConfig::default()
        } else {
            match PipelineConfig::from_yaml_path(&self.config.config_path) {
                Ok(pipeline) => pipeline,
                Err(error) => {
                    tracing::error!(
                        config_path = %self.config.config_path,
                        error = %error,
                        "failed to load FAST-LIO YAML; falling back to defaults"
                    );
                    PipelineConfig::default()
                }
            }
        };
        pipeline.max_velocity = self.config.max_velocity;
        if let Some(value) = self.config.filter_size_surf {
            pipeline.scan_resolution = value;
        }
        if let Some(value) = self.config.filter_size_map {
            pipeline.map_resolution = value;
        }
        self.builder = Some(MapBuilder::new(pipeline));
        tracing::info!(
            frame_id = %self.config.frame_id,
            child_frame_id = %self.config.child_frame_id,
            max_velocity = self.config.max_velocity,
            map_freq = self.config.map_freq,
            debug = self.config.debug,
            "fastlio2_rust initialized"
        );
    }

    async fn on_imu(&mut self, msg: Imu) {
        self.imu_buffer.push(IMUData {
            acc: V3D::new(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
            gyro: V3D::new(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
            time: stamp_to_sec(&msg.header.stamp),
        });
        debug_log!(
            self.config.debug,
            buffered = self.imu_buffer.len(),
            acc = ?[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            gyro = ?[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            stamp = stamp_to_sec(&msg.header.stamp),
            "on_imu"
        );
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        let debug = self.config.debug;
        debug_log!(
            debug,
            width = msg.width,
            height = msg.height,
            point_step = msg.point_step,
            data_bytes = msg.data.len(),
            fields = ?msg.fields.iter().map(|field| field.name.as_str()).collect::<Vec<_>>(),
            imu_buffered = self.imu_buffer.len(),
            "on_lidar: received scan"
        );

        // The pipeline needs imu samples spanning the scan before it can start.
        if self.imu_buffer.is_empty() {
            debug_log!(debug, "on_lidar: skipped, imu buffer empty");
            return;
        }

        let cloud = match extract_cloud(&msg) {
            Ok(cloud) => cloud,
            Err(error) => {
                warn_throttled!(Duration::from_secs(1), error = %error, "dropped a lidar scan");
                debug_log!(debug, error = %error, "on_lidar: extract_cloud failed");
                return;
            }
        };
        if cloud.is_empty() {
            debug_log!(debug, "on_lidar: skipped, extracted cloud empty");
            return;
        }

        // Per-point curvature holds the ms offset from the scan start (the "time"
        // field). The largest offset is the scan end; it collapses to the start
        // when the source carries no per-point time, disabling undistortion.
        let stamp_sec = stamp_to_sec(&msg.header.stamp);
        let max_offset_sec = cloud
            .iter()
            .fold(0.0f32, |acc, point| acc.max(point.curvature))
            as f64
            / 1000.0;
        debug_log!(
            debug,
            extracted_points = cloud.len(),
            imus = self.imu_buffer.len(),
            stamp_sec,
            max_offset_sec,
            "on_lidar: extracted cloud"
        );
        let mut package = SyncPackage {
            imus: std::mem::take(&mut self.imu_buffer),
            cloud,
            cloud_start_time: stamp_sec,
            cloud_end_time: stamp_sec + max_offset_sec,
        };

        let builder = self.builder.as_mut().expect("builder set in on_start");
        builder.process(&mut package);
        let status = builder.status();
        debug_log!(debug, status = ?status, "on_lidar: builder processed");

        if status != BuilderStatus::Mapping {
            debug_log!(debug, status = ?status, "on_lidar: not publishing, builder not in Mapping state");
            return;
        }

        if should_publish(self.config.odom_freq, stamp_sec, self.last_odom_pub_sec) {
            let odometry = build_odometry(builder, &self.config, msg.header.stamp.clone());
            let position = &odometry.pose.pose.position;
            debug_log!(
                debug,
                position = ?[position.x, position.y, position.z],
                "on_lidar: publishing odometry"
            );
            match self.odometry.publish(&odometry).await {
                Ok(()) => self.last_odom_pub_sec = Some(stamp_sec),
                Err(error) => {
                    error_throttled!(Duration::from_secs(1), error = %error, "odometry publish failed");
                    debug_log!(debug, error = %error, "on_lidar: odometry publish failed");
                }
            }
        } else {
            debug_log!(
                debug,
                odom_freq = self.config.odom_freq,
                "on_lidar: skipping odometry (gated by odom_freq)"
            );
        }

        // Both global_map and registered_scan carry the same per-scan registered
        // (world-frame) cloud; build it once if either output wants it.
        let want_map = should_publish(self.config.map_freq, stamp_sec, self.last_map_pub_sec);
        // registered_scan_freq differs from map/odom: 0 is off (not "every scan").
        let want_registered = self.config.registered_scan_freq > 0.0
            && should_publish(
                self.config.registered_scan_freq,
                stamp_sec,
                self.last_registered_pub_sec,
            );
        if !want_map && !want_registered {
            debug_log!(
                debug,
                map_freq = self.config.map_freq,
                "on_lidar: skipping registered cloud (no output enabled)"
            );
            return;
        }

        let world = register_cloud(builder, &package.cloud);
        let world_msg = build_pointcloud(&world, &self.config.frame_id, msg.header.stamp);

        if want_map {
            debug_log!(
                debug,
                world_points = world.len(),
                "on_lidar: publishing global_map"
            );
            match self.global_map.publish(&world_msg).await {
                Ok(()) => self.last_map_pub_sec = Some(stamp_sec),
                Err(error) => {
                    error_throttled!(Duration::from_secs(1), error = %error, "global_map publish failed");
                    debug_log!(debug, error = %error, "on_lidar: global_map publish failed");
                }
            }
        }

        if want_registered {
            debug_log!(
                debug,
                world_points = world.len(),
                "on_lidar: publishing registered_scan"
            );
            match self.registered_scan.publish(&world_msg).await {
                Ok(()) => self.last_registered_pub_sec = Some(stamp_sec),
                Err(error) => {
                    error_throttled!(Duration::from_secs(1), error = %error, "registered_scan publish failed");
                    debug_log!(debug, error = %error, "on_lidar: registered_scan publish failed");
                }
            }
        }
    }
}

fn stamp_to_sec(stamp: &Time) -> f64 {
    stamp.sec as f64 + stamp.nsec as f64 * 1e-9
}

fn build_odometry(builder: &MapBuilder, config: &ModuleConfig, stamp: Time) -> Odometry {
    let state = &builder.kf.x;
    let translation = state.t_wi;
    let quaternion = UnitQuaternion::from_matrix(&state.r_wi);
    let velocity = state.v;
    Odometry {
        header: Header {
            seq: 0,
            stamp,
            frame_id: config.frame_id.clone(),
        },
        child_frame_id: config.child_frame_id.clone(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: GeomPoint {
                    x: translation.x,
                    y: translation.y,
                    z: translation.z,
                },
                orientation: Quaternion {
                    x: quaternion.i,
                    y: quaternion.j,
                    z: quaternion.k,
                    w: quaternion.w,
                },
            },
            covariance: [0.0; 36],
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: velocity.x,
                    y: velocity.y,
                    z: velocity.z,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            },
            covariance: [0.0; 36],
        },
    }
}

fn register_cloud(builder: &MapBuilder, body_cloud: &PointCloud) -> PointCloud {
    let r_wl = builder.lidar_processor.r_wl(&builder.kf);
    let t_wl = builder.lidar_processor.t_wl(&builder.kf);
    LidarProcessor::transform_cloud(body_cloud, &r_wl, &t_wl)
}

#[derive(Debug)]
struct ExtractError(&'static str);

impl std::fmt::Display for ExtractError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(self.0)
    }
}

fn extract_cloud(msg: &PointCloud2) -> Result<PointCloud, ExtractError> {
    if msg.is_bigendian {
        return Err(ExtractError("big-endian point data not supported"));
    }
    let mut x_offset: Option<usize> = None;
    let mut y_offset: Option<usize> = None;
    let mut z_offset: Option<usize> = None;
    let mut intensity_offset: Option<usize> = None;
    let mut time_offset: Option<usize> = None;
    for field in &msg.fields {
        if field.datatype != FLOAT32 {
            continue;
        }
        match field.name.as_str() {
            "x" => x_offset = Some(field.offset as usize),
            "y" => y_offset = Some(field.offset as usize),
            "z" => z_offset = Some(field.offset as usize),
            "intensity" => intensity_offset = Some(field.offset as usize),
            "time" | "t" | "timestamp" | "curvature" => time_offset = Some(field.offset as usize),
            _ => {}
        }
    }
    let x_offset = x_offset.ok_or(ExtractError("missing float32 x field"))?;
    let y_offset = y_offset.ok_or(ExtractError("missing float32 y field"))?;
    let z_offset = z_offset.ok_or(ExtractError("missing float32 z field"))?;

    let count = (msg.width as usize) * (msg.height as usize);
    let step = msg.point_step as usize;
    if step == 0 {
        return Err(ExtractError("point_step is 0"));
    }
    if msg.data.len() < count * step {
        return Err(ExtractError("data shorter than width*height*point_step"));
    }
    let max_offset = x_offset.max(y_offset).max(z_offset);
    if max_offset + 4 > step {
        return Err(ExtractError("xyz offsets do not fit within point_step"));
    }

    let mut cloud = Vec::with_capacity(count);
    for index in 0..count {
        let base = index * step;
        let x = read_f32(&msg.data, base + x_offset);
        let y = read_f32(&msg.data, base + y_offset);
        let z = read_f32(&msg.data, base + z_offset);
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            continue;
        }
        let intensity = match intensity_offset {
            Some(offset) if offset + 4 <= step => read_f32(&msg.data, base + offset),
            _ => 0.0,
        };
        // Pipeline expects curvature as the ms offset from scan start.
        let curvature = match time_offset {
            Some(offset) if offset + 4 <= step => read_f32(&msg.data, base + offset) * 1000.0,
            _ => 0.0,
        };
        cloud.push(Point {
            x,
            y,
            z,
            intensity,
            curvature,
        });
    }
    Ok(cloud)
}

fn build_pointcloud(cloud: &PointCloud, frame_id: &str, stamp: Time) -> PointCloud2 {
    let mut data = Vec::with_capacity(cloud.len() * POINT_STEP as usize);
    for point in cloud {
        data.extend_from_slice(&point.x.to_le_bytes());
        data.extend_from_slice(&point.y.to_le_bytes());
        data.extend_from_slice(&point.z.to_le_bytes());
        data.extend_from_slice(&point.intensity.to_le_bytes());
    }
    let make_field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: FLOAT32,
        count: 1,
    };
    let width = cloud.len() as i32;
    PointCloud2 {
        header: Header {
            seq: 0,
            stamp,
            frame_id: frame_id.into(),
        },
        height: 1,
        width,
        fields: vec![
            make_field("x", 0),
            make_field("y", 4),
            make_field("z", 8),
            make_field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: POINT_STEP,
        row_step: POINT_STEP * width,
        data,
        is_dense: true,
    }
}

fn read_f32(buffer: &[u8], offset: usize) -> f32 {
    let bytes: [u8; 4] = buffer[offset..offset + 4]
        .try_into()
        .expect("bounds checked by caller");
    f32::from_le_bytes(bytes)
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<FastLio2Rust, _>(transport)
        .await
        .expect("fastlio2_rust run failed");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_cloud_reads_xyz_intensity() {
        let msg = build_pointcloud(
            &vec![
                Point {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                    intensity: 9.0,
                    curvature: 0.0,
                },
                Point {
                    x: 4.0,
                    y: 5.0,
                    z: 6.0,
                    intensity: 8.0,
                    curvature: 0.0,
                },
            ],
            "lidar",
            Time::default(),
        );
        let cloud = extract_cloud(&msg).unwrap();
        assert_eq!(cloud.len(), 2);
        assert_eq!(cloud[0].x, 1.0);
        assert_eq!(cloud[1].intensity, 8.0);
    }

    #[test]
    fn extract_cloud_reads_time_into_curvature_ms() {
        // x, y, z, intensity, time (float32) => point_step 20. time is seconds
        // from scan start; curvature must come back in milliseconds.
        let points: [(f32, f32, f32, f32, f32); 2] =
            [(1.0, 2.0, 3.0, 0.0, 0.0), (4.0, 5.0, 6.0, 0.0, 0.005)];
        let mut data = Vec::new();
        for (x, y, z, intensity, time) in points {
            for value in [x, y, z, intensity, time] {
                data.extend_from_slice(&value.to_le_bytes());
            }
        }
        let make_field = |name: &str, offset: i32| PointField {
            name: name.into(),
            offset,
            datatype: FLOAT32,
            count: 1,
        };
        let msg = PointCloud2 {
            header: Header {
                seq: 0,
                stamp: Time::default(),
                frame_id: "lidar".into(),
            },
            height: 1,
            width: 2,
            fields: vec![
                make_field("x", 0),
                make_field("y", 4),
                make_field("z", 8),
                make_field("intensity", 12),
                make_field("time", 16),
            ],
            is_bigendian: false,
            point_step: 20,
            row_step: 40,
            data,
            is_dense: true,
        };
        let cloud = extract_cloud(&msg).unwrap();
        assert_eq!(cloud.len(), 2);
        assert_eq!(cloud[0].curvature, 0.0);
        assert!((cloud[1].curvature - 5.0).abs() < 1e-4);
    }
}
