// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Message-input Point-LIO native module (issue #2821, option 1): consumes
// PointCloud2 + Imu from the transport instead of Livox network packets, so
// any driver — or a replayed recording — can feed it.
//
// Ports:
//   raw_lidar: PointCloud2 in   (sensor-frame sweeps, e.g. from the Mid-360
//                                driver's `lidar` output or RawSensorReplay)
//   imu:       Imu in           (driver conventions: rad/s, m/s^2)
//   lidar:     PointCloud2 out  (named for drop-in compatibility with the
//                                current PointLio consumers)
//   odometry:  Odometry out
//
// SCAFFOLD STATE: the estimator is not implemented yet. The module passes
// clouds through re-stamped into the odom frame chain and publishes identity
// odometry, which exercises the full wiring (config over stdin, transport
// subscribe/publish, blueprint, replay) end to end. The estimator slot is
// `Estimator::update` below; direction (FFI wrap of the GPL C++ core vs a
// port living in a separate GPL repo) is pending maintainer input on #2821.

use std::time::Duration;

use dimos_module::{
    error_throttled, info_throttled, native_config, run_with_transport, Input, Module, Output,
};
use lcm_msgs::geometry_msgs::{
    Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{Imu, PointCloud2};
use lcm_msgs::std_msgs::Header;

#[native_config]
struct Config {
    /// Fixed frame odometry is expressed in (parent of sensor_frame_id).
    map_frame_id: String,
    /// Moving sensor frame; published clouds are stamped with it.
    sensor_frame_id: String,
}

/// Estimator slot. Today: identity pose, counts only. The IESKF (either via
/// FFI to the existing core or a Rust implementation) replaces this struct
/// without touching the module wiring around it.
#[derive(Default)]
struct Estimator {
    clouds: u64,
    imu_samples: u64,
}

impl Estimator {
    /// Ingest one IMU sample. (Future: propagate the filter state.)
    fn on_imu(&mut self, _msg: &Imu) {
        self.imu_samples += 1;
    }

    /// Ingest one sweep, return the pose estimate. (Future: undistort,
    /// register against the map, IESKF update.) Identity for now.
    fn on_cloud(&mut self, _msg: &PointCloud2) -> Pose {
        self.clouds += 1;
        Pose {
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        }
    }
}

#[derive(Module)]
struct PointLio {
    #[input(decode = PointCloud2::decode, handler = on_raw_lidar)]
    raw_lidar: Input<PointCloud2>,

    #[input(decode = Imu::decode, handler = on_imu)]
    imu: Input<Imu>,

    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    #[output(encode = Odometry::encode)]
    odometry: Output<Odometry>,

    #[config]
    config: Config,

    estimator: Estimator,
}

impl PointLio {
    async fn on_imu(&mut self, msg: Imu) {
        self.estimator.on_imu(&msg);
    }

    async fn on_raw_lidar(&mut self, msg: PointCloud2) {
        let pose = self.estimator.on_cloud(&msg);
        let stamp = msg.header.stamp.clone();

        info_throttled!(
            Duration::from_secs(5),
            clouds = self.estimator.clouds,
            imu_samples = self.estimator.imu_samples,
            "pointlio scaffold: passing through (estimator not implemented)",
        );

        let mut cloud = msg;
        cloud.header.frame_id = self.config.sensor_frame_id.clone();
        if let Err(e) = self.lidar.publish(&cloud).await {
            error_throttled!(Duration::from_secs(1), error = %e, "lidar publish failed");
        }

        let odom = Odometry {
            header: Header {
                seq: 0,
                stamp,
                frame_id: self.config.map_frame_id.clone(),
            },
            child_frame_id: self.config.sensor_frame_id.clone(),
            pose: PoseWithCovariance {
                pose,
                covariance: [0.0; 36],
            },
            twist: TwistWithCovariance {
                twist: Twist {
                    linear: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                },
                covariance: [0.0; 36],
            },
        };
        if let Err(e) = self.odometry.publish(&odom).await {
            error_throttled!(Duration::from_secs(1), error = %e, "odometry publish failed");
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<PointLio>().await;
}
