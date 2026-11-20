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
// LCM messages in and out of the dim_slam library's plain types.

use dim_slam::nalgebra::{Isometry3, Matrix6, Quaternion, Translation3, UnitQuaternion, Vector3};
use dim_slam::{
    CameraModel, DeskewedCloud, ImageFrame, ImuSample, LidarScan, OdometryEstimate, PointCloud,
    Twist,
};
use dimos_module::{Tf, Transform};
use lcm_msgs::geometry_msgs;
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{CameraInfo, Image, Imu, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};

const NS_PER_SEC: i64 = 1_000_000_000;
const BYTES_PER_POINT: i32 = 12;

fn stamp_to_ns(header: &Header) -> i64 {
    header.stamp.sec as i64 * NS_PER_SEC + header.stamp.nsec as i64
}

fn to_stamp(timestamp_ns: i64) -> Time {
    Time {
        sec: (timestamp_ns / NS_PER_SEC) as i32,
        nsec: (timestamp_ns % NS_PER_SEC) as i32,
    }
}

fn to_header(timestamp_ns: i64, frame_id: &str) -> Header {
    Header {
        seq: 0,
        stamp: to_stamp(timestamp_ns),
        frame_id: frame_id.to_string(),
    }
}

fn to_vector3(vector: &geometry_msgs::Vector3) -> Vector3<f64> {
    Vector3::new(vector.x, vector.y, vector.z)
}

fn to_vector3_msg(vector: &Vector3<f64>) -> geometry_msgs::Vector3 {
    geometry_msgs::Vector3 {
        x: vector.x,
        y: vector.y,
        z: vector.z,
    }
}

// nalgebra stores column-major, the message wants row-major.
fn row_major(matrix: &Matrix6<f64>) -> [f64; 36] {
    let mut flat = [0.0; 36];
    flat.copy_from_slice(matrix.transpose().as_slice());
    flat
}

pub fn tf_lookup(tf: &Tf) -> impl Fn(&str, &str) -> Option<Isometry3<f64>> + '_ {
    move |parent: &str, child: &str| {
        tf.get_latest(parent, child).map(|transform| {
            Isometry3::from_parts(
                Translation3::from(transform.translation()),
                transform.rotation(),
            )
        })
    }
}

pub fn to_image_frame(img: Image) -> ImageFrame {
    ImageFrame {
        timestamp_ns: stamp_to_ns(&img.header),
        frame_id: img.header.frame_id,
        width: img.width,
        height: img.height,
        encoding: img.encoding,
        step: img.step,
        data: img.data,
    }
}

pub fn to_camera_model(info: CameraInfo) -> CameraModel {
    CameraModel {
        timestamp_ns: stamp_to_ns(&info.header),
        frame_id: info.header.frame_id,
        width: info.width,
        height: info.height,
        distortion_model: info.distortion_model,
        distortion: info.D,
        intrinsics: info.K,
    }
}

pub fn to_imu_sample(msg: &Imu) -> ImuSample {
    ImuSample {
        timestamp_ns: stamp_to_ns(&msg.header),
        frame_id: msg.header.frame_id.clone(),
        angular_velocity: to_vector3(&msg.angular_velocity),
        linear_acceleration: to_vector3(&msg.linear_acceleration),
    }
}

pub fn to_estimate(msg: &Odometry) -> OdometryEstimate {
    let position = &msg.pose.pose.position;
    let orientation = &msg.pose.pose.orientation;
    OdometryEstimate {
        timestamp_ns: stamp_to_ns(&msg.header),
        frame_id: msg.header.frame_id.clone(),
        child_frame_id: msg.child_frame_id.clone(),
        pose: Isometry3::from_parts(
            Translation3::new(position.x, position.y, position.z),
            UnitQuaternion::from_quaternion(Quaternion::new(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            )),
        ),
        pose_covariance: Matrix6::from_row_slice(&msg.pose.covariance),
        twist: Twist {
            linear: to_vector3(&msg.twist.twist.linear),
            angular: to_vector3(&msg.twist.twist.angular),
        },
        twist_covariance: Matrix6::from_row_slice(&msg.twist.covariance),
    }
}

pub fn to_odometry_msg(estimate: &OdometryEstimate) -> Odometry {
    let translation = estimate.pose.translation;
    let rotation = estimate.pose.rotation.quaternion();
    Odometry {
        header: to_header(estimate.timestamp_ns, &estimate.frame_id),
        child_frame_id: estimate.child_frame_id.clone(),
        pose: geometry_msgs::PoseWithCovariance {
            pose: geometry_msgs::Pose {
                position: geometry_msgs::Point {
                    x: translation.x,
                    y: translation.y,
                    z: translation.z,
                },
                orientation: geometry_msgs::Quaternion {
                    x: rotation.i,
                    y: rotation.j,
                    z: rotation.k,
                    w: rotation.w,
                },
            },
            covariance: row_major(&estimate.pose_covariance),
        },
        twist: geometry_msgs::TwistWithCovariance {
            twist: geometry_msgs::Twist {
                linear: to_vector3_msg(&estimate.twist.linear),
                angular: to_vector3_msg(&estimate.twist.angular),
            },
            covariance: row_major(&estimate.twist_covariance),
        },
    }
}

pub fn to_transform(estimate: &OdometryEstimate) -> Transform {
    Transform::new(
        estimate.frame_id.clone(),
        estimate.child_frame_id.clone(),
        estimate.timestamp_ns as f64 / NS_PER_SEC as f64,
        estimate.pose,
    )
}

/// A cloud as Livox drivers write it: float32 x/y/z, and when present a uint32 `offset_time` (ns
/// after the stamp), a uint8 `tag` and a float32 `intensity` (0..255). None when x/y/z are missing,
/// the data is big-endian, shorter than its points or a field overruns its point.
pub fn to_lidar_scan(msg: &PointCloud2) -> Option<LidarScan> {
    let field = |name: &str, datatype: i32| {
        msg.fields
            .iter()
            .find(|f| f.name == name && f.datatype as i32 == datatype)
            .map(|f| f.offset as usize)
    };
    let float32 = PointField::FLOAT32 as i32;
    let (x, y, z) = (
        field("x", float32)?,
        field("y", float32)?,
        field("z", float32)?,
    );
    let time = field("offset_time", PointField::UINT32 as i32);
    let tag = field("tag", PointField::UINT8 as i32);
    let intensity = field("intensity", float32);
    let step = msg.point_step as usize;
    let count = (msg.width.max(0) as usize) * (msg.height.max(0) as usize);
    let widest = [Some(x), Some(y), Some(z), time, intensity]
        .into_iter()
        .flatten()
        .max()
        .unwrap_or(0)
        + 4;
    if msg.is_bigendian
        || step == 0
        || msg.data.len() < count * step
        || widest > step
        || tag.is_some_and(|at| at >= step)
    {
        return None;
    }
    let bytes_at = |p: &[u8], at: usize| -> [u8; 4] {
        p.get(at..at + 4)
            .and_then(|b| b.try_into().ok())
            .unwrap_or_default()
    };
    let f32_at = |p: &[u8], at: usize| f32::from_le_bytes(bytes_at(p, at));
    let mut scan = LidarScan {
        timestamp_ns: stamp_to_ns(&msg.header),
        frame_id: msg.header.frame_id.clone(),
        points: Vec::with_capacity(count),
        ..Default::default()
    };
    for p in msg.data.chunks_exact(step).take(count) {
        scan.points.push([f32_at(p, x), f32_at(p, y), f32_at(p, z)]);
        if let Some(at) = time {
            scan.offsets_ns.push(u32::from_le_bytes(bytes_at(p, at)));
        }
        if let Some(at) = tag {
            scan.tags.push(p.get(at).copied().unwrap_or_default());
        }
        if let Some(at) = intensity {
            scan.intensities.push(f32_at(p, at) / 255.0);
        }
    }
    Some(scan)
}

/// A deskewed scan as float32 x/y/z/intensity (0..255) and a uint8 `lidar`: its lidar's index in `lidars`.
pub fn to_deskewed_cloud_msg(deskewed: &DeskewedCloud) -> PointCloud2 {
    let field = |name: &str, offset: i32, datatype: u8| PointField {
        name: name.into(),
        offset,
        datatype,
        count: 1,
    };
    let step = 20usize;
    let lidar = u8::try_from(deskewed.lidar).unwrap_or(u8::MAX);
    let points = &deskewed.cloud.points;
    let mut data = Vec::with_capacity(points.len() * step);
    for (i, p) in points.iter().enumerate() {
        for c in p {
            data.extend_from_slice(&c.to_le_bytes());
        }
        let intensity = deskewed.intensities.get(i).copied().unwrap_or(0.0);
        data.extend_from_slice(&intensity.to_le_bytes());
        data.extend_from_slice(&[lidar, 0, 0, 0]);
    }
    PointCloud2 {
        header: to_header(deskewed.cloud.timestamp_ns, &deskewed.cloud.frame_id),
        height: 1,
        width: points.len() as i32,
        fields: vec![
            field("x", 0, PointField::FLOAT32 as u8),
            field("y", 4, PointField::FLOAT32 as u8),
            field("z", 8, PointField::FLOAT32 as u8),
            field("intensity", 12, PointField::FLOAT32 as u8),
            field("lidar", 16, PointField::UINT8 as u8),
        ],
        is_bigendian: false,
        point_step: step as i32,
        row_step: (step * points.len()) as i32,
        data,
        is_dense: true,
    }
}

fn xyz_field(name: &str, offset: i32) -> PointField {
    PointField {
        name: name.to_string(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    }
}

pub fn to_point_cloud2(cloud: &PointCloud) -> PointCloud2 {
    let width = cloud.points.len() as i32;
    let mut data = Vec::with_capacity(cloud.points.len() * BYTES_PER_POINT as usize);
    for point in &cloud.points {
        for coordinate in point {
            data.extend_from_slice(&coordinate.to_le_bytes());
        }
    }
    PointCloud2 {
        header: to_header(cloud.timestamp_ns, &cloud.frame_id),
        height: 1,
        width,
        fields: vec![xyz_field("x", 0), xyz_field("y", 4), xyz_field("z", 8)],
        is_bigendian: false,
        point_step: BYTES_PER_POINT,
        row_step: BYTES_PER_POINT * width,
        data,
        is_dense: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamp_round_trips_through_ns() {
        let header = Header {
            stamp: to_stamp(1_234_567_890_123_456_789),
            ..Default::default()
        };
        assert_eq!(stamp_to_ns(&header), 1_234_567_890_123_456_789);
    }

    #[test]
    fn odometry_round_trips_through_the_estimate() {
        let mut msg = Odometry::default();
        msg.header.stamp = to_stamp(42 * NS_PER_SEC);
        msg.header.frame_id = "odom".to_string();
        msg.child_frame_id = "base_link".to_string();
        msg.pose.pose.position.x = 1.5;
        msg.pose.pose.orientation.w = 1.0;
        msg.twist.twist.linear.x = 0.3;
        msg.twist.twist.angular.z = -0.2;
        for dim in 0..6 {
            msg.pose.covariance[dim * 7] = 0.1 * (dim + 1) as f64;
            msg.twist.covariance[dim * 7] = 0.2 * (dim + 1) as f64;
        }
        msg.pose.covariance[1] = 0.05;

        let back = to_odometry_msg(&to_estimate(&msg));
        assert_eq!(back, msg);
    }

    #[test]
    fn a_livox_style_cloud_becomes_a_timed_scan() {
        let field = |name: &str, offset: i32, datatype: u8| PointField {
            name: name.into(),
            offset,
            datatype,
            count: 1,
        };
        let mut data = Vec::new();
        for (i, point) in [[1.0f32, 2.0, 3.0], [-4.0, 5.0, -6.0]].iter().enumerate() {
            for c in point {
                data.extend_from_slice(&c.to_le_bytes());
            }
            data.extend_from_slice(&127.5f32.to_le_bytes());
            data.extend_from_slice(&(1000 * i as u32).to_le_bytes());
        }
        let msg = PointCloud2 {
            header: to_header(7 * NS_PER_SEC, "front_lidar"),
            height: 1,
            width: 2,
            fields: vec![
                field("x", 0, PointField::FLOAT32 as u8),
                field("y", 4, PointField::FLOAT32 as u8),
                field("z", 8, PointField::FLOAT32 as u8),
                field("intensity", 12, PointField::FLOAT32 as u8),
                field("offset_time", 16, PointField::UINT32 as u8),
            ],
            is_bigendian: false,
            point_step: 20,
            row_step: 40,
            data,
            is_dense: true,
        };
        let scan = to_lidar_scan(&msg).expect("a usable cloud");
        assert_eq!(scan.timestamp_ns, 7 * NS_PER_SEC);
        assert_eq!(scan.frame_id, "front_lidar");
        assert_eq!(scan.points, vec![[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]]);
        assert_eq!(scan.offsets_ns, vec![0, 1000]);
        assert!(scan.tags.is_empty());
        assert_eq!(scan.intensities, vec![0.5, 0.5]);
    }

    #[test]
    fn point_cloud_packs_little_endian_xyz() {
        let cloud = PointCloud {
            timestamp_ns: 7 * NS_PER_SEC,
            frame_id: "depth".to_string(),
            points: vec![[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]],
        };
        let msg = to_point_cloud2(&cloud);
        assert_eq!(msg.width, 2);
        assert_eq!(msg.row_step, 24);
        assert_eq!(msg.data.len(), 24);
        let z1 = f32::from_le_bytes(msg.data[20..24].try_into().unwrap());
        assert_eq!(z1, -6.0);
        assert_eq!(msg.fields.len(), 3);
        assert_eq!(msg.fields[1].offset, 4);
    }
}
