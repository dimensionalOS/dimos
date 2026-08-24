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
    CameraModel, ImageFrame, ImuNoiseModel, ImuSample, OdometryEstimate, PointCloud, Twist,
};
use dimos_module::Transform;
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{CameraInfo, Image, Imu, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};

use crate::imu_info::ImuInfo;

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

fn vector3(vector: &lcm_msgs::geometry_msgs::Vector3) -> Vector3<f64> {
    Vector3::new(vector.x, vector.y, vector.z)
}

fn flatten(matrix: &Matrix6<f64>, flat: &mut [f64; 36]) {
    for row in 0..6 {
        for column in 0..6 {
            flat[row * 6 + column] = matrix[(row, column)];
        }
    }
}

pub fn to_isometry(transform: &Transform) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(transform.translation()),
        transform.rotation(),
    )
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
        distortion: info.D,
        intrinsics: info.K,
    }
}

pub fn to_imu_sample(msg: &Imu) -> ImuSample {
    ImuSample {
        timestamp_ns: stamp_to_ns(&msg.header),
        frame_id: msg.header.frame_id.clone(),
        angular_velocity: vector3(&msg.angular_velocity),
        linear_acceleration: vector3(&msg.linear_acceleration),
    }
}

pub fn to_noise_model(info: ImuInfo) -> ImuNoiseModel {
    ImuNoiseModel {
        frame_id: info.header.frame_id,
        gyro_noise_density: info.gyro_noise_density,
        gyro_random_walk: info.gyro_random_walk,
        accel_noise_density: info.accel_noise_density,
        accel_random_walk: info.accel_random_walk,
        frequency: info.frequency,
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
            linear: vector3(&msg.twist.twist.linear),
            angular: vector3(&msg.twist.twist.angular),
        },
        twist_covariance: Matrix6::from_row_slice(&msg.twist.covariance),
    }
}

pub fn to_odometry_msg(estimate: &OdometryEstimate) -> Odometry {
    let mut msg = Odometry::default();
    msg.header.stamp = to_stamp(estimate.timestamp_ns);
    msg.header.frame_id = estimate.frame_id.clone();
    msg.child_frame_id = estimate.child_frame_id.clone();
    msg.pose.pose.position.x = estimate.pose.translation.x;
    msg.pose.pose.position.y = estimate.pose.translation.y;
    msg.pose.pose.position.z = estimate.pose.translation.z;
    let rotation = estimate.pose.rotation.quaternion();
    msg.pose.pose.orientation.x = rotation.i;
    msg.pose.pose.orientation.y = rotation.j;
    msg.pose.pose.orientation.z = rotation.k;
    msg.pose.pose.orientation.w = rotation.w;
    msg.twist.twist.linear.x = estimate.twist.linear.x;
    msg.twist.twist.linear.y = estimate.twist.linear.y;
    msg.twist.twist.linear.z = estimate.twist.linear.z;
    msg.twist.twist.angular.x = estimate.twist.angular.x;
    msg.twist.twist.angular.y = estimate.twist.angular.y;
    msg.twist.twist.angular.z = estimate.twist.angular.z;
    flatten(&estimate.pose_covariance, &mut msg.pose.covariance);
    flatten(&estimate.twist_covariance, &mut msg.twist.covariance);
    msg
}

pub fn to_transform(estimate: &OdometryEstimate) -> Transform {
    Transform::new(
        estimate.frame_id.clone(),
        estimate.child_frame_id.clone(),
        estimate.timestamp_ns as f64 / NS_PER_SEC as f64,
        estimate.pose,
    )
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
    let mut msg = PointCloud2::default();
    msg.header.stamp = to_stamp(cloud.timestamp_ns);
    msg.header.frame_id = cloud.frame_id.clone();
    msg.height = 1;
    msg.width = cloud.points.len() as i32;
    msg.fields = vec![xyz_field("x", 0), xyz_field("y", 4), xyz_field("z", 8)];
    msg.is_bigendian = false;
    msg.point_step = BYTES_PER_POINT;
    msg.row_step = BYTES_PER_POINT * msg.width;
    msg.data = Vec::with_capacity(cloud.points.len() * BYTES_PER_POINT as usize);
    for point in &cloud.points {
        for coordinate in point {
            msg.data.extend_from_slice(&coordinate.to_le_bytes());
        }
    }
    msg.is_dense = true;
    msg
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
