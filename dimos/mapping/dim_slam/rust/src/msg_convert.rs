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
// Generated ROS2 messages in and out of the dim_slam library's plain types.

use dim_slam::nalgebra::{Isometry3, Matrix6, Quaternion, Translation3, UnitQuaternion, Vector3};
use dim_slam::{CameraModel, ImageFrame, ImuSample, OdometryEstimate, PointCloud, Twist};
use dimos_generated_messages::builtin_interfaces::msg::Time;
use dimos_generated_messages::geometry_msgs::msg as geometry_msgs;
use dimos_generated_messages::nav_msgs::msg::Odometry;
use dimos_generated_messages::sensor_msgs::msg::{CameraInfo, Image, Imu, PointCloud2, PointField};
use dimos_generated_messages::std_msgs::msg::Header;
use dimos_module::{Tf, Transform};

const NS_PER_SEC: i64 = 1_000_000_000;
const BYTES_PER_POINT: u32 = 12;

fn stamp_to_ns(header: &Header) -> i64 {
    header.stamp.sec as i64 * NS_PER_SEC + header.stamp.nanosec as i64
}

fn to_stamp(timestamp_ns: i64) -> Time {
    Time {
        sec: timestamp_ns.div_euclid(NS_PER_SEC) as i32,
        nanosec: timestamp_ns.rem_euclid(NS_PER_SEC) as u32,
    }
}

fn to_header(timestamp_ns: i64, frame_id: &str) -> Header {
    Header {
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

pub fn to_image_frame(mut img: Image) -> Result<ImageFrame, String> {
    let (scalar_bytes, channels) = match img.encoding.as_str() {
        "mono8" => (1usize, 1),
        "rgb8" | "bgr8" => (1, 3),
        "mono16" | "16UC1" => (2, 1),
        "32FC1" => (4, 1),
        _ => return Err(format!("unsupported image encoding {}", img.encoding)),
    };
    let row_bytes = img.width as usize * scalar_bytes * channels;
    let step = img.step as usize;
    if step < row_bytes || img.data.len() != step * img.height as usize {
        return Err("image dimensions/stride do not match data".into());
    }
    if img.is_bigendian != 0 && scalar_bytes > 1 && step > 0 {
        for row in img.data.chunks_exact_mut(step) {
            for value in row[..row_bytes].chunks_exact_mut(scalar_bytes) {
                value.reverse();
            }
        }
    }
    if img.encoding == "bgr8" && step > 0 {
        for row in img.data.chunks_exact_mut(step) {
            for pixel in row[..row_bytes].as_chunks_mut::<3>().0 {
                pixel.swap(0, 2);
            }
        }
        img.encoding = "rgb8".into();
    }
    Ok(ImageFrame {
        timestamp_ns: stamp_to_ns(&img.header),
        frame_id: img.header.frame_id,
        width: img
            .width
            .try_into()
            .map_err(|_| "image width exceeds backend range")?,
        height: img
            .height
            .try_into()
            .map_err(|_| "image height exceeds backend range")?,
        encoding: img.encoding,
        step: img
            .step
            .try_into()
            .map_err(|_| "image step exceeds backend range")?,
        data: img.data,
    })
}

pub fn to_camera_model(info: CameraInfo) -> Result<CameraModel, String> {
    Ok(CameraModel {
        timestamp_ns: stamp_to_ns(&info.header),
        frame_id: info.header.frame_id,
        width: info
            .width
            .try_into()
            .map_err(|_| "camera width exceeds backend range")?,
        height: info
            .height
            .try_into()
            .map_err(|_| "camera height exceeds backend range")?,
        distortion: info.d,
        intrinsics: info.k,
    })
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

fn xyz_field(name: &str, offset: u32) -> PointField {
    PointField {
        name: name.to_string(),
        offset,
        datatype: PointField::FLOAT32,
        count: 1,
    }
}

pub fn to_point_cloud2(cloud: &PointCloud) -> PointCloud2 {
    let width = cloud.points.len() as u32;
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
    fn image_adapter_normalizes_depth_byte_order_and_preserves_row_padding() {
        let image = Image {
            width: 1,
            height: 2,
            step: 4,
            encoding: "16UC1".into(),
            is_bigendian: 1,
            data: vec![0x12, 0x34, 99, 98, 0x56, 0x78, 97, 96],
            ..Default::default()
        };
        let frame = to_image_frame(image).unwrap();
        assert_eq!(frame.data, [0x34, 0x12, 99, 98, 0x78, 0x56, 97, 96]);
        assert_eq!((frame.width, frame.height, frame.step), (1, 2, 4));
    }

    #[test]
    fn image_adapter_normalizes_bgr_for_the_backend() {
        let frame = to_image_frame(Image {
            width: 1,
            height: 1,
            step: 4,
            encoding: "bgr8".into(),
            data: vec![1, 2, 3, 99],
            ..Default::default()
        })
        .unwrap();
        assert_eq!(frame.encoding, "rgb8");
        assert_eq!(frame.data, [3, 2, 1, 99]);
    }

    #[test]
    fn image_adapter_rejects_invalid_layout_and_backend_dimension_overflow() {
        assert!(to_image_frame(Image {
            width: 1,
            height: 1,
            step: 1,
            encoding: "rgb8".into(),
            data: vec![1],
            ..Default::default()
        })
        .is_err());
        assert!(to_camera_model(CameraInfo {
            width: u32::MAX,
            ..Default::default()
        })
        .is_err());
        assert!(to_image_frame(Image {
            encoding: "16UC1".into(),
            is_bigendian: 1,
            ..Default::default()
        })
        .is_ok());
    }

    #[test]
    fn negative_source_stamp_is_normalized() {
        let stamp = to_stamp(-250000000);
        assert_eq!((stamp.sec, stamp.nanosec), (-1, 750000000));
    }

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
