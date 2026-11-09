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
// CDR messages in and out of the dim_slam library's plain types.

use dim_slam::nalgebra::{Isometry3, Matrix6, Quaternion, Translation3, UnitQuaternion, Vector3};
use dim_slam::{CameraModel, ImageFrame, ImuSample, OdometryEstimate, PointCloud, Twist};
use dimos_generated_messages::{
    builtin_interfaces::msg::time::Time,
    geometry_msgs::msg::{
        point::Point, pose::Pose, pose_with_covariance::PoseWithCovariance,
        quaternion::Quaternion as WireQuaternion, transform::Transform as WireTransform,
        twist::Twist as WireTwist, twist_with_covariance::TwistWithCovariance,
        vector3::Vector3 as WireVector3,
    },
    nav_msgs::msg::odometry::Odometry,
    sensor_msgs::msg::{
        camera_info::CameraInfo, image::Image, imu::Imu, point_cloud2::PointCloud2,
        point_field::PointField,
    },
    std_msgs::msg::header::Header,
};
use dimos_module::{Tf, TransformStamped};

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

fn to_vector3(vector: &WireVector3) -> Vector3<f64> {
    Vector3::new(vector.x, vector.y, vector.z)
}

fn to_vector3_msg(vector: &Vector3<f64>) -> WireVector3 {
    WireVector3 {
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
        tf.get_latest(parent, child).map(|edge| {
            let p = edge.transform.translation;
            let q = edge.transform.rotation;
            Isometry3::from_parts(
                Translation3::new(p.x, p.y, p.z),
                UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z)),
            )
        })
    }
}

pub fn to_image_frame(img: Image) -> ImageFrame {
    ImageFrame {
        timestamp_ns: stamp_to_ns(&img.header),
        frame_id: img.header.frame_id,
        width: img.width as i32,
        height: img.height as i32,
        encoding: img.encoding,
        step: img.step as i32,
        data: img.data,
    }
}

pub fn to_camera_model(info: CameraInfo) -> CameraModel {
    CameraModel {
        timestamp_ns: stamp_to_ns(&info.header),
        frame_id: info.header.frame_id,
        width: info.width as i32,
        height: info.height as i32,
        distortion: info.d,
        intrinsics: info.k,
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
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: translation.x,
                    y: translation.y,
                    z: translation.z,
                },
                orientation: WireQuaternion {
                    x: rotation.i,
                    y: rotation.j,
                    z: rotation.k,
                    w: rotation.w,
                },
            },
            covariance: row_major(&estimate.pose_covariance),
        },
        twist: TwistWithCovariance {
            twist: WireTwist {
                linear: to_vector3_msg(&estimate.twist.linear),
                angular: to_vector3_msg(&estimate.twist.angular),
            },
            covariance: row_major(&estimate.twist_covariance),
        },
    }
}

pub fn to_transform(estimate: &OdometryEstimate) -> TransformStamped {
    let odometry = to_odometry_msg(estimate);
    TransformStamped {
        header: odometry.header,
        child_frame_id: odometry.child_frame_id,
        transform: WireTransform {
            translation: WireVector3 {
                x: odometry.pose.pose.position.x,
                y: odometry.pose.pose.position.y,
                z: odometry.pose.pose.position.z,
            },
            rotation: odometry.pose.pose.orientation,
        },
    }
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
    fn stamp_round_trips_through_ns() {
        let header = Header {
            stamp: to_stamp(1_234_567_890_123_456_789),
            frame_id: String::new(),
        };
        assert_eq!(stamp_to_ns(&header), 1_234_567_890_123_456_789);
    }

    #[test]
    fn odometry_round_trips_through_the_estimate() {
        let mut msg = to_odometry_msg(&OdometryEstimate::default());
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
    fn generated_messages_cross_cdr_boundary_with_exact_stamps_and_calibration() {
        use dimos_module::cdr;
        let image = Image {
            header: to_header(12_000_000_001, "left"),
            width: 2,
            height: 1,
            step: 6,
            encoding: "rgb8".into(),
            data: vec![1, 2, 3, 4, 5, 6],
            is_bigendian: 0,
        };
        let info = CameraInfo {
            header: image.header.clone(),
            width: 2,
            height: 1,
            d: vec![0.1, 0.2],
            k: [2., 0., 1., 0., 3., 0.5, 0., 0., 1.],
            distortion_model: "plumb_bob".into(),
            r: [0.; 9],
            p: [0.; 12],
            binning_x: 0,
            binning_y: 0,
            roi: dimos_generated_messages::sensor_msgs::msg::region_of_interest::RegionOfInterest {
                x_offset: 0,
                y_offset: 0,
                height: 0,
                width: 0,
                do_rectify: false,
            },
        };
        for little_endian in [true, false] {
            let frame = to_image_frame(
                cdr::decode(&cdr::encode_endian(&image, little_endian).unwrap()).unwrap(),
            );
            assert_eq!(frame.timestamp_ns, 12_000_000_001);
            assert_eq!((frame.width, frame.height, frame.step), (2, 1, 6));
            assert_eq!(frame.data, vec![1, 2, 3, 4, 5, 6]);
            let calibration = to_camera_model(
                cdr::decode(&cdr::encode_endian(&info, little_endian).unwrap()).unwrap(),
            );
            assert_eq!(calibration.distortion, vec![0.1, 0.2]);
            assert_eq!(
                calibration.intrinsics,
                [2., 0., 1., 0., 3., 0.5, 0., 0., 1.]
            );
            let estimate = OdometryEstimate {
                timestamp_ns: 12_000_000_001,
                frame_id: "odom".into(),
                child_frame_id: "base".into(),
                ..Default::default()
            };
            let output = to_odometry_msg(&estimate);
            let restored: Odometry =
                cdr::decode(&cdr::encode_endian(&output, little_endian).unwrap()).unwrap();
            assert_eq!(to_estimate(&restored).timestamp_ns, 12_000_000_001);
            assert_eq!(to_transform(&estimate).header, restored.header);
        }
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
