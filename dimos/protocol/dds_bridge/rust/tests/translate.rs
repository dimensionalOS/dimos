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

//! CDR decode of every table row against bytes from an independent ROS 2 serializer
//! (python `rosbags`, ROS 2 Humble typestore), and the crate's own encoding of the
//! same messages, which the synthetic publisher uses, against those bytes.

use dimos_dds_bridge::ros;
use dimos_dds_bridge::translate::{self, decode_payload, DimosMsg};
use serde::Serialize;

const INT32: &[&str] = &["00010000c01dfeff"];
const UINT32: &[&str] = &["0001000000286bee"];
const BOOL: &[&str] = &["0001000001"];
const TWIST: &[&str] = &[
    "00010000000000000000e03f000000000000d0bf00000000000000000000000000000000000000000000000000000000",
    "0000f43f",
];
const POSESTAMPED: &[&str] = &[
    "0001000000f1536515cd5b07040000006d617000000000000000f03f0000000000000040000000000000084000000000",
    "000000000000000000000000cd3b7f669ea0e63fcd3b7f669ea0e63f",
];
const ODOMETRY: &[&str] = &[
    "0001000000f1536515cd5b07050000006f646f6d000000000a000000626173655f6c696e6b0000000000000000000000",
    "0000f03f0000000000000040000000000000084000000000000000000000000000000000cd3b7f669ea0e63fcd3b7f66",
    "9ea0e63f0000000000000000000000000000e03f000000000000f03f000000000000f83f000000000000004000000000",
    "0000044000000000000008400000000000000c4000000000000010400000000000001240000000000000144000000000",
    "0000164000000000000018400000000000001a400000000000001c400000000000001e40000000000000204000000000",
    "000021400000000000002240000000000000234000000000000024400000000000002540000000000000264000000000",
    "00002740000000000000284000000000000029400000000000002a400000000000002b400000000000002c4000000000",
    "00002d400000000000002e400000000000002f4000000000000030400000000000803040000000000000314000000000",
    "00803140000000000000e03f00000000000000000000000000000000000000000000000000000000000000009a999999",
    "9999b93f00000000008031400000000000003140000000000080304000000000000030400000000000002f4000000000",
    "00002e400000000000002d400000000000002c400000000000002b400000000000002a40000000000000294000000000",
    "000028400000000000002740000000000000264000000000000025400000000000002440000000000000234000000000",
    "00002240000000000000214000000000000020400000000000001e400000000000001c400000000000001a4000000000",
    "0000184000000000000016400000000000001440000000000000124000000000000010400000000000000c4000000000",
    "0000084000000000000004400000000000000040000000000000f83f000000000000f03f000000000000e03f00000000",
    "00000000",
];
const POINTCLOUD2: &[&str] = &[
    "0001000000f1536515cd5b07060000006c69646172000000010000000200000003000000020000007800000000000000",
    "07000000010000000200000079000000040000000700000001000000020000007a000000080000000700000001000000",
    "000000000c00000018000000180000000000803f0000004000004040000080400000a0400000c04001",
];
const IMU: &[&str] = &[
    "0001000000f1536515cd5b0704000000696d750000000000000000000000000000000000cd3b7f669ea0e63fcd3b7f66",
    "9ea0e63f7b14ae47e17a843f7b14ae47e17a843f7b14ae47e17a843f7b14ae47e17a843f7b14ae47e17a843f7b14ae47",
    "e17a843f7b14ae47e17a843f7b14ae47e17a843f7b14ae47e17a843f9a9999999999b93f9a9999999999c93f33333333",
    "3333d33f7b14ae47e17a943f7b14ae47e17a943f7b14ae47e17a943f7b14ae47e17a943f7b14ae47e17a943f7b14ae47",
    "e17a943f7b14ae47e17a943f7b14ae47e17a943f7b14ae47e17a943f000000000000000000000000000000001f85eb51",
    "b89e2340b81e85eb51b89e3fb81e85eb51b89e3fb81e85eb51b89e3fb81e85eb51b89e3fb81e85eb51b89e3fb81e85eb",
    "51b89e3fb81e85eb51b89e3fb81e85eb51b89e3fb81e85eb51b89e3f",
];
const TFMESSAGE: &[&str] = &[
    "000100000200000000f1536515cd5b07050000006f646f6d000000000a000000626173655f6c696e6b00000000000000",
    "0000f03f0000000000000000000000000000000000000000000000000000000000000000cd3b7f669ea0e63fcd3b7f66",
    "9ea0e63f00f1536515cd5b070a000000626173655f6c696e6b000000060000006c696461720000000000000000000000",
    "000000000000000000000000333333333333d33f00000000000000000000000000000000000000000000000000000000",
    "0000f03f",
];
const BE_POSE_STAMPED: &[&str] = &[
    "000000006553f100075bcd15000000046d6170003ff00000000000004000000000000000400800000000000000000000",
    "0000000000000000000000003fe6a09e667f3bcd3fe6a09e667f3bcd",
];

const SQRT_HALF: f64 = std::f64::consts::FRAC_1_SQRT_2;

fn bytes(chunks: &[&str]) -> Vec<u8> {
    let hex: String = chunks.concat();
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
        .collect()
}

fn decode(ros_type: &str, chunks: &[&str]) -> DimosMsg {
    let translation = translate::lookup(ros_type).expect("a table row");
    decode_payload(translation, &bytes(chunks)).expect("decodes")
}

/// The crate's little-endian encoding of `msg`, header included, must equal the fixture.
fn assert_encodes_to<T: Serialize>(msg: &T, chunks: &[&str]) {
    let mut payload = vec![0x00, 0x01, 0x00, 0x00];
    payload.extend(cdr_encoding::to_vec::<T, byteorder::LittleEndian>(msg).expect("encodes"));
    let expected = bytes(chunks);
    // CDR may pad the tail differently; the body up to the fixture's length must match.
    assert_eq!(&payload[..expected.len()], &expected[..]);
}

fn assert_header(h: &lcm_msgs::std_msgs::Header, frame_id: &str) {
    assert_eq!(h.stamp.sec, 1_700_000_000);
    assert_eq!(h.stamp.nsec, 123_456_789);
    assert_eq!(h.frame_id, frame_id);
}

fn ros_header(frame_id: &str) -> ros::Header {
    ros::Header {
        stamp: ros::Time {
            sec: 1_700_000_000,
            nanosec: 123_456_789,
        },
        frame_id: frame_id.into(),
    }
}

fn yaw_quarter() -> ros::Quaternion {
    ros::Quaternion {
        x: 0.0,
        y: 0.0,
        z: SQRT_HALF,
        w: SQRT_HALF,
    }
}

#[test]
fn int32() {
    let DimosMsg::Int32(m) = decode("std_msgs/msg/Int32", INT32) else {
        panic!()
    };
    assert_eq!(m.data, -123_456);
    assert_encodes_to(&ros::Int32 { data: -123_456 }, INT32);
}

#[test]
fn uint32_keeps_its_bits_in_an_int32() {
    let DimosMsg::UInt32(m) = decode("std_msgs/msg/UInt32", UINT32) else {
        panic!()
    };
    assert_eq!(m.data as u32, 4_000_000_000);
    assert_encodes_to(
        &ros::UInt32 {
            data: 4_000_000_000,
        },
        UINT32,
    );
}

#[test]
fn bool() {
    let DimosMsg::Bool(m) = decode("std_msgs/msg/Bool", BOOL) else {
        panic!()
    };
    assert!(m.data);
    assert_encodes_to(&ros::Bool { data: true }, BOOL);
}

#[test]
fn twist() {
    let DimosMsg::Twist(m) = decode("geometry_msgs/msg/Twist", TWIST) else {
        panic!()
    };
    assert_eq!((m.linear.x, m.linear.y, m.linear.z), (0.5, -0.25, 0.0));
    assert_eq!((m.angular.x, m.angular.y, m.angular.z), (0.0, 0.0, 1.25));
    let ros_twist = ros::Twist {
        linear: ros::Vector3 {
            x: 0.5,
            y: -0.25,
            z: 0.0,
        },
        angular: ros::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 1.25,
        },
    };
    assert_encodes_to(&ros_twist, TWIST);
}

fn ros_pose_stamped() -> ros::PoseStamped {
    ros::PoseStamped {
        header: ros_header("map"),
        pose: ros::Pose {
            position: ros::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            orientation: yaw_quarter(),
        },
    }
}

#[test]
fn pose_stamped() {
    let DimosMsg::PoseStamped(m) = decode("geometry_msgs/msg/PoseStamped", POSESTAMPED) else {
        panic!()
    };
    assert_header(&m.header, "map");
    assert_eq!(
        (m.pose.position.x, m.pose.position.y, m.pose.position.z),
        (1.0, 2.0, 3.0)
    );
    assert_eq!(
        (m.pose.orientation.z, m.pose.orientation.w),
        (SQRT_HALF, SQRT_HALF)
    );
    assert_encodes_to(&ros_pose_stamped(), POSESTAMPED);
}

#[test]
fn big_endian_decodes_the_same() {
    let little = decode("geometry_msgs/msg/PoseStamped", POSESTAMPED);
    let big = decode("geometry_msgs/msg/PoseStamped", BE_POSE_STAMPED);
    assert_eq!(little, big);
}

#[test]
fn odometry() {
    let DimosMsg::Odometry(m) = decode("nav_msgs/msg/Odometry", ODOMETRY) else {
        panic!()
    };
    assert_header(&m.header, "odom");
    assert_eq!(m.child_frame_id, "base_link");
    assert_eq!(m.pose.pose.position.y, 2.0);
    assert_eq!(m.pose.pose.orientation.w, SQRT_HALF);
    let ramp: Vec<f64> = (0..36).map(|k| k as f64 * 0.5).collect();
    assert_eq!(m.pose.covariance.to_vec(), ramp);
    assert_eq!(
        m.twist.covariance.to_vec(),
        ramp.iter().rev().copied().collect::<Vec<_>>()
    );
    assert_eq!(
        (m.twist.twist.linear.x, m.twist.twist.angular.z),
        (0.5, 0.1)
    );
}

#[test]
fn point_cloud2() {
    let DimosMsg::PointCloud2(m) = decode("sensor_msgs/msg/PointCloud2", POINTCLOUD2) else {
        panic!()
    };
    assert_header(&m.header, "lidar");
    assert_eq!(
        (m.height, m.width, m.point_step, m.row_step),
        (1, 2, 12, 24)
    );
    let names: Vec<&str> = m.fields.iter().map(|f| f.name.as_str()).collect();
    assert_eq!(names, ["x", "y", "z"]);
    assert_eq!(m.fields[2].offset, 8);
    assert!(m.fields.iter().all(|f| f.datatype == 7 && f.count == 1));
    let floats: Vec<f32> = m
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|c| f32::from_le_bytes(*c))
        .collect();
    assert_eq!(floats, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    assert!(!m.is_bigendian && m.is_dense);
}

#[test]
fn imu() {
    let DimosMsg::Imu(m) = decode("sensor_msgs/msg/Imu", IMU) else {
        panic!()
    };
    assert_header(&m.header, "imu");
    assert_eq!(m.orientation.z, SQRT_HALF);
    assert_eq!(
        (
            m.angular_velocity.x,
            m.angular_velocity.y,
            m.angular_velocity.z
        ),
        (0.1, 0.2, 0.3)
    );
    assert_eq!(m.linear_acceleration.z, 9.81);
    assert_eq!(m.orientation_covariance, [0.01; 9]);
    assert_eq!(m.angular_velocity_covariance, [0.02; 9]);
    assert_eq!(m.linear_acceleration_covariance, [0.03; 9]);
}

#[test]
fn tf_message() {
    let DimosMsg::TfMessage(m) = decode("tf2_msgs/msg/TFMessage", TFMESSAGE) else {
        panic!()
    };
    assert_eq!(m.transforms.len(), 2);
    let (first, second) = (&m.transforms[0], &m.transforms[1]);
    assert_header(&first.header, "odom");
    assert_eq!(first.child_frame_id, "base_link");
    assert_eq!(first.transform.translation.x, 1.0);
    assert_eq!(first.transform.rotation.z, SQRT_HALF);
    assert_header(&second.header, "base_link");
    assert_eq!(second.child_frame_id, "lidar");
    assert_eq!(second.transform.translation.z, 0.3);
    assert_eq!(second.transform.rotation.w, 1.0);
}

#[test]
fn translated_messages_encode_as_their_dimos_type() {
    // The LCM fingerprint leads the encoding, so a decode as the named DimOS type proves the pairing.
    let msg = decode("nav_msgs/msg/Odometry", ODOMETRY);
    let DimosMsg::Odometry(expected) = &msg else {
        panic!()
    };
    assert_eq!(
        &lcm_msgs::nav_msgs::Odometry::decode(&msg.encode()).unwrap(),
        &**expected
    );
}

#[test]
fn every_row_names_a_ros2_type_and_its_dds_type_name() {
    for row in translate::TABLE {
        let name = translate::ros2_dds_type_name(row.ros_type).expect("pkg/msg/Name");
        assert!(
            name.ends_with('_') && name.contains("::msg::dds_::"),
            "{name}"
        );
    }
    assert_eq!(
        translate::ros2_dds_type_name("nav_msgs/msg/Odometry").as_deref(),
        Some("nav_msgs::msg::dds_::Odometry_")
    );
    assert_eq!(translate::ros2_dds_type_name("Odometry"), None);
}

#[test]
fn a_truncated_payload_is_an_error_not_a_panic() {
    let translation = translate::lookup("nav_msgs/msg/Odometry").unwrap();
    let full = bytes(ODOMETRY);
    assert!(decode_payload(translation, &full[..40]).is_err());
    assert!(decode_payload(translation, &full[..2]).is_err());
}
