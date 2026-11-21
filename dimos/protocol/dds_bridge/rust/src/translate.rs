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

//! The hardcoded table from ROS 2 CDR types on DDS to DimOS (LCM-encoded) messages.

use std::io;

use lcm_msgs::{geometry_msgs, nav_msgs, sensor_msgs, std_msgs, tf2_msgs};
use serde::de::DeserializeOwned;

use crate::ros;

/// A translated sample, ready to LCM-encode onto its DimOS stream.
#[derive(Debug, Clone, PartialEq)]
pub enum DimosMsg {
    Int32(std_msgs::Int32),
    UInt32(std_msgs::UInt32),
    Bool(std_msgs::Bool),
    Twist(geometry_msgs::Twist),
    PoseStamped(geometry_msgs::PoseStamped),
    // Boxed: these are hundreds of bytes of covariance, the rest a few pointers.
    Odometry(Box<nav_msgs::Odometry>),
    PointCloud2(sensor_msgs::PointCloud2),
    Imu(Box<sensor_msgs::Imu>),
    TfMessage(tf2_msgs::TFMessage),
}

impl DimosMsg {
    pub fn encode(&self) -> Vec<u8> {
        match self {
            DimosMsg::Int32(m) => m.encode(),
            DimosMsg::UInt32(m) => m.encode(),
            DimosMsg::Bool(m) => m.encode(),
            DimosMsg::Twist(m) => m.encode(),
            DimosMsg::PoseStamped(m) => m.encode(),
            DimosMsg::Odometry(m) => m.encode(),
            DimosMsg::PointCloud2(m) => m.encode(),
            DimosMsg::Imu(m) => m.encode(),
            DimosMsg::TfMessage(m) => m.encode(),
        }
    }
}

/// One row of the table: a ROS 2 type, the DimOS message it becomes, and the decoder between them.
pub struct Translation {
    /// The ROS 2 interface name, e.g. `nav_msgs/msg/Odometry`.
    pub ros_type: &'static str,
    /// The DimOS message name, the type suffix of the stream's channel.
    pub dimos_type: &'static str,
    /// Decodes a CDR body (encapsulation header already stripped).
    pub decode: fn(&[u8], bool) -> io::Result<DimosMsg>,
}

pub const TABLE: &[Translation] = &[
    Translation {
        ros_type: "std_msgs/msg/Int32",
        dimos_type: std_msgs::Int32::NAME,
        decode: decode_int32,
    },
    Translation {
        ros_type: "std_msgs/msg/UInt32",
        dimos_type: std_msgs::UInt32::NAME,
        decode: decode_uint32,
    },
    Translation {
        ros_type: "std_msgs/msg/Bool",
        dimos_type: std_msgs::Bool::NAME,
        decode: decode_bool,
    },
    Translation {
        ros_type: "geometry_msgs/msg/Twist",
        dimos_type: geometry_msgs::Twist::NAME,
        decode: decode_twist,
    },
    Translation {
        ros_type: "geometry_msgs/msg/PoseStamped",
        dimos_type: geometry_msgs::PoseStamped::NAME,
        decode: decode_pose_stamped,
    },
    Translation {
        ros_type: "nav_msgs/msg/Odometry",
        dimos_type: nav_msgs::Odometry::NAME,
        decode: decode_odometry,
    },
    Translation {
        ros_type: "sensor_msgs/msg/PointCloud2",
        dimos_type: sensor_msgs::PointCloud2::NAME,
        decode: decode_point_cloud2,
    },
    Translation {
        ros_type: "sensor_msgs/msg/Imu",
        dimos_type: sensor_msgs::Imu::NAME,
        decode: decode_imu,
    },
    Translation {
        ros_type: "tf2_msgs/msg/TFMessage",
        dimos_type: tf2_msgs::TFMessage::NAME,
        decode: decode_tf_message,
    },
];

pub fn lookup(ros_type: &str) -> Option<&'static Translation> {
    TABLE.iter().find(|t| t.ros_type == ros_type)
}

pub fn supported_types() -> Vec<&'static str> {
    TABLE.iter().map(|t| t.ros_type).collect()
}

/// The DDS type name ROS 2 registers for an interface: `pkg/msg/Name` -> `pkg::msg::dds_::Name_`.
pub fn ros2_dds_type_name(ros_type: &str) -> Option<String> {
    let parts: Vec<&str> = ros_type.split('/').collect();
    match parts.as_slice() {
        [package, kind, name] if !package.is_empty() && !kind.is_empty() && !name.is_empty() => {
            Some(format!("{package}::{kind}::dds_::{name}_"))
        }
        _ => None,
    }
}

/// Decode a full serialized payload: the 4-byte encapsulation header, then the CDR body.
pub fn decode_payload(translation: &Translation, payload: &[u8]) -> io::Result<DimosMsg> {
    if payload.len() < 4 {
        return Err(invalid(format!(
            "payload of {} bytes has no encapsulation header",
            payload.len()
        )));
    }
    let little_endian = match [payload[0], payload[1]] {
        [0x00, 0x01] => true,
        [0x00, 0x00] => false,
        other => {
            return Err(invalid(format!(
                "unsupported CDR encapsulation {other:02x?}"
            )))
        }
    };
    (translation.decode)(&payload[4..], little_endian)
}

fn invalid(message: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

fn cdr<T: DeserializeOwned>(body: &[u8], little_endian: bool) -> io::Result<T> {
    let decoded = if little_endian {
        cdr_encoding::from_bytes::<T, byteorder::LittleEndian>(body)
    } else {
        cdr_encoding::from_bytes::<T, byteorder::BigEndian>(body)
    };
    decoded
        .map(|(value, _consumed)| value)
        .map_err(|e| invalid(format!("CDR decode: {e}")))
}

fn header(h: ros::Header) -> std_msgs::Header {
    // ROS 2 headers carry no seq; DimOS readers ignore it.
    std_msgs::Header {
        seq: 0,
        stamp: std_msgs::Time {
            sec: h.stamp.sec,
            nsec: h.stamp.nanosec as i32,
        },
        frame_id: h.frame_id,
    }
}

fn vector3(v: ros::Vector3) -> geometry_msgs::Vector3 {
    geometry_msgs::Vector3 {
        x: v.x,
        y: v.y,
        z: v.z,
    }
}

fn quaternion(q: ros::Quaternion) -> geometry_msgs::Quaternion {
    geometry_msgs::Quaternion {
        x: q.x,
        y: q.y,
        z: q.z,
        w: q.w,
    }
}

fn pose(p: ros::Pose) -> geometry_msgs::Pose {
    geometry_msgs::Pose {
        position: geometry_msgs::Point {
            x: p.position.x,
            y: p.position.y,
            z: p.position.z,
        },
        orientation: quaternion(p.orientation),
    }
}

fn twist(t: ros::Twist) -> geometry_msgs::Twist {
    geometry_msgs::Twist {
        linear: vector3(t.linear),
        angular: vector3(t.angular),
    }
}

fn decode_int32(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::Int32 = cdr(body, le)?;
    Ok(DimosMsg::Int32(std_msgs::Int32 { data: m.data }))
}

fn decode_uint32(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::UInt32 = cdr(body, le)?;
    // LCM has no unsigned type: the DimOS UInt32 carries the same 32 bits in an int32.
    Ok(DimosMsg::UInt32(std_msgs::UInt32 {
        data: m.data as i32,
    }))
}

fn decode_bool(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::Bool = cdr(body, le)?;
    Ok(DimosMsg::Bool(std_msgs::Bool { data: m.data }))
}

fn decode_twist(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::Twist = cdr(body, le)?;
    Ok(DimosMsg::Twist(twist(m)))
}

fn decode_pose_stamped(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::PoseStamped = cdr(body, le)?;
    Ok(DimosMsg::PoseStamped(geometry_msgs::PoseStamped {
        header: header(m.header),
        pose: pose(m.pose),
    }))
}

fn decode_odometry(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::Odometry = cdr(body, le)?;
    Ok(DimosMsg::Odometry(Box::new(nav_msgs::Odometry {
        header: header(m.header),
        child_frame_id: m.child_frame_id,
        pose: geometry_msgs::PoseWithCovariance {
            pose: pose(m.pose.pose),
            covariance: m.pose.covariance,
        },
        twist: geometry_msgs::TwistWithCovariance {
            twist: twist(m.twist.twist),
            covariance: m.twist.covariance,
        },
    })))
}

fn decode_point_cloud2(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::PointCloud2 = cdr(body, le)?;
    Ok(DimosMsg::PointCloud2(sensor_msgs::PointCloud2 {
        header: header(m.header),
        height: m.height as i32,
        width: m.width as i32,
        fields: m
            .fields
            .into_iter()
            .map(|f| sensor_msgs::PointField {
                name: f.name,
                offset: f.offset as i32,
                datatype: f.datatype,
                count: f.count as i32,
            })
            .collect(),
        is_bigendian: m.is_bigendian,
        point_step: m.point_step as i32,
        row_step: m.row_step as i32,
        data: m.data.into_vec(),
        is_dense: m.is_dense,
    }))
}

fn decode_imu(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::Imu = cdr(body, le)?;
    Ok(DimosMsg::Imu(Box::new(sensor_msgs::Imu {
        header: header(m.header),
        orientation: quaternion(m.orientation),
        orientation_covariance: m.orientation_covariance,
        angular_velocity: vector3(m.angular_velocity),
        angular_velocity_covariance: m.angular_velocity_covariance,
        linear_acceleration: vector3(m.linear_acceleration),
        linear_acceleration_covariance: m.linear_acceleration_covariance,
    })))
}

fn decode_tf_message(body: &[u8], le: bool) -> io::Result<DimosMsg> {
    let m: ros::TfMessage = cdr(body, le)?;
    Ok(DimosMsg::TfMessage(tf2_msgs::TFMessage {
        transforms: m
            .transforms
            .into_iter()
            .map(|t| geometry_msgs::TransformStamped {
                header: header(t.header),
                child_frame_id: t.child_frame_id,
                transform: geometry_msgs::Transform {
                    translation: vector3(t.transform.translation),
                    rotation: quaternion(t.transform.rotation),
                },
            })
            .collect(),
    }))
}
