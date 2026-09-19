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

//! The Go2's DDS types we read and write, field order 1:1 with the unitree_sdk2 IDL
//! (plain FINAL CDR). The on-wire type name uses `::` separators, set per type below;
//! the topic name is given at topic creation.

use cyclonedds_rs::TopicType;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

/// Impl `TopicType` for a keyless ROS2 type with an explicit DDS type name.
macro_rules! ros2_topic_type {
    ($t:ty, $typename:literal) => {
        impl TopicType for $t {
            fn has_key() -> bool {
                false
            }
            fn key_cdr(&self) -> Vec<u8> {
                Vec::new()
            }
            fn force_md5_keyhash() -> bool {
                false
            }
            fn typename() -> std::ffi::CString {
                std::ffi::CString::new($typename).expect("valid type name")
            }
        }
    };
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct StdString {
    pub data: String,
}
ros2_topic_type!(StdString, "std_msgs::msg::dds_::String_");

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Quaternion,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            covariance: [0.0; 36],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}

impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self {
            twist: Twist::default(),
            covariance: [0.0; 36],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}
ros2_topic_type!(Odometry, "nav_msgs::msg::dds_::Odometry_");

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}
ros2_topic_type!(PointCloud2, "sensor_msgs::msg::dds_::PointCloud2_");

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestIdentity {
    pub id: i64,
    pub api_id: i64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestLease {
    pub id: i64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestPolicy {
    pub priority: i32,
    pub noreply: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RequestHeader {
    pub identity: RequestIdentity,
    pub lease: RequestLease,
    pub policy: RequestPolicy,
}

/// `unitree_api::Request_`, the sport RPC envelope.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Request {
    pub header: RequestHeader,
    pub parameter: String,
    pub binary: Vec<u8>,
}
ros2_topic_type!(Request, "unitree_api::msg::dds_::Request_");

impl Request {
    pub fn new(api_id: i64, parameter: impl Into<String>) -> Self {
        Request {
            header: RequestHeader {
                identity: RequestIdentity { id: 0, api_id },
                ..Default::default()
            },
            parameter: parameter.into(),
            binary: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cdr::{CdrLe, Infinite};

    fn roundtrip<T: Serialize + serde::de::DeserializeOwned>(v: &T) {
        let bytes = cdr::serialize::<_, _, CdrLe>(v, Infinite).expect("serialize");
        let _back: T = cdr::deserialize(&bytes).expect("deserialize");
    }

    #[test]
    fn cdr_roundtrips() {
        roundtrip(&Request::new(1004, "{}"));
        roundtrip(&StdString { data: "ON".into() });
        roundtrip(&Odometry::default());
        roundtrip(&PointCloud2 {
            point_step: 16,
            width: 2,
            height: 1,
            fields: vec![PointField {
                name: "x".into(),
                offset: 0,
                datatype: 7,
                count: 1,
            }],
            data: vec![0u8; 32],
            is_dense: true,
            ..Default::default()
        });
    }
}
