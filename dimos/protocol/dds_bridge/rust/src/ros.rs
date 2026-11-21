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

//! ROS 2 message layouts as serde structs, in IDL field order, so CDR decodes them directly.

use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;
use serde_bytes::ByteBuf;

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Int32 {
    pub data: i32,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct UInt32 {
    pub data: u32,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Bool {
    pub data: bool,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    // sequence<octet> read in one slice rather than byte by byte
    pub data: ByteBuf,
    pub is_dense: bool,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TfMessage {
    pub transforms: Vec<TransformStamped>,
}
