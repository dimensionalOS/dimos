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

//! ROS 2 wire structs and explicit conversions from the recorder's LCM input.
//! Field order and integer widths here match the bundled Jazzy .msg definitions.

use anyhow::{ensure, Result};
use lcm_msgs::{geometry_msgs as geometry, nav_msgs as nav, sensor_msgs as sensor};
use serde::{ser::SerializeTuple, Serialize, Serializer};

#[derive(Serialize)]
pub struct Time {
    sec: i32,
    nanosec: u32,
}

#[derive(Serialize)]
pub struct Header {
    stamp: Time,
    frame_id: String,
}

impl TryFrom<lcm_msgs::std_msgs::Header> for Header {
    type Error = anyhow::Error;
    fn try_from(value: lcm_msgs::std_msgs::Header) -> Result<Self> {
        ensure!(
            (0..1_000_000_000).contains(&value.stamp.nsec),
            "invalid header nanoseconds"
        );
        Ok(Self {
            stamp: Time {
                sec: value.stamp.sec,
                nanosec: value.stamp.nsec as u32,
            },
            frame_id: value.frame_id,
        })
    }
}

// Covariance arrays are fixed-length CDR arrays, not length-prefixed sequences.
#[derive(Clone)]
pub struct Covariance([f64; 36]);

impl Serialize for Covariance {
    fn serialize<S: Serializer>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error> {
        let mut tuple = serializer.serialize_tuple(36)?;
        for value in self.0 {
            tuple.serialize_element(&value)?;
        }
        tuple.end()
    }
}

impl From<[f64; 36]> for Covariance {
    fn from(value: [f64; 36]) -> Self {
        Self(value)
    }
}

// Each declaration states the target layout and its source-field conversion.
macro_rules! mapped {
    ($name:ident, $source:ty, $value:ident, { $($field:ident: $ty:ty = $expr:expr),* $(,)? }) => {
        #[derive(Serialize)]
        pub struct $name { $(pub $field: $ty),* }
        impl TryFrom<$source> for $name {
            type Error = anyhow::Error;
            fn try_from($value: $source) -> Result<Self> { Ok(Self { $($field: $expr),* }) }
        }
    };
}

mapped!(Vector3, geometry::Vector3, v, { x: f64 = v.x, y: f64 = v.y, z: f64 = v.z });
mapped!(Point, geometry::Point, v, { x: f64 = v.x, y: f64 = v.y, z: f64 = v.z });
mapped!(Quaternion, geometry::Quaternion, v, {
    x: f64 = v.x, y: f64 = v.y, z: f64 = v.z, w: f64 = v.w,
});
mapped!(Pose, geometry::Pose, v, {
    position: Point = v.position.try_into()?, orientation: Quaternion = v.orientation.try_into()?,
});
mapped!(Twist, geometry::Twist, v, {
    linear: Vector3 = v.linear.try_into()?, angular: Vector3 = v.angular.try_into()?,
});
mapped!(PoseWithCovariance, geometry::PoseWithCovariance, v, {
    pose: Pose = v.pose.try_into()?, covariance: Covariance = v.covariance.into(),
});
mapped!(TwistWithCovariance, geometry::TwistWithCovariance, v, {
    twist: Twist = v.twist.try_into()?, covariance: Covariance = v.covariance.into(),
});
mapped!(PoseStamped, geometry::PoseStamped, v, {
    header: Header = v.header.try_into()?, pose: Pose = v.pose.try_into()?,
});
mapped!(Transform, geometry::Transform, v, {
    translation: Vector3 = v.translation.try_into()?, rotation: Quaternion = v.rotation.try_into()?,
});
mapped!(TransformStamped, geometry::TransformStamped, v, {
    header: Header = v.header.try_into()?, child_frame_id: String = v.child_frame_id,
    transform: Transform = v.transform.try_into()?,
});
mapped!(TFMessage, lcm_msgs::tf2_msgs::TFMessage, v, {
    transforms: Vec<TransformStamped> = v.transforms.into_iter().map(TryInto::try_into).collect::<Result<_>>()?,
});
mapped!(Path, nav::Path, v, {
    header: Header = v.header.try_into()?,
    poses: Vec<PoseStamped> = v.poses.into_iter().map(TryInto::try_into).collect::<Result<_>>()?,
});
mapped!(Odometry, nav::Odometry, v, {
    header: Header = v.header.try_into()?, child_frame_id: String = v.child_frame_id,
    pose: PoseWithCovariance = v.pose.try_into()?, twist: TwistWithCovariance = v.twist.try_into()?,
});
mapped!(Imu, sensor::Imu, v, {
    header: Header = v.header.try_into()?, orientation: Quaternion = v.orientation.try_into()?,
    orientation_covariance: [f64; 9] = v.orientation_covariance,
    angular_velocity: Vector3 = v.angular_velocity.try_into()?,
    angular_velocity_covariance: [f64; 9] = v.angular_velocity_covariance,
    linear_acceleration: Vector3 = v.linear_acceleration.try_into()?,
    linear_acceleration_covariance: [f64; 9] = v.linear_acceleration_covariance,
});
mapped!(Image, sensor::Image, v, {
    header: Header = v.header.try_into()?, height: u32 = v.height.try_into()?,
    width: u32 = v.width.try_into()?, encoding: String = v.encoding,
    is_bigendian: u8 = v.is_bigendian, step: u32 = v.step.try_into()?, data: Vec<u8> = v.data,
});
mapped!(PointField, sensor::PointField, v, {
    name: String = v.name, offset: u32 = v.offset.try_into()?, datatype: u8 = v.datatype,
    count: u32 = v.count.try_into()?,
});
mapped!(PointCloud2, sensor::PointCloud2, v, {
    header: Header = v.header.try_into()?, height: u32 = v.height.try_into()?, width: u32 = v.width.try_into()?,
    fields: Vec<PointField> = v.fields.into_iter().map(TryInto::try_into).collect::<Result<_>>()?,
    is_bigendian: bool = v.is_bigendian, point_step: u32 = v.point_step.try_into()?,
    row_step: u32 = v.row_step.try_into()?, data: Vec<u8> = v.data, is_dense: bool = v.is_dense,
});
mapped!(RegionOfInterest, sensor::RegionOfInterest, v, {
    x_offset: u32 = v.x_offset.try_into()?, y_offset: u32 = v.y_offset.try_into()?,
    height: u32 = v.height.try_into()?, width: u32 = v.width.try_into()?, do_rectify: bool = v.do_rectify,
});
mapped!(CameraInfo, sensor::CameraInfo, v, {
    header: Header = v.header.try_into()?, height: u32 = v.height.try_into()?, width: u32 = v.width.try_into()?,
    distortion_model: String = v.distortion_model, d: Vec<f64> = v.D,
    k: [f64; 9] = v.K, r: [f64; 9] = v.R, p: [f64; 12] = v.P,
    binning_x: u32 = v.binning_x.try_into()?, binning_y: u32 = v.binning_y.try_into()?,
    roi: RegionOfInterest = v.roi.try_into()?,
});
mapped!(JointState, sensor::JointState, v, {
    header: Header = v.header.try_into()?, name: Vec<String> = v.name,
    position: Vec<f64> = v.position, velocity: Vec<f64> = v.velocity, effort: Vec<f64> = v.effort,
});

#[derive(Serialize)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String,
    pub data: Vec<u8>,
}
