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

use anyhow::{Context, Result};
use dimos_generated_messages::dimos_msgs::msg::LineSegments3D;
use dimos_generated_messages::foxglove_msgs::msg::CompressedVideo;
use dimos_generated_messages::geometry_msgs::msg::{
    PointStamped, PoseStamped, PoseWithCovarianceStamped, TransformStamped, TwistStamped,
    TwistWithCovarianceStamped, WrenchStamped,
};
use dimos_generated_messages::nav_msgs::msg::{OccupancyGrid, Odometry, Path};
use dimos_generated_messages::sensor_msgs::msg::{
    CameraInfo, CompressedImage, Image, Imu, JointState, Joy, PointCloud2,
};
use dimos_generated_messages::tf2_msgs::msg::TFMessage;
use dimos_generated_messages::vision_msgs::msg::{
    Detection2D, Detection2DArray, Detection3D, Detection3DArray,
};

use crate::StreamConfig;
use dimos_generated_messages::codec::Message;

/// One decoded and timestamped observation before storage encoding.
#[derive(Debug)]
pub(crate) struct DecodedObservation {
    pub(crate) ts: i64,
    pub(crate) payload: Vec<u8>,
}

/// Decode a transport packet once and normalize it into storage observations.
pub(crate) fn decode(
    stream: &StreamConfig,
    data: &[u8],
    reception_ts: i64,
) -> Result<Vec<DecodedObservation>> {
    if stream.is_tf() {
        return decode_tf(data, reception_ts);
    }
    let ts = source_timestamp(&stream.schema_name, data, reception_ts)?;
    Ok(vec![DecodedObservation {
        ts,
        payload: data.to_vec(),
    }])
}

fn decode_tf(data: &[u8], _reception_ts: i64) -> Result<Vec<DecodedObservation>> {
    let message = TFMessage::decode(data).context("invalid CDR TFMessage")?;
    message
        .transforms
        .into_iter()
        .map(|transform| {
            Ok(DecodedObservation {
                ts: header_timestamp(transform.header.stamp.sec, transform.header.stamp.nanosec)?,
                payload: TFMessage {
                    transforms: vec![transform],
                }
                .encode()?,
            })
        })
        .collect()
}

/// Read source time while transport-decoding common stamped CDR payloads.
///
/// Unknown CDR payloads remain opaque and use reception time, matching the
/// Python recorder's fallback for messages without a recognized timestamp layout.
fn source_timestamp(payload_type: &str, data: &[u8], reception_ts: i64) -> Result<i64> {
    macro_rules! stamped {
        ($message_type:ty) => {{
            let message = <$message_type>::decode(data)
                .with_context(|| format!("invalid CDR {payload_type}"))?;
            (message.header.stamp.sec, message.header.stamp.nanosec)
        }};
    }

    let (sec, nsec) = match payload_type {
        "geometry_msgs/msg/TransformStamped" => stamped!(TransformStamped),
        "dimos_msgs/msg/LineSegments3D" => stamped!(LineSegments3D),
        "geometry_msgs/msg/PointStamped" => stamped!(PointStamped),
        "geometry_msgs/msg/PoseStamped" => stamped!(PoseStamped),
        "geometry_msgs/msg/PoseWithCovarianceStamped" => {
            stamped!(PoseWithCovarianceStamped)
        }
        "geometry_msgs/msg/TwistStamped" => stamped!(TwistStamped),
        "geometry_msgs/msg/TwistWithCovarianceStamped" => {
            stamped!(TwistWithCovarianceStamped)
        }
        "geometry_msgs/msg/WrenchStamped" => stamped!(WrenchStamped),
        "nav_msgs/msg/Path" => {
            stamped!(Path)
        }
        "nav_msgs/msg/OccupancyGrid" => stamped!(OccupancyGrid),
        "nav_msgs/msg/Odometry" => stamped!(Odometry),
        "sensor_msgs/msg/Image" => stamped!(Image),
        "sensor_msgs/msg/CameraInfo" => stamped!(CameraInfo),
        "sensor_msgs/msg/CompressedImage" => stamped!(CompressedImage),
        "sensor_msgs/msg/Imu" => stamped!(Imu),
        "sensor_msgs/msg/JointState" => stamped!(JointState),
        "sensor_msgs/msg/Joy" => stamped!(Joy),
        "sensor_msgs/msg/PointCloud2" => stamped!(PointCloud2),
        "vision_msgs/msg/Detection2D" => stamped!(Detection2D),
        "vision_msgs/msg/Detection2DArray" => {
            stamped!(Detection2DArray)
        }
        "vision_msgs/msg/Detection3D" => stamped!(Detection3D),
        "vision_msgs/msg/Detection3DArray" => {
            stamped!(Detection3DArray)
        }
        "foxglove_msgs/msg/CompressedVideo" => {
            let message = CompressedVideo::decode(data)
                .context("invalid CDR foxglove_msgs.CompressedVideo")?;
            (message.timestamp.sec, message.timestamp.nanosec)
        }
        _ => return Ok(reception_ts),
    };
    header_timestamp(sec, nsec)
}

pub(crate) fn header_timestamp(sec: i32, nanosec: u32) -> Result<i64> {
    anyhow::ensure!(nanosec < 1_000_000_000, "invalid ROS nanosecond field");
    Ok(i64::from(sec) * 1_000_000_000 + i64::from(nanosec))
}
