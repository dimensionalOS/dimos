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
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::geometry_msgs::{
    PointStamped, PoseStamped, PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped,
    WrenchStamped,
};
use lcm_msgs::nav_msgs::{OccupancyGrid, Odometry, Path};
use lcm_msgs::sensor_msgs::{
    CameraInfo, CompressedImage, Image, Imu, JointState, Joy, PointCloud2,
};
use lcm_msgs::tf2_msgs::TFMessage;
use lcm_msgs::vision_msgs::{Detection2D, Detection2DArray, Detection3D, Detection3DArray};

/// Extract source time for message types supported by the native fast path.
///
/// The Python recorder also falls back to reception time when a payload does
/// not expose a recognized source timestamp. Keep that behavior until message
/// generation provides a shared Rust timestamp trait.
pub(crate) fn timestamp_for(payload_type: &str, data: &[u8], reception_ts: f64) -> Result<f64> {
    macro_rules! stamped {
        ($message_type:ty) => {{
            let message = <$message_type>::decode(data)
                .with_context(|| format!("invalid LCM {payload_type}"))?;
            (message.header.stamp.sec, message.header.stamp.nsec)
        }};
    }

    let (sec, nsec) = match payload_type {
        "dimos.msgs.geometry_msgs.PointStamped.PointStamped" => stamped!(PointStamped),
        "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped" => stamped!(PoseStamped),
        "dimos.msgs.geometry_msgs.PoseWithCovarianceStamped.PoseWithCovarianceStamped" => {
            stamped!(PoseWithCovarianceStamped)
        }
        "dimos.msgs.geometry_msgs.TwistStamped.TwistStamped" => stamped!(TwistStamped),
        "dimos.msgs.geometry_msgs.TwistWithCovarianceStamped.TwistWithCovarianceStamped" => {
            stamped!(TwistWithCovarianceStamped)
        }
        "dimos.msgs.geometry_msgs.WrenchStamped.WrenchStamped" => stamped!(WrenchStamped),
        "dimos.msgs.nav_msgs.LineSegments3D.LineSegments3D" | "dimos.msgs.nav_msgs.Path.Path" => {
            stamped!(Path)
        }
        "dimos.msgs.nav_msgs.OccupancyGrid.OccupancyGrid" => stamped!(OccupancyGrid),
        "dimos.msgs.nav_msgs.Odometry.Odometry" => stamped!(Odometry),
        "dimos.msgs.sensor_msgs.CameraInfo.CameraInfo" => stamped!(CameraInfo),
        "dimos.msgs.sensor_msgs.CompressedImage.CompressedImage" => stamped!(CompressedImage),
        "dimos.msgs.sensor_msgs.Image.Image" => stamped!(Image),
        "dimos.msgs.sensor_msgs.Imu.Imu" => stamped!(Imu),
        "dimos.msgs.sensor_msgs.JointState.JointState" => stamped!(JointState),
        "dimos.msgs.sensor_msgs.Joy.Joy" => stamped!(Joy),
        "dimos.msgs.sensor_msgs.PointCloud2.PointCloud2" => stamped!(PointCloud2),
        "dimos.msgs.vision_msgs.Detection2D.Detection2D" => stamped!(Detection2D),
        "dimos.msgs.vision_msgs.Detection2DArray.Detection2DArray" => {
            stamped!(Detection2DArray)
        }
        "dimos.msgs.vision_msgs.Detection3D.Detection3D" => stamped!(Detection3D),
        "dimos.msgs.vision_msgs.Detection3DArray.Detection3DArray" => {
            stamped!(Detection3DArray)
        }
        "dimos.msgs.foxglove_msgs.CompressedVideo.CompressedVideo" => {
            let message = CompressedVideo::decode(data)
                .context("invalid LCM foxglove_msgs.CompressedVideo")?;
            (message.timestamp.sec, message.timestamp.nanosec)
        }
        "dimos.msgs.tf2_msgs.TFMessage.TFMessage"
        | "dimos.msgs.geometry_msgs.Transform.Transform" => {
            let message = TFMessage::decode(data).context("invalid LCM TFMessage")?;
            let Some(transform) = message.transforms.first() else {
                return Ok(reception_ts);
            };
            (transform.header.stamp.sec, transform.header.stamp.nsec)
        }
        _ => return Ok(reception_ts),
    };
    Ok(header_timestamp(sec, nsec, reception_ts))
}

pub(crate) fn header_timestamp(sec: i32, nsec: i32, fallback: f64) -> f64 {
    if sec > 0 {
        f64::from(sec) + f64::from(nsec) / 1_000_000_000.0
    } else {
        fallback
    }
}
