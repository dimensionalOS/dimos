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
use lcm_msgs::imitation_msgs::EpisodeStatus;
use lcm_msgs::nav_msgs::{OccupancyGrid, Odometry, Path};
use lcm_msgs::sensor_msgs::{
    CameraInfo, CompressedImage, Image, Imu, JointState, Joy, PointCloud2,
};
use lcm_msgs::tf2_msgs::TFMessage;
use lcm_msgs::vision_msgs::{Detection2D, Detection2DArray, Detection3D, Detection3DArray};

use crate::{StreamConfig, IMAGE_PAYLOAD_TYPE};

/// Transport-decoded payload consumed by storage codecs.
#[derive(Debug)]
pub(crate) enum DecodedPayload {
    /// An LCM payload whose storage codecs operate on its canonical wire form.
    Lcm(Vec<u8>),
    /// Images stay typed so JPEG encoding reuses the transport decode.
    Image(Image),
}

/// One decoded and timestamped observation before storage encoding.
#[derive(Debug)]
pub(crate) struct DecodedObservation {
    pub(crate) ts: f64,
    pub(crate) payload: DecodedPayload,
}

/// Decode a transport packet once and normalize it into storage observations.
pub(crate) fn decode(
    stream: &StreamConfig,
    data: &[u8],
    reception_ts: f64,
) -> Result<Vec<DecodedObservation>> {
    if stream.is_tf() {
        return decode_tf(data, reception_ts);
    }
    if stream.payload_type == IMAGE_PAYLOAD_TYPE {
        let image = Image::decode(data).context("invalid LCM Image")?;
        let ts = header_timestamp(
            image.header.stamp.sec,
            image.header.stamp.nsec,
            reception_ts,
        );
        return Ok(vec![DecodedObservation {
            ts,
            payload: DecodedPayload::Image(image),
        }]);
    }

    let ts = source_timestamp(&stream.payload_type, data, reception_ts)?;
    Ok(vec![DecodedObservation {
        ts,
        payload: DecodedPayload::Lcm(data.to_vec()),
    }])
}

fn decode_tf(data: &[u8], reception_ts: f64) -> Result<Vec<DecodedObservation>> {
    let message = TFMessage::decode(data).context("invalid LCM TFMessage")?;
    Ok(message
        .transforms
        .into_iter()
        .map(|transform| DecodedObservation {
            ts: header_timestamp(
                transform.header.stamp.sec,
                transform.header.stamp.nsec,
                reception_ts,
            ),
            payload: DecodedPayload::Lcm(
                TFMessage {
                    transforms: vec![transform],
                }
                .encode(),
            ),
        })
        .collect())
}

/// Read source time while transport-decoding common stamped LCM payloads.
///
/// Unknown LCM payloads remain opaque and use reception time, matching the
/// Python recorder's fallback for messages without a ``ts`` attribute.
fn source_timestamp(payload_type: &str, data: &[u8], reception_ts: f64) -> Result<f64> {
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
        "dimos.msgs.imitation_msgs.EpisodeStatus.EpisodeStatus" => {
            let message =
                EpisodeStatus::decode(data).context("invalid LCM imitation_msgs.EpisodeStatus")?;
            (message.header.stamp.sec, message.header.stamp.nsec)
        }
        "dimos.msgs.geometry_msgs.Transform.Transform" => {
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

#[cfg(test)]
mod tests {
    use lcm_msgs::imitation_msgs::EpisodeStatus;
    use lcm_msgs::std_msgs::{Header, Time};

    use super::source_timestamp;

    #[test]
    fn episode_status_uses_source_timestamp() {
        let status = EpisodeStatus {
            header: Header {
                stamp: Time {
                    sec: 42,
                    nsec: 250_000_000,
                },
                ..Header::default()
            },
            state: "recording".to_string(),
            last_event: "start".to_string(),
            ..EpisodeStatus::default()
        };

        let timestamp = source_timestamp(
            "dimos.msgs.imitation_msgs.EpisodeStatus.EpisodeStatus",
            &status.encode(),
            99.0,
        )
        .expect("generated status should decode");

        assert_eq!(timestamp, 42.25);
    }
}
