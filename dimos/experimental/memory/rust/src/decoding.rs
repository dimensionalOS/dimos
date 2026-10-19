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
use dimos_generated_messages::dimos_msgs::msg::line_segments3_d::LineSegments3D;
use dimos_generated_messages::foxglove_msgs::msg::compressed_video::CompressedVideo;
use dimos_generated_messages::geometry_msgs::msg::{
    point_stamped::PointStamped, pose_stamped::PoseStamped,
    pose_with_covariance_stamped::PoseWithCovarianceStamped, transform_stamped::TransformStamped,
    twist_stamped::TwistStamped, twist_with_covariance_stamped::TwistWithCovarianceStamped,
    wrench_stamped::WrenchStamped,
};
use dimos_generated_messages::nav_msgs::msg::{
    occupancy_grid::OccupancyGrid, odometry::Odometry, path::Path,
};
use dimos_generated_messages::sensor_msgs::msg::{
    camera_info::CameraInfo, compressed_image::CompressedImage, image::Image, imu::Imu,
    joint_state::JointState, joy::Joy, point_cloud2::PointCloud2,
};
use dimos_generated_messages::tf2_msgs::msg::tf_message::TFMessage;
use dimos_generated_messages::vision_msgs::msg::{
    detection2_d::Detection2D, detection2_d_array::Detection2DArray, detection3_d::Detection3D,
    detection3_d_array::Detection3DArray,
};

use crate::{Codec, StreamConfig};
use dimos_generated_messages::dimos_msgs::msg::{
    contacts::Contacts, region_bounds::RegionBounds, region_line_segments3_d::RegionLineSegments3D,
    region_point_cloud2::RegionPointCloud2,
};
use dimos_generated_messages::std_msgs::msg::string::String as StringMessage;
use dimos_module::cdr;

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
    if stream.codec == Codec::Json {
        anyhow::ensure!(
            stream.schema_name == "std_msgs/msg/String",
            "JSON codec requires std_msgs.String"
        );
        let message = cdr::decode::<StringMessage>(data).context("invalid CDR String envelope")?;
        let document: serde_json::Value =
            serde_json::from_str(&message.data).context("invalid JSON document")?;
        let ts = if let Some(field) = &stream.timestamp_field {
            let ts = document
                .get(field)
                .and_then(serde_json::Value::as_f64)
                .context("JSON timestamp field must be a number")?;
            anyhow::ensure!(ts.is_finite(), "JSON timestamp must be finite");
            anyhow::ensure!(
                ts >= i64::MIN as f64 / 1e9 && ts < i64::MAX as f64 / 1e9,
                "JSON timestamp exceeds signed nanosecond range"
            );
            (ts * 1e9).round() as i64
        } else {
            reception_ts
        };
        return Ok(vec![DecodedObservation {
            ts,
            payload: message.data.into_bytes(),
        }]);
    }
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
    let message = cdr::decode::<TFMessage>(data).context("invalid CDR TFMessage")?;
    message
        .transforms
        .into_iter()
        .map(|transform| {
            Ok(DecodedObservation {
                ts: header_timestamp(transform.header.stamp.sec, transform.header.stamp.nanosec)?,
                payload: cdr::encode(&TFMessage {
                    transforms: vec![transform],
                })?,
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
            let message = cdr::decode::<$message_type>(data)
                .with_context(|| format!("invalid CDR {payload_type}"))?;
            (message.header.stamp.sec, message.header.stamp.nanosec)
        }};
    }

    let (sec, nsec) = match payload_type {
        "geometry_msgs/msg/TransformStamped" => stamped!(TransformStamped),
        "dimos_msgs/msg/LineSegments3D" => stamped!(LineSegments3D),
        "dimos_msgs/msg/RegionBounds" => stamped!(RegionBounds),
        "dimos_msgs/msg/Contacts" => stamped!(Contacts),
        "dimos_msgs/msg/RegionPointCloud2" => {
            let message = cdr::decode::<RegionPointCloud2>(data)?;
            (
                message.cloud.header.stamp.sec,
                message.cloud.header.stamp.nanosec,
            )
        }
        "dimos_msgs/msg/RegionLineSegments3D" => {
            let message = cdr::decode::<RegionLineSegments3D>(data)?;
            (
                message.lines.header.stamp.sec,
                message.lines.header.stamp.nanosec,
            )
        }
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
            let message = cdr::decode::<CompressedVideo>(data)
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

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_generated_messages::std_msgs::msg::string::String as StringMessage;

    fn json_stream(field: Option<&str>) -> StreamConfig {
        StreamConfig {
            port: "events".into(),
            name: "events".into(),
            payload_type: "dimos_generated.std_msgs.msg.String".into(),
            schema_name: "std_msgs/msg/String".into(),
            schema_definition: "string data\n".into(),
            codec: Codec::Json,
            timestamp_field: field.map(str::to_string),
            json_schema: None,
        }
    }

    #[test]
    fn json_codec_preserves_document_and_explicit_source_time() {
        let text = r#"{"sent":42.25,"label":"拿起积木","counter":2147483647}"#;
        let message = StringMessage { data: text.into() };
        let mut result = decode(
            &json_stream(Some("sent")),
            &cdr::encode(&message).unwrap(),
            99_000_000_000,
        )
        .unwrap();
        let obs = result.pop().unwrap();
        assert_eq!(obs.ts, 42_250_000_000);
        let data = obs.payload;
        assert_eq!(data, text.as_bytes());
        assert_eq!(
            decode(
                &json_stream(None),
                &cdr::encode(&message).unwrap(),
                99_000_000_000
            )
            .unwrap()[0]
                .ts,
            99_000_000_000
        );
    }

    #[test]
    fn json_codec_rejects_malformed_or_invalid_explicit_time() {
        for data in ["not json", r#"{"sent":"42"}"#, "{}", r#"{"sent":NaN}"#] {
            let message = StringMessage { data: data.into() };
            assert!(decode(
                &json_stream(Some("sent")),
                &cdr::encode(&message).unwrap(),
                99_000_000_000
            )
            .is_err());
        }
    }

    #[test]
    fn ordinary_strings_remain_opaque_and_use_reception_time() {
        let mut stream = json_stream(None);
        stream.codec = Codec::Cdr;
        for data in ["not json", r#"{"ts":42.25}"#] {
            let message = StringMessage { data: data.into() };
            let result = decode(&stream, &cdr::encode(&message).unwrap(), 99_000_000_000).unwrap();
            assert_eq!(result[0].ts, 99_000_000_000);
        }
    }
    #[test]
    fn region_envelopes_use_nested_source_time_without_losing_signed_id() {
        use dimos_generated_messages::builtin_interfaces::msg::time::Time;
        use dimos_generated_messages::std_msgs::msg::header::Header;
        let header = Header {
            stamp: Time {
                sec: 0,
                nanosec: 34,
            },
            frame_id: "map".into(),
        };
        let cloud = RegionPointCloud2 {
            region_id: -196603,
            cloud: PointCloud2 {
                header: header.clone(),
                height: 1,
                width: 0,
                fields: vec![],
                is_bigendian: false,
                point_step: 12,
                row_step: 0,
                data: vec![],
                is_dense: true,
            },
        };
        let lines = RegionLineSegments3D {
            region_id: cloud.region_id,
            lines: LineSegments3D {
                header,
                segments: vec![],
            },
        };
        for (name, bytes) in [
            (
                "dimos_msgs/msg/RegionPointCloud2",
                cdr::encode(&cloud).unwrap(),
            ),
            (
                "dimos_msgs/msg/RegionLineSegments3D",
                cdr::encode(&lines).unwrap(),
            ),
        ] {
            assert_eq!(source_timestamp(name, &bytes, 99_000_000_000).unwrap(), 34);
        }
        assert_eq!(
            cdr::decode::<RegionPointCloud2>(&cdr::encode(&cloud).unwrap())
                .unwrap()
                .region_id,
            -196603
        );
    }
}
