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
use dimos_generated_msgs::foxglove_msgs::msg::compressed_video::CompressedVideo;
use dimos_generated_msgs::geometry_msgs::msg::point_stamped::PointStamped;
use dimos_generated_msgs::geometry_msgs::msg::pose_stamped::PoseStamped;
use dimos_generated_msgs::geometry_msgs::msg::pose_with_covariance_stamped::PoseWithCovarianceStamped;
use dimos_generated_msgs::geometry_msgs::msg::twist_stamped::TwistStamped;
use dimos_generated_msgs::geometry_msgs::msg::twist_with_covariance_stamped::TwistWithCovarianceStamped;
use dimos_generated_msgs::geometry_msgs::msg::wrench_stamped::WrenchStamped;
use dimos_generated_msgs::nav_msgs::msg::occupancy_grid::OccupancyGrid;
use dimos_generated_msgs::nav_msgs::msg::odometry::Odometry;
use dimos_generated_msgs::nav_msgs::msg::path::Path;
use dimos_generated_msgs::sensor_msgs::msg::camera_info::CameraInfo;
use dimos_generated_msgs::sensor_msgs::msg::compressed_image::CompressedImage;
use dimos_generated_msgs::sensor_msgs::msg::image::Image;
use dimos_generated_msgs::sensor_msgs::msg::imu::Imu;
use dimos_generated_msgs::sensor_msgs::msg::joint_state::JointState;
use dimos_generated_msgs::sensor_msgs::msg::joy::Joy;
use dimos_generated_msgs::sensor_msgs::msg::point_cloud2::PointCloud2;
use dimos_generated_msgs::tf2_msgs::msg::tf_message::TFMessage;
use dimos_generated_msgs::vision_msgs::msg::detection2_d::Detection2D;
use dimos_generated_msgs::vision_msgs::msg::detection2_d_array::Detection2DArray;
use dimos_generated_msgs::vision_msgs::msg::detection3_d::Detection3D;
use dimos_generated_msgs::vision_msgs::msg::detection3_d_array::Detection3DArray;

use crate::{Codec, StreamConfig, IMAGE_PAYLOAD_TYPE};

/// Transport-decoded payload consumed by storage codecs.
#[derive(Debug)]
pub(crate) enum DecodedPayload {
    /// An CDR payload whose storage codecs operate on its canonical wire form.
    Cdr(Vec<u8>),
    /// UTF-8 JSON after removing the existing String transport envelope.
    Json(Vec<u8>),
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
    if matches!(stream.codec, Codec::Cdr | Codec::Lz4Cdr) && !stream.source_timestamp {
        anyhow::ensure!(
            data.starts_with(&[0, 1, 0, 0]) || data.starts_with(&[0, 0, 0, 0]),
            "Expected plain CDR encapsulation"
        );
        return Ok(vec![DecodedObservation {
            ts: reception_ts,
            payload: DecodedPayload::Cdr(data.to_vec()),
        }]);
    }
    if stream.codec == Codec::Json {
        anyhow::ensure!(
            stream.payload_type == "std_msgs/msg/String",
            "JSON codec requires std_msgs.String"
        );
        let message =
            dimos_module::cdr::decode::<dimos_generated_msgs::std_msgs::msg::string::String>(data)
                .context("invalid String envelope")?;
        let document: serde_json::Value =
            serde_json::from_str(&message.data).context("invalid JSON document")?;
        let ts = if let Some(field) = &stream.timestamp_field {
            let ts = document
                .get(field)
                .and_then(serde_json::Value::as_f64)
                .context("JSON timestamp field must be a number")?;
            anyhow::ensure!(ts.is_finite(), "JSON timestamp must be finite");
            ts
        } else {
            reception_ts
        };
        return Ok(vec![DecodedObservation {
            ts,
            payload: DecodedPayload::Json(message.data.into_bytes()),
        }]);
    }
    if stream.is_tf() {
        return decode_tf(data, reception_ts);
    }
    if stream.payload_type == IMAGE_PAYLOAD_TYPE {
        let image = dimos_module::cdr::decode::<Image>(data).context("invalid CDR Image")?;
        let ts = header_timestamp(
            image.header.stamp.sec,
            image.header.stamp.nanosec,
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
        payload: DecodedPayload::Cdr(data.to_vec()),
    }])
}

fn decode_tf(data: &[u8], reception_ts: f64) -> Result<Vec<DecodedObservation>> {
    let message = dimos_module::cdr::decode::<TFMessage>(data).context("invalid CDR TFMessage")?;
    Ok(message
        .transforms
        .into_iter()
        .map(|transform| DecodedObservation {
            ts: header_timestamp(
                transform.header.stamp.sec,
                transform.header.stamp.nanosec,
                reception_ts,
            ),
            payload: DecodedPayload::Cdr(
                dimos_module::cdr::encode(&TFMessage {
                    transforms: vec![transform],
                })
                .expect("generated TF is serializable"),
            ),
        })
        .collect())
}

/// Read source time while transport-decoding common stamped CDR payloads.
///
/// Unknown CDR payloads remain opaque and use reception time, matching the
/// Python recorder's fallback for messages without a ``ts`` attribute.
fn source_timestamp(payload_type: &str, data: &[u8], reception_ts: f64) -> Result<f64> {
    macro_rules! stamped {
        ($message_type:ty) => {{
            let message = dimos_module::cdr::decode::<$message_type>(data)
                .with_context(|| format!("invalid CDR {payload_type}"))?;
            (message.header.stamp.sec, message.header.stamp.nanosec)
        }};
    }

    let (sec, nsec) = match payload_type {
        "geometry_msgs/msg/PoseWithCovarianceStamped" => stamped!(PoseWithCovarianceStamped),
        "geometry_msgs/msg/TwistWithCovarianceStamped" => stamped!(TwistWithCovarianceStamped),
        "geometry_msgs/msg/WrenchStamped" => stamped!(WrenchStamped),
        "geometry_msgs/msg/PointStamped" => stamped!(PointStamped),
        "geometry_msgs/msg/PoseStamped" => stamped!(PoseStamped),
        "geometry_msgs/msg/TwistStamped" => stamped!(TwistStamped),
        "nav_msgs/msg/Path" => stamped!(Path),
        "dimos_msgs/msg/LineSegments3D" => {
            stamped!(dimos_generated_msgs::dimos_msgs::msg::line_segments3_d::LineSegments3D)
        }
        "nav_msgs/msg/OccupancyGrid" => stamped!(OccupancyGrid),
        "nav_msgs/msg/Odometry" => stamped!(Odometry),
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
            let message = dimos_module::cdr::decode::<CompressedVideo>(data)
                .context("invalid LCM foxglove_msgs.CompressedVideo")?;
            (message.timestamp.sec, message.timestamp.nanosec)
        }
        _ => return Ok(reception_ts),
    };
    Ok(header_timestamp(sec, nsec, reception_ts))
}

pub(crate) fn header_timestamp(sec: i32, nsec: u32, fallback: f64) -> f64 {
    if sec > 0 {
        f64::from(sec) + f64::from(nsec) / 1_000_000_000.0
    } else {
        fallback
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_generated_msgs::std_msgs::msg::string::String as LcmString;

    #[test]
    fn generated_segment_identity_uses_header_and_plain_transform_uses_reception() {
        let segments = dimos_generated_msgs::dimos_msgs::msg::line_segments3_d::LineSegments3D {
            header: dimos_generated_msgs::std_msgs::msg::header::Header {
                stamp: dimos_generated_msgs::builtin_interfaces::msg::time::Time {
                    sec: 42,
                    nanosec: 250_000_000,
                },
                frame_id: "map".into(),
            },
            segments: vec![],
        };
        assert_eq!(
            source_timestamp(
                "dimos_msgs/msg/LineSegments3D",
                &dimos_module::cdr::encode(&segments).unwrap(),
                99.0
            )
            .unwrap(),
            42.25
        );
        let plain = dimos_module::tf::from_isometry(
            "map",
            "child",
            segments.header.stamp,
            dimos_module::nalgebra::Isometry3::identity(),
        )
        .transform;
        assert_eq!(
            source_timestamp(
                "geometry_msgs/msg/Transform",
                &dimos_module::cdr::encode(&plain).unwrap(),
                99.0
            )
            .unwrap(),
            99.0
        );
    }

    #[test]
    fn covariance_and_wrench_stamped_messages_keep_source_time() {
        let header =
            serde_json::json!({"stamp":{"sec":42,"nanosec":250000000},"frame_id":"sensor"});
        let vector = serde_json::json!({"x":1.0,"y":2.0,"z":3.0});
        let pose =
            serde_json::json!({"position":vector,"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}});
        let covariance = vec![0.0; 36];
        let pose: PoseWithCovarianceStamped = serde_json::from_value(
            serde_json::json!({"header":header,"pose":{"pose":pose,"covariance":covariance}}),
        )
        .unwrap();
        let twist: TwistWithCovarianceStamped = serde_json::from_value(serde_json::json!({"header":header,"twist":{"twist":{"linear":vector,"angular":vector},"covariance":covariance}})).unwrap();
        let wrench: WrenchStamped = serde_json::from_value(
            serde_json::json!({"header":header,"wrench":{"force":vector,"torque":vector}}),
        )
        .unwrap();
        for (name, bytes) in [
            (
                "geometry_msgs/msg/PoseWithCovarianceStamped",
                dimos_module::cdr::encode(&pose).unwrap(),
            ),
            (
                "geometry_msgs/msg/TwistWithCovarianceStamped",
                dimos_module::cdr::encode(&twist).unwrap(),
            ),
            (
                "geometry_msgs/msg/WrenchStamped",
                dimos_module::cdr::encode(&wrench).unwrap(),
            ),
        ] {
            assert_eq!(source_timestamp(name, &bytes, 99.0).unwrap(), 42.25);
        }
    }

    fn json_stream(field: Option<&str>) -> StreamConfig {
        StreamConfig {
            port: "events".into(),
            name: "events".into(),
            payload_type: "std_msgs/msg/String".into(),
            codec: Codec::Json,
            timestamp_field: field.map(str::to_string),
            json_schema: None,
            cdr_schema: None,
            source_timestamp: true,
        }
    }

    #[test]
    fn json_codec_preserves_document_and_explicit_source_time() {
        let text = r#"{"sent":42.25,"label":"拿起积木","counter":2147483647}"#;
        let message = LcmString { data: text.into() };
        let mut result = decode(
            &json_stream(Some("sent")),
            &dimos_module::cdr::encode(&message).unwrap(),
            99.0,
        )
        .unwrap();
        let obs = result.pop().unwrap();
        assert_eq!(obs.ts, 42.25);
        let DecodedPayload::Json(data) = obs.payload else {
            panic!("not JSON")
        };
        assert_eq!(data, text.as_bytes());
        assert_eq!(
            decode(
                &json_stream(None),
                &dimos_module::cdr::encode(&message).unwrap(),
                99.0
            )
            .unwrap()[0]
                .ts,
            99.0
        );
    }

    #[test]
    fn json_codec_rejects_malformed_or_invalid_explicit_time() {
        for data in ["not json", r#"{"sent":"42"}"#, "{}", r#"{"sent":NaN}"#] {
            let message = LcmString { data: data.into() };
            assert!(decode(
                &json_stream(Some("sent")),
                &dimos_module::cdr::encode(&message).unwrap(),
                99.0
            )
            .is_err());
        }
    }

    #[test]
    fn ordinary_strings_remain_opaque_and_use_reception_time() {
        let mut stream = json_stream(None);
        stream.codec = Codec::Cdr;
        for data in ["not json", r#"{"ts":42.25}"#] {
            let message = LcmString { data: data.into() };
            let result =
                decode(&stream, &dimos_module::cdr::encode(&message).unwrap(), 99.0).unwrap();
            assert_eq!(result[0].ts, 99.0);
        }
    }
}
