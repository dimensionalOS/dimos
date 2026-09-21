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

//! The MCAP recording contract: input type, output schema, and native converter.

use anyhow::{anyhow, ensure, Result};
use lcm_msgs::{nav_msgs::Path, sensor_msgs::Image, tf2_msgs::TFMessage};
use serde::Serialize;

use crate::decoding::header_timestamp;
use crate::encoding::{jpeg_image, StoredObservation};
use crate::{ros, Codec, StreamConfig};

pub struct Mapping {
    pub payload_type: &'static str,
    pub codec: Codec,
    pub schema_name: &'static str,
    pub schema_encoding: &'static str,
    pub schema: &'static str,
    encode: fn(&[u8], f64) -> Result<Vec<StoredObservation>>,
}

fn cdr<T: Serialize>(value: &T, ts: f64) -> Result<Vec<StoredObservation>> {
    Ok(vec![StoredObservation {
        ts,
        data: cdr::serialize::<_, _, cdr::CdrLe>(value, cdr::Infinite)?,
    }])
}

macro_rules! standard {
    ($package:ident, $name:ident) => {
        Mapping {
            payload_type: concat!(
                "dimos.msgs.",
                stringify!($package),
                ".",
                stringify!($name),
                ".",
                stringify!($name)
            ),
            codec: Codec::Cdr,
            schema_name: concat!(stringify!($package), "/msg/", stringify!($name)),
            schema_encoding: "ros2msg",
            schema: include_str!(concat!("../schemas/", stringify!($name), ".msg")),
            encode: |data, reception_ts| {
                let message = lcm_msgs::$package::$name::decode(data)?;
                let ts = header_timestamp(
                    message.header.stamp.sec,
                    message.header.stamp.nsec,
                    reception_ts,
                );
                cdr(&ros::$name::try_from(message)?, ts)
            },
        }
    };
}

pub static MAPPINGS: &[Mapping] = &[
    standard!(sensor_msgs, Imu),
    standard!(sensor_msgs, CameraInfo),
    standard!(sensor_msgs, JointState),
    standard!(sensor_msgs, PointCloud2),
    standard!(geometry_msgs, PoseStamped),
    standard!(nav_msgs, Odometry),
    standard!(nav_msgs, Path),
    Mapping {
        payload_type: "dimos.msgs.sensor_msgs.Image.Image",
        codec: Codec::Cdr,
        schema_name: "sensor_msgs/msg/Image",
        schema_encoding: "ros2msg",
        schema: include_str!("../schemas/Image.msg"),
        encode: |data, reception_ts| {
            let image = Image::decode(data)?;
            ensure!(
                image.encoding != "jpeg",
                "JPEG input requires the jpeg MCAP codec"
            );
            validate_image_layout(&image)?;
            let ts = header_timestamp(
                image.header.stamp.sec,
                image.header.stamp.nsec,
                reception_ts,
            );
            cdr(&ros::Image::try_from(image)?, ts)
        },
    },
    Mapping {
        payload_type: "dimos.msgs.sensor_msgs.Image.Image",
        codec: Codec::RosJpeg,
        schema_name: "sensor_msgs/msg/CompressedImage",
        schema_encoding: "ros2msg",
        schema: include_str!("../schemas/CompressedImage.msg"),
        encode: |data, reception_ts| {
            let mut image = Image::decode(data)?;
            let ts = header_timestamp(
                image.header.stamp.sec,
                image.header.stamp.nsec,
                reception_ts,
            );
            let format = if image.encoding == "jpeg" {
                "jpeg".to_string()
            } else {
                let channels = match image.encoding.as_str() {
                    "rgb8" | "bgr8" => 3,
                    "rgba8" | "bgra8" => 4,
                    "mono8" => 1,
                    _ => {
                        return Err(anyhow!(
                            "JPEG MCAP encoding requires an 8-bit color or mono8 image"
                        ))
                    }
                };
                validate_image_layout(&image)?;
                let row_size = usize::try_from(image.width)?
                    .checked_mul(channels)
                    .ok_or_else(|| anyhow!("image row size overflow"))?;
                let pitch = usize::try_from(image.step)?;
                ensure!(pitch >= row_size && pitch > 0, "invalid JPEG image stride");
                // TurboJPEG consumes packed pixels; remove any transport row padding.
                image.data = image
                    .data
                    .chunks_exact(pitch)
                    .flat_map(|row| row[..row_size].iter().copied())
                    .collect();
                image.step = row_size.try_into()?;
                let compressed = if image.encoding.starts_with("rgb") {
                    "rgb8"
                } else {
                    "bgr8"
                };
                if channels == 1 {
                    format!("{}; jpeg", image.encoding)
                } else {
                    format!("{}; jpeg {compressed}", image.encoding)
                }
            };
            let image = jpeg_image(image)?;
            cdr(
                &ros::CompressedImage {
                    header: image.header.try_into()?,
                    format,
                    data: image.data,
                },
                ts,
            )
        },
    },
    Mapping {
        payload_type: "dimos.msgs.tf2_msgs.TFMessage.TFMessage",
        codec: Codec::Cdr,
        schema_name: "tf2_msgs/msg/TFMessage",
        schema_encoding: "ros2msg",
        schema: include_str!("../schemas/TFMessage.msg"),
        encode: |data, reception_ts| {
            TFMessage::decode(data)?
                .transforms
                .into_iter()
                .map(|transform| {
                    let ts = header_timestamp(
                        transform.header.stamp.sec,
                        transform.header.stamp.nsec,
                        reception_ts,
                    );
                    let message = ros::TFMessage::try_from(TFMessage {
                        transforms: vec![transform],
                    })?;
                    Ok(StoredObservation {
                        ts,
                        data: cdr::serialize::<_, _, cdr::CdrLe>(&message, cdr::Infinite)?,
                    })
                })
                .collect()
        },
    },
    Mapping {
        payload_type: "dimos.msgs.nav_msgs.LineSegments3D.LineSegments3D",
        codec: Codec::Json,
        schema_name: "dimos.LineSegments3D",
        schema_encoding: "jsonschema",
        schema: include_str!("../schemas/LineSegments3D.json"),
        encode: |data, reception_ts| {
            let message = Path::decode(data)?;
            let ts = header_timestamp(
                message.header.stamp.sec,
                message.header.stamp.nsec,
                reception_ts,
            );
            ensure!(
                message.poses.len().is_multiple_of(2),
                "LineSegments3D requires pose pairs"
            );
            let mut segments = Vec::new();
            let mut weights = Vec::new();
            for pair in message.poses.as_chunks::<2>().0 {
                let points = pair
                    .iter()
                    .map(|pose| {
                        let p = &pose.pose.position;
                        [p.x, p.y, p.z]
                    })
                    .collect::<Vec<_>>();
                let weight = pair[0].pose.orientation.w;
                ensure!(
                    weight.is_finite() && points.iter().flatten().all(|v| v.is_finite()),
                    "nonfinite LineSegments3D value"
                );
                segments.push(points);
                weights.push(weight);
            }
            Ok(vec![StoredObservation {
                ts,
                data: serde_json::to_vec(&serde_json::json!({
                    "ts": ts, "frame_id": message.header.frame_id,
                    "segments": segments, "weights": weights,
                }))?,
            }])
        },
    },
];

fn validate_image_layout(image: &Image) -> Result<()> {
    let height = usize::try_from(image.height)?;
    let step = usize::try_from(image.step)?;
    ensure!(
        image.width >= 0 && image.is_bigendian <= 1,
        "invalid Image dimensions or endianness"
    );
    ensure!(
        height.checked_mul(step) == Some(image.data.len()),
        "Image data length does not match height * step"
    );
    Ok(())
}

pub fn mapping(stream: &StreamConfig) -> Result<&'static Mapping> {
    MAPPINGS
        .iter()
        .find(|mapping| {
            mapping.payload_type == stream.payload_type && mapping.codec == stream.codec
        })
        .ok_or_else(|| {
            anyhow!(
                "no MCAP mapping for {} with codec {:?}",
                stream.payload_type,
                stream.codec
            )
        })
}

pub fn encode(
    stream: &StreamConfig,
    data: &[u8],
    reception_ts: f64,
) -> Result<Vec<StoredObservation>> {
    (mapping(stream)?.encode)(data, reception_ts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{store::RecordingStoreConfig, RecorderConfig, RecorderEngine};
    use std::sync::Arc;

    fn stream(name: &str, codec: Codec) -> StreamConfig {
        StreamConfig {
            port: name.into(),
            name: name.into(),
            payload_type: crate::IMAGE_PAYLOAD_TYPE.into(),
            codec,
        }
    }

    #[test]
    fn invalid_mapping_preserves_existing_artifact() {
        let file = tempfile::NamedTempFile::new().unwrap();
        std::fs::write(file.path(), b"keep me").unwrap();
        let result = RecorderEngine::start(RecorderConfig {
            store: RecordingStoreConfig::Mcap {
                path: file.path().to_string_lossy().into(),
            },
            encoding_threads: 1,
            streams: vec![stream("image", Codec::Lcm)],
        });
        assert!(result.is_err());
        assert_eq!(std::fs::read(file.path()).unwrap(), b"keep me");
    }

    #[test]
    fn rejects_depth_jpeg_and_malformed_raw_images() {
        let mut image = Image {
            height: 1,
            width: 2,
            step: 4,
            encoding: "16UC1".into(),
            data: vec![0, 1, 0, 2],
            ..Default::default()
        };
        assert!(encode(&stream("image", Codec::Cdr), &image.encode(), 1.0).is_ok());
        assert!(encode(&stream("image", Codec::RosJpeg), &image.encode(), 1.0).is_err());
        image.data.pop();
        assert!(encode(&stream("image", Codec::Cdr), &image.encode(), 1.0).is_err());
        image.data.push(2);
        image.header.stamp.nsec = 1_000_000_000;
        assert!(encode(&stream("image", Codec::Cdr), &image.encode(), 1.0).is_err());
    }

    // Python prepares real transport bytes and independently decodes this output.
    // Kept test-only so interop coverage needs no extra production entry point.
    #[test]
    #[ignore = "invoked by test_mcap_recording_e2e.py with an input directory"]
    fn write_interop_fixture() {
        let directory = std::path::PathBuf::from(std::env::var("DIMOS_MCAP_INTEROP_DIR").unwrap());
        let config: RecorderConfig =
            serde_json::from_slice(&std::fs::read(directory.join("config.json")).unwrap()).unwrap();
        let streams = config.streams.clone();
        let engine = RecorderEngine::start(config).unwrap();
        let handle = engine.handle();
        for stream in streams {
            let data = std::fs::read(directory.join(format!("{}.lcm", stream.name))).unwrap();
            handle.record(Arc::new(stream), &data);
        }
        let stats = engine.shutdown().unwrap();
        assert_eq!(stats.encode_errors, 0);
        assert!(stats.written > 0);
    }
}
