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

use std::io::Write;

use anyhow::{anyhow, Context, Result};
use lcm_msgs::sensor_msgs::Image;
use lcm_msgs::tf2_msgs::TFMessage;
use lz4_flex::frame::FrameEncoder;
use turbojpeg::{PixelFormat, Subsamp};

use crate::timestamp::{header_timestamp, timestamp_for};
use crate::{Codec, StreamConfig};

pub(crate) const JPEG_QUALITY: i32 = 50;

#[derive(Debug)]
pub(crate) struct StoredObservation {
    pub(crate) ts: f64,
    pub(crate) data: Vec<u8>,
}

pub(crate) fn encode(
    stream: &StreamConfig,
    data: &[u8],
    reception_ts: f64,
    stores_wire_bytes: bool,
) -> Result<Vec<StoredObservation>> {
    if stores_wire_bytes {
        return Ok(vec![StoredObservation {
            ts: timestamp_for(&stream.payload_type, data, reception_ts)?,
            data: data.to_vec(),
        }]);
    }
    if stream.is_tf() {
        return encode_tf(stream, data, reception_ts);
    }

    let (ts, data) = match stream.codec {
        Codec::Lcm => (
            timestamp_for(&stream.payload_type, data, reception_ts)?,
            data.to_vec(),
        ),
        Codec::Lz4Lcm => (
            timestamp_for(&stream.payload_type, data, reception_ts)?,
            lz4_frame(data)?,
        ),
        Codec::Jpeg => encode_jpeg(data, reception_ts)?,
    };
    Ok(vec![StoredObservation { ts, data }])
}

fn encode_tf(
    stream: &StreamConfig,
    data: &[u8],
    reception_ts: f64,
) -> Result<Vec<StoredObservation>> {
    if stream.codec != Codec::Lcm {
        return Err(anyhow!("tf only supports the lcm codec"));
    }
    let message = TFMessage::decode(data).context("invalid LCM TFMessage")?;
    Ok(message
        .transforms
        .into_iter()
        .map(|transform| StoredObservation {
            ts: header_timestamp(
                transform.header.stamp.sec,
                transform.header.stamp.nsec,
                reception_ts,
            ),
            data: TFMessage {
                transforms: vec![transform],
            }
            .encode(),
        })
        .collect())
}

pub(crate) fn encode_jpeg(data: &[u8], reception_ts: f64) -> Result<(f64, Vec<u8>)> {
    let mut image = Image::decode(data).context("invalid LCM Image")?;
    let ts = header_timestamp(
        image.header.stamp.sec,
        image.header.stamp.nsec,
        reception_ts,
    );
    if image.encoding == "jpeg" {
        return Ok((ts, data.to_vec()));
    }

    let format = pixel_format(&image.encoding)?;
    let pixels = normalize_pixels(&image)?;
    let source = turbojpeg::Image {
        pixels: pixels.as_slice(),
        width: image.width as usize,
        pitch: image.width as usize * format.size(),
        height: image.height as usize,
        format,
    };
    let jpeg = turbojpeg::compress(source, JPEG_QUALITY, Subsamp::Sub2x1)
        .context("TurboJPEG compression failed")?;
    image.encoding = "jpeg".to_string();
    image.is_bigendian = 0;
    image.step = 0;
    image.data = jpeg.to_vec();
    Ok((ts, image.encode()))
}

fn pixel_format(encoding: &str) -> Result<PixelFormat> {
    match encoding {
        "rgb8" => Ok(PixelFormat::RGB),
        "bgr8" => Ok(PixelFormat::BGR),
        "rgba8" => Ok(PixelFormat::RGBA),
        "bgra8" => Ok(PixelFormat::BGRA),
        "mono8" | "mono16" | "16UC1" | "16SC1" => Ok(PixelFormat::GRAY),
        other => Err(anyhow!(
            "JPEG codec does not support image encoding {other:?}"
        )),
    }
}

fn normalize_pixels(image: &Image) -> Result<Vec<u8>> {
    match image.encoding.as_str() {
        "mono16" | "16UC1" | "16SC1" => {
            if !image.data.len().is_multiple_of(2) {
                return Err(anyhow!("16-bit image has an odd byte count"));
            }
            let high_byte = usize::from(image.is_bigendian == 0);
            Ok(image
                .data
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pixel| pixel[high_byte])
                .collect())
        }
        _ => Ok(image.data.clone()),
    }
}

pub(crate) fn lz4_frame(data: &[u8]) -> Result<Vec<u8>> {
    let mut encoder = FrameEncoder::new(Vec::new());
    encoder.write_all(data).context("LZ4 compression failed")?;
    encoder.finish().context("LZ4 frame finalization failed")
}
