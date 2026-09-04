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
use lz4_flex::frame::FrameEncoder;
use turbojpeg::{PixelFormat, Subsamp};

use crate::decoding::{DecodedObservation, DecodedPayload};
use crate::{Codec, StreamConfig};

pub(crate) const JPEG_QUALITY: i32 = 50;

#[derive(Debug)]
pub(crate) struct StoredObservation {
    pub(crate) ts: f64,
    pub(crate) data: Vec<u8>,
}

pub(crate) fn encode(
    stream: &StreamConfig,
    observation: DecodedObservation,
) -> Result<StoredObservation> {
    let data = match stream.codec {
        Codec::Lcm => lcm_encode(observation.payload),
        Codec::Lz4Lcm => lz4_frame(&lcm_encode(observation.payload))?,
        Codec::Jpeg => jpeg_encode(observation.payload)?,
    };
    Ok(StoredObservation {
        ts: observation.ts,
        data,
    })
}

fn lcm_encode(payload: DecodedPayload) -> Vec<u8> {
    match payload {
        DecodedPayload::Lcm(data) => data,
        DecodedPayload::Image(image) => image.encode(),
    }
}

fn jpeg_encode(payload: DecodedPayload) -> Result<Vec<u8>> {
    let DecodedPayload::Image(mut image) = payload else {
        return Err(anyhow!("JPEG storage codec requires an Image payload"));
    };
    if image.encoding == "jpeg" {
        return Ok(image.encode());
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
    Ok(image.encode())
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
