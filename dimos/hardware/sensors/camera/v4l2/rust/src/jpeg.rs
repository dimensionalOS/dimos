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
//! CPU JPEG for frames the hardware did not encode: packed 4:2:2 straight into libjpeg-turbo's planar YUV path,
//! with no RGB round trip.

/// Byte offsets of Y0, U, Y1, V within one packed 4:2:2 macropixel.
fn packed_422_layout(fourcc: u32) -> Option<[usize; 4]> {
    match &fourcc.to_le_bytes() {
        b"UYVY" => Some([1, 0, 3, 2]),
        b"YUYV" => Some([0, 1, 2, 3]),
        b"VYUY" => Some([1, 2, 3, 0]),
        b"YVYU" => Some([0, 3, 2, 1]),
        _ => None,
    }
}

/// Encodes a raw packed 4:2:2 frame as JPEG.
pub fn encode_packed_422(
    frame: &[u8],
    stride: usize,
    width: usize,
    height: usize,
    fourcc: u32,
    quality: i32,
) -> Result<Vec<u8>, String> {
    let [y0, u, y1, v] =
        packed_422_layout(fourcc).ok_or("CPU JPEG supports packed 4:2:2 formats only")?;
    if !width.is_multiple_of(2) || stride < width * 2 || frame.len() < stride * height {
        return Err(format!(
            "frame of {} bytes does not hold {width}x{height} at stride {stride}",
            frame.len()
        ));
    }
    let half = width / 2;
    let mut planar = vec![0u8; width * height + 2 * half * height];
    let (luma, chroma) = planar.split_at_mut(width * height);
    let (cb, cr) = chroma.split_at_mut(half * height);
    for row in 0..height {
        let line = &frame[row * stride..row * stride + width * 2];
        for (x, pixel) in line.as_chunks::<4>().0.iter().enumerate() {
            luma[row * width + 2 * x] = pixel[y0];
            luma[row * width + 2 * x + 1] = pixel[y1];
            cb[row * half + x] = pixel[u];
            cr[row * half + x] = pixel[v];
        }
    }
    let image = turbojpeg::YuvImage {
        pixels: planar.as_slice(),
        width,
        align: 1,
        height,
        subsamp: turbojpeg::Subsamp::Sub2x1,
    };
    let mut compressor = turbojpeg::Compressor::new().map_err(|e| e.to_string())?;
    compressor.set_quality(quality).map_err(|e| e.to_string())?;
    compressor
        .set_subsamp(turbojpeg::Subsamp::Sub2x1)
        .map_err(|e| e.to_string())?;
    compressor
        .compress_yuv_to_vec(image)
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::fourcc;

    #[test]
    fn a_flat_grey_uyvy_frame_round_trips_through_jpeg() {
        let (width, height) = (64, 48);
        let frame: Vec<u8> = [128u8, 100, 128, 100].repeat(width / 2 * height);
        let jpeg = encode_packed_422(
            &frame,
            width * 2,
            width,
            height,
            fourcc("UYVY").unwrap(),
            90,
        )
        .unwrap();
        let decoded: turbojpeg::Image<Vec<u8>> =
            turbojpeg::decompress(&jpeg, turbojpeg::PixelFormat::GRAY).unwrap();
        assert_eq!((decoded.width, decoded.height), (width, height));
        assert!(decoded.pixels.iter().all(|&y| (98..=102).contains(&y)));
    }

    #[test]
    fn rgb_formats_are_refused() {
        assert!(encode_packed_422(&[0; 64], 8, 4, 2, fourcc("RGB3").unwrap(), 90).is_err());
    }
}
