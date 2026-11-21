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

//! MJPEG and packed 4:2:2 into the 8-bit I420 the encoder wants.
//!
//! A UVC camera hands out 4:2:2 (MJPEG usually carries 4:2:2, and the raw formats are all 4:2:2), while AV1 is
//! encoded from 4:2:0, so the chroma rows are averaged in pairs on the way through. That averaging is the only
//! picture loss this module adds; the luma is untouched.

/// Planar 8-bit I420, planes tightly packed (stride == width).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct I420 {
    pub y: Vec<u8>,
    pub u: Vec<u8>,
    pub v: Vec<u8>,
    pub width: usize,
    pub height: usize,
}

impl I420 {
    /// A mid-grey frame, which is what a decoder shows for "nothing yet".
    pub fn grey(width: usize, height: usize) -> Self {
        Self {
            y: vec![16u8; width * height],
            u: vec![128u8; width.div_ceil(2) * height.div_ceil(2)],
            v: vec![128u8; width.div_ceil(2) * height.div_ceil(2)],
            width,
            height,
        }
    }

    pub fn uv_width(&self) -> usize {
        self.width.div_ceil(2)
    }

    pub fn uv_height(&self) -> usize {
        self.height.div_ceil(2)
    }
}

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

/// Whether this fourcc is one `packed_422` accepts.
pub fn is_packed_422(fourcc: u32) -> bool {
    packed_422_layout(fourcc).is_some()
}

/// A packed 4:2:2 frame straight from the driver, with its chroma rows averaged down to 4:2:0.
pub fn packed_422(
    frame: &[u8],
    stride: usize,
    width: usize,
    height: usize,
    fourcc: u32,
) -> Result<I420, String> {
    let [y0, u, y1, v] =
        packed_422_layout(fourcc).ok_or("only packed 4:2:2 formats convert without a decoder")?;
    if !width.is_multiple_of(2) || stride < width * 2 || frame.len() < stride * height {
        return Err(format!(
            "frame of {} bytes does not hold {width}x{height} at stride {stride}",
            frame.len()
        ));
    }
    let half_w = width / 2;
    let half_h = height.div_ceil(2);
    let mut out = I420 {
        y: vec![0u8; width * height],
        u: vec![0u8; half_w * half_h],
        v: vec![0u8; half_w * half_h],
        width,
        height,
    };
    // Chroma is summed over each pair of rows and divided once, so an odd last row is not halved away.
    let mut rows_in_pair = vec![0u16; 2 * half_w];
    for row in 0..height {
        let line = &frame[row * stride..row * stride + width * 2];
        if row.is_multiple_of(2) {
            rows_in_pair.fill(0);
        }
        for (x, pixel) in line.as_chunks::<4>().0.iter().enumerate() {
            out.y[row * width + 2 * x] = pixel[y0];
            out.y[row * width + 2 * x + 1] = pixel[y1];
            rows_in_pair[x] += u16::from(pixel[u]);
            rows_in_pair[half_w + x] += u16::from(pixel[v]);
        }
        let last_of_pair = row.is_multiple_of(2) && row + 1 == height;
        if !row.is_multiple_of(2) || last_of_pair {
            let divisor = if last_of_pair { 1 } else { 2 };
            let out_row = row / 2;
            for x in 0..half_w {
                out.u[out_row * half_w + x] = (rows_in_pair[x] / divisor) as u8;
                out.v[out_row * half_w + x] = (rows_in_pair[half_w + x] / divisor) as u8;
            }
        }
    }
    Ok(out)
}

/// An MJPEG frame decoded with libjpeg-turbo, at whatever chroma the JPEG carries, converted to 4:2:0.
pub fn mjpeg(frame: &[u8], decompressor: &mut turbojpeg::Decompressor) -> Result<I420, String> {
    let header = decompressor.read_header(frame).map_err(|e| e.to_string())?;
    let (width, height) = (header.width, header.height);
    let subsamp = header.subsamp;
    let mut image = turbojpeg::YuvImage {
        // align 1 keeps the planes tight, so they can be handed to the encoder as they are when the JPEG is
        // already 4:2:0.
        pixels: vec![
            0u8;
            turbojpeg::yuv_pixels_len(width, 1, height, subsamp)
                .map_err(|e| e.to_string())?
        ],
        width,
        align: 1,
        height,
        subsamp,
    };
    decompressor
        .decompress_to_yuv(frame, image.as_deref_mut())
        .map_err(|e| e.to_string())?;
    let (y_w, y_h) = image.y_size();
    let (uv_w, uv_h) = image.uv_size();
    let (luma, chroma) = image.pixels.split_at(y_w * y_h);
    let (cb, cr) = chroma.split_at(uv_w * uv_h);
    let want_uv_w = width.div_ceil(2);
    let want_uv_h = height.div_ceil(2);
    let (u, v) = match subsamp {
        turbojpeg::Subsamp::Sub2x2 => (cb.to_vec(), cr.to_vec()),
        turbojpeg::Subsamp::Sub2x1 => (halve_rows(cb, uv_w, uv_h), halve_rows(cr, uv_w, uv_h)),
        turbojpeg::Subsamp::Gray => (
            vec![128u8; want_uv_w * want_uv_h],
            vec![128u8; want_uv_w * want_uv_h],
        ),
        other => {
            return Err(format!(
                "a {other:?} JPEG is not one of the chroma layouts a camera sends"
            ))
        }
    };
    Ok(I420 {
        y: luma.to_vec(),
        u,
        v,
        width,
        height,
    })
}

/// Averages each pair of rows, so a `width` x `height` plane becomes `width` x `ceil(height / 2)`.
fn halve_rows(plane: &[u8], width: usize, height: usize) -> Vec<u8> {
    let out_h = height.div_ceil(2);
    let mut out = vec![0u8; width * out_h];
    for row in 0..out_h {
        let top = &plane[row * 2 * width..row * 2 * width + width];
        let bottom = if row * 2 + 1 < height {
            Some(&plane[(row * 2 + 1) * width..(row * 2 + 1) * width + width])
        } else {
            None
        };
        for x in 0..width {
            out[row * width + x] = match bottom {
                Some(bottom) => ((u16::from(top[x]) + u16::from(bottom[x])) / 2) as u8,
                None => top[x],
            };
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::fourcc;

    #[test]
    fn a_packed_422_frame_keeps_its_luma_and_averages_its_chroma_rows() {
        let (width, height) = (4usize, 4usize);
        // UYVY: U Y0 V Y1. Row n gets luma 10n and chroma 100 + n, so the row averaging is visible.
        let mut frame = Vec::new();
        for row in 0..height {
            for _ in 0..width / 2 {
                frame.extend_from_slice(&[
                    100 + row as u8,
                    10 * row as u8,
                    200 + row as u8,
                    10 * row as u8,
                ]);
            }
        }
        let out = packed_422(&frame, width * 2, width, height, fourcc("UYVY").unwrap()).unwrap();
        assert_eq!(out.y[..width], [0, 0, 0, 0], "row 0 luma");
        assert_eq!(out.y[3 * width..], [30, 30, 30, 30], "row 3 luma");
        // Rows 0 and 1 average to 100.5 -> 100; rows 2 and 3 to 102.5 -> 102.
        assert_eq!(out.u[..out.uv_width()], [100, 100]);
        assert_eq!(out.u[out.uv_width()..], [102, 102]);
        assert_eq!(out.v[..out.uv_width()], [200, 200]);
    }

    #[test]
    fn an_odd_height_keeps_its_last_chroma_row_unhalved() {
        // One row of 4:2:2 chroma, 200, must come back as 200 and not 100.
        let (width, height) = (2usize, 1usize);
        let frame = vec![200u8, 50, 200, 50];
        let out = packed_422(&frame, width * 2, width, height, fourcc("UYVY").unwrap()).unwrap();
        assert_eq!(out.u, vec![200]);
        assert_eq!(out.v, vec![200]);
    }

    #[test]
    fn rgb_and_short_frames_are_refused() {
        assert!(packed_422(&[0; 64], 8, 4, 2, fourcc("RGB3").unwrap()).is_err());
        assert!(packed_422(&[0; 8], 8, 4, 2, fourcc("UYVY").unwrap()).is_err());
    }

    #[test]
    fn halving_an_even_plane_averages_and_an_odd_one_copies_the_last_row() {
        let plane = vec![0u8, 0, 10, 10, 40, 40];
        assert_eq!(halve_rows(&plane, 2, 3), vec![5, 5, 40, 40]);
        assert_eq!(halve_rows(&plane[..4], 2, 2), vec![5, 5]);
    }

    #[test]
    fn a_real_jpeg_decodes_to_i420_of_the_right_shape() {
        // Encode a 4:2:0 and a 4:2:2 JPEG with libjpeg-turbo, then decode both back.
        let (width, height) = (32usize, 16usize);
        let grey: Vec<u8> = (0..width * height * 3)
            .map(|i| ((i / 3) % 255) as u8)
            .collect();
        let image = turbojpeg::Image {
            pixels: grey.as_slice(),
            width,
            pitch: width * 3,
            height,
            format: turbojpeg::PixelFormat::RGB,
        };
        let mut decompressor = turbojpeg::Decompressor::new().unwrap();
        for subsamp in [turbojpeg::Subsamp::Sub2x2, turbojpeg::Subsamp::Sub2x1] {
            let mut compressor = turbojpeg::Compressor::new().unwrap();
            compressor.set_subsamp(subsamp).unwrap();
            compressor.set_quality(90).unwrap();
            let jpeg = compressor.compress_to_vec(image).unwrap();
            let out = mjpeg(&jpeg, &mut decompressor).unwrap();
            assert_eq!((out.width, out.height), (width, height), "{subsamp:?}");
            assert_eq!(out.y.len(), width * height, "{subsamp:?}");
            assert_eq!(out.u.len(), width / 2 * height / 2, "{subsamp:?}");
            assert_eq!(out.v.len(), out.u.len(), "{subsamp:?}");
            // The source ramps along x, so the decoded luma must rise too.
            assert!(out.y[0] < out.y[width - 1], "{subsamp:?}");
        }
    }

    #[test]
    fn a_grey_frame_is_full_mid_chroma() {
        let out = I420::grey(8, 6);
        assert!(out.u.iter().all(|&c| c == 128) && out.v.iter().all(|&c| c == 128));
        assert_eq!(out.u.len(), 4 * 3);
    }
}
