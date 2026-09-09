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

//! Pinhole unprojection of a depth image into packed xyz+intensity points.
//!
//! Kept free of transport and module types so the geometry can be unit tested
//! on plain structs.

use lcm_msgs::sensor_msgs::{CameraInfo, Image};
use std::fmt;

/// Bytes per point in the emitted buffer: x, y, z, intensity, all f32.
pub const POINT_STEP: usize = 16;

pub struct Params {
    /// Keep every Nth pixel on each axis.
    pub decimation: usize,
    pub min_range_m: f32,
    pub max_range_m: f32,
    /// Multiplier onto metres for integer depth. Float depth is already metres
    /// and ignores this.
    pub depth_scale: f32,
}

#[derive(Debug, PartialEq)]
pub enum UnprojectError {
    /// Colour, or a depth encoding with more than one channel.
    NotSingleChannelDepth(String),
    /// `data` is too short for `height * width`.
    ShortBuffer,
    /// A zero focal length would divide every point to infinity.
    DegenerateIntrinsics,
}

impl fmt::Display for UnprojectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotSingleChannelDepth(enc) => {
                write!(f, "{enc} is not a single-channel depth encoding")
            }
            Self::ShortBuffer => write!(f, "data buffer shorter than height*width"),
            Self::DegenerateIntrinsics => write!(f, "camera_info has a zero focal length"),
        }
    }
}

/// One depth sample, as stored on the wire.
trait DepthPixel: Copy {
    const SIZE: usize;
    fn read(buf: &[u8], offset: usize) -> Self;
    /// `scale` applies to integer depth only; float depth is already metres.
    fn metres(self, scale: f32) -> f32;
}

impl DepthPixel for f32 {
    const SIZE: usize = 4;
    fn read(buf: &[u8], offset: usize) -> Self {
        f32::from_le_bytes([
            buf[offset],
            buf[offset + 1],
            buf[offset + 2],
            buf[offset + 3],
        ])
    }
    fn metres(self, _scale: f32) -> f32 {
        self
    }
}

impl DepthPixel for f64 {
    const SIZE: usize = 8;
    fn read(buf: &[u8], offset: usize) -> Self {
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&buf[offset..offset + 8]);
        f64::from_le_bytes(bytes)
    }
    fn metres(self, _scale: f32) -> f32 {
        self as f32
    }
}

impl DepthPixel for u16 {
    const SIZE: usize = 2;
    fn read(buf: &[u8], offset: usize) -> Self {
        u16::from_le_bytes([buf[offset], buf[offset + 1]])
    }
    fn metres(self, scale: f32) -> f32 {
        self as f32 * scale
    }
}

impl DepthPixel for i16 {
    const SIZE: usize = 2;
    fn read(buf: &[u8], offset: usize) -> Self {
        i16::from_le_bytes([buf[offset], buf[offset + 1]])
    }
    fn metres(self, scale: f32) -> f32 {
        self as f32 * scale
    }
}

struct Intrinsics {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
}

/// Registered depth is often published at a different resolution than the
/// CameraInfo was calibrated at; rescale rather than emit skewed geometry.
fn intrinsics_for(
    info: &CameraInfo,
    width: i32,
    height: i32,
) -> Result<Intrinsics, UnprojectError> {
    let (mut fx, mut fy) = (info.K[0] as f32, info.K[4] as f32);
    let (mut cx, mut cy) = (info.K[2] as f32, info.K[5] as f32);
    if info.width != 0 && info.height != 0 {
        let x_ratio = width as f32 / info.width as f32;
        let y_ratio = height as f32 / info.height as f32;
        fx *= x_ratio;
        cx *= x_ratio;
        fy *= y_ratio;
        cy *= y_ratio;
    }
    if fx == 0.0 || fy == 0.0 || !fx.is_finite() || !fy.is_finite() {
        return Err(UnprojectError::DegenerateIntrinsics);
    }
    Ok(Intrinsics { fx, fy, cx, cy })
}

/// Unproject `depth` through `info`, returning the packed point buffer and its
/// point count.
pub fn unproject(
    depth: &Image,
    info: &CameraInfo,
    params: &Params,
) -> Result<(Vec<u8>, i32), UnprojectError> {
    let intrinsics = intrinsics_for(info, depth.width, depth.height)?;
    match depth.encoding.as_str() {
        "32FC1" => run::<f32>(depth, &intrinsics, params),
        "64FC1" => run::<f64>(depth, &intrinsics, params),
        "16UC1" | "mono16" => run::<u16>(depth, &intrinsics, params),
        "16SC1" => run::<i16>(depth, &intrinsics, params),
        other => Err(UnprojectError::NotSingleChannelDepth(other.to_string())),
    }
}

fn run<P: DepthPixel>(
    depth: &Image,
    intrinsics: &Intrinsics,
    params: &Params,
) -> Result<(Vec<u8>, i32), UnprojectError> {
    let width = depth.width.max(0) as usize;
    let height = depth.height.max(0) as usize;
    let tight_row = width * P::SIZE;
    if depth.data.len() < height * tight_row {
        return Err(UnprojectError::ShortBuffer);
    }
    // Honour `step` only when the buffer is actually padded to it; dimos's own
    // Python encoder packs rows tightly and leaves `step` unset.
    let row_stride = match depth.step as usize {
        s if s >= tight_row && depth.data.len() >= height * s => s,
        _ => tight_row,
    };

    let step = params.decimation.max(1);
    let scale = params.depth_scale;
    let mut data: Vec<u8> =
        Vec::with_capacity(height.div_ceil(step) * width.div_ceil(step) * POINT_STEP);
    let mut count: i32 = 0;

    for row in (0..height).step_by(step) {
        let row_base = row * row_stride;
        let v = row as f32;
        for column in (0..width).step_by(step) {
            let z = P::read(&depth.data, row_base + column * P::SIZE).metres(scale);
            // NaN fails both comparisons, so it is excluded here.
            if !(z >= params.min_range_m && z <= params.max_range_m) {
                continue;
            }
            let x = (column as f32 - intrinsics.cx) * z / intrinsics.fx;
            let y = (v - intrinsics.cy) * z / intrinsics.fy;
            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());
            data.extend_from_slice(&0.0f32.to_le_bytes());
            count += 1;
        }
    }
    Ok((data, count))
}

#[cfg(test)]
mod tests {
    use super::*;

    const WIDTH: i32 = 8;
    const HEIGHT: i32 = 6;
    const FX: f64 = 100.0;
    const FY: f64 = 120.0;
    const CX: f64 = 3.5;
    const CY: f64 = 2.5;

    fn camera_info(width: i32, height: i32) -> CameraInfo {
        let mut info = CameraInfo {
            width,
            height,
            ..Default::default()
        };
        info.K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0];
        info
    }

    fn depth_image(metres: &[f32]) -> Image {
        let mut data = Vec::new();
        for m in metres {
            data.extend_from_slice(&m.to_le_bytes());
        }
        Image {
            width: WIDTH,
            height: HEIGHT,
            encoding: "32FC1".into(),
            data,
            ..Default::default()
        }
    }

    fn params() -> Params {
        Params {
            decimation: 1,
            min_range_m: 0.2,
            max_range_m: 6.0,
            depth_scale: 0.001,
        }
    }

    fn points(buffer: &[u8]) -> Vec<(f32, f32, f32)> {
        buffer
            .as_chunks::<POINT_STEP>()
            .0
            .iter()
            .map(|p| {
                (
                    f32::from_le_bytes(p[0..4].try_into().unwrap()),
                    f32::from_le_bytes(p[4..8].try_into().unwrap()),
                    f32::from_le_bytes(p[8..12].try_into().unwrap()),
                )
            })
            .collect()
    }

    #[test]
    fn unprojects_with_the_pinhole_model() {
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let (buffer, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        assert_eq!(count, WIDTH * HEIGHT);
        let got = points(&buffer);
        for row in 0..HEIGHT {
            for column in 0..WIDTH {
                let (x, y, z) = got[(row * WIDTH + column) as usize];
                assert!((x - (column as f32 - CX as f32) * 2.0 / FX as f32).abs() < 1e-6);
                assert!((y - (row as f32 - CY as f32) * 2.0 / FY as f32).abs() < 1e-6);
                assert!((z - 2.0).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn float_depth_is_already_metres() {
        // The R1 Pro head publishes 32FC1 metres, so depth_scale must not apply.
        let flat = vec![3.0f32; (WIDTH * HEIGHT) as usize];
        let (buffer, _) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        for (_, _, z) in points(&buffer) {
            assert!((z - 3.0).abs() < 1e-6);
        }
    }

    #[test]
    fn integer_depth_is_scaled() {
        let mut data = Vec::new();
        for _ in 0..(WIDTH * HEIGHT) {
            data.extend_from_slice(&3000u16.to_le_bytes());
        }
        let image = Image {
            width: WIDTH,
            height: HEIGHT,
            encoding: "16UC1".into(),
            data,
            ..Default::default()
        };

        let (buffer, _) = unproject(&image, &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        for (_, _, z) in points(&buffer) {
            assert!((z - 3.0).abs() < 1e-6);
        }
    }

    #[test]
    fn range_gate_drops_out_of_range_pixels() {
        let mut flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        flat[0] = 0.05;
        flat[1] = 50.0;
        flat[2] = 0.0;
        let (_, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        assert_eq!(count, WIDTH * HEIGHT - 3);
    }

    #[test]
    fn nan_depth_is_dropped() {
        let mut flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        flat[0] = f32::NAN;
        let (_, count) =
            unproject(&depth_image(&flat), &camera_info(WIDTH, HEIGHT), &params()).unwrap();

        assert_eq!(count, WIDTH * HEIGHT - 1);
    }

    #[test]
    fn decimation_keeps_full_resolution_pixel_coordinates() {
        // Decimated points must be a strict subset of the undecimated ones.
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let image = depth_image(&flat);
        let info = camera_info(WIDTH, HEIGHT);
        let (every_pixel, _) = unproject(&image, &info, &params()).unwrap();
        let (every_other, count) = unproject(
            &image,
            &info,
            &Params {
                decimation: 2,
                ..params()
            },
        )
        .unwrap();

        assert_eq!(count, (HEIGHT / 2) * (WIDTH / 2));
        let all = points(&every_pixel);
        for point in points(&every_other) {
            assert!(all
                .iter()
                .any(|p| (p.0 - point.0).abs() < 1e-6 && (p.1 - point.1).abs() < 1e-6));
        }
    }

    #[test]
    fn rescales_intrinsics_to_the_depth_resolution() {
        // Registered depth at half the calibrated resolution must not skew geometry.
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let (buffer, _) = unproject(
            &depth_image(&flat),
            &camera_info(WIDTH * 2, HEIGHT * 2),
            &params(),
        )
        .unwrap();

        let got = points(&buffer);
        for row in 0..HEIGHT {
            for column in 0..WIDTH {
                let (x, y, _) = got[(row * WIDTH + column) as usize];
                assert!(
                    (x - (column as f32 - CX as f32 / 2.0) * 2.0 / (FX as f32 / 2.0)).abs() < 1e-6
                );
                assert!(
                    (y - (row as f32 - CY as f32 / 2.0) * 2.0 / (FY as f32 / 2.0)).abs() < 1e-6
                );
            }
        }
    }

    #[test]
    fn colour_encodings_are_rejected() {
        let image = Image {
            width: WIDTH,
            height: HEIGHT,
            encoding: "rgb8".into(),
            data: vec![0; (WIDTH * HEIGHT * 3) as usize],
            ..Default::default()
        };

        assert_eq!(
            unproject(&image, &camera_info(WIDTH, HEIGHT), &params()),
            Err(UnprojectError::NotSingleChannelDepth("rgb8".into()))
        );
    }

    #[test]
    fn a_zero_focal_length_is_rejected() {
        // Otherwise every point divides out to infinity and the map is poisoned.
        let flat = vec![2.0f32; (WIDTH * HEIGHT) as usize];
        let info = CameraInfo::default();

        assert_eq!(
            unproject(&depth_image(&flat), &info, &params()),
            Err(UnprojectError::DegenerateIntrinsics)
        );
    }

    #[test]
    fn a_short_buffer_is_rejected() {
        let mut image = depth_image(&vec![2.0f32; (WIDTH * HEIGHT) as usize]);
        image.data.truncate(8);

        assert_eq!(
            unproject(&image, &camera_info(WIDTH, HEIGHT), &params()),
            Err(UnprojectError::ShortBuffer)
        );
    }

    #[test]
    fn padded_rows_are_honoured() {
        // A driver that pads rows sets `step`; reading tightly would shear the image.
        let width = 3usize;
        let height = 2usize;
        let stride = width * 4 + 8;
        let mut data = vec![0u8; height * stride];
        for row in 0..height {
            for column in 0..width {
                let z = 1.0f32 + row as f32;
                let at = row * stride + column * 4;
                data[at..at + 4].copy_from_slice(&z.to_le_bytes());
            }
        }
        let image = Image {
            width: width as i32,
            height: height as i32,
            encoding: "32FC1".into(),
            step: stride as i32,
            data,
            ..Default::default()
        };

        let (buffer, count) = unproject(&image, &camera_info(3, 2), &params()).unwrap();

        assert_eq!(count, 6);
        let got = points(&buffer);
        assert!(got[0..3].iter().all(|p| (p.2 - 1.0).abs() < 1e-6));
        assert!(got[3..6].iter().all(|p| (p.2 - 2.0).abs() < 1e-6));
    }
}
