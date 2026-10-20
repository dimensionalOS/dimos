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

//! A wide-angle frame resampled to a pinhole one: Depth Anything and the lidar
//! projection both assume straight lines stay straight.

use depth2depth::Pinhole;
use lcm_msgs::sensor_msgs::CameraInfo;
use rayon::prelude::*;

/// Pinhole + distortion, as it arrives in a `CameraInfo`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lens {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub model: Model,
    /// Brown-Conrady: `[k1, k2, p1, p2, k3, k4, k5, k6]`, zero-extended so a 5-coefficient plumb_bob works unchanged.
    /// Equidistant: `[k1, k2, k3, k4, 0, 0, 0, 0]`.
    pub distortion: [f64; 8],
}

/// The distortion models a `CameraInfo` can name that this resamples.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Model {
    /// OpenCV's standard model: plumb_bob, or rational_polynomial with eight coefficients.
    BrownConrady,
    /// OpenCV's fisheye (Kannala-Brandt) model, which ROS names equidistant.
    Equidistant,
}

impl Model {
    /// None for a model this cannot undistort.
    pub fn from_name(name: &str) -> Option<Self> {
        match name.trim().to_ascii_lowercase().as_str() {
            "" | "plumb_bob" | "rational_polynomial" => Some(Self::BrownConrady),
            "equidistant" | "fisheye" | "kannala_brandt" => Some(Self::Equidistant),
            _ => None,
        }
    }
}

impl Lens {
    /// The lens as seen in an image `scale` times the calibration's resolution; None without usable intrinsics
    /// or for a distortion model this cannot undistort.
    pub fn from_info(info: &CameraInfo, scale: f64) -> Option<Self> {
        let (fx, fy) = (info.K[0], info.K[4]);
        if !(fx > 0.0 && fy > 0.0 && fx.is_finite() && fy.is_finite()) {
            return None;
        }
        let model = Model::from_name(&info.distortion_model)?;
        // Eight coefficients mean the rational model whatever `distortion_model` claims; some drivers mislabel it plumb_bob.
        let mut distortion = [0.0; 8];
        for (slot, value) in distortion.iter_mut().zip(&info.D) {
            *slot = *value;
        }
        Some(Self {
            fx: fx * scale,
            fy: fy * scale,
            cx: (info.K[2] + 0.5) * scale - 0.5,
            cy: (info.K[5] + 0.5) * scale - 0.5,
            model,
            distortion,
        })
    }

    /// Where a normalised ray lands in the distorted image (NaN past the model's valid range).
    pub fn distort(&self, x: f64, y: f64) -> (f64, f64) {
        let (xd, yd) = match self.model {
            Model::BrownConrady => self.brown_conrady(x, y),
            Model::Equidistant => self.equidistant(x, y),
        };
        (self.fx * xd + self.cx, self.fy * yd + self.cy)
    }

    /// As OpenCV's `fisheye::distortPoints`: the angle off the axis, not the radius, carries the polynomial.
    fn equidistant(&self, x: f64, y: f64) -> (f64, f64) {
        let [k1, k2, k3, k4, ..] = self.distortion;
        let r = (x * x + y * y).sqrt();
        if r < 1e-12 {
            return (x, y);
        }
        let theta = r.atan();
        let t2 = theta * theta;
        let theta_d = theta * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))));
        (x * theta_d / r, y * theta_d / r)
    }

    fn brown_conrady(&self, x: f64, y: f64) -> (f64, f64) {
        let [k1, k2, p1, p2, k3, k4, k5, k6] = self.distortion;
        let r2 = x * x + y * y;
        let (r4, r6) = (r2 * r2, r2 * r2 * r2);
        let denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
        if denominator.abs() < 1e-12 {
            return (f64::NAN, f64::NAN);
        }
        // Numerator and denominator can both turn negative towards a wide lens's corners and the ray is
        // still fine; only a negative ratio mirrors it through the principal point.
        let radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) / denominator;
        if radial <= 0.0 {
            return (f64::NAN, f64::NAN);
        }
        let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        (xd, yd)
    }
}

/// Per output pixel, where to sample the distorted source.
pub struct UndistortMap {
    pub camera: Pinhole,
    pub width: usize,
    pub height: usize,
    source_xy: Vec<[f32; 2]>,
}

impl UndistortMap {
    /// A `width` x `height` pinhole view through `lens`, focal length `focal_px`, centred.
    pub fn new(lens: &Lens, width: usize, height: usize, focal_px: f64) -> Self {
        let camera = Pinhole {
            fx: focal_px as f32,
            fy: focal_px as f32,
            cx: (width as f32 - 1.0) / 2.0,
            cy: (height as f32 - 1.0) / 2.0,
        };
        let source_xy = (0..width * height)
            .map(|i| {
                let x = ((i % width) as f64 - camera.cx as f64) / focal_px;
                let y = ((i / width) as f64 - camera.cy as f64) / focal_px;
                let (sx, sy) = lens.distort(x, y);
                [sx as f32, sy as f32]
            })
            .collect();
        Self {
            camera,
            width,
            height,
            source_xy,
        }
    }

    /// Bilinearly resample an interleaved RGB `source`; pixels mapping outside it stay black.
    pub fn apply(&self, source: &[u8], source_width: usize, source_height: usize) -> Vec<u8> {
        let mut out = vec![0u8; self.width * self.height * 3];
        out.par_chunks_mut(3)
            .zip(&self.source_xy)
            .for_each(|(pixel, &[sx, sy])| {
                if !(sx >= 0.0 && sy >= 0.0) {
                    return;
                }
                let (x0, y0) = (sx as usize, sy as usize);
                if x0 + 1 >= source_width || y0 + 1 >= source_height {
                    return;
                }
                let (tx, ty) = (sx - x0 as f32, sy - y0 as f32);
                let at =
                    |x: usize, y: usize, c: usize| source[(y * source_width + x) * 3 + c] as f32;
                for (c, value) in pixel.iter_mut().enumerate() {
                    let top = at(x0, y0, c) * (1.0 - tx) + at(x0 + 1, y0, c) * tx;
                    let bottom = at(x0, y0 + 1, c) * (1.0 - tx) + at(x0 + 1, y0 + 1, c) * tx;
                    *value = (top * (1.0 - ty) + bottom * ty).round() as u8;
                }
            });
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(coefficients: &[f64]) -> CameraInfo {
        CameraInfo {
            width: 1920,
            height: 1536,
            K: [1000.0, 0.0, 959.5, 0.0, 1000.0, 767.5, 0.0, 0.0, 1.0],
            D: coefficients.to_vec(),
            ..Default::default()
        }
    }

    #[test]
    fn without_distortion_the_map_is_a_crop_and_scale() {
        let lens = Lens::from_info(&info(&[0.0; 5]), 0.5).unwrap();
        assert_eq!((lens.fx, lens.cx, lens.cy), (500.0, 479.5, 383.5));
        let map = UndistortMap::new(&lens, 100, 80, 500.0);
        // The output centre samples the source centre; one pixel right samples one pixel right.
        let centre = map.source_xy[40 * 100 + 50];
        assert!(
            (centre[0] - 480.0).abs() < 0.51 && (centre[1] - 384.0).abs() < 0.51,
            "{centre:?}"
        );
        let next = map.source_xy[40 * 100 + 51];
        assert!((next[0] - centre[0] - 1.0).abs() < 1e-3);
    }

    #[test]
    fn eight_coefficients_are_the_rational_model() {
        let lens = Lens::from_info(
            &info(&[-0.68, -0.64, 0.0, 0.0, -0.03, -0.29, -1.0, -0.18]),
            1.0,
        )
        .unwrap();
        // Barrel-ish: a ray off-axis lands closer to the centre than a pinhole would put it.
        let (x, _) = lens.distort(0.5, 0.0);
        assert!(x < 959.5 + 500.0 && x > 959.5, "{x}");
        // And a 5-coefficient model is the same model with k4..k6 zero.
        let five = Lens::from_info(&info(&[-0.1, 0.01, 0.0, 0.0, 0.0]), 1.0).unwrap();
        assert_eq!(five.distortion[5..], [0.0; 3]);
    }

    #[test]
    fn the_corners_of_a_wide_lens_match_opencv() {
        // The R1 Pro head camera; OpenCV's projectPoints puts these rays at these source pixels.
        let mut head = info(&[
            -0.6792, -0.638, 0.0002, -0.0002, -0.0302, -0.2868, -0.9987, -0.1752,
        ]);
        head.K = [1012.59, 0.0, 962.21, 0.0, 1012.13, 765.67, 0.0, 0.0, 1.0];
        let lens = Lens::from_info(&head, 1.0).unwrap();
        for ((x, y), (u, v)) in [
            ((-0.9467, -0.7570), (300.77, 237.5)),
            ((0.9467, 0.7570), (1622.91, 1294.32)),
        ] {
            let (su, sv) = lens.distort(x, y);
            assert!(
                (su - u).abs() < 0.05 && (sv - v).abs() < 0.05,
                "({x}, {y}) -> ({su}, {sv}), OpenCV ({u}, {v})"
            );
        }
    }

    #[test]
    fn an_equidistant_lens_matches_opencv_fisheye() {
        // The Go2 front camera (front_camera_720.yaml); OpenCV's fisheye::distortPoints puts these rays here.
        let mut go2 = info(&[-0.0730943, -0.0234114, -0.0069306, 0.0092387]);
        go2.distortion_model = "equidistant".into();
        go2.K = [
            797.4756, 0.0, 643.5352, 0.0, 796.4872, 349.2784, 0.0, 0.0, 1.0,
        ];
        let lens = Lens::from_info(&go2, 1.0).unwrap();
        assert_eq!(lens.model, Model::Equidistant);
        for ((x, y), (u, v)) in [
            ((-0.9, -0.5), (117.447, 57.369)),
            ((0.9, 0.5), (1169.624, 641.188)),
            ((0.3, -0.1), (873.612, 272.681)),
        ] {
            let (su, sv) = lens.distort(x, y);
            assert!(
                (su - u).abs() < 0.05 && (sv - v).abs() < 0.05,
                "({x}, {y}) -> ({su}, {sv}), OpenCV ({u}, {v})"
            );
        }
    }

    #[test]
    fn an_unknown_distortion_model_is_refused() {
        let mut odd = info(&[0.0; 5]);
        odd.distortion_model = "double_sphere".into();
        assert!(Lens::from_info(&odd, 1.0).is_none());
    }

    #[test]
    fn zero_focal_length_is_refused() {
        let mut bad = info(&[0.0; 5]);
        bad.K[0] = 0.0;
        assert!(Lens::from_info(&bad, 1.0).is_none());
    }

    #[test]
    fn resampling_keeps_colour_and_blanks_what_falls_outside() {
        let lens = Lens::from_info(&info(&[0.0; 5]), 1.0 / 64.0).unwrap(); // a 30x24 source
        let source: Vec<u8> = (0..30 * 24).flat_map(|_| [10u8, 20, 30]).collect();
        let map = UndistortMap::new(&lens, 60, 48, lens.fx);
        let out = map.apply(&source, 30, 24);
        assert_eq!(&out[(24 * 60 + 30) * 3..][..3], &[10, 20, 30]);
        assert_eq!(&out[..3], &[0, 0, 0]);
    }
}
