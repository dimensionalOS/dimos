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

//! A stereo pair's intrinsics, read from a file, for the offline tools.
//!
//! The live module gets its intrinsics from two `CameraInfo` messages. The
//! offline tools -- `stereo_offline`, `stereo_bench` -- get them from disk, in
//! one of two shapes, and both tools must read them the same way: an earlier
//! version of `stereo_offline` took one focal length for both eyes and no
//! distortion at all, and so measured a pipeline the robot does not run. On
//! the R1's head that alone read the floor a third too far away.
//!
//! Shapes:
//!
//! - `calib.txt`: one line per eye, left then right, each
//!   `width height fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6` at the resolution the
//!   calibration was taken at. Blank lines and `#` comments are ignored.
//! - `manifest.json` as `depth_eval` writes it: a `calibration` object with
//!   `left` and `right`, each `{width, height, fx, fy, cx, cy, distortion}`.

use std::fs;
use std::path::Path;

use lcm_msgs::sensor_msgs::CameraInfo;
use serde::Deserialize;

use crate::stereo::Camera;

/// One eye's calibration, at the resolution it was taken at.
#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    pub width: usize,
    pub height: usize,
    pub camera: Camera,
}

impl Calibration {
    /// The same lens described at `width x height` after a `factor` downscale.
    pub fn scaled(&self, width: usize, height: usize, factor: usize) -> Camera {
        let sx = (width as f64 / self.width as f64) / factor as f64;
        let sy = (height as f64 / self.height as f64) / factor as f64;
        Camera {
            fx: self.camera.fx * sx,
            fy: self.camera.fy * sy,
            cx: self.camera.cx * sx,
            cy: self.camera.cy * sy,
            distortion: self.camera.distortion,
        }
    }

    /// The same calibration as the message the robot would publish, so an
    /// offline tool can feed the module's own code path rather than a copy.
    pub fn camera_info(&self) -> CameraInfo {
        CameraInfo {
            width: self.width as i32,
            height: self.height as i32,
            distortion_model: "rational_polynomial".into(),
            D: self.camera.distortion.to_vec(),
            K: [
                self.camera.fx,
                0.0,
                self.camera.cx,
                0.0,
                self.camera.fy,
                self.camera.cy,
                0.0,
                0.0,
                1.0,
            ],
            ..Default::default()
        }
    }
}

/// Read a pair of calibrations, choosing the shape by the file's extension.
pub fn read_pair(path: &Path) -> Result<(Calibration, Calibration), String> {
    let text =
        fs::read_to_string(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let label = path.display().to_string();
    if path.extension().is_some_and(|ext| ext == "json") {
        parse_manifest(&text, &label)
    } else {
        parse_text(&text, &label)
    }
}

/// The `calib.txt` shape: 14 numbers per eye, left then right.
pub fn parse_text(text: &str, label: &str) -> Result<(Calibration, Calibration), String> {
    let mut eyes = Vec::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let values = line
            .split_whitespace()
            .map(|token| {
                token
                    .parse::<f64>()
                    .map_err(|e| format!("{label}: {token:?} is not a number: {e}"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        if values.len() != 14 {
            return Err(format!(
                "{label}: each eye needs 14 numbers \
                 (width height fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6), got {}",
                values.len()
            ));
        }
        let mut distortion = [0.0f64; 8];
        distortion.copy_from_slice(&values[6..14]);
        eyes.push(Calibration {
            width: values[0] as usize,
            height: values[1] as usize,
            camera: Camera {
                fx: values[2],
                fy: values[3],
                cx: values[4],
                cy: values[5],
                distortion,
            },
        });
    }
    if eyes.len() != 2 {
        return Err(format!(
            "{label}: needs exactly two eyes, left then right, got {}",
            eyes.len()
        ));
    }
    let right = eyes.pop().expect("two eyes");
    let left = eyes.pop().expect("two eyes");
    Ok((left, right))
}

#[derive(Deserialize)]
struct ManifestEye {
    width: usize,
    height: usize,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    distortion: Vec<f64>,
}

#[derive(Deserialize)]
struct ManifestCalibration {
    left: ManifestEye,
    right: ManifestEye,
}

#[derive(Deserialize)]
struct Manifest {
    calibration: ManifestCalibration,
}

/// The `manifest.json` shape, as `depth_eval`'s frame store writes it.
pub fn parse_manifest(text: &str, label: &str) -> Result<(Calibration, Calibration), String> {
    let manifest: Manifest =
        serde_json::from_str(text).map_err(|e| format!("{label}: not a frame manifest: {e}"))?;
    let eye = |eye: ManifestEye, which: &str| -> Result<Calibration, String> {
        // Fewer than eight coefficients is plumb_bob; the missing rational
        // terms are zero, which collapses the model back to plumb_bob exactly.
        if eye.distortion.len() > 8 {
            return Err(format!(
                "{label}: {which} eye has {} distortion coefficients, at most 8 are understood",
                eye.distortion.len()
            ));
        }
        let mut distortion = [0.0f64; 8];
        distortion[..eye.distortion.len()].copy_from_slice(&eye.distortion);
        Ok(Calibration {
            width: eye.width,
            height: eye.height,
            camera: Camera {
                fx: eye.fx,
                fy: eye.fy,
                cx: eye.cx,
                cy: eye.cy,
                distortion,
            },
        })
    };
    Ok((
        eye(manifest.calibration.left, "left")?,
        eye(manifest.calibration.right, "right")?,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEXT: &str = "# width height fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6\n\
        1920 1536 1012.6 1012.1 962.2 765.7 -0.68 -0.64 0.0002 -0.0002 -0.03 -0.29 -1.0 -0.18\n\
        \n\
        1920 1536 1013.8 1013.4 958.7 768.2 -0.25 -0.43 0.0001 0.0 -0.02 0.14 -0.62 -0.12\n";

    const MANIFEST: &str = r#"{
        "recording": "x", "frames": [],
        "calibration": {
            "left": {"width": 1920, "height": 1536, "fx": 1012.6, "fy": 1012.1,
                     "cx": 962.2, "cy": 765.7,
                     "distortion": [-0.68, -0.64, 0.0002, -0.0002, -0.03, -0.29, -1.0, -0.18]},
            "right": {"width": 1920, "height": 1536, "fx": 1013.8, "fy": 1013.4,
                      "cx": 958.7, "cy": 768.2,
                      "distortion": [-0.25, -0.43, 0.0001, 0.0, -0.02]}
        }
    }"#;

    #[test]
    fn the_text_shape_reads_both_eyes_in_order() {
        let (left, right) = parse_text(TEXT, "calib.txt").expect("parses");
        assert_eq!(left.width, 1920);
        assert!((left.camera.fx - 1012.6).abs() < 1e-9);
        assert!((right.camera.fx - 1013.8).abs() < 1e-9);
        assert!((left.camera.distortion[7] - -0.18).abs() < 1e-9);
    }

    #[test]
    fn the_manifest_shape_reads_the_same_numbers() {
        let (left, right) = parse_manifest(MANIFEST, "manifest.json").expect("parses");
        assert_eq!(left.height, 1536);
        assert!((right.camera.cy - 768.2).abs() < 1e-9);
        assert!((left.camera.distortion[6] - -1.0).abs() < 1e-9);
        // Five coefficients pad out to eight zeros, like camera_from does.
        assert_eq!(right.camera.distortion[5], 0.0);
        assert_eq!(right.camera.distortion[7], 0.0);
    }

    #[test]
    fn one_eye_is_refused() {
        let one = TEXT.lines().take(2).collect::<Vec<_>>().join("\n");
        let err = parse_text(&one, "calib.txt").expect_err("one eye is not a pair");
        assert!(err.contains("two eyes"), "{err}");
    }

    #[test]
    fn the_wrong_number_count_names_the_shape() {
        let err = parse_text("1 2 3\n4 5 6\n", "calib.txt").expect_err("three numbers");
        assert!(err.contains("14 numbers"), "{err}");
    }

    #[test]
    fn a_camera_info_carries_the_same_lens() {
        let (left, _) = parse_text(TEXT, "calib.txt").expect("parses");
        let info = left.camera_info();
        assert_eq!(info.width, 1920);
        assert!((info.K[0] - 1012.6).abs() < 1e-9);
        assert!((info.K[2] - 962.2).abs() < 1e-9);
        assert_eq!(info.D.len(), 8);
    }

    #[test]
    fn scaling_folds_the_calibration_resolution_and_the_downscale_together() {
        let (left, _) = parse_text(TEXT, "calib.txt").expect("parses");
        // Frames at half the calibrated size, then downscaled by 4: 1/8 overall.
        let camera = left.scaled(960, 768, 4);
        assert!((camera.fx - 1012.6 / 8.0).abs() < 1e-9);
        assert!((camera.cy - 765.7 / 8.0).abs() < 1e-9);
    }
}
