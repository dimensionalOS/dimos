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
//! The cameras' static transforms and intrinsics (raw, and deskewed).

use dimos_module::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use dimos_module::Transform;
use lcm_msgs::sensor_msgs::{CameraInfo, RegionOfInterest};
use lcm_msgs::std_msgs::{Header, Time};
use rockchip_h264::deskew::{Intrinsics, Model, Output, Projection};

/// Body (REP-103) from optical axes, x y z w.
const OPTICAL_ROTATION: [f64; 4] = [-0.5, 0.5, -0.5, 0.5];

/// `base_link -> <name>_link -> <name>_optical` for a camera at `xyz` turned `yaw_rad` about z.
pub fn camera_transforms(name: &str, xyz: [f64; 3], yaw_rad: f64, ts: f64) -> [Transform; 2] {
    let [x, y, z] = xyz;
    let mount = Isometry3::from_parts(
        Translation3::new(x, y, z),
        UnitQuaternion::from_euler_angles(0.0, 0.0, yaw_rad),
    );
    let [qx, qy, qz, qw] = OPTICAL_ROTATION;
    let optical = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz)),
    );
    [
        Transform::new("base_link", format!("{name}_link"), ts, mount),
        Transform::new(
            format!("{name}_link"),
            format!("{name}_optical"),
            ts,
            optical,
        ),
    ]
}

/// The capture's camera: centered, its focal length scaled from one calibrated at `focal_length_width_px` wide, and
/// the lens distortion as configured.
pub fn intrinsics(
    width: u32,
    height: u32,
    focal_length_px: f64,
    focal_length_width_px: f64,
    model: Model,
    coefficients: &[f64],
) -> Intrinsics {
    let focal = focal_length_px * width as f64 / focal_length_width_px;
    Intrinsics {
        width,
        height,
        fx: focal,
        fy: focal,
        cx: width as f64 * 0.5,
        cy: height as f64 * 0.5,
        model,
        coefficients: coefficients.to_vec(),
    }
}

/// The CameraInfo of raw frames from `camera`: its lens model and coefficients (padded to the model's length).
pub fn camera_info(camera: &Intrinsics, frame_id: &str, ts: f64) -> CameraInfo {
    let mut d = camera.coefficients.clone();
    d.resize(camera.model.coefficient_count().max(d.len()), 0.0);
    info(
        camera.width,
        camera.height,
        [camera.fx, camera.fy, camera.cx, camera.cy],
        camera.model.name(),
        d,
        frame_id,
        ts,
    )
}

/// The CameraInfo of deskewed frames: no lens distortion. A pinhole output is a plain `plumb_bob` with zero
/// coefficients; a cylindrical or equirectangular one names its projection in `distortion_model` (no standard
/// CameraInfo model covers them), with K holding its angular scale and center.
pub fn rectified_camera_info(camera: &Output, frame_id: &str, ts: f64) -> CameraInfo {
    let (model, d) = match camera.projection {
        Projection::Pinhole => (Model::PlumbBob.name(), vec![0.0; 5]),
        other => (other.name(), vec![]),
    };
    info(
        camera.width,
        camera.height,
        [camera.fx, camera.fy, camera.cx, camera.cy],
        model,
        d,
        frame_id,
        ts,
    )
}

#[allow(clippy::too_many_arguments)]
fn info(
    width: u32,
    height: u32,
    [fx, fy, cx, cy]: [f64; 4],
    model: &str,
    d: Vec<f64>,
    frame_id: &str,
    ts: f64,
) -> CameraInfo {
    CameraInfo {
        header: Header {
            seq: 0,
            stamp: Time {
                sec: ts.trunc() as i32,
                nsec: (ts.fract() * 1e9) as i32,
            },
            frame_id: frame_id.into(),
        },
        height: height as i32,
        width: width as i32,
        distortion_model: model.into(),
        D: d,
        K: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P: [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x: 0,
        binning_y: 0,
        roi: RegionOfInterest::default(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_module::nalgebra::Point3;

    #[test]
    fn focal_length_scales_with_the_stream_width() {
        let camera = intrinsics(800, 600, 607.0, 1280.0, Model::PlumbBob, &[0.0; 5]);
        let info = camera_info(&camera, "front_camera_optical", 12.5);
        assert!((info.K[0] - 379.375).abs() < 1e-9);
        assert_eq!(info.K[0], info.K[4]);
        assert_eq!((info.K[2], info.K[5]), (400.0, 300.0));
        assert_eq!((info.width, info.height), (800, 600));
        assert_eq!(
            (info.header.stamp.sec, info.header.stamp.nsec),
            (12, 500_000_000)
        );
    }

    #[test]
    fn the_calibrated_width_keeps_its_focal_length() {
        let camera = intrinsics(1280, 720, 607.0, 1280.0, Model::PlumbBob, &[]);
        let info = camera_info(&camera, "rear_camera_optical", 0.0);
        assert_eq!(info.K[0], 607.0);
        assert_eq!(info.D, vec![0.0; 5]);
        assert_eq!(info.P[0], 607.0);
        assert_eq!(info.header.frame_id, "rear_camera_optical");
    }

    #[test]
    fn a_fisheye_reports_its_model_and_coefficients() {
        let camera = intrinsics(1280, 720, 486.0, 1280.0, Model::Equidistant, &[0.01]);
        let info = camera_info(&camera, "rear_camera_optical", 0.0);
        assert_eq!(info.distortion_model, "equidistant");
        assert_eq!(info.D, vec![0.01, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn the_deskewed_camera_has_no_distortion_and_its_own_matrix() {
        let rectified = Output {
            projection: Projection::Pinhole,
            width: 1280,
            height: 720,
            fx: 393.68,
            fy: 393.68,
            cx: 640.0,
            cy: 360.0,
        };
        let info = rectified_camera_info(&rectified, "rear_camera_optical", 0.0);
        assert_eq!(info.distortion_model, "plumb_bob");
        assert_eq!(info.D, vec![0.0; 5]);
        assert_eq!((info.K[0], info.P[0], info.K[2]), (393.68, 393.68, 640.0));
        assert_eq!(info.header.frame_id, "rear_camera_optical");
    }

    #[test]
    fn a_cylindrical_output_names_its_projection() {
        let wide = Output {
            projection: Projection::Cylindrical,
            width: 1280,
            height: 720,
            fx: 486.0,
            fy: 486.0,
            cx: 640.0,
            cy: 360.0,
        };
        let info = rectified_camera_info(&wide, "rear_camera_optical", 0.0);
        assert_eq!(info.distortion_model, "cylindrical");
        assert!(info.D.is_empty());
    }

    #[test]
    fn a_rational_polynomial_camera_reports_eight_coefficients() {
        let camera = intrinsics(
            1280,
            720,
            600.0,
            1280.0,
            Model::RationalPolynomial,
            &[0.1, 0.2],
        );
        let info = camera_info(&camera, "front_camera_optical", 0.0);
        assert_eq!(info.distortion_model, "rational_polynomial");
        assert_eq!(info.D.len(), 8);
    }

    #[test]
    fn a_rear_camera_looks_backwards() {
        let [mount, optical] = camera_transforms(
            "rear_camera",
            [-0.37646, 0.0, 0.03738],
            std::f64::consts::PI,
            0.0,
        );
        assert_eq!(
            (mount.parent.as_str(), mount.child.as_str()),
            ("base_link", "rear_camera_link")
        );
        assert_eq!(
            (optical.parent.as_str(), optical.child.as_str()),
            ("rear_camera_link", "rear_camera_optical")
        );
        // The optical +z axis is the camera's view direction: -x of the body for the rear camera.
        let view = mount.rotation() * optical.rotation() * Point3::new(0.0, 0.0, 1.0);
        assert!((view - Point3::new(-1.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((mount.translation().x + 0.37646).abs() < 1e-12);
    }
}
