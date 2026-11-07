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
//! The cameras' static transforms and pinhole intrinsics.

use dimos_module::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use dimos_module::Transform;
use lcm_msgs::sensor_msgs::{CameraInfo, RegionOfInterest};
use lcm_msgs::std_msgs::{Header, Time};

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

/// A distortion-free pinhole for a `width` x `height` stream, centered, its focal length scaled
/// from one calibrated at `focal_length_width_px` wide.
pub fn camera_info(
    width: u32,
    height: u32,
    focal_length_px: f64,
    focal_length_width_px: f64,
    frame_id: &str,
    ts: f64,
) -> CameraInfo {
    let focal = focal_length_px * width as f64 / focal_length_width_px;
    let (cx, cy) = (width as f64 * 0.5, height as f64 * 0.5);
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
        distortion_model: "plumb_bob".into(),
        D: vec![0.0; 5],
        K: [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0],
        R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P: [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
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
        let info = camera_info(800, 600, 607.0, 1280.0, "front_camera_optical", 12.5);
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
        let info = camera_info(1280, 720, 607.0, 1280.0, "rear_camera_optical", 0.0);
        assert_eq!(info.K[0], 607.0);
        assert_eq!(info.P[0], 607.0);
        assert_eq!(info.header.frame_id, "rear_camera_optical");
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
