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
//! A Yesense frame turned into a `sensor_msgs/Imu` in base_link's axes or the sensor's own.

use dimos_module::nalgebra::{Quaternion, UnitQuaternion, Vector3};
use lcm_msgs::geometry_msgs;
use lcm_msgs::sensor_msgs::Imu;
use lcm_msgs::std_msgs::{Header, Time};
use yesense_imu::Frame;

/// The sensor's rotation in base_link from roll, pitch, yaw about base_link's x, y, z.
pub fn mount_rotation(rpy_rad: [f64; 3]) -> UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(rpy_rad[0], rpy_rad[1], rpy_rad[2])
}

/// yesense_node's rt/IMU_DATA roll, pitch, yaw (deg): f32(i32) x f32(1e-6) per 0x40 count (0x16eec8), roll - 90 (0x15cf30).
pub fn vendor_rpy_deg(frame: &Frame) -> Option<[f32; 3]> {
    let e = frame.euler?;
    // yesense-imu's f64 radians back to the 0x40 counts (exact: its error is far below half a count)
    let count = |rad: f64| (rad.to_degrees() * 1e6).round() as i32;
    let deg = |rad: f64| count(rad) as f32 * 1e-6_f32;
    Some([deg(e.roll) - 90.0_f32, deg(e.pitch), deg(e.yaw)])
}

/// `frame` as an Imu stamped `ts`. With `body_axes`, vectors are rotated into base_link (`mount * v`) and the
/// orientation is base_link's (`q * mount⁻¹`), as the vendor driver publishes; otherwise both stay in sensor axes.
pub fn imu_message(
    frame: &Frame,
    mount: &UnitQuaternion<f64>,
    body_axes: bool,
    frame_id: &str,
    ts: f64,
) -> Imu {
    let rotate = |v: [f64; 3]| {
        let v = Vector3::from(v);
        let v = if body_axes { mount * v } else { v };
        geometry_msgs::Vector3 {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    };
    let mut orientation_covariance = [0.0; 9];
    // a zero or non-finite quaternion (no fusion yet) would normalize to NaN: treat it as missing
    let unit = frame.orientation().and_then(|[w, x, y, z]| {
        let q = Quaternion::new(w, x, y, z);
        let finite = [w, x, y, z].iter().all(|v| v.is_finite());
        finite
            .then(|| UnitQuaternion::try_new(q, f64::EPSILON))
            .flatten()
    });
    let orientation = match unit {
        Some(q) => {
            let q = if body_axes { q * mount.inverse() } else { q };
            geometry_msgs::Quaternion {
                x: q.i,
                y: q.j,
                z: q.k,
                w: q.w,
            }
        }
        None => {
            // no orientation in this frame: identity, flagged by -1 covariance
            orientation_covariance[0] = -1.0;
            geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }
        }
    };
    let mut angular_velocity_covariance = [0.0; 9];
    let angular_velocity = frame.gyro_rad_s.map(rotate).unwrap_or_else(|| {
        angular_velocity_covariance[0] = -1.0;
        geometry_msgs::Vector3::default()
    });
    let mut linear_acceleration_covariance = [0.0; 9];
    let linear_acceleration = frame.accel_mps2.map(rotate).unwrap_or_else(|| {
        linear_acceleration_covariance[0] = -1.0;
        geometry_msgs::Vector3::default()
    });
    Imu {
        header: Header {
            seq: 0,
            stamp: Time {
                sec: ts.trunc() as i32,
                nsec: (ts.fract() * 1e9) as i32,
            },
            frame_id: frame_id.into(),
        },
        orientation,
        orientation_covariance,
        angular_velocity,
        angular_velocity_covariance,
        linear_acceleration,
        linear_acceleration_covariance,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;
    use yesense_imu::Euler;

    fn frame(euler_deg: [f64; 3], accel: [f64; 3], gyro_dps: [f64; 3]) -> Frame {
        let [pitch, roll, yaw] = euler_deg.map(f64::to_radians);
        Frame {
            euler: Some(Euler { roll, pitch, yaw }),
            accel_mps2: Some(accel),
            gyro_rad_s: Some(gyro_dps.map(f64::to_radians)),
            ..Frame::default()
        }
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn vendor_angles_are_the_device_euler_block_in_f32_with_roll_less_90() {
        // a device euler block of (pitch, roll, yaw) counts of 1e-6 deg
        let counts = [1_234_567_i32, 91_234_567, -179_999_999];
        let [pitch, roll, yaw] = counts.map(|c| (c as f64 * 1e-6).to_radians());
        let f = Frame {
            euler: Some(Euler { roll, pitch, yaw }),
            ..Frame::default()
        };
        let want = [
            91_234_567_f32 * 1e-6_f32 - 90.0_f32,
            1_234_567_f32 * 1e-6_f32,
            -179_999_999_f32 * 1e-6_f32,
        ];
        let got = vendor_rpy_deg(&f).unwrap();
        assert_eq!(got.map(f32::to_bits), want.map(f32::to_bits));
        assert!(vendor_rpy_deg(&Frame::default()).is_none());
    }

    #[test]
    fn body_axes_match_the_vendor_driver() {
        // the vendor yesense_node, emulated: euler (pitch, roll, yaw) = (10, 20, 30) deg, accel (1, 2, 3), gyro (4, 5, 6) deg/s
        let mount = mount_rotation([FRAC_PI_2, 0.0, 0.0]);
        let msg = imu_message(
            &frame([10.0, 20.0, 30.0], [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
            &mount,
            true,
            "imu_link",
            0.0,
        );
        let q = &msg.orientation;
        assert!(
            close(q.x, -0.5704)
                && close(q.y, -0.07893)
                && close(q.z, 0.25949)
                && close(q.w, 0.77529),
            "{q:?}"
        );
        // vendor: (x, -z, y)
        let a = &msg.linear_acceleration;
        assert!(
            close(a.x, 1.0) && close(a.y, -3.0) && close(a.z, 2.0),
            "{a:?}"
        );
        let g = &msg.angular_velocity;
        assert!(
            close(g.x, 4f64.to_radians())
                && close(g.y, -6f64.to_radians())
                && close(g.z, 5f64.to_radians())
        );
    }

    #[test]
    fn resting_sensor_on_its_side_reads_level_in_body_axes() {
        // a sensor mounted at roll 90 deg (as on the M20) rests with gravity on its +y
        let mount = mount_rotation([FRAC_PI_2, 0.0, 0.0]);
        let msg = imu_message(
            &frame([0.0, 90.0, 0.0], [0.0, 9.8, 0.0], [0.0; 3]),
            &mount,
            true,
            "imu_link",
            0.0,
        );
        let q = &msg.orientation;
        assert!(close(q.w.abs(), 1.0), "{q:?}");
        assert!(close(msg.linear_acceleration.z, 9.8));
    }

    #[test]
    fn sensor_axes_pass_through() {
        let mount = mount_rotation([FRAC_PI_2, 0.0, 0.0]);
        let msg = imu_message(
            &frame([0.0, 0.0, 0.0], [1.0, 2.0, 3.0], [0.0; 3]),
            &mount,
            false,
            "imu_link",
            12.25,
        );
        assert!(close(msg.linear_acceleration.y, 2.0));
        assert!(close(msg.orientation.w, 1.0));
        assert_eq!(
            (msg.header.stamp.sec, msg.header.stamp.nsec),
            (12, 250_000_000)
        );
    }

    #[test]
    fn missing_blocks_are_flagged() {
        let msg = imu_message(
            &Frame::default(),
            &UnitQuaternion::identity(),
            true,
            "imu_link",
            0.0,
        );
        assert_eq!(msg.orientation_covariance[0], -1.0);
        assert_eq!(msg.angular_velocity_covariance[0], -1.0);
        assert_eq!(msg.linear_acceleration_covariance[0], -1.0);
    }

    #[test]
    fn a_zero_quaternion_is_flagged_missing_not_nan() {
        let msg = imu_message(
            &Frame {
                quaternion: Some([0.0; 4]),
                ..Frame::default()
            },
            &UnitQuaternion::identity(),
            true,
            "imu_link",
            0.0,
        );
        assert_eq!(msg.orientation_covariance[0], -1.0);
        assert_eq!(msg.orientation.w, 1.0);
    }
}
