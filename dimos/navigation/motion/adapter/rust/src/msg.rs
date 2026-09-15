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

//! Marshalling between dimos messages and the plain arrays the pure crates
//! take. Both modules share it so a plan cannot be written by one convention
//! and read back by another.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dimos_module::{error_throttled, Output};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion, Vector3};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};

/// A plan waypoint as the laws and the planner see it: `(x, y, yaw)`.
pub type State = [f64; 3];

/// Yaw of a quaternion, matching `Quaternion.euler[2]`.
///
/// The python goes through `scipy.Rotation.as_euler("xyz")`, i.e. extrinsic
/// x-then-y-then-z, whose composite is `Rz(yaw) Ry(pitch) Rx(roll)` -- the
/// standard ROS convention, and this is its standard closed form. Only the
/// yaw is ever taken: both modules work in SE(2).
pub fn yaw_of(q: &Quaternion) -> f64 {
    let siny = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    siny.atan2(cosy)
}

/// The quaternion for a planar heading, the inverse of [`yaw_of`].
pub fn quat_of_yaw(yaw: f64) -> Quaternion {
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: (yaw / 2.0).sin(),
        w: (yaw / 2.0).cos(),
    }
}

/// Seconds since the unix epoch as one float, the python `PoseStamped.ts`.
pub fn secs_of(t: &Time) -> f64 {
    t.sec as f64 + t.nsec as f64 * 1e-9
}

/// A float second count as a `Time`, matching `Header.__init__`'s truncation
/// (`sec = int(ts)`, `nsec = int((ts - sec) * 1e9)`).
pub fn time_of_secs(ts: f64) -> Time {
    if !ts.is_finite() {
        return Time { sec: 0, nsec: 0 };
    }
    let sec = ts.trunc();
    Time {
        sec: sec.clamp(i32::MIN as f64, i32::MAX as f64) as i32,
        nsec: ((ts - sec) * 1e9) as i32,
    }
}

/// Wall clock now, as the stamp a published message carries.
pub fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

pub fn header(frame_id: &str, ts: f64) -> Header {
    Header {
        seq: 0,
        stamp: time_of_secs(ts),
        frame_id: frame_id.into(),
    }
}

/// One plan waypoint as a stamped pose in `frame_id`, on the plane at
/// `ground_z`.
///
/// The search is planar and knows no z, but `odom` z = 0 is wherever the LIO
/// frame started -- on a lidar-odometry stack, a sensor's height above the
/// floor. Stamping the plan with the surface the feet stand on is what puts the
/// route on the ground instead of floating it over the robot. Every consumer of
/// the path is planar; only a viewer reads the z.
pub fn pose_stamped(state: &State, ts: f64, frame_id: &str, ground_z: f64) -> PoseStamped {
    PoseStamped {
        header: header(frame_id, ts),
        pose: Pose {
            position: Point {
                x: state[0],
                y: state[1],
                z: ground_z,
            },
            orientation: quat_of_yaw(state[2]),
        },
    }
}

/// A plan as a nav Path, per-waypoint stamps carrying the precision profile.
///
/// `stamps` of the wrong length leaves every pose at `t0`, which is what an
/// unstamped path looks like to `decode_ceilings` -- the honest shape for a
/// plan whose profile could not be computed.
pub fn build_path(
    states: &[State],
    stamps: &[f64],
    t0: f64,
    frame_id: &str,
    ground_z: f64,
) -> Path {
    let poses = states
        .iter()
        .enumerate()
        .map(|(k, s)| pose_stamped(s, stamps.get(k).copied().unwrap_or(t0), frame_id, ground_z))
        .collect();
    Path {
        header: header(frame_id, t0),
        poses,
    }
}

/// The plan as `(x, y, yaw)` rows -- the python `controller.path_xy_yaw`.
pub fn path_states(path: &Path) -> Vec<State> {
    path.poses
        .iter()
        .map(|p| {
            [
                p.pose.position.x,
                p.pose.position.y,
                yaw_of(&p.pose.orientation),
            ]
        })
        .collect()
}

/// The plan's own per-waypoint stamps, in seconds.
pub fn path_stamps(path: &Path) -> Vec<f64> {
    path.poses
        .iter()
        .map(|p| secs_of(&p.header.stamp))
        .collect()
}

/// A body-frame twist, the only shape either module publishes.
pub fn twist(vx: f64, vy: f64, wz: f64) -> lcm_msgs::geometry_msgs::Twist {
    lcm_msgs::geometry_msgs::Twist {
        linear: Vector3 {
            x: vx,
            y: vy,
            z: 0.0,
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: wz,
        },
    }
}

/// Publish, logging a throttled error instead of unwinding the loop.
pub async fn publish<T>(out: &Output<T>, msg: &T) {
    if let Err(e) = out.publish(msg).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "failed to publish",
        );
    }
}

#[derive(Debug)]
pub struct ExtractError(&'static str);

impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

/// The cloud as f32 xyz triples, non-finite points dropped.
///
/// f32 and not f64 on purpose: the python reads `cloud.points_f32()`,
/// references it to the body in f32, and only then widens for `plan()`. Both
/// the planner's SDF and the room hint therefore see the same rounded numbers,
/// and a port that widened first would disagree in the last bits.
pub fn extract_xyz(msg: &PointCloud2) -> Result<Vec<[f32; 3]>, ExtractError> {
    let mut offsets: [Option<usize>; 3] = [None; 3];
    for f in &msg.fields {
        if f.datatype != PointField::FLOAT32 as u8 {
            continue;
        }
        match f.name.as_str() {
            "x" => offsets[0] = Some(f.offset as usize),
            "y" => offsets[1] = Some(f.offset as usize),
            "z" => offsets[2] = Some(f.offset as usize),
            _ => {}
        }
    }
    let [Some(xo), Some(yo), Some(zo)] = offsets else {
        return Err(ExtractError("missing a float32 x/y/z field"));
    };

    let n = (msg.width as usize) * (msg.height as usize);
    let step = msg.point_step as usize;
    if step == 0 {
        return Err(ExtractError("point_step is 0"));
    }
    if msg.data.len() < n * step {
        return Err(ExtractError(
            "data buffer shorter than width*height*point_step",
        ));
    }
    if xo + 4 > step || yo + 4 > step || zo + 4 > step {
        return Err(ExtractError(
            "xyz field offsets do not fit within point_step",
        ));
    }
    if msg.is_bigendian {
        return Err(ExtractError("big-endian point data not supported"));
    }

    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let base = i * step;
        let x = read_f32_le(&msg.data, base + xo);
        let y = read_f32_le(&msg.data, base + yo);
        let z = read_f32_le(&msg.data, base + zo);
        if x.is_finite() && y.is_finite() && z.is_finite() {
            out.push([x, y, z]);
        }
    }
    Ok(out)
}

#[inline]
fn read_f32_le(buf: &[u8], off: usize) -> f32 {
    let bytes: [u8; 4] = buf[off..off + 4]
        .try_into()
        .expect("bounds checked by caller");
    f32::from_le_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// An xyz float32 cloud, the shape RayTracingVoxelMap publishes.
    pub(crate) fn cloud_of(points: &[[f32; 3]]) -> PointCloud2 {
        let mut data = Vec::with_capacity(points.len() * 12);
        for p in points {
            for v in p {
                data.extend_from_slice(&v.to_le_bytes());
            }
        }
        let field = |name: &str, off: i32| PointField {
            name: name.into(),
            offset: off,
            datatype: PointField::FLOAT32 as u8,
            count: 1,
        };
        PointCloud2 {
            header: Header::default(),
            height: 1,
            width: points.len() as i32,
            fields: vec![field("x", 0), field("y", 4), field("z", 8)],
            is_bigendian: false,
            point_step: 12,
            row_step: 12 * points.len() as i32,
            data,
            is_dense: true,
        }
    }

    #[test]
    fn yaw_round_trips_across_the_wrap() {
        for yaw in [-PI + 1e-9, -2.0, -0.1, 0.0, 0.1, 2.0, PI - 1e-9] {
            let got = yaw_of(&quat_of_yaw(yaw));
            assert!((got - yaw).abs() < 1e-12, "{yaw} -> {got}");
        }
    }

    #[test]
    fn stamps_round_trip_through_the_wire_form() {
        // a real planner t0 is a unix second count, so the nanosecond field is
        // where the precision profile actually lives
        for ts in [0.0, 1754212345.75, 1754212345.000000001, 7.0] {
            let got = secs_of(&time_of_secs(ts));
            assert!((got - ts).abs() < 1e-6, "{ts} -> {got}");
        }
    }

    #[test]
    fn a_non_finite_stamp_does_not_wrap_around() {
        // `as i32` on a NaN or an out-of-range float is a saturating cast, but
        // a zero stamp is the honest "no time" rather than i32::MAX seconds
        assert_eq!(time_of_secs(f64::NAN), Time { sec: 0, nsec: 0 });
        assert_eq!(time_of_secs(f64::INFINITY), Time { sec: 0, nsec: 0 });
    }

    #[test]
    fn a_built_path_carries_its_stamps_and_yaws() {
        let states = vec![[0.0, 0.0, 0.0], [1.0, 0.0, PI / 2.0]];
        let path = build_path(&states, &[10.0, 12.0], 10.0, "odom", 0.0);
        assert_eq!(path.header.frame_id, "odom");
        assert_eq!(secs_of(&path.header.stamp), 10.0);
        assert_eq!(path.poses[1].pose.position.x, 1.0);
        assert!((secs_of(&path.poses[1].header.stamp) - 12.0).abs() < 1e-6);
        // and reads back as the same rows the laws take
        let back = path_states(&path);
        for (a, b) in back.iter().zip(states.iter()) {
            for k in 0..3 {
                assert!((a[k] - b[k]).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn a_wrong_length_stamp_vector_leaves_the_path_unstamped() {
        // flat ts is exactly what `decode_ceilings` rejects, which is the
        // honest signal for "this plan carries no profile"
        let states = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let path = build_path(&states, &[], 5.0, "odom", 0.0);
        assert_eq!(path_stamps(&path), vec![5.0, 5.0]);
    }

    #[test]
    fn extract_drops_non_finite_points_and_moves_nothing() {
        let cloud = cloud_of(&[[1.0, 2.0, 0.3], [f32::NAN, 0.0, 0.0], [0.0, 0.0, 0.0]]);
        let pts = extract_xyz(&cloud).expect("well-formed cloud");
        assert_eq!(pts, vec![[1.0, 2.0, 0.3], [0.0, 0.0, 0.0]]);
    }

    #[test]
    fn extract_refuses_a_cloud_it_cannot_read() {
        let mut cloud = cloud_of(&[[0.0, 0.0, 0.0]]);
        cloud.fields.remove(2); // no z
        assert!(extract_xyz(&cloud).is_err());

        let mut cloud = cloud_of(&[[0.0, 0.0, 0.0]]);
        cloud.is_bigendian = true;
        assert!(extract_xyz(&cloud).is_err());

        let mut cloud = cloud_of(&[[0.0, 0.0, 0.0]]);
        cloud.width = 99; // claims more points than the buffer holds
        assert!(extract_xyz(&cloud).is_err());
    }
}
