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

//! Marshalling between dimos messages and the plain arrays the pure crates take;
//! shared by both modules so a plan is written and read by one convention.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dimos_generated_messages::builtin_interfaces::msg::time::Time;
use dimos_generated_messages::geometry_msgs::msg::{
    point::Point, pose::Pose, pose_stamped::PoseStamped, quaternion::Quaternion, twist::Twist,
    vector3::Vector3,
};
use dimos_generated_messages::nav_msgs::msg::path::Path;
use dimos_generated_messages::sensor_msgs::msg::{
    point_cloud2::PointCloud2, point_field::PointField,
};
use dimos_generated_messages::std_msgs::msg::header::Header;
use dimos_module::cdr;
use dimos_module::{error_throttled, Output};

/// A plan waypoint as the laws and the planner see it: `(x, y, yaw)`.
pub type State = [f64; 3];

/// Yaw of a quaternion, matching `Quaternion.euler[2]` (scipy "xyz" extrinsic, the ROS
/// convention, closed form).
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
    t.sec as f64 + t.nanosec as f64 * 1e-9
}

pub fn nanoseconds(time: &Time) -> i64 {
    time.sec as i64 * 1_000_000_000 + time.nanosec as i64
}

/// Normalize a floating second count into the canonical ROS Time fields.
pub fn time_of_secs(ts: f64) -> Time {
    if !ts.is_finite() {
        return Time { sec: 0, nanosec: 0 };
    }
    let ns = (ts * 1_000_000_000.).round() as i64;
    let sec = ns
        .div_euclid(1_000_000_000)
        .clamp(i32::MIN as i64, i32::MAX as i64) as i32;
    Time {
        sec,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
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
        stamp: time_of_secs(ts),
        frame_id: frame_id.into(),
    }
}

/// One plan waypoint as a stamped pose on the plane at `ground_z`; `odom` z = 0 is
/// wherever LIO started, and only a viewer reads the z.
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

/// A plan as a nav Path, per-waypoint stamps carrying the precision profile;
/// `stamps` of the wrong length leaves every pose at `t0` (unstamped to `decode_ceilings`).
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

/// An xyz float32 cloud.
pub fn build_cloud(points: &[[f32; 3]], frame_id: &str, ts: f64) -> PointCloud2 {
    let mut data = Vec::with_capacity(points.len() * 12);
    for p in points {
        for v in p {
            data.extend_from_slice(&v.to_le_bytes());
        }
    }
    let field = |name: &str, offset: u32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32,
        count: 1,
    };
    PointCloud2 {
        header: header(frame_id, ts),
        height: 1,
        width: points.len() as u32,
        fields: vec![field("x", 0), field("y", 4), field("z", 8)],
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * points.len() as u32,
        data,
        is_dense: true,
    }
}

/// The plan as `(x, y, yaw)` rows: the python `controller.path_xy_yaw`.
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

/// Relative seconds for laws that consume only profile deltas.
pub fn relative_path_stamps(path: &Path) -> Vec<f64> {
    let Some(first) = path.poses.first() else {
        return Vec::new();
    };
    let base = nanoseconds(&first.header.stamp);
    path.poses
        .iter()
        .map(|pose| (nanoseconds(&pose.header.stamp) - base) as f64 / 1_000_000_000.)
        .collect()
}

/// A body-frame twist, the only shape either module publishes.
pub fn twist(vx: f64, vy: f64, wz: f64) -> Twist {
    Twist {
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

// Reading a cloud off the wire is dimos-module's; the path stays `msg::extract_xyz`.
pub use dimos_module::pointcloud::ExtractError;

pub fn extract_xyz(
    cloud: &dimos_generated_messages::sensor_msgs::msg::point_cloud2::PointCloud2,
) -> Result<Vec<[f32; 3]>, ExtractError> {
    use dimos_generated_messages::sensor_msgs::msg::point_field::PointField;
    let offset = |name: &str| {
        cloud
            .fields
            .iter()
            .find(|f| f.name == name && f.datatype == PointField::FLOAT32 && f.count == 1)
            .map(|f| f.offset as usize)
            .ok_or(ExtractError("missing scalar float32 xyz field"))
    };
    dimos_module::pointcloud::extract_xyz_rows(
        &cloud.data,
        [cloud.width as usize, cloud.height as usize],
        [cloud.point_step as usize, cloud.row_step as usize],
        [offset("x")?, offset("y")?, offset("z")?],
        cloud.is_bigendian,
    )
}

pub fn encode<T: serde::Serialize>(value: &T) -> Vec<u8> {
    dimos_module::cdr::encode(value).expect("serialize a valid generated navigation value")
}

pub fn decode_tf(bytes: &[u8]) -> std::io::Result<Vec<dimos_module::tf::Transform>> {
    use dimos_generated_messages::tf2_msgs::msg::tf_message::TFMessage;
    use dimos_module::nalgebra::{Quaternion, Translation3, UnitQuaternion};
    let message: TFMessage = cdr::decode(bytes)?;
    message
        .transforms
        .into_iter()
        .map(|edge| {
            let p = edge.transform.translation;
            let q = edge.transform.rotation;
            if ![p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                .iter()
                .all(|v| v.is_finite())
            {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "nonfinite TF components",
                ));
            }
            let rotation = UnitQuaternion::try_new(Quaternion::new(q.w, q.x, q.y, q.z), 1e-9)
                .ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidData, "zero TF quaternion")
                })?;
            if edge.header.stamp.nanosec >= 1_000_000_000 {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "unnormalized TF stamp",
                ));
            }
            Ok(dimos_module::tf::Transform::from_nanoseconds(
                edge.header.frame_id,
                edge.child_frame_id,
                nanoseconds(&edge.header.stamp),
                dimos_module::nalgebra::Isometry3::from_parts(
                    Translation3::new(p.x, p.y, p.z),
                    rotation,
                ),
            ))
        })
        .collect()
}

pub fn encode_tf(transforms: &[dimos_module::tf::Transform]) -> std::io::Result<Vec<u8>> {
    use dimos_generated_messages::{
        geometry_msgs::msg::{
            quaternion::Quaternion, transform::Transform, transform_stamped::TransformStamped,
            vector3::Vector3,
        },
        tf2_msgs::msg::tf_message::TFMessage,
    };
    let transforms = transforms
        .iter()
        .map(|value| {
            let ns = value.stamp_nanoseconds().ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "TF timestamp is outside range",
                )
            })?;
            let sec = i32::try_from(ns.div_euclid(1_000_000_000)).map_err(|_| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "TF seconds exceed ROS int32",
                )
            })?;
            let p = value.translation();
            let q = value.rotation();
            Ok(TransformStamped {
                header: Header {
                    stamp: Time {
                        sec,
                        nanosec: ns.rem_euclid(1_000_000_000) as u32,
                    },
                    frame_id: value.parent.clone(),
                },
                child_frame_id: value.child.clone(),
                transform: Transform {
                    translation: Vector3 {
                        x: p.x,
                        y: p.y,
                        z: p.z,
                    },
                    rotation: Quaternion {
                        x: q.i,
                        y: q.j,
                        z: q.k,
                        w: q.w,
                    },
                },
            })
        })
        .collect::<std::io::Result<Vec<_>>>()?;
    cdr::encode(&TFMessage { transforms })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn yaw_round_trips_across_the_wrap() {
        for yaw in [-PI + 1e-9, -2.0, -0.1, 0.0, 0.1, 2.0, PI - 1e-9] {
            let got = yaw_of(&quat_of_yaw(yaw));
            assert!((got - yaw).abs() < 1e-12, "{yaw} -> {got}");
        }
    }

    #[test]
    fn stamps_round_trip_through_the_wire_form() {
        // a real t0 is a unix second count, so the profile lives in the nanosecond field
        for ts in [0.0, 1754212345.75, 1754212345.000000001, 7.0] {
            let got = secs_of(&time_of_secs(ts));
            assert!((got - ts).abs() < 1e-6, "{ts} -> {got}");
        }
    }

    #[test]
    fn a_non_finite_stamp_does_not_wrap_around() {
        // a saturating `as i32` would give i32::MAX seconds; zero is the honest "no time"
        assert_eq!(time_of_secs(f64::NAN), Time { sec: 0, nanosec: 0 });
        assert_eq!(time_of_secs(f64::INFINITY), Time { sec: 0, nanosec: 0 });
    }

    #[test]
    fn a_built_path_carries_its_stamps_and_yaws() {
        let states = vec![[0.0, 0.0, 0.0], [1.0, 0.0, PI / 2.0]];
        let path = build_path(&states, &[10.0, 12.0], 10.0, "odom", 0.0);
        assert_eq!(path.header.frame_id, "odom");
        assert_eq!(secs_of(&path.header.stamp), 10.0);
        assert_eq!(path.poses[1].pose.position.x, 1.0);
        assert!((secs_of(&path.poses[1].header.stamp) - 12.0).abs() < 1e-6);
        let back = path_states(&path);
        for (a, b) in back.iter().zip(states.iter()) {
            for k in 0..3 {
                assert!((a[k] - b[k]).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn a_wrong_length_stamp_vector_leaves_the_path_unstamped() {
        let states = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let path = build_path(&states, &[], 5.0, "odom", 0.0);
        assert_eq!(path_stamps(&path), vec![5.0, 5.0]);
    }
    #[test]
    fn generated_tf_cdr_round_trip_retains_exact_wire_time() {
        use dimos_generated_messages::tf2_msgs::msg::tf_message::TFMessage;
        use dimos_module::nalgebra::Isometry3;
        for ns in [0, -1, 1_700_000_000_123_456_789] {
            let source = dimos_module::Transform::from_nanoseconds(
                "world",
                "base",
                ns,
                Isometry3::translation(1., 2., 3.),
            );
            let payload = encode_tf(&[source]).unwrap();
            let value: TFMessage = cdr::decode(&payload).unwrap();
            assert_eq!(nanoseconds(&value.transforms[0].header.stamp), ns);
            assert_eq!(value.transforms[0].header.frame_id, "world");
            for little in [false, true] {
                let received = decode_tf(&cdr::encode_endian(&value, little).unwrap()).unwrap();
                assert_eq!(received[0].stamp_nanoseconds(), Some(ns));
                assert_eq!(received[0].translation().x, 1.);
            }
        }
    }

    #[test]
    fn tf_codec_rejects_invalid_packets_and_ros_time() {
        use dimos_generated_messages::tf2_msgs::msg::tf_message::TFMessage;
        use dimos_module::nalgebra::Isometry3;
        let payload = encode_tf(&[dimos_module::Transform::from_nanoseconds(
            "a",
            "b",
            0,
            Isometry3::identity(),
        )])
        .unwrap();
        let mut message: TFMessage = cdr::decode(&payload).unwrap();
        message.transforms[0].header.stamp.nanosec = 1_000_000_000;
        assert!(decode_tf(&encode(&message)).is_err());
        assert!(decode_tf(&[0, 3, 0, 0]).is_err());
        let mut trailing = payload;
        trailing.push(0);
        assert!(decode_tf(&trailing).is_err());
    }

    #[test]
    fn generated_path_codec_and_relative_profile_keep_nanosecond_differences() {
        let mut path = build_path(
            &[[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]],
            &[],
            0.,
            "map",
            0.,
        );
        let base = 1_700_000_000_123_456_789i64;
        for (pose, delta) in path.poses.iter_mut().zip([0, 1, 3]) {
            let ns = base + delta;
            pose.header.stamp = Time {
                sec: ns.div_euclid(1_000_000_000) as i32,
                nanosec: ns.rem_euclid(1_000_000_000) as u32,
            };
        }
        for little in [false, true] {
            let payload = cdr::encode_endian(&path, little).unwrap();
            let decoded: Path = cdr::decode(&payload).unwrap();
            assert_eq!(relative_path_stamps(&decoded), vec![0., 1e-9, 3e-9]);
            assert_eq!(decoded.header.frame_id, "map");
        }
        assert_eq!(
            time_of_secs(-0.25),
            Time {
                sec: -1,
                nanosec: 750_000_000
            }
        );
    }

    #[test]
    fn generated_cloud_layout_handles_padding_endian_and_cdr() {
        use dimos_generated_messages::sensor_msgs::msg::{
            point_cloud2::PointCloud2, point_field::PointField,
        };
        let values = [[1f32, 2., 3.], [4., 5., 6.], [7., 8., 9.], [10., 11., 12.]];
        for big in [false, true] {
            let mut data = vec![0u8; 80];
            for (index, point) in values.iter().enumerate() {
                for (axis, value) in point.iter().enumerate() {
                    let offset = (index / 2) * 40 + (index % 2) * 16 + axis * 4;
                    let bytes = if big {
                        value.to_be_bytes()
                    } else {
                        value.to_le_bytes()
                    };
                    data[offset..offset + 4].copy_from_slice(&bytes);
                }
            }
            let fields = ["x", "y", "z"]
                .iter()
                .enumerate()
                .map(|(i, name)| PointField {
                    name: (*name).into(),
                    offset: (i * 4) as u32,
                    datatype: PointField::FLOAT32,
                    count: 1,
                })
                .collect();
            let cloud = PointCloud2 {
                header: header("map", 0.),
                height: 2,
                width: 2,
                fields,
                is_bigendian: big,
                point_step: 16,
                row_step: 40,
                data,
                is_dense: true,
            };
            let mut decoded: PointCloud2 = cdr::decode(&encode(&cloud)).unwrap();
            assert_eq!(extract_xyz(&decoded).unwrap(), values);
            decoded.row_step = 7;
            assert!(extract_xyz(&decoded).is_err());
        }
    }
}
