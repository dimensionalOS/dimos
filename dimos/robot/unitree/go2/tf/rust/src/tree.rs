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

//! The Go2's tf tree as geometry: no transport, no clock.
//!
//! The quaternion math is transcribed line for line from dimos' python
//! (`msgs/geometry_msgs/Quaternion.py`, `msgs/geometry_msgs/Transform.py`)
//! rather than taken from a rotation crate. This module's whole job is to
//! reproduce what `GO2Zenoh.transforms()` puts on tf, and a crate whose euler
//! order or quaternion handedness differed by a convention would not fail —
//! it would quietly place the body 0.30 m from where it is.

use std::f64::consts::FRAC_PI_2;

use dimos_module::native_config;
use lcm_msgs::geometry_msgs::TransformStamped;
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::std_msgs::{Header, Time};

/// Fixed rather than configurable: these are baked into the rest of the stack
/// (the planner and the follower resolve `base_link` by name), so a knob here
/// would only let a deployment disagree with itself.
pub const BASE_FRAME: &str = "base_link";
pub const CAMERA_FRAME: &str = "front_camera";
pub const LIDAR_FRAME: &str = "mid360_link";
pub const OPTICAL_FRAME: &str = "camera_optical";

/// rpy mapping a sensor frame to its optical frame (x-right, y-down, z-forward).
/// Not config either: it is the definition of an optical frame, not rig geometry.
const OPTICAL_RPY: [f64; 3] = [-FRAC_PI_2, 0.0, -FRAC_PI_2];

#[native_config]
pub struct Config {
    /// How often the static mount tree is re-published. tf has no latched path,
    /// so a one-shot publish would be missed by anything subscribing later.
    #[validate(range(exclusive_min = 0.0))]
    pub publish_hz: f64,
    /// base_link -> front_camera, metres.
    pub camera_xyz: [f64; 3],
    /// front_camera -> mid360_link, metres.
    pub mid360_xyz: [f64; 3],
    /// front_camera -> mid360_link, fixed-axis rpy in degrees. The 60 deg tilt
    /// lands on roll because the lidar sits yawed 90 deg on its bracket. Both
    /// yaw signs level the body but differ by 180 deg of heading — flip it if
    /// the camera looks backwards.
    pub mid360_mount_rpy_deg: [f64; 3],
}

/// A quaternion in dimos' storage order and multiplication convention.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    /// Fixed-axis rpy (radians) to quaternion — `Quaternion.from_euler`.
    pub fn from_euler(rpy: [f64; 3]) -> Self {
        let (sr, cr) = (rpy[0] * 0.5).sin_cos();
        let (sp, cp) = (rpy[1] * 0.5).sin_cos();
        let (sy, cy) = (rpy[2] * 0.5).sin_cos();
        Quat {
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
            w: cr * cp * cy + sr * sp * sy,
        }
    }

    pub fn from_euler_deg(rpy_deg: [f64; 3]) -> Self {
        Self::from_euler([
            rpy_deg[0].to_radians(),
            rpy_deg[1].to_radians(),
            rpy_deg[2].to_radians(),
        ])
    }

    pub fn conjugate(self) -> Self {
        Quat {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    /// The inverse. Every quaternion this module inverts comes out of
    /// [`Quat::from_euler`] and is therefore unit, which is the branch python's
    /// `Quaternion.inverse` takes as well — so this is the conjugate, exactly,
    /// not an approximation of the general conj/norm² form.
    pub fn inverse(self) -> Self {
        self.conjugate()
    }

    /// `q * v * q̄` — `Quaternion.rotate_vector`.
    pub fn rotate(self, v: [f64; 3]) -> [f64; 3] {
        let pure = Quat {
            x: v[0],
            y: v[1],
            z: v[2],
            w: 0.0,
        };
        let r = self * pure * self.conjugate();
        [r.x, r.y, r.z]
    }
}

/// Hamilton product. `a * b` rotates by `b` first, then `a` — the order
/// python's `Quaternion.__mul__` documents, and the reverse of how it reads.
impl std::ops::Mul for Quat {
    type Output = Quat;

    fn mul(self, other: Quat) -> Quat {
        Quat {
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        }
    }
}

/// One `parent -> child` edge of the tf tree.
#[derive(Clone, Debug)]
pub struct Edge {
    pub parent: String,
    pub child: String,
    pub translation: [f64; 3],
    pub rotation: Quat,
}

impl Edge {
    pub fn new(parent: &str, child: &str, translation: [f64; 3], rotation: Quat) -> Self {
        Edge {
            parent: parent.to_string(),
            child: child.to_string(),
            translation,
            rotation,
        }
    }

    /// The same rigid offset read the other way — python's `Transform.inverse`.
    pub fn inverse(&self) -> Edge {
        let rotation = self.rotation.inverse();
        let t = rotation.rotate(self.translation);
        Edge {
            parent: self.child.clone(),
            child: self.parent.clone(),
            translation: [-t[0], -t[1], -t[2]],
            rotation,
        }
    }

    /// Compose `self` (a -> b) with `other` (b -> c) — python's `Transform.__add__`.
    pub fn then(&self, other: &Edge) -> Edge {
        let rotated = self.rotation.rotate(other.translation);
        Edge {
            parent: self.parent.clone(),
            child: other.child.clone(),
            translation: [
                self.translation[0] + rotated[0],
                self.translation[1] + rotated[1],
                self.translation[2] + rotated[2],
            ],
            rotation: self.rotation * other.rotation,
        }
    }

    pub fn to_stamped(&self, ts: f64) -> TransformStamped {
        let (sec, nsec) = split_stamp(ts);
        TransformStamped {
            header: Header {
                seq: 0,
                stamp: Time { sec, nsec },
                frame_id: self.parent.clone(),
            },
            child_frame_id: self.child.clone(),
            transform: lcm_msgs::geometry_msgs::Transform {
                translation: lcm_msgs::geometry_msgs::Vector3 {
                    x: self.translation[0],
                    y: self.translation[1],
                    z: self.translation[2],
                },
                rotation: lcm_msgs::geometry_msgs::Quaternion {
                    x: self.rotation.x,
                    y: self.rotation.y,
                    z: self.rotation.z,
                    w: self.rotation.w,
                },
            },
        }
    }
}

/// The mount tree, rooted at mid360_link because Point-LIO owns that frame.
///
/// Measured outward from the body, but `odom -> mid360_link` is the only live
/// edge, so the two edges above the lidar are inverted — otherwise mid360_link
/// has two parents and the body snaps between them at 35 Hz.
pub fn mount_tree(config: &Config) -> Vec<Edge> {
    let base_to_camera = Edge::new(BASE_FRAME, CAMERA_FRAME, config.camera_xyz, Quat::IDENTITY);
    let camera_to_mid360 = Edge::new(
        CAMERA_FRAME,
        LIDAR_FRAME,
        config.mid360_xyz,
        Quat::from_euler_deg(config.mid360_mount_rpy_deg),
    );
    let camera_to_optical = Edge::new(
        CAMERA_FRAME,
        OPTICAL_FRAME,
        [0.0; 3],
        Quat::from_euler(OPTICAL_RPY),
    );
    vec![
        camera_to_mid360.inverse(),
        base_to_camera.inverse(),
        camera_to_optical,
    ]
}

/// The one moving edge. Its frames come from the message, not from config —
/// python builds it as `Transform.from_pose(odom.child_frame_id,
/// odom.to_pose_stamped())`, so whatever the LIO called its frames is what lands
/// on tf.
pub fn odom_edge(odom: &Odometry) -> Edge {
    let p = &odom.pose.pose.position;
    let q = &odom.pose.pose.orientation;
    Edge {
        parent: odom.header.frame_id.clone(),
        child: odom.child_frame_id.clone(),
        translation: [p.x, p.y, p.z],
        rotation: Quat {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        },
    }
}

pub fn stamp_secs(header: &Header) -> f64 {
    f64::from(header.stamp.sec) + f64::from(header.stamp.nsec) * 1e-9
}

fn split_stamp(ts: f64) -> (i32, i32) {
    let mut sec = ts.floor();
    let mut nsec = ((ts - sec) * 1e9).round();
    if nsec >= 1e9 {
        sec += 1.0;
        nsec -= 1e9;
    }
    (sec as i32, nsec as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The rig geometry the wrapper defaults to, so the fixture cases below are
    /// the tree the robot actually publishes.
    fn config() -> Config {
        Config {
            publish_hz: 5.0,
            camera_xyz: [0.32715, -0.00003, 0.04297],
            mid360_xyz: [-0.032, 0.0, 0.12],
            mid360_mount_rpy_deg: [-60.0, 0.0, -90.0],
        }
    }

    fn assert_close(got: f64, want: f64, what: &str) {
        assert!(
            (got - want).abs() < 1e-12,
            "{what}: got {got}, want {want} (delta {})",
            (got - want).abs()
        );
    }

    /// Find an edge the way a tf buffer would — by the frames it joins, not by
    /// where it sits in the list. A tree that no longer carries this direction
    /// fails here, which is the point: inverting the wrong edge removes it.
    fn leg<'a>(tree: &'a [Edge], parent: &str, child: &str) -> &'a Edge {
        tree.iter()
            .find(|e| e.parent == parent && e.child == child)
            .unwrap_or_else(|| panic!("no {parent} -> {child} edge in the mount tree"))
    }

    /// Same transform, compared by what it does rather than by how it is spelled:
    /// a quaternion and its negation are the same rotation.
    fn assert_same_edge(got: &Edge, want_t: [f64; 3], want_r: Quat, what: &str) {
        for (i, (lhs, rhs)) in got.translation.iter().zip(want_t).enumerate() {
            assert_close(*lhs, rhs, &format!("{what} t[{i}]"));
        }
        for axis in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
            let turned = got.rotation.rotate(axis);
            for (i, (lhs, rhs)) in turned.iter().zip(want_r.rotate(axis)).enumerate() {
                assert_close(*lhs, rhs, &format!("{what} R*e[{i}]"));
            }
        }
    }

    #[test]
    fn inverting_an_edge_swaps_its_frames_and_undoes_it() {
        let edge = Edge::new(
            "a",
            "b",
            [0.3, -0.1, 0.2],
            Quat::from_euler_deg([-60.0, 10.0, -90.0]),
        );
        let back = edge.inverse();
        assert_eq!((back.parent.as_str(), back.child.as_str()), ("b", "a"));

        let identity = edge.then(&back);
        assert_same_edge(&identity, [0.0; 3], Quat::IDENTITY, "edge . edge^-1");
    }

    #[test]
    fn the_tree_is_rooted_at_the_lidar_so_nothing_else_parents_it() {
        let tree = mount_tree(&config());
        assert_eq!(tree.len(), 3);
        // Exactly one edge may end at mid360_link: the live odometry one, which
        // is not in this tree. Two parents and the body snaps between them.
        assert!(tree.iter().all(|e| e.child != LIDAR_FRAME));
        assert_eq!(
            tree.iter().filter(|e| e.parent == LIDAR_FRAME).count(),
            1,
            "the lidar must be the root of the mount tree"
        );
    }

    #[test]
    fn the_mount_offsets_survive_the_inversion() {
        let tree = mount_tree(&config());
        let base = leg(&tree, LIDAR_FRAME, CAMERA_FRAME).then(leg(&tree, CAMERA_FRAME, BASE_FRAME));
        // Read back the other way, the measured rig reappears: the lidar sits
        // 0.29515 m ahead of the body origin and 0.16297 m above it, tilted by
        // the mount rpy. A sign flip in either inversion moves this, and the
        // ground projections that use it, without anything failing.
        assert_same_edge(
            &base.inverse(),
            [0.32715 - 0.032, -0.00003, 0.04297 + 0.12],
            Quat::from_euler_deg([-60.0, 0.0, -90.0]),
            "base->lidar",
        );
    }

    #[test]
    fn a_stamp_round_trips_through_sec_nsec() {
        let edge = Edge::new("a", "b", [0.0; 3], Quat::IDENTITY);
        let stamped = edge.to_stamped(1_700_000_000.25);
        assert_eq!(stamped.header.stamp.sec, 1_700_000_000);
        assert_eq!(stamped.header.stamp.nsec, 250_000_000);
        assert_eq!(stamped.header.frame_id, "a");
        assert_eq!(stamped.child_frame_id, "b");
    }

    #[test]
    fn the_live_edge_takes_its_frames_and_pose_from_the_message() {
        let mut odom = Odometry::default();
        odom.header.frame_id = "odom".into();
        odom.header.stamp.sec = 42;
        odom.header.stamp.nsec = 500_000_000;
        odom.child_frame_id = "mid360_link".into();
        odom.pose.pose.position.x = 1.5;
        odom.pose.pose.orientation.w = 1.0;

        let edge = odom_edge(&odom);
        assert_eq!(
            (edge.parent.as_str(), edge.child.as_str()),
            ("odom", "mid360_link")
        );
        assert_close(edge.translation[0], 1.5, "x");
        assert_close(stamp_secs(&odom.header), 42.5, "ts");
    }

    // ---- parity with the python GO2Zenoh tree ----

    /// The fixture is written by `test_go2_tf.py` from the python module itself
    /// (`GO2Zenoh.transforms()` + `Transform.from_pose`), composed through the
    /// same `MultiTBuffer` the planner and follower read tf with. Both sides
    /// assert against it, so neither can drift alone.
    const PARITY: &str = include_str!("../../tf_parity.json");

    fn vec3(v: &serde_json::Value) -> [f64; 3] {
        [
            v[0].as_f64().unwrap(),
            v[1].as_f64().unwrap(),
            v[2].as_f64().unwrap(),
        ]
    }

    fn quat(v: &serde_json::Value) -> Quat {
        Quat {
            x: v[0].as_f64().unwrap(),
            y: v[1].as_f64().unwrap(),
            z: v[2].as_f64().unwrap(),
            w: v[3].as_f64().unwrap(),
        }
    }

    fn parity_config(v: &serde_json::Value) -> Config {
        Config {
            publish_hz: v["publish_hz"].as_f64().unwrap(),
            camera_xyz: vec3(&v["camera_xyz"]),
            mid360_xyz: vec3(&v["mid360_xyz"]),
            mid360_mount_rpy_deg: vec3(&v["mid360_mount_rpy_deg"]),
        }
    }

    #[test]
    fn the_static_tree_matches_the_python_edge_for_edge() {
        let fixture: serde_json::Value = serde_json::from_str(PARITY).unwrap();
        let tree = mount_tree(&parity_config(&fixture["config"]));
        let want = fixture["static"].as_array().unwrap();
        assert_eq!(
            tree.len(),
            want.len(),
            "the python publishes {} edges",
            want.len()
        );
        for want in want {
            let parent = want["parent"].as_str().unwrap();
            let child = want["child"].as_str().unwrap();
            assert_same_edge(
                leg(&tree, parent, child),
                vec3(&want["translation"]),
                quat(&want["rotation"]),
                &format!("{parent} -> {child}"),
            );
        }
    }

    /// The one that matters: for each odometry pose, resolve `odom -> base_link`
    /// through this crate's tree and check it against what the python's tf
    /// buffer resolves. A sign flip anywhere above the lidar lands here as a
    /// 0.30 m body offset, which is the bug this module could quietly ship.
    #[test]
    fn the_body_pose_matches_the_python_for_every_odometry_case() {
        let fixture: serde_json::Value = serde_json::from_str(PARITY).unwrap();
        let config = parity_config(&fixture["config"]);
        let tree = mount_tree(&config);

        for case in fixture["cases"].as_array().unwrap() {
            let mut odom = Odometry::default();
            odom.header.frame_id = case["odom"]["frame_id"].as_str().unwrap().into();
            odom.child_frame_id = case["odom"]["child_frame_id"].as_str().unwrap().into();
            let p = vec3(&case["odom"]["position"]);
            odom.pose.pose.position.x = p[0];
            odom.pose.pose.position.y = p[1];
            odom.pose.pose.position.z = p[2];
            let q = quat(&case["odom"]["orientation"]);
            odom.pose.pose.orientation.x = q.x;
            odom.pose.pose.orientation.y = q.y;
            odom.pose.pose.orientation.z = q.z;
            odom.pose.pose.orientation.w = q.w;

            // odom -> mid360_link -> front_camera -> base_link, the only route
            // through the published tree.
            let to_camera = odom_edge(&odom).then(leg(&tree, LIDAR_FRAME, CAMERA_FRAME));
            let base = to_camera.then(leg(&tree, CAMERA_FRAME, BASE_FRAME));
            assert_eq!(base.parent, "odom");
            assert_eq!(base.child, BASE_FRAME);
            assert_same_edge(
                &base,
                vec3(&case["base_link"]["translation"]),
                quat(&case["base_link"]["rotation"]),
                case["name"].as_str().unwrap(),
            );

            // And the optical leg, which hangs off the same camera frame.
            let optical = to_camera.then(leg(&tree, CAMERA_FRAME, OPTICAL_FRAME));
            assert_same_edge(
                &optical,
                vec3(&case["camera_optical"]["translation"]),
                quat(&case["camera_optical"]["rotation"]),
                "camera_optical",
            );
        }
    }
}
