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

//! Resolve odometry into the base frame off tf -- the rust twin of
//! `dimos/navigation/tf_pose.py`, which is the specification.
//!
//! LIO reports where the SENSOR is. On the Go2 that is 0.30 m ahead of the
//! robot and 0.16 m above it, so a stack that reads odometry as the body pose
//! plans a footprint that is not where the robot is. The mount is a rotation
//! AND a lever arm; composing the full transform handles both, which is why
//! this replaced the mount-quaternion module that only handled the first.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use dimos_module::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use dimos_module::Tf;
use lcm_msgs::geometry_msgs::Pose;
use lcm_msgs::nav_msgs::Odometry;
use tracing::{info, warn};

use crate::msg::{yaw_of, State};

/// While the leg is missing, retry the lookup at most this often. The buffer
/// warns on every miss, so per-message retries would flood the log.
const RETRY_PERIOD: Duration = Duration::from_secs(1);

/// A parent -> child transform lookup. `Tf` is the only implementation that
/// ships; the indirection exists so the arithmetic below is testable without a
/// live transport, which is the only way to build a `Tf`.
type LegLookup = Box<dyn Fn(&str, &str) -> Option<Isometry3<f64>> + Send>;

/// Turn odometry messages into the base-frame `(x, y, yaw)` they imply.
pub struct OdomBasePose {
    lookup: LegLookup,
    base_frame: String,
    legs: HashMap<String, Isometry3<f64>>,
    waiting: bool,
    next_lookup: Option<Instant>,
}

impl OdomBasePose {
    pub fn new(tf: Tf, base_frame: impl Into<String>) -> Self {
        Self::with_lookup(
            Box::new(move |parent, child| tf.get_latest(parent, child).map(iso_of_transform)),
            base_frame,
        )
    }

    fn with_lookup(lookup: LegLookup, base_frame: impl Into<String>) -> Self {
        Self {
            lookup,
            base_frame: base_frame.into(),
            legs: HashMap::new(),
            waiting: false,
            next_lookup: None,
        }
    }

    /// The base pose one message implies. `None` until tf has the mount leg,
    /// which the caller must DROP rather than fall back on: the sensor pose
    /// wearing the body's name is the bug this exists to fix.
    pub fn resolve(&mut self, msg: &Odometry) -> Option<State> {
        self.resolve_iso(msg).map(|iso| state_of(&iso))
    }

    /// The full base transform, for callers that need more than `(x, y, yaw)`
    /// -- the floor prior needs the base's HEIGHT.
    pub fn resolve_iso(&mut self, msg: &Odometry) -> Option<Isometry3<f64>> {
        let pose = &msg.pose.pose;
        if msg.child_frame_id == self.base_frame {
            return Some(iso_of(pose));
        }
        let leg = self.sensor_to_base(&msg.child_frame_id)?;
        Some(iso_of(pose) * leg)
    }

    /// How far the base sits above the ground while standing, off the mount
    /// leg. `None` for base-stamped odometry, which has no leg to subtract --
    /// the twin of `tf_pose.base_height_above_ground` plus GoalRelay's guard.
    pub fn base_height_above_ground(
        &mut self,
        sensor_frame: &str,
        lidar_height: f64,
    ) -> Option<f64> {
        if sensor_frame == self.base_frame {
            return None;
        }
        let leg = self.sensor_to_base(sensor_frame)?;
        Some(lidar_height - leg.inverse().translation.z)
    }

    /// The cached static sensor -> base leg. Logs once per outage, not per
    /// message.
    fn sensor_to_base(&mut self, sensor_frame: &str) -> Option<Isometry3<f64>> {
        if let Some(leg) = self.legs.get(sensor_frame) {
            return Some(*leg);
        }
        if self.waiting && self.next_lookup.is_some_and(|at| Instant::now() < at) {
            return None;
        }
        // dimos stamps a transform parent -> child, i.e. the pose of `child`
        // expressed in `parent` -- the same order tf.rs takes.
        let Some(leg) = (self.lookup)(sensor_frame, &self.base_frame) else {
            self.next_lookup = Some(Instant::now() + RETRY_PERIOD);
            if !self.waiting {
                self.waiting = true;
                warn!(
                    sensor_frame,
                    base_frame = %self.base_frame,
                    "no transform on tf yet, dropping odometry until it arrives",
                );
            }
            return None;
        };
        if self.waiting {
            self.waiting = false;
            info!(sensor_frame, base_frame = %self.base_frame, "got the transform, resuming");
        }
        self.legs.insert(sensor_frame.to_string(), leg);
        Some(leg)
    }
}

fn iso_of_transform(t: dimos_module::Transform) -> Isometry3<f64> {
    Isometry3::from_parts(Translation3::from(t.translation()), t.rotation())
}

fn iso_of(pose: &Pose) -> Isometry3<f64> {
    let q = &pose.orientation;
    Isometry3::from_parts(
        Translation3::new(pose.position.x, pose.position.y, pose.position.z),
        UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z)),
    )
}

/// The planar `(x, y, yaw)` a full transform implies.
pub fn state_of(iso: &Isometry3<f64>) -> State {
    let q = iso.rotation.quaternion();
    let yaw = yaw_of(&lcm_msgs::geometry_msgs::Quaternion {
        x: q.i,
        y: q.j,
        z: q.k,
        w: q.w,
    });
    [iso.translation.x, iso.translation.y, yaw]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::quat_of_yaw;
    use lcm_msgs::geometry_msgs::Point;

    /// The Go2's real mount as GO2Zenoh publishes it: base_link -> front_camera
    /// plus front_camera -> mid360_link, i.e. 0.295 m ahead and 0.163 m up. The
    /// python twin pins the same numbers.
    const LEVER: [f64; 3] = [0.29515, -0.00003, 0.16297];

    fn pose(x: f64, y: f64, yaw: f64) -> Pose {
        Pose {
            position: Point { x, y, z: 0.0 },
            orientation: quat_of_yaw(yaw),
        }
    }

    fn odometry(child_frame_id: &str, x: f64, y: f64, yaw: f64) -> Odometry {
        let mut msg = Odometry {
            child_frame_id: child_frame_id.to_string(),
            ..Default::default()
        };
        msg.pose.pose = pose(x, y, yaw);
        msg
    }

    /// The leg as tf hands it back: mid360_link -> base_link, so the arm points
    /// BACK from the sensor to the body. Rotation dropped: what these tests
    /// check is the arm, and the levelling is nalgebra's.
    fn arm(xyz: [f64; 3]) -> LegLookup {
        Box::new(move |_, _| {
            Some(Isometry3::from_parts(
                Translation3::new(-xyz[0], -xyz[1], -xyz[2]),
                UnitQuaternion::identity(),
            ))
        })
    }

    #[test]
    fn the_arm_comes_off_along_the_body_axes() {
        // facing +x, so the arm subtracts straight off x
        let mut resolver = OdomBasePose::with_lookup(arm(LEVER), "base_link");
        let got = resolver
            .resolve(&odometry("mid360_link", 1.0, 2.0, 0.0))
            .expect("the leg is there");
        assert!((got[0] - (1.0 - LEVER[0])).abs() < 1e-12, "{got:?}");
        assert!((got[1] - (2.0 - LEVER[1])).abs() < 1e-12, "{got:?}");
    }

    #[test]
    fn the_arm_swings_with_the_body_not_the_map() {
        // yawed a quarter turn, a forward arm comes off y, not x
        let mut resolver = OdomBasePose::with_lookup(arm([1.0, 0.0, 0.0]), "base_link");
        let yaw = std::f64::consts::FRAC_PI_2;
        let got = resolver
            .resolve(&odometry("mid360_link", 1.0, 2.0, yaw))
            .expect("the leg is there");
        assert!((got[0] - 1.0).abs() < 1e-12, "{got:?}");
        assert!((got[1] - 1.0).abs() < 1e-12, "{got:?}");
        assert!((got[2] - yaw).abs() < 1e-12, "{got:?}");
    }

    #[test]
    fn a_base_stamped_message_passes_straight_through() {
        let mut resolver = OdomBasePose::with_lookup(Box::new(|_, _| None), "base_link");
        let got = resolver
            .resolve(&odometry("base_link", 1.0, 2.0, 0.5))
            .expect("base-stamped needs no tf");
        assert!((got[0] - 1.0).abs() < 1e-12, "{got:?}");
        assert!((got[1] - 2.0).abs() < 1e-12, "{got:?}");
        assert!((got[2] - 0.5).abs() < 1e-12, "{got:?}");
    }

    #[test]
    fn a_sensor_stamped_message_is_dropped_until_tf_has_the_leg() {
        let mut resolver = OdomBasePose::with_lookup(Box::new(|_, _| None), "base_link");
        assert!(resolver
            .resolve(&odometry("mid360_link", 1.0, 2.0, 0.0))
            .is_none());
    }

    #[test]
    fn the_leg_is_looked_up_once_and_then_cached() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;
        let calls = Arc::new(AtomicUsize::new(0));
        let seen = Arc::clone(&calls);
        let mut resolver = OdomBasePose::with_lookup(
            Box::new(move |_, _| {
                seen.fetch_add(1, Ordering::Relaxed);
                Some(Isometry3::identity())
            }),
            "base_link",
        );
        for _ in 0..5 {
            assert!(resolver
                .resolve(&odometry("mid360_link", 0.0, 0.0, 0.0))
                .is_some());
        }
        assert_eq!(calls.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn the_base_height_is_the_lidar_height_less_the_mount() {
        let mut resolver = OdomBasePose::with_lookup(arm(LEVER), "base_link");
        let got = resolver
            .base_height_above_ground("mid360_link", 0.45)
            .expect("the leg is there");
        assert!((got - (0.45 - LEVER[2])).abs() < 1e-12, "{got}");
    }

    #[test]
    fn base_stamped_odometry_has_no_base_height() {
        let mut resolver = OdomBasePose::with_lookup(arm(LEVER), "base_link");
        assert_eq!(resolver.base_height_above_ground("base_link", 0.45), None);
    }

    #[test]
    fn the_resolved_iso_keeps_the_height_the_state_drops() {
        let mut resolver = OdomBasePose::with_lookup(arm(LEVER), "base_link");
        let mut msg = odometry("mid360_link", 1.0, 2.0, 0.0);
        msg.pose.pose.position.z = 0.34;
        let iso = resolver.resolve_iso(&msg).expect("the leg is there");
        assert!(
            (iso.translation.z - (0.34 - LEVER[2])).abs() < 1e-12,
            "{iso:?}"
        );
    }

    #[test]
    fn the_yaw_round_trips_through_the_lcm_quaternion() {
        for yaw in [-3.0, -1.0, 0.0, 0.7, 3.0] {
            let got = state_of(&iso_of(&pose(0.0, 0.0, yaw)));
            assert!((got[2] - yaw).abs() < 1e-12, "yaw {yaw} -> {got:?}");
        }
    }
}
