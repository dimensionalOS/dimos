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

//! The base pose off tf, read per tick -- the rust twin of `TfPose` in
//! `dimos/navigation/tf_pose.py`, which is the specification.
//!
//! go2_tf publishes `odom -> mid360_link` off odometry at odometry rate and
//! the static mounts beside it, so tf carries `odom -> base_link` -- LIO's
//! sensor pose already resolved into the body (0.30 m back and 0.16 m down on
//! this rig). The deadman is on OUR clock, keyed by the edge's stamp: the
//! stamp is the robot's clock, which need not be ours, and what the deadman
//! asks is how long since tf last moved.

use std::time::Instant;

use dimos_module::nalgebra::{Isometry3, Translation3};
use dimos_module::{Tf, Transform};

use crate::msg::{yaw_of, State};

/// The base as one tick sees it: the planar state, and the height the floor
/// prior needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BasePose {
    pub state: State,
    pub z: f64,
}

/// The latest `world -> base` edge, `None` while it is missing or once its
/// stamp has stopped advancing for `max_age_s`.
pub struct PoseWatch {
    max_age_s: f64,
    /// The stamp last seen, and when it was first seen.
    seen: Option<(f64, Instant)>,
}

impl PoseWatch {
    pub fn new(max_age_s: f64) -> Self {
        Self {
            max_age_s,
            seen: None,
        }
    }

    pub fn get(&mut self, tf: &Tf, world: &str, base: &str, now: Instant) -> Option<BasePose> {
        self.observe(tf.get_latest(world, base).map(edge_of), now)
    }

    /// The tick's pose off what tf handed back, as `(stamp, transform)`.
    pub fn observe(
        &mut self,
        edge: Option<(f64, Isometry3<f64>)>,
        now: Instant,
    ) -> Option<BasePose> {
        let Some((ts, iso)) = edge else {
            self.seen = None;
            return None;
        };
        let since = match self.seen {
            Some((seen_ts, since)) if seen_ts == ts => since,
            _ => {
                self.seen = Some((ts, now));
                now
            }
        };
        if now.duration_since(since).as_secs_f64() > self.max_age_s {
            return None;
        }
        Some(BasePose {
            state: state_of(&iso),
            z: iso.translation.z,
        })
    }
}

fn edge_of(t: Transform) -> (f64, Isometry3<f64>) {
    (
        t.ts,
        Isometry3::from_parts(Translation3::from(t.translation()), t.rotation()),
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
    use dimos_module::nalgebra::UnitQuaternion;
    use std::time::Duration;

    fn body_at(x: f64, yaw: f64, ts: f64) -> Option<(f64, Isometry3<f64>)> {
        Some((
            ts,
            Isometry3::from_parts(
                Translation3::new(x, 0.0, 0.3),
                UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
            ),
        ))
    }

    #[test]
    fn the_pose_is_the_edge_with_its_height_kept() {
        let mut watch = PoseWatch::new(2.5);
        let got = watch
            .observe(body_at(1.0, 0.5, 5.0), Instant::now())
            .expect("live");
        assert!((got.state[0] - 1.0).abs() < 1e-12, "{got:?}");
        assert!((got.state[2] - 0.5).abs() < 1e-12, "{got:?}");
        assert!((got.z - 0.3).abs() < 1e-12, "{got:?}");
    }

    #[test]
    fn a_missing_edge_is_no_pose() {
        let mut watch = PoseWatch::new(2.5);
        assert_eq!(watch.observe(None, Instant::now()), None);
    }

    #[test]
    fn the_pose_goes_stale_when_the_stamp_stops_advancing() {
        let mut watch = PoseWatch::new(2.5);
        let t0 = Instant::now();
        assert!(watch.observe(body_at(1.0, 0.0, 5.0), t0).is_some());
        // the boundary is exclusive
        assert!(watch
            .observe(body_at(1.0, 0.0, 5.0), t0 + Duration::from_secs_f64(2.5))
            .is_some());
        assert!(watch
            .observe(body_at(1.0, 0.0, 5.0), t0 + Duration::from_secs_f64(2.6))
            .is_none());
        // a fresh stamp is a live edge again, whatever the robot's clock says
        assert!(watch
            .observe(body_at(2.0, 0.0, 5.1), t0 + Duration::from_secs_f64(2.7))
            .is_some());
    }

    #[test]
    fn age_is_measured_on_our_clock_not_the_stamp() {
        let mut watch = PoseWatch::new(2.5);
        // a robot clock nowhere near ours
        assert!(watch
            .observe(body_at(1.0, 0.0, 0.0), Instant::now())
            .is_some());
    }

    #[test]
    fn a_dropped_edge_forgets_the_spell_it_was_counting() {
        let mut watch = PoseWatch::new(2.5);
        let t0 = Instant::now();
        assert!(watch.observe(body_at(1.0, 0.0, 5.0), t0).is_some());
        assert!(watch.observe(None, t0 + Duration::from_secs(1)).is_none());
        // the same stamp back is a new sighting, not the old one aged
        assert!(watch
            .observe(body_at(1.0, 0.0, 5.0), t0 + Duration::from_secs(4))
            .is_some());
    }

    #[test]
    fn the_yaw_round_trips_through_the_lcm_quaternion() {
        for yaw in [-3.0, -1.0, 0.0, 0.7, 3.0] {
            let iso = Isometry3::from_parts(
                Translation3::new(0.0, 0.0, 0.0),
                UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
            );
            let got = state_of(&iso);
            assert!((got[2] - yaw).abs() < 1e-12, "yaw {yaw} -> {got:?}");
        }
    }
}
