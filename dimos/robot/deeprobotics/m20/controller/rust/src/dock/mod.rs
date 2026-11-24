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

//! Docking onto the M20's charging dock without the vendor charge_manager / reflective_column_node (NOS):
//!
//! - [`detector`]: reflective_column_node's per-sweep pose of the dock's two retro-reflective posts (decompiled, step
//!   for step) -> the dock's pose in base_link. Runs in its own module next to the lidars
//!   (`m20_dock_detector_module`), so full sweeps never reach the controller's process.
//! - [`docking`]: charge_manager's dock / undock sequence as a command source in front of the operator's, stepped
//!   by the controller's 1 kHz loop: NaviCharge to a staging point, then the fine point, FinetuneStand, lie down,
//!   confirm every pack's charge MOS (m20-battery's rule sends the charge-enable request on its own), retry by
//!   backing off; undock stands up and backs away.
//!
//! - [`vendor`]: charge_manager's config file and per-pose logic, and reflective_column_node's output stage.
//! - [`obstacle`]: charge_manager's obstacle check (the detector module runs it next to the lidars).
//! - [`pcl`]: the PCL 1.10 filters reflective_column_node runs, as the library does them.
//!
//! charge_manager.toml is read at startup; `[dock]` in m20_controller.toml holds what the binaries compile in.

pub mod config;
pub mod detector;
#[cfg(feature = "module")]
pub mod detector_module;
pub mod docking;
pub mod libm;
pub mod obstacle;
pub mod pcl;
pub mod vendor;
pub mod vendor_log;

/// The dock in base_link: the midpoint between the posts (m) and the direction of the dock's outward axis (rad), the
/// axis in the floor plane perpendicular to the posts' line that points from the posts toward an arriving robot. A
/// robot squarely facing the dock D metres away sees (D, 0, pi).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockPose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

/// Wrap an angle to (-pi, pi].
pub fn wrap(a: f64) -> f64 {
    let t = std::f64::consts::TAU;
    let r = a.rem_euclid(t);
    if r > std::f64::consts::PI {
        r - t
    } else {
        r
    }
}

impl DockPose {
    /// The robot's error to the pose `x_goal` metres out along the dock's axis, facing the posts, in base_link:
    /// (forward, left, yaw to turn), each 0 when there.
    pub fn error_to(&self, x_goal: f64) -> [f64; 3] {
        [
            self.x + x_goal * self.yaw.cos(),
            self.y + x_goal * self.yaw.sin(),
            wrap(self.yaw + std::f64::consts::PI),
        ]
    }

    /// The robot's own pose in the dock frame (x out along the axis, y to its left, yaw 0 = facing the posts).
    pub fn robot_in_dock(&self) -> [f64; 3] {
        let (s, c) = self.yaw.sin_cos();
        // inverse of (x, y, yaw): -R(-yaw) p, heading -yaw, measured from facing the posts (heading pi)
        let x = -(c * self.x + s * self.y);
        let y = -(-s * self.x + c * self.y);
        [x, y, wrap(std::f64::consts::PI - self.yaw)]
    }

    /// The dock whose [`Self::robot_in_dock`] is `r` (x out along the axis, y, yaw 0 = facing the posts).
    pub fn from_robot_in_dock(r: [f64; 3]) -> DockPose {
        let yaw = wrap(std::f64::consts::PI - r[2]);
        let (s, c) = yaw.sin_cos();
        DockPose {
            x: -(c * r[0] - s * r[1]),
            y: -(s * r[0] + c * r[1]),
            yaw,
        }
    }
}

/// What the batteries say about charging, from the newest battery report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChargeReading {
    /// counts reports, so a reading is known to be newer than another
    pub seq: u64,
    /// packs reporting a level (charge_manager counts battery_level != 0)
    pub packs_present: u8,
    /// of those, packs whose charge MOS is on (m20-battery `charge_mos_on`)
    pub packs_charge_mos_on: u8,
    /// any pack sees the dock's charger on its contacts (m20-battery `charger_connected`)
    pub charger_connected: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn errors_are_zero_at_the_goal_and_signed_toward_it() {
        let facing = DockPose {
            x: 1.6,
            y: 0.0,
            yaw: PI,
        };
        let e = facing.error_to(0.6);
        assert!((e[0] - 1.0).abs() < 1e-12 && e[1].abs() < 1e-12 && e[2].abs() < 1e-12);
        // the dock 0.1 m to the robot's left: step left
        let left = DockPose {
            x: 0.6,
            y: 0.1,
            yaw: PI,
        };
        assert!((left.error_to(0.6)[1] - 0.1).abs() < 1e-12);
        // robot turned 10 deg left of facing the posts: the dock's axis appears turned right, turn right
        let turned = DockPose {
            x: 1.0,
            y: 0.0,
            yaw: PI - 0.1745,
        };
        assert!((turned.error_to(1.0)[2] + 0.1745).abs() < 1e-9);
        let r = turned.robot_in_dock();
        assert!((r[2] - 0.1745).abs() < 1e-9, "{r:?}");
    }

    #[test]
    fn robot_in_dock_frame() {
        // the robot 1 m out and 0.2 m to the dock's left (seen from the robot: the dock is right of it), facing it
        let d = DockPose {
            x: 1.0,
            y: 0.2,
            yaw: PI,
        };
        let r = d.robot_in_dock();
        assert!((r[0] - 1.0).abs() < 1e-12, "{r:?}");
        assert!(r[2].abs() < 1e-12, "{r:?}");
        assert!((r[1] - 0.2).abs() < 1e-12, "{r:?}");
    }

    #[test]
    fn from_robot_in_dock_inverts_robot_in_dock() {
        for r in [[0.76, 0.0, 0.0], [1.3, -0.2, 0.3], [2.0, 0.4, -0.5]] {
            let back = DockPose::from_robot_in_dock(r).robot_in_dock();
            for k in 0..3 {
                assert!((back[k] - r[k]).abs() < 1e-12, "{back:?} vs {r:?}");
            }
        }
    }
}
