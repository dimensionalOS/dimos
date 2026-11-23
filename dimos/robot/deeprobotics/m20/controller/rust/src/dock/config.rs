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

//! `[dock]` in m20_controller.toml: the detector's and the dock / undock sequence's tunables.

use anyhow::{bail, Result};
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct DockConfig {
    pub detector: DetectorConfig,
    pub filter: FilterConfig,
    pub obstacle: ObstacleConfig,
    pub sequence: SequenceConfig,
}

/// The reflector-post detector (reflective_column_node's numbers are compiled in; these are the sim dock's).
#[derive(Debug, Clone, Deserialize)]
pub struct DetectorConfig {
    /// a point at or above this intensity (the Airy's 0-255 reflectivity) is a reflector return
    pub min_intensity: f64,
    /// kept: points this far (m, floor plane) from the lidar at most, and at least
    pub max_range_m: f64,
    pub min_range_m: f64,
    /// kept: base_link heights (m) between these
    pub min_z_m: f64,
    pub max_z_m: f64,
    /// kept: within this angle (deg) of straight ahead of base_link
    pub half_angle_deg: f64,
    /// more reflector returns than this in one sweep: refuse the sweep (bounds the clustering)
    pub max_points: usize,
    /// reflector returns closer than this (m, floor plane) are one post
    pub cluster_distance_m: f64,
    /// a cluster needs this many returns to be a post
    pub min_cluster_points: usize,
    /// a cluster wider than this (m) is not a post (a reflective wall, a sign)
    pub max_cluster_width_m: f64,
    /// a post's returns farther than this (m) behind its nearest are ghosts along the ray, dropped
    pub max_post_depth_m: f64,
    /// the posts' centre-to-centre distance (m) and how far a pair may be off it
    pub post_spacing_m: f64,
    pub spacing_tolerance_m: f64,
    /// each post's radius (m): a return lies on the near surface, the centre is this far further along the ray
    pub post_radius_m: f64,
}

/// reflective_column_node's output stage (compiled into it; the binary's numbers).
#[derive(Debug, Clone, Deserialize)]
pub struct FilterConfig {
    /// its lidar position (x m, negative = in front of the posts) kept only in this range, and |y| (m) at most this
    pub x_range: [f64; 2],
    pub max_y: f64,
    /// the published pose's x is the lidar position's plus this (m)
    pub pose_offset_x: f64,
    /// after a (re)start: sweeps held back, and the sigmas the start fix keeps samples within
    pub start_sweeps: usize,
    pub start_sigmas: f64,
    /// a sweep this far (m) from the sweep before is dropped
    pub max_jump_m: f64,
}

/// charge_manager's obstacle check: the numbers that are not in charge_manager.toml.
#[derive(Debug, Clone, Deserialize)]
pub struct ObstacleConfig {
    /// the lidar driver drops returns closer than this (m) to the lidar
    pub min_lidar_range_m: f64,
    /// the body height (MotionInfo.height) estimate: base_link above the wheel axles plus this (m)
    pub wheel_radius_m: f64,
    /// ours: the body height goes to the detector module when it has moved this far (m) since the last
    pub body_height_step_m: f64,
}

/// The dock / undock sequence's numbers that charge_manager compiles in (charge_manager.toml has the rest, read at
/// startup; see super::vendor).
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct SequenceConfig {
    /// gaits: NaviCharge (approach, undock), FinetuneStand, BasicFlat (NarrowWalk) and the gait restored after an
    /// undock when no dock this run recorded one
    pub approach_gait: u32,
    pub finetune_gait: u32,
    pub basic_flat_gait: u32,
    pub default_restore_gait: u32,
    /// base_link's distance (m) out along the dock's axis when docked (our detector's frame to charge_manager's)
    pub docked_x_m: f64,
    /// VelocityController (kp, ki, kd) for x, y, yaw
    pub pid_x: [f32; 3],
    pub pid_y: [f32; 3],
    pub pid_yaw: [f32; 3],
    /// the start point is chosen once this many poses with x > 0 are taken (the next one)
    pub start_samples: u32,
    /// staging point: reached inside delta + margin (m, m, rad); steer maxima; x / y minima (yaw: min_speed_yaw)
    pub staging_margin: [f64; 3],
    pub staging_max: [f32; 3],
    pub staging_min: [f32; 2],
    /// fine point: reached inside delta + margin; steer maxima (minima: min_speed_*)
    pub fine_margin: [f64; 3],
    pub fine_max: [f32; 3],
    /// undock: steer maxima (minima: ext_min_speed_*); the exit staging point counts as passed this far before it
    pub exit_max: [f32; 3],
    pub exit_staging_margin: f64,
    /// FinetuneStand: z = k x step, k from 2 up to z_last; done inside delta after min_ms, or after max_ms; then z =
    /// sit_z held sit_hold_ms before lying down
    pub finetune_z_step: f64,
    pub finetune_z_last: u32,
    pub finetune_min_ms: u64,
    pub finetune_max_ms: u64,
    pub sit_z: f32,
    pub sit_hold_ms: u64,
    /// setFineTuneStandMode waits this long (ms) for FinetuneStand, then goes on regardless
    pub finetune_gait_wait_ms: u64,
    /// rl_deploy's /STEER on FinetuneStand: dx, dy, yaw pose inputs = these x steer x, y, yaw; height = max(-z, 0)
    pub finetune_pose_scale: [f64; 3],
    /// the steer x written (five times) before an undock's loop and a retry's approach
    pub back_off_x: f32,
    /// limits (ms): the approach (from the dock's start, again from FinetuneStand), the undock, an obstacle stop
    pub approach_limit_ms: u64,
    pub exit_limit_ms: u64,
    pub obstacle_limit_ms: u64,
    /// a wait for the next pose (ms) before it counts as missed (and the robot stops)
    pub pose_wait_ms: u64,
    /// rl_deploy zeroes the steer this long (ms) after the last one
    pub steer_timeout_ms: u64,
    /// held still (ms): after the switch to NaviCharge (WideWalk), after a retry's stand-up, before the undock's gait
    /// is restored
    pub gait_settle_ms: u64,
    pub retry_settle_ms: u64,
    pub restore_ms: u64,
    /// lying, the power-on (three charge-enable requests 2 s apart) before the charge is checked (ms)
    pub power_on_ms: u64,
    /// ours: a stand-up / gait switch / lie-down not done in this long (s) fails the sequence
    pub transition_limit_s: f64,
}

fn positive(name: &str, v: f64) -> Result<()> {
    if !(v.is_finite() && v > 0.0) {
        bail!("dock.{name} must be a positive number, is {v}");
    }
    Ok(())
}

impl DockConfig {
    pub fn validate(&self) -> Result<()> {
        let d = &self.detector;
        for (name, v) in [
            ("detector.max_range_m", d.max_range_m),
            ("detector.cluster_distance_m", d.cluster_distance_m),
            ("detector.max_cluster_width_m", d.max_cluster_width_m),
            ("detector.max_post_depth_m", d.max_post_depth_m),
            ("detector.post_spacing_m", d.post_spacing_m),
            ("detector.spacing_tolerance_m", d.spacing_tolerance_m),
            ("detector.half_angle_deg", d.half_angle_deg),
        ] {
            positive(name, v)?;
        }
        if !(d.min_range_m.is_finite() && d.min_range_m >= 0.0 && d.min_range_m < d.max_range_m) {
            bail!("dock.detector.min_range_m must be in [0, max_range_m)");
        }
        if d.min_z_m.partial_cmp(&d.max_z_m) != Some(std::cmp::Ordering::Less)
            || !d.post_radius_m.is_finite()
            || !d.min_intensity.is_finite()
        {
            bail!("dock.detector: min_z_m < max_z_m, finite post_radius_m and min_intensity");
        }
        if d.min_cluster_points == 0 || d.max_points == 0 {
            bail!("dock.detector: min_cluster_points and max_points must be at least 1");
        }
        let s = &self.sequence;
        for (name, v) in [
            ("sequence.docked_x_m", s.docked_x_m),
            ("sequence.transition_limit_s", s.transition_limit_s),
            ("filter.max_y", self.filter.max_y),
            ("filter.max_jump_m", self.filter.max_jump_m),
            ("filter.start_sigmas", self.filter.start_sigmas),
            ("obstacle.wheel_radius_m", self.obstacle.wheel_radius_m),
            (
                "obstacle.body_height_step_m",
                self.obstacle.body_height_step_m,
            ),
        ] {
            positive(name, v)?;
        }
        if self.filter.x_range[0] >= self.filter.x_range[1] || self.filter.start_sweeps == 0 {
            bail!("dock.filter: x_range ascending and start_sweeps at least 1");
        }
        if s.pose_wait_ms == 0 || s.finetune_z_last < 3 || s.finetune_min_ms > s.finetune_max_ms {
            bail!("dock.sequence: pose_wait_ms > 0, finetune_z_last >= 3, finetune_min_ms <= finetune_max_ms");
        }
        let finite = s
            .pid_x
            .iter()
            .chain(&s.pid_y)
            .chain(&s.pid_yaw)
            .chain(&s.staging_max)
            .chain(&s.staging_min)
            .chain(&s.fine_max)
            .chain(&s.exit_max)
            .all(|v| v.is_finite());
        if !finite
            || s.staging_max
                .iter()
                .chain(&s.fine_max)
                .chain(&s.exit_max)
                .any(|v| *v <= 0.0)
        {
            bail!("dock.sequence: finite gains, positive steer maxima");
        }
        Ok(())
    }
}
