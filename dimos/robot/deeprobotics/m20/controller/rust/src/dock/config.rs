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

/// The dock / undock sequence (charge_manager v4.0.30's, NOS charge_manager.toml).
#[derive(Debug, Clone, Deserialize)]
pub struct SequenceConfig {
    /// gaits: approach and exit on NaviCharge, in-place adjustment on FinetuneStand, BasicFlat after undocking
    pub approach_gait: u32,
    pub finetune_gait: u32,
    pub exit_gait: u32,
    /// the staging point the approach goes to first: this far (m) out from the docked point along the dock's axis
    pub staging_x_m: f64,
    /// base_link's distance (m) out along the dock's axis when docked
    pub docked_x_m: f64,
    /// docked this much (m) further forward than docked_x_m
    pub distance_compensation_x_m: f64,
    /// undock backs this far (m) out from docked_x_m
    pub exit_distance_m: f64,
    /// at the fine point (m, m, deg); at the staging point and the undock's end point (m, m, deg)
    pub tolerance: [f64; 3],
    pub staging_tolerance: [f64; 3],
    pub exit_tolerance: [f64; 3],
    /// normalized steer (x, y, yaw): the least the approach and the exit send while off the goal, and the most
    pub min_axes: [f64; 3],
    pub exit_min_axes: [f64; 3],
    pub max_axes: [f64; 3],
    /// steer = gain x error (1/s), in physical units before normalizing by the gait's command_max
    pub gain: [f64; 3],
    /// a goal counts as reached when this many sweeps, taken standing still, agree (their mean is within tolerance)
    pub settle_sweeps: u32,
    /// no dock pose for this long (s, counted in control ticks) while driving: stop (and lie down when docking)
    pub dock_lost_s: f64,
    /// the approach (staging + fine point) gives up after this long (s, in ticks)
    pub approach_limit_s: f64,
    /// after the switch to the approach gait's runner, stand this long (s, in ticks) before steering
    pub gait_settle_s: f64,
    /// a gait switch or a stand-up not done after this long (s, in ticks) fails the sequence
    pub transition_limit_s: f64,
    /// FinetuneStand adjusts for at most this long (s, in ticks), then lies down regardless
    pub finetune_limit_s: f64,
    /// FinetuneStand shifts the body onto the remaining error (charge_manager's in-place adjustment); off, it only
    /// stands on the finetune gait for gait_settle_s before lying down
    pub finetune_adjust: bool,
    /// FinetuneStand: base_link moves this far (m, m, rad) per unit of its pose inputs dx, dy, yaw
    pub finetune_body_per_param: [f64; 3],
    /// after lying down (legs passive), wait this long (s, in ticks) for every pack's charge MOS
    pub charge_confirm_s: f64,
    /// with no charge: stand, back to the staging point and try again this many more times
    pub retries: u32,
    /// every present pack must report its charge MOS on (else one is enough)
    pub all_packs_charge: bool,
    /// the dock farther than this (m) or not in sight when docking starts: refuse
    pub max_start_distance_m: f64,
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
            ("sequence.staging_x_m", s.staging_x_m),
            ("sequence.docked_x_m", s.docked_x_m),
            ("sequence.exit_distance_m", s.exit_distance_m),
            ("sequence.dock_lost_s", s.dock_lost_s),
            ("sequence.approach_limit_s", s.approach_limit_s),
            ("sequence.transition_limit_s", s.transition_limit_s),
            ("sequence.finetune_limit_s", s.finetune_limit_s),
            ("sequence.charge_confirm_s", s.charge_confirm_s),
            ("sequence.max_start_distance_m", s.max_start_distance_m),
        ] {
            positive(name, v)?;
        }
        if !(s.gait_settle_s.is_finite() && s.gait_settle_s >= 0.0) {
            bail!("dock.sequence.gait_settle_s must be a non-negative number of seconds");
        }
        for (name, v) in [
            ("tolerance", s.tolerance),
            ("staging_tolerance", s.staging_tolerance),
            ("exit_tolerance", s.exit_tolerance),
            ("gain", s.gain),
            ("finetune_body_per_param", s.finetune_body_per_param),
        ] {
            if v.iter().any(|x| !(x.is_finite() && *x != 0.0))
                || (name != "finetune_body_per_param" && v.iter().any(|x| *x < 0.0))
            {
                bail!("dock.sequence.{name}: three finite non-zero entries (positive except finetune_body_per_param)");
            }
        }
        for (name, v) in [
            ("min_axes", s.min_axes),
            ("exit_min_axes", s.exit_min_axes),
            ("max_axes", s.max_axes),
        ] {
            if v.iter()
                .any(|x| !(x.is_finite() && (0.0..=1.0).contains(x)))
            {
                bail!("dock.sequence.{name}: entries in [0, 1]");
            }
        }
        if s.min_axes.iter().zip(&s.max_axes).any(|(lo, hi)| lo > hi)
            || s.exit_min_axes
                .iter()
                .zip(&s.max_axes)
                .any(|(lo, hi)| lo > hi)
        {
            bail!("dock.sequence: min_axes / exit_min_axes above max_axes");
        }
        if s.settle_sweeps == 0 {
            bail!("dock.sequence.settle_sweeps must be at least 1");
        }
        Ok(())
    }
}
