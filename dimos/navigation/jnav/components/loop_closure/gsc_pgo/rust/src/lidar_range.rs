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

//! The odometry error estimate scaled by the lidar's range, for a run whose config states none
//! (`odometry_error_estimate_from_lidar_range`). Each scan's range is its farthest point from the scan's frame origin
//! (O(scan)). Every ceil(window / 5) scans the median of the last `window_scans` ranges is blended into the running
//! range estimate, new = median x decay + current x (1 - decay), with decay halving every `decay_half_life_updates`
//! blends (1 at the first: the first median is taken whole), so the estimate settles. The odometry error estimate is
//! then linear in that range between two calibrated lidars: the Go2 L1 (leg odometry) and the Mid-360 (FAST-LIO /
//! Point-LIO), held at the nearer one's outside them.

use std::collections::VecDeque;

/// Per noise source (xy, z, yaw, roll/pitch tilt): [per sqrt m, per sqrt rad], the units of
/// `Config::odometry_error_estimate_*_per_sqrt_{m,rad}`.
pub type OdometryErrorEstimate = [[f64; 2]; 4];

#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "native",
    derive(serde::Serialize, serde::Deserialize),
    serde(default, deny_unknown_fields)
)]
pub struct LidarRangeConfig {
    /// Scans whose ranges the median is taken over; it is taken every ceil(window_scans / 5) scans.
    pub window_scans: usize,
    /// Blends after which the weight a new median gets has halved. Default 2: of 1..128 it left the most of ten
    /// recordings (8; 7 Mid-360, 3 L1) within 25% of their lidar's median range.
    pub decay_half_life_updates: f64,
    /// The Go2 L1's median scan range (m; median over every scan of small_l1, huge_l1 and grassy_field) and its
    /// calibrated odometry error estimate (Go2 leg odometry).
    pub l1_range_m: f64,
    pub l1_odometry_error_estimate: OdometryErrorEstimate,
    /// The Mid-360's median scan range (m; median over every scan of small, gir1, gir2, park1, office555, athens and huge)
    /// and its calibrated odometry error estimate (FAST-LIO / Point-LIO: the `Config` defaults).
    pub mid360_range_m: f64,
    pub mid360_odometry_error_estimate: OdometryErrorEstimate,
}

impl Default for LidarRangeConfig {
    fn default() -> LidarRangeConfig {
        LidarRangeConfig {
            window_scans: 50,
            decay_half_life_updates: 2.0,
            l1_range_m: 4.53,
            l1_odometry_error_estimate: [
                [2.948197514740707e-4, 4.787993406292453e-3],
                [2.3613682139832578e-4, 1.419763443298345e-3],
                [1.1336520957451434e-6, 9.817427149152942e-3],
                [3.837937416144e-4, 1.8328789396925035e-3],
            ],
            mid360_range_m: 27.0,
            mid360_odometry_error_estimate: [
                [3.91e-4, 6.35e-3],
                [6.42e-4, 3.86e-3],
                [1e-6, 8.66e-3],
                [6.91e-4, 3.3e-3],
            ],
        }
    }
}

impl LidarRangeConfig {
    /// An error naming the first setting out of its allowed range, if any.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), String> {
        if self.window_scans == 0 {
            return Err("lidar_range.window_scans must be > 0".into());
        }
        if !(self.decay_half_life_updates > 0.0) {
            return Err(format!(
                "lidar_range.decay_half_life_updates must be > 0 (got {})",
                self.decay_half_life_updates
            ));
        }
        if !(self.l1_range_m > 0.0 && self.mid360_range_m > 0.0) || self.l1_range_m == self.mid360_range_m {
            return Err(format!(
                "lidar_range.l1_range_m and mid360_range_m must be positive and differ (got {} and {})",
                self.l1_range_m, self.mid360_range_m
            ));
        }
        Ok(())
    }

    /// The odometry error estimate at `range_m`: linear between the L1 and the Mid-360, the nearer one's outside them.
    pub fn odometry_error_estimate_at(&self, range_m: f64) -> OdometryErrorEstimate {
        let t = ((range_m - self.l1_range_m) / (self.mid360_range_m - self.l1_range_m)).clamp(0.0, 1.0);
        std::array::from_fn(|source| {
            std::array::from_fn(|unit| {
                self.l1_odometry_error_estimate[source][unit] * (1.0 - t)
                    + self.mid360_odometry_error_estimate[source][unit] * t
            })
        })
    }
}

/// A scan's range: its farthest point from the scan's frame origin (the origin counts as a point, so it is >= 0).
pub fn scan_range(points: &[[f32; 3]]) -> f64 {
    points
        .iter()
        .map(|p| p.iter().map(|&c| c as f64 * c as f64).sum::<f64>())
        .fold(0.0, f64::max)
        .sqrt()
}

/// The running range estimate (`LidarRangeConfig`).
#[derive(Debug, Clone, Default)]
pub struct LidarRange {
    window: VecDeque<f64>,
    scans: usize,
    updates: usize,
    estimate_m: Option<f64>,
}

impl LidarRange {
    /// Add a scan's range; returns the new range estimate when this scan blended one in.
    pub fn add(&mut self, range_m: f64, config: &LidarRangeConfig) -> Option<f64> {
        if self.window.len() == config.window_scans {
            self.window.pop_front();
        }
        self.window.push_back(range_m);
        self.scans += 1;
        if !self.scans.is_multiple_of(config.window_scans.div_ceil(5)) {
            return None;
        }
        let mut sorted: Vec<f64> = self.window.iter().copied().collect();
        let middle = sorted.len() / 2;
        let median = *sorted.select_nth_unstable_by(middle, f64::total_cmp).1;
        let decay = 0.5f64.powf(self.updates as f64 / config.decay_half_life_updates);
        self.updates += 1;
        let estimate = median * decay + self.estimate_m.unwrap_or(median) * (1.0 - decay);
        self.estimate_m = Some(estimate);
        Some(estimate)
    }

    /// The range estimate so far (m), if a median has been blended in.
    pub fn estimate_m(&self) -> Option<f64> {
        self.estimate_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> LidarRangeConfig {
        LidarRangeConfig {
            l1_range_m: 10.0,
            mid360_range_m: 30.0,
            ..LidarRangeConfig::default()
        }
    }

    #[test]
    fn range_counts_the_origin() {
        assert_eq!(scan_range(&[]), 0.0);
        assert_eq!(scan_range(&[[3.0, 4.0, 0.0], [1.0, 0.0, 0.0]]), 5.0);
    }

    #[test]
    fn estimate_is_linear_between_the_lidars_and_held_outside() {
        let c = config();
        assert_eq!(c.odometry_error_estimate_at(10.0), c.l1_odometry_error_estimate);
        assert_eq!(c.odometry_error_estimate_at(30.0), c.mid360_odometry_error_estimate);
        assert_eq!(c.odometry_error_estimate_at(100.0), c.mid360_odometry_error_estimate);
        assert_eq!(c.odometry_error_estimate_at(1.0), c.l1_odometry_error_estimate);
        let half = c.odometry_error_estimate_at(20.0)[0][1];
        let expected = (c.l1_odometry_error_estimate[0][1] + c.mid360_odometry_error_estimate[0][1]) / 2.0;
        assert!((half - expected).abs() < 1e-12);
    }

    #[test]
    fn first_median_is_taken_whole_then_the_estimate_cools() {
        let c = config();
        let mut range = LidarRange::default();
        let updates: Vec<f64> = (0..40).filter_map(|_| range.add(20.0, &c)).collect();
        assert_eq!(updates, vec![20.0; 4]);
        // blending every scan (a window of 5): once the median is the new range, each blend moves the estimate less
        let c = LidarRangeConfig { window_scans: 5, ..c };
        let mut range = LidarRange::default();
        (0..10).for_each(|_| _ = range.add(20.0, &c));
        let moves: Vec<f64> = (0..10).filter_map(|_| range.add(40.0, &c)).skip(2).collect();
        let steps: Vec<f64> = moves.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(steps.windows(2).all(|s| s[1] < s[0]), "{moves:?}");
    }
}
