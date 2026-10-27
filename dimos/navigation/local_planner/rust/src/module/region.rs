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

//! A local_map covers only the region its scans observed. Inside it the new map
//! replaces everything. Outside it the points last seen stand, out to the planning reach.

use std::collections::HashSet;
use std::sync::Arc;

use lcm_msgs::geometry_msgs::PoseStamped;

/// Snap for matching a kept point to a new one. Map points are voxel centers.
const KEY_M: f64 = 0.01;

/// The cylinder a local_map covers, as region_bounds carries it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegionBounds {
    pub cx: f64,
    pub cy: f64,
    pub radius: f64,
    pub z_min: f64,
    pub z_max: f64,
}

impl RegionBounds {
    /// Position is the center, orientation holds radius, z_min, z_max.
    pub fn of_msg(msg: &PoseStamped) -> Self {
        RegionBounds {
            cx: msg.pose.position.x,
            cy: msg.pose.position.y,
            radius: msg.pose.orientation.x,
            z_min: msg.pose.orientation.y,
            z_max: msg.pose.orientation.z,
        }
    }

    fn covers(&self, p: &[f32; 3], inset: f64) -> bool {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        (x - self.cx).hypot(y - self.cy) <= self.radius - inset
            && (self.z_min + inset..=self.z_max - inset).contains(&z)
    }
}

fn key(p: &[f32; 3]) -> (i64, i64, i64) {
    let k = |v: f32| (v as f64 / KEY_M).round() as i64;
    (k(p[0]), k(p[1]), k(p[2]))
}

/// `fresh` plus the `kept` points outside `bounds` within `reach` of the robot.
pub fn merge(
    kept: &[[f32; 3]],
    fresh: Vec<[f32; 3]>,
    bounds: &RegionBounds,
    robot: (f64, f64),
    reach: f64,
) -> Vec<[f32; 3]> {
    // Only a fresh point near the edge can share a key with a kept point outside.
    let seen: HashSet<_> = fresh
        .iter()
        .filter(|p| !bounds.covers(p, 2.0 * KEY_M))
        .map(key)
        .collect();
    let mut out = fresh;
    out.extend(kept.iter().filter(|p| {
        !bounds.covers(p, 0.0)
            && (p[0] as f64 - robot.0).hypot(p[1] as f64 - robot.1) <= reach
            && !seen.contains(&key(p))
    }));
    out
}

/// The points planned on for each local_map, and the merged set the next one builds on.
#[derive(Default)]
pub struct Memory {
    latest: Option<(u64, Arc<Vec<[f32; 3]>>)>,
    kept: Arc<Vec<[f32; 3]>>,
}

impl Memory {
    /// The points already planned on for local_map `seq`.
    pub fn cached(&self, seq: u64) -> Option<Arc<Vec<[f32; 3]>>> {
        self.latest
            .as_ref()
            .filter(|(s, _)| *s == seq)
            .map(|(_, pts)| Arc::clone(pts))
    }

    /// The points to plan on for local_map `seq`. A cloud without bounds stands alone and
    /// leaves the memory as it was.
    pub fn update(
        &mut self,
        seq: u64,
        fresh: Vec<[f32; 3]>,
        bounds: Option<&RegionBounds>,
        robot: (f64, f64),
        reach: f64,
    ) -> Arc<Vec<[f32; 3]>> {
        let pts = match bounds {
            Some(b) => {
                self.kept = Arc::new(merge(&self.kept, fresh, b, robot, reach));
                Arc::clone(&self.kept)
            }
            None => Arc::new(fresh),
        };
        self.latest = Some((seq, Arc::clone(&pts)));
        pts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds(radius: f64) -> RegionBounds {
        RegionBounds {
            cx: 0.0,
            cy: 0.0,
            radius,
            z_min: -1.0,
            z_max: 2.0,
        }
    }

    #[test]
    fn a_wall_past_shrunk_bounds_stays() {
        let wall = vec![[3.7, 0.0, 0.5], [3.7, 0.1, 0.5]];
        let near = vec![[1.0, 0.0, 0.5]];
        let first = merge(
            &[],
            [near.clone(), wall.clone()].concat(),
            &bounds(4.0),
            (0.0, 0.0),
            6.0,
        );
        let second = merge(&first, near.clone(), &bounds(3.4), (0.0, 0.0), 6.0);
        assert_eq!(second.len(), 3);
        assert!(wall.iter().all(|w| second.contains(w)));
    }

    #[test]
    fn inside_the_bounds_the_new_map_wins() {
        let gone = [[2.0, 0.0, 0.5]];
        let merged = merge(&gone, vec![[1.0, 0.0, 0.5]], &bounds(4.0), (0.0, 0.0), 6.0);
        assert_eq!(merged, vec![[1.0, 0.0, 0.5]]);
    }

    #[test]
    fn points_past_the_reach_are_dropped() {
        let far = [[5.0, 0.0, 0.5]];
        assert!(merge(&far, vec![], &bounds(3.0), (0.0, 0.0), 4.5).is_empty());
    }

    #[test]
    fn a_point_above_the_bounds_is_kept() {
        let high = [[1.0, 0.0, 2.5]];
        assert_eq!(
            merge(&high, vec![], &bounds(3.0), (0.0, 0.0), 4.5),
            high.to_vec()
        );
    }

    #[test]
    fn a_point_the_new_map_repeats_is_not_doubled() {
        let p = [3.6, 0.0, 0.5];
        let mut merged = vec![p];
        for _ in 0..5 {
            merged = merge(&merged, vec![p], &bounds(3.0), (0.0, 0.0), 6.0);
        }
        assert_eq!(merged, vec![p]);
    }

    #[test]
    fn a_repeat_across_the_edge_is_not_doubled() {
        let inside = [2.996, 0.0, 0.5];
        let merged = merge(
            &[[3.004, 0.0, 0.5]],
            vec![inside],
            &bounds(3.0),
            (0.0, 0.0),
            6.0,
        );
        assert_eq!(merged, vec![inside]);
    }

    #[test]
    fn a_cloud_without_bounds_keeps_the_memory() {
        let (wall, near) = ([-3.4f32, 1.0, 0.3], [0.5f32, 0.0, 0.3]);
        let mut memory = Memory::default();
        memory.update(1, vec![near, wall], Some(&bounds(3.6)), (0.0, 0.0), 5.7);
        memory.update(2, vec![near], Some(&bounds(3.0)), (0.0, 0.0), 5.7);
        assert_eq!(
            *memory.update(3, vec![near], None, (0.0, 0.0), 5.7),
            vec![near]
        );
        let pts = memory.update(4, vec![near], Some(&bounds(3.0)), (0.0, 0.0), 5.7);
        assert!(pts.contains(&wall));
        assert_eq!(memory.cached(4), Some(pts));
    }
}
