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

//! The last full route and what the map did to it since: which of its cells and points a region update, a
//! migration or a closure reached, so a replan can be skipped while nothing did and can reuse the part past it.

use super::{Config, Planner};
use crate::adjacency::CellId;
use crate::lazy::Version;
use crate::planner::{RouteCache, RouteChanges};
use crate::voxel::Xyz;

/// The cached route, indexed by cell id and by the column each smoothed point is in, and how far along it a change reached.
pub(crate) struct RouteWatch {
    pub(crate) cache: RouteCache,
    /// The column each smoothed point is in, with its index, sorted.
    point_at: Vec<((i32, i32), usize)>,
    pub(crate) changes: RouteChanges,
    /// Whether any change reached the route since it was planned.
    touched: bool,
}

impl RouteWatch {
    pub(crate) fn new(cache: RouteCache, voxel_size: f32) -> Self {
        let mut point_at: Vec<((i32, i32), usize)> = cache
            .points
            .iter()
            .enumerate()
            .map(|(i, &p)| (column_of(p, voxel_size), i))
            .collect();
        point_at.sort_unstable();
        Self {
            changes: RouteChanges::none(cache.assembled.len(), cache.points.len()),
            cache,
            point_at,
            touched: false,
        }
    }

    /// Cells whose surface, links, wall distance or layer changed, or that died.
    pub(crate) fn note_cells(&mut self, ids: impl IntoIterator<Item = CellId>) {
        for id in ids {
            if let Some(&i) = self.cache.cell_at.get(&id) {
                self.changes.cells[i] = true;
                self.touched = true;
            }
        }
    }

    /// Current-frame columns a change reached.
    pub(crate) fn note_columns(&mut self, columns: impl IntoIterator<Item = (i32, i32)>) {
        for col in columns {
            let from = self.point_at.partition_point(|&(c, _)| c < col);
            for &(_, i) in self.point_at[from..].iter().take_while(|&&(c, _)| c == col) {
                self.changes.points[i] = true;
                self.touched = true;
            }
        }
    }

    /// A closure moved the keyframes, so every stale-layer cell on the route may stand elsewhere now.
    pub(crate) fn note_closure(&mut self, current_layer: Version) {
        for i in self.cache.stale_cells(current_layer) {
            self.changes.cells[i] = true;
            self.touched = true;
        }
    }

    /// Whether any change reached the route since it was planned.
    pub(crate) fn touched(&self) -> bool {
        self.touched
    }

    /// Ground-plane distance from `p` to the route.
    pub(crate) fn distance_to(&self, p: Xyz) -> f32 {
        let pts = &self.cache.points;
        if pts.len() < 2 {
            return pts
                .first()
                .map_or(f32::INFINITY, |q| (p.0 - q.0).hypot(p.1 - q.1));
        }
        pts.windows(2)
            .map(|w| {
                let (a, b) = (w[0], w[1]);
                let (abx, aby) = (b.0 - a.0, b.1 - a.1);
                let len2 = abx * abx + aby * aby;
                let t = if len2 == 0.0 {
                    0.0
                } else {
                    (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / len2).clamp(0.0, 1.0)
                };
                (p.0 - a.0 - t * abx).hypot(p.1 - a.1 - t * aby)
            })
            .fold(f32::INFINITY, f32::min)
    }
}

fn column_of(p: Xyz, voxel_size: f32) -> (i32, i32) {
    (
        (p.0 / voxel_size).floor() as i32,
        (p.1 / voxel_size).floor() as i32,
    )
}

/// Why a pass has to plan again rather than keep the route it published.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplanReason {
    /// No full route is held: none planned yet, or the last plan found none.
    NoRoute,
    /// The goal is not the one the route leads to.
    GoalMoved,
    /// A region update, migration or closure changed cells on the route or within the repair window of it.
    RouteChanged,
    /// The robot is further from the route than the goal tolerance plus a node spacing.
    OffRoute,
}

impl Planner {
    /// Why the robot at `robot` heading to `goal` needs a new plan, or None to keep the route it has.
    pub fn replan_reason(&self, robot: Xyz, goal: Xyz, config: &Config) -> Option<ReplanReason> {
        let Some(route) = &self.route else {
            return Some(ReplanReason::NoRoute);
        };
        if route.cache.goal_pose != goal {
            return Some(ReplanReason::GoalMoved);
        }
        if route.touched() {
            return Some(ReplanReason::RouteChanged);
        }
        if route.distance_to(robot) > config.goal_tolerance + config.node_spacing_m {
            return Some(ReplanReason::OffRoute);
        }
        None
    }

    /// Whether the robot at `robot` heading to `goal` needs a new plan; false keeps the route it has.
    pub fn replan_needed(&self, robot: Xyz, goal: Xyz, config: &Config) -> bool {
        self.replan_reason(robot, goal, config).is_some()
    }
}
