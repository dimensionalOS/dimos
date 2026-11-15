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

//! When the module's worker plans and what a plan pass decides, in one place so an offline driver can run the
//! planner exactly as the module does: hand it the same events in the same order.

use crate::mls_planner::{ClosureOutcome, Config, Plan, Planner};
use crate::planner::snap_pose_to_cell;
use crate::voxel::{surface_point_xyz, Xyz};

/// What a worker pass handled before it may plan, in the order the worker handles them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PassTrigger {
    /// A loop closure Planner::on_closure applied; `new_layer` from its outcome.
    Closure { new_layer: bool },
    /// A goal change and a live map update (region or global map), with whether the update was applied.
    Update {
        goal_changed: bool,
        live_update_applied: bool,
    },
}

/// Whether a worker pass goes on to a plan pass. A seed region never does, nor a closure that kept its layer.
pub fn pass_replans(trigger: PassTrigger) -> bool {
    match trigger {
        PassTrigger::Closure { new_layer } => new_layer,
        PassTrigger::Update {
            goal_changed,
            live_update_applied,
        } => goal_changed || live_update_applied,
    }
}

/// Where a world-frame goal is after a closure: carried by the newest keyframe's move when it started a layer.
pub fn carry_goal(goal: Xyz, outcome: &ClosureOutcome) -> Xyz {
    if !outcome.new_layer {
        return goal;
    }
    let p = outcome
        .carry_newest
        .apply([goal.0 as f64, goal.1 as f64, goal.2 as f64]);
    (p[0] as f32, p[1] as f32, p[2] as f32)
}

/// What a plan pass decided for the active goal.
#[derive(Debug, PartialEq)]
pub enum Replan {
    /// Within tolerance of the goal: no path needed.
    Reached,
    /// Nothing reached the published route and the robot is on it: keep following it.
    Kept,
    /// Planned again.
    Planned(Plan),
}

/// Decide a plan pass from `start` to `goal`. A goal already within tolerance needs no path; a goal that did not
/// change keeps the route while Planner::replan_needed says so; otherwise plan (reusing the route past the robot).
pub fn replan(
    planner: &mut Planner,
    start: Xyz,
    goal: Xyz,
    goal_changed: bool,
    config: &Config,
) -> Replan {
    if is_at_goal(planner, start, goal, config) {
        return Replan::Reached;
    }
    if !goal_changed && !planner.replan_needed(start, goal, config) {
        return Replan::Kept;
    }
    Replan::Planned(planner.plan_or_truncate(start, goal, config))
}

/// True if within tolerance of the goal on the ground plane and vertically. The goal's height is that of the
/// surface it plans to, or its own z when it snaps to none.
pub fn is_at_goal(planner: &Planner, start: Xyz, goal: Xyz, config: &Config) -> bool {
    if (start.0 - goal.0).hypot(start.1 - goal.1) >= config.goal_tolerance {
        return false;
    }
    let goal_z = snap_pose_to_cell(
        &planner.graph().surface_lookup,
        goal,
        config.voxel_size,
        config.robot_height,
    )
    .map_or(goal.2, |(ix, iy, iz)| {
        surface_point_xyz(ix, iy, iz, config.voxel_size).2
    });
    (start.2 - goal_z).abs() < config.goal_z_tolerance
}
