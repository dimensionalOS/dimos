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

//! When a planned route has to be planned again: what a region update did near it, and where the robot is.

use super::region_tests::{bare, slice};
use super::*;

/// A 12 m by 6 m room with a 3 m wide hall up from its far end to y 12 m.
fn floor(vs: f32) -> Vec<(f32, f32, f32)> {
    let half = 0.5 * vs;
    (0..120)
        .flat_map(|ix| (0..120).map(move |iy| (ix, iy)))
        .filter(|&(ix, iy)| iy < 60 || ix >= 90)
        .map(|(ix, iy)| (ix as f32 * vs + half, iy as f32 * vs + half, half))
        .collect()
}

const START: Xyz = (1.0, 1.0, 0.1);
const GOAL: Xyz = (11.0, 11.0, 0.1);

/// A planner on the floor with a full route from START to GOAL, around the hall's corner.
fn planned(cfg: &Config) -> (Planner, Vec<Xyz>) {
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&bare(&floor(cfg.voxel_size)), cfg);
    let Plan::Full(waypoints) = p.plan_or_truncate(START, GOAL, cfg) else {
        panic!("a route up the hall");
    };
    (p, waypoints)
}

/// The route's point about `along` m from its start.
fn point_along(waypoints: &[Xyz], along: f32) -> (f32, f32) {
    let mut walked = 0.0;
    for w in waypoints.windows(2) {
        walked += (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1);
        if walked >= along {
            return (w[1].0, w[1].1);
        }
    }
    panic!("the route is shorter than {along} m");
}

/// A region update around `at` that sees the floor plus a junk voxel there, a hand's width up.
fn junk_at(p: &mut Planner, at: (f32, f32), cfg: &Config) {
    let bounds = RegionBounds {
        origin_x: at.0,
        origin_y: at.1,
        radius: 1.0,
        z_min: -1.0,
        z_max: 2.0,
    };
    let mut pts = slice(&floor(cfg.voxel_size), &bounds, cfg.voxel_size);
    pts.push((at.0, at.1, 0.45));
    assert!(p.update_region(&bare(&pts), &bounds, cfg).is_some());
}

#[test]
fn an_unchanged_map_needs_no_replan() {
    let cfg = test_config();
    let (mut p, waypoints) = planned(&cfg);
    let (x, y) = point_along(&waypoints, 3.0);
    let along = (x, y, 0.1);
    assert_eq!(p.replan_reason(START, GOAL, &cfg), None);
    assert_eq!(p.replan_reason(along, GOAL, &cfg), None);
    // a region update that sees what the map already holds changes nothing
    let bounds = RegionBounds {
        origin_x: 6.0,
        origin_y: 1.0,
        radius: 1.0,
        z_min: -1.0,
        z_max: 2.0,
    };
    let pts = slice(&floor(cfg.voxel_size), &bounds, cfg.voxel_size);
    p.update_region(&bare(&pts), &bounds, &cfg);
    assert!(!p.replan_needed(along, GOAL, &cfg));
}

#[test]
fn an_edit_off_the_route_needs_no_replan_and_one_on_it_does() {
    let cfg = test_config();
    let (mut p, waypoints) = planned(&cfg);
    junk_at(&mut p, (11.0, 1.0), &cfg);
    assert_eq!(p.replan_reason(START, GOAL, &cfg), None);
    let on = point_along(&waypoints, 6.0);
    junk_at(&mut p, on, &cfg);
    assert_eq!(
        p.replan_reason(START, GOAL, &cfg),
        Some(ReplanReason::RouteChanged)
    );
    // planning again takes the change in, and the new route is clean
    let Plan::Full(waypoints) = p.plan_or_truncate(START, GOAL, &cfg) else {
        panic!("a route around the junk");
    };
    assert!(waypoints
        .iter()
        .all(|w| (w.0 - on.0).hypot(w.1 - on.1) > cfg.voxel_size));
    assert_eq!(p.replan_reason(START, GOAL, &cfg), None);
}

#[test]
fn a_robot_off_the_route_or_a_new_goal_needs_a_replan() {
    let cfg = test_config();
    let mut p = Planner::new(cfg.worker_threads);
    assert_eq!(
        p.replan_reason(START, GOAL, &cfg),
        Some(ReplanReason::NoRoute)
    );
    p = planned(&cfg).0;
    let off = cfg.goal_tolerance + cfg.node_spacing_m;
    let beside = (START.0, START.1 - 0.5 * off, 0.1);
    assert_eq!(p.replan_reason(beside, GOAL, &cfg), None);
    let away = (START.0, START.1 - 1.5 * off, 0.1);
    assert_eq!(
        p.replan_reason(away, GOAL, &cfg),
        Some(ReplanReason::OffRoute)
    );
    assert_eq!(
        p.replan_reason(START, (11.0, 4.0, 0.1), &cfg),
        Some(ReplanReason::GoalMoved)
    );
    // no full route found drops the route
    assert_eq!(
        p.plan_or_truncate(START, (30.0, 1.0, 0.1), &cfg),
        Plan::Blocked
    );
    assert_eq!(
        p.replan_reason(START, GOAL, &cfg),
        Some(ReplanReason::NoRoute)
    );
}

#[test]
fn a_replan_after_an_edit_matches_a_fresh_plan() {
    let cfg = test_config();
    let (mut p, waypoints) = planned(&cfg);
    let near = point_along(&waypoints, 1.5);
    junk_at(&mut p, near, &cfg);
    let Plan::Full(reused) = p.plan_or_truncate(START, GOAL, &cfg) else {
        panic!("a route around the junk");
    };
    assert!(p.route.is_some(), "a full route is watched");
    let mut fresh = Planner::new(cfg.worker_threads);
    fresh.update_global_map(&bare(&floor(cfg.voxel_size)), &cfg);
    junk_at(&mut fresh, near, &cfg);
    let Plan::Full(whole) = fresh.plan_or_truncate(START, GOAL, &cfg) else {
        panic!("a route around the junk");
    };
    for q in &reused {
        let gap = whole
            .iter()
            .map(|w| (q.0 - w.0).hypot(q.1 - w.1))
            .fold(f32::INFINITY, f32::min);
        assert!(
            gap < cfg.voxel_size,
            "reused point {q:?} is {gap} m off a fresh plan"
        );
    }
}
