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

use super::*;

const VS: f32 = 0.08;

/// The Go2's planner settings (go2_nav).
fn go2() -> Config {
    Config {
        world_frame: String::new(),
        base_frame: String::new(),
        voxel_size: VS,
        robot_height: 0.4,
        start_z_offset_m: 0.0,
        max_overhead_m: 2.0,
        surface_closing_radius: 0.4,
        node_spacing_m: 1.0,
        wall_clearance_m: 0.05,
        wall_buffer_m: 0.2,
        wall_buffer_weight: 20.0,
        step_threshold_m: 0.16,
        step_penalty_weight: 4.0,
        approach_when_blocked: true,
        goal_tolerance: 0.3,
        viz_publish_hz: 0.0,
        viz_cell_m: 4.0,
        viz_sweep_cells: 0,
        worker_threads: 2,
    }
}

/// An 8 m by 2 m corridor crossed by a 12 cm sill at x = 4.0, as the robot sees
/// it from the near end: the floor in the sill's shadow, out to `seen_from`,
/// was never observed.
fn sill_corridor(seen_from: f32) -> Vec<(f32, f32, f32)> {
    let half = VS * 0.5;
    let mut pts = Vec::new();
    for ix in 0..100 {
        for iy in 0..25 {
            let (x, y) = (ix as f32 * VS + half, iy as f32 * VS + half);
            if (4.0..4.1).contains(&x) {
                pts.extend([(x, y, half), (x, y, 0.12)]);
            } else if x < 4.0 || x >= seen_from {
                pts.push((x, y, half));
            }
        }
    }
    pts
}

fn planner(points: &[(f32, f32, f32)], cfg: &Config) -> Planner {
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(points, cfg);
    p
}

#[test]
fn a_sill_shadow_splits_the_floor() {
    let cfg = go2();
    let p = planner(&sill_corridor(5.0), &cfg);
    assert!(p.plan((1.0, 1.0, 0.0), (7.0, 1.0, 0.0), &cfg).is_none());
    // seen up close, the shadow is gone and the sill is one step
    let p = planner(&sill_corridor(4.1), &cfg);
    assert!(p.plan((1.0, 1.0, 0.0), (7.0, 1.0, 0.0), &cfg).is_some());
}

#[test]
fn with_no_full_path_the_robot_closes_in_on_the_goal() {
    let cfg = go2();
    let mut p = planner(&sill_corridor(5.0), &cfg);
    let path = p.plan_or_truncate((1.0, 1.0, 0.0), (7.0, 1.0, 0.0), &cfg);
    let end = *path.last().expect("an approach path");
    // it ends at the sill, the reachable surface nearest the goal
    assert!((3.8..=4.1).contains(&end.0), "approach ends at {end:?}");
}

#[test]
fn an_approach_that_gains_nothing_is_a_stop() {
    let cfg = go2();
    let mut p = planner(&sill_corridor(5.0), &cfg);
    // already at the sill: nowhere reachable is meaningfully closer
    let path = p.plan_or_truncate((3.9, 1.0, 0.0), (7.0, 1.0, 0.0), &cfg);
    assert!(path.is_empty(), "{path:?}");
}

#[test]
fn with_the_approach_off_a_blocked_goal_is_a_stop() {
    let cfg = Config {
        approach_when_blocked: false,
        ..go2()
    };
    let mut p = planner(&sill_corridor(5.0), &cfg);
    assert!(p
        .plan_or_truncate((1.0, 1.0, 0.0), (7.0, 1.0, 0.0), &cfg)
        .is_empty());
}

/// The same corridor closed at x = 4.0 by a wall the robot cannot climb, with
/// floor seen beyond it.
fn walled_corridor() -> Vec<(f32, f32, f32)> {
    let half = VS * 0.5;
    let mut pts = Vec::new();
    for ix in 0..100 {
        for iy in 0..25 {
            let (x, y) = (ix as f32 * VS + half, iy as f32 * VS + half);
            pts.push((x, y, half));
            if (4.0..4.1).contains(&x) {
                pts.extend((1..13).map(|iz| (x, y, iz as f32 * VS + half)));
            }
        }
    }
    pts
}

#[test]
fn an_approach_to_an_unreachable_goal_stops_clear_of_what_blocks_it() {
    let cfg = go2();
    let mut p = planner(&walled_corridor(), &cfg);
    let path = p.plan_or_truncate((1.0, 1.0, 0.0), (7.0, 1.0, 0.0), &cfg);
    let end = *path.last().expect("an approach path");
    let standoff = 4.0 - end.0;
    let buffered = cfg.wall_clearance_m + cfg.wall_buffer_m;
    assert!(
        standoff >= buffered,
        "approach ends {standoff} m from the wall"
    );
    assert!(
        standoff <= buffered + 2.0 * VS,
        "approach ends {standoff} m from the wall"
    );
}

/// An 8 m by 6 m walled room with a box too tall to see past, 4 m wide at
/// x = 4.0, and the floor in its shadow unseen. Seen from the start, the box's
/// face is the reachable surface nearest a goal deep in the shadow.
fn room_behind_a_wide_box() -> Vec<(f32, f32, f32)> {
    let half = VS * 0.5;
    let column = |x: f32, y: f32| (0..13).map(move |iz| (x, y, iz as f32 * VS + half));
    let mut pts = Vec::new();
    for ix in 0..100 {
        for iy in 0..75 {
            let (x, y) = (ix as f32 * VS + half, iy as f32 * VS + half);
            let wall = ix == 0 || ix == 99 || iy == 0 || iy == 74;
            let in_box = (4.0..4.4).contains(&x) && (1.0..5.0).contains(&y);
            if wall || in_box {
                pts.extend(column(x, y));
            } else if !(x >= 4.4 && (0.8..5.2).contains(&y)) {
                pts.push((x, y, half));
            }
        }
    }
    pts
}

#[test]
fn an_approach_past_a_box_it_cannot_see_over_goes_round_its_side() {
    let cfg = go2();
    let mut p = planner(&room_behind_a_wide_box(), &cfg);
    let (start, goal) = ((1.0, 3.0, 0.0), (6.0, 3.0, 0.0));
    assert!(p.plan(start, goal, &cfg).is_none());
    let path = p.plan_or_truncate(start, goal, &cfg);
    let end = *path.last().expect("an approach path");
    // beside the box where its shadow shows, not parked at its face
    assert!(
        end.0 > 4.0 && (end.1 - 3.0).abs() > 2.0,
        "approach ends at {end:?}"
    );
}

/// An 8 m by 6 m walled room split at x = 4.0 by a 1 m thick wall with two
/// openings onto the far half, near y = 1.0 and near y = 5.0. Openings not yet
/// `seen` are unseen floor. Seen, the one near y = 1.0 is only a recess in the
/// wall and the one near y = 5.0 goes through.
fn room_with_two_openings(seen: [bool; 2]) -> Vec<(f32, f32, f32)> {
    let half = VS * 0.5;
    let column = |x: f32, y: f32| (0..13).map(move |iz| (x, y, iz as f32 * VS + half));
    let mut pts = Vec::new();
    for ix in 0..100 {
        for iy in 0..75 {
            let (x, y) = (ix as f32 * VS + half, iy as f32 * VS + half);
            let shell = ix == 0 || ix == 99 || iy == 0 || iy == 74;
            let in_wall = (4.0..5.0).contains(&x);
            let opening = [(0.6..1.4).contains(&y), (4.6..5.4).contains(&y)];
            let recess_back = opening[0] && x >= 4.6;
            if shell || in_wall && !(opening[0] || opening[1]) || seen[0] && recess_back {
                pts.extend(column(x, y));
            } else if !in_wall || opening[0] && seen[0] || opening[1] && seen[1] {
                pts.push((x, y, half));
            }
        }
    }
    pts
}

#[test]
fn an_approach_that_finds_a_dead_end_walks_back_to_the_other_frontier() {
    let cfg = go2();
    let goal = (6.5, 2.8, 0.0);
    let mut p = planner(&room_with_two_openings([false, false]), &cfg);
    // the recess is the frontier nearer the goal, so it is tried first
    let first = *p
        .plan_or_truncate((1.0, 3.0, 0.0), goal, &cfg)
        .last()
        .expect("an approach path");
    assert!(first.1 < 2.0, "first approach ends at {first:?}");
    // there it sees the recess is closed, and turns back for the other opening
    p.update_global_map(&room_with_two_openings([true, false]), &cfg);
    let back = *p
        .plan_or_truncate(first, goal, &cfg)
        .last()
        .expect("a path back");
    assert!(back.1 > 4.0, "second approach ends at {back:?}");
    // there it sees the way through, and plans to the goal
    p.update_global_map(&room_with_two_openings([true, true]), &cfg);
    let full = p.plan_or_truncate(back, goal, &cfg);
    assert_eq!(full.last(), Some(&goal));
}

/// An 8 m by 6 m walled room split at x = 4.0 by a wall with one gap a single
/// cell wide, too narrow for the hard clearance, and the floor beyond the wall
/// unseen from x = 5.0 on.
fn room_with_a_slot() -> Vec<(f32, f32, f32)> {
    let half = VS * 0.5;
    let column = |x: f32, y: f32| (0..13).map(move |iz| (x, y, iz as f32 * VS + half));
    let mut pts = Vec::new();
    for ix in 0..100 {
        for iy in 0..75 {
            let (x, y) = (ix as f32 * VS + half, iy as f32 * VS + half);
            let shell = ix == 0 || ix == 99 || iy == 0 || iy == 74;
            let wall = (4.0..4.16).contains(&x) && iy != 37;
            if shell || wall {
                pts.extend(column(x, y));
            } else if x < 5.0 {
                pts.push((x, y, half));
            }
        }
    }
    pts
}

#[test]
fn an_approach_never_heads_through_a_gap_too_narrow_to_plan_through() {
    let cfg = go2();
    let mut p = planner(&room_with_a_slot(), &cfg);
    // the frontier past the slot is nearest the goal, but no plan reaches it
    let path = p.plan_or_truncate((1.0, 3.0, 0.0), (7.0, 3.0, 0.0), &cfg);
    let end = *path.last().expect("an approach path on the robot's side");
    assert!(end.0 < 4.0, "approach ends at {end:?}");
}

#[test]
fn a_goal_sent_again_starts_without_the_old_dead_ends() {
    let cfg = go2();
    let mut p = planner(&walled_corridor(), &cfg);
    let (start, goal) = ((1.0, 1.0, 0.0), (7.0, 1.0, 0.0));
    // walk to each approach target until the approach gives up
    let mut at = start;
    for _ in 0..planner::MAX_DEAD_ENDS + 2 {
        match p.plan_or_truncate(at, goal, &cfg).last() {
            Some(&end) => at = end,
            None => break,
        }
    }
    assert!(p.plan_or_truncate(start, goal, &cfg).is_empty());
    p.reset_approach();
    assert!(!p.plan_or_truncate(start, goal, &cfg).is_empty());
}
