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

use std::collections::BinaryHeap;

use ahash::{AHashMap, AHashSet};

use crate::adjacency::{rise, CellId, SurfaceCells, SurfaceLookup};
use crate::bridge::{self, Bridge, BridgeState};
use crate::dijkstra::{goal_node_of, node_dijkstra, walk_preds, Scored};
use crate::edges::{NodeEdgeIdx, NodeId, PlannerGraph, NO_NODE};
use crate::mls_planner::Config;
use crate::nodes::penalty_of;
use crate::surfaces::ColumnIz;
use crate::voxel::{surface_point_xyz, VoxelKey};

/// Robot-rooted candidate search radius, in multiples of node spacing.
const CANDIDATE_RADIUS_FACTOR: f32 = 3.0;

/// Horizontal search radius when snapping a pose to the surface.
const SNAP_SEARCH_RADIUS_M: f32 = 1.5;

/// Max snap candidates tried when connecting the start.
const MAX_SNAP_ATTEMPTS: usize = 64;

/// On a blocked path, stop this far short of the last traversable point.
const BEST_EFFORT_DISTANCE_M: f32 = 1.0;

/// Reselection attempts when a bridge target turns out unplannable.
const BRIDGE_PLAN_RETRIES: usize = 3;

/// A cached path farther than this from the robot is stale, not resumable.
/// Guards against projecting onto a path on another floor, straight through
/// the slab between them.
const CACHED_PATH_MAX_DEVIATION_M: f32 = 1.0;

/// Snap candidates within this ground distance of the robot count as under
/// its footprint and seed its reachable component. The robot may stand on a
/// sub-clearance tread whose own edges are all impassable, so a single-cell
/// seed would strand the bridge search at its feet, while the full snap
/// radius would leak into pockets it cannot actually reach.
const FOOTPRINT_RADIUS_M: f32 = 0.4;

/// World-frame waypoints with the string-pulled cell path that produced them.
/// The cell path is cached for later safe truncation. A best-effort route
/// carries the optimistic bridge from its end to the goal component's closest
/// approach point.
pub struct PlannedPath {
    pub waypoints: Vec<(f32, f32, f32)>,
    pub cells: Vec<VoxelKey>,
    pub bridge: Option<Bridge>,
}

/// Waypoints and the string-pulled cell path of one planning attempt.
type PlannedRoute = (Vec<(f32, f32, f32)>, Vec<VoxelKey>);

/// Surface cells near the pose, nearest first in xy.
pub fn snap_candidates(
    surface_lookup: &SurfaceLookup,
    pose: (f32, f32, f32),
    voxel_size: f32,
    tolerance_m: f32,
) -> Vec<VoxelKey> {
    let ix = (pose.0 / voxel_size).floor() as i32;
    let iy = (pose.1 / voxel_size).floor() as i32;
    let target_iz = (pose.2 / voxel_size).floor() as i32 - 1;
    let tol_cells = (tolerance_m / voxel_size).ceil() as i32;
    let search_radius = (SNAP_SEARCH_RADIUS_M / voxel_size).ceil() as i32;

    let mut found: Vec<(i32, VoxelKey)> = Vec::new();
    for dix in -search_radius..=search_radius {
        for diy in -search_radius..=search_radius {
            if let Some(cell) =
                best_iz_in_column(surface_lookup, ix + dix, iy + diy, target_iz, tol_cells)
            {
                found.push((dix * dix + diy * diy, cell));
            }
        }
    }
    found.sort_by_key(|&(d2, _)| d2);
    found.into_iter().map(|(_, c)| c).collect()
}

/// Snap a pose to the nearest surface cell.
pub fn snap_pose_to_cell(
    surface_lookup: &SurfaceLookup,
    pose: (f32, f32, f32),
    voxel_size: f32,
    tolerance_m: f32,
) -> Option<VoxelKey> {
    snap_candidates(surface_lookup, pose, voxel_size, tolerance_m)
        .into_iter()
        .next()
}

fn best_iz_in_column(
    surface_lookup: &SurfaceLookup,
    ix: i32,
    iy: i32,
    target_iz: i32,
    tol_cells: i32,
) -> Option<VoxelKey> {
    let zs = surface_lookup.get(&(ix, iy))?;
    let mut best: Option<(i32, i32)> = None;
    for &iz in zs {
        let d = (iz - target_iz).abs();
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, iz));
        }
    }
    let (bd, iz) = best?;
    if bd > tol_cells {
        return None;
    }
    Some((ix, iy, iz))
}

/// Cell nearest the pose by euclidean distance among the given ids. Ties
/// break by id.
fn nearest_cell(
    cells: &SurfaceCells,
    ids: impl Iterator<Item = CellId>,
    pose: (f32, f32, f32),
    voxel_size: f32,
) -> Option<CellId> {
    let mut best: Option<(f32, CellId)> = None;
    for id in ids {
        let (ix, iy, iz) = cells.coord(id);
        let (x, y, z) = surface_point_xyz(ix, iy, iz, voxel_size);
        let d2 = (x - pose.0).powi(2) + (y - pose.1).powi(2) + (z - pose.2).powi(2);
        if best.is_none_or(|(bd, bid)| d2 < bd || (d2 == bd && id < bid)) {
            best = Some((d2, id));
        }
    }
    best.map(|(_, id)| id)
}

/// Resolve an unseen goal to the nearest standable cell, so replans converge
/// on it as the map grows. Zero-clearance fringe crumbs lose to passable
/// floor, else a barely-mapped goal region drags planning onto junk.
fn nearest_goal_cell(plg: &PlannerGraph, pose: (f32, f32, f32), config: &Config) -> Option<CellId> {
    let passable = plg.cells.ids().filter(|&id| {
        plg.wall_state
            .dist
            .get(id as usize)
            .copied()
            .unwrap_or(f32::INFINITY)
            >= config.wall_clearance_m
    });
    nearest_cell(&plg.cells, passable, pose, config.voxel_size)
        .or_else(|| nearest_cell(&plg.cells, plg.cells.ids(), pose, config.voxel_size))
}

/// CellId-indexed mask of the cells reachable from the sources over passable
/// edges.
fn reachable_set(cells: &SurfaceCells, sources: &[CellId]) -> Vec<bool> {
    let mut mask = vec![false; cells.slot_capacity()];
    let mut queue: Vec<CellId> = Vec::new();
    for &s in sources {
        if !mask[s as usize] {
            mask[s as usize] = true;
            queue.push(s);
        }
    }
    let mut head = 0;
    while head < queue.len() {
        let u = queue[head];
        head += 1;
        for edge in cells.neighbors(u) {
            if edge.cost.is_finite() && !mask[edge.dest as usize] {
                mask[edge.dest as usize] = true;
                queue.push(edge.dest);
            }
        }
    }
    mask
}

/// Retry-invariant inputs of one plan call.
struct PlanCtx<'a> {
    plg: &'a PlannerGraph,
    by_col: &'a ColumnIz,
    config: &'a Config,
    node_cells: &'a AHashSet<NodeId>,
    start_pose: (f32, f32, f32),
    start_candidates: &'a [VoxelKey],
    start_cells: &'a [CellId],
}

/// Plan path from start pose to goal pose using the node graph.
///
/// With a bridge state, an unreachable goal falls back to optimism: a proxy
/// goal on the nearest standable cell, or a best-effort route to the near
/// side of the best bridge toward the goal, both carrying `bridge`. Without
/// one, or with bridging disabled by config, only full paths to the snapped
/// goal are returned.
pub fn plan(
    plg: &PlannerGraph,
    by_col: &ColumnIz,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    config: &Config,
    bridge_state: Option<&BridgeState>,
) -> Option<PlannedPath> {
    let voxel_size = config.voxel_size;
    let z_tolerance_m = config.robot_height;
    let start_candidates =
        snap_candidates(&plg.surface_lookup, start_pose, voxel_size, z_tolerance_m);
    if start_candidates.is_empty() {
        tracing::debug!(
            ?start_pose,
            "plan failed: start does not snap to any surface cell"
        );
        return None;
    }
    let snapped_goal = snap_pose_to_cell(&plg.surface_lookup, goal_pose, voxel_size, z_tolerance_m)
        .and_then(|coord| plg.cells.id(coord));
    if snapped_goal.is_none() {
        tracing::debug!(?goal_pose, "goal is not on a known surface");
    }
    let start_cells: Vec<CellId> = start_candidates
        .iter()
        .take(MAX_SNAP_ATTEMPTS)
        .filter_map(|&candidate| plg.cells.id(candidate))
        .collect();
    if start_cells.is_empty() {
        return None;
    }
    let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
    let ctx = PlanCtx {
        plg,
        by_col,
        config,
        node_cells: &node_cells,
        start_pose,
        start_candidates: &start_candidates,
        start_cells: &start_cells,
    };

    if let Some(goal_cell) = snapped_goal {
        if let Some((waypoints, cells)) = plan_to_cell(
            plg,
            start_pose,
            goal_pose,
            goal_cell,
            &start_cells,
            &node_cells,
            config,
        ) {
            return Some(PlannedPath {
                waypoints,
                cells,
                bridge: None,
            });
        }
    }

    // Everything below is optimism, gated on the caller opting in and on
    // bridging being enabled at all.
    let state = bridge_state?;
    if config.bridge_max_hop_m <= 0.0 {
        return None;
    }

    let mut reachable: Option<Vec<bool>> = None;
    if let Some(goal_cell) = snapped_goal {
        if let Some(path) = ctx.bridge_fallback(goal_cell, goal_pose, &mut reachable, state) {
            return Some(path);
        }
    }

    // Retry against the nearest standable cell as a proxy goal: a goal
    // snapped onto a degraded fringe crumb must not stall planning while
    // solid floor sits just as close. The proxy path carries a bridge to the
    // true goal so replans keep converging on it.
    let Some(alt_goal) =
        nearest_goal_cell(plg, goal_pose, config).filter(|&a| Some(a) != snapped_goal)
    else {
        if snapped_goal.is_none() {
            tracing::debug!(?goal_pose, "plan failed: map has no surface cells");
        }
        return None;
    };
    let attempt_pose = plg.cells.xyz(alt_goal, voxel_size);
    if let Some((waypoints, cells)) = plan_to_cell(
        plg,
        start_pose,
        attempt_pose,
        alt_goal,
        &start_cells,
        &node_cells,
        config,
    ) {
        let bridge = Some((*waypoints.last().expect("non-empty path"), goal_pose));
        return Some(PlannedPath {
            waypoints,
            cells,
            bridge,
        });
    }
    ctx.bridge_fallback(alt_goal, attempt_pose, &mut reachable, state)
}

impl PlanCtx<'_> {
    /// Best-effort route to the near side of the best optimistic bridge
    /// toward an unreachable goal. A selection whose near side turns out
    /// unplannable is banned and the runner-up tried, so one bad target
    /// never silently drops the whole replan. No admissible bridge returns
    /// none, so the caller holds instead of wandering.
    fn bridge_fallback(
        &self,
        goal_cell: CellId,
        goal_pose: (f32, f32, f32),
        reachable: &mut Option<Vec<bool>>,
        state: &BridgeState,
    ) -> Option<PlannedPath> {
        let voxel_size = self.config.voxel_size;
        let reachable = reachable.get_or_insert_with(|| self.footprint_reachable());
        // Start the bridge search from standable footing under the robot,
        // not necessarily its nearest cell.
        let bridge_start = self
            .start_cells
            .iter()
            .copied()
            .find(|&id| {
                reachable[id as usize]
                    && self
                        .plg
                        .wall_state
                        .dist
                        .get(id as usize)
                        .copied()
                        .unwrap_or(f32::INFINITY)
                        >= self.config.wall_clearance_m
            })
            .unwrap_or(self.start_cells[0]);
        let selector = bridge::Selector::new(
            self.plg,
            self.by_col,
            reachable,
            (bridge_start, self.start_pose),
            (goal_cell, goal_pose),
            self.node_cells,
            self.config,
        );
        let mut banned_aims: Vec<(f32, f32, f32)> = Vec::new();
        for _ in 0..BRIDGE_PLAN_RETRIES {
            let selection = selector.select(state, &banned_aims)?;
            let target_pose = self.plg.cells.xyz(selection.near, voxel_size);
            tracing::debug!(
                ?goal_pose,
                ?target_pose,
                aim = ?selection.aim,
                "goal unreachable; planning to the bridge toward it"
            );
            let planned = plan_to_cell(
                self.plg,
                self.start_pose,
                target_pose,
                selection.near,
                self.start_cells,
                self.node_cells,
                self.config,
            );
            let Some((waypoints, cells)) = planned else {
                banned_aims.push(selection.aim);
                continue;
            };
            return Some(PlannedPath {
                waypoints,
                cells,
                bridge: Some((target_pose, selection.aim)),
            });
        }
        None
    }

    /// Cells reachable over passable edges from the columns under the
    /// robot's feet. A level within the footprint radius but a step up or
    /// down is a different floor, not footing.
    fn footprint_reachable(&self) -> Vec<bool> {
        let voxel_size = self.config.voxel_size;
        let footprint: Vec<CellId> = self
            .start_candidates
            .iter()
            .filter(|&&(ix, iy, iz)| {
                let x = (ix as f32 + 0.5) * voxel_size;
                let y = (iy as f32 + 0.5) * voxel_size;
                let z = (iz as f32 + 1.0) * voxel_size;
                (x - self.start_pose.0).hypot(y - self.start_pose.1) <= FOOTPRINT_RADIUS_M
                    && (z - self.start_pose.2).abs() <= self.config.step_threshold_m + voxel_size
            })
            .filter_map(|&c| self.plg.cells.id(c))
            .collect();
        if footprint.is_empty() {
            reachable_set(&self.plg.cells, &self.start_cells[..1])
        } else {
            reachable_set(&self.plg.cells, &footprint)
        }
    }
}

/// Plan from the start snap candidates to a resolved goal cell, or none when
/// no start candidate connects to its component.
fn plan_to_cell(
    plg: &PlannerGraph,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    goal_cell: CellId,
    start_cells: &[CellId],
    node_cells: &AHashSet<NodeId>,
    config: &Config,
) -> Option<PlannedRoute> {
    let voxel_size = config.voxel_size;

    let Some((goal_node, goal_segment)) = goal_node_of(plg, goal_cell, node_cells) else {
        tracing::debug!(
            ?goal_pose,
            "plan failed: goal's connected component has no graph node"
        );
        return None;
    };

    // Rooted at the goal so one pass covers every node's cost-to-go.
    let (cost_to_go, pred_to_goal) = node_dijkstra(plg, goal_node);

    let radius = (config.node_spacing_m * CANDIDATE_RADIUS_FACTOR).max(voxel_size);
    // Nearest candidate first so the lead-in anchors at the robot's own cell.
    // The rest are only a fallback when it cannot connect.
    let (near, rest) = start_cells.split_at(start_cells.len().min(1));
    let entry = select_entry(
        plg,
        near,
        goal_cell,
        goal_node,
        &cost_to_go,
        &pred_to_goal,
        node_cells,
        radius,
    )
    .or_else(|| {
        select_entry(
            plg,
            rest,
            goal_cell,
            goal_node,
            &cost_to_go,
            &pred_to_goal,
            node_cells,
            radius,
        )
    });
    let Some((lead_in, node_seq)) = entry else {
        tracing::debug!(
            candidates = start_cells.len(),
            reachable_nodes = cost_to_go.len(),
            total_nodes = plg.nodes.len(),
            "no plan: start and goal cells lie on separate connected surface components",
        );
        return None;
    };

    // Max traversable step in cells, the hard bound shared with the graph.
    let step_cells = config.step_cells();

    let wall_cost = WallCost {
        clearance_m: config.wall_clearance_m,
        buffer_m: config.wall_buffer_m,
        buffer_weight: config.wall_buffer_weight,
        voxel_size,
    };
    // A direct goal connection carries the whole route in its lead-in.
    let goal_segment: &[CellId] = if node_seq.is_empty() {
        &[]
    } else {
        &goal_segment
    };
    let cells = assemble_cells(plg, &node_seq, &lead_in, goal_segment);
    let cells = string_pull(plg, &cells, step_cells, &wall_cost);
    let waypoints = cells_to_waypoints(plg, &cells, start_pose, goal_pose, voxel_size);
    let waypoints = crate::smoother::smooth_path(
        plg,
        waypoints,
        step_cells,
        &wall_cost,
        config.step_penalty_weight,
    );
    let path_cells: Vec<VoxelKey> = cells.iter().map(|&id| plg.cells.coord(id)).collect();
    Some((waypoints, path_cells))
}

/// Re-validate a cached path against the current surface. Returns the route
/// ahead up to a standoff short of the first blockage, or empty when nothing
/// ahead is safe, which the follower reads as a stop.
pub fn truncate_to_safe(
    plg: &PlannerGraph,
    cached: &[VoxelKey],
    start_pose: (f32, f32, f32),
    config: &Config,
) -> Vec<(f32, f32, f32)> {
    if cached.len() < 2 {
        return Vec::new();
    }
    let voxel_size = config.voxel_size;
    let step_cells = config.step_cells();
    let wall_cost = WallCost {
        clearance_m: config.wall_clearance_m,
        buffer_m: config.wall_buffer_m,
        buffer_weight: config.wall_buffer_weight,
        voxel_size,
    };

    // The cached path's head is the stale original start. Resume from where the
    // robot sits on it so the follower is pulled forward, never back to it. A
    // robot no longer near the path at all has left it, so nothing resumes.
    let Some(resume) = resume_segment(cached, start_pose, voxel_size) else {
        return Vec::new();
    };

    // Walk each chord ahead at surface resolution. On a blockage, keep up to the
    // last traversable cell so the path ends at the obstacle, not the prior anchor.
    let mut waypoints = vec![start_pose];
    let mut blocked = false;
    for j in resume..cached.len() - 1 {
        let (last_safe, cut) = last_safe_on_chord(
            plg,
            cached[j],
            cached[j + 1],
            step_cells,
            &wall_cost,
            voxel_size,
        );
        if cut {
            if let Some(p) = last_safe {
                if waypoints.last() != Some(&p) {
                    waypoints.push(p);
                }
            }
            blocked = true;
            break;
        }
        let (ix, iy, iz) = cached[j + 1];
        waypoints.push(surface_point_xyz(ix, iy, iz, voxel_size));
    }

    if waypoints.len() < 2 {
        return Vec::new();
    }

    // Hold the standoff only when blocked. A clean run to the goal has nothing
    // to stand off from.
    if blocked {
        return back_off_tail(&waypoints, BEST_EFFORT_DISTANCE_M);
    }
    waypoints
}

/// Trim `distance` off the goal end, measured in the ground plane.
fn back_off_tail(waypoints: &[(f32, f32, f32)], distance: f32) -> Vec<(f32, f32, f32)> {
    let mut remaining = distance;
    for i in (1..waypoints.len()).rev() {
        let (b, a) = (waypoints[i], waypoints[i - 1]);
        let seg = (b.0 - a.0).hypot(b.1 - a.1);
        if seg < remaining {
            remaining -= seg;
            continue;
        }
        let t = if seg == 0.0 {
            0.0
        } else {
            (seg - remaining) / seg
        };
        let cut = (
            a.0 + (b.0 - a.0) * t,
            a.1 + (b.1 - a.1) * t,
            a.2 + (b.2 - a.2) * t,
        );
        let mut out = waypoints[..i].to_vec();
        if out.last() != Some(&cut) {
            out.push(cut);
        }
        return if out.len() >= 2 { out } else { Vec::new() };
    }
    Vec::new()
}

/// Walk the straight chord a -> b at surface resolution, the same sampling the
/// segment validator uses. Returns the world point of the last traversable
/// surface cell reached and whether the chord was cut short of b. The point is
/// None only when the chord's start column is already off the surface.
fn last_safe_on_chord(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    step_cells: i32,
    wc: &WallCost,
    voxel_size: f32,
) -> (Option<(f32, f32, f32)>, bool) {
    let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let samples = dx.abs().max(dy.abs()) * 2;
    if samples == 0 {
        // Same column: traversable only if it is not a pure vertical move.
        return if dz == 0 {
            (Some(surface_point_xyz(a.0, a.1, a.2, voxel_size)), false)
        } else {
            (None, true)
        };
    }
    let (mut last_ix, mut last_iy) = (i32::MIN, i32::MIN);
    let mut prev_iz: Option<i32> = None;
    let mut last_safe: Option<(f32, f32, f32)> = None;
    for k in 0..=samples {
        let t = k as f32 / samples as f32;
        let ix = (a.0 as f32 + t * dx as f32).round() as i32;
        let iy = (a.1 as f32 + t * dy as f32).round() as i32;
        if ix == last_ix && iy == last_iy {
            continue;
        }
        last_ix = ix;
        last_iy = iy;
        let iz_line = a.2 as f32 + t * dz as f32;
        let Some(zs) = plg.surface_lookup.get(&(ix, iy)) else {
            return (last_safe, true);
        };
        // Surface cell in this column nearest the interpolated segment height.
        let mut nearest: Option<(f32, i32)> = None;
        for &iz in zs {
            let d = (iz as f32 - iz_line).abs();
            if nearest.is_none_or(|(bd, _)| d < bd) {
                nearest = Some((d, iz));
            }
        }
        let Some((d, iz)) = nearest else {
            return (last_safe, true);
        };
        if d > step_cells as f32 {
            return (last_safe, true);
        }
        if prev_iz.is_some_and(|p| (iz - p).abs() > step_cells) {
            return (last_safe, true);
        }
        let pen = match plg.cells.id((ix, iy, iz)) {
            Some(id) => {
                let wall_dist = plg
                    .wall_state
                    .dist
                    .get(id as usize)
                    .copied()
                    .unwrap_or(f32::INFINITY);
                penalty_of(wall_dist, wc.clearance_m, wc.buffer_m, wc.buffer_weight)
            }
            None => 1.0,
        };
        if !pen.is_finite() {
            return (last_safe, true);
        }
        prev_iz = Some(iz);
        last_safe = Some(surface_point_xyz(ix, iy, iz, voxel_size));
    }
    (last_safe, false)
}

/// Index of the cached segment the robot is on, by nearest-point projection
/// in 3D, or none when the robot has strayed too far from the whole path.
/// Distance must count z: an xy projection would resume a path on another
/// floor directly below the robot.
fn resume_segment(cached: &[VoxelKey], start: (f32, f32, f32), voxel_size: f32) -> Option<usize> {
    let mut best = 0usize;
    let mut best_d2 = f32::INFINITY;
    for i in 0..cached.len() - 1 {
        let a = surface_point_xyz(cached[i].0, cached[i].1, cached[i].2, voxel_size);
        let b = surface_point_xyz(
            cached[i + 1].0,
            cached[i + 1].1,
            cached[i + 1].2,
            voxel_size,
        );
        let d2 = point_segment_dist2(a, b, start);
        if d2 < best_d2 {
            best_d2 = d2;
            best = i;
        }
    }
    (best_d2 <= CACHED_PATH_MAX_DEVIATION_M * CACHED_PATH_MAX_DEVIATION_M).then_some(best)
}

/// Squared distance from point p to segment a-b.
fn point_segment_dist2(a: (f32, f32, f32), b: (f32, f32, f32), p: (f32, f32, f32)) -> f32 {
    let (abx, aby, abz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let denom = abx * abx + aby * aby + abz * abz;
    let t = if denom == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * abx + (p.1 - a.1) * aby + (p.2 - a.2) * abz) / denom).clamp(0.0, 1.0)
    };
    let (cx, cy, cz) = (a.0 + t * abx, a.1 + t * aby, a.2 + t * abz);
    let (dx, dy, dz) = (p.0 - cx, p.1 - cy, p.2 - cz);
    dx * dx + dy * dy + dz * dz
}

/// Pick the entry node by connect cost plus cost-to-go, with its on-surface
/// lead-in and the node sequence to the goal. A goal cell inside the search
/// radius connects directly instead, signalled by an empty node sequence.
#[allow(clippy::too_many_arguments)]
fn select_entry(
    plg: &PlannerGraph,
    start_cells: &[CellId],
    goal_cell: CellId,
    goal_node: NodeId,
    cost_to_go: &AHashMap<NodeId, f32>,
    pred_to_goal: &AHashMap<NodeId, NodeId>,
    node_cells: &AHashSet<NodeId>,
    radius_m: f32,
) -> Option<(Vec<CellId>, Vec<NodeId>)> {
    let (connect_dist, connect_pred) = robot_search(&plg.cells, start_cells, radius_m);

    if connect_dist.contains_key(&goal_cell) {
        let mut lead = walk_local_preds(&connect_pred, goal_cell);
        lead.reverse();
        return Some((lead, Vec::new()));
    }

    let mut entry_node = NO_NODE;
    let mut best_score = f32::INFINITY;
    // Scan the bounded reachable set, not every node. Tie-break by cell id for
    // deterministic order.
    for (&cell, &connect) in &connect_dist {
        if !node_cells.contains(&cell) {
            continue;
        }
        let Some(&ctg) = cost_to_go.get(&cell) else {
            continue;
        };
        let score = connect + ctg;
        let better = match score.partial_cmp(&best_score) {
            Some(std::cmp::Ordering::Less) => true,
            Some(std::cmp::Ordering::Equal) => cell < entry_node,
            _ => false,
        };
        if better {
            best_score = score;
            entry_node = cell;
        }
    }

    if best_score.is_finite() {
        let mut lead = walk_local_preds(&connect_pred, entry_node);
        lead.reverse();
        return Some((lead, follow_preds(entry_node, goal_node, pred_to_goal)?));
    }

    for &start_cell in start_cells {
        let start_segment = walk_preds(&plg.cell_state, start_cell);
        let region_node = *start_segment
            .last()
            .expect("walk_preds returns at least the start cell");
        if !node_cells.contains(&region_node)
            || !cost_to_go.get(&region_node).is_some_and(|c| c.is_finite())
        {
            continue;
        }
        let Some(node_seq) = follow_preds(region_node, goal_node, pred_to_goal) else {
            continue;
        };
        return Some((start_segment, node_seq));
    }
    None
}

/// Bounded multi-source Dijkstra from the robot snap candidate cells. Cost is
/// wall-penalized for steering, but the radius bounds metric distance, not
/// penalized cost.
fn robot_search(
    cells: &SurfaceCells,
    sources: &[CellId],
    radius_m: f32,
) -> (AHashMap<CellId, f32>, AHashMap<CellId, CellId>) {
    let mut dist: AHashMap<CellId, f32> = AHashMap::new();
    let mut geo: AHashMap<CellId, f32> = AHashMap::new();
    let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
    let mut heap: BinaryHeap<Scored<NodeId>> = BinaryHeap::new();
    for &source in sources {
        dist.insert(source, 0.0);
        geo.insert(source, 0.0);
        heap.push(Scored(0.0, source));
    }

    while let Some(Scored(d, u)) = heap.pop() {
        if d > dist.get(&u).copied().unwrap_or(f32::INFINITY) {
            continue;
        }
        // Stop expanding past the metric radius.
        if geo.get(&u).copied().unwrap_or(f32::INFINITY) > radius_m {
            continue;
        }
        for edge in cells.neighbors(u) {
            let nd = d + edge.cost;
            if nd < dist.get(&edge.dest).copied().unwrap_or(f32::INFINITY) {
                dist.insert(edge.dest, nd);
                geo.insert(edge.dest, geo[&u] + edge.base_cost);
                pred.insert(edge.dest, u);
                heap.push(Scored(nd, edge.dest));
            }
        }
    }
    (dist, pred)
}

/// Walk predecessors back to the search source.
fn walk_local_preds(pred: &AHashMap<CellId, CellId>, from: CellId) -> Vec<CellId> {
    let mut path = vec![from];
    let mut cur = from;
    while let Some(&p) = pred.get(&cur) {
        cur = p;
        path.push(cur);
    }
    path
}

/// Build the node sequence by following goal-pointing predecessors.
fn follow_preds(
    from: NodeId,
    goal: NodeId,
    pred: &AHashMap<NodeId, NodeId>,
) -> Option<Vec<NodeId>> {
    let mut seq = vec![from];
    let mut cur = from;
    while cur != goal {
        let &next = pred.get(&cur)?;
        cur = next;
        seq.push(cur);
    }
    Some(seq)
}

/// Append a cell, cancelling an out-and-back spur when the next cell retraces
/// the second-to-last.
fn push_cell(cells: &mut Vec<CellId>, c: CellId) {
    if cells.len() >= 2 && cells[cells.len() - 2] == c {
        cells.pop();
    } else if cells.last() != Some(&c) {
        cells.push(c);
    }
}

/// Build the cell path from the entry lead-in through the node edges to the goal.
fn assemble_cells(
    plg: &PlannerGraph,
    node_seq: &[NodeId],
    lead_in: &[CellId],
    goal_segment: &[CellId],
) -> Vec<CellId> {
    let mut cells: Vec<CellId> = Vec::new();
    for &c in lead_in {
        push_cell(&mut cells, c);
    }

    for pair in node_seq.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let edge_idx =
            edge_between(plg, a, b).expect("consecutive nodes in path must share an edge");
        let edge = &plg.node_edges[edge_idx as usize];
        let (start_side, end_side) = if a == edge.a {
            (edge.boundary_u, edge.boundary_v)
        } else {
            (edge.boundary_v, edge.boundary_u)
        };

        let mut from_a = walk_preds(&plg.cell_state, start_side);
        from_a.reverse();
        let to_b = walk_preds(&plg.cell_state, end_side);

        for c in from_a.into_iter().chain(to_b) {
            push_cell(&mut cells, c);
        }
    }

    for &c in goal_segment.iter().rev() {
        push_cell(&mut cells, c);
    }

    cells
}

/// Convert the cell path to world waypoints, with the raw start and goal poses
/// as the endpoints.
fn cells_to_waypoints(
    plg: &PlannerGraph,
    cells: &[CellId],
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    voxel_size: f32,
) -> Vec<(f32, f32, f32)> {
    let mut waypoints: Vec<(f32, f32, f32)> = Vec::with_capacity(cells.len() + 2);
    waypoints.push(start_pose);
    for &id in cells {
        let (ix, iy, iz) = plg.cells.coord(id);
        waypoints.push(surface_point_xyz(ix, iy, iz, voxel_size));
    }
    waypoints.push(goal_pose);
    waypoints
}

/// Clearance and step limits the smoother holds the path to.
pub(crate) struct WallCost {
    pub(crate) clearance_m: f32,
    pub(crate) buffer_m: f32,
    pub(crate) buffer_weight: f32,
    pub(crate) voxel_size: f32,
}

/// Replace runs of cells with straight chords that come no closer to a wall and
/// climb no more than the run they replace.
fn string_pull(
    plg: &PlannerGraph,
    cells: &[CellId],
    step_cells: i32,
    wc: &WallCost,
) -> Vec<CellId> {
    if cells.len() <= 2 {
        return cells.to_vec();
    }
    let metrics = |from: CellId, to: CellId| {
        segment_metrics(
            plg,
            plg.cells.coord(from),
            plg.cells.coord(to),
            step_cells,
            wc,
        )
    };
    let mut out = vec![cells[0]];
    let mut anchor = 0;
    while anchor + 1 < cells.len() {
        let mut best = anchor + 1;
        let mut rough_pen = 1.0_f32;
        let mut rough_rise = 0.0_f32;
        let mut j = anchor + 1;
        while j < cells.len() {
            match metrics(cells[j - 1], cells[j]) {
                Some((pen, rise)) => {
                    rough_pen = rough_pen.max(pen);
                    rough_rise += rise;
                }
                // Infeasible step. Raise the baseline so the chord breaks.
                None => {
                    rough_pen = f32::INFINITY;
                    rough_rise = f32::INFINITY;
                }
            }
            match metrics(cells[anchor], cells[j]) {
                Some((pen, rise)) if pen <= rough_pen + 1e-3 && rise <= rough_rise + 1e-3 => {
                    best = j
                }
                _ => break,
            }
            j += 1;
        }
        out.push(cells[best]);
        anchor = best;
    }
    out
}

/// Worst wall penalty and total climb along the straight segment a -> b. None if
/// it leaves the surface, exceeds step_cells, or enters the hard clearance.
fn segment_metrics(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    step_cells: i32,
    wc: &WallCost,
) -> Option<(f32, f32)> {
    let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let samples = dx.abs().max(dy.abs()) * 2;
    if samples == 0 {
        // A same-column vertical chord is not traversable.
        return (dz == 0).then_some((1.0, 0.0));
    }
    let (mut last_ix, mut last_iy) = (i32::MIN, i32::MIN);
    let mut prev_iz: Option<i32> = None;
    let mut max_pen = 1.0_f32;
    let mut rise_cells = 0i32;
    for k in 0..=samples {
        let t = k as f32 / samples as f32;
        let ix = (a.0 as f32 + t * dx as f32).round() as i32;
        let iy = (a.1 as f32 + t * dy as f32).round() as i32;
        if ix == last_ix && iy == last_iy {
            continue;
        }
        last_ix = ix;
        last_iy = iy;
        let iz_line = a.2 as f32 + t * dz as f32;
        let zs = plg.surface_lookup.get(&(ix, iy))?;
        // Surface cell in this column nearest the interpolated segment height.
        let mut nearest: Option<(f32, i32)> = None;
        for &iz in zs {
            let d = (iz as f32 - iz_line).abs();
            if nearest.is_none_or(|(bd, _)| d < bd) {
                nearest = Some((d, iz));
            }
        }
        let (d, iz) = nearest?;
        if d > step_cells as f32 {
            return None;
        }
        // Tally climb and reject an untraversable step between columns.
        if let Some(p) = prev_iz {
            let step = (iz - p).abs();
            if step > step_cells {
                return None;
            }
            rise_cells += step;
        }
        prev_iz = Some(iz);
        // Columns on the surface but not in the graph carry no wall penalty.
        let p = match plg.cells.id((ix, iy, iz)) {
            Some(id) => {
                let wall_dist = plg
                    .wall_state
                    .dist
                    .get(id as usize)
                    .copied()
                    .unwrap_or(f32::INFINITY);
                penalty_of(wall_dist, wc.clearance_m, wc.buffer_m, wc.buffer_weight)
            }
            None => 1.0,
        };
        if !p.is_finite() {
            return None;
        }
        max_pen = max_pen.max(p);
    }
    Some((max_pen, rise(rise_cells, wc.voxel_size)))
}

fn edge_between(plg: &PlannerGraph, a: NodeId, b: NodeId) -> Option<NodeEdgeIdx> {
    for &edge_idx in plg.node_adj.get(&a)? {
        let edge = &plg.node_edges[edge_idx as usize];
        let other = if edge.a == a { edge.b } else { edge.a };
        if other == b {
            return Some(edge_idx);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup};
    use crate::edges::build_node_edges;
    use crate::nodes::NodeData;

    const VOXEL: f32 = 0.1;
    const Z_TOL: f32 = 1.5;

    fn graph_with_nodes(surface_cells: &[VoxelKey], node_cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(surface_cells, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        plg.nodes = node_cells
            .iter()
            .map(|&c| {
                let id = plg.cells.id(c).expect("node cell must be in surface");
                NodeData {
                    cell_id: id,
                    pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
                }
            })
            .collect();
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
            &mut plg.node_adj,
        );
        plg
    }

    fn strip(n: i32) -> Vec<VoxelKey> {
        (0..n).map(|x| (x, 0, 0)).collect()
    }

    use crate::mls_planner::test_config;

    fn plan_with(
        plg: &PlannerGraph,
        by_col: &ColumnIz,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
    ) -> Option<PlannedPath> {
        plan(
            plg,
            by_col,
            start,
            goal,
            &test_config(),
            Some(&BridgeState::default()),
        )
    }

    fn plan_simple(
        plg: &PlannerGraph,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
    ) -> Option<Vec<(f32, f32, f32)>> {
        plan_with(plg, &ColumnIz::default(), start, goal).map(|p| p.waypoints)
    }

    fn surface_graph(cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(cells, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        plg
    }

    fn truncate_config() -> Config {
        test_config()
    }

    #[test]
    fn truncate_keeps_the_full_clear_route_ahead() {
        let cfg = truncate_config();
        // Cached route 0 -> 39 (cells), still fully traversable, robot at start.
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let start = surface_point_xyz(0, 0, 0, VOXEL);

        // No blockage, so no standoff: start pose plus every cell ahead, where
        // the robot's own cell 0 is dropped.
        let full = truncate_to_safe(&surface_graph(&cached), &cached, start, &cfg);
        assert_eq!(full.len(), cached.len(), "start pose + cells 1..=39");
        assert_eq!(full[1], surface_point_xyz(1, 0, 0, VOXEL));
        assert_eq!(*full.last().unwrap(), surface_point_xyz(39, 0, 0, VOXEL));
    }

    #[test]
    fn truncate_holds_standoff_ahead_of_advanced_robot() {
        let cfg = truncate_config();
        // Robot has advanced to x=20 along a 0 -> 39 route, and the surface is
        // now gone at x=35 (a door closed ahead).
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let robot = surface_point_xyz(20, 0, 0, VOXEL);
        let blocked: Vec<VoxelKey> = (0..40).filter(|&x| x != 35).map(|x| (x, 0, 0)).collect();

        let wp = truncate_to_safe(&surface_graph(&blocked), &cached, robot, &cfg);
        assert_eq!(wp[0], robot);

        // Never behind the robot, always forward toward the goal.
        let xs: Vec<f32> = wp.iter().map(|w| w.0).collect();
        assert!(
            xs.iter().all(|&x| x >= robot.0 - 1e-4),
            "backtracked: {xs:?}"
        );
        assert!(xs.windows(2).all(|p| p[1] >= p[0]), "not forward: {xs:?}");

        // Stops a standoff short of the last traversable cell (x=34).
        let last_safe = surface_point_xyz(34, 0, 0, VOXEL);
        let last = *wp.last().unwrap();
        let gap = (last_safe.0 - last.0).hypot(last_safe.1 - last.1);
        assert!(
            (gap - BEST_EFFORT_DISTANCE_M).abs() < VOXEL,
            "standoff is {gap} m, expected ~{BEST_EFFORT_DISTANCE_M}"
        );
    }

    #[test]
    fn truncate_walks_into_a_sparse_chord_to_the_blockage() {
        let cfg = truncate_config();
        // Sparse cached route: one 0 -> 39 chord, as string_pull leaves a
        // straight approach. The surface is gone at x=20, mid-chord, where the
        // old anchor-level cut would have discarded the whole chord and stopped
        // back at x=0.
        let cached: Vec<VoxelKey> = vec![(0, 0, 0), (39, 0, 0)];
        let surface: Vec<VoxelKey> = (0..40).filter(|&x| x != 20).map(|x| (x, 0, 0)).collect();
        let robot = surface_point_xyz(0, 0, 0, VOXEL);

        let wp = truncate_to_safe(&surface_graph(&surface), &cached, robot, &cfg);

        // It advances well into the chord and stops a standoff short of the gap.
        let last = *wp.last().unwrap();
        assert!(last.0 > 0.5, "did not walk into the chord: {last:?}");
        let last_safe = surface_point_xyz(19, 0, 0, VOXEL);
        let gap = (last_safe.0 - last.0).hypot(last_safe.1 - last.1);
        assert!(
            (gap - BEST_EFFORT_DISTANCE_M).abs() < VOXEL,
            "standoff is {gap} m from the last safe cell, expected ~{BEST_EFFORT_DISTANCE_M}"
        );
    }

    #[test]
    fn truncate_refuses_a_path_on_another_floor() {
        let cfg = truncate_config();
        // Cached ground-floor route, robot now one floor above it. Resuming
        // would send the follower straight down through the slab.
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let robot = (2.0, 0.05, 2.6);
        assert!(
            truncate_to_safe(&surface_graph(&cached), &cached, robot, &cfg).is_empty(),
            "a cached path on another floor is stale, not resumable"
        );
    }

    #[test]
    fn truncate_stops_inside_standoff_or_at_blockage() {
        let cfg = truncate_config();
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let robot = surface_point_xyz(0, 0, 0, VOXEL);

        // Blockage at the next step: nothing safe ahead, stop.
        let at_robot: Vec<VoxelKey> = (0..40).filter(|&x| x != 1).map(|x| (x, 0, 0)).collect();
        assert!(
            truncate_to_safe(&surface_graph(&at_robot), &cached, robot, &cfg).is_empty(),
            "blockage at the next step -> stop"
        );

        // Blockage only ~0.4 m ahead, inside the standoff: the best-effort point
        // is behind the robot, so stop.
        let near: Vec<VoxelKey> = (0..40).filter(|&x| x != 5).map(|x| (x, 0, 0)).collect();
        assert!(
            truncate_to_safe(&surface_graph(&near), &cached, robot, &cfg).is_empty(),
            "inside the standoff -> stop"
        );
    }

    /// Wide strips so interior cells exist and the islands are not all
    /// boundary crumbs.
    fn slab(x0: i32, x1: i32, y0: i32, y1: i32, z: i32) -> Vec<VoxelKey> {
        let mut cells = Vec::new();
        for x in x0..=x1 {
            for y in y0..=y1 {
                cells.push((x, y, z));
            }
        }
        cells
    }

    #[test]
    fn plan_bridges_a_short_gap_to_a_disconnected_goal() {
        // Start slab and a goal island past a 0.7 m gap, farther from the
        // start pose than SNAP_SEARCH_RADIUS_M so snapping cannot cross. The
        // path must end at the start slab's lip and carry a bridge aimed at
        // the island's near edge.
        let mut cells = slab(0, 30, 0, 4, 0);
        cells.extend(slab(38, 46, 0, 4, 0));
        let plg = graph_with_nodes(&cells, &[(5, 2, 0), (42, 2, 0)]);
        let path = plan_with(
            &plg,
            &ColumnIz::default(),
            (0.25, 0.25, 0.1),
            (4.25, 0.25, 0.1),
        )
        .unwrap();
        let end = path.waypoints.last().unwrap();
        assert!(
            (end.0 - 3.05).abs() < 2.0 * VOXEL,
            "path must end at the start slab's lip: {end:?}"
        );
        let (near, aim) = path.bridge.expect("best-effort plan carries a bridge");
        assert!((near.0 - 3.05).abs() < 2.0 * VOXEL, "bridge near: {near:?}");
        assert!((aim.0 - 3.85).abs() < 2.0 * VOXEL, "bridge aim: {aim:?}");
        let gap = ((aim.0 - near.0).powi(2) + (aim.1 - near.1).powi(2)).sqrt();
        assert!(gap < 1.0, "bridge must be a short hop: {gap}");
    }

    #[test]
    fn plan_bridges_toward_the_gap_not_under_the_goal() {
        // Start strip at y=0. The goal component is a far row at y=30 plus an
        // arm at x=25 reaching down to y=8: a 0.6 m gap, farther from the
        // start pose than the snap radius. The direct line to the goal spans
        // 2.8 m of nothing, so the only admissible bridge is the arm tip and
        // the path must head for x=25 instead of under the goal.
        let mut cells = slab(0, 30, 0, 2, 0);
        cells.extend(slab(0, 30, 30, 32, 0));
        cells.extend(slab(24, 26, 8, 30, 0));
        let plg = graph_with_nodes(&cells, &[(5, 1, 0), (25, 1, 0), (2, 31, 0), (25, 9, 0)]);
        let path = plan_with(
            &plg,
            &ColumnIz::default(),
            (0.55, 0.05, 0.1),
            (0.25, 3.15, 0.1),
        )
        .unwrap();
        let end = path.waypoints.last().unwrap();
        assert!(
            (end.0 - 2.55).abs() < 2.0 * VOXEL && end.1 < 0.35,
            "path must end on the strip under the arm tip: {end:?}"
        );
        let (_, aim) = path.bridge.expect("best-effort plan carries a bridge");
        assert!(
            (aim.0 - 2.55).abs() < 2.0 * VOXEL && (aim.1 - 0.85).abs() < 2.0 * VOXEL,
            "bridge must aim at the arm tip: {aim:?}"
        );
    }

    #[test]
    fn plan_refuses_a_gap_beyond_the_hop_cap() {
        // 2.5 m of nothing between the components: no admissible bridge, so
        // the plan reports no path instead of marching at the void.
        let mut cells = slab(0, 5, 0, 4, 0);
        cells.extend(slab(30, 35, 0, 4, 0));
        let plg = graph_with_nodes(&cells, &[(2, 2, 0), (32, 2, 0)]);
        assert!(plan_simple(&plg, (0.25, 0.25, 0.1), (3.25, 0.25, 0.1)).is_none());
    }

    #[test]
    fn plan_refuses_a_vertical_bridge_through_a_slab() {
        // The goal floor sits 0.8 m directly above the start floor. Every
        // candidate hop is nearly vertical and fails the grade gate.
        let mut cells = slab(0, 10, 0, 6, 0);
        cells.extend(slab(0, 10, 0, 6, 8));
        let plg = graph_with_nodes(&cells, &[(5, 3, 0), (5, 3, 8)]);
        assert!(plan_simple(&plg, (0.25, 0.35, 0.1), (0.85, 0.35, 0.95)).is_none());
    }

    #[test]
    fn plan_chains_across_sparse_treads() {
        // Sparse stair fragments climb from the floor lip toward the goal
        // platform: hops of 0.4-0.5 m each, rising 0.3 m. The chain scores the
        // route and the bridge is its first hop, aimed at the first tread.
        let mut cells = slab(0, 20, 0, 4, 0);
        cells.extend(slab(25, 27, 1, 3, 3));
        cells.extend(slab(32, 34, 1, 3, 6));
        cells.extend(slab(39, 41, 1, 3, 9));
        cells.extend(slab(46, 60, 0, 4, 12));
        let plg = graph_with_nodes(
            &cells,
            &[(5, 2, 0), (26, 2, 3), (33, 2, 6), (40, 2, 9), (53, 2, 12)],
        );
        let path = plan_with(
            &plg,
            &ColumnIz::default(),
            (0.25, 0.25, 0.1),
            (5.5, 0.25, 1.3),
        )
        .unwrap();
        let (near, aim) = path.bridge.expect("best-effort plan carries a bridge");
        assert!(
            (near.0 - 2.05).abs() < 2.0 * VOXEL,
            "bridge starts at the floor lip: {near:?}"
        );
        assert!(
            (aim.0 - 2.55).abs() < 2.0 * VOXEL && (aim.2 - 0.4).abs() < 1.5 * VOXEL,
            "bridge aims at the first tread: {aim:?}"
        );
    }

    #[test]
    fn plan_refuses_a_ladder_of_crumbs_up_a_wall() {
        // Fragments big enough to pass the landing-size gate, stacked so each
        // hop sneaks under the per-hop grade but the chain sums to a wall.
        let mut cells = slab(0, 10, 0, 4, 0);
        cells.extend(slab(12, 13, 1, 2, 4));
        cells.extend(slab(15, 16, 1, 2, 8));
        cells.extend(slab(18, 19, 1, 2, 12));
        cells.extend(slab(21, 31, 0, 4, 16));
        let plg = graph_with_nodes(
            &cells,
            &[(5, 2, 0), (12, 1, 4), (15, 1, 8), (18, 1, 12), (26, 2, 16)],
        );
        assert!(
            plan_simple(&plg, (0.25, 0.25, 0.1), (2.65, 0.25, 1.7)).is_none(),
            "a chain as steep as a wall must be refused"
        );
    }

    #[test]
    fn plan_bridges_across_a_sub_clearance_pinch() {
        // Two passable slabs joined by a measured 1-wide strip whose edges
        // the wall penalty made impassable: one adjacency component with no
        // passable route. The bridge must run along the strip, ending at its
        // near side and aiming at the far side, not give up.
        let mut cells = slab(0, 15, 0, 4, 0);
        cells.extend(slab(16, 24, 2, 2, 0));
        cells.extend(slab(25, 40, 0, 4, 0));
        let mut plg = graph_with_nodes(&cells, &[(5, 2, 0), (33, 2, 0)]);
        let pinch_ids: AHashSet<CellId> = plg
            .cells
            .ids()
            .filter(|&id| (16..=24).contains(&plg.cells.coord(id).0))
            .collect();
        let all: Vec<CellId> = plg.cells.ids().collect();
        for id in all {
            let src_in = pinch_ids.contains(&id);
            for e in plg.cells.edges_mut(id) {
                if src_in || pinch_ids.contains(&e.dest) {
                    e.cost = f32::INFINITY;
                }
            }
        }
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
            &mut plg.node_adj,
        );

        let path = plan_with(
            &plg,
            &ColumnIz::default(),
            (0.25, 0.25, 0.1),
            (3.75, 0.25, 0.1),
        )
        .unwrap();
        let (near, aim) = path.bridge.expect("pinch plan carries a bridge");
        assert!(
            (near.0 - 1.55).abs() < 2.0 * VOXEL,
            "bridge starts at the pinch's near side: {near:?}"
        );
        assert!(
            (aim.0 - 2.55).abs() < 2.0 * VOXEL,
            "bridge aims at the pinch's far side: {aim:?}"
        );
        let end = path.waypoints.last().unwrap();
        assert!(
            (end.0 - near.0).abs() < 1e-4,
            "path ends at the bridge near side: {end:?}"
        );
    }

    #[test]
    fn ribbon_aim_is_capped_on_a_long_pinch() {
        // A 3.5 m sub-clearance strip: the aim must sit at the ~2 m lookahead
        // into the ribbon, not at its far end, so the bridge claim stays
        // local to what the robot is about to attempt.
        let mut cells = slab(0, 15, 0, 4, 0);
        cells.extend(slab(16, 50, 2, 2, 0));
        cells.extend(slab(51, 70, 0, 4, 0));
        let mut plg = graph_with_nodes(&cells, &[(5, 2, 0), (60, 2, 0)]);
        let pinch_ids: AHashSet<CellId> = plg
            .cells
            .ids()
            .filter(|&id| (16..=50).contains(&plg.cells.coord(id).0))
            .collect();
        let all: Vec<CellId> = plg.cells.ids().collect();
        for id in all {
            let src_in = pinch_ids.contains(&id);
            for e in plg.cells.edges_mut(id) {
                if src_in || pinch_ids.contains(&e.dest) {
                    e.cost = f32::INFINITY;
                }
            }
        }
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
            &mut plg.node_adj,
        );

        let path = plan_with(
            &plg,
            &ColumnIz::default(),
            (0.25, 0.25, 0.1),
            (6.55, 0.25, 0.1),
        )
        .unwrap();
        let (near, aim) = path.bridge.expect("long pinch carries a bridge");
        assert!(
            (near.0 - 1.55).abs() < 2.0 * VOXEL,
            "bridge starts at the pinch's near side: {near:?}"
        );
        let reach = aim.0 - near.0;
        assert!(
            (1.8..=2.4).contains(&reach),
            "aim must sit at the ~2 m lookahead, not the far end: reach={reach}, aim={aim:?}"
        );
    }

    #[test]
    fn plan_refuses_all_optimism_when_bridging_is_disabled() {
        let mut config = test_config();
        config.bridge_max_hop_m = 0.0;
        let state = BridgeState::default();

        // A short gap that would otherwise bridge.
        let mut cells = slab(0, 30, 0, 4, 0);
        cells.extend(slab(38, 46, 0, 4, 0));
        let plg = graph_with_nodes(&cells, &[(5, 2, 0), (42, 2, 0)]);
        assert!(plan(
            &plg,
            &ColumnIz::default(),
            (0.25, 0.25, 0.1),
            (4.25, 0.25, 0.1),
            &config,
            Some(&state),
        )
        .is_none());

        // An unseen goal that would otherwise get a proxy best-effort path.
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        assert!(plan(
            &plg,
            &ColumnIz::default(),
            (0.2, 0.0, 0.05),
            (5.0, 0.0, 0.05),
            &config,
            Some(&state),
        )
        .is_none());
    }

    #[test]
    fn stateless_plan_skips_optimism_but_still_plans_full_paths() {
        let config = test_config();

        // Connected goal: the stateless query still returns the full path.
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let full = plan(
            &plg,
            &ColumnIz::default(),
            (0.2, 0.0, 0.05),
            (1.7, 0.0, 0.05),
            &config,
            None,
        );
        assert!(full.is_some_and(|p| p.bridge.is_none()));

        // Unseen goal: no proxy path without a bridge state.
        assert!(plan(
            &plg,
            &ColumnIz::default(),
            (0.2, 0.0, 0.05),
            (5.0, 0.0, 0.05),
            &config,
            None,
        )
        .is_none());

        // Disconnected goal: no bridge without a bridge state.
        let mut cells = slab(0, 30, 0, 4, 0);
        cells.extend(slab(38, 46, 0, 4, 0));
        let plg = graph_with_nodes(&cells, &[(5, 2, 0), (42, 2, 0)]);
        assert!(plan(
            &plg,
            &ColumnIz::default(),
            (0.25, 0.25, 0.1),
            (4.25, 0.25, 0.1),
            &config,
            None,
        )
        .is_none());
    }

    #[test]
    fn held_bridge_wins_over_a_marginally_cheaper_rival() {
        // Two stepping-stone islands toward the goal island, far enough
        // apart to be distinct bridges. A is larger, so its chain carries
        // more evidence and scores marginally cheaper. A fresh state picks
        // A. A state holding B keeps B, since A wins by less than the
        // switch margin.
        let stone_a = slab(24, 26, 0, 4, 0);
        let stone_b = slab(24, 26, 12, 14, 0);
        let island = slab(32, 40, 0, 16, 0);
        let mut both = slab(0, 20, 0, 16, 0);
        both.extend(stone_a.clone());
        both.extend(stone_b.clone());
        both.extend(island.clone());
        let nodes = [(5, 8, 0), (36, 8, 0)];
        let plg = graph_with_nodes(&both, &nodes);
        let start = (0.55, 0.85, 0.1);
        let goal = (3.65, 0.85, 0.1);

        let fresh = plan_with(&plg, &ColumnIz::default(), start, goal).unwrap();
        let (_, fresh_aim) = fresh.bridge.expect("disconnected goal bridges");
        assert!(
            fresh_aim.1 < 0.5,
            "fresh state picks the larger stone: {fresh_aim:?}"
        );

        // Adopt stone B on a map where it is the only route.
        let mut b_only = slab(0, 20, 0, 16, 0);
        b_only.extend(stone_b);
        b_only.extend(island);
        let plg_b = graph_with_nodes(&b_only, &nodes);
        let adopted = plan_with(&plg_b, &ColumnIz::default(), start, goal).unwrap();
        let bridge_b = adopted.bridge.expect("single stone bridges");
        assert!(bridge_b.1 .1 > 0.5, "stone B sits at high y: {bridge_b:?}");
        let mut state = BridgeState::default();
        state.note_bridge(bridge_b, start);

        let held = plan(
            &plg,
            &ColumnIz::default(),
            start,
            goal,
            &test_config(),
            Some(&state),
        )
        .unwrap();
        let (_, held_aim) = held.bridge.expect("held state still bridges");
        assert!(
            held_aim.1 > 0.5,
            "held bridge must not flap to the marginally cheaper rival: {held_aim:?}"
        );
    }

    #[test]
    fn plan_refuses_a_bridge_through_a_wall() {
        // Two slabs 0.7 m apart with an occupied wall column between them.
        // The chord has no body clearance, so the pair is inadmissible.
        let mut cells = slab(0, 30, 0, 4, 0);
        cells.extend(slab(38, 46, 0, 4, 0));
        let plg = graph_with_nodes(&cells, &[(5, 2, 0), (42, 2, 0)]);
        let mut by_col = ColumnIz::default();
        for y in -2..7 {
            by_col.insert((34, y), (0..20).collect());
        }
        assert!(
            plan_with(&plg, &by_col, (0.25, 0.25, 0.1), (4.25, 0.25, 0.1)).is_none(),
            "a bridge through an occupied wall must be refused"
        );
    }

    #[test]
    fn plan_reaches_toward_an_unseen_goal() {
        // Goal well past the mapped strip and outside the snap radius. The
        // path must end at the mapped cell nearest the goal.
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan_simple(&plg, (0.2, 0.0, 0.05), (5.0, 0.0, 0.05)).unwrap();
        let edge = surface_point_xyz(19, 0, 0, VOXEL);
        let end = wp.last().unwrap();
        assert!(
            (end.0 - edge.0).abs() < 1e-5 && (end.1 - edge.1).abs() < 1e-5,
            "path must end at the map edge nearest the goal: {end:?}"
        );
        assert!(
            wp.windows(2).all(|p| p[1].0 >= p[0].0 - 1e-4),
            "walked backward"
        );
    }

    #[test]
    fn plan_traces_surface_from_pose_to_first_node() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan_simple(&plg, (0.2, 0.0, 0.05), (1.7, 0.0, 0.05)).unwrap();
        // The path leaves from the robot's own cell, not a jump ahead, and
        // walks x monotonically to the goal.
        assert!(
            (wp[1].0 - 0.2).abs() < 2.0 * VOXEL,
            "jumped ahead: {:?}",
            wp[1]
        );
        assert!(
            wp.windows(2).all(|p| p[1].0 >= p[0].0 - 1e-4),
            "walked backward"
        );
    }

    #[test]
    fn plan_lead_in_does_not_backtrack_to_region_node() {
        // Robot at cell 5 is in node 3's region but sits between it and node 15.
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan_simple(&plg, (0.55, 0.0, 0.05), (1.7, 0.0, 0.05)).unwrap();
        let xs: Vec<i32> = wp[1..wp.len() - 1]
            .iter()
            .map(|w| (w.0 / VOXEL).floor() as i32)
            .collect();
        assert!(
            *xs.first().unwrap() >= 5,
            "backtracked to the region node: {xs:?}"
        );
        assert!(
            xs.windows(2).all(|p| p[1] >= p[0]),
            "lead-in walked backward: {xs:?}"
        );
    }

    fn waypoint_key(w: &(f32, f32, f32)) -> VoxelKey {
        (
            (w.0 / VOXEL).floor() as i32,
            (w.1 / VOXEL).floor() as i32,
            (w.2 / VOXEL).round() as i32 - 1,
        )
    }

    #[test]
    fn plan_path_segments_stay_on_the_surface() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let wp = plan_simple(&plg, (0.2, 0.0, 0.05), (1.9, 0.0, 0.05)).unwrap();
        // Smoothed waypoints are no longer cell-adjacent, but each segment
        // between them must still stay on the surface.
        let step_cells = (0.25f32 / VOXEL).floor() as i32;
        for w in &wp[1..wp.len() - 1] {
            assert!(
                plg.cells.id(waypoint_key(w)).is_some(),
                "waypoint {w:?} is off the surface"
            );
        }
        let wc = WallCost {
            clearance_m: 0.2,
            buffer_m: 0.5,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
        };
        for pair in wp[1..wp.len() - 1].windows(2) {
            assert!(
                segment_metrics(
                    &plg,
                    waypoint_key(&pair[0]),
                    waypoint_key(&pair[1]),
                    step_cells,
                    &wc
                )
                .is_some(),
                "segment {:?} -> {:?} leaves the surface",
                pair[0],
                pair[1]
            );
        }
    }

    #[test]
    fn string_pull_straightens_open_area() {
        // Filled rectangle: every straight segment is on-surface, so the diagonal
        // path collapses instead of staircasing through the nodes.
        let mut cells: Vec<VoxelKey> = Vec::new();
        for x in 0..10 {
            for y in 0..6 {
                cells.push((x, y, 0));
            }
        }
        let plg = graph_with_nodes(&cells, &[(2, 2, 0), (7, 3, 0)]);
        let wp = plan_simple(&plg, (0.05, 0.05, 0.05), (0.85, 0.55, 0.05)).unwrap();
        let length: f32 = wp
            .windows(2)
            .map(|w| ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt())
            .sum();
        let direct = (0.8f32.powi(2) + 0.5f32.powi(2)).sqrt();
        assert!(
            length <= direct * 1.2,
            "path not straightened: length {length} vs direct {direct}"
        );
    }
    #[test]
    fn string_pull_refuses_shortcut_through_sub_clearance_cell() {
        // Straight strip: with open clearance the run collapses to its
        // endpoints. Drop one mid cell below the hard clearance and the shortcut
        // spanning it is refused, so the smoothed path retains that cell.
        let mut plg = PlannerGraph::new();
        build_surface_lookup(&strip(10), &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        let path: Vec<CellId> = (0..10).map(|x| plg.cells.id((x, 0, 0)).unwrap()).collect();

        let wc = WallCost {
            clearance_m: 0.2,
            buffer_m: 0.5,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
        };
        plg.wall_state.dist = vec![f32::INFINITY; plg.cells.slot_capacity()];
        let open = string_pull(&plg, &path, 1, &wc);
        assert_eq!(open.len(), 2, "open strip should collapse to its endpoints");

        let mid = plg.cells.id((5, 0, 0)).unwrap();
        plg.wall_state.dist[mid as usize] = 0.1; // below the 0.2 clearance
        let guarded = string_pull(&plg, &path, 1, &wc);
        assert!(
            guarded.len() > 2,
            "shortcut across a sub-clearance cell must be refused: {guarded:?}"
        );
        assert!(
            guarded.contains(&mid),
            "smoothed path must still traverse the low-clearance cell"
        );
    }

    #[test]
    fn select_entry_connects_straight_to_in_radius_goal() {
        // The only node is behind the robot and the goal is just ahead.
        // Entry must connect straight to the goal, not dogleg via the node.
        let plg = graph_with_nodes(&strip(20), &[(2, 0, 0)]);
        let start = plg.cells.id((10, 0, 0)).unwrap();
        let goal = plg.cells.id((15, 0, 0)).unwrap();
        let goal_node = plg.nodes[0].cell_id;
        let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
        let (ctg, pred) = node_dijkstra(&plg, goal_node);

        let (lead, node_seq) = select_entry(
            &plg,
            &[start],
            goal,
            goal_node,
            &ctg,
            &pred,
            &node_cells,
            3.0,
        )
        .unwrap();

        assert!(
            node_seq.is_empty(),
            "endgame routed via nodes: {node_seq:?}"
        );
        assert_eq!(lead.first(), Some(&start));
        assert_eq!(lead.last(), Some(&goal));
        let xs: Vec<i32> = lead.iter().map(|&c| plg.cells.coord(c).0).collect();
        assert!(
            xs.windows(2).all(|p| p[1] >= p[0]),
            "lead-in walked backward: {xs:?}"
        );
    }

    #[test]
    fn plan_enters_on_goalward_node_not_nearest() {
        // Robot sits past node 2 toward the goal. Entry must skip it for node 10.
        let plg = graph_with_nodes(&strip(20), &[(2, 0, 0), (10, 0, 0)]);
        let wp = plan_simple(&plg, (0.45, 0.0, 0.05), (1.25, 0.0, 0.05)).unwrap();
        let nearest = surface_point_xyz(2, 0, 0, VOXEL);
        assert!(
            !wp.iter().any(|w| (w.0 - nearest.0).abs() < 1e-5),
            "path doubled back to the nearest node: {wp:?}"
        );
        let xs: Vec<i32> = wp[1..wp.len() - 1]
            .iter()
            .map(|w| (w.0 / VOXEL).floor() as i32)
            .collect();
        assert!(
            xs.windows(2).all(|p| p[1] >= p[0]),
            "path stepped backward: {xs:?}"
        );
    }

    #[test]
    fn back_off_tail_trims_from_the_goal_end() {
        let path = vec![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
        ];
        // Trim within the last segment.
        assert_eq!(*back_off_tail(&path, 0.5).last().unwrap(), (2.5, 0.0, 0.0));
        // Trim exactly to a vertex without leaving a duplicate point.
        let to_vertex = back_off_tail(&path, 1.0);
        assert_eq!(*to_vertex.last().unwrap(), (2.0, 0.0, 0.0));
        assert_eq!(to_vertex.len(), 3);
        // Trimming more than the path length stops (empty).
        assert!(back_off_tail(&path, 5.0).is_empty());
    }

    #[test]
    fn snap_picks_in_column_cell() {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&strip(20), &mut lookup);
        let cell = snap_pose_to_cell(&lookup, (0.5, 0.0, 0.1), VOXEL, Z_TOL).unwrap();
        assert_eq!(cell, (5, 0, 0));
    }

    #[test]
    fn snap_falls_back_to_nearby_column() {
        let mut cells = strip(20);
        cells.retain(|c| c.0 != 2);
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&cells, &mut lookup);
        let cell = snap_pose_to_cell(&lookup, (0.25, 0.0, 0.1), VOXEL, Z_TOL).unwrap();
        assert!(cell == (1, 0, 0) || cell == (3, 0, 0));
    }

    #[test]
    fn snap_rejects_outside_z_tolerance() {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&strip(20), &mut lookup);
        assert!(snap_pose_to_cell(&lookup, (0.5, 0.0, 2.0), VOXEL, 1.5).is_none());
    }

    #[test]
    fn segment_metrics_rejects_vertical_chord() {
        let plg = PlannerGraph::new();
        let wc = WallCost {
            clearance_m: 0.2,
            buffer_m: 0.3,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
        };
        assert!(segment_metrics(&plg, (5, 5, 0), (5, 5, 4), 2, &wc).is_none());
        assert_eq!(
            segment_metrics(&plg, (5, 5, 0), (5, 5, 0), 2, &wc),
            Some((1.0, 0.0))
        );
    }
}
