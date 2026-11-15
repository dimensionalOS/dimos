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

use std::collections::{BinaryHeap, VecDeque};

use ahash::{AHashMap, AHashSet};

use crate::adjacency::{CellId, SurfaceCells, SurfaceLookup};
use crate::dijkstra::{walk_preds, Scored};
use crate::edges::PlannerGraph;
use crate::lazy::Version;
use crate::mls_planner::{ms_since, Config};
use crate::nodes::penalty_of;
use crate::nodes::{NodeId, Nodes, NO_NODE};
use crate::terrain::{climb, within_step};
use crate::voxel::{surface_point_xyz, VoxelKey};

/// Robot-rooted candidate search radius, in multiples of node spacing.
const CANDIDATE_RADIUS_FACTOR: f32 = 3.0;

/// Horizontal search radius when snapping a pose to the surface.
pub(crate) const SNAP_SEARCH_RADIUS_M: f32 = 1.5;

/// Max snap candidates tried when connecting the start.
const MAX_SNAP_ATTEMPTS: usize = 64;

/// On a blocked path, stop this far short of the last traversable point.
const BEST_EFFORT_DISTANCE_M: f32 = 1.0;

/// World-frame waypoints paired with the string-pulled cell path that produced
/// them. The cell path is cached for later safe truncation.
type PlannedPath = (Vec<(f32, f32, f32)>, Vec<VoxelKey>);

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

/// Where a stale layer's cell is now, None for a current-layer cell.
pub type StalePoint<'a> = &'a dyn Fn(CellId) -> Option<(f32, f32, f32)>;

/// The stale layers as a plan sees them: where a stale cell is now, and the stale cell nearest a goal no current-layer cell is near.
#[derive(Clone, Copy)]
pub struct Stale<'a> {
    pub point: StalePoint<'a>,
    pub nearest: &'a dyn Fn((f32, f32, f32)) -> Option<CellId>,
}

impl Stale<'_> {
    /// No stale layers.
    pub const NONE: Stale<'static> = Stale {
        point: &|_| None,
        nearest: &|_| None,
    };
}

/// Plan path from start pose to goal pose using the node graph.
/// Returns the waypoints and the string-pulled cell path, or none if either
/// pose can't be snapped to surface or there is no valid path.
pub fn plan(
    plg: &PlannerGraph,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    config: &Config,
    stale: Stale<'_>,
) -> Option<PlannedPath> {
    plan_reusing(plg, start_pose, goal_pose, config, stale, None).map(|(path, _)| path)
}

/// plan, reusing `reuse`'s cached route: every stretch of it the new cell path runs along that no change reached
/// since, a slack clear of the robot and of what changed, keeps its pulled cells and smoothed points, and only the
/// stretches between are pulled and smoothed and spliced in, so a replan costs what changed, not the route's length.
pub fn plan_reusing(
    plg: &PlannerGraph,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    config: &Config,
    stale: Stale<'_>,
    reuse: Option<(&RouteCache, &RouteChanges)>,
) -> Option<(PlannedPath, RouteCache)> {
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
    // The goal snaps in the current layer, or else to the stale cell standing nearest it now.
    let goal_cell = snap_pose_to_cell(&plg.surface_lookup, goal_pose, voxel_size, z_tolerance_m)
        .and_then(|k| plg.cells.id(k))
        .or_else(|| (stale.nearest)(goal_pose));
    let Some(goal_cell) = goal_cell else {
        tracing::debug!(
            ?goal_pose,
            "plan failed: goal does not snap to any surface cell"
        );
        return None;
    };
    let goal_coord = plg.cells.coord(goal_cell);
    let start_cells: Vec<CellId> = start_candidates
        .iter()
        .take(MAX_SNAP_ATTEMPTS)
        .filter_map(|&candidate| plg.cells.id(candidate))
        .collect();

    let node_cells = &plg.nodes;

    // The penalized Voronoi cannot own sub-clearance goals, so fall back to the
    // nearest node by hops. A real failure then reports as disconnected below.
    let mut goal_segment = walk_live_preds(plg, goal_cell);
    let mut goal_node = *goal_segment
        .last()
        .expect("the goal cell is live, so the walk keeps at least it");
    if !node_cells.has(goal_node) {
        let Some((node, path)) = nearest_node(&plg.cells, goal_cell, node_cells) else {
            tracing::debug!(
                ?goal_coord,
                "plan failed: goal's connected component has no graph node"
            );
            return None;
        };
        goal_node = node;
        goal_segment = path;
    }

    let began = std::time::Instant::now();
    let radius = (config.node_spacing_m * CANDIDATE_RADIUS_FACTOR).max(voxel_size);
    // Nearest candidate first so the lead-in anchors at the robot's own cell.
    // The rest are only a fallback when it cannot connect.
    let (near, rest) = start_cells.split_at(start_cells.len().min(1));
    let searches = [near, rest].map(|group| robot_search(&plg.cells, group, radius));
    // Rooted at the goal, and run only until every node an entry could take is settled, so its cost is the
    // stretch of map out to the robot rather than all of it.
    let mut entries: AHashSet<NodeId> = searches
        .iter()
        .flat_map(|(dist, _)| dist.keys().copied())
        .filter(|&c| node_cells.has(c))
        .collect();
    entries.extend(
        start_cells
            .iter()
            .filter_map(|&c| walk_live_preds(plg, c).last().copied())
            .filter(|&c| node_cells.has(c)),
    );
    let (cost_to_go, pred_to_goal) = node_dijkstra(plg, goal_node, &entries);
    let search_ms = ms_since(began);
    let entry = [near, rest]
        .into_iter()
        .zip(&searches)
        .find_map(|(group, search)| {
            select_entry_from(
                plg,
                group,
                search,
                goal_cell,
                goal_node,
                &cost_to_go,
                &pred_to_goal,
                node_cells,
            )
        });
    let Some((lead_in, node_seq)) = entry else {
        tracing::debug!(
            candidates = start_cells.len(),
            reachable_nodes = cost_to_go.len(),
            total_nodes = plg.nodes.len(),
            "plan failed: start and goal lie on separate connected surface components",
        );
        return None;
    };

    let limits = PathLimits::from_config(config);
    // A direct goal connection carries the whole route in its lead-in.
    let goal_segment: &[CellId] = if node_seq.is_empty() {
        &[]
    } else {
        &goal_segment
    };
    let cells = assemble_cells(plg, &node_seq, &lead_in, goal_segment);
    let entry_ms = ms_since(began) - search_ms;
    let stage = std::time::Instant::now();
    let kept = match reuse {
        Some((cache, changes)) if cache.goal_pose == goal_pose => {
            kept_stretches(plg, cache, changes, &cells, config)
        }
        _ => Vec::new(),
    };
    let align_ms = ms_since(stage);
    let stage = std::time::Instant::now();
    let fresh = Fresh {
        plg,
        cells: &cells,
        limits: &limits,
        stale_point: stale.point,
        step_weight: config.step_penalty_weight,
    };
    let mut route = Spliced::default();
    // Pull and smooth from the robot to the first kept stretch, between kept stretches, and on to the goal past the last.
    let (mut from, mut first) = (0, End::Pose(start_pose));
    let (mut to_goal, mut kept_cells) = (false, 0);
    for (k, cache) in kept
        .iter()
        .zip(reuse.map(|(cache, _)| cache).into_iter().cycle())
    {
        let span = cache.pulled_at[k.last] - cache.pulled_at[k.first];
        let join = cache.points[cache.point_of_pulled[k.first]];
        route.append(fresh.piece(from, k.at, first, End::Join(join)));
        route.append(cache.stretch(k));
        kept_cells += span;
        to_goal = k.to_goal;
        if to_goal {
            break;
        }
        from = k.at + span;
        first = End::Join(cache.points[cache.point_of_pulled[k.last]]);
    }
    if !to_goal {
        route.append(fresh.piece(from, cells.len() - 1, first, End::Pose(goal_pose)));
    }
    let splice_ms = ms_since(stage);
    tracing::debug!(
        search_ms,
        entry_ms,
        align_ms,
        splice_ms,
        assembled = cells.len(),
        kept_cells,
        kept_stretches = kept.len(),
        smoothed = route.smoothed,
        points = route.points.len(),
        "plan stages"
    );
    let path_cells: Vec<VoxelKey> = route
        .pulled
        .iter()
        .map(|&i| plg.cells.coord(cells[i]))
        .collect();
    let layers = cells.iter().map(|&id| plg.cells.layer(id)).collect();
    let mut cell_at = AHashMap::with_capacity(cells.len());
    for (i, &id) in cells.iter().enumerate() {
        cell_at.insert(id, i);
    }
    let cache = RouteCache {
        goal_pose,
        assembled: cells,
        layers,
        cell_at,
        pulled_at: route.pulled,
        points: route.points.clone(),
        point_of_pulled: route.point_of_pulled,
        smoothed: route.smoothed,
        kept_cells,
    };
    Some(((route.points, path_cells), cache))
}

/// A full plan's route, kept so the next plan to the same goal pulls and smooths only what changed: the assembled
/// cell path with each cell's layer then and index by id, which of its cells the pull kept, and the smoothed points
/// with where each kept cell sits among them.
#[derive(Clone, Debug)]
pub struct RouteCache {
    pub(crate) goal_pose: (f32, f32, f32),
    pub(crate) assembled: Vec<CellId>,
    layers: Vec<Version>,
    pub(crate) cell_at: AHashMap<CellId, usize>,
    pub(crate) pulled_at: Vec<usize>,
    pub(crate) points: Vec<(f32, f32, f32)>,
    point_of_pulled: Vec<usize>,
    /// Points this plan put through the smoother; the rest it took from the route before.
    pub(crate) smoothed: usize,
    /// Assembled cells this plan took, pulled and smoothed, from the route before.
    pub(crate) kept_cells: usize,
}

impl RouteCache {
    /// The assembled indices whose cell is in a stale layer, the cells a closure carries.
    pub(crate) fn stale_cells(&self, current_layer: Version) -> impl Iterator<Item = usize> + '_ {
        (0..self.layers.len()).filter(move |&i| self.layers[i] != current_layer)
    }

    /// A kept stretch's points and pulled cells, as a piece starting on its first pulled cell.
    fn stretch(&self, k: &Kept) -> Piece {
        let first_point = self.point_of_pulled[k.first];
        let end = if k.to_goal {
            self.points.len()
        } else {
            self.point_of_pulled[k.last] + 1
        };
        let base = self.pulled_at[k.first];
        Piece {
            at: k.at,
            points: self.points[first_point..end].to_vec(),
            pulled: self.pulled_at[k.first..=k.last]
                .iter()
                .map(|&i| i - base)
                .collect(),
            point_of_pulled: self.point_of_pulled[k.first..=k.last]
                .iter()
                .map(|&i| i - first_point)
                .collect(),
            smoothed: 0,
        }
    }
}

/// What changed on a cached route since it was planned: per assembled cell and per smoothed point, whether a change
/// reached it. Empty is unchanged.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RouteChanges {
    pub cells: Vec<bool>,
    pub points: Vec<bool>,
}

impl RouteChanges {
    /// No change, on a route of `cells` assembled cells and `points` points.
    pub fn none(cells: usize, points: usize) -> Self {
        Self {
            cells: vec![false; cells],
            points: vec![false; points],
        }
    }
}

/// A stretch of the cached route kept as it was: its first and last pulled cells (indices into the cache's pulled
/// cells), where its first sits in the new cell path, and whether it runs on to the goal.
#[derive(Clone, Copy, Debug)]
struct Kept {
    first: usize,
    last: usize,
    at: usize,
    to_goal: bool,
}

/// The cached route's stretches a new cell path can keep: the same cells in the same order, none of them or the
/// points over them reached by a change, dead or in another layer since, all at least route_reuse_slack_m along the
/// new path from the robot and from anything that is not kept, and spanning two or more pulled cells.
fn kept_stretches(
    plg: &PlannerGraph,
    cache: &RouteCache,
    changes: &RouteChanges,
    cells: &[CellId],
    config: &Config,
) -> Vec<Kept> {
    let n = cache.assembled.len();
    // cached cells that cannot be kept: changed, dead, moved layer, or under a changed point's chord
    let mut bad: Vec<bool> = (0..n)
        .map(|i| {
            let id = cache.assembled[i];
            changes.cells.get(i).copied().unwrap_or(false)
                || !plg.cells.is_live(id)
                || plg.cells.layer(id) != cache.layers[i]
        })
        .collect();
    for (t, _) in changes.points.iter().enumerate().filter(|(_, &c)| c) {
        // the chord pulled k -> k + 1 holds points point_of_pulled[k] ..= point_of_pulled[k + 1]
        let k = cache
            .point_of_pulled
            .partition_point(|&p| p <= t)
            .saturating_sub(1);
        let from = cache.pulled_at[k];
        let to = cache.pulled_at.get(k + 1).copied().unwrap_or(n - 1);
        bad[from..=to].iter_mut().for_each(|b| *b = true);
    }
    // each new cell's cached index, and whether it continues the cached path from the cell before
    let at: Vec<Option<usize>> = cells
        .iter()
        .map(|id| cache.cell_at.get(id).copied())
        .collect();
    let follows =
        |j: usize| j > 0 && matches!((at[j - 1], at[j]), (Some(a), Some(b)) if b == a + 1);
    let slack = (config.route_reuse_slack_m / config.voxel_size).ceil() as usize;
    let mut fresh: Vec<bool> = (0..cells.len())
        .map(|j| j < slack || at[j].is_none_or(|i| bad[i]))
        .collect();
    // grow what is planned fresh by the slack either way, so the smoothing settles before it meets what is kept
    let seeds: Vec<usize> = (0..cells.len()).filter(|&j| fresh[j]).collect();
    let mut grown = fresh.clone();
    let mut reach_until = 0usize;
    for &j in &seeds {
        let lo = j.saturating_sub(slack).max(reach_until);
        let hi = (j + slack).min(cells.len() - 1);
        grown[lo..=hi].iter_mut().for_each(|g| *g = true);
        reach_until = hi;
    }
    fresh = grown;
    let mut kept = Vec::new();
    let mut j = 0;
    while j < cells.len() {
        if fresh[j] {
            j += 1;
            continue;
        }
        let a = j;
        j += 1;
        while j < cells.len() && !fresh[j] && follows(j) {
            j += 1;
        }
        let (lo, hi) = (at[a].unwrap(), at[j - 1].unwrap());
        let first = cache.pulled_at.partition_point(|&i| i < lo);
        let Some(last) = cache.pulled_at.partition_point(|&i| i <= hi).checked_sub(1) else {
            continue;
        };
        if first >= last {
            continue;
        }
        let at_first = a + cache.pulled_at[first] - lo;
        let to_goal = last == cache.pulled_at.len() - 1
            && cache.pulled_at[last] == n - 1
            && at_first + cache.pulled_at[last] - cache.pulled_at[first] == cells.len() - 1;
        kept.push(Kept {
            first,
            last,
            at: at_first,
            to_goal,
        });
    }
    kept
}

/// How a pulled and smoothed piece ends: on a pose before or after its cells (the robot, the goal), or on a kept
/// stretch's point in place of its end cell's, held fixed.
#[derive(Clone, Copy)]
enum End {
    Pose((f32, f32, f32)),
    Join((f32, f32, f32)),
}

/// Part of a route: its points, its pulled cells as offsets from cell `at` of the new path, and where each sits among the points.
struct Piece {
    at: usize,
    points: Vec<(f32, f32, f32)>,
    pulled: Vec<usize>,
    point_of_pulled: Vec<usize>,
    smoothed: usize,
}

/// What a fresh piece of route is pulled and smoothed over.
struct Fresh<'a> {
    plg: &'a PlannerGraph,
    cells: &'a [CellId],
    limits: &'a PathLimits,
    stale_point: StalePoint<'a>,
    step_weight: f32,
}

impl Fresh<'_> {
    /// String-pull cells `from ..= to` per current-layer run, then smooth each current run of the points.
    fn piece(&self, from: usize, to: usize, first: End, last: End) -> Piece {
        let (plg, voxel_size) = (self.plg, self.limits.voxel_size);
        let cells = &self.cells[from..=to];
        let current = |id: CellId| plg.cells.layer(id) == plg.cells.current_layer();
        let mut pulled: Vec<usize> = Vec::with_capacity(cells.len());
        let mut run_start = 0;
        for run in cells.chunk_by(|&a, &b| current(a) == current(b)) {
            if current(run[0]) {
                pulled.extend(
                    string_pull_at(plg, run, self.limits)
                        .into_iter()
                        .map(|i| run_start + i),
                );
            } else {
                pulled.extend(run_start..run_start + run.len());
            }
            run_start += run.len();
        }
        let mut points: Vec<((f32, f32, f32), bool)> = Vec::with_capacity(pulled.len() + 2);
        if let End::Pose(p) = first {
            points.push((p, true));
        }
        let lead = points.len();
        for &i in &pulled {
            match (self.stale_point)(cells[i]) {
                Some(p) => points.push((p, false)),
                None => points.push((plg.cells.point(cells[i], voxel_size), true)),
            }
        }
        if let End::Join(p) = first {
            points[0].0 = p;
        }
        match last {
            End::Pose(p) => points.push((p, true)),
            End::Join(p) => points.last_mut().expect("a piece has a cell").0 = p,
        }
        let mut out: Vec<(f32, f32, f32)> = Vec::with_capacity(points.len());
        // where each of `points` landed among the smoothed points
        let mut landed: Vec<usize> = Vec::with_capacity(points.len());
        let mut smoothed = 0;
        for run in points.chunk_by(|a, b| a.1 == b.1) {
            let run_points: Vec<(f32, f32, f32)> = run.iter().map(|&(p, _)| p).collect();
            let offset = out.len();
            if run[0].1 {
                let (pts, at) = crate::smoother::smooth_path_indexed(
                    plg,
                    run_points,
                    self.limits,
                    self.step_weight,
                );
                smoothed += pts.len();
                landed.extend(at.into_iter().map(|i| offset + i));
                out.extend(pts);
            } else {
                landed.extend(offset..offset + run_points.len());
                out.extend(run_points);
            }
        }
        let point_of_pulled = landed[lead..lead + pulled.len()].to_vec();
        Piece {
            at: from,
            points: out,
            pulled,
            point_of_pulled,
            smoothed,
        }
    }
}

/// A route assembled piece by piece, each new piece starting on the last one's end point.
#[derive(Default)]
struct Spliced {
    points: Vec<(f32, f32, f32)>,
    /// Pulled cells as indices into the new cell path.
    pulled: Vec<usize>,
    point_of_pulled: Vec<usize>,
    smoothed: usize,
}

impl Spliced {
    fn append(&mut self, piece: Piece) {
        // after the first piece, a piece's first point and first pulled cell are the last piece's end
        let skip = usize::from(!self.points.is_empty());
        let base = self.points.len() - skip;
        self.points.extend_from_slice(&piece.points[skip..]);
        self.pulled
            .extend(piece.pulled[skip..].iter().map(|&i| piece.at + i));
        self.point_of_pulled
            .extend(piece.point_of_pulled[skip..].iter().map(|&i| base + i));
        self.smoothed += piece.smoothed;
    }
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
    let limits = PathLimits::from_config(config);

    // The cached path's head is the stale original start. Resume from where the
    // robot sits on it so the follower is pulled forward, never back to it.
    let resume = resume_segment(cached, start_pose, voxel_size);

    // Walk each chord ahead at surface resolution. On a blockage, keep up to the
    // last traversable cell so the path ends at the obstacle, not the prior anchor.
    let mut waypoints = vec![start_pose];
    let mut blocked = false;
    for j in resume..cached.len() - 1 {
        let (last_safe, cut) = last_safe_on_chord(plg, cached[j], cached[j + 1], &limits);
        if cut {
            if let Some(p) = last_safe {
                if waypoints.last() != Some(&p) {
                    waypoints.push(p);
                }
            }
            blocked = true;
            break;
        }
        waypoints.push(key_point(plg, cached[j + 1], voxel_size));
    }

    if waypoints.len() < 2 {
        return Vec::new();
    }

    // Hold the standoff only when blocked. A clean run to the goal has nothing
    // to stand off from.
    if blocked {
        let held = back_off_tail(&waypoints, BEST_EFFORT_DISTANCE_M);
        // A robot already at the standoff has nothing left to follow.
        return match held.last() {
            Some(end)
                if (end.0 - start_pose.0).hypot(end.1 - start_pose.1) >= config.goal_tolerance =>
            {
                held
            }
            _ => Vec::new(),
        };
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

/// Surface point of a cell key, its voxel's top face once it left the graph.
fn key_point(plg: &PlannerGraph, k: VoxelKey, voxel_size: f32) -> (f32, f32, f32) {
    plg.cells.id(k).map_or_else(
        || surface_point_xyz(k.0, k.1, k.2, voxel_size),
        |id| plg.cells.point(id, voxel_size),
    )
}

/// Hand `visit` each cell under chord a -> b with its rise until one strays, can't be stepped onto, or `visit` says stop; true if b was reached.
fn walk_chord(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    limits: &PathLimits,
    mut visit: impl FnMut(CellId, f32) -> bool,
) -> bool {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let samples = dx.abs().max(dy.abs()) * 2;
    if samples == 0 {
        // A same-column chord is traversable only within one cell.
        return a == b && plg.cells.id(a).is_none_or(|id| visit(id, 0.0));
    }
    let (za, zb) = (
        key_point(plg, a, limits.voxel_size).2,
        key_point(plg, b, limits.voxel_size).2,
    );
    let (mut last_ix, mut last_iy) = (i32::MIN, i32::MIN);
    let mut prev: Option<CellId> = None;
    for k in 0..=samples {
        let t = k as f32 / samples as f32;
        let ix = (a.0 as f32 + t * dx as f32).round() as i32;
        let iy = (a.1 as f32 + t * dy as f32).round() as i32;
        if ix == last_ix && iy == last_iy {
            continue;
        }
        last_ix = ix;
        last_iy = iy;
        let line_z = za + t * (zb - za);
        let Some(id) = plg.surface_near((ix, iy), line_z) else {
            return false;
        };
        if (plg.cells.z(id) - line_z).abs() > limits.step_m {
            return false;
        }
        let rise = prev.map_or(0.0, |p| plg.cells.rise(p, id, limits.voxel_size));
        if prev.is_some_and(|p| !limits.steps(plg, p, id)) || !visit(id, rise) {
            return false;
        }
        prev = Some(id);
    }
    true
}

/// Walk the straight chord a -> b at surface resolution, the same sampling the
/// segment validator uses. Returns the world point of the last traversable
/// surface cell reached and whether the chord was cut short of b. The point is
/// None when the chord's first cell is off the surface or inside the hard clearance.
fn last_safe_on_chord(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    limits: &PathLimits,
) -> (Option<(f32, f32, f32)>, bool) {
    let mut last_safe: Option<(f32, f32, f32)> = None;
    let reached = walk_chord(plg, a, b, limits, |id, _| {
        if !limits.penalty(plg, id).is_finite() {
            return false;
        }
        last_safe = Some(plg.cells.point(id, limits.voxel_size));
        true
    });
    (last_safe, !reached)
}

/// Index of the cached segment the robot is on, by nearest-point projection in
/// the ground plane. The route ahead resumes at the following cell.
fn resume_segment(cached: &[VoxelKey], start: (f32, f32, f32), voxel_size: f32) -> usize {
    let p = (start.0, start.1);
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
        let d2 = point_segment_dist2((a.0, a.1), (b.0, b.1), p);
        if d2 < best_d2 {
            best_d2 = d2;
            best = i;
        }
    }
    best
}

/// Squared distance from point p to segment a-b in the plane.
fn point_segment_dist2(a: (f32, f32), b: (f32, f32), p: (f32, f32)) -> f32 {
    let (abx, aby) = (b.0 - a.0, b.1 - a.1);
    let denom = abx * abx + aby * aby;
    let t = if denom == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / denom).clamp(0.0, 1.0)
    };
    let (cx, cy) = (a.0 + t * abx, a.1 + t * aby);
    let (dx, dy) = (p.0 - cx, p.1 - cy);
    dx * dx + dy * dy
}

/// Pick the entry node by connect cost plus cost-to-go, with its on-surface
/// lead-in and the node sequence to the goal. A goal cell inside the search
/// radius connects directly instead, signalled by an empty node sequence.
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
fn select_entry(
    plg: &PlannerGraph,
    start_cells: &[CellId],
    goal_cell: CellId,
    goal_node: NodeId,
    cost_to_go: &AHashMap<NodeId, f32>,
    pred_to_goal: &AHashMap<NodeId, NodeId>,
    node_cells: &Nodes,
    radius_m: f32,
) -> Option<(Vec<CellId>, Vec<NodeId>)> {
    let search = robot_search(&plg.cells, start_cells, radius_m);
    select_entry_from(
        plg,
        start_cells,
        &search,
        goal_cell,
        goal_node,
        cost_to_go,
        pred_to_goal,
        node_cells,
    )
}

/// select_entry, from the robot search over `start_cells` already run.
#[allow(clippy::too_many_arguments)]
fn select_entry_from(
    plg: &PlannerGraph,
    start_cells: &[CellId],
    (connect_dist, connect_pred): &(AHashMap<CellId, f32>, AHashMap<CellId, CellId>),
    goal_cell: CellId,
    goal_node: NodeId,
    cost_to_go: &AHashMap<NodeId, f32>,
    pred_to_goal: &AHashMap<NodeId, NodeId>,
    node_cells: &Nodes,
) -> Option<(Vec<CellId>, Vec<NodeId>)> {
    if connect_dist.contains_key(&goal_cell) {
        let mut lead = walk_local_preds(connect_pred, goal_cell);
        lead.reverse();
        return Some((lead, Vec::new()));
    }

    let mut entry_node = NO_NODE;
    let mut best_score = f32::INFINITY;
    // Scan the bounded reachable set, not every node. Tie-break by cell id for
    // deterministic order.
    for (&cell, &connect) in connect_dist {
        if !node_cells.has(cell) {
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
        let mut lead = walk_local_preds(connect_pred, entry_node);
        lead.reverse();
        return Some((lead, follow_preds(entry_node, goal_node, pred_to_goal)?));
    }

    for &start_cell in start_cells {
        let start_segment = walk_live_preds(plg, start_cell);
        let region_node = *start_segment
            .last()
            .expect("the start cell is live, so the walk keeps at least it");
        if !node_cells.has(region_node)
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

/// Bounded Dijkstra from the robot cell. Cost is wall-penalized for steering,
/// but the radius bounds metric distance, not penalized cost.
fn robot_search(
    cells: &SurfaceCells,
    sources: &[CellId],
    radius_m: f32,
) -> (AHashMap<CellId, f32>, AHashMap<CellId, CellId>) {
    let mut dist: AHashMap<CellId, f32> = AHashMap::new();
    let mut geo: AHashMap<CellId, f32> = AHashMap::new();
    let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
    let mut heap: BinaryHeap<Scored<CellId>> = BinaryHeap::new();
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

/// Nearest node to `from` by hops, ignoring edge cost so it reaches a node
/// across cells the wall penalty makes impassable. Returns the node and the
/// path from `from`.
fn nearest_node(
    cells: &SurfaceCells,
    from: CellId,
    node_cells: &Nodes,
) -> Option<(NodeId, Vec<CellId>)> {
    if node_cells.has(from) {
        return Some((from, vec![from]));
    }
    let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
    let mut seen: AHashSet<CellId> = AHashSet::new();
    let mut queue: VecDeque<CellId> = VecDeque::new();
    seen.insert(from);
    queue.push_back(from);

    while let Some(u) = queue.pop_front() {
        for edge in cells.neighbors(u) {
            let v = edge.dest;
            if !seen.insert(v) {
                continue;
            }
            pred.insert(v, u);
            if node_cells.has(v) {
                let mut path = vec![v];
                let mut cur = v;
                while let Some(&p) = pred.get(&cur) {
                    cur = p;
                    path.push(cur);
                }
                path.reverse();
                return Some((v, path));
            }
            queue.push_back(v);
        }
    }
    None
}

/// walk_preds through the persistent Voronoi state, truncated at the first
/// cell that died since the state was written. Regional updates leave
/// out-of-window pred chains stale, and a freed slot's coord is garbage.
fn walk_live_preds(plg: &PlannerGraph, from: CellId) -> Vec<CellId> {
    let mut chain = walk_preds(&plg.cell_state, from);
    if let Some(cut) = chain.iter().position(|&c| !plg.cells.is_live(c)) {
        chain.truncate(cut);
    }
    chain
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

/// Cost-to-go to source for the reachable nodes, with a predecessor pointing one hop toward it, settled out until
/// every node in `targets` is (or nothing more is reachable). Nodes are keyed by their CellId; unsettled ones are absent.
fn node_dijkstra(
    plg: &PlannerGraph,
    source: NodeId,
    targets: &AHashSet<NodeId>,
) -> (AHashMap<NodeId, f32>, AHashMap<NodeId, NodeId>) {
    let mut tentative: AHashMap<NodeId, f32> = AHashMap::new();
    let mut dist: AHashMap<NodeId, f32> = AHashMap::new();
    let mut pred: AHashMap<NodeId, NodeId> = AHashMap::new();
    tentative.insert(source, 0.0);
    let mut heap: BinaryHeap<Scored<NodeId>> = BinaryHeap::new();
    heap.push(Scored(0.0, source));
    let mut left = targets.len();

    while let Some(Scored(d, u)) = heap.pop() {
        if left == 0 {
            break;
        }
        if dist.contains_key(&u) || d > tentative.get(&u).copied().unwrap_or(f32::INFINITY) {
            continue;
        }
        dist.insert(u, d);
        if targets.contains(&u) {
            left -= 1;
        }
        for &edge_idx in plg.node_edges.adjacent(u) {
            let edge = &plg.node_edges.edges()[edge_idx as usize];
            let neighbor = if edge.a == u { edge.b } else { edge.a };
            let nd = d + edge.cost;
            if !dist.contains_key(&neighbor)
                && nd < tentative.get(&neighbor).copied().unwrap_or(f32::INFINITY)
            {
                tentative.insert(neighbor, nd);
                pred.insert(neighbor, u);
                heap.push(Scored(nd, neighbor));
            }
        }
    }
    pred.retain(|n, _| dist.contains_key(n));
    (dist, pred)
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
        let edge_idx = plg
            .node_edges
            .between(a, b)
            .expect("consecutive nodes in path must share an edge");
        let edge = &plg.node_edges.edges()[edge_idx as usize];

        // The stored corridor runs a to b. Dead cells are skipped: the update
        // safety walk drops broken corridors, so gaps here are rare races.
        let live = |&&c: &&CellId| plg.cells.is_live(c);
        if a == edge.a {
            for &c in edge.chain.iter().filter(live) {
                push_cell(&mut cells, c);
            }
        } else {
            for &c in edge.chain.iter().rev().filter(live) {
                push_cell(&mut cells, c);
            }
        }
    }

    for &c in goal_segment.iter().rev() {
        push_cell(&mut cells, c);
    }

    cells
}

/// Clearance and step limits the smoother holds the path to.
pub(crate) struct PathLimits {
    pub(crate) clearance_m: f32,
    pub(crate) buffer_m: f32,
    pub(crate) buffer_weight: f32,
    pub(crate) voxel_size: f32,
    /// Max traversable vertical step, the hard bound shared with the graph.
    pub(crate) step_m: f32,
    /// How far fit noise alone moves a surface height.
    pub(crate) noise_m: f32,
}

impl PathLimits {
    /// The wall penalty of a cell, infinite inside the hard clearance.
    pub(crate) fn penalty(&self, plg: &PlannerGraph, id: CellId) -> f32 {
        let wall_dist = plg
            .wall_state
            .dist
            .get(id as usize)
            .copied()
            .unwrap_or(f32::INFINITY);
        penalty_of(
            wall_dist,
            self.clearance_m,
            self.buffer_m,
            self.buffer_weight,
        )
    }

    /// The robot steps from cell `a` to cell `b`.
    pub(crate) fn steps(&self, plg: &PlannerGraph, a: CellId, b: CellId) -> bool {
        let gap = plg.cells.gap(a, b, self.voxel_size);
        within_step(gap, self.voxel_size, self.step_m, self.noise_m)
    }

    pub(crate) fn from_config(config: &Config) -> Self {
        Self {
            clearance_m: config.wall_clearance_m,
            buffer_m: config.wall_buffer_m,
            buffer_weight: config.wall_buffer_weight,
            voxel_size: config.voxel_size,
            step_m: config.step_threshold_m,
            noise_m: config.plane_noise_m,
        }
    }
}

/// Replace runs of cells with straight chords that come no closer to a wall and
/// climb no more than the run they replace.
#[cfg(test)]
fn string_pull(plg: &PlannerGraph, cells: &[CellId], limits: &PathLimits) -> Vec<CellId> {
    string_pull_at(plg, cells, limits)
        .into_iter()
        .map(|i| cells[i])
        .collect()
}

/// string_pull, as the indices of the kept cells.
fn string_pull_at(plg: &PlannerGraph, cells: &[CellId], limits: &PathLimits) -> Vec<usize> {
    if cells.len() <= 2 {
        return (0..cells.len()).collect();
    }
    let metrics = |from: CellId, to: CellId| {
        segment_metrics(plg, plg.cells.coord(from), plg.cells.coord(to), limits)
    };
    let mut out = vec![0];
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
        out.push(best);
        anchor = best;
    }
    out
}

/// Worst wall penalty and total climb along the straight segment a -> b. None if
/// it leaves the surface, climbs past a step, or enters the hard clearance.
fn segment_metrics(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    limits: &PathLimits,
) -> Option<(f32, f32)> {
    let mut max_pen = 1.0_f32;
    let mut total = 0.0_f32;
    let reached = walk_chord(plg, a, b, limits, |id, rise| {
        let pen = limits.penalty(plg, id);
        max_pen = max_pen.max(pen);
        total += climb(rise, limits.noise_m);
        pen.is_finite()
    });
    reached.then_some((max_pen, total))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_lookup, build_whole_voxel_cells};
    use crate::edges::build_node_edges;
    use crate::nodes::{NodeData, Nodes};

    const VOXEL: f32 = 0.1;
    const Z_TOL: f32 = 1.5;

    fn graph_with_nodes(surface_cells: &[VoxelKey], node_cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(surface_cells, &mut plg.surface_lookup);
        build_whole_voxel_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        let list = node_cells
            .iter()
            .map(|&c| {
                let id = plg.cells.id(c).expect("node cell must be in surface");
                NodeData {
                    cell_id: id,
                    pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
                }
            })
            .collect();
        let params = truncate_config().placement_params();
        plg.nodes = Nodes::from_list(&plg.cells, list, &params);
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
        );
        plg
    }

    fn strip(n: i32) -> Vec<VoxelKey> {
        (0..n).map(|x| (x, 0, 0)).collect()
    }

    fn plan_simple(
        plg: &PlannerGraph,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
    ) -> Option<Vec<(f32, f32, f32)>> {
        let config = truncate_config();
        plan(plg, start, goal, &config, Stale::NONE).map(|(wp, _)| wp)
    }

    fn surface_graph(cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(cells, &mut plg.surface_lookup);
        build_whole_voxel_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        plg
    }

    fn truncate_config() -> Config {
        Config {
            voxel_size: VOXEL,
            robot_height: Z_TOL,
            surface_closing_radius: 0.0,
            wall_clearance_m: 0.2,
            wall_buffer_m: 0.5,
            wall_buffer_weight: 4.0,
            step_threshold_m: 0.25,
            max_surface_slope_deg: 45.0,
            plane_noise_m: 0.01,
            step_penalty_weight: 4.0,
            ..crate::mls_planner::test_config()
        }
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

    #[test]
    fn truncate_stops_a_robot_parked_at_the_standoff() {
        let cfg = truncate_config();
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let blocked: Vec<VoxelKey> = (0..40).filter(|&x| x != 35).map(|x| (x, 0, 0)).collect();
        let standoff_x = surface_point_xyz(34, 0, 0, VOXEL).0 - BEST_EFFORT_DISTANCE_M;
        let z = surface_point_xyz(0, 0, 0, VOXEL).2;

        // Parked short of the standoff, inside the arrival tolerance: nothing left to follow.
        for short in [0.0, 0.1, 0.2, 0.29] {
            let robot = (standoff_x - short, 0.0, z);
            let wp = truncate_to_safe(&surface_graph(&blocked), &cached, robot, &cfg);
            assert!(wp.is_empty(), "{short} m short of the standoff: {wp:?}");
        }

        // Further back, the standoff is still ahead to drive to.
        let robot = (standoff_x - 0.5, 0.0, z);
        assert!(!truncate_to_safe(&surface_graph(&blocked), &cached, robot, &cfg).is_empty());
    }

    /// An L corridor 5 cells wide, nodes along it, a start at its foot and a goal at its top.
    fn l_corridor() -> PlannerGraph {
        let mut cells: Vec<VoxelKey> = Vec::new();
        for x in 0..80 {
            for y in 0..5 {
                cells.push((x, y, 0));
            }
        }
        for x in 75..80 {
            for y in 5..60 {
                cells.push((x, y, 0));
            }
        }
        let nodes = [
            (10, 2, 0),
            (30, 2, 0),
            (50, 2, 0),
            (70, 2, 0),
            (77, 20, 0),
            (77, 40, 0),
            (77, 57, 0),
        ];
        graph_with_nodes(&cells, &nodes)
    }

    /// A staircase corridor 5 cells wide: `steps` legs along x, each 20 cells and then 10 up, with a node per leg and turn.
    fn staircase(steps: i32) -> (PlannerGraph, (f32, f32, f32), (f32, f32, f32)) {
        let mut cells: AHashSet<VoxelKey> = AHashSet::new();
        let mut nodes: Vec<VoxelKey> = Vec::new();
        for s in 0..steps {
            let (x0, y0) = (20 * s, 10 * s);
            for x in x0..x0 + 25 {
                for y in y0..y0 + 5 {
                    cells.insert((x, y, 0));
                }
            }
            for x in x0 + 20..x0 + 25 {
                for y in y0..y0 + 15 {
                    cells.insert((x, y, 0));
                }
            }
            nodes.push((x0 + 10, y0 + 2, 0));
            nodes.push((x0 + 22, y0 + 8, 0));
        }
        let mut cells: Vec<VoxelKey> = cells.into_iter().collect();
        cells.sort_unstable();
        let start = (0.25, 0.25, 0.05);
        // at the top of the last turn
        let goal = (
            (20 * steps + 2) as f32 * VOXEL + 0.05,
            (10 * steps + 2) as f32 * VOXEL + 0.05,
            0.05,
        );
        (graph_with_nodes(&cells, &nodes), start, goal)
    }

    const L_GOAL: (f32, f32, f32) = (7.75, 5.85, 0.05);

    fn plan_l(
        plg: &PlannerGraph,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        reuse: Option<(&RouteCache, &RouteChanges)>,
    ) -> (PlannedPath, RouteCache) {
        plan_reusing(plg, start, goal, &truncate_config(), Stale::NONE, reuse).unwrap()
    }

    fn unchanged(route: &RouteCache) -> RouteChanges {
        RouteChanges::none(route.assembled.len(), route.points.len())
    }

    /// Every point of `a` within `tol` m of some point of `b`, in the ground plane.
    fn assert_near(a: &[(f32, f32, f32)], b: &[(f32, f32, f32)], tol: f32) {
        for p in a {
            let gap = b
                .iter()
                .map(|q| (p.0 - q.0).hypot(p.1 - q.1))
                .fold(f32::INFINITY, f32::min);
            assert!(gap < tol, "point {p:?} is {gap} m off the other route");
        }
    }

    /// How close a reused route stays to a whole replan's: a voxel.
    const MATCH_M: f32 = VOXEL;

    #[test]
    fn a_replan_to_the_same_goal_reuses_the_route_past_the_robot() {
        let plg = l_corridor();
        let (_, route) = plan_l(&plg, (0.25, 0.25, 0.05), L_GOAL, None);
        assert_eq!(route.kept_cells, 0);
        // a little further along: the robot's end is planned again, the cached rest kept as it was
        let start = (0.65, 0.25, 0.05);
        let ((reused, cells), again) =
            plan_l(&plg, start, L_GOAL, Some((&route, &unchanged(&route))));
        let ((whole, whole_cells), fresh) = plan_l(&plg, start, L_GOAL, None);
        assert_eq!(again.assembled, fresh.assembled);
        assert!(
            again.kept_cells > again.assembled.len() / 3,
            "{} of {} cells kept",
            again.kept_cells,
            again.assembled.len()
        );
        assert!(
            again.smoothed < fresh.smoothed,
            "{} vs {} points smoothed",
            again.smoothed,
            fresh.smoothed
        );
        let tail = 20;
        assert_eq!(
            reused[reused.len() - tail..],
            route.points[route.points.len() - tail..],
            "the cached tail was not kept"
        );
        assert_eq!(reused.first(), Some(&start));
        assert_eq!(reused.last(), Some(&L_GOAL));
        // the pulled cells near the robot are a whole plan's
        assert_eq!(cells[0], whole_cells[0]);
        // and the route is the one a whole plan smooths, to within a voxel
        assert_near(&reused, &whole, MATCH_M);
        assert_near(&whole, &reused, MATCH_M);
    }

    #[test]
    fn a_change_on_a_long_route_resmooths_only_its_stretch() {
        let (plg, start, goal) = staircase(30);
        let config = truncate_config();
        let (_, route) = plan_l(&plg, start, goal, None);
        let n = route.assembled.len();
        // one cell changed halfway: the stretch around it is pulled and smoothed again, both ends kept
        let mut halfway = unchanged(&route);
        halfway.cells[n / 2] = true;
        let ((reused, _), again) = plan_l(&plg, start, goal, Some((&route, &halfway)));
        let ((whole, _), fresh) = plan_l(&plg, start, goal, None);
        assert_eq!(again.assembled, fresh.assembled);
        let slack_cells = (config.route_reuse_slack_m / config.voxel_size).ceil() as usize;
        // what is planned again: the robot's slack, the change a slack either side, and the chords those reach into
        assert!(
            again.kept_cells * 2 > n && again.kept_cells + 4 * slack_cells < n,
            "kept {} of {n} cells around a {slack_cells}-cell slack",
            again.kept_cells
        );
        assert!(
            again.smoothed * 3 < fresh.smoothed,
            "{} of {} points smoothed",
            again.smoothed,
            fresh.smoothed
        );
        assert_eq!(reused.first(), Some(&start));
        assert_eq!(reused.last(), Some(&goal));
        assert_near(&reused, &whole, MATCH_M);
        assert_near(&whole, &reused, MATCH_M);
        // a changed point near the goal leaves the goal's end planned again
        let mut near_goal = unchanged(&route);
        near_goal.points[route.points.len() - 3] = true;
        let ((reused, _), _) = plan_l(&plg, start, goal, Some((&route, &near_goal)));
        let tail = &reused[reused.len() - 10..];
        assert_near(tail, &whole, MATCH_M);
        // a route to a goal that moved is planned whole
        let moved = (goal.0, goal.1 - VOXEL, goal.2);
        let ((reused, _), _) = plan_l(&plg, start, moved, Some((&route, &unchanged(&route))));
        let ((whole, _), _) = plan_l(&plg, start, moved, None);
        assert_eq!(reused, whole);
    }

    /// Timing, not a check: a whole plan of a long staircase route against a replan with one cell changed halfway.
    #[test]
    #[ignore]
    fn route_reuse_cost_on_a_long_route() {
        let (plg, start, goal) = staircase(200);
        let time = |f: &dyn Fn()| {
            let mut ms: Vec<f64> = (0..20)
                .map(|_| {
                    let t = std::time::Instant::now();
                    f();
                    t.elapsed().as_secs_f64() * 1e3
                })
                .collect();
            ms.sort_by(f64::total_cmp);
            ms[ms.len() / 2]
        };
        let (_, route) = plan_l(&plg, start, goal, None);
        let mut halfway = unchanged(&route);
        halfway.cells[route.assembled.len() / 2] = true;
        let whole = time(&|| {
            plan_l(&plg, start, goal, None);
        });
        let reused = time(&|| {
            plan_l(&plg, start, goal, Some((&route, &halfway)));
        });
        eprintln!(
            "route of {} cells, {} points: whole plan {whole:.2} ms, replan with one change {reused:.2} ms",
            route.assembled.len(),
            route.points.len()
        );
    }

    #[test]
    fn a_change_everywhere_plans_the_whole_route_again() {
        let plg = l_corridor();
        let (_, route) = plan_l(&plg, (0.25, 0.25, 0.05), L_GOAL, None);
        let start = (0.65, 0.25, 0.05);
        let mut all = unchanged(&route);
        all.cells.iter_mut().for_each(|c| *c = true);
        let ((reused, _), again) = plan_l(&plg, start, L_GOAL, Some((&route, &all)));
        let ((whole, _), _) = plan_l(&plg, start, L_GOAL, None);
        assert_eq!(again.kept_cells, 0);
        assert_eq!(reused, whole);
    }

    #[test]
    fn a_route_cell_that_changed_layer_is_planned_again() {
        let (mut plg, start, goal) = staircase(8);
        // a closure froze the map, and all of it but a band across the corridor migrated into the new layer
        let band = |k: VoxelKey| (80..83).contains(&k.0);
        let ids: Vec<CellId> = plg.cells.ids().collect();
        plg.cells.begin_layer(1);
        for &id in &ids {
            let k = plg.cells.coord(id);
            if !band(k) {
                plg.cells.move_to_layer(id, k);
            }
        }
        let (_, route) = plan_l(&plg, start, goal, None);
        let in_band: Vec<usize> = (0..route.assembled.len())
            .filter(|&i| band(plg.cells.coord(route.assembled[i])))
            .collect();
        assert!(!in_band.is_empty(), "the route crosses the band");
        let (_, kept) = plan_l(&plg, start, goal, Some((&route, &unchanged(&route))));
        // the band migrates too
        for &i in &in_band {
            let id = route.assembled[i];
            plg.cells.move_to_layer(id, plg.cells.coord(id));
        }
        let ((reused, _), again) = plan_l(&plg, start, goal, Some((&route, &unchanged(&route))));
        let ((whole, _), fresh) = plan_l(&plg, start, goal, None);
        assert!(
            again.kept_cells < kept.kept_cells,
            "the band is planned again"
        );
        assert!(again.kept_cells > 0, "the rest is kept");
        assert!(again.smoothed < fresh.smoothed);
        assert_near(&reused, &whole, MATCH_M);
    }

    #[test]
    fn plan_returns_none_if_disconnected() {
        // The gap must exceed SNAP_SEARCH_RADIUS_M so no start candidate
        // can relocate onto the goal island.
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((30..35).map(|x| (x, 0, 0)));
        let plg = graph_with_nodes(&cells, &[(2, 0, 0), (32, 0, 0)]);
        let result = plan_simple(&plg, (0.25, 0.0, 0.1), (3.25, 0.0, 0.1));
        assert!(result.is_none());
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
        for w in &wp[1..wp.len() - 1] {
            assert!(
                plg.cells.id(waypoint_key(w)).is_some(),
                "waypoint {w:?} is off the surface"
            );
        }
        let limits = PathLimits {
            clearance_m: 0.2,
            buffer_m: 0.5,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
            step_m: 0.25,
            noise_m: 0.0,
        };
        for pair in wp[1..wp.len() - 1].windows(2) {
            assert!(
                segment_metrics(
                    &plg,
                    waypoint_key(&pair[0]),
                    waypoint_key(&pair[1]),
                    &limits
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
        build_whole_voxel_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        let path: Vec<CellId> = (0..10).map(|x| plg.cells.id((x, 0, 0)).unwrap()).collect();

        let limits = PathLimits {
            clearance_m: 0.2,
            buffer_m: 0.5,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
            step_m: VOXEL,
            noise_m: 0.0,
        };
        plg.wall_state.dist = vec![f32::INFINITY; plg.cells.slot_capacity()];
        let open = string_pull(&plg, &path, &limits);
        assert_eq!(open.len(), 2, "open strip should collapse to its endpoints");

        let mid = plg.cells.id((5, 0, 0)).unwrap();
        plg.wall_state.dist[mid as usize] = 0.1; // below the 0.2 clearance
        let guarded = string_pull(&plg, &path, &limits);
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
        let node_cells = &plg.nodes;
        let all: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
        let (ctg, pred) = node_dijkstra(&plg, goal_node, &all);

        let (lead, node_seq) = select_entry(
            &plg,
            &[start],
            goal,
            goal_node,
            &ctg,
            &pred,
            node_cells,
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
    fn select_entry_anchors_lead_at_the_nearest_candidate() {
        // The nearest candidate connects, so the lead-in starts at the
        // robot's own cell even though other candidates exist.
        let plg = graph_with_nodes(&strip(20), &[(12, 0, 0)]);
        let near = plg.cells.id((10, 0, 0)).unwrap();
        let goal = plg.cells.id((19, 0, 0)).unwrap();
        let goal_node = plg.nodes[0].cell_id;
        let node_cells = &plg.nodes;
        let all: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
        let (ctg, pred) = node_dijkstra(&plg, goal_node, &all);

        let (lead, node_seq) =
            select_entry(&plg, &[near], goal, goal_node, &ctg, &pred, node_cells, 0.3).unwrap();
        assert_eq!(
            lead.first(),
            Some(&near),
            "lead must anchor at the robot's cell"
        );
        assert_eq!(lead.last(), Some(&goal_node));
        assert_eq!(node_seq, vec![goal_node]);
    }

    #[test]
    fn select_entry_falls_back_to_merged_rest_candidates() {
        // The nearest candidate sits on a disconnected fragment. The fallback
        // runs one merged wavefront over the rest, and only the candidate
        // that actually reaches the node roots the lead-in.
        let mut surface = strip(20);
        surface.extend((0..3).map(|x| (x, 10, 0)));
        let plg = graph_with_nodes(&surface, &[(12, 0, 0)]);
        let near = plg.cells.id((1, 10, 0)).unwrap();
        let far_rest = plg.cells.id((5, 0, 0)).unwrap();
        let reaching_rest = plg.cells.id((14, 0, 0)).unwrap();
        let goal = plg.cells.id((19, 0, 0)).unwrap();
        let goal_node = plg.nodes[0].cell_id;
        let node_cells = &plg.nodes;
        let all: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
        let (ctg, pred) = node_dijkstra(&plg, goal_node, &all);

        let entry = select_entry(&plg, &[near], goal, goal_node, &ctg, &pred, node_cells, 0.3);
        assert!(entry.is_none(), "disconnected candidate must not connect");

        let (lead, node_seq) = select_entry(
            &plg,
            &[far_rest, reaching_rest],
            goal,
            goal_node,
            &ctg,
            &pred,
            node_cells,
            0.3,
        )
        .unwrap();
        assert_eq!(
            lead.first(),
            Some(&reaching_rest),
            "lead must root at the candidate that reaches the node"
        );
        assert_eq!(lead.last(), Some(&goal_node));
        assert_eq!(node_seq, vec![goal_node]);
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
        let limits = PathLimits {
            clearance_m: 0.2,
            buffer_m: 0.3,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
            step_m: 2.0 * VOXEL,
            noise_m: 0.0,
        };
        assert!(segment_metrics(&plg, (5, 5, 0), (5, 5, 4), &limits).is_none());
        assert_eq!(
            segment_metrics(&plg, (5, 5, 0), (5, 5, 0), &limits),
            Some((1.0, 0.0))
        );
    }
}
