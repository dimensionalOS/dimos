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

//! Optimistic bridge selection between disconnected surface components.
//!
//! When the goal's component is unreachable, pick where to wait out the map:
//! a short hop across unobserved space that mapping is likely to fill in as
//! the robot approaches. Candidates pair boundary cells of different
//! components within a hop cap, gated by slope and headroom so a bridge is
//! never steeper or tighter than anything the robot could traverse. The
//! selected bridge is the first hop of the cheapest gated chain of hops from
//! the start component to the goal component, so long unknown stretches lose
//! to chains of short hops across observed fragments.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use ahash::{AHashMap, AHashSet};

use crate::adjacency::{CellId, NO_CELL};
use crate::dijkstra::walk_preds;
use crate::edges::{NodeId, PlannerGraph};
use crate::mls_planner::Config;
use crate::planner::{goal_node_of, node_dijkstra, Bridge};
use crate::surfaces::ColumnIz;
use crate::voxel::{surface_point_xyz, VoxelKey};

type Xyz = (f32, f32, f32);

/// Bridges kept per component pair after gating.
const PAIRS_PER_COMPONENT_PAIR: usize = 2;
/// Ungated candidates screened per component pair before the headroom walk.
const CANDIDATES_PER_COMPONENT_PAIR: usize = 8;
/// Max hops a chain may take.
const MAX_CHAIN_HOPS: usize = 4;
/// Max total bridged distance across a chain.
const MAX_CHAIN_GAP_M: f32 = 2.5;
/// Max contiguous sub-clearance run a ribbon geodesic may cross. Each run is
/// its own widen-as-approached claim, so the bound is per run rather than
/// summed over the route, and it is far looser than the void budget because
/// a ribbon is measured surface, only thin.
const RIBBON_RUN_MAX_M: f32 = 4.0;
/// Cap on how far past the near side a ribbon aim reaches. The bridge claim
/// stays local to where the robot actually attempts, so demotion and
/// hysteresis anchor there and the drawn bridge reflects the next stretch,
/// not the whole ribbon.
const RIBBON_AIM_LOOKAHEAD_M: f32 = 2.0;
/// Smallest component worth hopping to. Start and goal components are exempt.
const MIN_LANDING_CELLS: u32 = 3;
/// Evidence scale: hops touching sparse components cost proportionally more.
const EVIDENCE_CELLS: f32 = 8.0;
/// Ban-and-retry rounds when a winning chain fails the aggregate gates.
const CHAIN_RETRIES: usize = 4;
/// The robot counts as arrived at its bridge target within this ground distance.
const ARRIVE_DIST_M: f32 = 1.2;
/// Bridges whose endpoints moved less than this are the same bridge.
const SAME_BRIDGE_TOL_M: f32 = 0.75;
/// Consecutive arrived-and-stalled replans before a bridge is demoted.
const STALL_REPLANS: u32 = 40;
/// Replans a demoted bridge endpoint stays blacklisted.
const BLACKLIST_REPLANS: u32 = 300;
/// Candidates this close to a demoted endpoint are rejected too.
const BLACKLIST_RADIUS_M: f32 = 0.5;

/// The chosen bridge: the start-side cell to plan to and the far-side point.
pub struct Selection {
    pub near: CellId,
    pub aim: Xyz,
}

/// Cross-replan bridge memory: the held bridge for hysteresis, stall
/// tracking, and demoted endpoints the selector must avoid.
#[derive(Default)]
pub struct BridgeState {
    held: Option<Bridge>,
    stall: u32,
    /// The robot has been away from the held bridge's near side, so arriving
    /// counts as an attempt. Without this a robot loitering next to a gap
    /// would demote every nearby bridge without ever trying one.
    armed: bool,
    blacklist: Vec<(Xyz, u32)>,
}

impl BridgeState {
    /// Record the bridge a replan selected. Traveling to the target and then
    /// holding the same bridge without the components connecting demotes it,
    /// so the next replan tries the runner-up gap instead.
    pub fn note_bridge(&mut self, bridge: Bridge, start: Xyz) {
        self.tick();
        let same = self
            .held
            .is_some_and(|(_, far)| dist3(far, bridge.1) < SAME_BRIDGE_TOL_M);
        let at_target = ground_dist(start, bridge.0) < ARRIVE_DIST_M;
        if !same {
            self.stall = 0;
            self.armed = !at_target;
        } else if !at_target {
            self.armed = true;
            self.stall = 0;
        } else if self.armed {
            self.stall += 1;
            if self.stall >= STALL_REPLANS {
                self.blacklist.push((bridge.1, BLACKLIST_REPLANS));
                self.stall = 0;
                self.held = None;
                tracing::info!(aim = ?bridge.1, "bridge never connected, demoting it");
                return;
            }
        }
        self.held = Some(bridge);
    }

    /// A full plan or a no-bridge replan drops the held bridge.
    pub fn note_no_bridge(&mut self) {
        self.tick();
        self.held = None;
        self.stall = 0;
    }

    fn tick(&mut self) {
        for entry in self.blacklist.iter_mut() {
            entry.1 = entry.1.saturating_sub(1);
        }
        self.blacklist.retain(|&(_, ttl)| ttl > 0);
    }

    fn is_blacklisted(&self, p: Xyz) -> bool {
        self.blacklist
            .iter()
            .any(|&(b, _)| dist3(b, p) < BLACKLIST_RADIUS_M)
    }
}

/// A gated candidate hop between two components.
struct Hop {
    a: CellId,
    b: CellId,
    label_a: u32,
    label_b: u32,
    gap: f32,
    horiz: f32,
    rise: f32,
    cost: f32,
}

/// Pick the optimistic bridge, or none when no gated chain of hops connects
/// the start component to the goal component.
#[allow(clippy::too_many_arguments)]
pub fn select_bridge(
    plg: &PlannerGraph,
    by_col: &ColumnIz,
    reachable_mask: &[bool],
    start_cell: CellId,
    start_pose: Xyz,
    goal_cell: CellId,
    goal_pose: Xyz,
    state: &BridgeState,
    banned_aims: &[Xyz],
    config: &Config,
) -> Option<Selection> {
    if config.bridge_max_hop_m <= 0.0 {
        return None;
    }
    let t0 = std::time::Instant::now();
    let (label, sizes) = label_components(plg);
    let start_label = label[start_cell as usize];
    let goal_label = label[goal_cell as usize];
    // Same adjacency component: the route exists as measured surface but is
    // passably broken, typically a sparse stretch under the wall clearance.
    // Approach widens such surface, so bridge along it instead of over voids.
    if start_label == goal_label {
        return select_ribbon_bridge(plg, start_cell, goal_cell, state, banned_aims, config);
    }

    let t1 = std::time::Instant::now();
    let hops = collect_hops(
        plg,
        by_col,
        &label,
        &sizes,
        start_label,
        goal_label,
        state,
        banned_aims,
        config,
    );
    if hops.is_empty() {
        return None;
    }

    let t2 = std::time::Instant::now();
    // Surface-aware terminal costs via the node graph and the Voronoi fields.
    let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
    let goal_ctg = goal_node_of(plg, goal_cell, &node_cells).map(|(n, _)| node_dijkstra(plg, n).0);
    let start_node = *walk_preds(&plg.cell_state, start_cell)
        .last()
        .expect("walk_preds returns at least the start cell");
    let start_ctg = node_cells
        .contains(&start_node)
        .then(|| node_dijkstra(plg, start_node).0);

    let t3 = std::time::Instant::now();
    let graph = EpGraph::build(plg, &hops, config.voxel_size);
    tracing::debug!(
        label_ms = t1.duration_since(t0).as_secs_f64() * 1e3,
        hops_ms = t2.duration_since(t1).as_secs_f64() * 1e3,
        ctg_ms = t3.duration_since(t2).as_secs_f64() * 1e3,
        n_hops = hops.len(),
        n_eps = graph.labels.len(),
        "bridge selection phases"
    );
    let mut banned: AHashSet<usize> = AHashSet::new();
    for _ in 0..CHAIN_RETRIES {
        let sol = graph.solve(
            plg,
            &hops,
            &banned,
            reachable_mask,
            start_label,
            goal_label,
            start_ctg.as_ref(),
            goal_ctg.as_ref(),
            start_pose,
            goal_pose,
            state.held,
            config,
        )?;
        let best_chain = chain_of(&sol.pred, &hops, sol.best_hop, sol.best_far_ep);
        if let Some(worst) = chain_violation(&hops, &best_chain, config) {
            banned.insert(worst);
            continue;
        }
        // Hold the previous bridge while it stays admissible and close in
        // cost, so the target does not flap between rival gaps.
        let mut choice = sol.best_hop;
        if let Some((held_hop, held_far, held_total)) = sol.held {
            if held_hop != sol.best_hop
                && held_total <= sol.best_total * (1.0 + config.bridge_switch_margin)
            {
                let held_chain = chain_of(&sol.pred, &hops, held_hop, held_far);
                if chain_violation(&hops, &held_chain, config).is_none() {
                    choice = held_hop;
                }
            }
        }
        let hop = &hops[choice];
        let (near, far) = if hop.label_a == start_label {
            (hop.a, hop.b)
        } else {
            (hop.b, hop.a)
        };
        let (ix, iy, iz) = plg.cells.coord(far);
        return Some(Selection {
            near,
            aim: surface_point_xyz(ix, iy, iz, config.voxel_size),
        });
    }
    None
}

/// Bridge along measured-but-impassable surface. Goal-rooted Dijkstra where
/// an impassable edge costs its geometric length times the bridge weight,
/// with the total impassable length capped. The bridge spans the first
/// sub-clearance ribbon on the winning geodesic: the robot drives to the
/// ribbon's near side and mapping widens the sparse surface as it arrives.
fn select_ribbon_bridge(
    plg: &PlannerGraph,
    start_cell: CellId,
    goal_cell: CellId,
    state: &BridgeState,
    banned_aims: &[Xyz],
    config: &Config,
) -> Option<Selection> {
    let n = plg.cells.slot_capacity();
    let mut dist: Vec<f32> = vec![f32::INFINITY; n];
    let mut grey: Vec<f32> = vec![0.0; n];
    let mut pred: Vec<CellId> = vec![NO_CELL; n];
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    // A* toward the start keeps the search off the far reaches of the
    // passable region. Straight-line distance under-counts every weight, so
    // the heuristic is admissible.
    let start_pos = {
        let c = plg.cells.coord(start_cell);
        surface_point_xyz(c.0, c.1, c.2, config.voxel_size)
    };
    let h = |id: CellId| {
        let c = plg.cells.coord(id);
        dist3(
            surface_point_xyz(c.0, c.1, c.2, config.voxel_size),
            start_pos,
        )
    };
    dist[goal_cell as usize] = 0.0;
    heap.push(Scored(h(goal_cell), goal_cell as usize));
    while let Some(Scored(f, u)) = heap.pop() {
        let u = u as CellId;
        if u == start_cell {
            break;
        }
        if f > dist[u as usize] + h(u) + 1e-3 {
            continue;
        }
        for e in plg.cells.neighbors(u) {
            // Geometric costs on both sides: this search weighs evidence, so
            // a meter of real surface counts as a meter no matter how tight
            // its soft wall penalty is. Comfort belongs to the path planner.
            let (w, ng) = if e.cost.is_finite() {
                (e.base_cost, 0.0)
            } else {
                // Demoted and banned areas are not crossable as ribbon, so
                // the geodesic reroutes through another corridor if any.
                let c = plg.cells.coord(e.dest);
                let p = surface_point_xyz(c.0, c.1, c.2, config.voxel_size);
                if state.is_blacklisted(p) || banned_aims.iter().any(|&b| dist3(b, p) < 1e-3) {
                    continue;
                }
                (
                    e.base_cost * config.bridge_cost_weight,
                    grey[u as usize] + e.base_cost,
                )
            };
            if ng > RIBBON_RUN_MAX_M {
                continue;
            }
            let nd = dist[u as usize] + w;
            if nd < dist[e.dest as usize] {
                dist[e.dest as usize] = nd;
                grey[e.dest as usize] = ng;
                pred[e.dest as usize] = u;
                heap.push(Scored(nd + h(e.dest), e.dest as usize));
            }
        }
    }
    if !dist[start_cell as usize].is_finite() {
        tracing::debug!("ribbon bridge: start unreached within the grey run cap");
        return None;
    }

    // Walk the geodesic from the start. The bridge starts at the last cell
    // before the first impassable edge and aims at the cell where the ribbon
    // hands back to passable surface, or at the lookahead cap into a long
    // ribbon, whichever comes first.
    let mut cur = start_cell;
    let mut near = start_cell;
    let mut in_ribbon = false;
    let mut ribbon_len = 0.0_f32;
    let mut aim_cell: Option<CellId> = None;
    while aim_cell.is_none() {
        let next = pred[cur as usize];
        if next == NO_CELL {
            // Reached the goal. Inside a ribbon it is the aim itself; on a
            // fully passable geodesic there is nothing to bridge.
            if in_ribbon {
                aim_cell = Some(cur);
            } else {
                tracing::debug!("ribbon bridge: geodesic fully passable, nothing to bridge");
                return None;
            }
            break;
        }
        let edge = plg.cells.neighbors(cur).iter().find(|e| e.dest == next);
        let passable = edge.is_some_and(|e| e.cost.is_finite());
        let step_len = edge.map_or(0.0, |e| e.base_cost);
        if !in_ribbon {
            if passable {
                near = next;
            } else {
                in_ribbon = true;
                ribbon_len = step_len;
            }
        } else if passable {
            aim_cell = Some(cur);
        } else {
            ribbon_len += step_len;
        }
        if in_ribbon && aim_cell.is_none() && ribbon_len >= RIBBON_AIM_LOOKAHEAD_M {
            aim_cell = Some(next);
        }
        cur = next;
    }
    let aim_cell = aim_cell?;
    let (ix, iy, iz) = plg.cells.coord(aim_cell);
    let aim = surface_point_xyz(ix, iy, iz, config.voxel_size);
    if state.is_blacklisted(aim) || banned_aims.iter().any(|&b| dist3(b, aim) < 1e-3) {
        tracing::debug!(?aim, "ribbon bridge: aim is demoted or banned");
        return None;
    }
    Some(Selection { near, aim })
}

/// Connected-component label per cell over surface adjacency, edge costs
/// ignored, plus each component's cell count. Adjacency rather than passable
/// connectivity so a measured-impassable link never reads as a bridgeable
/// unknown, and sub-clearance fringe attaches to its parent surface.
fn label_components(plg: &PlannerGraph) -> (Vec<u32>, Vec<u32>) {
    let n = plg.cells.slot_capacity();
    let mut label: Vec<u32> = vec![u32::MAX; n];
    let mut sizes: Vec<u32> = Vec::new();
    let mut stack: Vec<CellId> = Vec::new();
    for id in plg.cells.ids() {
        if label[id as usize] != u32::MAX {
            continue;
        }
        let comp = sizes.len() as u32;
        sizes.push(0);
        label[id as usize] = comp;
        stack.push(id);
        while let Some(u) = stack.pop() {
            sizes[comp as usize] += 1;
            for e in plg.cells.neighbors(u) {
                if label[e.dest as usize] == u32::MAX {
                    label[e.dest as usize] = comp;
                    stack.push(e.dest);
                }
            }
        }
    }
    (label, sizes)
}

/// Boundary cells of one spatial bin and the distinct components in it.
#[derive(Default)]
struct BinData {
    cells: Vec<CellId>,
    labels: Vec<u32>,
}

/// Gather gated candidate hops: cross-component boundary-cell pairs within
/// the hop cap, sloped no steeper than the grade, with body clearance along
/// the chord, keeping the cheapest few per component pair. Pairs that cannot
/// sit on a start-to-goal chain within the total gap budget are dropped
/// before the expensive chord walk.
#[allow(clippy::too_many_arguments)]
fn collect_hops(
    plg: &PlannerGraph,
    by_col: &ColumnIz,
    label: &[u32],
    sizes: &[u32],
    start_label: u32,
    goal_label: u32,
    state: &BridgeState,
    banned_aims: &[Xyz],
    config: &Config,
) -> Vec<Hop> {
    let voxel = config.voxel_size;
    let hop_cap = config.bridge_max_hop_m;
    let grade = config.bridge_max_grade;
    let step_slack = config.step_threshold_m;

    let passes_size = |l: u32| -> bool {
        l == start_label || l == goal_label || sizes[l as usize] >= MIN_LANDING_CELLS
    };

    // Boundary cells are where the known surface stops: some 4-neighbor
    // direction has no traversable continuation. Only they can face a gap.
    let bin = ((hop_cap / voxel).ceil() as i32).max(1);
    let mut bins: AHashMap<(i32, i32), BinData> = AHashMap::new();
    for id in plg.cells.ids() {
        let l = label[id as usize];
        if !passes_size(l) || !is_boundary(&plg.cells, id) {
            continue;
        }
        let (ix, iy, _) = plg.cells.coord(id);
        let entry = bins
            .entry((ix.div_euclid(bin), iy.div_euclid(bin)))
            .or_default();
        entry.cells.push(id);
        if !entry.labels.contains(&l) {
            entry.labels.push(l);
        }
    }

    // Screen cross-component pairs, cheapest few per component pair. Bins
    // whose whole neighborhood holds one component cannot produce a pair, and
    // that is almost every bin along the mapped perimeter.
    let mut screened: AHashMap<(u32, u32), Vec<Hop>> = AHashMap::new();
    for (&(bx, by), bd) in &bins {
        if !neighborhood_mixed(&bins, bx, by, bd) {
            continue;
        }
        for &u in &bd.cells {
            let lu = label[u as usize];
            let cu = plg.cells.coord(u);
            for dbx in -1..=1 {
                for dby in -1..=1 {
                    let Some(nd) = bins.get(&(bx + dbx, by + dby)) else {
                        continue;
                    };
                    for &v in &nd.cells {
                        if v <= u {
                            continue;
                        }
                        let lv = label[v as usize];
                        if lv == lu {
                            continue;
                        }
                        let cv = plg.cells.coord(v);
                        let dx = (cv.0 - cu.0) as f32 * voxel;
                        let dy = (cv.1 - cu.1) as f32 * voxel;
                        let dz = (cv.2 - cu.2) as f32 * voxel;
                        let horiz = (dx * dx + dy * dy).sqrt();
                        let rise = dz.abs();
                        let gap = (horiz * horiz + dz * dz).sqrt();
                        if gap > hop_cap || rise > grade * horiz + step_slack {
                            continue;
                        }
                        let evidence = 1.0
                            + EVIDENCE_CELLS / sizes[lu as usize].min(sizes[lv as usize]) as f32;
                        let cost = config.bridge_cost_weight * gap * evidence;
                        let key = if lu < lv { (lu, lv) } else { (lv, lu) };
                        let list = screened.entry(key).or_default();
                        list.push(Hop {
                            a: u,
                            b: v,
                            label_a: lu,
                            label_b: lv,
                            gap,
                            horiz,
                            rise,
                            cost,
                        });
                        if list.len() > CANDIDATES_PER_COMPONENT_PAIR * 2 {
                            list.sort_by(|x, y| x.cost.total_cmp(&y.cost));
                            list.truncate(CANDIDATES_PER_COMPONENT_PAIR);
                        }
                    }
                }
            }
        }
    }

    // Component-level budget prune: a hop is only worth gating if some chain
    // through it fits the total bridged-gap budget end to end.
    let ds = comp_gap_dist(&screened, start_label);
    let dg = comp_gap_dist(&screened, goal_label);
    let within = |l: u32, d: &AHashMap<u32, f32>| d.get(&l).copied().unwrap_or(f32::INFINITY);
    let budget = MAX_CHAIN_GAP_M + 1e-3;

    let step_cells = config.step_cells();
    let headroom_cells = config.headroom_cells();
    let mut hops: Vec<Hop> = Vec::new();
    for (&(la, lb), list) in screened.iter_mut() {
        let through = (within(la, &ds) + within(lb, &dg)).min(within(lb, &ds) + within(la, &dg));
        let min_gap = list.iter().map(|h| h.gap).fold(f32::INFINITY, f32::min);
        if through + min_gap > budget {
            continue;
        }
        list.sort_by(|x, y| x.cost.total_cmp(&y.cost));
        let mut kept = 0;
        for h in list.drain(..) {
            if kept == PAIRS_PER_COMPONENT_PAIR {
                break;
            }
            let pu = {
                let c = plg.cells.coord(h.a);
                surface_point_xyz(c.0, c.1, c.2, voxel)
            };
            let pv = {
                let c = plg.cells.coord(h.b);
                surface_point_xyz(c.0, c.1, c.2, voxel)
            };
            let banned =
                |p: Xyz| state.is_blacklisted(p) || banned_aims.iter().any(|&b| dist3(b, p) < 1e-3);
            if banned(pu) || banned(pv) {
                continue;
            }
            if chord_clear(
                by_col,
                plg.cells.coord(h.a),
                plg.cells.coord(h.b),
                step_cells,
                headroom_cells,
            ) {
                hops.push(h);
                kept += 1;
            }
        }
    }
    hops
}

/// True when the bin or any 8-neighbor holds a component other than the
/// bin's own.
fn neighborhood_mixed(
    bins: &AHashMap<(i32, i32), BinData>,
    bx: i32,
    by: i32,
    bd: &BinData,
) -> bool {
    if bd.labels.len() > 1 {
        return true;
    }
    let l0 = bd.labels[0];
    for dbx in -1..=1 {
        for dby in -1..=1 {
            if let Some(nd) = bins.get(&(bx + dbx, by + dby)) {
                if nd.labels.iter().any(|&l| l != l0) {
                    return true;
                }
            }
        }
    }
    false
}

/// Least total bridged distance from the source component to every other,
/// over the screened pairs' minimum gaps. Intra-component travel is free
/// here, which only ever under-counts, so the prune never drops a chain the
/// exact gates would accept.
fn comp_gap_dist(screened: &AHashMap<(u32, u32), Vec<Hop>>, source: u32) -> AHashMap<u32, f32> {
    let mut adj: AHashMap<u32, Vec<(u32, f32)>> = AHashMap::new();
    for (&(la, lb), list) in screened {
        let min_gap = list.iter().map(|h| h.gap).fold(f32::INFINITY, f32::min);
        adj.entry(la).or_default().push((lb, min_gap));
        adj.entry(lb).or_default().push((la, min_gap));
    }
    let mut dist: AHashMap<u32, f32> = AHashMap::new();
    dist.insert(source, 0.0);
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    heap.push(Scored(0.0, source as usize));
    while let Some(Scored(d, u)) = heap.pop() {
        let u = u as u32;
        if d > dist.get(&u).copied().unwrap_or(f32::INFINITY) {
            continue;
        }
        if d > MAX_CHAIN_GAP_M {
            continue;
        }
        let Some(edges) = adj.get(&u) else { continue };
        for &(v, w) in edges {
            let nd = d + w;
            if nd < dist.get(&v).copied().unwrap_or(f32::INFINITY) {
                dist.insert(v, nd);
                heap.push(Scored(nd, v as usize));
            }
        }
    }
    dist
}

/// True when the cell lacks a surface edge in some 4-neighbor direction.
fn is_boundary(cells: &crate::adjacency::SurfaceCells, id: CellId) -> bool {
    let (cx, cy, _) = cells.coord(id);
    let mut mask = 0u8;
    for e in cells.neighbors(id) {
        let (nx, ny, _) = cells.coord(e.dest);
        mask |= match (nx - cx, ny - cy) {
            (-1, 0) => 1,
            (1, 0) => 2,
            (0, -1) => 4,
            (0, 1) => 8,
            _ => 0,
        };
    }
    mask != 15
}

/// True when no occupied voxel blocks the robot's body space above the
/// straight chord between the two cells. Empty columns pass: absence of data
/// is exactly what a bridge is optimistic about.
fn chord_clear(
    by_col: &ColumnIz,
    a: VoxelKey,
    b: VoxelKey,
    step_cells: i32,
    headroom_cells: i32,
) -> bool {
    let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let samples = dx.abs().max(dy.abs()) * 2;
    if samples == 0 {
        return true;
    }
    let (mut last_ix, mut last_iy) = (i32::MIN, i32::MIN);
    for k in 0..=samples {
        let t = k as f32 / samples as f32;
        let ix = (a.0 as f32 + t * dx as f32).round() as i32;
        let iy = (a.1 as f32 + t * dy as f32).round() as i32;
        if ix == last_ix && iy == last_iy {
            continue;
        }
        last_ix = ix;
        last_iy = iy;
        let Some(zs) = by_col.get(&(ix, iy)) else {
            continue;
        };
        let line = a.2 as f32 + t * dz as f32;
        for &z in zs {
            let above = z as f32 - line;
            if above > step_cells as f32 + 0.5 && above <= headroom_cells as f32 {
                return false;
            }
        }
    }
    true
}

/// The chain's aggregate-gate violation, as the index of its longest hop, or
/// none when the chain is admissible. Aggregates catch what per-hop gates
/// cannot: ladders of individually-legal hops that sum to an unclimbable wall.
fn chain_violation(hops: &[Hop], chain: &[usize], config: &Config) -> Option<usize> {
    let total_gap: f32 = chain.iter().map(|&i| hops[i].gap).sum();
    let total_rise: f32 = chain.iter().map(|&i| hops[i].rise).sum();
    let total_horiz: f32 = chain.iter().map(|&i| hops[i].horiz).sum();
    let ok = chain.len() <= MAX_CHAIN_HOPS
        && total_gap <= MAX_CHAIN_GAP_M
        && total_rise <= config.bridge_max_grade * total_horiz + config.step_threshold_m;
    if ok {
        return None;
    }
    chain
        .iter()
        .copied()
        .max_by(|&x, &y| hops[x].gap.total_cmp(&hops[y].gap))
}

/// Bridge hops on the path from a first hop through the goal component.
fn chain_of(pred: &[(usize, usize)], _hops: &[Hop], first_hop: usize, far_ep: usize) -> Vec<usize> {
    let mut chain = vec![first_hop];
    let mut cur = far_ep;
    while pred[cur].0 != usize::MAX {
        let (next, hi) = pred[cur];
        if hi != usize::MAX {
            chain.push(hi);
        }
        cur = next;
    }
    chain
}

/// Best first hop found by the endpoint-graph search.
struct Solution {
    best_hop: usize,
    best_far_ep: usize,
    best_total: f32,
    /// The held bridge's matching hop, far endpoint, and total, when it is
    /// still among the admissible first hops.
    held: Option<(usize, usize, f32)>,
    /// Predecessor (endpoint, hop or usize::MAX) toward the goal component.
    pred: Vec<(usize, usize)>,
}

/// Bridge endpoints as graph vertices: hops connect components, straight-line
/// links connect endpoints within a component.
struct EpGraph {
    index: AHashMap<CellId, usize>,
    cells: Vec<CellId>,
    labels: Vec<u32>,
    pos: Vec<Xyz>,
    /// (other endpoint, cost, hop index or usize::MAX for an intra link).
    adj: Vec<Vec<(usize, f32, usize)>>,
}

impl EpGraph {
    fn build(plg: &PlannerGraph, hops: &[Hop], voxel: f32) -> Self {
        let mut index: AHashMap<CellId, usize> = AHashMap::new();
        let mut cells: Vec<CellId> = Vec::new();
        let mut labels: Vec<u32> = Vec::new();
        let mut pos: Vec<Xyz> = Vec::new();
        let mut edges: Vec<(usize, usize, f32, usize)> = Vec::new();
        {
            let mut ep_of = |id: CellId, l: u32| -> usize {
                *index.entry(id).or_insert_with(|| {
                    let (ix, iy, iz) = plg.cells.coord(id);
                    cells.push(id);
                    labels.push(l);
                    pos.push(surface_point_xyz(ix, iy, iz, voxel));
                    labels.len() - 1
                })
            };
            for (hi, h) in hops.iter().enumerate() {
                let ea = ep_of(h.a, h.label_a);
                let eb = ep_of(h.b, h.label_b);
                edges.push((ea, eb, h.cost, hi));
            }
        }
        let n = labels.len();
        let mut adj: Vec<Vec<(usize, f32, usize)>> = vec![Vec::new(); n];
        for (ea, eb, cost, hi) in edges {
            adj[ea].push((eb, cost, hi));
            adj[eb].push((ea, cost, hi));
        }
        // Intra-component links between all endpoint pairs sharing a label.
        for i in 0..n {
            for j in (i + 1)..n {
                if labels[i] != labels[j] {
                    continue;
                }
                let d = dist3(pos[i], pos[j]);
                adj[i].push((j, d, usize::MAX));
                adj[j].push((i, d, usize::MAX));
            }
        }
        EpGraph {
            index,
            cells,
            labels,
            pos,
            adj,
        }
    }

    /// Dijkstra from the goal side, then score every admissible first hop as
    /// start cost + hop cost + remaining cost to the goal component.
    #[allow(clippy::too_many_arguments)]
    fn solve(
        &self,
        plg: &PlannerGraph,
        hops: &[Hop],
        banned: &AHashSet<usize>,
        reachable_mask: &[bool],
        start_label: u32,
        goal_label: u32,
        start_ctg: Option<&AHashMap<NodeId, f32>>,
        goal_ctg: Option<&AHashMap<NodeId, f32>>,
        start_pose: Xyz,
        goal_pose: Xyz,
        held_bridge: Option<Bridge>,
        config: &Config,
    ) -> Option<Solution> {
        let n = self.labels.len();
        let mut dist: Vec<f32> = vec![f32::INFINITY; n];
        let mut pred: Vec<(usize, usize)> = vec![(usize::MAX, usize::MAX); n];
        let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
        for (i, &l) in self.labels.iter().enumerate() {
            if l == goal_label {
                let d = terminal_cost(plg, goal_ctg, self.cells[i], goal_pose, config.voxel_size);
                dist[i] = d;
                heap.push(Scored(d, i));
            }
        }
        while let Some(Scored(d, u)) = heap.pop() {
            if d > dist[u] {
                continue;
            }
            for &(v, cost, hi) in &self.adj[u] {
                if hi != usize::MAX && banned.contains(&hi) {
                    continue;
                }
                let nd = d + cost;
                if nd < dist[v] {
                    dist[v] = nd;
                    pred[v] = (u, hi);
                    heap.push(Scored(nd, v));
                }
            }
        }

        let mut sol = Solution {
            best_hop: usize::MAX,
            best_far_ep: usize::MAX,
            best_total: f32::INFINITY,
            held: None,
            pred,
        };
        for (hi, h) in hops.iter().enumerate() {
            if banned.contains(&hi) {
                continue;
            }
            let (near, far) = if h.label_a == start_label {
                (h.a, h.b)
            } else if h.label_b == start_label {
                (h.b, h.a)
            } else {
                continue;
            };
            if !reachable_mask.get(near as usize).copied().unwrap_or(false) {
                continue;
            }
            let far_ep = self.index[&far];
            if !sol_dist_finite(&dist, far_ep) {
                continue;
            }
            let start_cost = terminal_cost(plg, start_ctg, near, start_pose, config.voxel_size);
            let total = start_cost + h.cost + dist[far_ep];
            if total < sol.best_total {
                sol.best_total = total;
                sol.best_hop = hi;
                sol.best_far_ep = far_ep;
            }
            if let Some((held_near, held_far)) = held_bridge {
                let near_ep = self.index[&near];
                if dist3(self.pos[near_ep], held_near) < SAME_BRIDGE_TOL_M
                    && dist3(self.pos[far_ep], held_far) < SAME_BRIDGE_TOL_M
                    && sol.held.is_none_or(|(_, _, t)| total < t)
                {
                    sol.held = Some((hi, far_ep, total));
                }
            }
        }
        (sol.best_hop != usize::MAX).then_some(sol)
    }
}

fn sol_dist_finite(dist: &[f32], ep: usize) -> bool {
    dist.get(ep).copied().unwrap_or(f32::INFINITY).is_finite()
}

/// Surface-aware cost from a terminal pose to a cell: the node-graph cost of
/// the cell's Voronoi node plus the cell's own distance to that node. Falls
/// back to straight-line distance where the fields do not cover the cell.
fn terminal_cost(
    plg: &PlannerGraph,
    ctg: Option<&AHashMap<NodeId, f32>>,
    id: CellId,
    fallback_from: Xyz,
    voxel: f32,
) -> f32 {
    let d = plg
        .cell_state
        .dist
        .get(id as usize)
        .copied()
        .unwrap_or(f32::INFINITY);
    if d.is_finite() {
        if let Some(ctg) = ctg {
            let src = plg.cell_state.source[id as usize];
            if let Some(&c) = ctg.get(&src) {
                return c + d;
            }
        }
    }
    let (ix, iy, iz) = plg.cells.coord(id);
    dist3(surface_point_xyz(ix, iy, iz, voxel), fallback_from)
}

fn dist3(a: Xyz, b: Xyz) -> f32 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt()
}

fn ground_dist(a: Xyz, b: Xyz) -> f32 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

struct Scored(f32, usize);

impl PartialEq for Scored {
    fn eq(&self, other: &Self) -> bool {
        self.0.total_cmp(&other.0) == Ordering::Equal && self.1 == other.1
    }
}
impl Eq for Scored {}
impl PartialOrd for Scored {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Scored {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mls_planner::Config;

    #[test]
    fn state_demotes_a_bridge_the_robot_traveled_to() {
        let mut s = BridgeState::default();
        let b: Bridge = ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0));
        // Adopted from afar, then the robot arrives and the components never
        // connect: the far endpoint must get demoted and blacklisted.
        s.note_bridge(b, (5.0, 0.0, 0.0));
        for _ in 0..STALL_REPLANS {
            s.note_bridge(b, (1.1, 0.0, 0.0));
        }
        assert!(s.held.is_none(), "stalled bridge must be dropped");
        assert!(s.is_blacklisted((2.1, 0.0, 0.0)));

        // The blacklist decays back out after enough replans.
        for _ in 0..BLACKLIST_REPLANS {
            s.note_no_bridge();
        }
        assert!(!s.is_blacklisted(b.1));
    }

    #[test]
    fn state_never_demotes_before_arrival() {
        let mut s = BridgeState::default();
        let b: Bridge = ((5.0, 0.0, 0.0), (6.0, 0.0, 0.0));
        for _ in 0..(STALL_REPLANS * 2) {
            s.note_bridge(b, (0.0, 0.0, 0.0));
        }
        assert!(s.held.is_some(), "en route to the bridge is not a stall");
        assert!(!s.is_blacklisted(b.1));
    }

    #[test]
    fn state_never_demotes_a_bridge_adopted_in_place() {
        let mut s = BridgeState::default();
        let b: Bridge = ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0));
        // The robot loiters at the near side the whole time: nothing was
        // attempted, so nothing gets demoted.
        for _ in 0..(STALL_REPLANS * 2) {
            s.note_bridge(b, (1.1, 0.0, 0.0));
        }
        assert!(s.held.is_some());
        assert!(!s.is_blacklisted(b.1));
    }

    #[test]
    fn chord_blocked_by_wall_but_not_by_unknown() {
        let mut by_col = ColumnIz::default();
        assert!(
            chord_clear(&by_col, (0, 0, 0), (10, 0, 0), 2, 5),
            "empty columns are unknown space, not a wall"
        );
        by_col.insert((5, 0), vec![4]);
        assert!(
            !chord_clear(&by_col, (0, 0, 0), (10, 0, 0), 2, 5),
            "a voxel in the body space above the chord blocks it"
        );
        by_col.insert((5, 0), vec![1]);
        assert!(
            chord_clear(&by_col, (0, 0, 0), (10, 0, 0), 2, 5),
            "a voxel within a step of the chord is walkable surface"
        );
        by_col.insert((5, 0), vec![20]);
        assert!(
            chord_clear(&by_col, (0, 0, 0), (10, 0, 0), 2, 5),
            "a ceiling above the headroom does not block"
        );
    }

    fn flat_hop(gap: f32, horiz: f32, rise: f32) -> Hop {
        Hop {
            a: 0,
            b: 1,
            label_a: 0,
            label_b: 1,
            gap,
            horiz,
            rise,
            cost: gap,
        }
    }

    #[test]
    fn chain_gates_reject_ladders_and_marathons() {
        let mut config = Config {
            world_frame: String::new(),
            voxel_size: 0.1,
            robot_height: 0.3,
            max_overhead_m: 2.0,
            surface_closing_radius: 0.0,
            node_spacing_m: 1.0,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.0,
            wall_buffer_weight: 0.0,
            step_threshold_m: 0.25,
            step_penalty_weight: 0.0,
            bridge_max_hop_m: 1.0,
            bridge_max_grade: 0.85,
            bridge_cost_weight: 4.0,
            bridge_switch_margin: 0.25,
            goal_tolerance: 0.3,
            viz_publish_hz: 0.0,
        };
        config.step_threshold_m = 0.25;

        // A gentle stair chain passes.
        let stairs: Vec<Hop> = (0..4).map(|_| flat_hop(0.58, 0.5, 0.3)).collect();
        let idx: Vec<usize> = (0..4).collect();
        assert!(chain_violation(&stairs, &idx, &config).is_none());

        // The same rise packed into half the run is a ladder: rejected.
        let ladder: Vec<Hop> = (0..4).map(|_| flat_hop(0.45, 0.2, 0.4)).collect();
        assert!(chain_violation(&ladder, &idx, &config).is_some());

        // Too much total unobserved distance: rejected.
        let marathon: Vec<Hop> = (0..4).map(|_| flat_hop(0.9, 0.9, 0.0)).collect();
        assert!(chain_violation(&marathon, &idx, &config).is_some());

        // Too many hops: rejected.
        let crumbs: Vec<Hop> = (0..5).map(|_| flat_hop(0.3, 0.3, 0.0)).collect();
        let idx5: Vec<usize> = (0..5).collect();
        assert!(chain_violation(&crumbs, &idx5, &config).is_some());
    }
}
