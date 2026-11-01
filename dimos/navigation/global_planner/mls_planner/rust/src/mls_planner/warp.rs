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

//! Move the whole planner into a corrected frame in place: every voxel, node,
//! the cached path and any held points go through the warp, and only what
//! the grid derives from them is repaired, around what moved.

use super::*;
use crate::reform::{landing_keys, with_chained, Warp};
use crate::voxel::surface_point_xyz;

/// A point's place before and after a warp.
pub type Displacement = ([f32; 3], [f32; 3]);

/// What one warp did and what each stage cost.
#[derive(Clone, Debug, Default)]
pub struct WarpReport {
    /// Voxels the warp moved in place.
    pub voxels_warped: usize,
    /// Surface cells the warp moved in place.
    pub cells_warped: usize,
    pub nodes: usize,
    pub warp_ms: f64,
    pub move_ms: f64,
    /// Points the warp sent somewhere non-finite or implausibly far, left in place.
    pub implausible: usize,
    /// The worst of those: where it was, where the warp sent it.
    pub worst_implausible: Option<Displacement>,
}

/// Leave in place every warped point that is non-finite or moved farther than
/// `max_move_m`, counting them and keeping the worst for the log.
pub(crate) fn hold_implausible(
    queries: &[[f32; 3]],
    warped: &mut [[f32; 3]],
    max_move_m: f64,
) -> (usize, Option<Displacement>) {
    let mut count = 0;
    let mut worst: Option<(f64, [f32; 3], [f32; 3])> = None;
    for (from, to) in queries.iter().zip(warped.iter_mut()) {
        let moved = ((to[0] - from[0]) as f64)
            .hypot((to[1] - from[1]) as f64)
            .hypot((to[2] - from[2]) as f64);
        let finite = to.iter().all(|v| v.is_finite());
        if finite && moved <= max_move_m {
            continue;
        }
        count += 1;
        let score = if finite { moved } else { f64::INFINITY };
        if worst.is_none_or(|(w, _, _)| score > w) {
            worst = Some((score, *from, *to));
        }
        *to = *from;
    }
    (count, worst.map(|(_, from, to)| (from, to)))
}

impl Planner {
    /// Carry the planner through a warp: fast-reform moves every voxel's
    /// point, every surface cell and node, and `held` (points the caller keeps
    /// in the map frame, such as the active goal) in place. Keys, edges and
    /// nodes stay as they are; a region update re-keys what it reaches.
    pub fn apply_warp(
        &mut self,
        warp: &Warp,
        held: &mut [(f32, f32, f32)],
        config: &Config,
    ) -> WarpReport {
        let pool = Arc::clone(&self.pool);
        pool.install(|| self.apply_warp_in_pool(warp, held, config))
    }

    fn apply_warp_in_pool(
        &mut self,
        warp: &Warp,
        held: &mut [(f32, f32, f32)],
        config: &Config,
    ) -> WarpReport {
        let vs = config.voxel_size;
        let mut report = WarpReport::default();
        let stage = Instant::now();
        // Voxel points off their keys live with the voxel times.
        self.track_voxel_times();
        let voxels: Vec<VoxelKey> = self.voxel_map.iter().copied().collect();
        let cells: Vec<CellId> = self.graph.cells.ids().collect();
        let path_goal = self.last_path.as_ref().map(|(goal, _)| *goal);
        let time_of = |k: &VoxelKey| match &self.voxel_times {
            Some(times) => times.get(k),
            None => f64::NAN,
        };
        let residual = |k: &VoxelKey| {
            self.voxel_times
                .as_ref()
                .and_then(|t| t.residual.get(k).copied())
                .unwrap_or([0.0; 3])
        };
        let mut queries: Vec<[f32; 3]> = voxels
            .par_iter()
            .map(|k| {
                let r = residual(k);
                [0, 1, 2].map(|a| ([k.0, k.1, k.2][a] as f32 + 0.5) * vs + r[a])
            })
            .collect();
        let mut times: Vec<f64> = voxels.par_iter().map(time_of).collect();
        let graph_cells = &self.graph.cells;
        queries.par_extend(cells.par_iter().map(|&id| {
            let (x, y, z) = surface_point_xyz_of(graph_cells.coord(id), vs);
            let o = graph_cells.offset(id);
            [x + o[0], y + o[1], z + o[2]]
        }));
        times.par_extend(cells.par_iter().map(|&id| time_of(&graph_cells.coord(id))));
        for p in held.iter().chain(path_goal.as_ref()) {
            queries.push([p.0, p.1, p.2]);
            times.push(time_of(&self.surface_under(*p, vs)));
        }
        let mut warped = warp.apply(&queries, &times);
        (report.implausible, report.worst_implausible) =
            hold_implausible(&queries, &mut warped, warp.max_plausible_move_m());
        report.warp_ms = ms_since(stage);

        let stage = Instant::now();
        let delta = |i: usize| [0, 1, 2].map(|a| warped[i][a] - queries[i][a]);
        if let Some(t) = &mut self.voxel_times {
            for (i, k) in voxels.iter().enumerate() {
                let d = delta(i);
                if d != [0.0; 3] {
                    let r = t.residual.entry(*k).or_insert([0.0; 3]);
                    for a in 0..3 {
                        r[a] += d[a];
                    }
                }
            }
        }
        let cell_deltas: Vec<[f32; 3]> =
            (0..cells.len()).map(|i| delta(voxels.len() + i)).collect();
        self.graph.cells.shift(&cells, &cell_deltas);
        self.graph.nodes.follow_cells(&self.graph.cells, vs);
        let moved_held = &warped[voxels.len() + cells.len()..];
        for (p, w) in held.iter_mut().zip(moved_held) {
            *p = (w[0], w[1], w[2]);
        }
        if let Some((goal, _)) = &mut self.last_path {
            let w = moved_held.last().expect("the path goal was queried last");
            *goal = (w[0], w[1], w[2]);
        }
        self.displaced = None;
        report.voxels_warped = voxels.len();
        report.cells_warped = cells.len();
        report.nodes = self.graph.nodes.len();
        report.move_ms = ms_since(stage);
        report
    }

    /// Re-key the voxels and surface cells a warp moved out of their keys
    /// within `radius_m` of (x, y), by where their points sit now, so a region
    /// update there meets them in place. A cell that lands on another that was
    /// not its neighbor before is dropped, as two passes the closure brought
    /// together; the region's own update settles the surface there.
    pub(crate) fn rebin_near(&mut self, at: (f32, f32), radius_m: f32, config: &Config) {
        self.rebin_around(&[at], radius_m, config);
    }

    /// rebin_near around every one of `centers`.
    pub(crate) fn rebin_around(&mut self, centers: &[(f32, f32)], radius_m: f32, config: &Config) {
        let vs = config.voxel_size;
        let half = 0.5 * vs;
        let r2 = radius_m * radius_m;
        let bin = |x: f32, y: f32| ((x / radius_m).floor() as i32, (y / radius_m).floor() as i32);
        let mut by_bin: AHashMap<(i32, i32), Vec<(f32, f32)>> = AHashMap::new();
        for &(x, y) in centers {
            by_bin.entry(bin(x, y)).or_default().push((x, y));
        }
        let near = |p: [f32; 3]| {
            let (bx, by) = bin(p[0], p[1]);
            (-1..=1).any(|dx| {
                (-1..=1).any(|dy| {
                    by_bin.get(&(bx + dx, by + dy)).is_some_and(|cs| {
                        cs.iter()
                            .any(|&(x, y)| (p[0] - x).powi(2) + (p[1] - y).powi(2) <= r2)
                    })
                })
            })
        };
        let center = |k: VoxelKey| [k.0, k.1, k.2].map(|v| (v as f32 + 0.5) * vs);
        let displaced = |o: &[f32; 3]| o.iter().any(|v| v.abs() > half);

        if self.displaced.is_none() {
            self.displaced = Some(self.index_displaced(radius_m, vs));
        }
        let (voxel_candidates, cell_candidates) = self
            .displaced
            .as_ref()
            .expect("built above")
            .near(centers, radius_m);
        let moving: Vec<(VoxelKey, [f32; 3])> = match &self.voxel_times {
            Some(times) => voxel_candidates
                .into_iter()
                .filter_map(|k| {
                    let r = times.residual.get(&k)?;
                    displaced(r).then(|| {
                        let c = center(k);
                        (k, [c[0] + r[0], c[1] + r[1], c[2] + r[2]])
                    })
                })
                .filter(|&(k, p)| near(p) || near(center(k)))
                .collect(),
            None => Vec::new(),
        };
        if !moving.is_empty() {
            let point_of = |k: &VoxelKey| {
                let r = self.voxel_times.as_ref()?.residual.get(k)?;
                let c = center(*k);
                Some([c[0] + r[0], c[1] + r[1], c[2] + r[2]])
            };
            let keys = with_chained(moving.into_iter().map(|m| m.0).collect(), |k| {
                let p = point_of(k)?;
                let to = voxelize((p[0], p[1], p[2]), vs);
                (to != *k).then_some(to)
            });
            let points: Vec<[f32; 3]> = keys.iter().map(|k| point_of(k).unwrap()).collect();
            let moving_set: AHashSet<VoxelKey> = keys.iter().copied().collect();
            let landed = landing_keys(&keys, &points, vs, |k| {
                self.voxel_map.contains(k) && !moving_set.contains(k)
            });
            self.rekey_voxels(&keys, &landed, &points, vs);
        }

        let cells = &self.graph.cells;
        let moving: Vec<(CellId, [f32; 3])> = cell_candidates
            .into_iter()
            .filter(|&id| cells.is_live(id) && displaced(&cells.offset(id)))
            .map(|id| {
                let c = center(cells.coord(id));
                let o = cells.offset(id);
                (id, [c[0] + o[0], c[1] + o[1], c[2] + o[2]])
            })
            .filter(|&(id, p)| near(p) || near(center(cells.coord(id))))
            .collect();
        if moving.is_empty() {
            return;
        }
        let cell_point = |id: CellId| {
            let c = center(cells.coord(id));
            let o = cells.offset(id);
            [c[0] + o[0], c[1] + o[1], c[2] + o[2]]
        };
        let keys = with_chained(moving.iter().map(|m| cells.coord(m.0)).collect(), |k| {
            let id = cells.id(*k)?;
            let placed = cells.placed(id, vs);
            (placed != *k).then_some(placed)
        });
        let moving: Vec<(CellId, [f32; 3])> = keys
            .iter()
            .map(|k| {
                let id = cells.id(*k).expect("a chained cell exists");
                (id, cell_point(id))
            })
            .collect();
        let points: Vec<[f32; 3]> = moving.iter().map(|m| m.1).collect();
        let moving_set: AHashSet<VoxelKey> = keys.iter().copied().collect();
        let landed = landing_keys(&keys, &points, vs, |k| {
            cells.id(*k).is_some() && !moving_set.contains(k)
        });
        // Strangers that met: every mover whose key a staying cell or an earlier mover holds.
        let mut taken: AHashSet<VoxelKey> = AHashSet::with_capacity(landed.len());
        let mut dropped: Vec<VoxelKey> = Vec::new();
        let mut moves: Vec<(CellId, VoxelKey)> = Vec::new();
        for ((&(id, _), &from), &to) in moving.iter().zip(&keys).zip(&landed) {
            let held = cells.id(to).is_some() && !moving_set.contains(&to);
            if held || !taken.insert(to) {
                dropped.push(from);
            } else if to != from {
                moves.push((id, to));
            }
        }
        if !dropped.is_empty() {
            for k in &dropped {
                if let Some(zs) = self.graph.surface_lookup.get_mut(&(k.0, k.1)) {
                    zs.retain(|&z| z != k.2);
                    if zs.is_empty() {
                        self.graph.surface_lookup.remove(&(k.0, k.1));
                    }
                }
            }
            self.rebuild_region_graph(
                Vec::new(),
                dropped,
                &VoxelEdits::default(),
                &[],
                config.node_window_cells(),
                config,
            );
        }
        let graph = &mut self.graph;
        for &(id, _) in &moves {
            let from = graph.cells.coord(id);
            if let Some(zs) = graph.surface_lookup.get_mut(&(from.0, from.1)) {
                zs.retain(|&z| z != from.2);
                if zs.is_empty() {
                    graph.surface_lookup.remove(&(from.0, from.1));
                }
            }
        }
        for &(_, to) in &moves {
            let zs = graph.surface_lookup.entry((to.0, to.1)).or_default();
            if let Err(at) = zs.binary_search(&to.2) {
                zs.insert(at, to.2);
            }
        }
        let renamed: AHashMap<VoxelKey, VoxelKey> = moves
            .iter()
            .map(|&(id, to)| (graph.cells.coord(id), to))
            .collect();
        graph.cells.rekey(&moves, vs);
        graph.node_edges.rename_cells(&renamed);
        if let Some((_, path)) = &mut self.last_path {
            for c in path.iter_mut() {
                if let Some(&to) = renamed.get(c) {
                    *c = to;
                }
            }
        }
        graph.nodes.rebin(&graph.cells, &config.placement_params());
    }

    /// Bucket every voxel and surface cell a warp left off its key, by its point and
    /// by its key, in buckets `side_m` wide.
    fn index_displaced(&self, side_m: f32, vs: f32) -> DisplacedIndex {
        let half = 0.5 * vs;
        let displaced = |o: &[f32; 3]| o.iter().any(|v| v.abs() > half);
        let mut index = DisplacedIndex {
            side_m,
            voxels: AHashMap::new(),
            cells: AHashMap::new(),
        };
        let center = |k: VoxelKey| [k.0, k.1, k.2].map(|v| (v as f32 + 0.5) * vs);
        if let Some(times) = &self.voxel_times {
            for (&k, r) in times.residual.iter().filter(|(_, r)| displaced(r)) {
                let c = center(k);
                let at = index.bucket(c[0], c[1]);
                let to = index.bucket(c[0] + r[0], c[1] + r[1]);
                index.voxels.entry(at).or_default().push(k);
                if to != at {
                    index.voxels.entry(to).or_default().push(k);
                }
            }
        }
        let cells = &self.graph.cells;
        for id in cells.ids().filter(|&id| displaced(&cells.offset(id))) {
            let c = center(cells.coord(id));
            let o = cells.offset(id);
            let at = index.bucket(c[0], c[1]);
            let to = index.bucket(c[0] + o[0], c[1] + o[1]);
            index.cells.entry(at).or_default().push(id);
            if to != at {
                index.cells.entry(to).or_default().push(id);
            }
        }
        index
    }

    /// Move voxels to the keys their points landed in, with their times and
    /// what is left of their offsets.
    fn rekey_voxels(
        &mut self,
        keys: &[VoxelKey],
        landed: &[VoxelKey],
        points: &[[f32; 3]],
        vs: f32,
    ) {
        let times = self
            .voxel_times
            .as_mut()
            .expect("only a warp moves voxels off their keys");
        let moved: Vec<(VoxelKey, Option<f32>)> = keys
            .iter()
            .map(|k| {
                times.residual.remove(k);
                (*k, times.by_key.remove(k))
            })
            .collect();
        for k in keys {
            self.voxel_map.remove(k);
            self.by_col.remove(*k);
        }
        for ((&to, p), (_, t)) in landed.iter().zip(points).zip(moved) {
            if self.voxel_map.insert(to) {
                self.by_col.add(to);
            }
            if let Some(t) = t {
                times
                    .by_key
                    .entry(to)
                    .and_modify(|e| *e = e.min(t))
                    .or_insert(t);
            }
            let c = [to.0, to.1, to.2].map(|v| (v as f32 + 0.5) * vs);
            times
                .residual
                .entry(to)
                .or_insert([p[0] - c[0], p[1] - c[1], p[2] - c[2]]);
        }
    }

    /// Waypoints planned over cell keys, placed where a warp moved their cells:
    /// each takes the offset of the path cell nearest it.
    pub(crate) fn placed_waypoints(
        &self,
        waypoints: Vec<(f32, f32, f32)>,
        path: &[VoxelKey],
        vs: f32,
    ) -> Vec<(f32, f32, f32)> {
        let cells = &self.graph.cells;
        let offsets: Vec<(VoxelKey, [f32; 3])> = path
            .iter()
            .map(|&k| (k, cells.id(k).map_or([0.0; 3], |id| cells.offset(id))))
            .collect();
        if offsets.iter().all(|(_, o)| *o == [0.0; 3]) {
            return waypoints;
        }
        waypoints
            .into_iter()
            .map(|p| {
                let k = surface_key(&[p.0, p.1, p.2], vs);
                let o = offsets
                    .iter()
                    .min_by_key(|(c, _)| {
                        (c.0 - k.0).pow(2) + (c.1 - k.1).pow(2) + (c.2 - k.2).pow(2)
                    })
                    .map_or([0.0; 3], |(_, o)| *o);
                (p.0 + o[0], p.1 + o[1], p.2 + o[2])
            })
            .collect()
    }

    /// The surface cell a point sits on: the voxel under it, else its own.
    fn surface_under(&self, p: (f32, f32, f32), vs: f32) -> VoxelKey {
        let below = surface_key(&[p.0, p.1, p.2], vs);
        if self.voxel_map.contains(&below) {
            below
        } else {
            voxelize(p, vs)
        }
    }
}

fn surface_point_xyz_of(k: VoxelKey, vs: f32) -> (f32, f32, f32) {
    surface_point_xyz(k.0, k.1, k.2, vs)
}

/// The surface cell whose top face a point sits on.
fn surface_key(p: &[f32; 3], vs: f32) -> VoxelKey {
    (
        (p[0] / vs).floor() as i32,
        (p[1] / vs).floor() as i32,
        (p[2] / vs).round() as i32 - 1,
    )
}

#[cfg(test)]
mod tests;

/// The voxels and surface cells a warp left off their keys, bucketed both by where
/// their point now sits and by their key, so a region update finds the ones near it
/// without scanning the map. Entries go stale as voxels are re-keyed or replaced;
/// callers re-check each candidate.
pub(crate) struct DisplacedIndex {
    side_m: f32,
    voxels: AHashMap<(i32, i32), Vec<VoxelKey>>,
    cells: AHashMap<(i32, i32), Vec<CellId>>,
}

impl DisplacedIndex {
    fn bucket(&self, x: f32, y: f32) -> (i32, i32) {
        (
            (x / self.side_m).floor() as i32,
            (y / self.side_m).floor() as i32,
        )
    }

    /// Every entry in the buckets a disc of `radius_m` around each center touches.
    fn near(
        &self,
        centers: &[(f32, f32)],
        radius_m: f32,
    ) -> (AHashSet<VoxelKey>, AHashSet<CellId>) {
        let reach = (radius_m / self.side_m).ceil() as i32;
        let mut voxels = AHashSet::new();
        let mut cells = AHashSet::new();
        for &(x, y) in centers {
            let (bx, by) = self.bucket(x, y);
            for dx in -reach..=reach {
                for dy in -reach..=reach {
                    if let Some(keys) = self.voxels.get(&(bx + dx, by + dy)) {
                        voxels.extend(keys.iter().copied());
                    }
                    if let Some(ids) = self.cells.get(&(bx + dx, by + dy)) {
                        cells.extend(ids.iter().copied());
                    }
                }
            }
        }
        (voxels, cells)
    }
}
