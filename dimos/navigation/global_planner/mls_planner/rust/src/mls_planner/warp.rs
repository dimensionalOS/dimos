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
use crate::columns::ColumnEdit;
use crate::reform::{Tile, Warp};
use crate::voxel::surface_point_xyz;

/// A point's place before and after a warp.
pub type Displacement = ([f32; 3], [f32; 3]);

/// What one warp did and what each stage cost.
#[derive(Clone, Debug, Default)]
pub struct WarpReport {
    /// Voxels in tiles the warp reaches.
    pub voxels_warped: usize,
    /// Of those, the ones that landed in another voxel.
    pub voxels_moved: usize,
    pub voxels_added: usize,
    pub voxels_removed: usize,
    pub surface_added: usize,
    pub surface_removed: usize,
    pub nodes_moved: usize,
    pub nodes: usize,
    /// Whether so much moved that the graph was rebuilt from the warped
    /// voxels instead of repaired, re-placing every node.
    pub rebuilt: bool,
    pub warp_ms: f64,
    pub voxel_ms: f64,
    pub surface_ms: f64,
    pub graph_ms: f64,
    /// Warped voxels in tiles that, with all their neighbors, moved by one whole-voxel
    /// offset: what a rigid per-tile translation could move without recomputing.
    pub rigid_voxels: usize,
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
    /// Carry the planner through a warp. `held` are points the caller keeps in
    /// the map frame, such as the active goal, warped in place alongside.
    /// When more than the warp's rebuild fraction of the voxels change,
    /// repairing around all of them costs more than a rebuild, so the graph
    /// is rebuilt from the warped voxels instead.
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

        // Which tiles the warp reaches, decided once per tile.
        let mut tiles: Vec<Tile> = self
            .by_col
            .par_columns()
            .map(|((ix, iy), _)| warp.tile((ix as f32 + 0.5) * vs, (iy as f32 + 0.5) * vs))
            .collect();
        tiles.par_sort_unstable();
        tiles.dedup();
        let reached: AHashSet<Tile> = tiles
            .into_par_iter()
            .filter(|&t| warp.affects_tile(t))
            .collect::<Vec<_>>()
            .into_iter()
            .collect();

        let voxels: Vec<VoxelKey> = self
            .by_col
            .par_columns()
            .filter(|((ix, iy), _)| {
                reached.contains(&warp.tile((*ix as f32 + 0.5) * vs, (*iy as f32 + 0.5) * vs))
            })
            .flat_map_iter(|((ix, iy), zs)| zs.iter().map(move |&iz| (ix, iy, iz)))
            .collect();
        report.voxels_warped = voxels.len();

        // Everything warps in one batch: voxel centers, nodes and path cells at
        // their surface, then the held points. Each takes its voxel's time.
        // Whether the warp reaches a node is decided once per tile, as for the voxels.
        let node_tile = |c: &VoxelKey| warp.tile((c.0 as f32 + 0.5) * vs, (c.1 as f32 + 0.5) * vs);
        let all_nodes: Vec<(usize, VoxelKey)> = self
            .graph
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (i, self.graph.cells.coord(n.cell_id)))
            .collect();
        let mut node_tiles: Vec<Tile> = all_nodes.iter().map(|(_, c)| node_tile(c)).collect();
        node_tiles.sort_unstable();
        node_tiles.dedup();
        let nodes_reached: AHashSet<Tile> = node_tiles
            .into_par_iter()
            .filter(|&t| warp.affects_tile(t))
            .collect::<Vec<_>>()
            .into_iter()
            .collect();
        let node_at: Vec<(usize, VoxelKey)> = all_nodes
            .into_iter()
            .filter(|(_, c)| nodes_reached.contains(&node_tile(c)))
            .collect();
        let path_cells: Vec<VoxelKey> = self
            .last_path
            .as_ref()
            .map(|(_, cells)| cells.clone())
            .unwrap_or_default();
        let path_goal = self.last_path.as_ref().map(|(goal, _)| *goal);
        let time_of = |k: &VoxelKey| match &self.voxel_times {
            Some(times) => times.get(k),
            None => f64::NAN,
        };
        let surface = |&(ix, iy, iz): &VoxelKey| {
            let (x, y, z) = surface_point_xyz(ix, iy, iz, vs);
            [x, y, z]
        };
        // Each voxel's point (its center plus residual) and time, looked up in parallel.
        let (mut queries, mut times): (Vec<[f32; 3]>, Vec<f64>) = voxels
            .par_iter()
            .map(|k| {
                let r = self
                    .voxel_times
                    .as_ref()
                    .and_then(|t| t.residual.get(k).copied())
                    .unwrap_or([0.0; 3]);
                (
                    [
                        (k.0 as f32 + 0.5) * vs + r[0],
                        (k.1 as f32 + 0.5) * vs + r[1],
                        (k.2 as f32 + 0.5) * vs + r[2],
                    ],
                    time_of(k),
                )
            })
            .unzip();
        queries.reserve(node_at.len());
        times.reserve(node_at.len());
        for (_, c) in &node_at {
            queries.push(surface(c));
            times.push(time_of(c));
        }
        for c in &path_cells {
            queries.push(surface(c));
            times.push(time_of(c));
        }
        for p in held.iter().chain(path_goal.as_ref()) {
            queries.push([p.0, p.1, p.2]);
            times.push(time_of(&self.surface_under(*p, vs)));
        }
        let mut warped = warp.apply(&queries, &times);
        (report.implausible, report.worst_implausible) =
            hold_implausible(&queries, &mut warped, warp.max_plausible_move_m());
        report.warp_ms = ms_since(stage);

        let (moved_voxels, rest) = warped.split_at(voxels.len());
        // Each warped voxel's point after the move, off the center of the voxel it lands in.
        let residuals: Vec<(VoxelKey, [f32; 3])> = moved_voxels
            .iter()
            .map(|w| {
                let to = voxelize((w[0], w[1], w[2]), vs);
                let c = [to.0, to.1, to.2].map(|k| (k as f32 + 0.5) * vs);
                (to, [w[0] - c[0], w[1] - c[1], w[2] - c[2]])
            })
            .collect();
        let (moved_nodes, rest) = rest.split_at(node_at.len());
        let (moved_path, moved_held) = rest.split_at(path_cells.len());
        for (p, w) in held.iter_mut().zip(moved_held) {
            *p = (w[0], w[1], w[2]);
        }
        if let Some((goal, cells)) = &mut self.last_path {
            *goal = moved_held
                .last()
                .map(|w| (w[0], w[1], w[2]))
                .expect("the path goal was queried last");
            let mut keys: Vec<VoxelKey> = moved_path.iter().map(|w| surface_key(w, vs)).collect();
            keys.dedup();
            *cells = keys;
        }

        report.rigid_voxels = rigid_tile_voxels(&voxels, moved_voxels, vs, warp);
        let moved_count = voxels
            .par_iter()
            .zip(moved_voxels.par_iter())
            .filter(|(k, w)| voxelize((w[0], w[1], w[2]), vs) != **k)
            .count();
        if moved_count as f64 > warp.rebuild_fraction() * self.voxel_map.len() as f64 {
            let stage = Instant::now();
            self.rekey_voxels(&voxels, moved_voxels, vs);
            self.set_residuals(&voxels, &residuals);
            report.voxels_moved = moved_count;
            report.voxel_ms = ms_since(stage);
            let stage = Instant::now();
            self.rebuild_from_voxels(config);
            report.graph_ms = ms_since(stage);
            report.rebuilt = true;
            report.nodes = self.graph.nodes.len();
            return report;
        }

        let stage = Instant::now();
        let edits = self.warp_voxels(&voxels, moved_voxels, vs, &mut report);
        self.set_residuals(&voxels, &residuals);
        report.voxel_ms = ms_since(stage);

        let moves: Vec<(usize, VoxelKey)> = node_at
            .iter()
            .zip(moved_nodes)
            .filter_map(|(&(i, from), w)| {
                let to = surface_key(w, vs);
                (to != from).then_some((i, to))
            })
            .collect();
        report.nodes_moved = moves.len();
        if edits.is_empty() && moves.is_empty() {
            report.nodes = self.graph.nodes.len();
            return report;
        }

        let stage = Instant::now();
        let (added, removed) = self.resurface(&edits, config);
        report.surface_added = added.len();
        report.surface_removed = removed.len();
        report.surface_ms = ms_since(stage);

        let stage = Instant::now();
        self.rebuild_region_graph(
            added,
            removed,
            &edits,
            &moves,
            config.buffer_window_cells(),
            config,
        );
        report.graph_ms = ms_since(stage);
        report.nodes = self.graph.nodes.len();
        report
    }

    /// Move every warped voxel to its new key at once, for a rebuild: each
    /// leaves its old key and lands in the new one with its time.
    fn rekey_voxels(&mut self, voxels: &[VoxelKey], warped: &[[f32; 3]], vs: f32) {
        let landed: Vec<VoxelKey> = warped
            .par_iter()
            .map(|w| voxelize((w[0], w[1], w[2]), vs))
            .collect();
        let times: Vec<f32> = match &self.voxel_times {
            Some(t) => voxels
                .iter()
                .map(|k| t.by_key.get(k).copied().unwrap_or(f32::NAN))
                .collect(),
            None => Vec::new(),
        };
        for k in voxels {
            self.voxel_map.remove(k);
        }
        self.voxel_map.extend(landed.iter().copied());
        if let Some(t) = &mut self.voxel_times {
            for k in voxels {
                t.by_key.remove(k);
            }
            for (k, time) in landed.into_iter().zip(times) {
                if time.is_finite() {
                    t.by_key
                        .entry(k)
                        .and_modify(|e| *e = e.min(time))
                        .or_insert(time);
                }
            }
        }
    }

    /// The warped voxels' residuals at the keys they landed in; the first to land in a key keeps it.
    fn set_residuals(&mut self, warped: &[VoxelKey], residuals: &[(VoxelKey, [f32; 3])]) {
        let Some(times) = &mut self.voxel_times else {
            return;
        };
        for k in warped {
            times.residual.remove(k);
        }
        for &(to, r) in residuals {
            if self.voxel_map.contains(&to) {
                times.residual.entry(to).or_insert(r);
            }
        }
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

    /// Re-key the warped voxels and apply the net change: a vacated voxel
    /// stays when another lands in it.
    fn warp_voxels(
        &mut self,
        voxels: &[VoxelKey],
        warped: &[[f32; 3]],
        vs: f32,
        report: &mut WarpReport,
    ) -> VoxelEdits {
        let mut vacated: AHashSet<VoxelKey> = AHashSet::new();
        // Each landing voxel keeps the earliest time of those landing in it.
        let mut landed: AHashMap<VoxelKey, f32> = AHashMap::new();
        for (from, w) in voxels.iter().zip(warped) {
            let to = voxelize((w[0], w[1], w[2]), vs);
            if to == *from {
                continue;
            }
            report.voxels_moved += 1;
            vacated.insert(*from);
            let t = self
                .voxel_times
                .as_ref()
                .and_then(|times| times.by_key.get(from).copied())
                .unwrap_or(f32::NAN);
            landed.entry(to).and_modify(|e| *e = e.min(t)).or_insert(t);
        }

        let mut by_col: AHashMap<(i32, i32), (Vec<i32>, Vec<i32>)> = AHashMap::new();
        for k in &vacated {
            if !landed.contains_key(k) {
                by_col.entry((k.0, k.1)).or_default().0.push(k.2);
            }
        }
        for k in landed.keys() {
            if !self.voxel_map.contains(k) {
                by_col.entry((k.0, k.1)).or_default().1.push(k.2);
            }
        }
        let edits: Vec<ColumnEdit> = by_col
            .into_iter()
            .map(|(col, (mut removed, mut added))| {
                removed.sort_unstable();
                added.sort_unstable();
                ColumnEdit {
                    col,
                    removed,
                    added,
                }
            })
            .collect();
        report.voxels_removed = edits.iter().map(|e| e.removed.len()).sum();
        report.voxels_added = edits.iter().map(|e| e.added.len()).sum();
        // A voxel a mover now holds alone takes the mover's time.
        let retimed: Vec<(VoxelKey, f32)> = landed
            .into_iter()
            .filter(|(k, _)| vacated.contains(k) || !self.voxel_map.contains(k))
            .collect();
        let voxel_edits = self.apply_column_edits(&edits);
        if let Some(times) = &mut self.voxel_times {
            for (k, t) in retimed {
                if t.is_finite() {
                    times.by_key.insert(k, t);
                } else {
                    times.by_key.remove(&k);
                }
            }
        }
        voxel_edits
    }

    /// Re-extract the surface around the edited columns, as a region update
    /// does. Returns the surface cells added and removed.
    fn resurface(&mut self, edits: &VoxelEdits, config: &Config) -> (Vec<VoxelKey>, Vec<VoxelKey>) {
        let mut cols = edits.columns();
        let Some(first) = cols.next() else {
            return (Vec::new(), Vec::new());
        };
        let mut bbox = (first.0, first.0, first.1, first.1);
        for (ix, iy) in cols {
            bbox = (
                bbox.0.min(ix),
                bbox.1.max(ix),
                bbox.2.min(iy),
                bbox.3.max(iy),
            );
        }
        let pad = (2 * config.closing_passes()) as i32;
        let mut footprint = ColumnMask::new(bbox, 2 * pad);
        for col in edits.columns() {
            footprint.set(col);
        }
        let write = footprint.dilated(pad);
        let new_cells = extract_surfaces_region(
            &self.by_col,
            config.headroom_cells(),
            config.closing_passes(),
            &write,
        );
        self.replace_surface_region(&write, &new_cells)
    }
}

/// Voxels in tiles whose voxels all moved by one whole-voxel offset, shared by every neighboring tile.
fn rigid_tile_voxels(voxels: &[VoxelKey], warped: &[[f32; 3]], vs: f32, warp: &Warp) -> usize {
    // Per tile: its voxel count and its single offset, None once two differ.
    let mut tiles: AHashMap<Tile, (usize, Option<VoxelKey>)> = AHashMap::new();
    for (k, w) in voxels.iter().zip(warped) {
        let to = voxelize((w[0], w[1], w[2]), vs);
        let offset = (to.0 - k.0, to.1 - k.1, to.2 - k.2);
        let tile = warp.tile((k.0 as f32 + 0.5) * vs, (k.1 as f32 + 0.5) * vs);
        let entry = tiles.entry(tile).or_insert((0, Some(offset)));
        entry.0 += 1;
        if entry.1 != Some(offset) {
            entry.1 = None;
        }
    }
    tiles
        .iter()
        .filter(|(&(tx, ty), (_, offset))| {
            offset.is_some()
                && (-1..=1).all(|dx| {
                    (-1..=1).all(|dy| {
                        tiles
                            .get(&(tx + dx, ty + dy))
                            .is_none_or(|(_, other)| other == offset)
                    })
                })
        })
        .map(|(_, (count, _))| count)
        .sum()
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
