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
        let node_at: Vec<(usize, VoxelKey)> = self
            .graph
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (i, self.graph.cells.coord(n.cell_id)))
            .filter(|(_, c)| warp.affects((c.0 as f32 + 0.5) * vs, (c.1 as f32 + 0.5) * vs))
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
        let mut queries: Vec<[f32; 3]> = Vec::with_capacity(voxels.len() + node_at.len());
        let mut times: Vec<f64> = Vec::with_capacity(queries.capacity());
        for k in &voxels {
            queries.push([
                (k.0 as f32 + 0.5) * vs,
                (k.1 as f32 + 0.5) * vs,
                (k.2 as f32 + 0.5) * vs,
            ]);
            times.push(time_of(k));
        }
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
        let warped = warp.apply(&queries, &times);
        report.warp_ms = ms_since(stage);

        let (moved_voxels, rest) = warped.split_at(voxels.len());
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

        let moved_count = voxels
            .par_iter()
            .zip(moved_voxels.par_iter())
            .filter(|(k, w)| voxelize((w[0], w[1], w[2]), vs) != **k)
            .count();
        if moved_count as f64 > warp.rebuild_fraction() * self.voxel_map.len() as f64 {
            let stage = Instant::now();
            self.rekey_voxels(&voxels, moved_voxels, vs);
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
