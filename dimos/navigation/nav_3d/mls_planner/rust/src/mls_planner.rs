// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Config and the owned-state Planner that builds and queries the MLS graph.

use ahash::AHashSet;
use rayon::prelude::*;
use serde::Deserialize;
use validator::Validate;

use crate::adjacency::{build_surface_cells, build_surface_lookup, rebuild_edges_around, CellId};
use crate::edges::{build_node_edges, build_node_edges_region, PlannerGraph};
use crate::nodes::{place_nodes, place_nodes_region};
use crate::planner;
use crate::surfaces::{
    add_to_by_col, extract_surfaces, extract_surfaces_region, remove_from_by_col, ColumnIz,
};
use crate::voxel::{voxelize, VoxelKey};

#[derive(Debug, Clone, Deserialize, Validate)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub world_frame: String,
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub robot_height: f32,
    #[validate(range(min = 0))]
    pub surface_dilation_passes: u32,
    #[validate(range(min = 0))]
    pub surface_erosion_passes: u32,
    #[validate(range(exclusive_min = 0.0))]
    pub node_spacing_m: f32,
    #[validate(range(min = 0.0))]
    pub node_wall_buffer_m: f32,
    #[validate(range(min = 0.0))]
    pub node_step_threshold_m: f32,
    /// Hard clearance floor: cells closer than this to a wall are impassable.
    #[serde(default = "default_robot_radius_m")]
    #[validate(range(min = 0.0))]
    pub robot_radius_m: f32,
    /// Strength of the soft wall penalty at the radius, decaying with distance.
    #[serde(default = "default_wall_penalty_weight")]
    #[validate(range(min = 0.0))]
    pub wall_penalty_weight: f32,
    /// Ground-plane distance from goal at which the planner stops replanning.
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f32,
    /// Rate cap for republishing the surface_map / nodes / node_edges viz
    /// artifacts. 0 disables them entirely. The path output is unthrottled.
    #[validate(range(min = 0.0))]
    pub viz_publish_hz: f32,
}

fn default_robot_radius_m() -> f32 {
    0.2
}

fn default_wall_penalty_weight() -> f32 {
    4.0
}

/// Cylindrical region the planner re-derives from a local map slice.
pub struct RegionBounds {
    pub origin_x: f32,
    pub origin_y: f32,
    pub radius: f32,
    pub z_min: f32,
    pub z_max: f32,
}

impl RegionBounds {
    fn contains_voxel(&self, (kx, ky, kz): VoxelKey, voxel_size: f32) -> bool {
        let half = voxel_size * 0.5;
        let z = kz as f32 * voxel_size + half;
        if z < self.z_min || z > self.z_max {
            return false;
        }
        let dx = kx as f32 * voxel_size + half - self.origin_x;
        let dy = ky as f32 * voxel_size + half - self.origin_y;
        dx * dx + dy * dy <= self.radius * self.radius
    }

    /// Inclusive voxel-column bounding box of the cylinder in the xy plane.
    fn column_bbox(&self, voxel_size: f32) -> (i32, i32, i32, i32) {
        let inv = 1.0 / voxel_size;
        let x0 = ((self.origin_x - self.radius) * inv).floor() as i32;
        let x1 = ((self.origin_x + self.radius) * inv).floor() as i32;
        let y0 = ((self.origin_y - self.radius) * inv).floor() as i32;
        let y1 = ((self.origin_y + self.radius) * inv).floor() as i32;
        (x0, x1, y0, y1)
    }
}

#[derive(Default)]
pub struct Planner {
    graph: PlannerGraph,
    voxel_map: AHashSet<VoxelKey>,
    by_col: ColumnIz,
}

impl Planner {
    pub fn update_global_map(&mut self, points: &[(f32, f32, f32)], config: &Config) {
        let voxel_size = config.voxel_size;
        let clearance = (config.robot_height / voxel_size).ceil() as i32;

        self.voxel_map.clear();
        for &p in points {
            self.voxel_map.insert(voxelize(p, voxel_size));
        }

        let mut surface: Vec<VoxelKey> = Vec::new();
        extract_surfaces(
            &self.voxel_map,
            clearance,
            config.surface_dilation_passes,
            config.surface_erosion_passes,
            &mut self.by_col,
            &mut surface,
        );
        build_surface_lookup(&surface, &mut self.graph.surface_lookup);

        self.rebuild_graph(config);
    }

    /// Update planner artifacts within a local region instead of recomputing
    /// the entire planner on the entire map.
    pub fn update_region(
        &mut self,
        local_points: &[(f32, f32, f32)],
        bounds: &RegionBounds,
        config: &Config,
    ) {
        let voxel_size = config.voxel_size;
        let clearance = (config.robot_height / voxel_size).ceil() as i32;
        let pad = (config.surface_dilation_passes + config.surface_erosion_passes) as i32;

        let changed = self.replace_region_voxels(local_points, bounds, voxel_size);

        // No voxel changed, so surfaces and the graph are untouched.
        let Some((bx0, bx1, by0, by1)) = changed else {
            return;
        };

        // A changed voxel column shifts surfaces only within pad of it, so the
        // write-back box is the changed-column bbox grown by pad.
        let write = (bx0 - pad, bx1 + pad, by0 - pad, by1 + pad);
        let new_cells = extract_surfaces_region(
            &self.by_col,
            clearance,
            config.surface_dilation_passes,
            config.surface_erosion_passes,
            write,
        );
        let (added, removed) = self.replace_surface_region(write, &new_cells);

        self.rebuild_region_graph(added, removed, config);
    }

    /// Patch cells for the changed surface, then re-place nodes and edges over
    /// the change window. A no-op when no surface cell changed.
    fn rebuild_region_graph(
        &mut self,
        added: Vec<VoxelKey>,
        removed: Vec<VoxelKey>,
        config: &Config,
    ) {
        let step = (config.node_step_threshold_m / config.voxel_size).floor() as i32;
        for &c in &removed {
            self.graph.cells.remove(c);
        }
        for &c in &added {
            self.graph.cells.insert(c);
        }
        let mut seeds = added;
        seeds.extend_from_slice(&removed);
        if seeds.is_empty() {
            return;
        }

        rebuild_edges_around(
            &mut self.graph.cells,
            &self.graph.surface_lookup,
            &seeds,
            config.voxel_size,
            step,
        );
        let window = self.node_window(&seeds, config);
        place_nodes_region(
            &mut self.graph.cells,
            &window,
            config.voxel_size,
            config.node_spacing_m,
            config.node_wall_buffer_m,
            config.robot_radius_m,
            config.wall_penalty_weight,
            &mut self.graph.wall_state,
            &mut self.graph.nodes,
        );
        build_node_edges_region(
            &self.graph.cells,
            &self.graph.nodes,
            &window,
            &mut self.graph.cell_state,
            &mut self.graph.node_edges,
            &mut self.graph.node_adj,
        );
    }

    /// Replace the cylinder's voxels with the local map points, ignoring
    /// points outside it. Returns the column bbox of changed voxels, or None
    /// if nothing changed.
    fn replace_region_voxels(
        &mut self,
        local_points: &[(f32, f32, f32)],
        bounds: &RegionBounds,
        voxel_size: f32,
    ) -> Option<(i32, i32, i32, i32)> {
        let new_set: AHashSet<VoxelKey> = local_points
            .iter()
            .map(|&p| voxelize(p, voxel_size))
            .collect();

        let (x0, x1, y0, y1) = bounds.column_bbox(voxel_size);
        let by_col = &self.by_col;
        let stale: Vec<VoxelKey> = (x0..(x1 + 1))
            .into_par_iter()
            .flat_map_iter(|ix| {
                let mut local: Vec<VoxelKey> = Vec::new();
                for iy in y0..=y1 {
                    let Some(zs) = by_col.get(&(ix, iy)) else {
                        continue;
                    };
                    for &iz in zs {
                        let k = (ix, iy, iz);
                        if bounds.contains_voxel(k, voxel_size) && !new_set.contains(&k) {
                            local.push(k);
                        }
                    }
                }
                local
            })
            .collect();

        let mut bb = ChangeBounds::new();
        for &k in &stale {
            bb.add(k.0, k.1);
            self.voxel_map.remove(&k);
            remove_from_by_col(&mut self.by_col, k);
        }
        for &k in &new_set {
            if !bounds.contains_voxel(k, voxel_size) {
                continue;
            }
            if self.voxel_map.insert(k) {
                bb.add(k.0, k.1);
                add_to_by_col(&mut self.by_col, k);
            }
        }
        bb.bounds()
    }

    /// Replace the surface_lookup entries for columns in the write box with
    /// the freshly extracted cells. Returns the added and removed cells so
    /// only the affected parts of the graph get patched.
    fn replace_surface_region(
        &mut self,
        write: (i32, i32, i32, i32),
        new_cells: &[VoxelKey],
    ) -> (Vec<VoxelKey>, Vec<VoxelKey>) {
        let (x0, x1, y0, y1) = write;
        let mut old: AHashSet<VoxelKey> = AHashSet::new();
        for ix in x0..=x1 {
            for iy in y0..=y1 {
                if let Some(zs) = self.graph.surface_lookup.remove(&(ix, iy)) {
                    for iz in zs {
                        old.insert((ix, iy, iz));
                    }
                }
            }
        }
        let new: AHashSet<VoxelKey> = new_cells.iter().copied().collect();

        let mut touched: AHashSet<(i32, i32)> = AHashSet::new();
        for &(ix, iy, iz) in new_cells {
            self.graph
                .surface_lookup
                .entry((ix, iy))
                .or_default()
                .push(iz);
            touched.insert((ix, iy));
        }
        for col in touched {
            if let Some(zs) = self.graph.surface_lookup.get_mut(&col) {
                zs.sort_unstable();
                zs.dedup();
            }
        }

        let added: Vec<VoxelKey> = new.iter().filter(|c| !old.contains(c)).copied().collect();
        let removed: Vec<VoxelKey> = old.iter().filter(|c| !new.contains(c)).copied().collect();
        (added, removed)
    }

    /// Rebuild all cells from surface_lookup, then nodes and edges.
    fn rebuild_graph(&mut self, config: &Config) {
        let voxel_size = config.voxel_size;
        let step = (config.node_step_threshold_m / voxel_size).floor() as i32;

        build_surface_cells(
            &mut self.graph.cells,
            &self.graph.surface_lookup,
            voxel_size,
            step,
        );
        self.rebuild_nodes(config);
    }

    /// Live cells within the changed-cell bbox grown by the node-graph margin,
    /// which covers the reach of any node, edge, or Voronoi change.
    fn node_window(&self, changed: &[VoxelKey], config: &Config) -> AHashSet<CellId> {
        // A few extra cells beyond the morphology, wall-buffer, and spacing reach.
        const SLACK_CELLS: i32 = 2;
        let voxel_size = config.voxel_size;
        let pad = (config.surface_dilation_passes + config.surface_erosion_passes) as i32;
        let buffer_cells = (config.node_wall_buffer_m / voxel_size).ceil() as i32;
        let spacing_cells = (config.node_spacing_m / voxel_size).ceil() as i32;
        let margin = pad + buffer_cells + spacing_cells + SLACK_CELLS;

        let mut bb = ChangeBounds::new();
        for &(ix, iy, _) in changed {
            bb.add(ix, iy);
        }
        let Some((min_x, max_x, min_y, max_y)) = bb.bounds() else {
            return AHashSet::new();
        };
        let (x0, x1, y0, y1) = (
            min_x - margin,
            max_x + margin,
            min_y - margin,
            max_y + margin,
        );

        let lookup = &self.graph.surface_lookup;
        let cells = &self.graph.cells;
        let ids: Vec<CellId> = (x0..(x1 + 1))
            .into_par_iter()
            .flat_map_iter(|ix| {
                let mut local: Vec<CellId> = Vec::new();
                for iy in y0..=y1 {
                    let Some(zs) = lookup.get(&(ix, iy)) else {
                        continue;
                    };
                    for &iz in zs {
                        if let Some(id) = cells.id((ix, iy, iz)) {
                            local.push(id);
                        }
                    }
                }
                local
            })
            .collect();
        ids.into_iter().collect()
    }

    /// Full rebuild of nodes and node edges from the current cells.
    fn rebuild_nodes(&mut self, config: &Config) {
        place_nodes(
            &mut self.graph.cells,
            config.voxel_size,
            config.node_spacing_m,
            config.node_wall_buffer_m,
            config.robot_radius_m,
            config.wall_penalty_weight,
            &mut self.graph.wall_state,
            &mut self.graph.nodes,
        );

        build_node_edges(
            &self.graph.cells,
            &self.graph.nodes,
            &mut self.graph.cell_state,
            &mut self.graph.node_edges,
            &mut self.graph.node_adj,
        );
    }

    pub fn plan(
        &self,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        config: &Config,
    ) -> Option<Vec<(f32, f32, f32)>> {
        if self.graph.nodes.is_empty() {
            return None;
        }
        planner::plan(&self.graph, start, goal, config)
    }

    pub fn graph(&self) -> &PlannerGraph {
        &self.graph
    }

    pub fn surface(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.graph
            .surface_lookup
            .iter()
            .flat_map(|(&(ix, iy), zs)| zs.iter().map(move |&iz| (ix, iy, iz)))
    }

    /// Surface cells paired with their wall clearance, the distance to the
    /// nearest untraversable edge. Unreached cells report +inf.
    pub fn surface_clearance(&self) -> Vec<(VoxelKey, f32)> {
        let dist = &self.graph.wall_state.dist;
        self.graph
            .cells
            .ids()
            .map(|id| {
                let d = dist.get(id as usize).copied().unwrap_or(f32::INFINITY);
                (self.graph.cells.coord(id), d)
            })
            .collect()
    }

    pub fn voxel_count(&self) -> usize {
        self.voxel_map.len()
    }

    pub fn voxel_keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.voxel_map.iter().copied()
    }
}

/// Running inclusive xy bounding box of changed columns.
struct ChangeBounds {
    min_x: i32,
    max_x: i32,
    min_y: i32,
    max_y: i32,
    any: bool,
}

impl ChangeBounds {
    fn new() -> Self {
        Self {
            min_x: i32::MAX,
            max_x: i32::MIN,
            min_y: i32::MAX,
            max_y: i32::MIN,
            any: false,
        }
    }

    fn add(&mut self, ix: i32, iy: i32) {
        self.any = true;
        self.min_x = self.min_x.min(ix);
        self.max_x = self.max_x.max(ix);
        self.min_y = self.min_y.min(iy);
        self.max_y = self.max_y.max(iy);
    }

    fn bounds(&self) -> Option<(i32, i32, i32, i32)> {
        self.any
            .then_some((self.min_x, self.max_x, self.min_y, self.max_y))
    }
}

#[cfg(test)]
mod region_tests {
    use super::*;
    use std::collections::{BTreeMap, BTreeSet};

    fn test_config() -> Config {
        Config {
            world_frame: String::new(),
            voxel_size: 0.1,
            robot_height: 0.5,
            surface_dilation_passes: 3,
            surface_erosion_passes: 3,
            node_spacing_m: 1.0,
            node_wall_buffer_m: 0.3,
            node_step_threshold_m: 0.25,
            robot_radius_m: 0.0,
            wall_penalty_weight: 1.0,
            goal_tolerance: 0.3,
            viz_publish_hz: 2.0,
        }
    }

    /// Floor slab with a wall down the middle, as world-frame point centers.
    fn world_points() -> Vec<(f32, f32, f32)> {
        let vs = 0.1_f32;
        let half = vs * 0.5;
        let mut pts = Vec::new();
        for ix in 0..40 {
            for iy in 0..40 {
                pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
            }
        }
        // a wall column from z=0 up, to create wall-adjacency for nodes
        for iy in 0..40 {
            for iz in 0..15 {
                pts.push((
                    20.0 * vs + half,
                    iy as f32 * vs + half,
                    iz as f32 * vs + half,
                ));
            }
        }
        pts
    }

    fn surface_set(p: &Planner) -> BTreeSet<VoxelKey> {
        p.surface().collect()
    }

    fn voxel_set(p: &Planner) -> BTreeSet<VoxelKey> {
        p.voxel_map.iter().copied().collect()
    }

    /// Cell adjacency keyed by coordinate, independent of CellId.
    fn cell_edges(p: &Planner) -> BTreeMap<VoxelKey, BTreeSet<(VoxelKey, u32)>> {
        let cells = &p.graph.cells;
        let mut out: BTreeMap<VoxelKey, BTreeSet<(VoxelKey, u32)>> = BTreeMap::new();
        for (id, edges) in cells.iter() {
            let src = cells.coord(id);
            let set = out.entry(src).or_default();
            for e in edges {
                set.insert((cells.coord(e.dest), e.cost.to_bits()));
            }
        }
        out
    }

    fn node_coords(p: &Planner) -> BTreeSet<VoxelKey> {
        p.graph
            .nodes
            .iter()
            .map(|n| p.graph.cells.coord(n.cell_id))
            .collect()
    }

    fn node_edge_pairs(p: &Planner) -> BTreeSet<(VoxelKey, VoxelKey, u32)> {
        let cells = &p.graph.cells;
        p.graph
            .node_edges
            .iter()
            .map(|e| {
                let a = cells.coord(e.a);
                let b = cells.coord(e.b);
                let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                (lo, hi, e.cost.to_bits())
            })
            .collect()
    }

    #[test]
    fn region_update_removes_stale_voxels() {
        let cfg = test_config();
        let bounds = RegionBounds {
            origin_x: 2.0,
            origin_y: 2.0,
            radius: 1.0,
            z_min: -1.0,
            z_max: 2.0,
        };
        let all = world_points();

        let mut full = Planner::default();
        full.update_global_map(&all, &cfg);

        let inside: Vec<_> = all
            .iter()
            .copied()
            .filter(|&p| bounds.contains_voxel(voxelize(p, cfg.voxel_size), cfg.voxel_size))
            .collect();
        let outside: Vec<_> = all
            .iter()
            .copied()
            .filter(|&p| !bounds.contains_voxel(voxelize(p, cfg.voxel_size), cfg.voxel_size))
            .collect();

        // Seed the cylinder with a stack of junk voxels not present in the
        // world, so update_region must clear them and the surface they induce.
        let mut seeded = outside.clone();
        for iz in 3..8 {
            seeded.push((2.05, 2.05, iz as f32 * cfg.voxel_size + 0.05));
        }
        let mut region = Planner::default();
        region.update_global_map(&seeded, &cfg);
        region.update_region(&inside, &bounds, &cfg);

        assert_eq!(voxel_set(&region), voxel_set(&full), "voxel mismatch");
        assert_eq!(surface_set(&region), surface_set(&full), "surface mismatch");
        assert_eq!(
            cell_edges(&region),
            cell_edges(&full),
            "cell edges mismatch"
        );
        assert_eq!(node_coords(&region), node_coords(&full), "node mismatch");
        assert_eq!(
            node_edge_pairs(&region),
            node_edge_pairs(&full),
            "node edge mismatch"
        );
    }

    /// A point outside the region bounds must not enter the planner's voxel
    /// map, where it could never be cleared and would inflate the rebuild box.
    #[test]
    fn region_update_ignores_points_outside_bounds() {
        let cfg = test_config();
        let bounds = RegionBounds {
            origin_x: 2.0,
            origin_y: 2.0,
            radius: 1.0,
            z_min: -1.0,
            z_max: 2.0,
        };
        let inside = (2.05, 2.05, 0.05);
        let outside = (10.05, 10.05, 0.05);

        let mut p = Planner::default();
        p.update_region(&[inside, outside], &bounds, &cfg);

        assert!(p.voxel_map.contains(&voxelize(inside, cfg.voxel_size)));
        assert!(!p.voxel_map.contains(&voxelize(outside, cfg.voxel_size)));
    }

    /// Floor 8m x 8m with a wall at x=4m that only a gap at y in [3.5, 4.5]
    /// passes through, so crossing the wall is a non-trivial route.
    fn big_world() -> Vec<(f32, f32, f32)> {
        let vs = 0.1_f32;
        let half = vs * 0.5;
        let mut pts = Vec::new();
        for ix in 0..80 {
            for iy in 0..80 {
                pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
            }
        }
        for iy in 0..80 {
            if (35..45).contains(&iy) {
                continue;
            }
            for iz in 0..15 {
                pts.push((
                    40.0 * vs + half,
                    iy as f32 * vs + half,
                    iz as f32 * vs + half,
                ));
            }
        }
        pts
    }

    fn slice(all: &[(f32, f32, f32)], b: &RegionBounds, vs: f32) -> Vec<(f32, f32, f32)> {
        all.iter()
            .copied()
            .filter(|&p| b.contains_voxel(voxelize(p, vs), vs))
            .collect()
    }

    fn path_len(w: &[(f32, f32, f32)]) -> f32 {
        w.windows(2)
            .map(|p| {
                let dx = p[1].0 - p[0].0;
                let dy = p[1].1 - p[0].1;
                let dz = p[1].2 - p[0].2;
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .sum()
    }

    type Pose = (f32, f32, f32);
    const PLAN_PAIRS: [(Pose, Pose); 4] = [
        ((0.5, 0.5, 0.05), (7.5, 7.5, 0.05)),
        ((0.5, 7.5, 0.05), (7.5, 0.5, 0.05)),
        ((0.5, 0.5, 0.05), (0.5, 7.5, 0.05)),
        ((7.5, 0.5, 0.05), (7.5, 7.5, 0.05)),
    ];

    fn assert_plans_equivalent(full: &Planner, region: &Planner, cfg: &Config) {
        for (s, g) in PLAN_PAIRS {
            let pf = full.plan(s, g, cfg);
            let pr = region.plan(s, g, cfg);
            assert_eq!(
                pf.is_some(),
                pr.is_some(),
                "path existence differs for {s:?} -> {g:?}"
            );
            if let (Some(pf), Some(pr)) = (pf, pr) {
                let (lf, lr) = (path_len(&pf), path_len(&pr));
                assert!(lr <= lf * 1.6 + 0.5, "region path too long: {lr} vs {lf}");
                assert!(lf <= lr * 1.6 + 0.5, "full path too long: {lf} vs {lr}");
            }
        }
    }

    /// Re-observing the same geometry must change nothing: no voxel, surface,
    /// cell, node, or edge moves. This is the anti-jitter guarantee, far nodes
    /// stay put when their region is re-seen, matching a full rebuild.
    #[test]
    fn region_reobserve_leaves_graph_bit_identical() {
        let cfg = test_config();
        let all = big_world();
        let vs = cfg.voxel_size;

        let mut p = Planner::default();
        p.update_global_map(&all, &cfg);
        let before_cells = cell_edges(&p);
        let before_nodes = node_coords(&p);
        let before_edges = node_edge_pairs(&p);

        for &(cx, cy) in &[(2.0, 2.0), (4.0, 4.0), (6.0, 3.0), (1.5, 7.0), (7.0, 7.0)] {
            let b = RegionBounds {
                origin_x: cx,
                origin_y: cy,
                radius: 1.2,
                z_min: -1.0,
                z_max: 2.0,
            };
            p.update_region(&slice(&all, &b, vs), &b, &cfg);
        }

        assert_eq!(
            cell_edges(&p),
            before_cells,
            "cells changed on re-observation"
        );
        assert_eq!(
            node_coords(&p),
            before_nodes,
            "nodes moved on re-observation"
        );
        assert_eq!(
            node_edge_pairs(&p),
            before_edges,
            "edges changed on re-observation"
        );
    }

    /// Build the planner purely from streamed local cylinders, as the live
    /// pipeline does, and require equivalent planning to a one-shot full build.
    #[test]
    fn region_stream_only_plans_like_full() {
        let cfg = test_config();
        let all = big_world();
        let vs = cfg.voxel_size;

        let mut full = Planner::default();
        full.update_global_map(&all, &cfg);

        let mut region = Planner::default();
        let mut cx = 0.5;
        while cx <= 7.5 {
            let mut cy = 0.5;
            while cy <= 7.5 {
                let b = RegionBounds {
                    origin_x: cx,
                    origin_y: cy,
                    radius: 1.5,
                    z_min: -1.0,
                    z_max: 2.0,
                };
                let s = slice(&all, &b, vs);
                if !s.is_empty() {
                    region.update_region(&s, &b, &cfg);
                }
                cy += 1.0;
            }
            cx += 1.0;
        }

        assert_eq!(
            voxel_set(&region),
            voxel_set(&full),
            "stream did not reconstruct the map"
        );
        assert_plans_equivalent(&full, &region, &cfg);
    }

    /// Floor split by a wall with a narrow 1-cell gap near x=1.0 and a wide gap
    /// near x=4.5. Start and goal straddle the narrow gap.
    fn two_gap_world() -> Vec<(f32, f32, f32)> {
        let vs = 0.1_f32;
        let half = vs * 0.5;
        let mut pts = Vec::new();
        for ix in 0..60 {
            for iy in 0..40 {
                pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
            }
        }
        for ix in 0..60 {
            if ix == 10 || (40..50).contains(&ix) {
                continue;
            }
            for iz in 0..7 {
                pts.push((
                    ix as f32 * vs + half,
                    20.0 * vs + half,
                    iz as f32 * vs + half,
                ));
            }
        }
        pts
    }

    /// The hard clearance floor must make the narrow gap impassable, forcing
    /// the longer detour through the wide gap.
    #[test]
    fn hard_clearance_floor_avoids_narrow_gap() {
        let mut cfg = test_config();
        cfg.node_spacing_m = 0.8;
        let pts = two_gap_world();
        let start = (1.0, 1.0, 0.05);
        let goal = (1.0, 3.5, 0.05);
        let max_x = |w: &[(f32, f32, f32)]| w.iter().map(|p| p.0).fold(f32::MIN, f32::max);

        // No floor: the shortest route slips straight through the narrow gap.
        cfg.robot_radius_m = 0.0;
        let mut open = Planner::default();
        open.update_global_map(&pts, &cfg);
        let wp_open = open.plan(start, goal, &cfg).expect("open plan exists");

        // Floor wider than the narrow gap: it is impassable, so detour wide.
        cfg.robot_radius_m = 0.2;
        let mut safe = Planner::default();
        safe.update_global_map(&pts, &cfg);
        let wp_safe = safe.plan(start, goal, &cfg).expect("safe plan exists");

        assert!(max_x(&wp_open) < 2.0, "open path should use the near gap");
        assert!(
            max_x(&wp_safe) > 3.5,
            "safe path should detour to the wide gap: max_x={}",
            max_x(&wp_safe)
        );
        assert!(
            path_len(&wp_safe) > path_len(&wp_open) * 1.5,
            "safe route should be substantially longer: {} vs {}",
            path_len(&wp_safe),
            path_len(&wp_open)
        );
    }
}
