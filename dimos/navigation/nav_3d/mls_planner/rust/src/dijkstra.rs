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

//! Multi-source Dijkstra over the CellId-indexed surface graph. State and
//! the heap live in a reusable struct so the inner loop never allocates.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use ahash::AHashSet;

use crate::adjacency::{CellId, SurfaceCells, NO_CELL};
use crate::voxel::VoxelKey;

#[derive(Default)]
pub struct DijkstraState {
    pub dist: Vec<f32>,
    pub pred: Vec<CellId>,
    pub source: Vec<u32>,
    // Window membership for the regional search, left all-false between calls.
    in_window: Vec<bool>,
    heap: BinaryHeap<Scored>,
}

impl DijkstraState {
    /// Reset all vecs to n slots.
    pub fn reset(&mut self, n: usize) {
        self.dist.clear();
        self.dist.resize(n, f32::INFINITY);
        self.pred.clear();
        self.pred.resize(n, NO_CELL);
        self.source.clear();
        self.source.resize(n, 0);
        self.in_window.clear();
        self.in_window.resize(n, false);
        self.heap.clear();
    }

    /// Grow the vecs to n slots without disturbing existing labels. New slots
    /// default to unreached.
    fn ensure_capacity(&mut self, n: usize) {
        if self.dist.len() < n {
            self.dist.resize(n, f32::INFINITY);
            self.pred.resize(n, NO_CELL);
            self.source.resize(n, 0);
        }
        if self.in_window.len() < n {
            self.in_window.resize(n, false);
        }
    }
}

/// Which edge weight a search uses.
#[derive(Clone, Copy)]
pub enum Weight {
    /// Geometric distance, for the wall-distance field.
    Base,
    /// Wall-safe penalized cost, for the node Voronoi.
    Penalized,
}

impl Weight {
    #[inline]
    fn of(self, edge: &crate::adjacency::Edge) -> f32 {
        match self {
            Weight::Base => edge.base_cost,
            Weight::Penalized => edge.cost,
        }
    }
}

/// Multi-source Dijkstra labeling each cell with its nearest source and path.
pub fn dijkstra(
    cells: &SurfaceCells,
    sources: &[CellId],
    state: &mut DijkstraState,
    weight: Weight,
) {
    state.reset(cells.slot_capacity());

    for &s in sources {
        if !cells.is_live(s) {
            continue;
        }
        state.dist[s as usize] = 0.0;
        state.source[s as usize] = s;
        state.heap.push(Scored(0.0, cells.coord(s), s));
    }

    while let Some(Scored(d, _, u)) = state.heap.pop() {
        let cur = state.dist[u as usize];
        if d > cur {
            continue;
        }
        let su = state.source[u as usize];
        for edge in cells.neighbors(u) {
            let nd = d + weight.of(edge);
            let v = edge.dest as usize;
            if nd < state.dist[v] {
                state.dist[v] = nd;
                state.pred[v] = u;
                state.source[v] = su;
                state
                    .heap
                    .push(Scored(nd, cells.coord(edge.dest), edge.dest));
            }
        }
    }
}

/// Multi-source Dijkstra that re-labels only cells in the window, seeded from
/// in-window sources and the cached frontier just outside it. Correct while the
/// window margin exceeds the reach of the change.
pub fn dijkstra_region(
    cells: &SurfaceCells,
    sources: &[CellId],
    window: &AHashSet<CellId>,
    state: &mut DijkstraState,
    weight: Weight,
) {
    let n_slots = cells.slot_capacity();
    state.ensure_capacity(n_slots);
    state.heap.clear();

    // Dense membership mask over the window cells.
    for &w in window {
        let i = w as usize;
        state.in_window[i] = true;
        state.dist[i] = f32::INFINITY;
        state.pred[i] = NO_CELL;
        state.source[i] = 0;
    }

    for &s in sources {
        if !cells.is_live(s) || !state.in_window[s as usize] {
            continue;
        }
        state.dist[s as usize] = 0.0;
        state.source[s as usize] = s;
        state.heap.push(Scored(0.0, cells.coord(s), s));
    }

    let mut frontier: AHashSet<CellId> = AHashSet::new();
    for &w in window {
        for edge in cells.neighbors(w) {
            let n = edge.dest;
            if !state.in_window[n as usize] && state.dist[n as usize].is_finite() {
                frontier.insert(n);
            }
        }
    }
    for &n in &frontier {
        state
            .heap
            .push(Scored(state.dist[n as usize], cells.coord(n), n));
    }

    while let Some(Scored(d, _, u)) = state.heap.pop() {
        if d > state.dist[u as usize] {
            continue;
        }
        let su = state.source[u as usize];
        for edge in cells.neighbors(u) {
            let v = edge.dest;
            if !state.in_window[v as usize] {
                continue;
            }
            let nd = d + weight.of(edge);
            if nd < state.dist[v as usize] {
                state.dist[v as usize] = nd;
                state.pred[v as usize] = u;
                state.source[v as usize] = su;
                state.heap.push(Scored(nd, cells.coord(v), v));
            }
        }
    }

    for &w in window {
        state.in_window[w as usize] = false;
    }
}

/// Reconstruct the path back to the nearest source.
///
/// Returns the start if the cell has not been reached by any dijkstra calls.
pub fn walk_preds(state: &DijkstraState, start: CellId) -> Vec<CellId> {
    let mut cells = vec![start];
    let mut cur = start;
    let mut seen: AHashSet<CellId> = AHashSet::new();
    seen.insert(start);
    loop {
        let p = state.pred[cur as usize];
        if p == NO_CELL || !seen.insert(p) {
            break;
        }
        cur = p;
        cells.push(cur);
    }
    cells
}

struct Scored(f32, VoxelKey, CellId);

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
        // Tie-break on cell id for repeatable ordering.
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{
        build_surface_cells, build_surface_lookup, SurfaceCells, SurfaceLookup,
    };
    use crate::voxel::VoxelKey;

    fn grid(n: i32) -> SurfaceCells {
        let cells: Vec<VoxelKey> = (0..n)
            .flat_map(|x| (0..n).map(move |y| (x, y, 0)))
            .collect();
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&cells, &mut lookup);
        let mut sc = SurfaceCells::default();
        build_surface_cells(&mut sc, &lookup, 0.1, 2);
        sc
    }

    fn root_of(state: &DijkstraState, start: CellId) -> CellId {
        let mut cur = start;
        while state.pred[cur as usize] != NO_CELL {
            cur = state.pred[cur as usize];
        }
        cur
    }

    #[test]
    fn walk_preds_breaks_on_pred_cycle() {
        let mut state = DijkstraState::default();
        state.reset(2);
        state.pred[0] = 1;
        state.pred[1] = 0;
        let path = walk_preds(&state, 0);
        assert_eq!(path, vec![0, 1]);
    }

    #[test]
    fn region_window_all_equals_full() {
        let sc = grid(10);
        let sources = [sc.id((0, 0, 0)).unwrap(), sc.id((9, 9, 0)).unwrap()];

        let mut full = DijkstraState::default();
        dijkstra(&sc, &sources, &mut full, Weight::Penalized);

        let window: AHashSet<CellId> = sc.ids().collect();
        let mut region = DijkstraState::default();
        dijkstra_region(&sc, &sources, &window, &mut region, Weight::Penalized);

        for id in sc.ids() {
            assert_eq!(
                region.dist[id as usize],
                full.dist[id as usize],
                "dist mismatch at {:?}",
                sc.coord(id)
            );
        }
    }

    #[test]
    fn region_partial_window_reproduces_cached_distances() {
        let sc = grid(12);
        let sources = [sc.id((0, 0, 0)).unwrap(), sc.id((11, 11, 0)).unwrap()];

        let mut full = DijkstraState::default();
        dijkstra(&sc, &sources, &mut full, Weight::Penalized);

        // Seed the regional state with the full result as the cache, then
        // recompute an interior block. Nothing changed, so the block must come
        // back identical and every cell must still trace to a real source.
        let mut region = DijkstraState {
            dist: full.dist.clone(),
            pred: full.pred.clone(),
            source: full.source.clone(),
            ..Default::default()
        };

        let window: AHashSet<CellId> = sc
            .ids()
            .filter(|&id| {
                let (x, y, _) = sc.coord(id);
                (3..=8).contains(&x) && (3..=8).contains(&y)
            })
            .collect();
        dijkstra_region(&sc, &sources, &window, &mut region, Weight::Penalized);

        for &id in &window {
            assert_eq!(
                region.dist[id as usize],
                full.dist[id as usize],
                "dist mismatch at {:?}",
                sc.coord(id)
            );
            let root = root_of(&region, id);
            assert!(
                sources.contains(&root),
                "cell {:?} traces to non-source {:?}",
                sc.coord(id),
                sc.coord(root)
            );
        }
    }

    fn chain(n: i32) -> (SurfaceCells, Vec<CellId>) {
        let mut sc = SurfaceCells::default();
        let ids: Vec<CellId> = (0..n).map(|i| sc.insert((i, 0, 0))).collect();
        for i in 0..n - 1 {
            sc.add_edge(ids[i as usize], ids[(i + 1) as usize], 1.0);
            sc.add_edge(ids[(i + 1) as usize], ids[i as usize], 1.0);
        }
        (sc, ids)
    }

    #[test]
    fn single_source_dist_and_pred() {
        let (sc, ids) = chain(5);
        let mut st = DijkstraState::default();
        dijkstra(&sc, &[ids[0]], &mut st, Weight::Penalized);
        for (i, &id) in ids.iter().enumerate().take(5) {
            assert_eq!(st.dist[id as usize], i as f32);
            assert_eq!(st.source[id as usize], 0);
        }
        assert_eq!(st.pred[ids[0] as usize], NO_CELL);
        let mut cur = ids[4];
        let mut hops = 0;
        while st.pred[cur as usize] != NO_CELL {
            cur = st.pred[cur as usize];
            hops += 1;
        }
        assert_eq!(cur, ids[0]);
        assert_eq!(hops, 4);
    }

    #[test]
    fn multi_source_labels_by_nearest() {
        let (sc, ids) = chain(5);
        let mut st = DijkstraState::default();
        dijkstra(&sc, &[ids[0], ids[4]], &mut st, Weight::Penalized);
        assert_eq!(st.source[ids[0] as usize], ids[0]);
        assert_eq!(st.source[ids[1] as usize], ids[0]);
        assert_eq!(st.source[ids[3] as usize], ids[4]);
        assert_eq!(st.source[ids[4] as usize], ids[4]);
        let s2 = st.source[ids[2] as usize];
        assert!(s2 == ids[0] || s2 == ids[4]);
        assert_eq!(st.dist[ids[0] as usize], 0.0);
        assert_eq!(st.dist[ids[1] as usize], 1.0);
        assert_eq!(st.dist[ids[2] as usize], 2.0);
        assert_eq!(st.dist[ids[3] as usize], 1.0);
        assert_eq!(st.dist[ids[4] as usize], 0.0);
    }

    #[test]
    fn disconnected_cells_stay_unreachable() {
        let mut sc = SurfaceCells::default();
        let a = sc.insert((0, 0, 0));
        let b = sc.insert((1, 0, 0));
        let c = sc.insert((2, 0, 0));
        let d = sc.insert((3, 0, 0));
        sc.add_edge(a, b, 1.0);
        sc.add_edge(b, a, 1.0);
        sc.add_edge(c, d, 1.0);
        sc.add_edge(d, c, 1.0);
        let mut st = DijkstraState::default();
        dijkstra(&sc, &[a], &mut st, Weight::Penalized);
        assert_eq!(st.dist[a as usize], 0.0);
        assert_eq!(st.dist[b as usize], 1.0);
        assert!(!st.dist[c as usize].is_finite());
        assert!(!st.dist[d as usize].is_finite());
    }

    #[test]
    fn shorter_path_overrides_longer() {
        let mut sc = SurfaceCells::default();
        let a = sc.insert((0, 0, 0));
        let b = sc.insert((1, 0, 0));
        let c = sc.insert((2, 0, 0));
        sc.add_edge(a, b, 10.0);
        sc.add_edge(b, a, 10.0);
        sc.add_edge(a, c, 1.0);
        sc.add_edge(c, a, 1.0);
        sc.add_edge(c, b, 1.0);
        sc.add_edge(b, c, 1.0);
        let mut st = DijkstraState::default();
        dijkstra(&sc, &[a], &mut st, Weight::Penalized);
        assert_eq!(st.dist[b as usize], 2.0);
        assert_eq!(st.pred[b as usize], c);
    }

    #[test]
    fn buffer_reuse_does_not_leak_prior_state() {
        let (sc1, ids1) = chain(5);
        let mut st = DijkstraState::default();
        dijkstra(&sc1, &[ids1[0]], &mut st, Weight::Penalized);
        let (sc2, ids2) = chain(3);
        dijkstra(&sc2, &[ids2[0]], &mut st, Weight::Penalized);
        for (i, &id) in ids2.iter().enumerate().take(3) {
            assert_eq!(st.dist[id as usize], i as f32);
        }
    }
}
