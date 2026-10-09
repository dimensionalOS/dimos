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
use rayon::prelude::*;

use crate::adjacency::{CellId, SurfaceCells, NO_CELL};
use crate::voxel::VoxelKey;

#[derive(Default)]
pub struct DijkstraState {
    pub dist: Vec<f32>,
    pub pred: Vec<CellId>,
    pub source: Vec<u32>,
    heap: BinaryHeap<Scored<u128>>,
}

/// Heap tie-breaker for the ordering. This is totally swappable, this is
/// just very fast.
#[inline]
fn heap_key(c: VoxelKey, id: CellId) -> u128 {
    let f = |v: i32| (v as u32 ^ 0x8000_0000) as u128;
    f(c.0) << 96 | f(c.1) << 64 | f(c.2) << 32 | id as u128
}

#[inline]
fn heap_id(key: u128) -> CellId {
    key as u32
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
        state.heap.push(Scored(0.0, heap_key(cells.coord(s), s)));
    }

    while let Some(Scored(d, key)) = state.heap.pop() {
        let u = heap_id(key);
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
                    .push(Scored(nd, heap_key(cells.coord(edge.dest), edge.dest)));
            }
        }
    }
}

/// Multi-source Dijkstra that re-labels only cells in the window, seeded from
/// in-window sources and the cached frontier just outside it. The reference
/// dijkstra_clusters must match.
#[cfg(test)]
pub fn dijkstra_region(
    cells: &SurfaceCells,
    sources: &[CellId],
    window: &[CellId],
    state: &mut DijkstraState,
    weight: Weight,
) {
    let n_slots = cells.slot_capacity();
    state.ensure_capacity(n_slots);
    state.heap.clear();
    let mut in_window = vec![false; n_slots];
    let mut in_frontier = vec![false; n_slots];
    let mut frontier: Vec<CellId> = Vec::new();

    // Dense membership mask over the window cells.
    for &w in window {
        let i = w as usize;
        in_window[i] = true;
        state.dist[i] = f32::INFINITY;
        state.pred[i] = NO_CELL;
        state.source[i] = 0;
    }

    for &s in sources {
        if !cells.is_live(s) || !in_window[s as usize] {
            continue;
        }
        state.dist[s as usize] = 0.0;
        state.source[s as usize] = s;
        state.heap.push(Scored(0.0, heap_key(cells.coord(s), s)));
    }

    for &w in window {
        for edge in cells.neighbors(w) {
            let n = edge.dest;
            if !in_window[n as usize]
                && !in_frontier[n as usize]
                && state.dist[n as usize].is_finite()
            {
                in_frontier[n as usize] = true;
                frontier.push(n);
            }
        }
    }
    for &n in &frontier {
        in_frontier[n as usize] = false;
        state
            .heap
            .push(Scored(state.dist[n as usize], heap_key(cells.coord(n), n)));
    }

    while let Some(Scored(d, key)) = state.heap.pop() {
        let u = heap_id(key);
        if d > state.dist[u as usize] {
            continue;
        }
        let su = state.source[u as usize];
        for edge in cells.neighbors(u) {
            let v = edge.dest;
            if !in_window[v as usize] {
                continue;
            }
            let nd = d + weight.of(edge);
            if nd < state.dist[v as usize] {
                state.dist[v as usize] = nd;
                state.pred[v as usize] = u;
                state.source[v as usize] = su;
                state.heap.push(Scored(nd, heap_key(cells.coord(v), v)));
            }
        }
    }

    for &w in window {
        in_window[w as usize] = false;
    }
}

/// The window split into its connected pieces over cell adjacency. Pieces
/// never touch, so each one repairs on its own.
pub fn window_clusters(
    cells: &SurfaceCells,
    window: &[CellId],
    seen: &mut Vec<bool>,
) -> Vec<Vec<CellId>> {
    if seen.len() < cells.slot_capacity() {
        seen.resize(cells.slot_capacity(), false);
    }
    for &w in window {
        seen[w as usize] = true;
    }
    let mut clusters: Vec<Vec<CellId>> = Vec::new();
    let mut stack: Vec<CellId> = Vec::new();
    for &w in window {
        if !seen[w as usize] {
            continue;
        }
        seen[w as usize] = false;
        stack.push(w);
        let mut cluster: Vec<CellId> = Vec::new();
        while let Some(u) = stack.pop() {
            cluster.push(u);
            for e in cells.neighbors(u) {
                let v = e.dest as usize;
                if seen[v] {
                    seen[v] = false;
                    stack.push(e.dest);
                }
            }
        }
        clusters.push(cluster);
    }
    clusters
}

pub const NO_CLUSTER: u32 = u32::MAX;

/// Which cluster each window cell belongs to and its position in it, dense
/// per slot. Cells outside the window read NO_CLUSTER.
#[derive(Default)]
pub struct ClusterIndex {
    cluster_of: Vec<u32>,
    local: Vec<u32>,
}

impl ClusterIndex {
    pub fn assign(&mut self, n_slots: usize, clusters: &[Vec<CellId>]) {
        if self.cluster_of.len() < n_slots {
            self.cluster_of.resize(n_slots, NO_CLUSTER);
            self.local.resize(n_slots, 0);
        }
        for (c, cluster) in clusters.iter().enumerate() {
            for (i, &id) in cluster.iter().enumerate() {
                self.cluster_of[id as usize] = c as u32;
                self.local[id as usize] = i as u32;
            }
        }
    }

    /// Back to all-outside for the next window.
    pub fn clear(&mut self, clusters: &[Vec<CellId>]) {
        for cluster in clusters {
            for &id in cluster {
                self.cluster_of[id as usize] = NO_CLUSTER;
            }
        }
    }

    /// Whether a cell is in the window the clusters were assigned from.
    #[inline]
    pub fn in_window(&self, id: CellId) -> bool {
        self.cluster(id) != NO_CLUSTER
    }

    #[inline]
    fn cluster(&self, id: CellId) -> u32 {
        self.cluster_of
            .get(id as usize)
            .copied()
            .unwrap_or(NO_CLUSTER)
    }

    #[inline]
    fn local(&self, id: CellId) -> usize {
        self.local[id as usize] as usize
    }
}

/// One cluster's labels, indexed like the cluster's cell list.
struct ClusterLabels {
    dist: Vec<f32>,
    pred: Vec<CellId>,
    source: Vec<u32>,
}

/// Which cached labels just outside the window may seed a search.
#[derive(Clone, Copy)]
pub enum Frontier<'a> {
    /// Every labeled neighbor. For fields where only the value matters.
    Any,
    /// Only a neighbor whose cached chain reaches a live source without
    /// entering the window, labeled with that source. A chain through the
    /// window was built on the labels this search replaces. The closure says
    /// whether a cell is a live source.
    Chained(&'a (dyn Fn(CellId) -> bool + Sync)),
}

/// Longest cached chain followed before it counts as broken.
const MAX_CHAIN: usize = 4096;

/// The live source a cell's cached chain reaches without entering the window,
/// or None when the chain enters the window, hits a dead cell or ends
/// elsewhere. Chains are kept valid by every repair, so hops are not checked
/// for adjacency here.
pub fn chain_source(
    cells: &SurfaceCells,
    state: &DijkstraState,
    index: &ClusterIndex,
    from: CellId,
    is_source: &dyn Fn(CellId) -> bool,
) -> Option<CellId> {
    let mut cur = from;
    for _ in 0..MAX_CHAIN {
        let i = cur as usize;
        if index.in_window(cur)
            || !cells.is_live(cur)
            || !state.dist.get(i).is_some_and(|d| d.is_finite())
        {
            return None;
        }
        let pred = state.pred[i];
        if pred == NO_CELL {
            return is_source(cur).then_some(cur);
        }
        cur = pred;
    }
    None
}

/// dijkstra_region run cluster by cluster in parallel. A cluster's search
/// relaxes only its own cells and reads the cached labels just outside the
/// window, so the clusters never interact. Returns the window cells that lost
/// their source or changed it.
pub fn dijkstra_clusters(
    cells: &SurfaceCells,
    sources: &[CellId],
    clusters: &[Vec<CellId>],
    index: &ClusterIndex,
    state: &mut DijkstraState,
    weight: Weight,
    frontier: Frontier,
) -> Vec<CellId> {
    state.ensure_capacity(cells.slot_capacity());
    let mut by_cluster: Vec<Vec<CellId>> = vec![Vec::new(); clusters.len()];
    for &s in sources {
        if !cells.is_live(s) {
            continue;
        }
        let c = index.cluster(s);
        if c != NO_CLUSTER {
            by_cluster[c as usize].push(s);
        }
    }
    let labels: Vec<ClusterLabels> = clusters
        .par_iter()
        .zip(by_cluster.par_iter())
        .map(|(cluster, srcs)| {
            dijkstra_cluster(cells, srcs, cluster, index, state, weight, frontier)
        })
        .collect();
    let mut changed: Vec<CellId> = Vec::new();
    for (cluster, l) in clusters.iter().zip(labels) {
        for (i, &id) in cluster.iter().enumerate() {
            let slot = id as usize;
            if state.dist[slot].is_finite()
                && (!l.dist[i].is_finite() || l.source[i] != state.source[slot])
            {
                changed.push(id);
            }
            state.dist[slot] = l.dist[i];
            state.pred[slot] = l.pred[i];
            state.source[slot] = l.source[i];
        }
    }
    changed
}

fn dijkstra_cluster(
    cells: &SurfaceCells,
    sources: &[CellId],
    cluster: &[CellId],
    index: &ClusterIndex,
    state: &DijkstraState,
    weight: Weight,
    frontier: Frontier,
) -> ClusterLabels {
    let me = index.cluster(cluster[0]);
    let k = cluster.len();
    let mut dist = vec![f32::INFINITY; k];
    let mut pred = vec![NO_CELL; k];
    let mut source = vec![0u32; k];
    let mut heap: BinaryHeap<Scored<u128>> = BinaryHeap::new();
    for &s in sources {
        let i = index.local(s);
        dist[i] = 0.0;
        source[i] = s;
        heap.push(Scored(0.0, heap_key(cells.coord(s), s)));
    }

    // Outside neighbors with the source each one seeds, sorted by cell.
    let mut outside: Vec<CellId> = Vec::new();
    for &w in cluster {
        for edge in cells.neighbors(w) {
            let n = edge.dest;
            if index.cluster(n) == NO_CLUSTER && state.dist[n as usize].is_finite() {
                outside.push(n);
            }
        }
    }
    outside.sort_unstable();
    outside.dedup();
    let seeds: Vec<(CellId, u32)> = outside
        .into_iter()
        .filter_map(|n| match frontier {
            Frontier::Any => Some((n, state.source[n as usize])),
            Frontier::Chained(is_source) => {
                chain_source(cells, state, index, n, is_source).map(|s| (n, s))
            }
        })
        .collect();
    for &(n, _) in &seeds {
        heap.push(Scored(state.dist[n as usize], heap_key(cells.coord(n), n)));
    }

    while let Some(Scored(d, key)) = heap.pop() {
        let u = heap_id(key);
        let (du, su) = if index.cluster(u) == me {
            let i = index.local(u);
            (dist[i], source[i])
        } else {
            let at = seeds
                .binary_search_by_key(&u, |&(n, _)| n)
                .expect("only seeds are pushed from outside the cluster");
            (state.dist[u as usize], seeds[at].1)
        };
        if d > du {
            continue;
        }
        for edge in cells.neighbors(u) {
            let v = edge.dest;
            if index.cluster(v) != me {
                continue;
            }
            let vi = index.local(v);
            let nd = d + weight.of(edge);
            if nd < dist[vi] {
                dist[vi] = nd;
                pred[vi] = u;
                source[vi] = su;
                heap.push(Scored(nd, heap_key(cells.coord(v), v)));
            }
        }
    }
    ClusterLabels { dist, pred, source }
}

/// Cells a labeled neighbor can step into that are still unreached after the
/// cluster searches, and the unreached cells passable from those. Inside the
/// window a cluster strands cells whose only way in runs through a neighbor
/// that hangs off another cluster. Beyond it, surface that had no way to a
/// node can gain one through the window.
pub fn stranded(
    cells: &SurfaceCells,
    window: &[CellId],
    state: &DijkstraState,
    weight: Weight,
) -> Vec<CellId> {
    let reached = |c: CellId| state.dist.get(c as usize).is_some_and(|d| d.is_finite());
    // The unreached end of every passable hop between a labeled and an
    // unreached cell that touches the window.
    let mut out: Vec<CellId> = window
        .par_iter()
        .flat_map_iter(|&u| {
            let labeled = reached(u);
            cells
                .neighbors(u)
                .iter()
                .filter(move |e| weight.of(e).is_finite() && labeled != reached(e.dest))
                .map(move |e| if labeled { e.dest } else { u })
        })
        .collect();
    out.sort_unstable();
    out.dedup();
    let mut seen: AHashSet<CellId> = out.iter().copied().collect();
    let mut next = 0;
    while next < out.len() {
        let u = out[next];
        next += 1;
        for e in cells.neighbors(u) {
            let v = e.dest;
            if weight.of(e).is_finite() && !reached(v) && seen.insert(v) {
                out.push(v);
            }
        }
    }
    out
}

/// Re-attach the cells outside the window whose cached chain ran through a
/// window cell that lost or changed its source, together with the window
/// cells the cluster searches left unreached. The first had labels built on
/// that cell. The second may only be reachable through a neighbor that was
/// waiting on another cluster. Both are searched again from the settled
/// cells around them. Returns the cells it searched.
pub fn reattach_descendants(
    cells: &SurfaceCells,
    changed: &[CellId],
    unreached: &[CellId],
    index: &ClusterIndex,
    state: &mut DijkstraState,
    weight: Weight,
) -> Vec<CellId> {
    // A cell has one pred, so the walk over children reaches each once.
    let mut patch: Vec<CellId> = unreached.to_vec();
    let mut stack: Vec<CellId> = changed.to_vec();
    while let Some(u) = stack.pop() {
        for edge in cells.neighbors(u) {
            let v = edge.dest;
            if !index.in_window(v) && state.pred[v as usize] == u {
                patch.push(v);
                stack.push(v);
            }
        }
    }
    if patch.is_empty() {
        return patch;
    }
    let inside: AHashSet<CellId> = patch.iter().copied().collect();
    for &c in &patch {
        let i = c as usize;
        state.dist[i] = f32::INFINITY;
        state.pred[i] = NO_CELL;
        state.source[i] = 0;
    }
    let mut heap: BinaryHeap<Scored<u128>> = BinaryHeap::new();
    let mut seeded: AHashSet<CellId> = AHashSet::new();
    for &c in &patch {
        for edge in cells.neighbors(c) {
            let n = edge.dest;
            if !inside.contains(&n) && state.dist[n as usize].is_finite() && seeded.insert(n) {
                heap.push(Scored(state.dist[n as usize], heap_key(cells.coord(n), n)));
            }
        }
    }
    while let Some(Scored(d, key)) = heap.pop() {
        let u = heap_id(key);
        if d > state.dist[u as usize] {
            continue;
        }
        let su = state.source[u as usize];
        for edge in cells.neighbors(u) {
            let v = edge.dest;
            if !inside.contains(&v) {
                continue;
            }
            let nd = d + weight.of(edge);
            if nd < state.dist[v as usize] {
                state.dist[v as usize] = nd;
                state.pred[v as usize] = u;
                state.source[v as usize] = su;
                heap.push(Scored(nd, heap_key(cells.coord(v), v)));
            }
        }
    }
    patch
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

/// Min-heap entry: cost first, with an ordered payload as a deterministic
/// tie-break.
pub(crate) struct Scored<T>(pub f32, pub T);

impl<T: Ord> PartialEq for Scored<T> {
    fn eq(&self, other: &Self) -> bool {
        self.0.total_cmp(&other.0) == Ordering::Equal && self.1 == other.1
    }
}
impl<T: Ord> Eq for Scored<T> {}
impl<T: Ord> PartialOrd for Scored<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<T: Ord> Ord for Scored<T> {
    fn cmp(&self, other: &Self) -> Ordering {
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
    fn clusters_label_like_the_whole_window_search() {
        let sc = grid(12);
        let sources = [sc.id((0, 0, 0)).unwrap(), sc.id((11, 11, 0)).unwrap()];
        let mut full = DijkstraState::default();
        dijkstra(&sc, &sources, &mut full, Weight::Penalized);

        // Two pieces that do not touch, re-labeled from new sources inside
        // them and the cached labels around them.
        let window: Vec<CellId> = sc
            .ids()
            .filter(|&id| {
                let (x, y, _) = sc.coord(id);
                (2..5).contains(&x) && (2..6).contains(&y)
                    || (7..10).contains(&x) && (5..9).contains(&y)
            })
            .collect();
        let new_sources = [sc.id((3, 3, 0)).unwrap(), sc.id((8, 7, 0)).unwrap()];

        let mut reference = DijkstraState {
            dist: full.dist.clone(),
            pred: full.pred.clone(),
            source: full.source.clone(),
            ..Default::default()
        };
        dijkstra_region(
            &sc,
            &new_sources,
            &window,
            &mut reference,
            Weight::Penalized,
        );

        let mut seen: Vec<bool> = Vec::new();
        let clusters = window_clusters(&sc, &window, &mut seen);
        assert_eq!(clusters.len(), 2);
        let mut index = ClusterIndex::default();
        index.assign(sc.slot_capacity(), &clusters);
        let mut clustered = DijkstraState {
            dist: full.dist.clone(),
            pred: full.pred.clone(),
            source: full.source.clone(),
            ..Default::default()
        };
        dijkstra_clusters(
            &sc,
            &new_sources,
            &clusters,
            &index,
            &mut clustered,
            Weight::Penalized,
            Frontier::Any,
        );

        for id in sc.ids() {
            let i = id as usize;
            assert_eq!(
                clustered.dist[i],
                reference.dist[i],
                "dist at {:?}",
                sc.coord(id)
            );
            assert_eq!(
                clustered.pred[i],
                reference.pred[i],
                "pred at {:?}",
                sc.coord(id)
            );
            assert_eq!(
                clustered.source[i],
                reference.source[i],
                "source at {:?}",
                sc.coord(id)
            );
        }
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

        let window: Vec<CellId> = sc.ids().collect();
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

        let window: Vec<CellId> = sc
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
