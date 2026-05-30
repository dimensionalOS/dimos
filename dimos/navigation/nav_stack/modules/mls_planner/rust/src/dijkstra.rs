// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Multi-source Dijkstra over the CellId-indexed surface graph. State and
//! the heap live in a reusable struct so the inner loop never allocates.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::adjacency::{CellId, SurfaceCells, NO_CELL};

#[derive(Default)]
pub struct DijkstraState {
    pub dist: Vec<f32>,
    pub pred: Vec<CellId>,
    pub source: Vec<u32>,
    heap: BinaryHeap<Scored>,
}

impl DijkstraState {
    /// Resize parallel vecs to `n` slots and reset their contents to the
    /// "unreached" sentinels. Capacity is preserved across calls.
    pub fn reset(&mut self, n: usize) {
        self.dist.clear();
        self.dist.resize(n, f32::INFINITY);
        self.pred.clear();
        self.pred.resize(n, NO_CELL);
        self.source.clear();
        self.source.resize(n, 0);
        self.heap.clear();
    }
}

pub fn dijkstra(cells: &SurfaceCells, sources: &[CellId], state: &mut DijkstraState) {
    state.reset(cells.slot_capacity());

    for (label, &s) in sources.iter().enumerate() {
        if !cells.is_live(s) {
            continue;
        }
        state.dist[s as usize] = 0.0;
        state.source[s as usize] = label as u32;
        state.heap.push(Scored(0.0, s));
    }

    while let Some(Scored(d, u)) = state.heap.pop() {
        let cur = state.dist[u as usize];
        if d > cur {
            continue;
        }
        let su = state.source[u as usize];
        for edge in cells.neighbors(u) {
            let nd = d + edge.cost;
            let v = edge.dst as usize;
            if nd < state.dist[v] {
                state.dist[v] = nd;
                state.pred[v] = u;
                state.source[v] = su;
                state.heap.push(Scored(nd, edge.dst));
            }
        }
    }
}

/// Walk predecessors from `start` back to its source, returning the chain
/// inclusive of both ends. Returns just [start] for source cells and for
/// unreached cells.
pub fn walk_preds(state: &DijkstraState, start: CellId) -> Vec<CellId> {
    let mut cells = vec![start];
    let mut cur = start;
    loop {
        let p = state.pred[cur as usize];
        if p == NO_CELL {
            break;
        }
        cur = p;
        cells.push(cur);
    }
    cells
}

struct Scored(f32, CellId);

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
        // Min-heap on f32 score with CellId tie-break for determinism.
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::SurfaceCells;

    fn chain(n: i32) -> (SurfaceCells, Vec<CellId>) {
        let mut sc = SurfaceCells::default();
        let ids: Vec<CellId> = (0..n).map(|i| sc.alloc((i, 0, 0))).collect();
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
        dijkstra(&sc, &[ids[0]], &mut st);
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
        dijkstra(&sc, &[ids[0], ids[4]], &mut st);
        assert_eq!(st.source[ids[0] as usize], 0);
        assert_eq!(st.source[ids[1] as usize], 0);
        assert_eq!(st.source[ids[3] as usize], 1);
        assert_eq!(st.source[ids[4] as usize], 1);
        let s2 = st.source[ids[2] as usize];
        assert!(s2 == 0 || s2 == 1);
        assert_eq!(st.dist[ids[0] as usize], 0.0);
        assert_eq!(st.dist[ids[1] as usize], 1.0);
        assert_eq!(st.dist[ids[2] as usize], 2.0);
        assert_eq!(st.dist[ids[3] as usize], 1.0);
        assert_eq!(st.dist[ids[4] as usize], 0.0);
    }

    #[test]
    fn disconnected_cells_stay_unreachable() {
        let mut sc = SurfaceCells::default();
        let a = sc.alloc((0, 0, 0));
        let b = sc.alloc((1, 0, 0));
        let c = sc.alloc((2, 0, 0));
        let d = sc.alloc((3, 0, 0));
        sc.add_edge(a, b, 1.0);
        sc.add_edge(b, a, 1.0);
        sc.add_edge(c, d, 1.0);
        sc.add_edge(d, c, 1.0);
        let mut st = DijkstraState::default();
        dijkstra(&sc, &[a], &mut st);
        assert_eq!(st.dist[a as usize], 0.0);
        assert_eq!(st.dist[b as usize], 1.0);
        assert!(!st.dist[c as usize].is_finite());
        assert!(!st.dist[d as usize].is_finite());
    }

    #[test]
    fn shorter_path_overrides_longer() {
        let mut sc = SurfaceCells::default();
        let a = sc.alloc((0, 0, 0));
        let b = sc.alloc((1, 0, 0));
        let c = sc.alloc((2, 0, 0));
        sc.add_edge(a, b, 10.0);
        sc.add_edge(b, a, 10.0);
        sc.add_edge(a, c, 1.0);
        sc.add_edge(c, a, 1.0);
        sc.add_edge(c, b, 1.0);
        sc.add_edge(b, c, 1.0);
        let mut st = DijkstraState::default();
        dijkstra(&sc, &[a], &mut st);
        assert_eq!(st.dist[b as usize], 2.0);
        assert_eq!(st.pred[b as usize], c);
    }

    #[test]
    fn buffer_reuse_does_not_leak_prior_state() {
        let (sc1, ids1) = chain(5);
        let mut st = DijkstraState::default();
        dijkstra(&sc1, &[ids1[0]], &mut st);
        let (sc2, ids2) = chain(3);
        dijkstra(&sc2, &[ids2[0]], &mut st);
        for (i, &id) in ids2.iter().enumerate().take(3) {
            assert_eq!(st.dist[id as usize], i as f32);
        }
    }
}
