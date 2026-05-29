// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Multi-source Dijkstra over the cell-keyed surface adjacency.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use ahash::AHashMap;

use crate::adjacency::SurfaceAdjacency;
use crate::voxel::VoxelKey;

#[derive(Clone, Copy, Debug)]
pub struct CellState {
    pub dist: f32,
    /// Predecessor node along the shortest path to source.
    pub pred: Option<VoxelKey>,
    /// Id of cheapest source to return to.
    pub source: u32,
}

pub struct DijkstraResult {
    pub state: AHashMap<VoxelKey, CellState>,
}

impl DijkstraResult {
    pub fn dist_map(&self) -> AHashMap<VoxelKey, f32> {
        self.state.iter().map(|(&c, s)| (c, s.dist)).collect()
    }
}

pub fn dijkstra(adj: &SurfaceAdjacency, sources: &[VoxelKey]) -> DijkstraResult {
    let mut state: AHashMap<VoxelKey, CellState> = AHashMap::new();
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();

    for (label, &s) in sources.iter().enumerate() {
        if !adj.contains(s) {
            continue;
        }
        state.insert(
            s,
            CellState {
                dist: 0.0,
                pred: None,
                source: label as u32,
            },
        );
        heap.push(Scored(0.0, s));
    }

    while let Some(Scored(d, u)) = heap.pop() {
        let Some(&CellState {
            dist: cur,
            source: su,
            ..
        }) = state.get(&u)
        else {
            continue;
        };
        if d > cur {
            continue;
        }
        for edge in adj.neighbors(u) {
            let nd = d + edge.cost;
            let should_update = match state.get(&edge.dst) {
                None => true,
                Some(s) => nd < s.dist,
            };
            if should_update {
                state.insert(
                    edge.dst,
                    CellState {
                        dist: nd,
                        pred: Some(u),
                        source: su,
                    },
                );
                heap.push(Scored(nd, edge.dst));
            }
        }
    }

    DijkstraResult { state }
}

struct Scored(f32, VoxelKey);

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
        // Min-heap on f32 score.
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chain(n: i32) -> SurfaceAdjacency {
        let mut adj = SurfaceAdjacency::new();
        for i in 0..n {
            adj.add_cell((i, 0, 0));
        }
        for i in 0..n - 1 {
            adj.add_edge((i, 0, 0), (i + 1, 0, 0), 1.0);
            adj.add_edge((i + 1, 0, 0), (i, 0, 0), 1.0);
        }
        adj
    }

    #[test]
    fn single_source_dist_and_pred() {
        let adj = chain(5);
        let r = dijkstra(&adj, &[(0, 0, 0)]);
        for i in 0..5 {
            let s = r.state[&(i, 0, 0)];
            assert_eq!(s.dist, i as f32);
            assert_eq!(s.source, 0);
        }
        assert!(r.state[&(0, 0, 0)].pred.is_none());
        let mut cur = (4, 0, 0);
        let mut hops = 0;
        while let Some(p) = r.state[&cur].pred {
            cur = p;
            hops += 1;
        }
        assert_eq!(cur, (0, 0, 0));
        assert_eq!(hops, 4);
    }

    #[test]
    fn multi_source_labels_by_nearest() {
        // Sources at 0 and 4 on a 5-cell chain. Cells 0-1 closer to source 0,
        // cells 3-4 closer to source 1. Cell 2 is equidistant.
        let adj = chain(5);
        let r = dijkstra(&adj, &[(0, 0, 0), (4, 0, 0)]);
        assert_eq!(r.state[&(0, 0, 0)].source, 0);
        assert_eq!(r.state[&(1, 0, 0)].source, 0);
        assert_eq!(r.state[&(3, 0, 0)].source, 1);
        assert_eq!(r.state[&(4, 0, 0)].source, 1);
        // Tie at cell 2 must resolve to one of the two sources.
        let s2 = r.state[&(2, 0, 0)].source;
        assert!(s2 == 0 || s2 == 1);
        assert_eq!(r.state[&(0, 0, 0)].dist, 0.0);
        assert_eq!(r.state[&(1, 0, 0)].dist, 1.0);
        assert_eq!(r.state[&(2, 0, 0)].dist, 2.0);
        assert_eq!(r.state[&(3, 0, 0)].dist, 1.0);
        assert_eq!(r.state[&(4, 0, 0)].dist, 0.0);
    }

    #[test]
    fn disconnected_cells_stay_unreachable() {
        // Two separate chains 0-1 and 2-3, source at 0.
        let mut adj = SurfaceAdjacency::new();
        for &c in &[(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)] {
            adj.add_cell(c);
        }
        adj.add_edge((0, 0, 0), (1, 0, 0), 1.0);
        adj.add_edge((1, 0, 0), (0, 0, 0), 1.0);
        adj.add_edge((2, 0, 0), (3, 0, 0), 1.0);
        adj.add_edge((3, 0, 0), (2, 0, 0), 1.0);
        let r = dijkstra(&adj, &[(0, 0, 0)]);
        assert_eq!(r.state[&(0, 0, 0)].dist, 0.0);
        assert_eq!(r.state[&(1, 0, 0)].dist, 1.0);
        assert!(!r.state.contains_key(&(2, 0, 0)));
        assert!(!r.state.contains_key(&(3, 0, 0)));
    }

    #[test]
    fn shorter_path_overrides_longer() {
        // 0 - 1 with cost 10, 0 - 2 - 1 with cost 1+1=2.
        let mut adj = SurfaceAdjacency::new();
        for &c in &[(0, 0, 0), (1, 0, 0), (2, 0, 0)] {
            adj.add_cell(c);
        }
        adj.add_edge((0, 0, 0), (1, 0, 0), 10.0);
        adj.add_edge((1, 0, 0), (0, 0, 0), 10.0);
        adj.add_edge((0, 0, 0), (2, 0, 0), 1.0);
        adj.add_edge((2, 0, 0), (0, 0, 0), 1.0);
        adj.add_edge((2, 0, 0), (1, 0, 0), 1.0);
        adj.add_edge((1, 0, 0), (2, 0, 0), 1.0);
        let r = dijkstra(&adj, &[(0, 0, 0)]);
        assert_eq!(r.state[&(1, 0, 0)].dist, 2.0);
        assert_eq!(r.state[&(1, 0, 0)].pred, Some((2, 0, 0)));
    }
}
