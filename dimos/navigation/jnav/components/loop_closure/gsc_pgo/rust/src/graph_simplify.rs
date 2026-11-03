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

//! Graph simplification (`Config::graph_simplify`) for very long runs: a reduced pose graph whose size follows the mapped
//! area, not the run's length.
//!
//! Each keyframe gets a fixed placement once it leaves the frontier (the newest `graph_simplify_frontier_keyframes`) and an
//! adopted solve has placed it:
//! - kept: its own node in every later solve (it keeps its cloud and descriptor, so loops can still close onto it);
//! - removed: it lies within `graph_simplify_place_radius_m` of a keyframe kept on an older pass, and a closure the newest
//!   solve kept is within `graph_simplify_localized_keyframes` of it (the map has localized this stretch). It rides that
//!   kept keyframe rigidly at the offset the adopted graph gave it, its cloud and descriptor are freed, and it is no closure
//!   target any more.
//!
//! The odometry is folded walking the chain: steps whose ends ride the same node compose into one edge from the node the
//! walk entered on to the node it leaves for, with the steps' variances summed (each is linear in distance and turn), and
//! only the first edge between a pair of nodes is kept. An edge that reaches a node through a frozen offset also carries
//! `graph_simplify_offset_sd_*` per such end: the offset is the adopted graph's guess, and taken as exact (odometry-tight)
//! it outvotes true closures (the GNC then rejects them and the map drifts lap by lap). A closure's ends ride their nodes the same way (one riding a
//! single node drops out); a folded closure goes to the solve only while its node pair has fewer than
//! `graph_simplify_closures_per_pair`, the rest keep their last verdict (closures between kept keyframes always go in). The solve's nodes expand back to every keyframe's pose.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use crate::big_picture_fit::{motion_variances, Edge, Solution};
use crate::gsc_pgo::{Config, GscPgo};
use crate::mat3::{self, Mat3, Vec3};
use crate::store::{Appended, Closures, KeyFrames};

/// A rigid pose (rotation, translation).
pub type Pose = (Mat3, Vec3);

pub fn compose(a: &Pose, b: &Pose) -> Pose {
    (mat3::mat_mul(&a.0, &b.0), mat3::add(&mat3::mat_vec(&a.0, &b.1), &a.1))
}

pub fn invert(a: &Pose) -> Pose {
    let back = mat3::transpose(&a.0);
    (back, mat3::mat_vec(&back, &a.1).map(|v| -v))
}

fn identity() -> Pose {
    (mat3::identity(), [0.0; 3])
}

/// Where a decided keyframe is in the reduced graph.
#[derive(Clone, Copy, Debug)]
pub enum Placement {
    /// Its own node (index into the kept nodes).
    Node(usize),
    /// Rides node `node` at `offset` (its pose = node's pose * offset).
    Anchored { node: usize, offset: Pose },
}

impl Placement {
    pub fn node_and_offset(&self) -> (usize, Pose) {
        match *self {
            Placement::Node(node) => (node, identity()),
            Placement::Anchored { node, offset } => (node, offset),
        }
    }
}

/// An odometry edge between two nodes: the composed relative pose and the distance and turn it spans.
#[derive(Clone, Copy, Debug)]
pub struct FoldedEdge {
    pub from: usize,
    pub to: usize,
    pub relative: Pose,
    pub distance: f64,
    pub turn: f64,
    /// Ends reached through a frozen offset (0-2): each adds the offset's variance (`graph_simplify_offset_sd_*`).
    pub anchored_ends: u32,
}

impl FoldedEdge {
    /// As a solve edge, its variances the reported odometry's over its distance and turn times `noise`.
    pub fn edge(&self, config: &Config, noise: [f64; 4]) -> Edge {
        let [xy, z, yaw, tilt]: [f64; 4] = {
            let v = motion_variances(config, self.distance, self.turn);
            std::array::from_fn(|c| v[c] * noise[c])
        };
        let mut variances = [tilt, tilt, yaw, xy, xy, z];
        add_offset_variances(&mut variances, config, self.anchored_ends);
        Edge {
            from: self.from,
            to: self.to,
            rotation: self.relative.0,
            translation: self.relative.1,
            variances,
        }
    }
}

/// Add `ends` frozen offsets' variances (rotation, then translation, in `Edge` order) to an edge's.
fn add_offset_variances(variances: &mut [f64; 6], config: &Config, ends: u32) {
    let (rotation, translation) =
        (config.graph_simplify_offset_sd_rad.powi(2), config.graph_simplify_offset_sd_m.powi(2));
    for (axis, variance) in variances.iter_mut().enumerate() {
        *variance += ends as f64 * if axis < 3 { rotation } else { translation };
    }
}

/// The chain walk at the newest decided keyframe: the node it rides, its offset there, distance and turn since entering.
#[derive(Clone, Copy, Debug)]
struct Walk {
    node: usize,
    offset: Pose,
    distance: f64,
    turn: f64,
    /// It entered the node through a frozen offset (a removed keyframe), not as the node itself.
    anchored: bool,
}

/// The reduced graph's state (see the module).
#[derive(Default)]
pub struct GraphSimplify {
    /// Placement per decided keyframe (the first `placements.len()`), and the keyframe of each kept node.
    pub(crate) placements: Appended<Placement>,
    pub(crate) kept: Appended<usize>,
    /// Kept keyframes per place cell (`graph_simplify_place_radius_m` cubes), for the nearest-kept lookup.
    cells: HashMap<[i64; 3], Vec<usize>>,
    walk: Option<Walk>,
    /// Folded odometry among kept nodes, and the node pairs it already joins.
    pub(crate) edges: Vec<FoldedEdge>,
    pairs: HashSet<(usize, usize)>,
    /// Ends of the closures the newest adopted solve kept, sorted (the localized test).
    localizing: Vec<usize>,
    /// Removed keyframes waiting for every snapshot that could still read their cloud (FIFO).
    pending_release: VecDeque<(usize, Arc<()>)>,
    pub(crate) removed: usize,
    /// The newest simplified solve's (nodes, closures).
    pub(crate) last_solve: (usize, usize),
}

/// The step from keyframe `from` to `to` (odometry frame): relative pose, distance, turn.
pub fn odometry_step(key_poses: &KeyFrames, from: usize, to: usize) -> (Pose, f64, f64) {
    let (a, b) = (key_poses.fixed(from), key_poses.fixed(to));
    let back = mat3::transpose(&a.rotation_local);
    (
        (
            mat3::mat_mul(&back, &b.rotation_local),
            mat3::mat_vec(&back, &mat3::sub(&b.translation_local, &a.translation_local)),
        ),
        mat3::norm(&mat3::sub(&b.translation_local, &a.translation_local)),
        mat3::angular_distance(&a.rotation_local, &b.rotation_local),
    )
}

fn cell(position: &Vec3, size: f64) -> [i64; 3] {
    position.map(|v| (v / size).floor() as i64)
}

impl GraphSimplify {
    /// Decided keyframes (every one below this has its placement).
    pub fn decided(&self) -> usize {
        self.placements.len()
    }
    /// Nodes a solve over `keyframes` keyframes has: the kept ones plus the undecided frontier.
    pub fn nodes(&self, keyframes: usize) -> usize {
        self.kept.len() + keyframes.saturating_sub(self.decided())
    }
    /// Keyframe `k`'s node and its offset there in a solve over keyframes up to the newest (frontier ones are their own).
    pub fn place(&self, k: usize) -> (usize, Pose) {
        if k < self.decided() {
            self.placements[k].node_and_offset()
        } else {
            (self.kept.len() + k - self.decided(), identity())
        }
    }
    /// Advance the chain walk over decided keyframe `k` placed at `placement`, folding the step into it.
    pub(crate) fn walk_to(&mut self, key_poses: &KeyFrames, k: usize, placement: Placement) {
        let (node, offset) = placement.node_and_offset();
        let anchored = matches!(placement, Placement::Anchored { .. });
        let Some(walk) = self.walk else {
            self.walk = Some(Walk { node, offset, distance: 0.0, turn: 0.0, anchored });
            return;
        };
        let (step, distance, turn) = odometry_step(key_poses, k - 1, k);
        let reached = compose(&walk.offset, &step);
        let (distance, turn) = (walk.distance + distance, walk.turn + turn);
        if node == walk.node {
            self.walk = Some(Walk { offset: reached, distance, turn, ..walk });
            return;
        }
        // leaving the node: one edge from the entry node to this one over every step since entering
        let pair = (walk.node.min(node), walk.node.max(node));
        if self.pairs.insert(pair) {
            self.edges.push(FoldedEdge {
                from: walk.node,
                to: node,
                relative: compose(&reached, &invert(&offset)),
                distance,
                turn,
                anchored_ends: walk.anchored as u32 + anchored as u32,
            });
        }
        self.walk = Some(Walk { node, offset, distance: 0.0, turn: 0.0, anchored });
    }
    /// The folded odometry of a solve over `keyframes` keyframes: kept edges, the walk into the frontier, the frontier chain.
    pub fn odometry(&self, key_poses: &KeyFrames, keyframes: usize) -> Vec<FoldedEdge> {
        let mut edges = self.edges.clone();
        let decided = self.decided();
        for k in decided.max(1)..keyframes {
            let (step, distance, turn) = odometry_step(key_poses, k - 1, k);
            let to = self.place(k).0;
            match (k == decided, self.walk) {
                (true, Some(walk)) => edges.push(FoldedEdge {
                    from: walk.node,
                    to,
                    relative: compose(&walk.offset, &step),
                    distance: walk.distance + distance,
                    turn: walk.turn + turn,
                    anchored_ends: walk.anchored as u32,
                }),
                _ => edges.push(FoldedEdge { from: to - 1, to, relative: step, distance, turn, anchored_ends: 0 }),
            }
        }
        edges
    }
    /// Closures as node edges (closure index, edge); one riding a single node drops out, and once a node pair has
    /// `graph_simplify_closures_per_pair` (0 = no cap) closures, further ones folded onto it wait out of the solve.
    pub fn closures(&self, closures: &Closures, count: usize, config: &Config) -> Vec<(usize, Edge)> {
        let per_pair = config.graph_simplify_closures_per_pair;
        let mut per: HashMap<(usize, usize), usize> = HashMap::new();
        (0..count)
            .filter_map(|i| {
                let fixed = closures.fixed(i);
                let (target, target_offset) = self.place(fixed.target_id);
                let (source, source_offset) = self.place(fixed.source_id);
                if target == source {
                    return None;
                }
                // only a closure folded onto a node (an end removed) can be left out; one between its own keyframes always goes in
                let ends = (target_offset != identity()) as u32 + (source_offset != identity()) as u32;
                let taken = per.entry((target.min(source), target.max(source))).or_default();
                if ends > 0 && per_pair > 0 && *taken >= per_pair {
                    return None;
                }
                *taken += 1;
                let relative = compose(
                    &compose(&target_offset, &(fixed.rotation_offset, fixed.translation_offset)),
                    &invert(&source_offset),
                );
                let mut variances = [fixed.score.max(1e-6); 6];
                add_offset_variances(&mut variances, config, ends);
                Some((
                    i,
                    Edge {
                        from: target,
                        to: source,
                        rotation: relative.0,
                        translation: relative.1,
                        variances,
                    },
                ))
            })
            .collect()
    }
}

/// `GscPgo::simplify_stats`.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct SimplifyStats {
    pub nodes: usize,
    pub removed: usize,
    pub folded_edges: usize,
    pub solve_nodes: usize,
    pub solve_closures: usize,
    /// Removed keyframes whose clouds still wait for an older snapshot to go.
    pub awaiting_release: usize,
}

/// What a simplified solve needs to expand its node solution back to every keyframe (taken at dispatch).
pub struct Expansion {
    pub placements: Appended<Placement>,
    pub kept: usize,
    pub keyframes: usize,
    /// Per solve closure, its committed closure index; the committed count and their verdicts at dispatch.
    pub closure_index: Vec<usize>,
    pub closures: Closures,
    pub closure_count: usize,
}

impl Expansion {
    fn node(&self, k: usize) -> (usize, Pose) {
        let decided = self.placements.len();
        if k < decided {
            self.placements[k].node_and_offset()
        } else {
            (self.kept + k - decided, identity())
        }
    }
    /// The node solution as every keyframe's (removed ones ride their node) and every committed closure's verdict.
    pub fn expand(&self, mut solution: Solution) -> Solution {
        let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = (0..self.keyframes)
            .map(|k| {
                let (node, offset) = self.node(k);
                compose(&(solution.rotations[node], solution.positions[node]), &offset)
            })
            .unzip();
        let covariance = std::mem::take(&mut solution.relative_position_covariance);
        if covariance.len() == solution.rotations.len() {
            solution.relative_position_covariance =
                (0..self.keyframes).map(|k| covariance[self.node(k).0]).collect();
        }
        solution.rotations = rotations;
        solution.positions = positions;
        // a closure out of this solve keeps its last verdict
        let spread = |weights: &[f64]| -> Vec<f64> {
            let mut all: Vec<f64> = (0..self.closure_count).map(|i| self.closures.weight(i)).collect();
            for (&index, &weight) in self.closure_index.iter().zip(weights) {
                all[index] = weight;
            }
            all
        };
        solution.closure_weights = spread(&solution.closure_weights);
        if let Some(trace) = &mut solution.trace {
            trace.gnc_weights = spread(&trace.gnc_weights);
        }
        solution
    }
}

impl GscPgo {
    /// Whether `graph_simplify` is on.
    pub(crate) fn simplifying(&self) -> bool {
        self.config.graph_simplify
    }
    /// Decide every keyframe that has left the frontier, then free the clouds no snapshot can read any more.
    pub(crate) fn simplify_step(&mut self) {
        if !self.simplifying() {
            return;
        }
        let frontier = self.config.graph_simplify_frontier_keyframes.max(1);
        // a keyframe is decided from the pose an adopted solve gave it, never from its odometry ride
        while self.simplify.decided() + frontier < self.key_poses.len()
            && self.simplify.decided() < self.key_poses.solved_len()
        {
            self.decide(self.simplify.decided());
        }
        self.release_simplified();
    }
    /// Place keyframe `k` (see the module) from the adopted graph.
    fn decide(&mut self, k: usize) {
        let radius = self.config.graph_simplify_place_radius_m.max(1e-3);
        let frontier = self.config.graph_simplify_frontier_keyframes.max(1);
        let localized_within = self.config.graph_simplify_localized_keyframes;
        let here = self.key_poses.global(k);
        let localized = {
            let localizing = &self.simplify.localizing;
            let first = localizing.partition_point(|&end| end + localized_within < k);
            localizing.get(first).is_some_and(|&end| end <= k + localized_within)
        };
        // the nearest keyframe kept on an older pass within the place radius
        let anchor = if localized && k > 0 {
            let centre = cell(&here.1, radius);
            let mut best: Option<(f64, usize)> = None;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let key = [centre[0] + dx, centre[1] + dy, centre[2] + dz];
                        for &kept in self.simplify.cells.get(&key).into_iter().flatten() {
                            if kept + frontier >= k {
                                continue;
                            }
                            let d = mat3::norm(&mat3::sub(&self.key_poses.position(kept), &here.1));
                            if d <= radius && best.is_none_or(|(b, i)| (d, kept) < (b, i)) {
                                best = Some((d, kept));
                            }
                        }
                    }
                }
            }
            best.map(|(_, kept)| kept)
        } else {
            None
        };
        let placement = match anchor {
            Some(kept) => {
                let node = match self.simplify.placements[kept] {
                    Placement::Node(node) => node,
                    Placement::Anchored { .. } => unreachable!("only kept keyframes are in the place cells"),
                };
                let offset = compose(&invert(&self.key_poses.global(kept)), &here);
                let guard = self.key_poses.remove(k);
                self.simplify.pending_release.push_back((k, guard));
                self.scan_context_ring_keys[k] = Vec::new();
                self.simplify.removed += 1;
                Placement::Anchored { node, offset }
            }
            None => {
                let node = self.simplify.kept.len();
                self.simplify.kept.push(k);
                self.simplify.cells.entry(cell(&here.1, radius)).or_default().push(k);
                Placement::Node(node)
            }
        };
        self.simplify.placements.push(placement);
        self.simplify.walk_to(&self.key_poses, k, placement);
    }
    /// Free removed keyframes' clouds in removal order once no snapshot from before their removal is left.
    fn release_simplified(&mut self) {
        while let Some((k, guard)) = self.simplify.pending_release.front() {
            if Arc::strong_count(guard) > 1 {
                return;
            }
            self.key_poses.release(*k);
            self.keyframe_clouds.forget(*k);
            self.simplify.pending_release.pop_front();
        }
    }
    /// After an adoption: the ends of the closures it kept localize the keyframes near them.
    pub(crate) fn simplify_note_adoption(&mut self) {
        if !self.simplifying() {
            return;
        }
        let mut ends: Vec<usize> = self
            .committed_closures
            .iter()
            .filter(|c| c.gnc_weight > 0.5)
            .flat_map(|c| [c.source_id, c.target_id])
            .collect();
        ends.sort_unstable();
        ends.dedup();
        self.simplify.localizing = ends;
        self.simplify_step();
    }
    /// The reduced graph over the keyframes so far: nodes, removed keyframes, folded odometry edges, and the newest
    /// solve's nodes and closures.
    pub fn simplify_stats(&self) -> SimplifyStats {
        SimplifyStats {
            nodes: self.simplify.nodes(self.key_poses.len()),
            removed: self.simplify.removed,
            folded_edges: self.simplify.edges.len(),
            solve_nodes: self.simplify.last_solve.0,
            solve_closures: self.simplify.last_solve.1,
            awaiting_release: self.simplify.pending_release.len(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::big_picture_fit::odometry_edges;
    use crate::store::KeyFixed;

    fn chain(poses: &[(f64, f64, f64)]) -> KeyFrames {
        let mut key_poses = KeyFrames::default();
        for (i, &(x, y, yaw)) in poses.iter().enumerate() {
            key_poses.push(KeyFixed::new(
                mat3::rot_z(yaw),
                [x, y, 0.1 * i as f64],
                i as f64,
                None,
                Arc::new(crate::similarity_discovery::Descriptor::empty()),
            ));
        }
        key_poses
    }

    fn close(a: &Pose, b: &Pose) -> bool {
        (0..3).all(|i| (a.1[i] - b.1[i]).abs() < 1e-9 && (0..3).all(|j| (a.0[i][j] - b.0[i][j]).abs() < 1e-9))
    }

    /// Keyframes riding a node by their odometry fold into one edge equal to the chained steps, variances summed.
    #[test]
    fn a_folded_edge_equals_the_chained_steps() {
        let key_poses = chain(&[(0.0, 0.0, 0.0), (1.0, 0.2, 0.3), (1.8, 1.0, 0.9), (2.0, 2.2, 1.4), (2.5, 3.0, 1.0)]);
        let config = Config::default();
        let mut simplify = GraphSimplify::default();
        // 0 kept; 1 and 2 removed, riding node 0 at their odometry offsets; 3 kept
        let mut offset = identity();
        for k in 0..4 {
            let placement = if k == 0 || k == 3 {
                let node = simplify.kept.len();
                simplify.kept.push(k);
                Placement::Node(node)
            } else {
                offset = compose(&offset, &odometry_step(&key_poses, k - 1, k).0);
                Placement::Anchored { node: 0, offset }
            };
            simplify.placements.push(placement);
            simplify.walk_to(&key_poses, k, placement);
        }
        assert_eq!(simplify.edges.len(), 1);
        let steps = odometry_edges(&key_poses, &config, [1.0; 4]);
        let chained = steps[..3]
            .iter()
            .fold(identity(), |pose, e| compose(&pose, &(e.rotation, e.translation)));
        let folded = simplify.edges[0].edge(&config, [1.0; 4]);
        assert_eq!((folded.from, folded.to), (0, 1));
        assert!(close(&(folded.rotation, folded.translation), &chained));
        for axis in 0..6 {
            let summed: f64 = steps[..3].iter().map(|e| e.variances[axis]).sum();
            assert!((folded.variances[axis] - summed).abs() < 1e-15 * summed.max(1.0), "axis {axis}");
        }
        // keyframe 4 is the frontier: its own node, joined to node 1 by its plain step
        let odometry = simplify.odometry(&key_poses, 5);
        assert_eq!(odometry.len(), 2);
        assert_eq!((odometry[1].from, odometry[1].to), (1, 2));
        assert!(close(&odometry[1].relative, &(steps[3].rotation, steps[3].translation)));
    }

    /// A closure onto a removed keyframe rides its node: the node-to-node edge reproduces the same keyframe poses.
    #[test]
    fn a_closure_on_a_removed_keyframe_rides_its_node() {
        let mut simplify = GraphSimplify::default();
        let offset: Pose = (mat3::rot_z(0.4), [0.3, -0.2, 0.05]);
        simplify.kept.push(0);
        simplify.placements.push(Placement::Node(0));
        simplify.kept.push(1);
        simplify.placements.push(Placement::Node(1));
        simplify.placements.push(Placement::Anchored { node: 0, offset });
        // closure: source 2 seen from target 1
        let mut closures = Closures::default();
        let measured: Pose = (mat3::rot_z(-1.1), [4.0, 1.0, 0.2]);
        closures.push(crate::store::ClosureFixed {
            source_id: 2,
            target_id: 1,
            rotation_offset: measured.0,
            translation_offset: measured.1,
            score: 0.01,
            fit_confidence: 1.0,
            yank_stdevs: 0.0,
        });
        let config = Config { graph_simplify_closures_per_pair: 0, ..Config::default() };
        let edges = simplify.closures(&closures, 1, &config);
        assert_eq!(edges.len(), 1);
        let (index, edge) = edges[0];
        assert_eq!((index, edge.from, edge.to), (0, 1, 0));
        // any node poses that satisfy the node edge put keyframe 2 where the original closure says
        let node1: Pose = (mat3::rot_z(0.7), [5.0, -2.0, 0.3]);
        let node0 = compose(&node1, &(edge.rotation, edge.translation));
        let keyframe2 = compose(&node0, &offset);
        assert!(close(&compose(&node1, &measured), &keyframe2));
        // a closure between two keyframes of one node drops out
        closures.push(crate::store::ClosureFixed { source_id: 2, target_id: 0, ..*closures.fixed(0) });
        assert_eq!(simplify.closures(&closures, 2, &config).len(), 1);
    }

    /// Past the per-pair cap, closures folded onto the same nodes wait out of the solve and keep their verdicts; closures
    /// between kept keyframes always go in.
    #[test]
    fn closures_past_the_per_pair_cap_keep_their_verdict() {
        let mut simplify = GraphSimplify::default();
        for k in 0..2 {
            simplify.kept.push(k);
            simplify.placements.push(Placement::Node(k));
        }
        // keyframe 2 rides node 1
        simplify.placements.push(Placement::Anchored { node: 1, offset: (mat3::rot_z(0.1), [0.2, 0.0, 0.0]) });
        let mut closures = Closures::default();
        for i in 0..4 {
            closures.push(crate::store::ClosureFixed {
                source_id: if i == 0 { 1 } else { 2 },
                target_id: 0,
                rotation_offset: mat3::identity(),
                translation_offset: [1.0, 0.0, 0.0],
                score: 0.01,
                fit_confidence: 1.0,
                yank_stdevs: 0.0,
            });
        }
        closures.adopt(Arc::new(vec![1.0, 1.0, 0.0, 1.0]));
        let config = Config { graph_simplify_closures_per_pair: 2, ..Config::default() };
        let edges = simplify.closures(&closures, 4, &config);
        assert_eq!(edges.iter().map(|e| e.0).collect::<Vec<_>>(), vec![0, 1]);
        let mut unfolded = Closures::default();
        for _ in 0..4 {
            unfolded.push(crate::store::ClosureFixed { source_id: 1, ..*closures.fixed(0) });
        }
        assert_eq!(simplify.closures(&unfolded, 4, &config).len(), 4);
        let expansion = Expansion {
            placements: simplify.placements.clone(),
            kept: 2,
            keyframes: 3,
            closure_index: vec![0, 1],
            closures: closures.clone(),
            closure_count: 4,
        };
        let solution = Solution {
            rotations: vec![mat3::identity(); 2],
            positions: vec![[0.0; 3], [1.0, 0.0, 0.0]],
            closure_weights: vec![0.0, 1.0],
            prior_weights: Vec::new(),
            world_alignment: Vec::new(),
            landmarks: Vec::new(),
            landmark_rotations: Vec::new(),
            sighting_weights: Vec::new(),
            error: 0.0,
            relative_position_covariance: Vec::new(),
            converged: true,
            odometry_error_estimate_evidence: None,
            trace: None,
            seeded: false,
        };
        let expanded = expansion.expand(solution);
        assert_eq!(expanded.closure_weights, vec![0.0, 1.0, 0.0, 1.0]);
        // the removed keyframe rides node 1
        assert!(close(&(expanded.rotations[2], expanded.positions[2]), &(mat3::rot_z(0.1), [1.2, 0.0, 0.0])));
    }
}
