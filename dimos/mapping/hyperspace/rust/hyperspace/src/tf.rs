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

//! A tf-style transform graph. Each (parent, child) edge holds timestamped
//! samples that are interpolated (lerp + slerp) and clamped at the ends. A
//! child may have several parents (e.g. `odom -> lidar` from odometry and
//! `base_link -> lidar` from the robot description), so lookups walk the
//! undirected graph and invert edges as needed.

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};

/// One timestamped transform edge: pose of `child_frame` expressed in `parent_frame`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Transform {
    pub parent_frame: String,
    pub child_frame: String,
    /// Seconds (any consistent epoch).
    pub timestamp: f64,
    pub translation: [f64; 3],
    /// Quaternion [x, y, z, w].
    pub rotation: [f64; 4],
}

impl Transform {
    pub fn from_isometry(
        parent_frame: &str,
        child_frame: &str,
        timestamp: f64,
        pose: &Isometry3<f64>,
    ) -> Self {
        let q = pose.rotation.quaternion();
        Transform {
            parent_frame: parent_frame.into(),
            child_frame: child_frame.into(),
            timestamp,
            translation: [pose.translation.x, pose.translation.y, pose.translation.z],
            rotation: [q.i, q.j, q.k, q.w],
        }
    }

    pub fn to_isometry(&self) -> Isometry3<f64> {
        let [x, y, z, w] = self.rotation;
        Isometry3::from_parts(
            Translation3::new(
                self.translation[0],
                self.translation[1],
                self.translation[2],
            ),
            UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)),
        )
    }
}

#[derive(Default)]
struct Edge {
    /// (timestamp, parent_from_child), sorted by timestamp.
    samples: Vec<(f64, Isometry3<f64>)>,
}

#[derive(Default)]
pub struct TfTree {
    /// (parent, child) -> samples
    edges: HashMap<(String, String), Edge>,
    /// frame -> neighbouring frames (both directions)
    adjacency: HashMap<String, Vec<String>>,
}

impl TfTree {
    pub fn insert(&mut self, transform: &Transform) {
        let isometry = transform.to_isometry();
        let key = (
            transform.parent_frame.clone(),
            transform.child_frame.clone(),
        );
        if !self.edges.contains_key(&key) {
            self.adjacency
                .entry(transform.parent_frame.clone())
                .or_default()
                .push(transform.child_frame.clone());
            self.adjacency
                .entry(transform.child_frame.clone())
                .or_default()
                .push(transform.parent_frame.clone());
        }
        let edge = self.edges.entry(key).or_default();
        let position = edge
            .samples
            .partition_point(|(sample_time, _)| *sample_time < transform.timestamp);
        // Same parent/child/timestamp again = a rewrite (loop closure), not a duplicate.
        if position < edge.samples.len() && edge.samples[position].0 == transform.timestamp {
            edge.samples[position].1 = isometry;
        } else {
            edge.samples
                .insert(position, (transform.timestamp, isometry));
        }
    }

    /// Replace every sample of the `parent -> child` edge (bulk loop-closure rewrite).
    pub fn replace_edge(
        &mut self,
        parent_frame: &str,
        child_frame: &str,
        samples: Vec<(f64, Isometry3<f64>)>,
    ) {
        let mut samples = samples;
        samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let key = (parent_frame.to_string(), child_frame.to_string());
        if !self.edges.contains_key(&key) {
            self.adjacency
                .entry(parent_frame.to_string())
                .or_default()
                .push(child_frame.to_string());
            self.adjacency
                .entry(child_frame.to_string())
                .or_default()
                .push(parent_frame.to_string());
        }
        self.edges.insert(key, Edge { samples });
    }

    /// Pose of `source_frame` expressed in `target_frame` at `timestamp`,
    /// i.e. target_from_source. None if the frames are not connected.
    pub fn get(
        &self,
        target_frame: &str,
        source_frame: &str,
        timestamp: f64,
    ) -> Option<Isometry3<f64>> {
        if target_frame == source_frame {
            return Some(Isometry3::identity());
        }
        let path = self.path(source_frame, target_frame)?;
        let mut target_from_source = Isometry3::identity();
        for pair in path.windows(2) {
            let (from, to) = (&pair[0], &pair[1]);
            // to_from_from: either `to` is the parent of `from`, or the inverse of the reverse edge.
            let to_from_from = if let Some(edge) = self.edges.get(&(to.clone(), from.clone())) {
                edge.sample_at(timestamp)?
            } else {
                self.edges
                    .get(&(from.clone(), to.clone()))?
                    .sample_at(timestamp)?
                    .inverse()
            };
            target_from_source = to_from_from * target_from_source;
        }
        Some(target_from_source)
    }

    /// Shortest frame path from `start` to `goal` (BFS over the undirected graph).
    fn path(&self, start: &str, goal: &str) -> Option<Vec<String>> {
        let mut previous: HashMap<String, String> = HashMap::new();
        let mut seen: HashSet<String> = HashSet::from([start.to_string()]);
        let mut queue = VecDeque::from([start.to_string()]);
        while let Some(frame) = queue.pop_front() {
            if frame == goal {
                let mut path = vec![goal.to_string()];
                let mut current = goal.to_string();
                while let Some(prev) = previous.get(&current) {
                    path.push(prev.clone());
                    current = prev.clone();
                }
                path.reverse();
                return Some(path);
            }
            for neighbour in self.adjacency.get(&frame).into_iter().flatten() {
                if seen.insert(neighbour.clone()) {
                    previous.insert(neighbour.clone(), frame.clone());
                    queue.push_back(neighbour.clone());
                }
            }
        }
        None
    }

    /// Angular (rad/s) and linear (m/s) speed of `source_frame` relative to
    /// `target_frame` around `timestamp`, from finite differences over ±half_window.
    pub fn speeds(
        &self,
        target_frame: &str,
        source_frame: &str,
        timestamp: f64,
        half_window: f64,
    ) -> Option<(f64, f64)> {
        let before = self.get(target_frame, source_frame, timestamp - half_window)?;
        let after = self.get(target_frame, source_frame, timestamp + half_window)?;
        let delta = before.inverse() * after;
        let dt = 2.0 * half_window;
        Some((
            delta.rotation.angle() / dt,
            delta.translation.vector.norm() / dt,
        ))
    }

    /// Every frame name known to the graph.
    pub fn frames(&self) -> Vec<String> {
        self.adjacency.keys().cloned().collect()
    }

    /// All samples, for persistence: (parent, child, timestamp, pose).
    pub fn samples(&self) -> impl Iterator<Item = (&str, &str, f64, &Isometry3<f64>)> + '_ {
        self.edges.iter().flat_map(|((parent, child), edge)| {
            edge.samples
                .iter()
                .map(move |(t, pose)| (parent.as_str(), child.as_str(), *t, pose))
        })
    }

    pub fn sample_count(&self) -> usize {
        self.edges.values().map(|e| e.samples.len()).sum()
    }
}

impl Edge {
    fn sample_at(&self, timestamp: f64) -> Option<Isometry3<f64>> {
        let samples = &self.samples;
        if samples.is_empty() {
            return None;
        }
        let after = samples.partition_point(|(sample_time, _)| *sample_time <= timestamp);
        if after == 0 {
            return Some(samples[0].1);
        }
        if after == samples.len() {
            return Some(samples[samples.len() - 1].1);
        }
        let (time_before, ref pose_before) = samples[after - 1];
        let (time_after, ref pose_after) = samples[after];
        let span = time_after - time_before;
        if span <= 0.0 {
            return Some(*pose_before);
        }
        let fraction = (timestamp - time_before) / span;
        Some(pose_before.lerp_slerp(pose_after, fraction))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    fn transform(parent: &str, child: &str, timestamp: f64, translation: [f64; 3]) -> Transform {
        Transform {
            parent_frame: parent.into(),
            child_frame: child.into(),
            timestamp,
            translation,
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn interpolates_between_samples() {
        let mut tree = TfTree::default();
        tree.insert(&transform("world", "base", 0.0, [0.0, 0.0, 0.0]));
        tree.insert(&transform("world", "base", 10.0, [10.0, 0.0, 0.0]));
        let world_from_base = tree.get("world", "base", 5.0).unwrap();
        assert!((world_from_base.translation.x - 5.0).abs() < 1e-9);
    }

    #[test]
    fn composes_across_common_ancestor() {
        let mut tree = TfTree::default();
        tree.insert(&transform("world", "base", 0.0, [1.0, 0.0, 0.0]));
        tree.insert(&transform("base", "camera", 0.0, [0.0, 2.0, 0.0]));
        tree.insert(&transform("base", "lidar", 0.0, [0.0, 0.0, 3.0]));
        let world_from_camera = tree.get("world", "camera", 0.0).unwrap();
        let point_in_world = world_from_camera * Point3::origin();
        assert_eq!((point_in_world.x, point_in_world.y), (1.0, 2.0));
        let camera_from_lidar = tree.get("camera", "lidar", 0.0).unwrap();
        let offset = camera_from_lidar * Point3::origin();
        assert_eq!((offset.y, offset.z), (-2.0, 3.0));
    }

    #[test]
    fn child_with_two_parents_is_a_bridge() {
        // odom -> lidar (odometry) and base -> lidar (static rig): odom -> camera goes through lidar.
        let mut tree = TfTree::default();
        tree.insert(&transform("odom", "lidar", 0.0, [5.0, 0.0, 0.0]));
        tree.insert(&transform("base", "lidar", 0.0, [1.0, 0.0, 0.0]));
        tree.insert(&transform("base", "camera", 0.0, [0.0, 1.0, 0.0]));
        let odom_from_camera = tree.get("odom", "camera", 0.0).unwrap();
        let camera_in_odom = odom_from_camera * Point3::origin();
        // camera is at base+(0,1,0); base is lidar-(1,0,0); lidar is odom+(5,0,0) => (4,1,0)
        assert!((camera_in_odom.x - 4.0).abs() < 1e-9 && (camera_in_odom.y - 1.0).abs() < 1e-9);
    }

    #[test]
    fn same_timestamp_overwrites() {
        let mut tree = TfTree::default();
        tree.insert(&transform("world", "base", 1.0, [1.0, 0.0, 0.0]));
        tree.insert(&transform("world", "base", 1.0, [5.0, 0.0, 0.0]));
        let pose = tree.get("world", "base", 1.0).unwrap();
        assert!((pose.translation.x - 5.0).abs() < 1e-9);
        assert_eq!(tree.sample_count(), 1);
    }

    #[test]
    fn speeds_from_finite_differences() {
        let mut tree = TfTree::default();
        tree.insert(&transform("world", "base", 0.0, [0.0, 0.0, 0.0]));
        tree.insert(&transform("world", "base", 1.0, [2.0, 0.0, 0.0]));
        let (angular, linear) = tree.speeds("world", "base", 0.5, 0.1).unwrap();
        assert!(angular.abs() < 1e-9);
        assert!((linear - 2.0).abs() < 1e-6);
    }

    #[test]
    fn disconnected_frames_return_none() {
        let mut tree = TfTree::default();
        tree.insert(&transform("world", "base", 0.0, [0.0, 0.0, 0.0]));
        assert!(tree.get("world", "mars", 0.0).is_none());
    }
}
