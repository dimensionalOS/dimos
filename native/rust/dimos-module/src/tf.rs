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

//! Consumer-side transform client for native modules.
//!
//! Each `/tf` edge is buffered per `(parent, child)`, and [`Tf::get`] composes
//! the shortest path through the frame graph. Lookups are nearest-in-time within
//! a tolerance, not interpolated.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::{Arc, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

use crate::module::Route;

/// How many seconds of history each edge keeps.
pub const DEFAULT_TF_BUFFER_SIZE: f64 = 10.0;

fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// A rigid transform from `parent` to `child` at a point in time.
///
/// The isometry maps a point expressed in `child` coordinates into `parent`
/// coordinates: `p_parent = transform.isometry() * p_child`.
#[derive(Clone, Debug)]
pub struct Transform {
    pub parent: String,
    pub child: String,
    pub ts: f64,
    iso: Isometry3<f64>,
}

impl Transform {
    /// The transform as an isometry.
    pub fn isometry(&self) -> Isometry3<f64> {
        self.iso
    }

    /// Translation component (`parent`-frame position of the `child` origin).
    pub fn translation(&self) -> Vector3<f64> {
        self.iso.translation.vector
    }

    /// Rotation component.
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        self.iso.rotation
    }

    fn inverse(&self) -> Transform {
        Transform {
            parent: self.child.clone(),
            child: self.parent.clone(),
            ts: self.ts,
            iso: self.iso.inverse(),
        }
    }

    // self a -> b, then other b -> c, gives a -> c.
    fn compose(&self, other: &Transform) -> Transform {
        Transform {
            parent: self.parent.clone(),
            child: other.child.clone(),
            ts: self.ts,
            iso: self.iso * other.iso,
        }
    }
}

struct Sample {
    ts: f64,
    iso: Isometry3<f64>,
}

// One edge's time-sorted history, capped to a fixed-duration window.
struct TBuffer {
    buffer_size: f64,
    samples: Vec<Sample>,
}

impl TBuffer {
    fn new(buffer_size: f64) -> Self {
        Self {
            buffer_size,
            samples: Vec::new(),
        }
    }

    fn add(&mut self, ts: f64, iso: Isometry3<f64>) {
        let pos = self.samples.partition_point(|s| s.ts <= ts);
        self.samples.insert(pos, Sample { ts, iso });
        self.prune(ts - self.buffer_size);
    }

    fn prune(&mut self, min_ts: f64) {
        let drop_to = self.samples.partition_point(|s| s.ts < min_ts);
        if drop_to > 0 {
            self.samples.drain(0..drop_to);
        }
    }

    fn last(&self) -> Option<&Sample> {
        self.samples.last()
    }

    // Nearest sample in time. On a tie, prefer the later sample. Returns None
    // when the closest sample is further than tolerance from ts.
    fn find_closest(&self, ts: f64, tolerance: Option<f64>) -> Option<&Sample> {
        let pos = self.samples.partition_point(|s| s.ts < ts);
        let prev = pos.checked_sub(1).and_then(|i| self.samples.get(i));
        let next = self.samples.get(pos);
        let best = match (prev, next) {
            (Some(p), Some(n)) => {
                if (n.ts - ts).abs() <= (ts - p.ts).abs() {
                    n
                } else {
                    p
                }
            }
            (Some(p), None) => p,
            (None, Some(n)) => n,
            (None, None) => return None,
        };
        match tolerance {
            Some(tol) if (best.ts - ts).abs() > tol => None,
            _ => Some(best),
        }
    }
}

/// The transform graph: one [`TBuffer`] per `(parent, child)` edge.
struct MultiTBuffer {
    buffer_size: f64,
    buffers: HashMap<(String, String), TBuffer>,
}

impl MultiTBuffer {
    fn new(buffer_size: f64) -> Self {
        Self {
            buffer_size,
            buffers: HashMap::new(),
        }
    }

    fn receive(&mut self, parent: &str, child: &str, ts: f64, iso: Isometry3<f64>) {
        let buffer_size = self.buffer_size;
        self.buffers
            .entry((parent.to_string(), child.to_string()))
            .or_insert_with(|| TBuffer::new(buffer_size))
            .add(ts, iso);
    }

    fn connections(&self, frame: &str) -> Vec<String> {
        let mut out = Vec::new();
        for (parent, child) in self.buffers.keys() {
            if parent == frame {
                out.push(child.clone());
            }
            if child == frame {
                out.push(parent.clone());
            }
        }
        out
    }

    fn sample(
        &self,
        buf: &TBuffer,
        parent: &str,
        child: &str,
        time: Option<f64>,
        tolerance: Option<f64>,
    ) -> Option<Transform> {
        let s = match time {
            None => buf.last()?,
            Some(t) => buf.find_closest(t, tolerance)?,
        };
        Some(Transform {
            parent: parent.to_string(),
            child: child.to_string(),
            ts: s.ts,
            iso: s.iso,
        })
    }

    // A single forward or reverse edge. Reverse returns the inverse.
    fn edge(
        &self,
        parent: &str,
        child: &str,
        time: Option<f64>,
        tolerance: Option<f64>,
    ) -> Option<Transform> {
        if parent == child {
            return Some(Transform {
                parent: parent.to_string(),
                child: child.to_string(),
                ts: time.unwrap_or_else(now_secs),
                iso: Isometry3::identity(),
            });
        }
        if let Some(buf) = self.buffers.get(&(parent.to_string(), child.to_string())) {
            return self.sample(buf, parent, child, time, tolerance);
        }
        if let Some(buf) = self.buffers.get(&(child.to_string(), parent.to_string())) {
            return self
                .sample(buf, child, parent, time, tolerance)
                .map(|t| t.inverse());
        }
        None
    }

    fn get(
        &self,
        parent: &str,
        child: &str,
        time: Option<f64>,
        tolerance: Option<f64>,
    ) -> Option<Transform> {
        if let Some(direct) = self.edge(parent, child, time, tolerance) {
            return Some(direct);
        }
        let path = self.bfs(parent, child, time, tolerance)?;
        let mut steps = path.into_iter();
        let first = steps.next()?;
        Some(steps.fold(first, |acc, step| acc.compose(&step)))
    }

    // Shortest path of edges from parent to child.
    fn bfs(
        &self,
        parent: &str,
        child: &str,
        time: Option<f64>,
        tolerance: Option<f64>,
    ) -> Option<Vec<Transform>> {
        let mut queue: VecDeque<(String, Vec<Transform>)> = VecDeque::new();
        queue.push_back((parent.to_string(), Vec::new()));
        let mut visited: HashSet<String> = HashSet::new();
        visited.insert(parent.to_string());

        while let Some((frame, path)) = queue.pop_front() {
            if frame == child {
                return Some(path);
            }
            for next in self.connections(&frame) {
                if visited.insert(next.clone()) {
                    if let Some(edge) = self.edge(&frame, &next, time, tolerance) {
                        let mut extended = path.clone();
                        extended.push(edge);
                        queue.push_back((next, extended));
                    }
                }
            }
        }
        None
    }
}

/// A cheap-to-clone handle for querying the transform graph.
///
/// Obtain one from `Builder::tf` (or a `#[tf]` field on a `#[derive(Module)]`
/// struct). The graph is filled in the background as `/tf` messages arrive.
#[derive(Clone)]
pub struct Tf {
    buffer: Arc<RwLock<MultiTBuffer>>,
}

impl Tf {
    /// The transform from `parent` to `child`.
    ///
    /// `time` selects the sample nearest that stamp (latest sample when `None`),
    /// and `tolerance` bounds how far that sample may be in seconds. Returns
    /// `None` when no path connects the frames or no sample is within tolerance.
    pub fn get(
        &self,
        parent: &str,
        child: &str,
        time: Option<f64>,
        tolerance: Option<f64>,
    ) -> Option<Transform> {
        self.buffer
            .read()
            .expect("tf buffer lock poisoned")
            .get(parent, child, time, tolerance)
    }

    /// The latest available transform from `parent` to `child`.
    pub fn get_latest(&self, parent: &str, child: &str) -> Option<Transform> {
        self.get(parent, child, None, None)
    }
}

// Decodes /tf messages into the shared graph. Registered as a Route so the
// module's existing recv loop dispatches tf traffic to it.
struct TfRoute {
    topic: String,
    buffer: Arc<RwLock<MultiTBuffer>>,
}

impl Route for TfRoute {
    fn try_dispatch(&self, data: &[u8]) {
        let msg = match lcm_msgs::tf2_msgs::TFMessage::decode(data) {
            Ok(msg) => msg,
            Err(e) => {
                crate::error_throttled!(
                    Duration::from_secs(1),
                    topic = %self.topic,
                    error = %e,
                    "tf decode error"
                );
                return;
            }
        };
        let mut buffer = self.buffer.write().expect("tf buffer lock poisoned");
        for st in &msg.transforms {
            let t = &st.transform.translation;
            let q = &st.transform.rotation;
            let iso = Isometry3::from_parts(
                Translation3::new(t.x, t.y, t.z),
                UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z)),
            );
            let ts = st.header.stamp.sec as f64 + st.header.stamp.nsec as f64 * 1e-9;
            buffer.receive(&st.header.frame_id, &st.child_frame_id, ts, iso);
        }
    }
}

// Builds the shared graph plus the handle and the route that feeds it.
pub(crate) fn tf_subscription(topic: String, buffer_size: f64) -> (Tf, Box<dyn Route>) {
    let buffer = Arc::new(RwLock::new(MultiTBuffer::new(buffer_size)));
    let tf = Tf {
        buffer: Arc::clone(&buffer),
    };
    let route = Box::new(TfRoute { topic, buffer });
    (tf, route)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn tf_with(buffer_size: f64) -> (Tf, MultiHandle) {
        let buffer = Arc::new(RwLock::new(MultiTBuffer::new(buffer_size)));
        (
            Tf {
                buffer: Arc::clone(&buffer),
            },
            MultiHandle { buffer },
        )
    }

    // Test-only writer that bypasses LCM and pushes edges straight into the graph.
    struct MultiHandle {
        buffer: Arc<RwLock<MultiTBuffer>>,
    }

    impl MultiHandle {
        fn add(&self, parent: &str, child: &str, ts: f64, xyz: (f64, f64, f64), yaw: f64) {
            let iso = Isometry3::from_parts(
                Translation3::new(xyz.0, xyz.1, xyz.2),
                UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
            );
            self.buffer.write().unwrap().receive(parent, child, ts, iso);
        }
    }

    #[test]
    fn direct_edge() {
        let (tf, h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        h.add("base_link", "arm", 1.0, (1.0, -1.0, 0.0), 0.0);
        let t = tf.get_latest("base_link", "arm").unwrap();
        assert!((t.translation().x - 1.0).abs() < 1e-9);
        assert!((t.translation().y + 1.0).abs() < 1e-9);
        assert_eq!(t.parent, "base_link");
        assert_eq!(t.child, "arm");
    }

    #[test]
    fn reverse_edge_returns_inverse() {
        let (tf, h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        h.add("base_link", "arm", 1.0, (1.0, 2.0, 3.0), 0.0);
        let inv = tf.get_latest("arm", "base_link").unwrap();
        assert!((inv.translation().x + 1.0).abs() < 1e-9);
        assert!((inv.translation().y + 2.0).abs() < 1e-9);
        assert!((inv.translation().z + 3.0).abs() < 1e-9);
        assert_eq!(inv.parent, "arm");
        assert_eq!(inv.child, "base_link");
    }

    // A 30-degree yaw then a pure translation.
    #[test]
    fn composes_ros_example_chain() {
        let (tf, h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        h.add("base_link", "arm", 1.0, (1.0, -1.0, 0.0), PI / 6.0);
        h.add("arm", "end_effector", 1.0, (1.0, 1.0, 0.0), 0.0);
        let t = tf.get_latest("base_link", "end_effector").unwrap();
        assert!(
            (t.translation().x - 1.366).abs() < 1e-3,
            "{}",
            t.translation().x
        );
        assert!(
            (t.translation().y - 0.366).abs() < 1e-3,
            "{}",
            t.translation().y
        );
        assert_eq!(t.parent, "base_link");
        assert_eq!(t.child, "end_effector");
    }

    // world->robot->sensor multi-hop composition.
    #[test]
    fn composes_multi_hop_chain() {
        let (tf, h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        h.add("world", "robot", 1.0, (1.0, 2.0, 3.0), 0.0);
        h.add("robot", "sensor", 1.0, (0.5, 0.0, 0.2), PI / 2.0);
        let t = tf.get_latest("world", "sensor").unwrap();
        assert!((t.translation().x - 1.5).abs() < 1e-3);
        assert!((t.translation().y - 2.0).abs() < 1e-3);
        assert!((t.translation().z - 3.2).abs() < 1e-3);
    }

    #[test]
    fn missing_path_returns_none() {
        let (tf, h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        h.add("world", "robot", 1.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.get_latest("world", "unconnected").is_none());
    }

    #[test]
    fn identity_for_same_frame() {
        let (tf, _h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        // No query time: identity is stamped now, not the epoch.
        let t = tf.get_latest("base_link", "base_link").unwrap();
        assert!((t.translation().norm()).abs() < 1e-12);
        assert!(
            t.ts > 0.0,
            "identity ts should be a fresh stamp, got {}",
            t.ts
        );
        // Explicit query time is echoed back.
        let at = tf.get("base_link", "base_link", Some(42.0), None).unwrap();
        assert!((at.ts - 42.0).abs() < 1e-9);
    }

    #[test]
    fn time_query_picks_nearest_sample() {
        let (tf, h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        h.add("a", "b", 10.0, (1.0, 0.0, 0.0), 0.0);
        h.add("a", "b", 20.0, (2.0, 0.0, 0.0), 0.0);
        let near_10 = tf.get("a", "b", Some(11.0), None).unwrap();
        assert!((near_10.translation().x - 1.0).abs() < 1e-9);
        let near_20 = tf.get("a", "b", Some(18.0), None).unwrap();
        assert!((near_20.translation().x - 2.0).abs() < 1e-9);
    }

    #[test]
    fn time_query_outside_tolerance_returns_none() {
        let (tf, h) = tf_with(DEFAULT_TF_BUFFER_SIZE);
        h.add("a", "b", 10.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.get("a", "b", Some(50.0), Some(1.0)).is_none());
        assert!(tf.get("a", "b", Some(10.5), Some(1.0)).is_some());
    }

    #[test]
    fn prunes_samples_outside_window() {
        let mut buf = TBuffer::new(5.0);
        buf.add(1.0, Isometry3::identity());
        buf.add(2.0, Isometry3::identity());
        buf.add(10.0, Isometry3::identity());
        // The window is [5.0, 10.0]. The 1.0 and 2.0 samples are dropped.
        assert_eq!(buf.samples.len(), 1);
        assert!((buf.last().unwrap().ts - 10.0).abs() < 1e-9);
    }

    #[test]
    fn tf_route_decodes_into_graph() {
        use lcm_msgs::geometry_msgs::{
            Quaternion as LQuat, Transform as LTransform, Vector3 as LVec3,
        };
        use lcm_msgs::std_msgs::{Header, Time};
        use lcm_msgs::tf2_msgs::TFMessage;

        let (tf, route) = tf_subscription("/tf".to_string(), DEFAULT_TF_BUFFER_SIZE);
        let msg = TFMessage {
            transforms: vec![lcm_msgs::geometry_msgs::TransformStamped {
                header: Header {
                    seq: 0,
                    stamp: Time {
                        sec: 5,
                        nsec: 500_000_000,
                    },
                    frame_id: "base_link".to_string(),
                },
                child_frame_id: "mid360_link".to_string(),
                transform: LTransform {
                    translation: LVec3 {
                        x: 0.1,
                        y: 0.2,
                        z: 0.3,
                    },
                    rotation: LQuat {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
            }],
        };
        route.try_dispatch(&msg.encode());

        let t = tf.get_latest("base_link", "mid360_link").unwrap();
        assert!((t.translation().x - 0.1).abs() < 1e-9);
        assert!((t.translation().y - 0.2).abs() < 1e-9);
        assert!((t.translation().z - 0.3).abs() < 1e-9);
        assert!((t.ts - 5.5).abs() < 1e-9);
    }
}
