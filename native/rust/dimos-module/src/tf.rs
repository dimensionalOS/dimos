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

//! Transform client for native modules.
//!
//! Each `/tf` edge is buffered per `(parent, child)`, and [`Tf::lookup`] composes
//! the shortest path through the frame graph. Lookups are nearest-in-time within
//! a tolerance, not interpolated. [`Tf::publish`] sends transforms onto the same
//! topic and feeds the local graph.

use std::collections::{HashMap, HashSet, VecDeque};
use std::io;
use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use tokio::sync::{mpsc, Notify};
use tracing::warn;

use crate::module::Route;

/// How many seconds of history each edge keeps.
pub(crate) const DEFAULT_TF_WINDOW_SECS: f64 = 10.0;

const WARN_INTERVAL: Duration = Duration::from_secs(1);

fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// A rigid transform from `parent` to `child` at a point in time.
///
/// It maps a point expressed in `child` coordinates into `parent` coordinates.
#[derive(Clone, Debug)]
pub struct Transform {
    pub parent: String,
    pub child: String,
    pub ts: f64,
    iso: Isometry3<f64>,
}

impl Transform {
    pub fn new(
        parent: impl Into<String>,
        child: impl Into<String>,
        ts: f64,
        iso: Isometry3<f64>,
    ) -> Self {
        Self {
            parent: parent.into(),
            child: child.into(),
            ts,
            iso,
        }
    }

    pub fn translation(&self) -> Vector3<f64> {
        self.iso.translation.vector
    }

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
    window_secs: f64,
    samples: VecDeque<Sample>,
}

impl TBuffer {
    fn new(window_secs: f64) -> Self {
        Self {
            window_secs,
            samples: VecDeque::new(),
        }
    }

    fn add(&mut self, ts: f64, iso: Isometry3<f64>) {
        // A stamp a whole window behind the newest is a clock reset, not jitter.
        if self
            .samples
            .back()
            .is_some_and(|s| ts < s.ts - self.window_secs)
        {
            self.samples.clear();
        }
        let pos = self.samples.partition_point(|s| s.ts <= ts);
        self.samples.insert(pos, Sample { ts, iso });
        // Anchored to the newest sample so a late message cannot widen the window.
        let newest = self.samples.back().map_or(ts, |s| s.ts);
        self.prune(newest - self.window_secs);
    }

    fn prune(&mut self, min_ts: f64) {
        let drop_to = self.samples.partition_point(|s| s.ts < min_ts);
        for _ in 0..drop_to {
            self.samples.pop_front();
        }
    }

    fn last(&self) -> Option<&Sample> {
        self.samples.back()
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

    fn sample(
        &self,
        parent: &str,
        child: &str,
        time: Option<f64>,
        tolerance: Option<f64>,
    ) -> Option<Transform> {
        let s = match time {
            None => self.last()?,
            Some(t) => self.find_closest(t, Some(tolerance.unwrap_or(self.window_secs)))?,
        };
        Some(Transform {
            parent: parent.to_string(),
            child: child.to_string(),
            ts: s.ts,
            iso: s.iso,
        })
    }
}

/// The transform graph: one [`TBuffer`] per `(parent, child)` edge.
struct MultiTBuffer {
    window_secs: f64,
    buffers: HashMap<(String, String), TBuffer>,
}

impl MultiTBuffer {
    fn new(window_secs: f64) -> Self {
        Self {
            window_secs,
            buffers: HashMap::new(),
        }
    }

    fn receive(&mut self, parent: &str, child: &str, ts: f64, iso: Isometry3<f64>) {
        let window_secs = self.window_secs;
        self.buffers
            .entry((parent.to_string(), child.to_string()))
            .or_insert_with(|| TBuffer::new(window_secs))
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
            return buf.sample(parent, child, time, tolerance);
        }
        if let Some(buf) = self.buffers.get(&(child.to_string(), parent.to_string())) {
            return buf
                .sample(child, parent, time, tolerance)
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
        // A composition is only as fresh as its stalest edge.
        let oldest = path.iter().map(|t| t.ts).fold(f64::INFINITY, f64::min);
        let mut steps = path.into_iter();
        let first = steps.next()?;
        let mut composed = steps.fold(first, |acc, step| acc.compose(&step));
        composed.ts = oldest;
        Some(composed)
    }

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
                if !visited.contains(&next) {
                    if let Some(edge) = self.edge(&frame, &next, time, tolerance) {
                        visited.insert(next.clone());
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

// The graph plus the signal that it changed. Every write notifies, so a writer
// cannot leave a waiter asleep on a transform that has already landed.
struct Graph {
    buffer: RwLock<MultiTBuffer>,
    changed: Notify,
    // Keyed per pair: warn_throttled! throttles per call site, and one site
    // serves every lookup, so a missing pair would mute all the others.
    warned: Mutex<HashMap<(String, String), AtomicU64>>,
}

impl Graph {
    fn new(window_secs: f64) -> Self {
        Self {
            buffer: RwLock::new(MultiTBuffer::new(window_secs)),
            changed: Notify::new(),
            warned: Mutex::new(HashMap::new()),
        }
    }

    fn should_warn(&self, parent: &str, child: &str) -> bool {
        let mut warned = self.warned.lock().expect("tf warn map lock poisoned");
        let last = warned
            .entry((parent.to_string(), child.to_string()))
            .or_default();
        crate::log::check_and_record(last, WARN_INTERVAL.as_nanos() as u64)
    }

    fn update(&self, edits: impl FnOnce(&mut MultiTBuffer)) {
        edits(&mut self.buffer.write().expect("tf buffer lock poisoned"));
        self.changed.notify_waiters();
    }

    fn get(
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
}

/// A cheap-to-clone handle for querying and publishing transforms.
///
/// Obtain one from `Builder::tf` (or a `#[tf]` field on a `#[derive(Module)]`
/// struct). The graph is filled in the background as `/tf` messages arrive.
#[derive(Clone)]
pub struct Tf {
    graph: Arc<Graph>,
    sender: mpsc::Sender<Vec<u8>>,
}

impl Tf {
    /// Start a lookup of the transform from `parent` to `child`.
    ///
    /// Refine it with [`Lookup::at`] and [`Lookup::tolerance`], then finish with
    /// [`Lookup::get`]. Use [`Tf::get_latest`] when no refinement is needed.
    ///
    /// ```ignore
    /// let at_scan = tf.lookup("map", "base_link").at(scan_ts).tolerance(0.1).get();
    /// ```
    pub fn lookup<'a>(&'a self, parent: &'a str, child: &'a str) -> Lookup<'a> {
        Lookup {
            tf: self,
            parent,
            child,
            time: None,
            tolerance: None,
        }
    }

    /// The latest transform from `parent` to `child`.
    ///
    /// Shorthand for `lookup(parent, child).get()`.
    pub fn get_latest(&self, parent: &str, child: &str) -> Option<Transform> {
        self.lookup(parent, child).get()
    }

    /// Publish transforms on the `tf` topic.
    ///
    /// The transforms also feed the local graph, so a lookup right after sees
    /// them without waiting for the transport round trip.
    pub async fn publish(&self, transforms: &[Transform]) -> io::Result<()> {
        self.graph.update(|buffer| {
            for t in transforms {
                buffer.receive(&t.parent, &t.child, t.ts, t.iso);
            }
        });
        let msg = lcm_msgs::tf2_msgs::TFMessage {
            transforms: transforms.iter().map(to_stamped).collect(),
        };
        crate::module::publish_encoded(&self.sender, msg.encode()).await
    }
}

/// A transform lookup being built. Created by [`Tf::lookup`].
pub struct Lookup<'a> {
    tf: &'a Tf,
    parent: &'a str,
    child: &'a str,
    time: Option<f64>,
    tolerance: Option<f64>,
}

impl Lookup<'_> {
    /// Take the sample nearest `time` rather than the latest one.
    pub fn at(mut self, time: f64) -> Self {
        self.time = Some(time);
        self
    }

    /// Bound how far, in seconds, the chosen sample may sit from [`Lookup::at`].
    pub fn tolerance(mut self, tolerance: f64) -> Self {
        self.tolerance = Some(tolerance);
        self
    }

    fn resolve(&self) -> Option<Transform> {
        self.tf
            .graph
            .get(self.parent, self.child, self.time, self.tolerance)
    }

    fn no_sample_can_still_arrive(&self) -> bool {
        let (Some(time), Some(tolerance)) = (self.time, self.tolerance) else {
            return false;
        };
        self.tf
            .graph
            .get(self.parent, self.child, None, None)
            .is_some_and(|latest| latest.ts > time + tolerance)
    }

    // A lookup that resolves to nothing is otherwise invisible: the caller sees
    // None and the buffer says nothing about which frames or stamp missed.
    fn warn_unresolved(&self) {
        if !self.tf.graph.should_warn(self.parent, self.child) {
            return;
        }
        warn!(
            parent = %self.parent,
            child = %self.child,
            at = self.time.unwrap_or_else(now_secs),
            tolerance = self.tolerance.unwrap_or(f64::NAN),
            "No transform found between frames",
        );
    }

    /// Resolve the lookup against the transforms buffered so far.
    ///
    /// `None` when no path connects the frames, or when the nearest sample is
    /// outside the tolerance.
    pub fn get(self) -> Option<Transform> {
        let found = self.resolve();
        if found.is_none() {
            self.warn_unresolved();
        }
        found
    }

    /// Resolve the lookup, waiting up to `timeout` for a late transform.
    ///
    /// Returns as soon as the lookup succeeds, or `None` at the deadline.
    /// Awaiting this inside a `handle_*` method parks the module's whole
    /// dispatch loop, so prefer [`Lookup::get`] there and move a long wait onto
    /// a task of its own.
    pub async fn within(self, timeout: Duration) -> Option<Transform> {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            // Registered before the resolve below, so a transform landing between
            // the two still wakes this waiter instead of it sleeping out the
            // whole timeout.
            let changed = self.tf.graph.changed.notified();
            tokio::pin!(changed);
            changed.as_mut().enable();

            if let Some(transform) = self.resolve() {
                return Some(transform);
            }
            // Edges only append forward in time, so once the stalest edge on the
            // path has passed the requested stamp no sample can still arrive.
            if self.no_sample_can_still_arrive() {
                self.warn_unresolved();
                return None;
            }
            // Only the deadline warns. An intermediate miss is the normal state
            // of a wait, not a failure.
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                self.warn_unresolved();
                return None;
            }
            if tokio::time::timeout(remaining, changed).await.is_err() {
                self.warn_unresolved();
                return None;
            }
        }
    }
}

fn to_stamped(t: &Transform) -> lcm_msgs::geometry_msgs::TransformStamped {
    let mut sec = t.ts.floor();
    let mut nsec = ((t.ts - sec) * 1e9).round();
    if nsec >= 1e9 {
        sec += 1.0;
        nsec -= 1e9;
    }
    let p = t.iso.translation.vector;
    let q = t.iso.rotation;
    lcm_msgs::geometry_msgs::TransformStamped {
        header: lcm_msgs::std_msgs::Header {
            seq: 0,
            stamp: lcm_msgs::std_msgs::Time {
                sec: sec as i32,
                nsec: nsec as i32,
            },
            frame_id: t.parent.clone(),
        },
        child_frame_id: t.child.clone(),
        transform: lcm_msgs::geometry_msgs::Transform {
            translation: lcm_msgs::geometry_msgs::Vector3 {
                x: p.x,
                y: p.y,
                z: p.z,
            },
            rotation: lcm_msgs::geometry_msgs::Quaternion {
                x: q.i,
                y: q.j,
                z: q.k,
                w: q.w,
            },
        },
    }
}

// Decodes /tf messages into the shared graph. Registered as a Route so the
// module's existing recv loop dispatches tf traffic to it.
struct TfRoute {
    topic: String,
    graph: Arc<Graph>,
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
        self.graph.update(|buffer| {
            for st in &msg.transforms {
                let t = &st.transform.translation;
                let q = &st.transform.rotation;
                // Normalizing a zero-norm quaternion yields a NaN rotation.
                let Some(rotation) =
                    UnitQuaternion::try_new(Quaternion::new(q.w, q.x, q.y, q.z), 1e-9)
                else {
                    crate::error_throttled!(
                        Duration::from_secs(1),
                        topic = %self.topic,
                        parent = %st.header.frame_id,
                        child = %st.child_frame_id,
                        "tf rotation is not a valid quaternion"
                    );
                    continue;
                };
                let iso = Isometry3::from_parts(Translation3::new(t.x, t.y, t.z), rotation);
                let ts = st.header.stamp.sec as f64 + st.header.stamp.nsec as f64 * 1e-9;
                buffer.receive(&st.header.frame_id, &st.child_frame_id, ts, iso);
            }
        });
    }
}

// Builds the shared graph plus the handle and the route that feeds it. The
// sender carries published messages to the tf topic's publish worker.
pub(crate) fn tf_subscription(
    topic: String,
    window_secs: f64,
    sender: mpsc::Sender<Vec<u8>>,
) -> (Tf, Box<dyn Route>) {
    let graph = Arc::new(Graph::new(window_secs));
    let tf = Tf {
        graph: Arc::clone(&graph),
        sender,
    };
    let route = Box::new(TfRoute { topic, graph });
    (tf, route)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn tf_with(window_secs: f64) -> (Tf, MultiHandle) {
        let (tf, _rx, handle) = tf_with_publish(window_secs);
        (tf, handle)
    }

    fn tf_with_publish(window_secs: f64) -> (Tf, mpsc::Receiver<Vec<u8>>, MultiHandle) {
        let graph = Arc::new(Graph::new(window_secs));
        let (tx, rx) = mpsc::channel(8);
        (
            Tf {
                graph: Arc::clone(&graph),
                sender: tx,
            },
            rx,
            MultiHandle { graph },
        )
    }

    // Test-only writer that bypasses LCM and pushes edges straight into the graph.
    struct MultiHandle {
        graph: Arc<Graph>,
    }

    impl MultiHandle {
        fn add(&self, parent: &str, child: &str, ts: f64, xyz: (f64, f64, f64), yaw: f64) {
            let iso = Isometry3::from_parts(
                Translation3::new(xyz.0, xyz.1, xyz.2),
                UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
            );
            self.graph
                .update(|buffer| buffer.receive(parent, child, ts, iso));
        }
    }

    #[test]
    fn direct_edge() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("base_link", "arm", 1.0, (1.0, -1.0, 0.0), 0.0);
        let t = tf.get_latest("base_link", "arm").unwrap();
        assert!((t.translation().x - 1.0).abs() < 1e-9);
        assert!((t.translation().y + 1.0).abs() < 1e-9);
        assert_eq!(t.parent, "base_link");
        assert_eq!(t.child, "arm");
    }

    #[test]
    fn reverse_edge_returns_inverse() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("base_link", "arm", 1.0, (1.0, 2.0, 3.0), 0.0);
        let inv = tf.get_latest("arm", "base_link").unwrap();
        assert!((inv.translation().x + 1.0).abs() < 1e-9);
        assert!((inv.translation().y + 2.0).abs() < 1e-9);
        assert!((inv.translation().z + 3.0).abs() < 1e-9);
        assert_eq!(inv.parent, "arm");
        assert_eq!(inv.child, "base_link");
    }

    #[test]
    fn composes_ros_example_chain() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
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

    #[test]
    fn composes_multi_hop_chain() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("world", "robot", 1.0, (1.0, 2.0, 3.0), 0.0);
        h.add("robot", "sensor", 1.0, (0.5, 0.0, 0.2), PI / 2.0);
        let t = tf.get_latest("world", "sensor").unwrap();
        assert!((t.translation().x - 1.5).abs() < 1e-3);
        assert!((t.translation().y - 2.0).abs() < 1e-3);
        assert!((t.translation().z - 3.2).abs() < 1e-3);
    }

    #[test]
    fn composed_stamp_is_the_stalest_edge() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("world", "robot", 700.0, (1.0, 0.0, 0.0), 0.0);
        h.add("robot", "sensor", 1000.0, (0.5, 0.0, 0.0), 0.0);
        assert_eq!(tf.get_latest("world", "sensor").unwrap().ts, 700.0);
        // Both directions, so the answer does not depend on which end is queried.
        assert_eq!(tf.get_latest("sensor", "world").unwrap().ts, 700.0);
    }

    #[test]
    fn missing_path_returns_none() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("world", "robot", 1.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.get_latest("world", "unconnected").is_none());
    }

    #[test]
    fn identity_for_same_frame() {
        let (tf, _h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        // No query time: identity is stamped now, not the epoch.
        let t = tf.get_latest("base_link", "base_link").unwrap();
        assert!((t.translation().norm()).abs() < 1e-12);
        assert!(
            t.ts > 0.0,
            "identity ts should be a fresh stamp, got {}",
            t.ts
        );
        // Explicit query time is echoed back.
        let at = tf.lookup("base_link", "base_link").at(42.0).get().unwrap();
        assert!((at.ts - 42.0).abs() < 1e-9);
    }

    #[test]
    fn time_query_picks_nearest_sample() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("a", "b", 10.0, (1.0, 0.0, 0.0), 0.0);
        h.add("a", "b", 20.0, (2.0, 0.0, 0.0), 0.0);
        let near_10 = tf.lookup("a", "b").at(11.0).get().unwrap();
        assert!((near_10.translation().x - 1.0).abs() < 1e-9);
        let near_20 = tf.lookup("a", "b").at(18.0).get().unwrap();
        assert!((near_20.translation().x - 2.0).abs() < 1e-9);
    }

    #[test]
    fn time_query_outside_tolerance_returns_none() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("a", "b", 10.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.lookup("a", "b").at(50.0).tolerance(1.0).get().is_none());
        assert!(tf.lookup("a", "b").at(10.5).tolerance(1.0).get().is_some());
    }

    #[test]
    fn time_query_beyond_the_window_returns_none_without_a_tolerance() {
        let (tf, h) = tf_with(10.0);
        h.add("a", "b", 100.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.lookup("a", "b").at(50.0).get().is_none());
    }

    #[test]
    fn time_query_inside_the_window_resolves_without_a_tolerance() {
        let (tf, h) = tf_with(10.0);
        h.add("a", "b", 100.0, (1.0, 0.0, 0.0), 0.0);
        let t = tf
            .lookup("a", "b")
            .at(95.0)
            .get()
            .expect("within the window");
        assert!((t.translation().x - 1.0).abs() < 1e-9);
    }

    // An explicit tolerance is the caller opting into staleness, so it widens
    // past the window rather than being clamped by it.
    #[test]
    fn an_explicit_tolerance_reaches_past_the_window() {
        let (tf, h) = tf_with(10.0);
        h.add("a", "b", 100.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.lookup("a", "b").at(50.0).tolerance(60.0).get().is_some());
    }

    // The window bounds queries against a stamp, not the latest sample. With no
    // requested time, the newest edge is returned however old it is.
    #[test]
    fn latest_is_not_bounded_by_the_window() {
        let (tf, h) = tf_with(10.0);
        h.add("a", "b", 100.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.get_latest("a", "b").is_some());
    }

    #[tokio::test]
    async fn within_returns_without_waiting_when_already_buffered() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("a", "b", 5.0, (1.0, 0.0, 0.0), 0.0);
        let t = tf
            .lookup("a", "b")
            .at(5.0)
            .tolerance(0.1)
            .within(Duration::from_secs(30))
            .await
            .expect("already available");
        assert!((t.translation().x - 1.0).abs() < 1e-9);
    }

    // Returns as soon as the transform lands, not at the deadline.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn within_resolves_when_the_transform_arrives_late() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(30)).await;
            h.add("a", "b", 5.0, (1.0, 0.0, 0.0), 0.0);
        });
        let started = tokio::time::Instant::now();
        let t = tf
            .lookup("a", "b")
            .at(5.0)
            .tolerance(0.1)
            .within(Duration::from_secs(30))
            .await
            .expect("arrived inside the budget");
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "waited {:?}, should have returned on arrival",
            started.elapsed()
        );
        assert!((t.translation().x - 1.0).abs() < 1e-9);
    }

    #[tokio::test]
    async fn within_times_out_when_nothing_arrives() {
        let (tf, _h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        let started = tokio::time::Instant::now();
        let t = tf
            .lookup("a", "b")
            .at(5.0)
            .tolerance(0.1)
            .within(Duration::from_millis(50))
            .await;
        assert!(t.is_none());
        assert!(
            started.elapsed() >= Duration::from_millis(50),
            "returned early"
        );
    }

    // The waiter is on a -> c, but what lands is b -> c. Waking only on the
    // queried edge would sleep through the composition becoming possible.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn within_wakes_when_a_later_edge_completes_the_chain() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("a", "b", 5.0, (1.0, 0.0, 0.0), 0.0);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(30)).await;
            h.add("b", "c", 5.0, (2.0, 0.0, 0.0), 0.0);
        });
        let t = tf
            .lookup("a", "c")
            .at(5.0)
            .tolerance(0.1)
            .within(Duration::from_secs(30))
            .await
            .expect("chain completed inside the budget");
        assert!(
            (t.translation().x - 3.0).abs() < 1e-9,
            "{}",
            t.translation().x
        );
    }

    // A waiting handler is woken by the transport's dispatch task, which its own
    // stall cannot block. Guards against the wait deadlocking against tf intake.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn within_is_woken_by_a_transform_dispatched_through_the_route() {
        let (tx, _rx) = mpsc::channel(8);
        let (tf, route) = tf_subscription("/tf".to_string(), DEFAULT_TF_WINDOW_SECS, tx);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(30)).await;
            route.try_dispatch(&stamped_message("a", "b", 5.0, 1.0));
        });
        let t = tf
            .lookup("a", "b")
            .at(5.0)
            .tolerance(0.1)
            .within(Duration::from_secs(30))
            .await
            .expect("route dispatch woke the waiter");
        assert!((t.translation().x - 1.0).abs() < 1e-9);
    }

    fn stamped_message(parent: &str, child: &str, ts: f64, x: f64) -> Vec<u8> {
        rotated_message(parent, child, ts, x, (0.0, 0.0, 0.0, 1.0))
    }

    fn rotated_message(
        parent: &str,
        child: &str,
        ts: f64,
        x: f64,
        quat: (f64, f64, f64, f64),
    ) -> Vec<u8> {
        use lcm_msgs::geometry_msgs::{
            Quaternion as LQuat, Transform as LTransform, TransformStamped, Vector3 as LVec3,
        };
        use lcm_msgs::std_msgs::{Header, Time};
        let (x_q, y_q, z_q, w_q) = quat;
        lcm_msgs::tf2_msgs::TFMessage {
            transforms: vec![TransformStamped {
                header: Header {
                    seq: 0,
                    stamp: Time {
                        sec: ts as i32,
                        nsec: 0,
                    },
                    frame_id: parent.to_string(),
                },
                child_frame_id: child.to_string(),
                transform: LTransform {
                    translation: LVec3 { x, y: 0.0, z: 0.0 },
                    rotation: LQuat {
                        x: x_q,
                        y: y_q,
                        z: z_q,
                        w: w_q,
                    },
                },
            }],
        }
        .encode()
    }

    #[test]
    fn a_zero_rotation_on_the_wire_is_dropped_rather_than_stored_as_nan() {
        let (tx, _rx) = mpsc::channel(4);
        let (tf, route) = tf_subscription("/tf".to_string(), DEFAULT_TF_WINDOW_SECS, tx);
        route.try_dispatch(&rotated_message("a", "b", 5.0, 1.0, (0.0, 0.0, 0.0, 0.0)));
        assert!(tf.get_latest("a", "b").is_none());

        route.try_dispatch(&stamped_message("a", "b", 6.0, 1.0));
        let t = tf.get_latest("a", "b").expect("valid rotation is accepted");
        assert!(t.rotation().coords.iter().all(|c| c.is_finite()));
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_lookup_that_finds_nothing_warns_with_the_frames() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("world", "robot", 1.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.get_latest("world", "gripper").is_none());
        assert!(logs_contain("No transform found between frames"));
        assert!(logs_contain("gripper"));
    }

    #[test]
    #[tracing_test::traced_test]
    fn repeated_misses_warn_once_per_frame_pair() {
        let (tf, _h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        for _ in 0..5 {
            assert!(tf.get_latest("world", "gripper").is_none());
        }
        logs_assert(|lines: &[&str]| {
            match lines.iter().filter(|l| l.contains("gripper")).count() {
                1 => Ok(()),
                n => Err(format!("expected 1 warning for the repeated pair, got {n}")),
            }
        });

        // A pair that is throttled must not mute an unrelated one.
        assert!(tf.get_latest("world", "camera").is_none());
        assert!(logs_contain("camera"));
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_resolved_lookup_stays_quiet() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("world", "robot", 1.0, (1.0, 0.0, 0.0), 0.0);
        assert!(tf.get_latest("world", "robot").is_some());
        assert!(!logs_contain("No transform found between frames"));
    }

    // A wait in progress is not a failure, so only the deadline warns.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    #[tracing_test::traced_test]
    async fn within_warns_only_once_it_gives_up() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(20)).await;
            h.add("a", "b", 5.0, (1.0, 0.0, 0.0), 0.0);
        });
        assert!(tf
            .lookup("a", "b")
            .at(5.0)
            .tolerance(0.1)
            .within(Duration::from_secs(30))
            .await
            .is_some());
        assert!(!logs_contain("No transform found between frames"));

        assert!(tf
            .lookup("a", "missing")
            .at(5.0)
            .tolerance(0.1)
            .within(Duration::from_millis(20))
            .await
            .is_none());
        assert!(logs_contain("No transform found between frames"));
    }

    #[test]
    fn prunes_samples_outside_window() {
        let mut buf = TBuffer::new(5.0);
        buf.add(1.0, Isometry3::identity());
        buf.add(2.0, Isometry3::identity());
        buf.add(10.0, Isometry3::identity());
        assert_eq!(buf.samples.len(), 1);
        assert!((buf.last().unwrap().ts - 10.0).abs() < 1e-9);
    }

    #[test]
    fn a_late_sample_does_not_spare_ones_the_window_has_aged_out() {
        let mut buf = TBuffer::new(5.0);
        buf.add(10.0, Isometry3::identity());
        buf.add(11.0, Isometry3::identity());
        // Late, but still inside the window.
        buf.add(7.0, Isometry3::identity());
        assert_eq!(buf.samples.len(), 3);

        buf.add(20.0, Isometry3::identity());
        assert_eq!(buf.samples.len(), 1);
        assert!((buf.last().unwrap().ts - 20.0).abs() < 1e-9);
    }

    #[test]
    fn a_clock_reset_drops_the_pre_jump_samples() {
        let mut buf = TBuffer::new(5.0);
        for i in 0..20 {
            buf.add(1000.0 + i as f64, Isometry3::identity());
        }
        for i in 0..20 {
            buf.add(100.0 + i as f64, Isometry3::identity());
        }
        assert!(
            buf.samples.len() <= 6,
            "buffer grew to {}",
            buf.samples.len()
        );
        assert!((buf.last().unwrap().ts - 119.0).abs() < 1e-9);
    }

    #[test]
    fn tf_route_decodes_into_graph() {
        use lcm_msgs::geometry_msgs::{
            Quaternion as LQuat, Transform as LTransform, Vector3 as LVec3,
        };
        use lcm_msgs::std_msgs::{Header, Time};
        use lcm_msgs::tf2_msgs::TFMessage;

        let (tx, _rx) = mpsc::channel(8);
        let (tf, route) = tf_subscription("/tf".to_string(), DEFAULT_TF_WINDOW_SECS, tx);
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

    #[tokio::test]
    async fn publish_feeds_local_graph() {
        let (tf, _rx, _h) = tf_with_publish(DEFAULT_TF_WINDOW_SECS);
        let iso = Isometry3::from_parts(
            Translation3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 2.0),
        );
        tf.publish(&[Transform::new("map", "base_link", 7.0, iso)])
            .await
            .unwrap();

        let t = tf.get_latest("map", "base_link").unwrap();
        assert!((t.translation().x - 1.0).abs() < 1e-9);
        assert!((t.translation().y - 2.0).abs() < 1e-9);
        assert!((t.translation().z - 3.0).abs() < 1e-9);
        assert!((t.ts - 7.0).abs() < 1e-9);
    }

    #[tokio::test]
    async fn publish_round_trips_through_route() {
        let (tf_out, mut rx, _h) = tf_with_publish(DEFAULT_TF_WINDOW_SECS);
        let iso = Isometry3::from_parts(
            Translation3::new(0.5, -0.5, 0.25),
            UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 6.0),
        );
        tf_out
            .publish(&[Transform::new("a", "b", 3.25, iso)])
            .await
            .unwrap();
        let bytes = rx.recv().await.unwrap();

        let (tx, _rx2) = mpsc::channel(8);
        let (tf_in, route) = tf_subscription("/tf".to_string(), DEFAULT_TF_WINDOW_SECS, tx);
        route.try_dispatch(&bytes);

        let t = tf_in.get_latest("a", "b").unwrap();
        assert!((t.translation().x - 0.5).abs() < 1e-9);
        assert!((t.translation().y + 0.5).abs() < 1e-9);
        assert!((t.translation().z - 0.25).abs() < 1e-9);
        assert!((t.ts - 3.25).abs() < 1e-9);
        let (_, _, yaw) = t.rotation().euler_angles();
        assert!((yaw - PI / 6.0).abs() < 1e-9);
    }

    #[test]
    fn stamp_rounding_does_not_overflow_nsec() {
        let st = to_stamped(&Transform::new(
            "a",
            "b",
            1.9999999999,
            Isometry3::identity(),
        ));
        assert_eq!(st.header.stamp.sec, 2);
        assert_eq!(st.header.stamp.nsec, 0);
    }

    #[test]
    fn add_out_of_order_keeps_samples_sorted() {
        let mut buf = TBuffer::new(DEFAULT_TF_WINDOW_SECS);
        buf.add(3.0, Isometry3::identity());
        buf.add(1.0, Isometry3::identity());
        buf.add(2.0, Isometry3::identity());
        assert!((buf.last().unwrap().ts - 3.0).abs() < 1e-9);
        let s = buf.find_closest(1.9, None).unwrap();
        assert!((s.ts - 2.0).abs() < 1e-9);
    }

    #[test]
    fn tie_prefers_the_later_sample() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("a", "b", 10.0, (1.0, 0.0, 0.0), 0.0);
        h.add("a", "b", 12.0, (2.0, 0.0, 0.0), 0.0);
        let t = tf.lookup("a", "b").at(11.0).get().unwrap();
        assert!((t.translation().x - 2.0).abs() < 1e-9);
    }

    // Two routes to d: three hops through b, c and two through x. BFS must
    // compose the two-hop route.
    #[test]
    fn bfs_takes_the_fewest_hops_on_a_branching_graph() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("a", "b", 1.0, (1.0, 0.0, 0.0), 0.0);
        h.add("b", "c", 1.0, (1.0, 0.0, 0.0), 0.0);
        h.add("c", "d", 1.0, (1.0, 0.0, 0.0), 0.0);
        h.add("a", "x", 1.0, (10.0, 0.0, 0.0), 0.0);
        h.add("x", "d", 1.0, (1.0, 0.0, 0.0), 0.0);
        let t = tf.get_latest("a", "d").unwrap();
        assert!(
            (t.translation().x - 11.0).abs() < 1e-9,
            "{}",
            t.translation().x
        );
    }

    // Inverse of a rotated edge is t' = -R^T t, the classic sign/order trap.
    #[test]
    fn reverse_edge_inverts_rotation_and_translation() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("base_link", "arm", 1.0, (1.0, 2.0, 3.0), PI / 2.0);
        let inv = tf.get_latest("arm", "base_link").unwrap();
        assert!(
            (inv.translation().x + 2.0).abs() < 1e-9,
            "{:?}",
            inv.translation()
        );
        assert!((inv.translation().y - 1.0).abs() < 1e-9);
        assert!((inv.translation().z + 3.0).abs() < 1e-9);
        let (_, _, yaw) = inv.rotation().euler_angles();
        assert!((yaw + PI / 2.0).abs() < 1e-9);
    }

    #[test]
    fn composed_chain_accumulates_rotation() {
        let (tf, h) = tf_with(DEFAULT_TF_WINDOW_SECS);
        h.add("a", "b", 1.0, (0.0, 0.0, 0.0), PI / 6.0);
        h.add("b", "c", 1.0, (0.0, 0.0, 0.0), PI / 6.0);
        let t = tf.get_latest("a", "c").unwrap();
        let (_, _, yaw) = t.rotation().euler_angles();
        assert!((yaw - PI / 3.0).abs() < 1e-9);
    }

    #[test]
    fn dispatch_of_undecodable_bytes_leaves_the_graph_empty() {
        let (tx, _rx) = mpsc::channel(8);
        let (tf, route) = tf_subscription("/tf".to_string(), DEFAULT_TF_WINDOW_SECS, tx);
        route.try_dispatch(b"garbage");
        assert!(tf.get_latest("a", "b").is_none());
    }

    #[tokio::test]
    async fn publish_errors_when_the_background_task_is_gone() {
        let (tf, rx, _h) = tf_with_publish(DEFAULT_TF_WINDOW_SECS);
        drop(rx);
        let err = tf
            .publish(&[Transform::new("a", "b", 1.0, Isometry3::identity())])
            .await
            .unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::BrokenPipe);
    }
}
