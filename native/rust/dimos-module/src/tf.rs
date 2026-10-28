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

//! ROS TF messages backed by the `transforms` frame tree.

use crate::module::Route;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::{
    io,
    sync::{Arc, RwLock},
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tokio::sync::{mpsc, Notify};
use transforms::{
    geometry::{
        Quaternion as FrameQuaternion, Transform as FrameTransform, Vector3 as FrameVector,
    },
    time::Stamp,
    Registry,
};

pub use dimos_generated_messages::{
    builtin_interfaces::msg::time::Time, geometry_msgs::msg::transform_stamped::TransformStamped,
    tf2_msgs::msg::tf_message::TFMessage,
};
use dimos_generated_messages::{
    geometry_msgs::msg::{
        quaternion::Quaternion as WireQuaternion, transform::Transform, vector3::Vector3,
    },
    std_msgs::msg::header::Header,
};

pub(crate) const DEFAULT_TF_WINDOW: Duration = Duration::from_secs(10);

fn invalid(error: impl std::fmt::Display) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, error.to_string())
}

pub fn nanoseconds(time: &Time) -> io::Result<i64> {
    if time.nanosec >= 1_000_000_000 {
        return Err(invalid("unnormalized ROS time"));
    }
    Ok(i64::from(time.sec) * 1_000_000_000 + i64::from(time.nanosec))
}

pub fn time_from_nanoseconds(ns: i64) -> io::Result<Time> {
    Ok(Time {
        sec: ns.div_euclid(1_000_000_000).try_into().map_err(invalid)?,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    })
}

fn system_time(time: &Time) -> io::Result<SystemTime> {
    let ns = nanoseconds(time)?;
    let duration = Duration::from_nanos(ns.unsigned_abs());
    (if ns >= 0 {
        UNIX_EPOCH.checked_add(duration)
    } else {
        UNIX_EPOCH.checked_sub(duration)
    })
    .ok_or_else(|| invalid("time out of range"))
}

fn ros_time(time: SystemTime) -> io::Result<Time> {
    let ns = match time.duration_since(UNIX_EPOCH) {
        Ok(d) => i64::try_from(d.as_nanos()).map_err(invalid)?,
        Err(e) => -i64::try_from(e.duration().as_nanos()).map_err(invalid)?,
    };
    time_from_nanoseconds(ns)
}

/// Adapt numeric geometry to a standard message; publication validates its values.
pub fn from_isometry(
    parent: impl Into<String>,
    child: impl Into<String>,
    stamp: Time,
    iso: Isometry3<f64>,
) -> TransformStamped {
    let p = iso.translation.vector;
    let q = iso.rotation;
    TransformStamped {
        header: Header {
            stamp,
            frame_id: parent.into(),
        },
        child_frame_id: child.into(),
        transform: Transform {
            translation: Vector3 {
                x: p.x,
                y: p.y,
                z: p.z,
            },
            rotation: WireQuaternion {
                x: q.i,
                y: q.j,
                z: q.k,
                w: q.w,
            },
        },
    }
}

pub fn isometry(edge: &TransformStamped) -> io::Result<Isometry3<f64>> {
    let p = &edge.transform.translation;
    let q = &edge.transform.rotation;
    if ![p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        .iter()
        .all(|v| v.is_finite())
    {
        return Err(invalid("nonfinite TF geometry"));
    }
    let norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm - 1.).abs() > 1e-6 {
        return Err(invalid("TF quaternion must be normalized"));
    }
    Ok(Isometry3::from_parts(
        Translation3::new(p.x, p.y, p.z),
        UnitQuaternion::new_normalize(Quaternion::new(q.w, q.x, q.y, q.z)),
    ))
}

fn to_frame(edge: &TransformStamped, is_static: bool) -> io::Result<FrameTransform<SystemTime>> {
    for frame in [&edge.header.frame_id, &edge.child_frame_id] {
        if frame.is_empty() || frame.starts_with('/') {
            return Err(invalid(
                "TF frame must be nonempty and have no leading slash",
            ));
        }
    }
    let time = system_time(&edge.header.stamp)?;
    isometry(edge)?;
    let p = &edge.transform.translation;
    let q = &edge.transform.rotation;
    FrameTransform::new(
        &edge.header.frame_id,
        &edge.child_frame_id,
        FrameVector::new(p.x, p.y, p.z),
        FrameQuaternion::from_wxyz(q.w, q.x, q.y, q.z),
        if is_static {
            Stamp::Static
        } else {
            Stamp::At(time)
        },
    )
    .map_err(invalid)
}

fn from_frame(edge: FrameTransform<SystemTime>) -> io::Result<TransformStamped> {
    edge.validate().map_err(invalid)?;
    let p = edge.translation();
    let q = edge.rotation();
    Ok(TransformStamped {
        header: Header {
            stamp: ros_time(edge.timestamp().at().unwrap_or(UNIX_EPOCH))?,
            frame_id: edge.parent().into(),
        },
        child_frame_id: edge.child().into(),
        transform: Transform {
            translation: Vector3 {
                x: p.x,
                y: p.y,
                z: p.z,
            },
            rotation: WireQuaternion {
                x: q.x,
                y: q.y,
                z: q.z,
                w: q.w,
            },
        },
    })
}

struct Graph {
    buffer: RwLock<Registry<SystemTime>>,
    changed: Notify,
}

impl Graph {
    fn insert(&self, message: &TFMessage, is_static: bool) -> io::Result<()> {
        let edges = message
            .transforms
            .iter()
            .map(|edge| to_frame(edge, is_static))
            .collect::<io::Result<Vec<_>>>()?;
        self.buffer
            .write()
            .map_err(invalid)?
            .add_transforms(edges)
            .map_err(invalid)?;
        self.changed.notify_waiters();
        Ok(())
    }

    fn lookup(
        &self,
        target: &str,
        source: &str,
        time: Option<&Time>,
    ) -> io::Result<TransformStamped> {
        if [target, source]
            .iter()
            .any(|frame| frame.is_empty() || frame.starts_with('/'))
        {
            return Err(invalid(
                "TF frame must be nonempty and have no leading slash",
            ));
        }
        let buffer = self.buffer.read().map_err(invalid)?;
        let time = match time {
            Some(t) => system_time(t)?,
            None => buffer
                .latest_common_time(target, source)
                .map_err(invalid)?
                .at()
                .unwrap_or(UNIX_EPOCH),
        };
        from_frame(
            buffer
                .get_transform(target, source, time)
                .map_err(invalid)?,
        )
    }
}

#[derive(Clone)]
pub struct Tf {
    graph: Arc<Graph>,
    sender: mpsc::Sender<Vec<u8>>,
    static_sender: mpsc::Sender<Vec<u8>>,
}

impl Tf {
    /// ROS lookup order: target, source, time. Zero time requests the latest common time.
    pub fn lookup_transform(
        &self,
        target: &str,
        source: &str,
        time: Time,
    ) -> io::Result<TransformStamped> {
        nanoseconds(&time)?;
        self.graph.lookup(
            target,
            source,
            if time.sec == 0 && time.nanosec == 0 {
                None
            } else {
                Some(&time)
            },
        )
    }

    pub fn lookup<'a>(&'a self, target: &'a str, source: &'a str) -> Lookup<'a> {
        Lookup {
            tf: self,
            target,
            source,
            time: None,
        }
    }
    pub fn get_latest(&self, target: &str, source: &str) -> Option<TransformStamped> {
        self.graph.lookup(target, source, None).ok()
    }

    pub async fn publish(&self, transforms: &[TransformStamped]) -> io::Result<()> {
        self.publish_message(
            &TFMessage {
                transforms: transforms.to_vec(),
            },
            false,
        )
        .await
    }
    pub async fn publish_static(&self, transforms: &[TransformStamped]) -> io::Result<()> {
        self.publish_message(
            &TFMessage {
                transforms: transforms.to_vec(),
            },
            true,
        )
        .await
    }

    async fn publish_message(&self, message: &TFMessage, is_static: bool) -> io::Result<()> {
        let payload = crate::cdr::encode(message)?;
        let sender = if is_static {
            &self.static_sender
        } else {
            &self.sender
        };
        // Reserve transport capacity before changing the graph. A closed queue
        // must neither publish nor make a transform visible locally.
        let permit = sender
            .reserve()
            .await
            .map_err(|error| io::Error::new(io::ErrorKind::BrokenPipe, error))?;
        self.graph.insert(message, is_static)?;
        permit.send(payload);
        Ok(())
    }
}

pub struct Lookup<'a> {
    tf: &'a Tf,
    target: &'a str,
    source: &'a str,
    time: Option<Time>,
}
impl Lookup<'_> {
    pub fn at(mut self, time: Time) -> Self {
        self.time = Some(time);
        self
    }
    pub fn get(self) -> Option<TransformStamped> {
        self.resolve().ok()
    }
    fn resolve(&self) -> io::Result<TransformStamped> {
        match &self.time {
            Some(t) => self
                .tf
                .lookup_transform(self.target, self.source, t.clone()),
            None => self.tf.graph.lookup(self.target, self.source, None),
        }
    }
    pub async fn within(self, timeout: Duration) -> Option<TransformStamped> {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            let changed = self.tf.graph.changed.notified();
            tokio::pin!(changed);
            changed.as_mut().enable();
            if let Ok(edge) = self.resolve() {
                return Some(edge);
            }
            if tokio::time::timeout_at(deadline, changed).await.is_err() {
                return None;
            }
        }
    }
}

struct TfRoute {
    graph: Arc<Graph>,
    is_static: bool,
}
impl Route for TfRoute {
    fn try_dispatch(&self, data: &[u8]) {
        let result = crate::cdr::decode::<TFMessage>(data)
            .and_then(|msg| self.graph.insert(&msg, self.is_static));
        if let Err(error) = result {
            tracing::warn!(%error, is_static=self.is_static, "TF message rejected");
        }
    }
}

pub(crate) fn tf_subscription(
    sender: mpsc::Sender<Vec<u8>>,
    static_sender: mpsc::Sender<Vec<u8>>,
    window: Duration,
) -> (Tf, Box<dyn Route>, Box<dyn Route>) {
    let graph = Arc::new(Graph {
        buffer: RwLock::new(Registry::with_max_age(window)),
        changed: Notify::new(),
    });
    (
        Tf {
            graph: graph.clone(),
            sender,
            static_sender,
        },
        Box::new(TfRoute {
            graph: graph.clone(),
            is_static: false,
        }),
        Box::new(TfRoute {
            graph,
            is_static: true,
        }),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use dimos_generated_messages::{
        builtin_interfaces::msg::time::Time as MessageTime,
        geometry_msgs::msg::transform_stamped::TransformStamped as MessageTransform,
        tf2_msgs::msg::tf_message::TFMessage as MessageBatch,
    };
    fn edge(parent: &str, child: &str, ns: i64, x: f64) -> TransformStamped {
        from_isometry(
            parent,
            child,
            time_from_nanoseconds(ns).unwrap(),
            Isometry3::translation(x, 0., 0.),
        )
    }
    fn fixture() -> (
        Tf,
        Box<dyn Route>,
        Box<dyn Route>,
        mpsc::Receiver<Vec<u8>>,
        mpsc::Receiver<Vec<u8>>,
    ) {
        let (tx, rx) = mpsc::channel(16);
        let (static_tx, static_rx) = mpsc::channel(16);
        let (tf, dynamic, static_route) = tf_subscription(tx, static_tx, Duration::from_secs(10));
        (tf, dynamic, static_route, rx, static_rx)
    }
    #[tokio::test]
    async fn generated_messages_pass_through_tf_without_type_conversion() {
        let stamp = MessageTime {
            sec: 1_700_000_000,
            nanosec: 123_456_789,
        };
        let batch = MessageBatch {
            transforms: vec![from_isometry(
                "map",
                "camera",
                stamp.clone(),
                Isometry3::translation(3., 0., 0.),
            )],
        };
        let (tf, _, _, mut packets, _) = fixture();
        tf.publish(&batch.transforms).await.unwrap();
        let wire: MessageBatch = crate::cdr::decode(&packets.recv().await.unwrap()).unwrap();
        assert_eq!(wire, batch);
        let result: MessageTransform = tf.lookup_transform("map", "camera", stamp.clone()).unwrap();
        assert_eq!(result.header.stamp, stamp);
        assert_eq!(result.transform.translation.x, 3.);
    }

    #[test]
    fn invalid_query_frames_cannot_resolve_to_identity() {
        let (tf, _, _, _, _) = fixture();
        for frame in ["", "/map"] {
            assert!(tf
                .lookup_transform(frame, frame, Time { sec: 0, nanosec: 0 })
                .is_err());
        }
        assert_eq!(
            tf.get_latest("map", "map").unwrap().transform.rotation.w,
            1.
        );
    }
    #[tokio::test]
    async fn latest_resolves_the_whole_chain_at_its_common_time() {
        let (tf, _, _, _rx, _static_rx) = fixture();
        tf.publish(&[
            edge("map", "odom", 1_000_000_000, 1.),
            edge("map", "odom", 2_000_000_000, 2.),
            edge("odom", "base", 1_000_000_000, 10.),
        ])
        .await
        .unwrap();
        let result = tf.get_latest("map", "base").unwrap();
        assert_eq!(result.transform.translation.x, 11.);
        assert_eq!(nanoseconds(&result.header.stamp).unwrap(), 1_000_000_000);
        assert_eq!(
            tf.get_latest("base", "map")
                .unwrap()
                .transform
                .translation
                .x,
            -11.
        );
    }
    #[tokio::test]
    async fn interpolates_translation_and_rotation_without_extrapolation() {
        let (tf, _, _, _rx, _static_rx) = fixture();
        let end = from_isometry(
            "map",
            "base",
            time_from_nanoseconds(3_000_000_000).unwrap(),
            Isometry3::from_parts(
                Translation3::new(2., 0., 0.),
                UnitQuaternion::from_euler_angles(0., 0., std::f64::consts::FRAC_PI_2),
            ),
        );
        tf.publish(&[edge("map", "base", 1_000_000_000, 0.), end])
            .await
            .unwrap();
        let result = tf
            .lookup_transform("map", "base", time_from_nanoseconds(2_000_000_000).unwrap())
            .unwrap();
        assert_eq!(result.transform.translation.x, 1.);
        assert!(
            (isometry(&result).unwrap().rotation.angle() - std::f64::consts::FRAC_PI_4).abs()
                < 1e-10
        );
        assert!(tf
            .lookup_transform("map", "base", time_from_nanoseconds(4_000_000_000).unwrap())
            .is_err());
        assert!(tf
            .lookup_transform("map", "base", time_from_nanoseconds(500_000_000).unwrap())
            .is_err());
    }
    #[tokio::test]
    async fn static_edges_survive_eviction_and_do_not_limit_dynamic_time() {
        let (tf, _, _, _rx, mut static_rx) = fixture();
        tf.publish_static(&[edge("base", "camera", 0, 3.)])
            .await
            .unwrap();
        let message: TFMessage = crate::cdr::decode(&static_rx.recv().await.unwrap()).unwrap();
        assert_eq!(message.transforms[0].child_frame_id, "camera");
        tf.publish(&[
            edge("map", "base", 1, 1.),
            edge("map", "base", 30_000_000_000, 2.),
        ])
        .await
        .unwrap();
        let result = tf.get_latest("map", "camera").unwrap();
        assert_eq!(result.transform.translation.x, 5.);
        assert_eq!(nanoseconds(&result.header.stamp).unwrap(), 30_000_000_000);
        let at_negative = tf
            .lookup_transform(
                "base",
                "camera",
                Time {
                    sec: -1,
                    nanosec: 999_999_999,
                },
            )
            .unwrap();
        assert_eq!(at_negative.transform.translation.x, 3.);
        assert_eq!(nanoseconds(&at_negative.header.stamp).unwrap(), -1);
    }
    #[tokio::test]
    async fn real_ros_messages_preserve_epoch_nanoseconds_and_negative_times_in_both_endians() {
        for ns in [-1, 0, 1_700_000_000_123_456_789, 1_700_000_000_123_456_790] {
            for little in [true, false] {
                let (tf, route, _, _rx, _static_rx) = fixture();
                let original = TFMessage {
                    transforms: vec![edge("map", "base", ns, 4.)],
                };
                route.try_dispatch(&crate::cdr::encode_endian(&original, little).unwrap());
                let found = tf.get_latest("map", "base").unwrap();
                assert_eq!(nanoseconds(&found.header.stamp).unwrap(), ns);
                assert_eq!(found, original.transforms[0]);
            }
        }
        let (tf, _, _, _rx, _static_rx) = fixture();
        let ns = 1_700_000_000_123_456_789;
        tf.publish(&[edge("map", "base", ns, 1.), edge("map", "base", ns + 1, 2.)])
            .await
            .unwrap();
        assert_eq!(
            tf.lookup_transform("map", "base", time_from_nanoseconds(ns).unwrap())
                .unwrap()
                .transform
                .translation
                .x,
            1.
        );
        assert_eq!(
            tf.get_latest("map", "base")
                .unwrap()
                .transform
                .translation
                .x,
            2.
        );
    }
    #[tokio::test]
    async fn invalid_batches_do_not_change_the_graph_or_publish() {
        let (tf, route, _, mut rx, _static_rx) = fixture();
        tf.publish(&[edge("map", "base", 1, 1.)]).await.unwrap();
        rx.recv().await.unwrap();
        let cycle = vec![edge("base", "camera", 1, 3.), edge("camera", "map", 1, 4.)];
        assert!(tf.publish(&cycle).await.is_err());
        assert!(tf.get_latest("base", "camera").is_none());
        assert!(rx.try_recv().is_err());
        let mut malformed = edge("base", "camera", 1, 3.);
        malformed.transform.rotation.w = 0.;
        let invalid = TFMessage {
            transforms: vec![edge("base", "sensor", 1, 1.), malformed],
        };
        route.try_dispatch(&crate::cdr::encode(&invalid).unwrap());
        assert!(tf.get_latest("base", "sensor").is_none());
        let valid = crate::cdr::encode(&TFMessage {
            transforms: vec![edge("base", "sensor", 1, 1.)],
        })
        .unwrap();
        for bytes in [
            &valid[..valid.len() - 1],
            &[0, 9, 0, 0],
            &[0, 1, 0, 0, 255][..],
        ] {
            route.try_dispatch(bytes);
        }
        let mut trailing = valid;
        trailing.push(0);
        route.try_dispatch(&trailing);
        assert!(tf.get_latest("base", "sensor").is_none());
        assert_eq!(
            tf.get_latest("map", "base")
                .unwrap()
                .transform
                .translation
                .x,
            1.
        );
    }
    #[tokio::test]
    async fn closed_publication_queue_leaves_graph_unchanged() {
        let (tf, _, _, rx, _static_rx) = fixture();
        drop(rx);
        assert_eq!(
            tf.publish(&[edge("map", "base", 1, 1.)])
                .await
                .unwrap_err()
                .kind(),
            io::ErrorKind::BrokenPipe
        );
        assert!(tf.get_latest("map", "base").is_none());
    }
    #[tokio::test]
    async fn waiting_lookup_wakes_on_static_and_dynamic_messages() {
        let (tf, route, static_route, _rx, _static_rx) = fixture();
        let waiter = tf.clone();
        let task = tokio::spawn(async move {
            waiter
                .lookup("map", "camera")
                .within(Duration::from_secs(1))
                .await
                .unwrap()
        });
        static_route.try_dispatch(
            &crate::cdr::encode(&TFMessage {
                transforms: vec![edge("base", "camera", 0, 3.)],
            })
            .unwrap(),
        );
        route.try_dispatch(
            &crate::cdr::encode(&TFMessage {
                transforms: vec![edge("map", "base", 1, 1.)],
            })
            .unwrap(),
        );
        assert_eq!(task.await.unwrap().transform.translation.x, 4.);
    }
}
