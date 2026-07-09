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

use std::collections::HashMap;
use std::io;
use std::sync::{Arc, OnceLock};

use ::zenoh::pubsub::Publisher;
use ::zenoh::qos::{CongestionControl, Reliability};
use ::zenoh::Session;
use tokio::sync::Mutex;

use crate::transport::{Dispatch, Transport};

/// Publisher QoS for one channel. `None` fields keep zenoh's defaults.
#[derive(Clone, Default)]
struct ChannelQos {
    reliability: Option<Reliability>,
    congestion_control: Option<CongestionControl>,
}

/// Parse the coordinator's `qos` object (channel -> {reliability,
/// congestion_control}) into a lookup. Unknown or absent fields keep defaults.
fn parse_channel_qos(value: &serde_json::Value) -> HashMap<String, ChannelQos> {
    let mut map = HashMap::new();
    let Some(object) = value.as_object() else {
        return map;
    };
    for (channel, entry) in object {
        let mut qos = ChannelQos::default();
        match entry.get("reliability").and_then(|v| v.as_str()) {
            Some("reliable") => qos.reliability = Some(Reliability::Reliable),
            Some("best_effort") => qos.reliability = Some(Reliability::BestEffort),
            _ => {}
        }
        match entry.get("congestion_control").and_then(|v| v.as_str()) {
            Some("drop") => qos.congestion_control = Some(CongestionControl::Drop),
            Some("block") => qos.congestion_control = Some(CongestionControl::Block),
            _ => {}
        }
        map.insert(channel.clone(), qos);
    }
    map
}

/// Zenoh transport for a native module.
pub struct ZenohTransport {
    session: Session,
    qos: OnceLock<HashMap<String, ChannelQos>>,
    publishers: Mutex<HashMap<String, Arc<Publisher<'static>>>>,
}

impl ZenohTransport {
    pub async fn new() -> io::Result<Self> {
        let session = ::zenoh::open(::zenoh::Config::default())
            .await
            .map_err(to_io)?;
        Ok(Self {
            session,
            qos: OnceLock::new(),
            publishers: Mutex::new(HashMap::new()),
        })
    }

    async fn declare_publisher(&self, channel: &str) -> io::Result<Publisher<'static>> {
        let qos = self
            .qos
            .get()
            .and_then(|map| map.get(channel))
            .cloned()
            .unwrap_or_default();
        let mut builder = self
            .session
            .declare_publisher(zenoh_key(channel).to_string());
        if let Some(congestion_control) = qos.congestion_control {
            builder = builder.congestion_control(congestion_control);
        }
        if let Some(reliability) = qos.reliability {
            builder = builder.reliability(reliability);
        }
        builder.await.map_err(to_io)
    }
}

impl Transport for ZenohTransport {
    async fn publish(&self, channel: &str, data: Vec<u8>) -> io::Result<()> {
        // Release the lock before `put` so a stalled channel can't block others.
        let publisher = {
            let mut publishers = self.publishers.lock().await;
            match publishers.get(channel) {
                Some(publisher) => Arc::clone(publisher),
                None => {
                    let publisher = Arc::new(self.declare_publisher(channel).await?);
                    publishers.insert(channel.to_string(), Arc::clone(&publisher));
                    publisher
                }
            }
        };
        publisher.put(data).await.map_err(to_io)
    }

    async fn subscribe(&self, channel: &str, on_msg: Dispatch) -> io::Result<()> {
        self.session
            .declare_subscriber(zenoh_key(channel).to_string())
            .callback(move |sample| on_msg(&sample.payload().to_bytes()))
            .background()
            .await
            .map_err(to_io)
    }

    fn set_publisher_qos(&self, qos: &serde_json::Value) {
        let _ = self.qos.set(parse_channel_qos(qos));
    }
}

fn to_io(e: ::zenoh::Error) -> io::Error {
    io::Error::other(e)
}

/// Zenoh keys can't start with '/'
fn zenoh_key(channel: &str) -> &str {
    channel.strip_prefix('/').unwrap_or(channel)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::time::Duration;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn round_trip_delivers_payload() {
        let transport = ZenohTransport::new().await.expect("open session");

        let (tx, mut rx) = tokio::sync::mpsc::channel::<Vec<u8>>(8);
        let sink: Dispatch = Arc::new(move |bytes: &[u8]| {
            let _ = tx.try_send(bytes.to_vec());
        });
        // A leading '/' is an invalid Zenoh key, so keys are slash-free.
        transport
            .subscribe("dimos_test/round_trip", sink)
            .await
            .expect("subscribe");

        let payload = b"hello zenoh";
        // Publish until the subscriber sees it, to tolerate subscription setup latency.
        let received = tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                transport
                    .publish("dimos_test/round_trip", payload.to_vec())
                    .await
                    .expect("publish");
                if let Ok(Some(got)) =
                    tokio::time::timeout(Duration::from_millis(100), rx.recv()).await
                {
                    break got;
                }
            }
        })
        .await
        .expect("payload not delivered within timeout");

        assert_eq!(received, payload);
    }

    #[test]
    fn parse_channel_qos_reads_set_fields() {
        let value = serde_json::json!({
            "dimos/img/sensor_msgs.Image": {"reliability": "best_effort", "congestion_control": "drop"},
            "dimos/agent": {"reliability": "reliable", "congestion_control": "block"},
        });
        let map = parse_channel_qos(&value);

        let img = &map["dimos/img/sensor_msgs.Image"];
        assert_eq!(img.reliability, Some(Reliability::BestEffort));
        assert_eq!(img.congestion_control, Some(CongestionControl::Drop));

        let agent = &map["dimos/agent"];
        assert_eq!(agent.reliability, Some(Reliability::Reliable));
        assert_eq!(agent.congestion_control, Some(CongestionControl::Block));
    }

    #[test]
    fn parse_channel_qos_leaves_absent_and_unknown_as_default() {
        let value = serde_json::json!({
            "only_reliability": {"reliability": "reliable"},
            "unknown_values": {"reliability": "sometimes", "congestion_control": "maybe"},
        });
        let map = parse_channel_qos(&value);

        let partial = &map["only_reliability"];
        assert_eq!(partial.reliability, Some(Reliability::Reliable));
        assert_eq!(partial.congestion_control, None);

        let unknown = &map["unknown_values"];
        assert_eq!(unknown.reliability, None);
        assert_eq!(unknown.congestion_control, None);
    }

    #[test]
    fn parse_channel_qos_ignores_non_object() {
        assert!(parse_channel_qos(&serde_json::Value::Null).is_empty());
    }

    #[test]
    fn zenoh_key_strips_only_the_leading_slash_fallback() {
        // Unmapped-port fallback `/{port}` is invalid as a Zenoh key; strip it.
        assert_eq!(zenoh_key("/cmd_vel"), "cmd_vel");
        // Mapped channels are already slash-free and pass through untouched.
        assert_eq!(
            zenoh_key("dimos/cmd_vel/geometry_msgs.Twist"),
            "dimos/cmd_vel/geometry_msgs.Twist"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn declared_publisher_carries_configured_qos() {
        // Verifies our plumbing: the QoS the coordinator sends lands on the
        // zenoh publisher. Zenoh owns whether it then drops/blocks on the wire.
        let transport = ZenohTransport::new().await.expect("open session");
        transport.set_publisher_qos(&serde_json::json!({
            "dimos_test/drop_chan": {"reliability": "best_effort", "congestion_control": "drop"},
            "dimos_test/block_chan": {"reliability": "reliable", "congestion_control": "block"},
        }));

        let dropper = transport
            .declare_publisher("dimos_test/drop_chan")
            .await
            .expect("declare drop publisher");
        assert_eq!(dropper.congestion_control(), CongestionControl::Drop);
        assert_eq!(dropper.reliability(), Reliability::BestEffort);

        let blocker = transport
            .declare_publisher("dimos_test/block_chan")
            .await
            .expect("declare block publisher");
        assert_eq!(blocker.congestion_control(), CongestionControl::Block);
        assert_eq!(blocker.reliability(), Reliability::Reliable);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn qos_channel_declares_publisher_and_delivers() {
        let transport = ZenohTransport::new().await.expect("open session");
        transport.set_publisher_qos(&serde_json::json!({
            "dimos_test/qos_channel": {"reliability": "best_effort", "congestion_control": "drop"},
        }));

        let (tx, mut rx) = tokio::sync::mpsc::channel::<Vec<u8>>(8);
        let sink: Dispatch = Arc::new(move |bytes: &[u8]| {
            let _ = tx.try_send(bytes.to_vec());
        });
        transport
            .subscribe("dimos_test/qos_channel", sink)
            .await
            .expect("subscribe");

        let payload = b"qos payload";
        let received = tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                transport
                    .publish("dimos_test/qos_channel", payload.to_vec())
                    .await
                    .expect("publish");
                if let Ok(Some(got)) =
                    tokio::time::timeout(Duration::from_millis(100), rx.recv()).await
                {
                    break got;
                }
            }
        })
        .await
        .expect("payload not delivered within timeout");

        assert_eq!(received, payload);
    }
}
