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

/// Env vars the python launcher sets to mirror its own session config
/// (native_env in zenohservice.py). Python owns every value. A var it
/// left unset keeps zenoh's own default, and an invalid value is an error.
const ENV_CONNECT: &str = "DIMOS_ZENOH_CONNECT";
const ENV_LISTEN: &str = "DIMOS_ZENOH_LISTEN";
const ENV_MODE: &str = "DIMOS_ZENOH_MODE";
const ENV_MULTICAST: &str = "DIMOS_ZENOH_MULTICAST";
const ENV_GOSSIP: &str = "DIMOS_ZENOH_GOSSIP";
const ENV_INTERFACE: &str = "DIMOS_ZENOH_INTERFACE";

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

/// Trimmed value of an env var, None when unset or blank.
fn env_setting(name: &str) -> Option<String> {
    let value = std::env::var(name).ok()?;
    let value = value.trim();
    (!value.is_empty()).then(|| value.to_string())
}

/// Comma-separated locator list as a JSON array, None if nothing remains.
fn endpoints_json(endpoints: &str) -> Option<String> {
    let list: Vec<&str> = endpoints
        .split(',')
        .map(str::trim)
        .filter(|e| !e.is_empty())
        .collect();
    if list.is_empty() {
        None
    } else {
        Some(serde_json::Value::from(list).to_string())
    }
}

/// A bare string as a JSON string literal.
fn json_str(value: &str) -> String {
    serde_json::Value::String(value.to_string()).to_string()
}

/// Session config with every setting the launcher provided applied verbatim.
fn apply_settings(
    mut config: ::zenoh::Config,
    get: impl Fn(&str) -> Option<String>,
) -> io::Result<::zenoh::Config> {
    let inserts = [
        (
            "connect/endpoints",
            get(ENV_CONNECT).as_deref().and_then(endpoints_json),
        ),
        (
            "listen/endpoints",
            get(ENV_LISTEN).as_deref().and_then(endpoints_json),
        ),
        ("mode", get(ENV_MODE).as_deref().map(json_str)),
        ("scouting/multicast/enabled", get(ENV_MULTICAST)),
        (
            "scouting/multicast/interface",
            get(ENV_INTERFACE).as_deref().map(json_str),
        ),
        ("scouting/gossip/enabled", get(ENV_GOSSIP)),
    ];
    for (key, value) in inserts {
        if let Some(value) = value {
            config.insert_json5(key, &value).map_err(to_io)?;
        }
    }
    Ok(config)
}

impl ZenohTransport {
    pub async fn new() -> io::Result<Self> {
        let config = apply_settings(::zenoh::Config::default(), env_setting)?;
        let session = ::zenoh::open(config).await.map_err(to_io)?;
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

    /// Settings map for tests, pinning the exact names python's native_env emits.
    fn from_map(pairs: &'static [(&'static str, &'static str)]) -> impl Fn(&str) -> Option<String> {
        move |name| {
            pairs
                .iter()
                .find(|(key, _)| *key == name)
                .map(|(_, value)| value.to_string())
        }
    }

    #[test]
    fn settings_reach_the_session_config() {
        let config = apply_settings(
            ::zenoh::Config::default(),
            from_map(&[
                (
                    "DIMOS_ZENOH_CONNECT",
                    "tcp/10.0.0.9:7447, tcp/10.0.0.10:7447",
                ),
                ("DIMOS_ZENOH_MODE", "client"),
                ("DIMOS_ZENOH_MULTICAST", "true"),
                ("DIMOS_ZENOH_GOSSIP", "false"),
                ("DIMOS_ZENOH_INTERFACE", "lo"),
            ]),
        )
        .expect("valid settings apply");
        assert_eq!(
            config.get_json("connect/endpoints").unwrap(),
            r#"["tcp/10.0.0.9:7447","tcp/10.0.0.10:7447"]"#
        );
        assert_eq!(config.get_json("mode").unwrap(), r#""client""#);
        assert_eq!(
            config.get_json("scouting/multicast/enabled").unwrap(),
            "true"
        );
        assert_eq!(config.get_json("scouting/gossip/enabled").unwrap(), "false");
        assert_eq!(
            config.get_json("scouting/multicast/interface").unwrap(),
            r#""lo""#
        );
    }

    #[test]
    fn absent_settings_keep_zenohs_defaults() {
        let config = apply_settings(::zenoh::Config::default(), |_| None).expect("no-op applies");
        let default = ::zenoh::Config::default();
        for key in [
            "mode",
            "connect/endpoints",
            "scouting/multicast/enabled",
            "scouting/multicast/interface",
            "scouting/gossip/enabled",
        ] {
            assert_eq!(
                config.get_json(key).unwrap(),
                default.get_json(key).unwrap()
            );
        }
    }

    #[test]
    fn invalid_setting_is_an_error_not_a_fallback() {
        let bad_bool = from_map(&[("DIMOS_ZENOH_MULTICAST", "bananas")]);
        assert!(apply_settings(::zenoh::Config::default(), bad_bool).is_err());
    }

    #[test]
    fn endpoints_json_empty_yields_none() {
        assert_eq!(endpoints_json(""), None);
        assert_eq!(endpoints_json("  , ,  "), None);
    }

    #[test]
    fn endpoints_json_filters_blank_segments() {
        assert_eq!(
            endpoints_json(" ,tcp/go2:7447,, "),
            Some(r#"["tcp/go2:7447"]"#.to_string())
        );
    }

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
