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

use std::collections::{HashMap, HashSet};
use std::io;
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use ::zenoh::pubsub::Publisher;
use ::zenoh::qos::{CongestionControl, Reliability};
use ::zenoh::sample::Locality;
use ::zenoh::Session;
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;

use crate::transport::{Dispatch, Transport};

pub(crate) const SESSION_KEY: &str = "session";

/// Poll interval while waiting for the dialed endpoints to link.
const CONNECT_POLL: Duration = Duration::from_millis(50);

/// How this session joins the network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
enum Mode {
    Peer,
    Client,
    Router,
}

/// The session settings python sends on the launch line. It owns every value
/// and resolves every derived one, so no field here has a default.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SessionSettings {
    mode: Mode,
    connect: Vec<String>,
    listen: Vec<String>,
    multicast: bool,
    scout_addr: String,
    gossip: bool,
    interface: String,
    connect_timeout_ms: u64,
}

/// A setting as the JSON text zenoh's config setter takes.
fn json_text<T: serde::Serialize>(value: &T) -> String {
    serde_json::to_string(value).expect("session settings are JSON")
}

impl SessionSettings {
    /// The settings the launch carried, absent when it carried none.
    ///
    /// A module started by hand keeps zenoh's own defaults.
    fn from_launch(launch: &serde_json::Value) -> io::Result<Option<Self>> {
        match launch.get(SESSION_KEY) {
            None | Some(serde_json::Value::Null) => Ok(None),
            Some(value) => serde_json::from_value(value.clone())
                .map(Some)
                .map_err(|e| {
                    io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("failed to deserialize zenoh session settings: {e}"),
                    )
                }),
        }
    }

    /// These settings as a zenoh session config.
    fn zenoh_config(&self) -> io::Result<::zenoh::Config> {
        let mut config = ::zenoh::Config::default();
        let mut inserts = vec![
            ("mode", json_text(&self.mode)),
            ("scouting/multicast/enabled", json_text(&self.multicast)),
            ("scouting/multicast/interface", json_text(&self.interface)),
            ("scouting/gossip/enabled", json_text(&self.gossip)),
        ];
        // Empty means the stock multicast group. A moved group is a private
        // discovery bus, which is how parallel sessions on one host stay apart.
        if !self.scout_addr.is_empty() {
            inserts.push(("scouting/multicast/address", json_text(&self.scout_addr)));
        }
        // An empty list means "whatever zenoh listens on by default", which for
        // a peer is an ephemeral port, not nothing at all.
        if !self.connect.is_empty() {
            inserts.push(("connect/endpoints", json_text(&self.connect)));
        }
        if !self.listen.is_empty() {
            inserts.push(("listen/endpoints", json_text(&self.listen)));
        }
        // Zero means "don't wait", which zenoh reads as "dial once and never
        // retry". Leaving the key unset keeps its own retry policy.
        if self.connect_timeout_ms > 0 {
            inserts.push(("connect/timeout_ms", json_text(&self.connect_timeout_ms)));
        }
        for (key, value) in inserts {
            config.insert_json5(key, &value).map_err(to_io)?;
        }
        Ok(config)
    }

    fn connect_timeout(&self) -> Duration {
        Duration::from_millis(self.connect_timeout_ms)
    }
}

/// Publisher QoS for one channel. `None` fields keep zenoh's defaults.
#[derive(Clone, Default)]
struct ChannelQos {
    reliability: Option<Reliability>,
    congestion_control: Option<CongestionControl>,
    /// Where the publisher is allowed to deliver. `SessionLocal` keeps a topic
    /// inside the process that publishes it, which is how a baked host hides
    /// its internal hops without changing the modules.
    locality: Option<Locality>,
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
        match entry.get("locality").and_then(|v| v.as_str()) {
            Some("session_local") => qos.locality = Some(Locality::SessionLocal),
            Some("remote") => qos.locality = Some(Locality::Remote),
            Some("any") => qos.locality = Some(Locality::Any),
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

/// The host:port forms a live link to this locator may report.
///
/// A locator dialed by name never matches the address the link reports.
async fn endpoint_addresses(endpoint: &str) -> HashSet<String> {
    let address = endpoint.rsplit('/').next().unwrap_or(endpoint);
    let mut out = HashSet::from([address.to_string()]);
    if let Ok(resolved) = tokio::net::lookup_host(address).await {
        out.extend(resolved.map(|addr| addr.to_string()));
    }
    out
}

/// Block until the dialed endpoints have links, bounded by the timeout.
///
/// A peer opens before its endpoints are dialed, so without this the first
/// published messages have nowhere to go.
async fn await_connect(session: &Session, endpoints: &[String], mode: Mode, timeout: Duration) {
    if endpoints.is_empty() || timeout.is_zero() {
        return;
    }
    let mut pending: Vec<(&str, HashSet<String>)> = Vec::with_capacity(endpoints.len());
    for endpoint in endpoints {
        pending.push((endpoint.as_str(), endpoint_addresses(endpoint).await));
    }
    // A client session holds one link. Zenoh dials the endpoints as
    // alternatives and keeps the first that answers.
    let needed = if mode == Mode::Client {
        1
    } else {
        pending.len()
    };
    let deadline = Instant::now() + timeout;
    loop {
        let linked: HashSet<String> = session
            .info()
            .links()
            .await
            .map(|link| link.dst().address().to_string())
            .collect();
        pending.retain(|(_, addresses)| addresses.is_disjoint(&linked));
        if endpoints.len() - pending.len() >= needed {
            return;
        }
        if Instant::now() >= deadline {
            let unlinked: Vec<&str> = pending.iter().map(|(endpoint, _)| *endpoint).collect();
            tracing::warn!(
                endpoints = ?unlinked,
                timeout_ms = timeout.as_millis(),
                "zenoh endpoints not linked, published messages may be dropped"
            );
            return;
        }
        tokio::time::sleep(CONNECT_POLL).await;
    }
}

impl ZenohTransport {
    /// Open a transport from the session settings in a coordinator launch object.
    pub async fn from_launch(launch: &serde_json::Value) -> io::Result<Self> {
        match SessionSettings::from_launch(launch)? {
            Some(settings) => Self::open(&settings).await,
            None => Self::new().await,
        }
    }

    /// Open a session on zenoh's own defaults.
    pub async fn new() -> io::Result<Self> {
        let session = ::zenoh::open(::zenoh::Config::default())
            .await
            .map_err(to_io)?;
        Ok(Self::wrap(session))
    }

    async fn open(settings: &SessionSettings) -> io::Result<Self> {
        if settings.mode == Mode::Client && settings.connect.len() > 1 {
            tracing::warn!(
                connect = ?settings.connect,
                "zenoh client mode holds a single link, traffic flows only through \
                 the first endpoint that connects"
            );
        }
        let session = ::zenoh::open(settings.zenoh_config()?)
            .await
            .map_err(to_io)?;
        tracing::info!(
            zid = %session.zid(),
            mode = ?settings.mode,
            connect = ?settings.connect,
            listen = ?settings.listen,
            "zenoh session opened"
        );
        await_connect(
            &session,
            &settings.connect,
            settings.mode,
            settings.connect_timeout(),
        )
        .await;
        Ok(Self::wrap(session))
    }

    fn wrap(session: Session) -> Self {
        Self {
            session,
            qos: OnceLock::new(),
            publishers: Mutex::new(HashMap::new()),
        }
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
        if let Some(locality) = qos.locality {
            builder = builder.allowed_destination(locality);
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

    /// A launch config carrying python's session settings, with these fields overridden.
    fn launch(overrides: serde_json::Value) -> serde_json::Value {
        let mut session = serde_json::json!({
            "mode": "peer",
            "connect": [],
            "listen": [],
            "multicast": true,
            "scout_addr": "",
            "gossip": false,
            "interface": "lo",
            "connect_timeout_ms": 1000,
        });
        let session_map = session.as_object_mut().unwrap();
        for (key, value) in overrides.as_object().expect("an object of overrides") {
            assert!(
                session_map.insert(key.clone(), value.clone()).is_some(),
                "{key} is not a session setting"
            );
        }
        serde_json::json!({"topics": {}, "config": null, "session": session})
    }

    fn settings(overrides: serde_json::Value) -> SessionSettings {
        SessionSettings::from_launch(&launch(overrides))
            .expect("settings deserialize")
            .expect("a session was sent")
    }

    // The goldens are shared with dimos/protocol/service/test_zenoh_wire.py,
    // which asserts to_wire produces exactly these bytes.
    fn golden(text: &str) -> SessionSettings {
        let session: serde_json::Value = serde_json::from_str(text).expect("golden is JSON");
        SessionSettings::from_launch(&serde_json::json!({"session": session}))
            .expect("golden deserializes")
            .expect("golden carries a session")
    }

    #[test]
    fn the_client_golden_parses_into_the_settings_python_sends() {
        let settings = golden(include_str!("../tests/fixtures/session_wire_client.json"));
        assert_eq!(settings.mode, Mode::Client);
        assert_eq!(settings.connect, ["tcp/192.0.2.10:7447"]);
        assert!(settings.listen.is_empty());
        assert!(settings.multicast);
        assert!(settings.scout_addr.is_empty());
        assert!(!settings.gossip);
        assert_eq!(settings.interface, "lo");
        assert_eq!(settings.connect_timeout_ms, 2000);
    }

    #[test]
    fn the_router_golden_parses_into_the_settings_python_sends() {
        let settings = golden(include_str!("../tests/fixtures/session_wire_router.json"));
        assert_eq!(settings.mode, Mode::Router);
        assert!(settings.connect.is_empty());
        assert_eq!(settings.listen, ["tcp/127.0.0.1:7447"]);
        assert!(!settings.multicast);
        assert!(settings.scout_addr.is_empty());
        assert!(settings.gossip);
        assert_eq!(settings.interface, "auto");
        assert_eq!(settings.connect_timeout_ms, 0);
    }

    #[test]
    fn settings_reach_the_session_config() {
        let config = settings(serde_json::json!({
            "mode": "client",
            "connect": ["tcp/10.0.0.9:7447", "tcp/10.0.0.10:7447"],
            "interface": "wlan0",
        }))
        .zenoh_config()
        .expect("valid settings apply");

        assert_eq!(config.get_json("mode").unwrap(), r#""client""#);
        assert_eq!(
            config.get_json("connect/endpoints").unwrap(),
            r#"["tcp/10.0.0.9:7447","tcp/10.0.0.10:7447"]"#
        );
        assert_eq!(
            config.get_json("scouting/multicast/enabled").unwrap(),
            "true"
        );
        assert_eq!(config.get_json("scouting/gossip/enabled").unwrap(), "false");
        assert_eq!(
            config.get_json("scouting/multicast/interface").unwrap(),
            r#""wlan0""#
        );
        assert_eq!(config.get_json("connect/timeout_ms").unwrap(), "1000");
    }

    #[test]
    fn a_moved_scout_group_reaches_the_session_config() {
        let config = settings(serde_json::json!({"scout_addr": "224.0.0.224:17700"}))
            .zenoh_config()
            .expect("valid settings apply");
        assert_eq!(
            config.get_json("scouting/multicast/address").unwrap(),
            r#""224.0.0.224:17700""#
        );
    }

    #[test]
    fn an_empty_scout_group_keeps_zenohs_own() {
        let config = settings(serde_json::json!({})).zenoh_config().unwrap();
        let default = ::zenoh::Config::default();
        assert_eq!(
            config.get_json("scouting/multicast/address").unwrap(),
            default.get_json("scouting/multicast/address").unwrap()
        );
    }

    #[test]
    fn empty_endpoint_lists_keep_zenohs_defaults() {
        // An empty listen list must not be sent as "listen on nothing".
        let config = settings(serde_json::json!({})).zenoh_config().unwrap();
        let default = ::zenoh::Config::default();
        for key in ["connect/endpoints", "listen/endpoints"] {
            assert_eq!(
                config.get_json(key).unwrap(),
                default.get_json(key).unwrap()
            );
        }
    }

    #[test]
    fn a_zero_timeout_keeps_zenohs_own_retry_policy() {
        // Zenoh reads a zero timeout as dial once and never retry.
        let config = settings(serde_json::json!({"connect_timeout_ms": 0}))
            .zenoh_config()
            .unwrap();
        assert_eq!(
            config.get_json("connect/timeout_ms").unwrap(),
            ::zenoh::Config::default()
                .get_json("connect/timeout_ms")
                .unwrap()
        );
    }

    #[test]
    fn a_launch_without_a_session_keeps_zenohs_defaults() {
        let launch = serde_json::json!({"topics": {}, "config": null});
        assert!(SessionSettings::from_launch(&launch).unwrap().is_none());
    }

    #[test]
    fn a_missing_setting_is_an_error_not_a_default() {
        let launch = serde_json::json!({"session": {"mode": "peer"}});
        assert!(SessionSettings::from_launch(&launch).is_err());
    }

    #[test]
    fn an_unknown_setting_is_an_error() {
        let mut incoming = launch(serde_json::json!({}));
        incoming["session"]["reliability"] = serde_json::json!("reliable");
        assert!(SessionSettings::from_launch(&incoming).is_err());
    }

    #[tokio::test]
    async fn endpoint_addresses_keeps_the_literal_address() {
        assert!(endpoint_addresses("tcp/192.0.2.10:7447")
            .await
            .contains("192.0.2.10:7447"));
    }

    #[tokio::test]
    async fn endpoint_addresses_resolves_names() {
        assert!(endpoint_addresses("tcp/localhost:7447")
            .await
            .contains("127.0.0.1:7447"));
    }

    /// Session in a test topology, discovery off so only the dialed endpoints
    /// can produce a link.
    async fn open_session(mode: &str, connect: &[&str], listen: &[&str]) -> Session {
        let config = settings(topology(mode, connect, listen))
            .zenoh_config()
            .expect("valid settings apply");
        ::zenoh::open(config).await.expect("open session")
    }

    /// A loopback endpoint nothing is listening on yet.
    fn free_endpoint() -> String {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind an ephemeral port");
        let port = listener.local_addr().expect("read the bound port").port();
        format!("tcp/127.0.0.1:{port}")
    }

    /// Session settings for a test topology, with discovery off.
    fn topology(mode: &str, connect: &[&str], listen: &[&str]) -> serde_json::Value {
        serde_json::json!({
            "mode": mode,
            "connect": connect,
            "listen": listen,
            "multicast": false,
        })
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn from_launch_opens_the_session_the_launch_describes() {
        let endpoint = free_endpoint();
        let transport = ZenohTransport::from_launch(&launch(topology("router", &[], &[&endpoint])))
            .await
            .expect("open from launch");

        // The listen endpoint reached zenoh only if a peer can dial it.
        let peer = open_session("peer", &[&endpoint], &[]).await;
        await_connect(
            &peer,
            std::slice::from_ref(&endpoint),
            Mode::Peer,
            Duration::from_secs(10),
        )
        .await;
        assert!(peer.info().links().await.next().is_some());
        drop(transport);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn from_launch_without_a_session_still_opens() {
        let launch = serde_json::json!({"topics": {}, "config": null});
        assert!(ZenohTransport::from_launch(&launch).await.is_ok());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn from_launch_rejects_settings_it_cannot_read() {
        let launch = serde_json::json!({"session": {"mode": "mesh"}});
        assert!(ZenohTransport::from_launch(&launch).await.is_err());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn await_connect_returns_once_the_endpoint_links() {
        let endpoint = free_endpoint();
        let _router = open_session("router", &[], &[&endpoint]).await;
        let peer = open_session("peer", &[&endpoint], &[]).await;

        let started = Instant::now();
        await_connect(
            &peer,
            std::slice::from_ref(&endpoint),
            Mode::Peer,
            Duration::from_secs(10),
        )
        .await;

        assert!(started.elapsed() < Duration::from_secs(5));
        assert!(peer.info().links().await.next().is_some());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn await_connect_gives_up_after_the_timeout() {
        // Nothing listens here, so the endpoint never links.
        let endpoint = free_endpoint();
        let peer = open_session("peer", &[&endpoint], &[]).await;

        let started = Instant::now();
        await_connect(
            &peer,
            std::slice::from_ref(&endpoint),
            Mode::Peer,
            Duration::from_millis(300),
        )
        .await;

        assert!(started.elapsed() >= Duration::from_millis(300));
        assert!(started.elapsed() < Duration::from_secs(5));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn await_connect_is_skipped_without_endpoints_or_timeout() {
        let peer = open_session("peer", &[], &[]).await;
        let started = Instant::now();
        await_connect(&peer, &[], Mode::Peer, Duration::from_secs(30)).await;
        await_connect(&peer, &[free_endpoint()], Mode::Peer, Duration::ZERO).await;
        assert!(started.elapsed() < Duration::from_secs(1));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn client_await_is_satisfied_by_one_of_its_alternatives() {
        let endpoint = free_endpoint();
        let unreachable = free_endpoint();
        let _router = open_session("router", &[], &[&endpoint]).await;
        let client = open_session("client", &[&endpoint, &unreachable], &[]).await;

        let started = Instant::now();
        await_connect(
            &client,
            &[endpoint.clone(), unreachable.clone()],
            Mode::Client,
            Duration::from_secs(10),
        )
        .await;

        assert!(started.elapsed() < Duration::from_secs(5));
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
    fn parse_channel_qos_reads_locality() {
        let value = serde_json::json!({
            "suppressed": {"locality": "session_local"},
            "explicit_any": {"locality": "any"},
            "plain": {"reliability": "reliable"},
        });
        let map = parse_channel_qos(&value);
        assert_eq!(map["suppressed"].locality, Some(Locality::SessionLocal));
        assert_eq!(map["explicit_any"].locality, Some(Locality::Any));
        assert_eq!(map["plain"].locality, None);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_session_local_publisher_still_reaches_its_own_session() {
        // Siblings share the session, so a session-local publisher must still
        // deliver in-process.
        let transport = ZenohTransport::new().await.expect("open session");
        transport.set_publisher_qos(&serde_json::json!({
            "dimos_test/suppressed": {"locality": "session_local"},
        }));

        let (tx, mut rx) = tokio::sync::mpsc::channel::<Vec<u8>>(8);
        let sink: Dispatch = Arc::new(move |bytes: &[u8]| {
            let _ = tx.try_send(bytes.to_vec());
        });
        transport
            .subscribe("dimos_test/suppressed", sink)
            .await
            .expect("subscribe");

        let payload = b"internal hop";
        let received = tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                transport
                    .publish("dimos_test/suppressed", payload.to_vec())
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
        .expect("a session-local publisher must still deliver in-session");

        assert_eq!(received, payload);
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
