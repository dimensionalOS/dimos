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

use std::collections::{BTreeSet, HashMap};
use std::fmt::Debug;
use std::io;
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::watch;
use tracing::{error, info};
use tracing_subscriber::EnvFilter;

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use validator::Validate;

use crate::transport::{BoxFuture, Deliver, Dispatch, Transport};

/// Marker trait for a config checked by `#[native_config]`: every field required,
/// no Rust-side defaults, no unknown fields. Implemented only by the macro.
pub trait NativeConfig {}

/// Trait required by `Module::Config`s to ensure that configurations are
/// validated correctly. `Send` because a config is parsed on the host's main
/// thread and handed to the module's own thread.
pub trait ModuleConfig:
    DeserializeOwned + Serialize + Debug + Validate + NativeConfig + Send
{
}
impl<T: DeserializeOwned + Serialize + Debug + Validate + NativeConfig + Send> ModuleConfig for T {}

/// Default config type used by `#[derive(Module)]` when no `#[config]` field
/// is used. Just a stand in for modules that don't use configurations.
#[derive(Debug, Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NoConfig;

impl NativeConfig for NoConfig {}

impl Validate for NoConfig {
    fn validate(&self) -> Result<(), validator::ValidationErrors> {
        Ok(())
    }
}

pub(crate) fn init_tracing() {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .json()
        .with_writer(std::io::stderr)
        .with_env_filter(filter)
        .try_init();
}

// Only absorbs jitter between the transport and the handler. A full buffer
// blocks the transport so congestion reaches the publisher's QoS.
const INPUT_BUFFER_LEN: usize = 8;

// Each input() call produces a TypedRoute that decodes its message type
// and forwards it to the right Input's channel.
struct TypedRoute<T: Send + 'static> {
    topic: String,
    decode: fn(&[u8]) -> io::Result<T>,
    sender: async_channel::Sender<T>,
}

impl<T: Send + 'static> TypedRoute<T> {
    fn parse(&self, data: &[u8]) -> Option<T> {
        match (self.decode)(data) {
            Ok(msg) => Some(msg),
            Err(e) => {
                error!(topic = %self.topic, error = %e, "decode error");
                None
            }
        }
    }
}

// A closed channel means the module is stopping.
impl<T: Send + 'static> Deliver for TypedRoute<T> {
    fn deliver(&self, data: &[u8]) {
        if let Some(msg) = self.parse(data) {
            let _ = self.sender.send_blocking(msg);
        }
    }

    fn deliver_async<'a>(&'a self, data: &'a [u8]) -> BoxFuture<'a, ()> {
        Box::pin(async move {
            if let Some(msg) = self.parse(data) {
                let _ = self.sender.send(msg).await;
            }
        })
    }
}
pub struct Input<T> {
    pub topic: String,
    receiver: async_channel::Receiver<T>,
}

impl<T> Input<T> {
    pub async fn recv(&mut self) -> Option<T> {
        self.receiver.recv().await.ok()
    }
}

/// Publishes straight to the transport from the calling task. Under `block`
/// QoS a stalled put parks the caller until the transport takes the message.
#[derive(Clone)]
pub struct Output<T> {
    pub topic: String,
    encode: fn(&T) -> Vec<u8>,
    transport: Arc<dyn Transport>,
}

impl<T> Output<T> {
    pub async fn publish(&self, msg: &T) -> io::Result<()> {
        self.transport
            .publish(&self.topic, (self.encode)(msg))
            .await
    }
}

/// A port that publishes to and subscribes on the same topic.
///
/// The transports deliver a message back to its own sender, so an `Io` port
/// sees the whole topic including its own publishes.
pub struct Io<T> {
    pub topic: String,
    receiver: async_channel::Receiver<T>,
    encode: fn(&T) -> Vec<u8>,
    transport: Arc<dyn Transport>,
}

impl<T> Io<T> {
    pub async fn recv(&mut self) -> Option<T> {
        self.receiver.recv().await.ok()
    }

    pub async fn publish(&self, msg: &T) -> io::Result<()> {
        self.transport
            .publish(&self.topic, (self.encode)(msg))
            .await
    }
}

/// Extract `(topics, config)` from an already-parsed launch line. The caller
/// parses the line once and the transport reads its own blocks from the same
/// value, so this takes the value.
pub(crate) fn parse_config_value<C: DeserializeOwned + Serialize>(
    json: &serde_json::Value,
) -> io::Result<(HashMap<String, String>, C)> {
    let mut topics = HashMap::new();
    if let Some(t) = json.get("topics").and_then(|v| v.as_object()) {
        for (port, topic) in t {
            if let Some(s) = topic.as_str() {
                topics.insert(port.clone(), s.to_string());
            }
        }
    }

    let config_value = json.get("config").ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            "missing 'config' field in stdin JSON: coordinator must always send a config object",
        )
    })?;

    let config: C = serde_json::from_value(config_value.clone()).map_err(|e| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("failed to deserialize config: {e}"),
        )
    })?;

    enforce_one_to_one(config_value, &config)?;

    Ok((topics, config))
}

fn object_keys(value: &serde_json::Value) -> BTreeSet<String> {
    value
        .as_object()
        .map(|m| m.keys().cloned().collect())
        .unwrap_or_default()
}

fn enforce_one_to_one<C: Serialize>(provided: &serde_json::Value, config: &C) -> io::Result<()> {
    let expected_value = serde_json::to_value(config).map_err(|e| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("failed to re-serialize config: {e}"),
        )
    })?;
    let provided_keys = object_keys(provided);
    let expected_keys = object_keys(&expected_value);
    if provided_keys == expected_keys {
        return Ok(());
    }
    let missing: Vec<&String> = expected_keys.difference(&provided_keys).collect();
    let unexpected: Vec<&String> = provided_keys.difference(&expected_keys).collect();
    Err(io::Error::new(
        io::ErrorKind::InvalidData,
        format!("config keys do not match struct fields: missing {missing:?}, unexpected {unexpected:?}"),
    ))
}

fn with_field(field: &str, message: String) -> String {
    if field == "__all__" {
        message
    } else {
        format!("{field}: {message}")
    }
}

fn format_validation_errors(errors: &validator::ValidationErrors) -> String {
    use validator::ValidationErrorsKind;
    let mut messages = Vec::new();
    for (field, kind) in errors.errors() {
        match kind {
            ValidationErrorsKind::Field(field_errs) => {
                for err in field_errs {
                    let label = err.message.as_deref().unwrap_or(err.code.as_ref());
                    let mut bounds: Vec<String> = err
                        .params
                        .iter()
                        .filter(|(k, _)| k.as_ref() != "value")
                        .map(|(k, v)| format!("{k}={v}"))
                        .collect();
                    bounds.sort();
                    let bounds_str = if bounds.is_empty() {
                        String::new()
                    } else {
                        format!(" ({})", bounds.join(", "))
                    };
                    let got = err
                        .params
                        .get("value")
                        .map(|v| format!(" got {v}"))
                        .unwrap_or_default();
                    messages.push(with_field(field, format!("{label}{bounds_str}{got}")));
                }
            }
            ValidationErrorsKind::Struct(nested) => {
                messages.push(with_field(field, format_validation_errors(nested)));
            }
            ValidationErrorsKind::List(list) => {
                for (idx, errs) in list {
                    messages.push(format!(
                        "{field}[{idx}]: {}",
                        format_validation_errors(errs)
                    ));
                }
            }
        }
    }
    messages.join("; ")
}

pub(crate) fn validate_config<C: Validate>(config: &C) -> io::Result<()> {
    config.validate().map_err(|errs| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "config validation failed: {}",
                format_validation_errors(&errs)
            ),
        )
    })
}

pub trait Module: Sized + Send + 'static {
    type Config: ModuleConfig;

    fn build(builder: &mut Builder, config: Self::Config) -> Self;

    fn setup(&mut self) -> impl std::future::Future<Output = ()> + Send {
        async {}
    }

    fn handle(&mut self) -> impl std::future::Future<Output = ()> + Send;

    fn teardown(&mut self) -> impl std::future::Future<Output = ()> + Send {
        async {}
    }
}

pub struct Builder {
    topics: HashMap<String, String>,
    // Every port the module asked for a topic, matched against topics after build.
    requested: BTreeSet<String>,
    // One subscription per port, in the order the module asked for them.
    routes: Vec<(String, Dispatch)>,
    // Handed to every output so it publishes straight to the transport.
    transport: Arc<dyn Transport>,
    tf: Option<crate::tf::Tf>,
}

impl Builder {
    pub(crate) fn new(topics: HashMap<String, String>, transport: Arc<dyn Transport>) -> Self {
        Self {
            topics,
            requested: BTreeSet::new(),
            routes: Vec::new(),
            transport,
            tf: None,
        }
    }

    fn topic_for(&mut self, port: &str) -> String {
        self.requested.insert(port.to_string());
        self.topics
            .get(port)
            .cloned()
            .unwrap_or_else(|| format!("/{port}"))
    }

    // A mismatch is dead wiring: an unclaimed topic reaches no port, and an
    // unsent one leaves the port on a fallback name nothing else publishes to.
    pub(crate) fn enforce_topics_match_ports(&self) -> io::Result<()> {
        let provided: BTreeSet<&String> = self.topics.keys().collect();
        let requested: BTreeSet<&String> = self.requested.iter().collect();
        if provided == requested {
            return Ok(());
        }
        let missing: Vec<&&String> = requested.difference(&provided).collect();
        let unexpected: Vec<&&String> = provided.difference(&requested).collect();
        Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "topics do not match module ports: missing {missing:?}, unexpected {unexpected:?}"
            ),
        ))
    }

    fn add_route<T: Send + 'static>(
        &mut self,
        topic: &str,
        decode: fn(&[u8]) -> io::Result<T>,
    ) -> async_channel::Receiver<T> {
        let (tx, rx) = async_channel::bounded(INPUT_BUFFER_LEN);
        let route = TypedRoute {
            topic: topic.to_string(),
            decode,
            sender: tx,
        };
        self.routes.push((topic.to_string(), Arc::new(route)));
        rx
    }

    pub fn input<T: Send + 'static>(
        &mut self,
        port: &str,
        decode: fn(&[u8]) -> io::Result<T>,
    ) -> Input<T> {
        let topic = self.topic_for(port);
        let receiver = self.add_route(&topic, decode);
        Input { topic, receiver }
    }

    pub fn output<T>(&mut self, port: &str, encode: fn(&T) -> Vec<u8>) -> Output<T> {
        let topic = self.topic_for(port);
        Output {
            topic,
            encode,
            transport: Arc::clone(&self.transport),
        }
    }

    /// A port that both subscribes and publishes on one topic.
    pub fn io<T: Send + 'static>(
        &mut self,
        port: &str,
        decode: fn(&[u8]) -> io::Result<T>,
        encode: fn(&T) -> Vec<u8>,
    ) -> Io<T> {
        let topic = self.topic_for(port);
        let receiver = self.add_route(&topic, decode);
        Io {
            topic,
            receiver,
            encode,
            transport: Arc::clone(&self.transport),
        }
    }

    /// A handle that answers transform queries and publishes on the `tf` topic.
    ///
    /// The graph fills in the background as `tf` messages arrive. Repeated calls
    /// share one graph.
    pub fn tf(&mut self) -> crate::tf::Tf {
        if let Some(tf) = &self.tf {
            return tf.clone();
        }
        let topic = self.topic_for("tf");
        let (tf, route) = crate::tf::tf_subscription(
            topic.clone(),
            crate::tf::DEFAULT_TF_WINDOW_SECS,
            Arc::clone(&self.transport),
        );
        self.routes.push((topic, route));
        self.tf = Some(tf.clone());
        tf
    }
}

/// Subscribe each port's route on the transport. Two ports on one topic are
/// two subscriptions, the transport fans the topic out.
pub(crate) async fn subscribe_routes(
    transport: &dyn Transport,
    routes: Vec<(String, Dispatch)>,
) -> io::Result<()> {
    for (channel, route) in routes {
        transport.subscribe(&channel, route).await?;
    }
    Ok(())
}

/// Read the launch config the coordinator writes to stdin as one JSON line.
pub(crate) async fn read_launch_config() -> io::Result<serde_json::Value> {
    let mut line = String::new();
    BufReader::new(tokio::io::stdin())
        .read_line(&mut line)
        .await?;
    parse_launch_config(&line)
}

fn parse_launch_config(line: &str) -> io::Result<serde_json::Value> {
    serde_json::from_str(line.trim()).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}

/// Build, wire and run one module over an already-open transport until it
/// finishes or `shutdown` flips. Shared by `run_with_transport`, one module
/// per process, and by the baked host, which drives several of these on one
/// transport.
pub(crate) async fn run_module_core<M: Module>(
    transport: Arc<dyn Transport>,
    topics: HashMap<String, String>,
    config: M::Config,
    mut shutdown: watch::Receiver<bool>,
) -> io::Result<()> {
    let mut builder = Builder::new(topics, Arc::clone(&transport));
    let mut module = M::build(&mut builder, config);
    builder.enforce_topics_match_ports()?;

    subscribe_routes(transport.as_ref(), builder.routes).await?;

    module.setup().await;

    tokio::select! {
        _ = module.handle() => {}
        _ = shutdown.changed() => {}
    }

    module.teardown().await;

    Ok(())
}

/// Log the resolved wiring of a module, tagged with whatever the operator sees
/// in `ps`: the executable for a lone module, the module id inside a host.
pub(crate) fn log_wiring<C: Debug>(exe: &str, topics: &HashMap<String, String>, config: &C) {
    for (port, topic) in topics {
        info!(exe = %exe, port = %port, topic = %topic, "topic mapping");
    }
    info!(exe = %exe, config = ?config, "config loaded");
}

pub(crate) fn exe_name() -> String {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        .unwrap_or_else(|| "unknown".to_string())
}

#[cfg(unix)]
pub(crate) async fn shutdown_signal() -> io::Result<()> {
    use tokio::signal::unix::{signal, SignalKind};

    let mut terminate = signal(SignalKind::terminate())?;
    tokio::select! {
        result = tokio::signal::ctrl_c() => result,
        _ = terminate.recv() => Ok(()),
    }
}

#[cfg(not(unix))]
pub(crate) async fn shutdown_signal() -> io::Result<()> {
    tokio::signal::ctrl_c().await
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
    use std::sync::mpsc::{Receiver, Sender};
    use std::sync::{Arc, Mutex};
    use std::time::{Duration, Instant};
    use tokio::sync::Notify;

    /// Parse a raw config line the way `run_with_transport` does, for exercising
    /// `parse_config_value` from the string form the coordinator sends.
    fn parse_config_json<C: DeserializeOwned + Serialize>(
        line: &str,
    ) -> io::Result<(HashMap<String, String>, C)> {
        parse_config_value(&parse_launch_config(line)?)
    }

    #[test]
    fn an_empty_launch_line_is_an_error_not_a_hang() {
        // The module is spawned with a pipe, so EOF arrives as an empty line.
        for line in ["", "\n", "not json"] {
            let err = parse_launch_config(line).expect_err("empty line rejected");
            assert_eq!(err.kind(), io::ErrorKind::InvalidData);
        }
    }

    type Inbound = (String, Vec<u8>);

    /// Mock push transport for testing receive/publish concurrency.
    ///
    /// `subscribe` registers callbacks. One delivery thread drains an inbound
    /// queue into them, independent of the publish path, like the real
    /// transports do.
    struct ControllableMockTransport {
        inbound: Sender<Inbound>,
        inbound_rx: Mutex<Option<Receiver<Inbound>>>,
        subscriptions: Arc<Mutex<HashMap<String, Vec<Dispatch>>>>,
        listening: Arc<AtomicBool>,
        publish_delay_ms: Arc<AtomicU64>,
        publish_entered: Arc<Notify>,
        publish_log: Arc<Mutex<Vec<Instant>>>,
        dispatch_entered: Arc<Notify>,
        dispatch_log: Arc<Mutex<Vec<Instant>>>,
    }

    impl ControllableMockTransport {
        fn new() -> Self {
            let (inbound, inbound_rx) = std::sync::mpsc::channel();
            Self {
                inbound,
                inbound_rx: Mutex::new(Some(inbound_rx)),
                subscriptions: Arc::new(Mutex::new(HashMap::new())),
                listening: Arc::new(AtomicBool::new(false)),
                publish_delay_ms: Arc::new(AtomicU64::new(0)),
                publish_entered: Arc::new(Notify::new()),
                publish_log: Arc::new(Mutex::new(Vec::new())),
                dispatch_entered: Arc::new(Notify::new()),
                dispatch_log: Arc::new(Mutex::new(Vec::new())),
            }
        }

        fn spawn_delivery_loop(&self) {
            let inbound_rx = self.inbound_rx.lock().unwrap().take().unwrap();
            let subscriptions = Arc::clone(&self.subscriptions);
            let dispatch_entered = Arc::clone(&self.dispatch_entered);
            let dispatch_log = Arc::clone(&self.dispatch_log);
            std::thread::spawn(move || {
                for (channel, data) in inbound_rx {
                    dispatch_entered.notify_one();
                    let callbacks = subscriptions.lock().unwrap().get(&channel).cloned();
                    if let Some(callbacks) = callbacks {
                        for cb in &callbacks {
                            cb.deliver(&data);
                        }
                    }
                    dispatch_log.lock().unwrap().push(Instant::now());
                }
            });
        }
    }

    #[async_trait::async_trait]
    impl crate::transport::Transport for ControllableMockTransport {
        async fn publish(&self, _channel: &str, _data: Vec<u8>) -> io::Result<()> {
            self.publish_entered.notify_one();
            let delay = self.publish_delay_ms.load(Ordering::Relaxed);
            if delay > 0 {
                tokio::time::sleep(Duration::from_millis(delay)).await;
            }
            self.publish_log.lock().unwrap().push(Instant::now());
            Ok(())
        }

        async fn subscribe(&self, channel: &str, on_msg: Dispatch) -> io::Result<()> {
            self.subscriptions
                .lock()
                .unwrap()
                .entry(channel.to_string())
                .or_default()
                .push(on_msg);
            if !self.listening.swap(true, Ordering::SeqCst) {
                self.spawn_delivery_loop();
            }
            Ok(())
        }
    }

    fn inject_inbound(inbound: &Sender<Inbound>, channel: &str, data: Vec<u8>) {
        inbound.send((channel.to_string(), data)).unwrap();
    }

    async fn wait_for(what: &str, mut cond: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !cond() {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    #[derive(Debug, Deserialize, Serialize, Default, PartialEq)]
    #[serde(deny_unknown_fields)]
    struct TestConfig {
        value: i64,
        name: String,
    }

    // parse_config_json
    #[test]
    fn parses_topics_and_config() {
        let json = r#"{"topics": {"data": "/foo/data", "confirm": "/foo/confirm"}, "config": {"value": 42, "name": "hello"}}"#;
        let (topics, config) = parse_config_json::<TestConfig>(json).unwrap();
        assert_eq!(topics["data"], "/foo/data");
        assert_eq!(topics["confirm"], "/foo/confirm");
        assert_eq!(
            config,
            TestConfig {
                value: 42,
                name: "hello".into()
            }
        );
    }

    /// A config dict whose keys mean something in order (a camera rig, a pipeline) is only
    /// possible while serde_json carries the order it read.
    #[test]
    fn a_config_object_keeps_the_key_order_it_was_written_in() {
        let json = r#"{"topics": {}, "config": {"zeta": 1, "alpha": 2}}"#;
        let value: serde_json::Value = serde_json::from_str(json).unwrap();
        let keys: Vec<&str> = value["config"]
            .as_object()
            .unwrap()
            .keys()
            .map(String::as_str)
            .collect();
        assert_eq!(keys, ["zeta", "alpha"]);
    }

    #[test]
    fn missing_config_field_returns_error() {
        let json = r#"{"topics": {"data": "/foo/data"}}"#;
        let result = parse_config_json::<TestConfig>(json);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("missing 'config' field"));
    }

    #[test]
    fn null_config_succeeds_for_unit_type() {
        let json = r#"{"topics": {}, "config": null}"#;
        let (_topics, _config) = parse_config_json::<()>(json).unwrap();
    }

    #[test]
    fn null_config_errors_when_struct_expects_fields() {
        let json = r#"{"topics": {}, "config": null}"#;
        let result = parse_config_json::<TestConfig>(json);
        assert!(result.is_err());
    }

    #[test]
    fn empty_config_object_errors_when_struct_expects_fields() {
        let json = r#"{"topics": {}, "config": {}}"#;
        let result = parse_config_json::<TestConfig>(json);
        assert!(result.is_err());
    }

    #[test]
    fn config_with_wrong_type_returns_error() {
        let json = r#"{"topics": {}, "config": {"value": "not_a_number", "name": "x"}}"#;
        let result = parse_config_json::<TestConfig>(json);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("failed to deserialize config"));
    }

    #[test]
    fn missing_topics_field_gives_empty_map() {
        let json = r#"{"config": {"value": 1, "name": "x"}}"#;
        let (topics, _config) = parse_config_json::<TestConfig>(json).unwrap();
        assert!(topics.is_empty());
    }

    #[test]
    fn malformed_json_returns_error() {
        let result = parse_config_json::<()>("not json at all");
        assert!(result.is_err());
    }

    #[test]
    fn unknown_config_field_returns_error() {
        let json = r#"{"topics": {}, "config": {"value": 1, "name": "x", "unexpected": true}}"#;
        let result = parse_config_json::<TestConfig>(json);
        assert!(result.is_err());
    }

    // one-to-one key check: serde alone would accept a missing Option field as None.

    #[derive(Debug, Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct OptionalConfig {
        required: i64,
        maybe: Option<i64>,
    }

    type MaybeI = Option<i64>;

    #[derive(Debug, Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct AliasedConfig {
        required: i64,
        maybe: MaybeI,
    }

    #[test]
    fn missing_optional_field_is_rejected() {
        let json = r#"{"config": {"required": 1}}"#;
        let err = parse_config_json::<OptionalConfig>(json)
            .expect_err("a missing Option field must be rejected, not defaulted to None");
        assert!(err.to_string().contains("maybe"), "{err}");
    }

    #[test]
    fn missing_aliased_option_field_is_rejected() {
        let json = r#"{"config": {"required": 1}}"#;
        assert!(parse_config_json::<AliasedConfig>(json).is_err());
    }

    #[test]
    fn optional_field_sent_explicitly_succeeds() {
        let json = r#"{"config": {"required": 1, "maybe": null}}"#;
        let (_topics, config) = parse_config_json::<OptionalConfig>(json).unwrap();
        assert_eq!(config.maybe, None);
    }

    // validate_config

    #[derive(Debug, Deserialize, Validate)]
    struct RangedConfig {
        #[validate(range(min = 1, max = 10))]
        value: i64,
    }

    #[test]
    fn validate_config_passes_when_in_range() {
        let cfg = RangedConfig { value: 5 };
        assert!(validate_config(&cfg).is_ok());
    }

    #[test]
    fn validate_config_returns_invalid_data_when_out_of_range() {
        let cfg = RangedConfig { value: 0 };
        let err = validate_config(&cfg).expect_err("expected validation failure");
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
        let msg = err.to_string();
        assert!(msg.contains("value"), "error should name the field: {msg}");
        assert!(
            msg.contains("config validation failed"),
            "error should be framed: {msg}",
        );
    }

    #[test]
    fn empty_config_validates() {
        assert!(validate_config(&crate::module::NoConfig).is_ok());
    }

    // topic_for fallback

    fn topics(pairs: &[(&str, &str)]) -> HashMap<String, String> {
        pairs
            .iter()
            .map(|(p, t)| (p.to_string(), t.to_string()))
            .collect()
    }

    /// Records every publish so a test can see the topic and bytes a port sent.
    #[derive(Default)]
    struct RecordingTransport {
        published: Mutex<Vec<(String, Vec<u8>)>>,
    }

    #[async_trait::async_trait]
    impl crate::transport::Transport for RecordingTransport {
        async fn publish(&self, channel: &str, data: Vec<u8>) -> io::Result<()> {
            self.published
                .lock()
                .unwrap()
                .push((channel.to_string(), data));
            Ok(())
        }

        async fn subscribe(&self, _channel: &str, _on_msg: Dispatch) -> io::Result<()> {
            Ok(())
        }
    }

    fn recording_builder(pairs: &[(&str, &str)]) -> (Builder, Arc<RecordingTransport>) {
        let transport = Arc::new(RecordingTransport::default());
        (Builder::new(topics(pairs), transport.clone()), transport)
    }

    fn builder_with_topics(pairs: &[(&str, &str)]) -> Builder {
        recording_builder(pairs).0
    }

    #[test]
    fn unmapped_port_falls_back_to_slash_port() {
        let mut builder = builder_with_topics(&[]);
        assert_eq!(builder.topic_for("cmd_vel"), "/cmd_vel");
    }

    #[test]
    fn mapped_port_uses_given_topic() {
        let mut builder = builder_with_topics(&[("cmd_vel", "/robot/cmd_vel")]);
        assert_eq!(builder.topic_for("cmd_vel"), "/robot/cmd_vel");
    }

    #[test]
    fn input_uses_mapped_topic() {
        let mut builder = builder_with_topics(&[("data", "/test/data")]);
        let input = builder.input("data", |b| Ok(b.to_vec()));
        assert_eq!(input.topic, "/test/data");
    }

    #[test]
    fn input_falls_back_to_slash_port_when_unmapped() {
        let mut builder = builder_with_topics(&[]);
        let input = builder.input("data", |b| Ok(b.to_vec()));
        assert_eq!(input.topic, "/data");
    }

    #[test]
    fn output_uses_mapped_topic() {
        let mut builder = builder_with_topics(&[("cmd_vel", "/robot/cmd_vel")]);
        let output = builder.output("cmd_vel", |b: &Vec<u8>| b.clone());
        assert_eq!(output.topic, "/robot/cmd_vel");
    }

    #[test]
    fn topics_matching_ports_exactly_pass() {
        let mut builder = builder_with_topics(&[("cmd", "/robot/cmd"), ("odom", "/robot/odom")]);
        builder.input("cmd", |b| Ok(b.to_vec()));
        builder.output("odom", |b: &Vec<u8>| b.clone());
        builder.enforce_topics_match_ports().expect("exact match");
    }

    #[test]
    fn a_port_the_coordinator_never_sent_is_rejected() {
        let mut builder = builder_with_topics(&[("cmd", "/robot/cmd")]);
        builder.input("cmd", |b| Ok(b.to_vec()));
        builder.output("odom", |b: &Vec<u8>| b.clone());
        let err = builder
            .enforce_topics_match_ports()
            .expect_err("odom has no topic");
        assert!(err.to_string().contains("missing [\"odom\"]"), "{err}");
    }

    #[test]
    fn a_topic_no_port_claimed_is_rejected() {
        let mut builder = builder_with_topics(&[("cmd", "/robot/cmd"), ("stale", "/robot/stale")]);
        builder.input("cmd", |b| Ok(b.to_vec()));
        let err = builder
            .enforce_topics_match_ports()
            .expect_err("stale is unclaimed");
        assert!(err.to_string().contains("unexpected [\"stale\"]"), "{err}");
    }

    #[test]
    fn a_tf_field_claims_the_tf_topic() {
        let mut builder = builder_with_topics(&[("tf", "/tf#tf2_msgs.TFMessage")]);
        builder.tf();
        builder.enforce_topics_match_ports().expect("tf claimed");
    }

    #[test]
    fn a_module_with_no_ports_and_no_topics_passes() {
        let builder = builder_with_topics(&[]);
        builder
            .enforce_topics_match_ports()
            .expect("nothing to match");
    }

    #[test]
    fn io_uses_mapped_topic() {
        let mut builder = builder_with_topics(&[("cmd", "/robot/cmd")]);
        let io = builder.io("cmd", |b| Ok(b.to_vec()), |b: &Vec<u8>| b.clone());
        assert_eq!(io.topic, "/robot/cmd");
    }

    #[test]
    fn io_registers_one_route_on_its_topic() {
        let mut builder = builder_with_topics(&[("cmd", "/robot/cmd")]);
        let _io = builder.io("cmd", |b| Ok(b.to_vec()), |b: &Vec<u8>| b.clone());
        assert_eq!(builder.routes.len(), 1);
        assert_eq!(builder.routes[0].0, "/robot/cmd");
    }

    #[tokio::test]
    async fn io_receives_on_its_route_and_publishes_to_the_transport() {
        let (mut builder, transport) = recording_builder(&[("cmd", "/robot/cmd")]);
        let mut io = builder.io("cmd", |b| Ok(b.to_vec()), |b: &Vec<u8>| b.clone());

        builder.routes[0].1.deliver(b"inbound");
        assert_eq!(io.recv().await.expect("inbound message"), b"inbound");

        io.publish(&b"outbound".to_vec()).await.expect("publish");
        assert_eq!(
            transport.published.lock().unwrap().as_slice(),
            [("/robot/cmd".to_string(), b"outbound".to_vec())]
        );
    }

    #[tokio::test]
    async fn tf_uses_mapped_topic() {
        let (mut builder, transport) = recording_builder(&[("tf", "/robot/tf")]);
        let tf = builder.tf();
        assert_eq!(builder.routes[0].0, "/robot/tf");

        let identity = nalgebra::Isometry3::identity();
        tf.publish(&[crate::Transform::new("a", "b", 1.0, identity)])
            .await
            .expect("publish");
        assert_eq!(transport.published.lock().unwrap()[0].0, "/robot/tf");
    }

    #[test]
    fn repeated_tf_calls_share_one_graph() {
        let mut builder = builder_with_topics(&[("tf", "/tf")]);
        builder.tf();
        builder.tf();
        assert_eq!(builder.routes.len(), 1);
    }

    // recv/publish concurrency

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn slow_publish_does_not_block_dispatch() {
        let transport = Arc::new(ControllableMockTransport::new());
        let dispatch_log = transport.dispatch_log.clone();
        let publish_log = transport.publish_log.clone();
        let inbound = transport.inbound.clone();
        let publish_delay_ms = transport.publish_delay_ms.clone();
        let publish_entered = transport.publish_entered.clone();

        // set publishing to take 200ms
        publish_delay_ms.store(200, Ordering::Relaxed);

        let mut builder = Builder::new(
            topics(&[("data", "/data"), ("out", "/out")]),
            transport.clone(),
        );
        let _input = builder.input("data", |b| Ok(b.to_vec()));
        let output = builder.output("out", |b: &Vec<u8>| b.clone());

        subscribe_routes(transport.as_ref(), builder.routes)
            .await
            .unwrap();

        // start the 200ms publish on a handler task
        let publishing = tokio::spawn(async move { output.publish(&vec![0u8]).await });

        // ensure the publish is in the transport before we deliver
        tokio::time::timeout(Duration::from_secs(1), publish_entered.notified())
            .await
            .expect("publish should reach the transport within 1s");

        inject_inbound(&inbound, "/data", vec![42u8]);

        wait_for("dispatch to fire and publish to complete", || {
            !dispatch_log.lock().unwrap().is_empty() && !publish_log.lock().unwrap().is_empty()
        })
        .await;

        let dispatch_time = dispatch_log.lock().unwrap()[0];
        let publish_time = publish_log.lock().unwrap()[0];
        assert!(
            dispatch_time < publish_time,
            "expected dispatch to fire during the slow publish, not after it. \
             The receive path should be independent of publish latency."
        );
        publishing.await.unwrap().unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn slow_dispatch_does_not_block_publish() {
        let transport = Arc::new(ControllableMockTransport::new());
        let publish_log = transport.publish_log.clone();
        let inbound = transport.inbound.clone();
        let dispatch_entered = transport.dispatch_entered.clone();

        let mut builder = Builder::new(
            topics(&[("slow", "/slow"), ("out", "/out")]),
            transport.clone(),
        );

        // block the delivery loop in decode until the test releases it
        static RECV_RELEASE: AtomicBool = AtomicBool::new(false);
        RECV_RELEASE.store(false, Ordering::SeqCst);
        let _input = builder.input("slow", |b| {
            let deadline = Instant::now() + Duration::from_secs(5);
            while !RECV_RELEASE.load(Ordering::SeqCst) && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(1));
            }
            Ok(b.to_vec())
        });
        let output = builder.output("out", |b: &Vec<u8>| b.clone());

        subscribe_routes(transport.as_ref(), builder.routes)
            .await
            .unwrap();

        // send a message to the receiver
        inject_inbound(&inbound, "/slow", vec![1u8]);

        // make sure dispatch begins and blocks in decode before we publish
        tokio::time::timeout(Duration::from_secs(1), dispatch_entered.notified())
            .await
            .expect("delivery loop should pick up inbound within 1s");

        output.publish(&vec![42u8]).await.ok();

        // publish must complete while the delivery loop stays blocked in decode
        wait_for("publish to complete while dispatch is blocked", || {
            !publish_log.lock().unwrap().is_empty()
        })
        .await;

        // release the blocked decode so the runtime can shut down
        RECV_RELEASE.store(true, Ordering::SeqCst);
    }

    // input backpressure

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn full_input_blocks_dispatch_instead_of_dropping() {
        let transport = Arc::new(ControllableMockTransport::new());
        let inbound = transport.inbound.clone();
        let dispatch_log = transport.dispatch_log.clone();

        let mut builder = Builder::new(topics(&[("data", "/data")]), transport.clone());
        let mut input = builder.input("data", |b| Ok(b.to_vec()));
        subscribe_routes(transport.as_ref(), builder.routes)
            .await
            .unwrap();

        // Nothing drains the input yet, as if the handler never returned.
        let total = INPUT_BUFFER_LEN + 4;
        for i in 0..total {
            inject_inbound(&inbound, "/data", vec![i as u8]);
        }

        wait_for("the input buffer to fill", || {
            dispatch_log.lock().unwrap().len() == INPUT_BUFFER_LEN
        })
        .await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(
            dispatch_log.lock().unwrap().len(),
            INPUT_BUFFER_LEN,
            "delivery must block on a full input, not drop"
        );

        for i in 0..total {
            let msg = tokio::time::timeout(Duration::from_secs(1), input.recv())
                .await
                .expect("delivery resumes once the handler drains")
                .expect("input open");
            assert_eq!(msg, vec![i as u8]);
        }
        wait_for("delivery to finish", || {
            dispatch_log.lock().unwrap().len() == total
        })
        .await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn full_input_suspends_async_delivery() {
        let mut builder = builder_with_topics(&[("data", "/data")]);
        let mut input = builder.input("data", |b| Ok(b.to_vec()));
        let (_, route) = builder.routes.pop().unwrap();

        for i in 0..INPUT_BUFFER_LEN {
            route.deliver_async(&[i as u8]).await;
        }
        let blocked = tokio::spawn({
            let route = Arc::clone(&route);
            async move { route.deliver_async(&[INPUT_BUFFER_LEN as u8]).await }
        });
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(
            !blocked.is_finished(),
            "delivery must suspend on a full input, not drop"
        );

        assert_eq!(input.recv().await.unwrap(), vec![0u8]);
        tokio::time::timeout(Duration::from_secs(1), blocked)
            .await
            .expect("delivery resumes once the handler drains")
            .unwrap();
        for i in 1..=INPUT_BUFFER_LEN {
            assert_eq!(input.recv().await.unwrap(), vec![i as u8]);
        }
    }

    // Exercises the code #[derive(Module)] generates for an #[io] field.
    mod derive_io {
        use super::*;
        use crate::Io;

        struct Msg(Vec<u8>);

        fn decode(bytes: &[u8]) -> io::Result<Msg> {
            Ok(Msg(bytes.to_vec()))
        }

        fn encode(msg: &Msg) -> Vec<u8> {
            msg.0.clone()
        }

        #[derive(crate::Module)]
        struct Echo {
            #[io(decode = decode, encode = encode)]
            cmd: Io<Msg>,
        }

        impl Echo {
            async fn handle_cmd(&mut self, msg: Msg) {
                if msg.0 == b"ping" {
                    self.cmd
                        .publish(&Msg(b"pong".to_vec()))
                        .await
                        .expect("publish");
                }
            }
        }

        #[tokio::test]
        async fn io_field_is_wired_to_its_handler_and_can_publish() {
            let (mut builder, transport) = recording_builder(&[("cmd", "/robot/cmd")]);
            let mut echo = Echo::build(&mut builder, NoConfig);

            builder.routes[0].1.deliver(b"ping");
            // Dropping the routes closes the sender, so handle() drains and returns.
            builder.routes.clear();
            echo.handle().await;

            assert_eq!(
                transport.published.lock().unwrap().as_slice(),
                [("/robot/cmd".to_string(), b"pong".to_vec())]
            );
        }
    }
}
