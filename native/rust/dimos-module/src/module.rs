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
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::mpsc;
use tracing::{error, info, warn};
use tracing_subscriber::EnvFilter;

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use validator::Validate;

use crate::transport::Transport;

/// Marker trait for a config checked by `#[native_config]`: every field required,
/// no Rust-side defaults, no unknown fields. Implemented only by the macro.
pub trait NativeConfig {}

/// Trait required by `Module::Config`s to ensure that configurations are
/// validated correctly.
pub trait ModuleConfig: DeserializeOwned + Serialize + Debug + Validate + NativeConfig {}
impl<T: DeserializeOwned + Serialize + Debug + Validate + NativeConfig> ModuleConfig for T {}

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

fn init_tracing() {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .json()
        .with_writer(std::io::stderr)
        .with_env_filter(filter)
        .try_init();
}

const INPUT_CHANNEL_CAPACITY: usize = 1024;
const PUBLISH_CHANNEL_CAPACITY: usize = 1024;

// Each input() call produces a TypedRoute that decodes its message type
// and forwards it to the right Input's mpsc channel.
pub(crate) trait Route: Send {
    fn try_dispatch(&self, data: &[u8]);
}

struct TypedRoute<T: Send + 'static> {
    topic: String,
    decode: fn(&[u8]) -> io::Result<T>,
    sender: mpsc::Sender<T>,
    drop_count: AtomicU64,
    last_log_ns: AtomicU64,
}

impl<T: Send + 'static> Route for TypedRoute<T> {
    fn try_dispatch(&self, data: &[u8]) {
        match (self.decode)(data) {
            Ok(msg) => match self.sender.try_send(msg) {
                Ok(()) => {}
                Err(mpsc::error::TrySendError::Full(_)) => {
                    // throttle the warning logging per route
                    // we can't use warn_throttled! because this code is shared across all route instances
                    let n = self.drop_count.fetch_add(1, Ordering::Relaxed) + 1;
                    if crate::log::check_and_record(
                        &self.last_log_ns,
                        Duration::from_secs(1).as_nanos() as u64,
                    ) {
                        warn!(
                            topic = %self.topic,
                            dropped = n,
                            queue_cap = INPUT_CHANNEL_CAPACITY,
                            "Dispatcher could not send message because handler was full.",
                        );
                    }
                }
                Err(mpsc::error::TrySendError::Closed(_)) => {}
            },
            Err(e) => error!(topic = %self.topic, error = %e, "decode error"),
        }
    }
}
pub struct Input<T> {
    pub topic: String,
    receiver: mpsc::Receiver<T>,
}

impl<T> Input<T> {
    pub async fn recv(&mut self) -> Option<T> {
        self.receiver.recv().await
    }
}

#[derive(Clone)]
pub struct Output<T> {
    pub topic: String,
    encode: fn(&T) -> Vec<u8>,
    sender: mpsc::Sender<(String, Vec<u8>)>,
}

impl<T> Output<T> {
    pub async fn publish(&self, msg: &T) -> io::Result<()> {
        let data = (self.encode)(msg);
        self.sender
            .send((self.topic.clone(), data))
            .await
            .map_err(|_| io::Error::new(io::ErrorKind::BrokenPipe, "background task gone"))
    }
}

/// Parse a JSON config line as written by the Python NativeModule coordinator.
/// Returns `(topics, config)`. Extracted so it can be unit-tested without stdin.
fn parse_config_json<C: DeserializeOwned + Serialize>(
    line: &str,
) -> io::Result<(HashMap<String, String>, C)> {
    let json: serde_json::Value = serde_json::from_str(line.trim())
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

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
            "missing 'config' field in stdin JSON — coordinator must always send a config object",
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

fn validate_config<C: Validate>(config: &C) -> io::Result<()> {
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
    routes: HashMap<String, Vec<Box<dyn Route>>>,
    publish_tx: mpsc::Sender<(String, Vec<u8>)>,
}

impl Builder {
    pub(crate) fn new(
        topics: HashMap<String, String>,
        publish_tx: mpsc::Sender<(String, Vec<u8>)>,
    ) -> Self {
        Self {
            topics,
            routes: HashMap::new(),
            publish_tx,
        }
    }

    fn topic_for(&self, port: &str) -> String {
        self.topics
            .get(port)
            .cloned()
            .unwrap_or_else(|| format!("/{port}"))
    }

    pub fn input<T: Send + 'static>(
        &mut self,
        port: &str,
        decode: fn(&[u8]) -> io::Result<T>,
    ) -> Input<T> {
        let topic = self.topic_for(port);
        let (tx, rx) = mpsc::channel(INPUT_CHANNEL_CAPACITY);
        self.routes
            .entry(topic.clone())
            .or_default()
            .push(Box::new(TypedRoute {
                topic: topic.clone(),
                decode,
                sender: tx,
                drop_count: AtomicU64::new(0),
                last_log_ns: AtomicU64::new(0),
            }));
        Input {
            topic,
            receiver: rx,
        }
    }

    pub fn output<T>(&self, port: &str, encode: fn(&T) -> Vec<u8>) -> Output<T> {
        Output {
            topic: self.topic_for(port),
            encode,
            sender: self.publish_tx.clone(),
        }
    }
}

pub(crate) fn spawn_pubsub_tasks<T: Transport>(
    transport: T,
    routes: HashMap<String, Vec<Box<dyn Route>>>,
    mut publish_rx: mpsc::Receiver<(String, Vec<u8>)>,
) -> (tokio::task::JoinHandle<()>, tokio::task::JoinHandle<()>) {
    let transport = Arc::new(transport);

    let recv_transport = Arc::clone(&transport);
    let recv_handle = tokio::spawn(async move {
        loop {
            match recv_transport.recv().await {
                Ok((channel, data)) => {
                    if let Some(rs) = routes.get(&channel) {
                        for route in rs {
                            route.try_dispatch(&data);
                        }
                    }
                }
                Err(e) => error!(error = %e, "recv error"),
            }
        }
    });

    let pub_transport = Arc::clone(&transport);
    let pub_handle = tokio::spawn(async move {
        while let Some((topic, data)) = publish_rx.recv().await {
            if let Err(e) = pub_transport.publish(&topic, &data).await {
                error!(topic = %topic, error = %e, "publish error");
            }
        }
    });

    (recv_handle, pub_handle)
}

fn propagate_task_failure(name: &str, res: Result<(), tokio::task::JoinError>) {
    match res {
        Ok(()) => error!(task = name, "task exited unexpectedly"),
        Err(e) => {
            error!(task = name, "task panicked, propagating");
            std::panic::resume_unwind(e.into_panic());
        }
    }
}

pub async fn run<M, T>(transport: T)
where
    M: Module,
    T: Transport,
{
    if let Err(e) = run_fallible::<M, T>(transport).await {
        error!("{e}");
        std::process::exit(1);
    }
}

async fn run_fallible<M, T>(transport: T) -> io::Result<()>
where
    M: Module,
    T: Transport,
{
    init_tracing();

    let mut line = String::new();
    BufReader::new(tokio::io::stdin())
        .read_line(&mut line)
        .await?;
    let (topics, config) = parse_config_json::<M::Config>(&line)?;
    validate_config(&config)?;

    let exe = std::env::current_exe()
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        .unwrap_or_else(|| "unknown".to_string());
    for (port, topic) in &topics {
        info!(exe = %exe, port = %port, topic = %topic, "topic mapping");
    }
    info!(exe = %exe, config = ?config, "config loaded");

    let (publish_tx, publish_rx) = mpsc::channel::<(String, Vec<u8>)>(PUBLISH_CHANNEL_CAPACITY);
    let mut builder = Builder::new(topics, publish_tx);
    let mut module = M::build(&mut builder, config);
    let (mut recv_handle, mut pub_handle) =
        spawn_pubsub_tasks(transport, builder.routes, publish_rx);

    module.setup().await;

    // record whatever resolves first, then teardown unconditionally
    let failure = tokio::select! {
        _ = module.handle() => None,
        _ = tokio::signal::ctrl_c() => None,
        res = &mut recv_handle => Some(("recv", res)),
        res = &mut pub_handle => Some(("publish", res)),
    };

    module.teardown().await;

    // if the result was an error, handle it here
    if let Some((name, res)) = failure {
        propagate_task_failure(name, res);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
    use std::sync::{Arc, Mutex};
    use std::time::{Duration, Instant};
    use tokio::sync::Notify;

    type InboundQueue = Mutex<VecDeque<(String, Vec<u8>)>>;

    /// Mock transport for testing message timing.
    ///
    /// Lets us test for concurrency and blocking when handling different messages.
    struct ControllableMockTransport {
        inbound: Arc<InboundQueue>,
        inbound_notify: Arc<Notify>,
        publish_delay_ms: Arc<AtomicU64>,
        publish_entered: Arc<Notify>,
        recv_returned: Arc<Notify>,
        recv_log: Arc<Mutex<Vec<Instant>>>,
        publish_log: Arc<Mutex<Vec<Instant>>>,
    }

    impl ControllableMockTransport {
        fn new() -> Self {
            Self {
                inbound: Arc::new(InboundQueue::new(VecDeque::new())),
                inbound_notify: Arc::new(Notify::new()),
                publish_delay_ms: Arc::new(AtomicU64::new(0)),
                publish_entered: Arc::new(Notify::new()),
                recv_returned: Arc::new(Notify::new()),
                recv_log: Arc::new(Mutex::new(Vec::new())),
                publish_log: Arc::new(Mutex::new(Vec::new())),
            }
        }
    }

    impl crate::transport::Transport for ControllableMockTransport {
        async fn publish(&self, _channel: &str, _data: &[u8]) -> io::Result<()> {
            self.publish_entered.notify_one();
            let delay = self.publish_delay_ms.load(Ordering::Relaxed);
            if delay > 0 {
                tokio::time::sleep(Duration::from_millis(delay)).await;
            }
            self.publish_log.lock().unwrap().push(Instant::now());
            Ok(())
        }

        async fn recv(&self) -> io::Result<(String, Vec<u8>)> {
            loop {
                let popped = self.inbound.lock().unwrap().pop_front();
                if let Some(msg) = popped {
                    self.recv_log.lock().unwrap().push(Instant::now());
                    self.recv_returned.notify_one();
                    return Ok(msg);
                }
                self.inbound_notify.notified().await;
            }
        }
    }

    fn inject_inbound(inbound: &InboundQueue, notify: &Notify, channel: &str, data: Vec<u8>) {
        inbound
            .lock()
            .unwrap()
            .push_back((channel.to_string(), data));
        notify.notify_one();
    }

    async fn wait_for(what: &str, mut cond: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(1);
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

    fn builder_with_topics(pairs: &[(&str, &str)]) -> Builder {
        let (publish_tx, _) = mpsc::channel(PUBLISH_CHANNEL_CAPACITY);
        Builder::new(topics(pairs), publish_tx)
    }

    #[test]
    fn unmapped_port_falls_back_to_slash_port() {
        let builder = builder_with_topics(&[]);
        assert_eq!(builder.topic_for("cmd_vel"), "/cmd_vel");
    }

    #[test]
    fn mapped_port_uses_given_topic() {
        let builder = builder_with_topics(&[("cmd_vel", "/robot/cmd_vel")]);
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
        let builder = builder_with_topics(&[("cmd_vel", "/robot/cmd_vel")]);
        let output = builder.output("cmd_vel", |b: &Vec<u8>| b.clone());
        assert_eq!(output.topic, "/robot/cmd_vel");
    }

    // recv/publish concurrency

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn slow_publish_does_not_block_recv() {
        let transport = ControllableMockTransport::new();
        let recv_log = transport.recv_log.clone();
        let publish_log = transport.publish_log.clone();
        let inbound = transport.inbound.clone();
        let inbound_notify = transport.inbound_notify.clone();
        let publish_delay_ms = transport.publish_delay_ms.clone();
        let publish_entered = transport.publish_entered.clone();

        // set publishing to take 200ms
        publish_delay_ms.store(200, Ordering::Relaxed);

        let (publish_tx, publish_rx) = mpsc::channel(PUBLISH_CHANNEL_CAPACITY);
        let mut builder = Builder::new(topics(&[("data", "/data"), ("out", "/out")]), publish_tx);
        let _input = builder.input("data", |b| Ok(b.to_vec()));
        let output = builder.output("out", |b: &Vec<u8>| b.clone());
        spawn_pubsub_tasks(transport, builder.routes, publish_rx);

        // start the 200ms publish
        output.publish(&vec![0u8]).await.ok();

        // ensure the publish starts getting handled before the receive
        tokio::time::timeout(Duration::from_secs(1), publish_entered.notified())
            .await
            .expect("dispatch task should pick up publish_rx within 1s");

        inject_inbound(&inbound, &inbound_notify, "/data", vec![42u8]);

        wait_for("recv to fire and publish to complete", || {
            !recv_log.lock().unwrap().is_empty() && !publish_log.lock().unwrap().is_empty()
        })
        .await;

        let recv_time = recv_log.lock().unwrap()[0];
        let publish_time = publish_log.lock().unwrap()[0];
        assert!(
            recv_time < publish_time,
            "expected recv to fire during the slow publish, not after it. \
             The recv path should be independent of publish latency."
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn slow_recv_dispatch_does_not_block_publish() {
        let transport = ControllableMockTransport::new();
        let publish_log = transport.publish_log.clone();
        let inbound = transport.inbound.clone();
        let inbound_notify = transport.inbound_notify.clone();
        let recv_returned = transport.recv_returned.clone();

        let (publish_tx, publish_rx) = mpsc::channel(PUBLISH_CHANNEL_CAPACITY);
        let mut builder = Builder::new(topics(&[("slow", "/slow"), ("out", "/out")]), publish_tx);

        // block the recv worker until the test releases it
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
        spawn_pubsub_tasks(transport, builder.routes, publish_rx);

        // send a message to the receiving
        inject_inbound(&inbound, &inbound_notify, "/slow", vec![1u8]);

        // make sure the receive gets picked up before we publish
        tokio::time::timeout(Duration::from_secs(1), recv_returned.notified())
            .await
            .expect("dispatch task should pick up inbound within 1s");

        output.publish(&vec![42u8]).await.ok();

        // publish must complete while the recv worker stays blocked
        wait_for("publish to complete while recv dispatch is blocked", || {
            !publish_log.lock().unwrap().is_empty()
        })
        .await;

        // release the blocked decode so the runtime can shut down
        RECV_RELEASE.store(true, Ordering::SeqCst);
    }

    // propagate_task_failure

    #[tokio::test]
    async fn propagates_task_panic_payload() {
        let handle = tokio::spawn(async { panic!("kaboom") });
        let res = handle.await;

        let caught = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            propagate_task_failure("recv", res);
        }));

        let payload = caught.expect_err("expected helper to re-panic");
        let msg = payload
            .downcast_ref::<&'static str>()
            .copied()
            .expect("panic payload should be a string literal");
        assert_eq!(msg, "kaboom");
    }

    #[test]
    fn ok_does_not_panic() {
        propagate_task_failure("recv", Ok(()));
    }

    #[test]
    #[tracing_test::traced_test]
    fn typed_route_warns_and_counts_on_drop() {
        let (tx, _rx) = mpsc::channel::<Vec<u8>>(1);
        let route = TypedRoute {
            topic: "/test".to_string(),
            decode: |b| Ok(b.to_vec()),
            sender: tx,
            drop_count: AtomicU64::new(0),
            last_log_ns: AtomicU64::new(0),
        };
        route.try_dispatch(&[1u8]); // fill queue
        route.try_dispatch(&[1u8]); // now we warn
        assert_eq!(route.drop_count.load(Ordering::Relaxed), 1);
        assert!(logs_contain("handler was full"));
    }
}
