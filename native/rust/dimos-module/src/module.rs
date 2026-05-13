use std::collections::HashMap;
use std::fmt::Debug;
use std::io;
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::mpsc;

use serde::de::DeserializeOwned;

use crate::transport::Transport;

const INPUT_CHANNEL_CAPACITY: usize = 16;
const PUBLISH_CHANNEL_CAPACITY: usize = 64;

// Each input() call produces a TypedRoute that decodes its message type
// and forwards it to the right Input's mpsc channel.
pub(crate) trait Route: Send {
    fn try_dispatch(&self, data: &[u8]);
}

struct TypedRoute<T: Send + 'static> {
    topic: String,
    decode: fn(&[u8]) -> io::Result<T>,
    sender: mpsc::Sender<T>,
}

impl<T: Send + 'static> Route for TypedRoute<T> {
    fn try_dispatch(&self, data: &[u8]) {
        match (self.decode)(data) {
            // If the input channel is full, the newest message is dropped.
            Ok(msg) => {
                let _ = self.sender.try_send(msg);
            }
            Err(e) => eprintln!("dimos_module: decode error on {}: {e}", self.topic),
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
fn parse_config_json<C: DeserializeOwned>(line: &str) -> io::Result<(HashMap<String, String>, C)> {
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

    let config: C = match json.get("config") {
        None => return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "missing 'config' field in stdin JSON — coordinator must always send a config object",
        )),
        Some(v) => serde_json::from_value(v.clone()).map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("failed to deserialize config: {e}"),
            )
        })?,
    };

    Ok((topics, config))
}

pub trait Module: Sized + Send + 'static {
    type Config: DeserializeOwned + Debug;

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
                Err(e) => {
                    eprintln!("dimos_module: recv error: {e}");
                    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
                }
            }
        }
    });

    let pub_transport = Arc::clone(&transport);
    let pub_handle = tokio::spawn(async move {
        while let Some((topic, data)) = publish_rx.recv().await {
            if let Err(e) = pub_transport.publish(&topic, &data).await {
                eprintln!("dimos_module: publish error on {topic}: {e}");
            }
        }
    });

    (recv_handle, pub_handle)
}

fn propagate_task_failure(name: &str, res: Result<(), tokio::task::JoinError>) {
    match res {
        Ok(()) => eprintln!("dimos_module: {name} task exited unexpectedly"),
        Err(e) => {
            eprintln!("dimos_module: {name} task panicked, propagating");
            std::panic::resume_unwind(e.into_panic());
        }
    }
}

pub async fn run<M, T>(transport: T) -> io::Result<()>
where
    M: Module,
    T: Transport,
{
    let mut line = String::new();
    BufReader::new(tokio::io::stdin())
        .read_line(&mut line)
        .await?;
    let (topics, config) = parse_config_json::<M::Config>(&line)?;

    let exe = std::env::current_exe()
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        .unwrap_or_else(|| "unknown".to_string());
    eprintln!("[{exe}] topics received:");
    for (port, topic) in &topics {
        eprintln!("  {port} -> {topic}");
    }
    eprintln!("[{exe}] config: {config:?}");

    let (publish_tx, publish_rx) = mpsc::channel::<(String, Vec<u8>)>(PUBLISH_CHANNEL_CAPACITY);
    let mut builder = Builder::new(topics, publish_tx);
    let mut module = M::build(&mut builder, config);
    let (mut recv_handle, mut pub_handle) =
        spawn_pubsub_tasks(transport, builder.routes, publish_rx);

    module.setup().await;

    tokio::select! {
        _ = module.handle() => {}
        _ = tokio::signal::ctrl_c() => {}
        res = &mut recv_handle => propagate_task_failure("recv", res),
        res = &mut pub_handle => propagate_task_failure("publish", res),
    }

    module.teardown().await;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicU64, Ordering};
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

    #[derive(Debug, Deserialize, Default, PartialEq)]
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

        tokio::time::sleep(Duration::from_millis(50)).await;

        let recv_count = recv_log.lock().unwrap().len();
        assert!(
            recv_count >= 1,
            "expected recv to fire during slow publish; got {recv_count} events. \
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

        // simulate slow processing function in a receive
        let _input = builder.input("slow", |b| {
            std::thread::sleep(Duration::from_millis(200));
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

        // receive should still be processing, but publish should go through by now
        tokio::time::sleep(Duration::from_millis(50)).await;

        let publish_count = publish_log.lock().unwrap().len();
        assert!(
            publish_count >= 1,
            "expected publish to fire during slow recv dispatch; got \
             {publish_count} events. The publish path should be independent \
             of recv-side CPU work."
        );
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
}
