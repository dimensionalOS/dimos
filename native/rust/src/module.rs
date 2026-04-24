use std::collections::HashMap;
use std::io::{self, BufRead};
use tokio::sync::mpsc;

use serde::de::DeserializeOwned;

use crate::transport::Transport;

const INPUT_CHANNEL_CAPACITY: usize = 16;
const PUBLISH_CHANNEL_CAPACITY: usize = 64;

// Each input() call produces a TypedRoute that decodes its message type
// and forwards it to the right Input's mpsc channel.
trait Route: Send {
    fn topic(&self) -> &str;
    fn try_dispatch(&self, data: &[u8]);
}

struct TypedRoute<T: Send + 'static> {
    topic: String,
    decode: fn(&[u8]) -> io::Result<T>,
    sender: mpsc::Sender<T>,
}

impl<T: Send + 'static> Route for TypedRoute<T> {
    fn topic(&self) -> &str {
        &self.topic
    }

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

/// High-level wrapper around a transport for use in dimos native modules.
///
/// Generic over any `T: Transport`. Use `LcmTransport` for the standard LCM
/// UDP multicast transport.
///
/// # Usage
///
/// ```ignore
/// let transport = LcmTransport::new().await?;
/// let (mut module, config) = NativeModule::from_stdin::<MyConfig>(transport).await?;
///
/// let mut image_in = module.input("color_image", Image::decode);
/// let cmd_out      = module.output("cmd_vel", Twist::encode);
/// let _handle      = module.spawn();
///
/// loop {
///     tokio::select! {
///         Some(frame) = image_in.recv() => { cmd_out.publish(&twist).await.ok(); }
///     }
/// }
/// ```
pub struct NativeModule<T: Transport> {
    transport: T,
    routes: Vec<Box<dyn Route>>,
    topics: HashMap<String, String>,
    publish_tx: mpsc::Sender<(String, Vec<u8>)>,
    publish_rx: mpsc::Receiver<(String, Vec<u8>)>,
}

impl<T: Transport> NativeModule<T> {
    pub(crate) fn new(transport: T) -> Self {
        let (publish_tx, publish_rx) = mpsc::channel(PUBLISH_CHANNEL_CAPACITY);
        Self {
            transport,
            routes: Vec::new(),
            topics: HashMap::new(),
            publish_tx,
            publish_rx,
        }
    }

    /// Parse `--port_name topic_string` pairs from argv, as injected by NativeModule.
    pub async fn from_args(transport: T) -> io::Result<Self> {
        let mut module = Self::new(transport);
        let args: Vec<String> = std::env::args().collect();
        let mut i = 1;
        while i < args.len() {
            if let Some(port) = args[i].strip_prefix("--") {
                if i + 1 < args.len() && !args[i + 1].starts_with("--") {
                    module.topics.insert(port.to_string(), args[i + 1].clone());
                    i += 2;
                    continue;
                }
            }
            i += 1;
        }
        Ok(module)
    }

    /// Read config from a single JSON line on stdin, as written by the Python NativeModule declaration.
    ///
    /// The JSON format is:
    /// ```json
    /// {"topics": {"port_name": "lcm/topic", ...}, "config": { ... }}
    /// ```
    ///
    /// `C` is the module-specific config type. Use `()` for modules with no configuration.
    pub async fn from_stdin<C: DeserializeOwned + std::fmt::Debug>(
        transport: T,
    ) -> io::Result<(Self, C)> {
        let mut line = String::new();
        io::stdin().lock().read_line(&mut line)?;

        let (topics, config) = parse_config_json::<C>(&line)?;

        let mut module = Self::new(transport);
        module.topics = topics;

        let exe = std::env::current_exe()
            .ok()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
            .unwrap_or_else(|| "unknown".to_string());
        eprintln!("[{exe}] topics received:");
        for (port, topic) in &module.topics {
            eprintln!("  {port} -> {topic}");
        }
        eprintln!("[{exe}] config: {config:?}");

        Ok((module, config))
    }

    /// Manually set a topic for a port — useful for testing without a parent process.
    pub fn map_topic(&mut self, port: &str, topic: &str) {
        self.topics.insert(port.to_string(), topic.to_string());
    }

    fn topic_for(&self, port: &str) -> String {
        self.topics
            .get(port)
            .cloned()
            .unwrap_or_else(|| format!("/{port}"))
    }

    /// Register an input port. Must be called before `spawn()`.
    pub fn input<M: Send + 'static>(
        &mut self,
        port: &str,
        decode: fn(&[u8]) -> io::Result<M>,
    ) -> Input<M> {
        let topic = self.topic_for(port);
        let (tx, rx) = mpsc::channel(INPUT_CHANNEL_CAPACITY);
        self.routes.push(Box::new(TypedRoute {
            topic: topic.clone(),
            decode,
            sender: tx,
        }));
        Input {
            topic,
            receiver: rx,
        }
    }

    /// Register an output port. Must be called before `spawn()`.
    pub fn output<M: Send + 'static>(&self, port: &str, encode: fn(&M) -> Vec<u8>) -> Output<M> {
        Output {
            topic: self.topic_for(port),
            encode,
            sender: self.publish_tx.clone(),
        }
    }

    /// Start the background recv/dispatch/publish loop.
    ///
    /// Consumes the module — no new ports can be registered after this point.
    pub fn spawn(self) -> NativeModuleHandle {
        let NativeModule {
            mut transport,
            routes,
            mut publish_rx,
            ..
        } = self;

        let handle = tokio::spawn(async move {
            loop {
                tokio::select! {
                    result = transport.recv() => match result {
                        Ok((channel, data)) => {
                            for route in &routes {
                                if route.topic() == channel {
                                    route.try_dispatch(&data);
                                }
                            }
                        }
                        Err(e) => {
                            eprintln!("dimos_module: recv error: {e}");
                            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
                        }
                    },
                    Some((topic, data)) = publish_rx.recv() => {
                        if let Err(e) = transport.publish(&topic, &data).await {
                            eprintln!("dimos_module: publish error on {topic}: {e}");
                        }
                    }
                }
            }
        });

        NativeModuleHandle(handle)
    }
}

pub struct NativeModuleHandle(tokio::task::JoinHandle<()>);

impl NativeModuleHandle {
    pub async fn join(self) -> Result<(), tokio::task::JoinError> {
        self.0.await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    struct MockTransport;

    impl crate::transport::Transport for MockTransport {
        async fn publish(&self, _channel: &str, _data: &[u8]) -> io::Result<()> {
            Ok(())
        }
        async fn recv(&mut self) -> io::Result<(String, Vec<u8>)> {
            std::future::pending().await
        }
    }

    #[derive(Debug, Deserialize, Default, PartialEq)]
    #[serde(deny_unknown_fields)]
    struct TestConfig {
        value: i64,
        name: String,
    }

    // --- parse_config_json ---

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

    // --- topic_for / map_topic ---

    #[test]
    fn unmapped_port_falls_back_to_slash_port() {
        let module = NativeModule::new(MockTransport);
        assert_eq!(module.topic_for("cmd_vel"), "/cmd_vel");
    }

    #[test]
    fn map_topic_overrides_fallback() {
        let mut module = NativeModule::new(MockTransport);
        module.map_topic("cmd_vel", "/robot/cmd_vel");
        assert_eq!(module.topic_for("cmd_vel"), "/robot/cmd_vel");
    }

    #[test]
    fn input_uses_mapped_topic() {
        let mut module = NativeModule::new(MockTransport);
        module.map_topic("data", "/test/data");
        let input = module.input("data", |b| Ok(b.to_vec()));
        assert_eq!(input.topic, "/test/data");
    }

    #[test]
    fn input_falls_back_to_slash_port_when_unmapped() {
        let mut module = NativeModule::new(MockTransport);
        let input = module.input("data", |b| Ok(b.to_vec()));
        assert_eq!(input.topic, "/data");
    }

    #[test]
    fn output_uses_mapped_topic() {
        let mut module = NativeModule::new(MockTransport);
        module.map_topic("cmd_vel", "/robot/cmd_vel");
        let output = module.output("cmd_vel", |b: &Vec<u8>| b.clone());
        assert_eq!(output.topic, "/robot/cmd_vel");
    }
}
