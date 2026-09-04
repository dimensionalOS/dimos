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
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use dimos_lcm::{Lcm, LcmOptions};
use url::Url;

use crate::transport::{Dispatch, TopicDispatch, Transport};

/// LCM UDP multicast transport. Wraps `dimos_lcm::Lcm`.
///
/// The multicast socket receives every channel, so `subscribe` registers a
/// callback locally and one recv loop routes each message by channel.
pub struct LcmTransport {
    inner: Arc<Lcm>,
    routes: Arc<Mutex<HashMap<String, Vec<Dispatch>>>>,
    all_routes: Arc<Mutex<Vec<TopicDispatch>>>,
    listening: AtomicBool,
    /// The runtime the transport was opened on. In a baked host each module has
    /// its own runtime, so the one shared recv loop must not land on whichever
    /// module happened to subscribe first.
    runtime: tokio::runtime::Handle,
}

/// liblcm reads this, so python modules follow it and native ones have to as well or the
/// two halves of a pipeline end up on different buses. Format: `udpm://group:port?ttl=n`.
fn options_from_env() -> LcmOptions {
    match std::env::var("LCM_DEFAULT_URL") {
        Ok(url) => options_from_url(&url),
        Err(_) => LcmOptions::default(),
    }
}

fn options_from_url(url: &str) -> LcmOptions {
    let mut options = LcmOptions::default();
    let parsed = match Url::parse(url) {
        Ok(parsed) if parsed.scheme() == "udpm" => parsed,
        _ => {
            tracing::warn!(
                url,
                "LCM_DEFAULT_URL is not a udpm:// url; using the defaults"
            );
            return options;
        }
    };
    // udpm is not a special scheme, so the host stays an opaque string.
    let group = parsed.host_str().and_then(|host| host.parse().ok());
    match (group, parsed.port()) {
        (Some(group), Some(port)) => {
            options.multicast_group = group;
            options.port = port;
        }
        _ => {
            tracing::warn!(
                url,
                "LCM_DEFAULT_URL has no parsable group:port; using the defaults"
            );
            return options;
        }
    }
    for (key, value) in parsed.query_pairs() {
        if key == "ttl" {
            if let Ok(ttl) = value.parse() {
                options.ttl = ttl;
            }
        }
    }
    options
}

impl LcmTransport {
    pub async fn new() -> io::Result<Self> {
        Self::with_options(options_from_env()).await
    }

    pub async fn with_options(opts: LcmOptions) -> io::Result<Self> {
        Ok(Self::wrap(Lcm::with_options(opts).await?))
    }

    fn wrap(inner: Lcm) -> Self {
        Self {
            inner: Arc::new(inner),
            routes: Arc::new(Mutex::new(HashMap::new())),
            all_routes: Arc::new(Mutex::new(Vec::new())),
            listening: AtomicBool::new(false),
            runtime: tokio::runtime::Handle::current(),
        }
    }

    fn spawn_recv_loop(&self) {
        let inner = Arc::clone(&self.inner);
        let routes = Arc::clone(&self.routes);
        let all_routes = Arc::clone(&self.all_routes);
        self.runtime.spawn(async move {
            loop {
                match inner.recv().await {
                    Ok(msg) => {
                        let callbacks = routes.lock().unwrap().get(&msg.channel).cloned();
                        if let Some(callbacks) = callbacks {
                            for cb in &callbacks {
                                cb(&msg.data);
                            }
                        }
                        dispatch_all(&all_routes, &msg.channel, &msg.data);
                    }
                    Err(e) => {
                        crate::error_throttled!(
                            Duration::from_secs(1),
                            error = %e,
                            "lcm recv error"
                        );
                    }
                }
            }
        });
    }
}

fn dispatch_all(all_routes: &Mutex<Vec<TopicDispatch>>, channel: &str, data: &[u8]) {
    if channel != "LCM_SELF_TEST" {
        let callbacks = all_routes.lock().unwrap().clone();
        for callback in &callbacks {
            callback(data, channel);
        }
    }
}

impl Transport for LcmTransport {
    async fn publish(&self, channel: &str, data: Vec<u8>) -> io::Result<()> {
        self.inner.publish(channel, &data).await
    }

    async fn subscribe(&self, channel: &str, on_msg: Dispatch) -> io::Result<()> {
        self.routes
            .lock()
            .unwrap()
            .entry(channel.to_string())
            .or_default()
            .push(on_msg);
        if !self.listening.swap(true, Ordering::SeqCst) {
            self.spawn_recv_loop();
        }
        Ok(())
    }

    async fn subscribe_all(&self, on_msg: TopicDispatch) -> io::Result<()> {
        self.all_routes.lock().unwrap().push(on_msg);
        if !self.listening.swap(true, Ordering::SeqCst) {
            self.spawn_recv_loop();
        }
        Ok(())
    }

    /// LCM has no per-topic publisher settings and no notion of a session-local
    /// publisher, so a baked host cannot hide an internal hop on this transport.
    fn set_publisher_qos(&self, qos: &serde_json::Value) {
        let suppressed: Vec<&String> = qos
            .as_object()
            .map(|map| {
                map.iter()
                    .filter(|(_, entry)| entry.get("locality").is_some())
                    .map(|(channel, _)| channel)
                    .collect()
            })
            .unwrap_or_default();
        if !suppressed.is_empty() {
            tracing::warn!(
                channels = ?suppressed,
                "LCM cannot suppress a topic; these stay visible on the multicast bus",
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{dispatch_all, options_from_url};
    use crate::transport::TopicDispatch;
    use std::net::Ipv4Addr;
    use std::sync::{Arc, Mutex};

    #[test]
    fn reads_group_port_and_ttl() {
        let options = options_from_url("udpm://239.255.76.67:7712?ttl=0");
        assert_eq!(options.multicast_group, Ipv4Addr::new(239, 255, 76, 67));
        assert_eq!(options.port, 7712);
        assert_eq!(options.ttl, 0);
    }

    #[test]
    fn ttl_is_optional() {
        let options = options_from_url("udpm://239.255.76.67:7712");
        assert_eq!(options.port, 7712);
        assert_eq!(options.ttl, dimos_lcm::LcmOptions::default().ttl);
    }

    #[test]
    fn an_unusable_url_leaves_the_defaults() {
        let defaults = dimos_lcm::LcmOptions::default();
        for url in [
            "tcp://127.0.0.1:7667",
            "udpm://not-an-ip:7667?ttl=42",
            "udpm://239.255.76.67?ttl=42",
            "239.255.76.67:7667",
        ] {
            let options = options_from_url(url);
            assert_eq!(options.multicast_group, defaults.multicast_group, "{url}");
            assert_eq!(options.port, defaults.port, "{url}");
            assert_eq!(options.ttl, defaults.ttl, "{url}");
        }
    }

    #[test]
    fn subscribe_all_receives_the_channel_and_payload() {
        let all = Arc::new(Mutex::new(Vec::new()));
        let all_sink = Arc::clone(&all);
        let all_routes = Mutex::new(vec![Arc::new(move |data: &[u8], topic: &str| {
            all_sink
                .lock()
                .unwrap()
                .push((topic.to_string(), data.to_vec()));
        }) as TopicDispatch]);

        dispatch_all(&all_routes, "camera", b"frame");
        dispatch_all(&all_routes, "LCM_SELF_TEST", b"ignore");

        assert_eq!(
            *all.lock().unwrap(),
            [("camera".to_string(), b"frame".to_vec())]
        );
    }
}
