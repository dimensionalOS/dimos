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

use crate::transport::{Dispatch, Transport};

/// LCM UDP multicast transport. Wraps `dimos_lcm::Lcm`.
///
/// The multicast socket receives every channel, so `subscribe` registers a
/// callback locally and one recv loop routes each message by channel.
pub struct LcmTransport {
    inner: Arc<Lcm>,
    routes: Arc<Mutex<HashMap<String, Vec<Dispatch>>>>,
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
    let Some(rest) = url.strip_prefix("udpm://") else {
        tracing::warn!(
            url,
            "LCM_DEFAULT_URL is not a udpm:// url; using the defaults"
        );
        return options;
    };
    let (address, query) = rest.split_once('?').unwrap_or((rest, ""));
    if let Some((group, port)) = address.rsplit_once(':') {
        match (group.parse(), port.parse()) {
            (Ok(group), Ok(port)) => {
                options.multicast_group = group;
                options.port = port;
            }
            _ => tracing::warn!(
                url,
                "LCM_DEFAULT_URL has no parsable group:port; using the defaults"
            ),
        }
    }
    for (key, value) in query.split('&').filter_map(|pair| pair.split_once('=')) {
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
            listening: AtomicBool::new(false),
            runtime: tokio::runtime::Handle::current(),
        }
    }

    fn spawn_recv_loop(&self) {
        let inner = Arc::clone(&self.inner);
        let routes = Arc::clone(&self.routes);
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
    use super::options_from_url;
    use std::net::Ipv4Addr;

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
            "udpm://not-an-ip:7667",
            "udpm://239.255.76.67",
        ] {
            let options = options_from_url(url);
            assert_eq!(options.multicast_group, defaults.multicast_group, "{url}");
            assert_eq!(options.port, defaults.port, "{url}");
        }
    }
}
