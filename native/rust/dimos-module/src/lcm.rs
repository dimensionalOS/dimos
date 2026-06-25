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
}

impl LcmTransport {
    pub async fn new() -> io::Result<Self> {
        Ok(Self::wrap(Lcm::new().await?))
    }

    pub async fn with_options(opts: LcmOptions) -> io::Result<Self> {
        Ok(Self::wrap(Lcm::with_options(opts).await?))
    }

    fn wrap(inner: Lcm) -> Self {
        Self {
            inner: Arc::new(inner),
            routes: Arc::new(Mutex::new(HashMap::new())),
            listening: AtomicBool::new(false),
        }
    }

    fn spawn_recv_loop(&self) {
        let inner = Arc::clone(&self.inner);
        let routes = Arc::clone(&self.routes);
        tokio::spawn(async move {
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
    async fn publish(&self, channel: &str, data: &[u8]) -> io::Result<()> {
        self.inner.publish(channel, data).await
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
}
