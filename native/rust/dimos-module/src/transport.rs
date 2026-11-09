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

use std::future::Future;
use std::io;
use std::pin::Pin;
use std::sync::Arc;

use async_trait::async_trait;

use crate::lcm::LcmTransport;
use crate::zenoh::ZenohTransport;

/// Per-channel delivery handed to a transport on `subscribe`. The transport
/// calls it with each message's raw payload. Decode and routing happen inside.
pub trait Deliver: Send + Sync {
    /// Blocks the calling thread while an input is full. For transports that
    /// call back from their own threads.
    fn deliver(&self, data: &[u8]);
    /// Suspends the calling task while an input is full. For transports that
    /// own their receive loop.
    fn deliver_async<'a>(&'a self, data: &'a [u8]) -> BoxFuture<'a, ()>;
}

pub type Dispatch = Arc<dyn Deliver>;

/// A plain callback is a delivery that never stalls.
impl<F: Fn(&[u8]) + Send + Sync> Deliver for F {
    fn deliver(&self, data: &[u8]) {
        self(data)
    }

    fn deliver_async<'a>(&'a self, data: &'a [u8]) -> BoxFuture<'a, ()> {
        Box::pin(async move { self(data) })
    }
}

/// Abstraction over the message transport used by a native module.
///
/// New transport protocols should implement this trait. A module runs over
/// an `Arc<dyn Transport>`.
#[async_trait]
pub trait Transport: Send + Sync + 'static {
    /// Send `data` on `channel`.
    async fn publish(&self, channel: &str, data: Vec<u8>) -> io::Result<()>;
    /// Deliver each message on `channel` to `on_msg`.
    async fn subscribe(&self, channel: &str, on_msg: Dispatch) -> io::Result<()>;
}

/// Open the transport `DIMOS_TRANSPORT` names from the launch line, which
/// carries the transport's `session` and `qos` blocks.
///
/// Every transport is compiled in, so one binary follows whichever transport
/// the coordinator picks at runtime. The coordinator always sets the variable,
/// so an unset or unknown value is an error.
pub async fn open_transport(launch: &serde_json::Value) -> io::Result<Arc<dyn Transport>> {
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => Ok(Arc::new(LcmTransport::from_launch(launch).await?)),
        Ok("zenoh") => Ok(Arc::new(ZenohTransport::from_launch(launch).await?)),
        other => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got {other:?}"),
        )),
    }
}

pub(crate) type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;
