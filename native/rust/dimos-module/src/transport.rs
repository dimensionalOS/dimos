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
use std::sync::Arc;

/// Per-channel dispatch closure handed to a transport on `subscribe`. The
/// transport calls it with each message's raw payload. Decode and routing
/// happen inside it.
pub type Dispatch = Arc<dyn Fn(&[u8]) + Send + Sync>;

/// Abstraction over the message transport used by a native module.
///
/// New transport protocols should implement this trait.
/// `NativeModule` is generic over any transport
pub trait Transport: Send + Sync + 'static {
    /// Send `data` on `channel`.
    fn publish(&self, channel: &str, data: Vec<u8>) -> impl Future<Output = io::Result<()>> + Send;
    /// Deliver each message on `channel` to `on_msg`.
    fn subscribe(
        &self,
        channel: &str,
        on_msg: Dispatch,
    ) -> impl Future<Output = io::Result<()>> + Send;

    /// Apply the per-channel publisher QoS the coordinator sends. The value is
    /// the `qos` object from the stdin config, or null when absent. Transports
    /// without per-topic QoS ignore it.
    fn set_publisher_qos(&self, _qos: &serde_json::Value) {}
}
