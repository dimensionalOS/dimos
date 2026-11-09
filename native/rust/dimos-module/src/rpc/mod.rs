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

//! RPC between native modules and the rest of a dimos graph, on the wire
//! Python speaks (`dimos/protocol/rpc`): CBOR payloads, a call named
//! `<rpc_name>/<method>`, and a failure carried as Python's serialized exception.

use std::collections::HashMap;
use std::future::Future;
use std::io;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use serde::{Deserialize, Serialize};
use thiserror::Error;

pub mod cbor;
pub mod zenoh;

pub use cbor::{decode, encode, DecodeError, EncodeError, Msg};
pub use ciborium::Value;

/// Python's `(args, kwargs)`.
pub type Args = (Vec<Value>, HashMap<String, Value>);

/// A failure on either side of a call, in the shape of Python's
/// `SerializedException`. One raised here has the module `rust`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Error)]
#[error("[Remote {type_module}.{type_name}] {args:?}\n{traceback}")]
pub struct RemoteError {
    pub type_name: String,
    pub type_module: String,
    pub args: Vec<Value>,
    pub traceback: String,
}

impl RemoteError {
    pub fn new(type_name: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            type_name: type_name.into(),
            type_module: "rust".into(),
            args: vec![Value::Text(message.into())],
            traceback: String::new(),
        }
    }
}

#[derive(Debug, Error)]
pub enum RpcError {
    #[error(transparent)]
    Remote(#[from] RemoteError),
    #[error("RPC call to {name:?} timed out after {timeout:?}")]
    Timeout { name: String, timeout: Duration },
    #[error(transparent)]
    Decode(#[from] DecodeError),
    #[error(transparent)]
    Encode(#[from] EncodeError),
    #[error(transparent)]
    Transport(#[from] io::Error),
}

/// Answers one served method. Runs on the module's tokio runtime.
pub type Handler = Arc<
    dyn Fn(Args) -> Pin<Box<dyn Future<Output = Result<Value, RemoteError>> + Send>> + Send + Sync,
>;

/// The per-call waits Python sends on the launch line, resolved as Python
/// does: the full name, then the method, then the default.
#[derive(Clone, Debug, PartialEq)]
pub struct Timeouts {
    pub by_name: HashMap<String, f64>,
    pub default: f64,
}

impl Timeouts {
    pub fn for_call(&self, name: &str) -> Duration {
        let method = name.rsplit('/').next().unwrap_or(name);
        let seconds = self
            .by_name
            .get(name)
            .or_else(|| self.by_name.get(method))
            .copied()
            .unwrap_or(self.default);
        Duration::from_secs_f64(seconds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_python_exception_decodes_into_the_remote_error() {
        let error: RemoteError = decode(&cbor::fixture("exception")).unwrap();
        assert_eq!(error.type_name, "ValueError");
        assert_eq!(error.type_module, "builtins");
        assert_eq!(error.args, vec![Value::Text("bad goal".into())]);
        assert_eq!(
            decode::<RemoteError>(&encode(&error).unwrap()).unwrap(),
            error
        );
    }
}
