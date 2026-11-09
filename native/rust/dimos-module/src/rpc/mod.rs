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

//! RPC between native modules and the rest of a dimos graph.
//!
//! The wire is the one Python speaks (`dimos/protocol/rpc`): CBOR payloads
//! with dimos' own tags, a call named `<rpc_name>/<method>`, and a failure
//! carried as the serialized exception Python raises on the caller's side.

use std::collections::HashMap;
use std::fmt;
use std::future::Future;
use std::io;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use thiserror::Error;

pub mod cbor;
pub mod zenoh;

pub use cbor::{decode, encode, DecodeError, Msg, Value};

/// The arguments of one call: Python's `(args, kwargs)`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Args {
    pub positional: Vec<Value>,
    pub keyword: Vec<(String, Value)>,
}

impl Args {
    pub fn positional(values: Vec<Value>) -> Self {
        Self {
            positional: values,
            keyword: Vec::new(),
        }
    }

    pub fn from_value(value: Value) -> Result<Self, RpcError> {
        let malformed = || RpcError::Malformed("arguments are not an (args, kwargs) pair");
        let [args, kwargs]: [Value; 2] = match value {
            Value::Tuple(items) | Value::List(items) => {
                items.try_into().map_err(|_| malformed())?
            }
            _ => return Err(malformed()),
        };
        let positional = match args {
            Value::Tuple(items) | Value::List(items) => items,
            _ => return Err(malformed()),
        };
        let Value::Map(entries) = kwargs else {
            return Err(malformed());
        };
        let keyword = entries
            .into_iter()
            .map(|(key, value)| match key {
                Value::Str(key) => Ok((key, value)),
                _ => Err(RpcError::Malformed(
                    "a keyword argument name is not a string",
                )),
            })
            .collect::<Result<_, _>>()?;
        Ok(Self {
            positional,
            keyword,
        })
    }

    pub fn to_value(&self) -> Value {
        Value::Tuple(vec![
            Value::Tuple(self.positional.clone()),
            Value::Map(
                self.keyword
                    .iter()
                    .map(|(key, value)| (Value::Str(key.clone()), value.clone()))
                    .collect(),
            ),
        ])
    }
}

/// A failure on either side of a call, in the shape of Python's
/// `SerializedException`. One raised here has the module `rust`.
#[derive(Clone, Debug, PartialEq)]
pub struct RemoteError {
    pub type_name: String,
    pub type_module: String,
    pub args: Vec<Value>,
    pub traceback: String,
}

impl RemoteError {
    /// A failure of this process: `type_name` is the kind, `message` says what went wrong.
    pub fn new(type_name: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            type_name: type_name.into(),
            type_module: "rust".into(),
            args: vec![Value::Str(message.into())],
            traceback: String::new(),
        }
    }

    pub fn from_value(value: Value) -> Result<Self, RpcError> {
        let field = |name: &str| {
            value.get(name).ok_or(RpcError::Malformed(
                "a serialized exception is missing a field",
            ))
        };
        let text = |name: &str| -> Result<String, RpcError> {
            field(name)?
                .as_str()
                .map(str::to_owned)
                .ok_or(RpcError::Malformed(
                    "a serialized exception field is not text",
                ))
        };
        Ok(Self {
            type_name: text("type_name")?,
            type_module: text("type_module")?,
            args: field("args")?.as_slice().unwrap_or_default().to_vec(),
            traceback: text("traceback")?,
        })
    }

    pub fn to_value(&self) -> Value {
        Value::Map(vec![
            (
                Value::from("type_name"),
                Value::from(self.type_name.as_str()),
            ),
            (
                Value::from("type_module"),
                Value::from(self.type_module.as_str()),
            ),
            (Value::from("args"), Value::Tuple(self.args.clone())),
            (
                Value::from("traceback"),
                Value::from(self.traceback.as_str()),
            ),
        ])
    }
}

impl fmt::Display for RemoteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[Remote {}.{}]", self.type_module, self.type_name)?;
        for arg in &self.args {
            match arg {
                Value::Str(text) => write!(f, " {text}")?,
                other => write!(f, " {other:?}")?,
            }
        }
        if !self.traceback.is_empty() {
            write!(f, "\n\nRemote traceback:\n{}", self.traceback)?;
        }
        Ok(())
    }
}

impl std::error::Error for RemoteError {}

#[derive(Debug, Error)]
pub enum RpcError {
    #[error(transparent)]
    Remote(#[from] RemoteError),
    #[error("RPC call to {name:?} timed out after {timeout:?}")]
    Timeout { name: String, timeout: Duration },
    #[error(transparent)]
    Decode(#[from] DecodeError),
    #[error("{0}")]
    Malformed(&'static str),
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

/// Calls and serves RPCs on the same wire as Python's `RPCSpec`.
pub trait Rpc: Send + Sync {
    /// Call `name` (`<rpc_name>/<method>`) and wait up to `timeout` for the result.
    fn call(
        &self,
        name: &str,
        args: &Args,
        timeout: Duration,
    ) -> impl Future<Output = Result<Value, RpcError>> + Send;

    /// Call `name` without waiting for, or receiving, a result.
    fn call_nowait(
        &self,
        name: &str,
        args: &Args,
    ) -> impl Future<Output = Result<(), RpcError>> + Send;

    /// Answer calls to `name` with `handler` for the rest of the session.
    fn serve(
        &self,
        name: &str,
        handler: Handler,
    ) -> impl Future<Output = Result<(), RpcError>> + Send;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arguments_match_the_python_envelope() {
        let fixtures: serde_json::Map<String, serde_json::Value> =
            serde_json::from_str(include_str!("../../tests/fixtures/rpc_cbor.json")).unwrap();
        let bytes = hex::decode(fixtures["rpc_arguments"].as_str().unwrap()).unwrap();
        let args = Args::from_value(decode(&bytes).unwrap()).unwrap();
        assert_eq!(
            args.positional,
            vec![Value::Float(1.0), Value::from("goal")]
        );
        assert_eq!(
            args.keyword,
            vec![("timeout".to_owned(), Value::Float(2.5))]
        );
        assert_eq!(encode(&args.to_value()), bytes);
    }

    #[test]
    fn a_list_of_positional_arguments_is_accepted_too() {
        let args = Args::from_value(Value::List(vec![
            Value::List(vec![Value::Int(1)]),
            Value::Map(vec![]),
        ]))
        .unwrap();
        assert_eq!(args, Args::positional(vec![Value::Int(1)]));
        assert!(matches!(
            Args::from_value(Value::Int(1)),
            Err(RpcError::Malformed(_))
        ));
    }

    #[test]
    fn a_remote_error_round_trips_through_the_python_shape() {
        let error = RemoteError::new("ValueError", "bad goal");
        let back = RemoteError::from_value(error.to_value()).unwrap();
        assert_eq!(back, error);
        assert_eq!(back.to_string(), "[Remote rust.ValueError] bad goal");
    }

    #[test]
    fn timeouts_resolve_like_python() {
        let timeouts = Timeouts {
            by_name: HashMap::from([("start".to_owned(), 1200.0), ("Nav/plan".to_owned(), 5.0)]),
            default: 120.0,
        };
        assert_eq!(timeouts.for_call("Nav/start"), Duration::from_secs(1200));
        assert_eq!(timeouts.for_call("Nav/plan"), Duration::from_secs(5));
        assert_eq!(timeouts.for_call("Other/plan"), Duration::from_secs(120));
    }
}
