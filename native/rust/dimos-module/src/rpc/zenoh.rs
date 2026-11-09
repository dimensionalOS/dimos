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

//! RPC over zenoh, as `dimos/protocol/rpc/zenohrpc.py` speaks it: a call is a
//! query on `dimos/rpc/<name>`, a server is a queryable on that key, and a
//! failure is an error reply with the `dimos/rpc-exception` encoding.

use std::time::{Duration, Instant};

use ::zenoh::bytes::Encoding;
use ::zenoh::qos::CongestionControl;
use ::zenoh::query::{ConsolidationMode, Query, QueryTarget};

use super::{decode, encode, Args, Handler, RemoteError, Rpc, RpcError, Value};
use crate::zenoh::{to_io, ZenohTransport};

/// Python's retry interval while no queryable has answered a query.
const RETRY: Duration = Duration::from_millis(50);

const NOWAIT: &[u8] = b"nowait";

fn key(name: &str) -> String {
    format!("dimos/rpc/{name}")
}

fn exception_encoding() -> Encoding {
    Encoding::from("dimos/rpc-exception")
}

impl Rpc for ZenohTransport {
    async fn call(&self, name: &str, args: &Args, timeout: Duration) -> Result<Value, RpcError> {
        let key = key(name);
        let payload = encode(&args.to_value());
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(RpcError::Timeout {
                    name: name.to_owned(),
                    timeout,
                });
            }
            let replies = self
                .session()
                .get(&key)
                .payload(payload.clone())
                .target(QueryTarget::All)
                .consolidation(ConsolidationMode::None)
                .congestion_control(CongestionControl::Block)
                .timeout(remaining)
                .await
                .map_err(to_io)?;
            if let Ok(reply) = replies.recv_async().await {
                return match reply.result() {
                    Ok(sample) => Ok(decode(&sample.payload().to_bytes())?),
                    Err(err) if *err.encoding() == exception_encoding() => {
                        let exception = decode(&err.payload().to_bytes())?;
                        Err(RemoteError::from_value(exception)?.into())
                    }
                    Err(err) => Err(RpcError::Transport(io_error(format!(
                        "reply to {name} failed with encoding {}",
                        err.encoding()
                    )))),
                };
            }
            tokio::time::sleep(RETRY.min(remaining)).await;
        }
    }

    async fn call_nowait(&self, name: &str, args: &Args) -> Result<(), RpcError> {
        self.session()
            .get(key(name))
            .payload(encode(&args.to_value()))
            .target(QueryTarget::All)
            .consolidation(ConsolidationMode::None)
            .congestion_control(CongestionControl::Block)
            .attachment(NOWAIT)
            .callback(|_| {})
            .await
            .map_err(to_io)?;
        Ok(())
    }

    async fn serve(&self, name: &str, handler: Handler) -> Result<(), RpcError> {
        let runtime = tokio::runtime::Handle::current();
        let name = name.to_owned();
        self.session()
            .declare_queryable(key(&name))
            .complete(true)
            .callback(move |query| {
                let handler = handler.clone();
                let name = name.clone();
                runtime.spawn(async move { answer(&name, query, handler).await });
            })
            .background()
            .await
            .map_err(to_io)?;
        Ok(())
    }
}

async fn answer(name: &str, query: Query, handler: Handler) {
    let nowait = query.attachment().is_some();
    let result = match arguments(&query) {
        Ok(args) => handler(args).await,
        Err(e) => Err(RemoteError::new("DecodeError", e.to_string())),
    };
    if let Err(e) = &result {
        tracing::warn!(name, error = %e, "rpc handler failed");
    }
    if nowait {
        return;
    }
    let sent = match result {
        Ok(value) => query.reply(query.key_expr().clone(), encode(&value)).await,
        Err(e) => {
            query
                .reply_err(encode(&e.to_value()))
                .encoding(exception_encoding())
                .await
        }
    };
    if let Err(e) = sent {
        tracing::warn!(name, error = %e, "rpc reply not sent");
    }
}

fn arguments(query: &Query) -> Result<Args, RpcError> {
    let payload = query
        .payload()
        .ok_or(RpcError::Malformed("the query carries no arguments"))?;
    Args::from_value(decode(&payload.to_bytes())?)
}

fn io_error(message: String) -> std::io::Error {
    std::io::Error::other(message)
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;

    /// One session calling itself, discovery off, as the Python suite does.
    async fn transport() -> ZenohTransport {
        let mut config = ::zenoh::Config::default();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        ZenohTransport::wrap(::zenoh::open(config).await.unwrap())
    }

    fn adder() -> Handler {
        Arc::new(|args: Args| {
            Box::pin(async move {
                let mut sum = 0.0;
                for value in &args.positional {
                    sum += value
                        .as_f64()
                        .ok_or_else(|| RemoteError::new("TypeError", "not a number"))?;
                }
                for (_, value) in &args.keyword {
                    sum += value.as_f64().unwrap_or(0.0);
                }
                Ok(Value::Float(sum))
            })
        })
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_call_reaches_the_handler_and_the_result_comes_back() {
        let transport = transport().await;
        transport.serve("Calc/add", adder()).await.unwrap();
        let args = Args {
            positional: vec![Value::Int(1), Value::Float(2.5)],
            keyword: vec![("offset".into(), Value::Float(10.0))],
        };
        let result = transport
            .call("Calc/add", &args, Duration::from_secs(5))
            .await
            .unwrap();
        assert_eq!(result, Value::Float(13.5));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_handler_failure_arrives_as_the_remote_error() {
        let transport = transport().await;
        transport.serve("Calc/add", adder()).await.unwrap();
        let args = Args::positional(vec![Value::from("x")]);
        let err = transport
            .call("Calc/add", &args, Duration::from_secs(5))
            .await
            .unwrap_err();
        match err {
            RpcError::Remote(remote) => {
                assert_eq!(remote.type_module, "rust");
                assert_eq!(remote.type_name, "TypeError");
                assert_eq!(remote.args, vec![Value::from("not a number")]);
            }
            other => panic!("expected a remote error, got {other}"),
        }
    }
}
