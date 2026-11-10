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
use serde::de::DeserializeOwned;

use super::{decode, encode, Args, Handler, RemoteError, RpcError};
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

impl ZenohTransport {
    /// Call `name` (`<rpc_name>/<method>`) and wait up to `timeout` for the result.
    pub async fn call<R: DeserializeOwned>(
        &self,
        name: &str,
        args: &Args,
        timeout: Duration,
    ) -> Result<R, RpcError> {
        let key = key(name);
        let payload = encode(args)?;
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
                        Err(decode::<RemoteError>(&err.payload().to_bytes())?.into())
                    }
                    Err(err) => Err(RpcError::Transport(std::io::Error::other(format!(
                        "reply to {name} failed with encoding {}",
                        err.encoding()
                    )))),
                };
            }
            tokio::time::sleep(RETRY.min(remaining)).await;
        }
    }

    /// Call `name` without waiting for, or receiving, a result.
    pub async fn call_nowait(&self, name: &str, args: &Args) -> Result<(), RpcError> {
        self.session()
            .get(key(name))
            .payload(encode(args)?)
            .target(QueryTarget::All)
            .consolidation(ConsolidationMode::None)
            .congestion_control(CongestionControl::Block)
            .attachment(NOWAIT)
            .callback(|_| {})
            .await
            .map_err(to_io)?;
        Ok(())
    }

    /// Answer calls to `name` with `handler` for the rest of the session.
    pub async fn serve(&self, name: &str, handler: Handler) -> Result<(), RpcError> {
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
    let result = match query.payload().map(|p| decode::<Args>(&p.to_bytes())) {
        Some(Ok(args)) => handler(args).await,
        Some(Err(e)) => Err(RemoteError::new("DecodeError", e.to_string())),
        None => Err(RemoteError::new(
            "DecodeError",
            "the query carries no arguments",
        )),
    };
    if let Err(e) = &result {
        tracing::warn!(name, error = %e, "rpc handler failed");
    }
    if nowait {
        return;
    }
    let sent = match result {
        Ok(value) => match encode(&value) {
            Ok(payload) => query.reply(query.key_expr().clone(), payload).await,
            Err(e) => Err(e.into()),
        },
        Err(e) => match encode(&e) {
            Ok(payload) => {
                query
                    .reply_err(payload)
                    .encoding(exception_encoding())
                    .await
            }
            Err(e) => Err(e.into()),
        },
    };
    if let Err(e) = sent {
        tracing::warn!(name, error = %e, "rpc reply not sent");
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::sync::Arc;

    use super::*;
    use crate::rpc::Value;

    /// One session calling itself, discovery off, as the Python suite does.
    async fn transport() -> ZenohTransport {
        let mut config = ::zenoh::Config::default();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        ZenohTransport::wrap(::zenoh::open(config).await.unwrap())
    }

    fn number(value: &Value) -> Result<f64, RemoteError> {
        value
            .as_float()
            .or_else(|| value.as_integer().map(|n| i128::from(n) as f64))
            .ok_or_else(|| RemoteError::new("TypeError", "not a number"))
    }

    fn adder() -> Handler {
        Arc::new(|(positional, keyword): Args| {
            Box::pin(async move {
                let mut sum = 0.0;
                for value in positional.iter().chain(keyword.values()) {
                    sum += number(value)?;
                }
                Ok(Value::Float(sum))
            })
        })
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_call_reaches_the_handler_and_the_result_comes_back() {
        let transport = transport().await;
        transport.serve("Calc/add", adder()).await.unwrap();
        let args = (
            vec![Value::from(1), Value::from(2.5)],
            HashMap::from([("offset".to_owned(), Value::from(10.0))]),
        );
        let result: f64 = transport
            .call("Calc/add", &args, Duration::from_secs(5))
            .await
            .unwrap();
        assert_eq!(result, 13.5);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_handler_failure_arrives_as_the_remote_error() {
        let transport = transport().await;
        transport.serve("Calc/add", adder()).await.unwrap();
        let args = (vec![Value::from("x")], HashMap::new());
        let err = transport
            .call::<f64>("Calc/add", &args, Duration::from_secs(5))
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
