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

//! Serves a few RPCs on zenoh and calls Python back, driven by
//! `dimos/protocol/rpc/test_rust_rpc.py`.
//!
//! Reads one launch line on stdin: the `session` a native module gets, plus
//! `name`, the prefix to serve under, and `python`, the prefix to call.
//! Runs until stdin closes.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use dimos_module::rpc::{Args, Handler, Msg, RemoteError, RpcError, Value};
use dimos_module::ZenohTransport;
use lcm_msgs::geometry_msgs::PoseStamped;
use serde::Deserialize;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, BufReader};

#[derive(Deserialize)]
struct Launch {
    name: String,
    python: String,
}

#[tokio::main]
async fn main() {
    let mut stdin = BufReader::new(tokio::io::stdin());
    let mut line = String::new();
    stdin.read_line(&mut line).await.expect("a launch line");
    let launch: serde_json::Value = serde_json::from_str(&line).expect("launch JSON");
    let Launch { name, python } = serde_json::from_value(launch.clone()).expect("name and python");

    let transport = Arc::new(
        ZenohTransport::from_launch(&launch)
            .await
            .expect("zenoh session"),
    );
    transport
        .serve(&format!("{name}/add"), add())
        .await
        .unwrap();
    transport
        .serve(&format!("{name}/frame"), frame())
        .await
        .unwrap();
    transport
        .serve(&format!("{name}/fail"), fail())
        .await
        .unwrap();
    transport
        .serve(
            &format!("{name}/call_python"),
            call_python(Arc::clone(&transport), python),
        )
        .await
        .unwrap();

    let mut rest = String::new();
    let _ = stdin.read_to_string(&mut rest).await;
}

fn number(value: &Value) -> Result<f64, RemoteError> {
    value
        .as_float()
        .or_else(|| value.as_integer().map(|n| i128::from(n) as f64))
        .ok_or_else(|| RemoteError::new("TypeError", "not a number"))
}

fn add() -> Handler {
    Arc::new(|(positional, keyword): Args| {
        Box::pin(async move {
            let mut sum = 0.0;
            for value in positional.iter().chain(keyword.values()) {
                sum += number(value)?;
            }
            Ok(Value::from(sum))
        })
    })
}

fn frame() -> Handler {
    Arc::new(|(positional, _): Args| {
        Box::pin(async move {
            let msg: Msg = positional
                .first()
                .ok_or_else(|| RemoteError::new("TypeError", "frame() needs a pose"))?
                .deserialized()
                .map_err(|e| RemoteError::new("TypeError", e.to_string()))?;
            let pose = PoseStamped::decode(&msg.data)
                .map_err(|e| RemoteError::new("ValueError", e.to_string()))?;
            Ok(Value::from(pose.header.frame_id))
        })
    })
}

fn fail() -> Handler {
    Arc::new(|_: Args| Box::pin(async { Err(RemoteError::new("ValueError", "nope")) }))
}

fn call_python(transport: Arc<ZenohTransport>, python: String) -> Handler {
    Arc::new(move |(_, keyword): Args| {
        let transport = Arc::clone(&transport);
        let python = python.clone();
        Box::pin(async move {
            let fail = keyword
                .get("fail")
                .and_then(Value::as_bool)
                .unwrap_or(false);
            let method = if fail { "boom" } else { "greet" };
            let args: Args = (vec![Value::from("rust")], HashMap::new());
            transport
                .call(&format!("{python}/{method}"), &args, Duration::from_secs(5))
                .await
                .map_err(|e| match e {
                    RpcError::Remote(remote) => remote,
                    other => RemoteError::new("RpcError", other.to_string()),
                })
        })
    })
}
