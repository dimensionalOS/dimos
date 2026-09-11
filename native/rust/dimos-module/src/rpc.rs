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

use std::collections::{BTreeMap, BTreeSet};
use std::io;

use serde::Deserialize;
use serde_json::{json, Value};
use zenoh::bytes::Encoding;

use crate::ZenohTransport;

pub struct Error {
    pub code: i32,
    pub message: String,
}

pub type Handler = fn(Value) -> Result<Value, Error>;

#[derive(Deserialize)]
struct Registration {
    name: String,
    methods: Vec<String>,
    token: String,
}

fn error(id: Value, code: i32, message: &str) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message}})
}

fn respond(
    payload: &[u8],
    route: &str,
    methods: &BTreeMap<&str, Handler>,
    token: &str,
) -> Option<Value> {
    let request: Value = match serde_json::from_slice(payload) {
        Ok(value) => value,
        Err(_) => return Some(error(Value::Null, -32700, "Parse error")),
    };
    let id = request.get("id").cloned().unwrap_or(Value::Null);
    if !request.is_object()
        || request["jsonrpc"] != "2.0"
        || !request["method"].is_string()
        || !(id.is_null() || id.is_string() || id.is_number())
    {
        return Some(error(Value::Null, -32600, "Invalid Request"));
    }
    // This request/reply subset does not execute notifications.
    request.get("id")?;
    if request["method"] != route {
        return Some(error(id, -32600, "Method does not match route"));
    }
    let params = request.get("params").cloned().unwrap_or_else(|| json!({}));
    if !params.is_object() {
        return Some(error(id, -32602, "Expected named parameters"));
    }
    let method = route.rsplit('/').next().unwrap_or_default();
    let result = if method == "_ready" {
        if params != json!({}) {
            return Some(error(id, -32602, "_ready takes no parameters"));
        }
        Ok(json!({"methods": methods.keys().collect::<Vec<_>>(), "token": token}))
    } else if let Some(handler) = methods.get(method) {
        match std::panic::catch_unwind(|| handler(params)) {
            Ok(result) => result,
            Err(_) => return Some(error(id, -32603, "Handler panicked")),
        }
    } else {
        return Some(error(id, -32601, "Method not found"));
    };
    Some(match result {
        Ok(value) => json!({"jsonrpc": "2.0", "id": id, "result": value}),
        Err(e) => error(id, e.code, &e.message),
    })
}

/// Run an RPC-only native process using NativeModule's stdin launch settings.
pub async fn run(handlers: &[(&str, Handler)]) -> zenoh::Result<()> {
    crate::module::init_tracing();
    if std::env::var("DIMOS_TRANSPORT").as_deref() != Ok("zenoh") {
        return Err(io::Error::other("native RPC requires Zenoh").into());
    }
    let launch = crate::module::read_launch_config().await?;
    let registration: Registration = serde_json::from_value(launch["rpc"].clone())?;
    let methods: BTreeMap<_, _> = handlers.iter().copied().collect();
    let declared: BTreeSet<_> = registration.methods.iter().map(String::as_str).collect();
    if methods.len() != handlers.len()
        || declared.len() != registration.methods.len()
        || declared != methods.keys().copied().collect()
        || methods
            .keys()
            .any(|name| name.starts_with('_') || name.contains('/') || name.contains('*'))
        || registration.name.is_empty()
        || registration.name.contains('*')
    {
        return Err(io::Error::other("native RPC declarations do not match handlers").into());
    }
    let transport = ZenohTransport::from_launch(&launch).await?;
    let prefix = format!("dimos/rpc/v1/{}", registration.name);
    let queryable = transport
        .session()
        .declare_queryable(format!("{prefix}/*"))
        .complete(true)
        .await?;
    loop {
        let query = queryable.recv_async().await?;
        let payload = query
            .payload()
            .map(|p| p.to_bytes().into_owned())
            .unwrap_or_default();
        let route = query
            .key_expr()
            .as_str()
            .strip_prefix("dimos/rpc/v1/")
            .unwrap_or_default();
        if let Some(reply) = respond(&payload, route, &methods, &registration.token) {
            query
                .reply(query.key_expr().clone(), serde_json::to_vec(&reply)?)
                .encoding(Encoding::APPLICATION_JSON)
                .await?;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handler_panic_does_not_stop_dispatch() {
        let methods: BTreeMap<&str, Handler> = BTreeMap::from([
            ("panic", (|_| panic!("test panic")) as Handler),
            ("health", (|_| Ok(json!({"ok": true}))) as Handler),
        ]);
        for (method, expected) in [
            ("panic", error(json!(1), -32603, "Handler panicked")),
            (
                "health",
                json!({"jsonrpc": "2.0", "id": 1, "result": {"ok": true}}),
            ),
        ] {
            let route = format!("Toy/{method}");
            let request = json!({"jsonrpc": "2.0", "id": 1, "method": route}).to_string();
            assert_eq!(
                respond(request.as_bytes(), &route, &methods, "test"),
                Some(expected)
            );
        }
    }

    #[test]
    fn rejects_bad_wire_requests() {
        let methods = BTreeMap::new();
        for (payload, code) in [
            ("not json", -32700),
            (r#"{"jsonrpc":"1.0","id":1,"method":"Toy/plan"}"#, -32600),
            (r#"{"jsonrpc":"2.0","id":1,"method":"Other/plan"}"#, -32600),
            (
                r#"{"jsonrpc":"2.0","id":1,"method":"Toy/plan","params":[]}"#,
                -32602,
            ),
            (r#"{"jsonrpc":"2.0","id":1,"method":"Toy/plan"}"#, -32601),
        ] {
            let response = respond(payload.as_bytes(), "Toy/plan", &methods, "test").unwrap();
            assert_eq!(response["error"]["code"], code);
        }
    }
}
