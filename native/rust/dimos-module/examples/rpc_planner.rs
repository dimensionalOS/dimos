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

use dimos_module::rpc::{self, Error};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Plan {
    start: [f64; 3],
    goal: [f64; 3],
}

fn plan(params: Value) -> Result<Value, Error> {
    let request: Plan = serde_json::from_value(params).map_err(|e| Error {
        code: -32602,
        message: e.to_string(),
    })?;
    // A wire-test result, not a collision-checked route.
    Ok(json!({"waypoints": [request.start, request.goal], "toy": true}))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Pause {
    seconds: f64,
}

static PAUSE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Answers after `seconds`, so a test can hold one call open while it makes another.
fn pause(params: Value) -> Result<Value, Error> {
    let invalid = |message: String| Error {
        code: -32602,
        message,
    };
    let request: Pause = serde_json::from_value(params).map_err(|e| invalid(e.to_string()))?;
    let duration =
        Duration::try_from_secs_f64(request.seconds).map_err(|e| invalid(e.to_string()))?;
    PAUSE_ACTIVE.store(true, Ordering::SeqCst);
    std::thread::sleep(duration);
    PAUSE_ACTIVE.store(false, Ordering::SeqCst);
    Ok(json!({"paused": request.seconds}))
}

fn pause_active(_params: Value) -> Result<Value, Error> {
    Ok(json!(PAUSE_ACTIVE.load(Ordering::SeqCst)))
}

#[tokio::main]
async fn main() -> zenoh::Result<()> {
    rpc::run(&[
        ("plan", plan),
        ("pause", pause),
        ("pause_active", pause_active),
    ])
    .await
}
