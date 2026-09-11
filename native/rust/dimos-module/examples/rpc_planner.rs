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

#[tokio::main]
async fn main() -> zenoh::Result<()> {
    rpc::run(&[("plan", plan)]).await
}
