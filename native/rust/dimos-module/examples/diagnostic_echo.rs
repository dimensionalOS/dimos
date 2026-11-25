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

//! Isolated Python/Rust conformance peer. Never connects to default robot discovery.
use dimos_module::{diagnostics, Transport, ZenohTransport};
use std::{sync::Arc, time::Duration};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let endpoint = std::env::var("DIMOS_DIAGNOSTICS_TEST_ENDPOINT")?;
    let key = std::env::var("DIMOS_DIAGNOSTICS_TEST_KEY")?;
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"client\"")?;
    config.insert_json5("connect/endpoints", &serde_json::to_string(&[endpoint])?)?;
    config.insert_json5("scouting/multicast/enabled", "false")?;
    config.insert_json5("scouting/gossip/enabled", "false")?;
    let session = zenoh::open(config).await.map_err(|e| e.to_string())?;
    let readiness = session.declare_publisher(format!("{key}/output")).await?;
    let matches = readiness.matching_listener().await?;
    if !readiness.matching_status().await?.matching() {
        tokio::time::timeout(Duration::from_secs(10), matches.recv_async()).await??;
    }
    let transport = ZenohTransport::from_session(session.clone());
    let (tx, mut rx) = tokio::sync::mpsc::channel(8);
    transport
        .subscribe(
            &format!("{key}/input"),
            Arc::new(move |bytes: &[u8]| {
                let _ = tx.try_send(diagnostics::Queued::capture(
                    bytes.to_vec(),
                    "echo",
                    diagnostics::current_context(),
                ));
            }),
        )
        .await?;
    println!("ready");
    let sample = tokio::time::timeout(Duration::from_secs(10), rx.recv())
        .await?
        .ok_or("closed")?;
    sample
        .handle(|bytes| async {
            let output = diagnostics::publication("echo/output");
            diagnostics::scope(output, transport.publish(&format!("{key}/output"), bytes)).await
        })
        .await?;
    diagnostics::shutdown();
    session.close().await?;
    Ok(())
}
