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

use std::collections::HashMap;
use std::io;
use std::sync::Arc;

use anyhow::{anyhow, Context, Result};
use dimos_memory_recorder::{RecorderConfig, RecorderEngine};
use dimos_module::transport::Dispatch;
use dimos_module::{LcmTransport, Transport, ZenohTransport};
use serde::Deserialize;
use serde_json::Value;
use tokio::io::{AsyncBufReadExt, BufReader};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

#[derive(Deserialize)]
struct LaunchConfig {
    topics: HashMap<String, String>,
    config: RecorderConfig,
}

#[tokio::main]
async fn main() {
    init_tracing();
    if let Err(error) = run().await {
        tracing::error!(error = %format!("{error:#}"), "memory recorder failed");
        std::process::exit(1);
    }
}

async fn run() -> Result<()> {
    let (launch_value, launch) = read_launch().await?;
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => run_with(LcmTransport::new().await?, launch).await,
        Ok("zenoh") => run_with(ZenohTransport::from_launch(&launch_value).await?, launch).await,
        other => Err(anyhow!(
            "DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got {other:?}"
        )),
    }
}

async fn run_with<T: Transport>(transport: T, launch: LaunchConfig) -> Result<()> {
    launch.config.validate()?;
    let engine = RecorderEngine::start(launch.config.clone())?;
    let handle = engine.handle();

    for stream in &launch.config.streams {
        let topic = launch
            .topics
            .get(&stream.port)
            .with_context(|| format!("no topic supplied for recorder port {:?}", stream.port))?;
        let stream = Arc::new(stream.clone());
        let callback_stream = Arc::clone(&stream);
        let callback_handle = handle.clone();
        let callback: Dispatch =
            Arc::new(move |data| callback_handle.record(Arc::clone(&callback_stream), data));
        transport
            .subscribe(topic, callback)
            .await
            .with_context(|| format!("failed to subscribe to {topic:?}"))?;
        info!(port = %stream.port, topic = %topic, store_stream = %stream.name, codec = ?stream.codec, "recording stream");
    }

    shutdown_signal().await?;
    let stats = engine.shutdown()?;
    if stats.encode_errors > 0 {
        warn!(
            encode_errors = stats.encode_errors,
            "recording stopped with encoding errors"
        );
    }
    Ok(())
}

#[cfg(unix)]
async fn shutdown_signal() -> io::Result<()> {
    use tokio::signal::unix::{signal, SignalKind};

    let mut terminate = signal(SignalKind::terminate())?;
    tokio::select! {
        result = tokio::signal::ctrl_c() => result,
        _ = terminate.recv() => Ok(()),
    }
}

#[cfg(not(unix))]
async fn shutdown_signal() -> io::Result<()> {
    tokio::signal::ctrl_c().await
}

async fn read_launch() -> Result<(Value, LaunchConfig)> {
    let mut line = String::new();
    BufReader::new(tokio::io::stdin())
        .read_line(&mut line)
        .await
        .context("failed to read launch config")?;
    let value: Value = serde_json::from_str(line.trim()).context("invalid launch config JSON")?;
    let launch = serde_json::from_value(value.clone()).context("invalid memory recorder config")?;
    Ok((value, launch))
}

fn init_tracing() {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .json()
        .with_writer(io::stderr)
        .with_env_filter(filter)
        .try_init();
}
