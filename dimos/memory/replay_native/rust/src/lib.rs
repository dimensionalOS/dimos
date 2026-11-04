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

//! Replays a memory recording: one output port per recorded stream, publishing the stored
//! LCM bytes at recorded timing. The native half of `dimos.memory.replay_native.native`.

use std::path::Path;
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use dimos_module::{Builder, Module, Output};
use tokio::task::{JoinHandle, JoinSet};
use tracing::{info, warn};

pub mod source;

use source::{Source, Storage, WindowStats};

#[dimos_module::native_config]
#[derive(Clone)]
pub struct ReplayConfig {
    /// Path to the `.db` or `.mcap`; empty leaves the module idle.
    pub db: String,
    #[validate(range(exclusive_min = 0.0))]
    pub speed: f64,
    pub r#loop: bool,
    /// Exit once the whole recording has played (ignored with `loop`).
    pub exit_on_complete: bool,
    /// Seconds after the recording's first message to start from.
    #[validate(range(min = 0.0))]
    pub seek: f64,
    /// Seconds to play from the start; 0 plays to the end.
    #[validate(range(min = 0.0))]
    pub duration: f64,
    /// Recorded-time gap between the end of one loop pass and the start of the next.
    #[validate(range(min = 0.0))]
    pub loop_gap: f64,
    /// Period at which a single-timestamp stream is republished.
    #[validate(range(exclusive_min = 0.0))]
    pub static_period: f64,
    /// Output ports, each named after the recorded stream it replays.
    pub ports: Vec<String>,
}

// Output<T> encodes from &T, so the payload type is Vec<u8> and this takes &Vec<u8>.
#[allow(clippy::ptr_arg)]
fn raw_bytes(data: &Vec<u8>) -> Vec<u8> {
    data.clone()
}

/// What to replay, worked out from the recording and the config before anything publishes.
#[derive(Debug, PartialEq)]
pub struct Plan {
    pub start: f64,
    pub end: f64,
    /// Streams replayed at recorded timing.
    pub timed: Vec<String>,
    /// Streams whose rows all carry one timestamp, republished every `static_period`.
    pub statics: Vec<String>,
    /// `(first, last)` timestamp over the timed streams, None when there are none.
    pub span: Option<(f64, f64)>,
}

impl Plan {
    pub fn new(source: &Source, config: &ReplayConfig) -> Result<Self> {
        let storages = source.storages();
        let mut ports = Vec::new();
        for port in &config.ports {
            match storages.get(port) {
                // A port declared on a subclass that this recording lacks stays silent.
                None => warn!(
                    "{port} is not a stream in {}; it will not publish",
                    config.db
                ),
                Some(Storage::Unsupported(codec)) => bail!(
                    "{port} is stored as {codec}; only LCM-encoded streams (lcm, jpeg, lz4+lcm) \
                     replay natively, leave it out with topics="
                ),
                Some(_) => ports.push(port.clone()),
            }
        }
        let first = source.recording_first()?.unwrap_or(0.0);
        let start = first + config.seek;
        let end = if config.duration > 0.0 {
            start + config.duration
        } else {
            f64::INFINITY
        };

        let mut plan = Plan {
            start,
            end,
            timed: Vec::new(),
            statics: Vec::new(),
            span: None,
        };
        for port in &ports {
            let WindowStats {
                count,
                first_ts,
                last_ts,
            } = source.stats(port, start, end)?;
            if count == 0 {
                continue;
            }
            // camera_info recorded from a constant stamped at import: replaying it would hold
            // the anchor back to that stamp, so it is republished instead.
            if count > 1 && first_ts == last_ts {
                plan.statics.push(port.clone());
                continue;
            }
            plan.timed.push(port.clone());
            plan.span = Some(match plan.span {
                None => (first_ts, last_ts),
                Some((lo, hi)) => (lo.min(first_ts), hi.max(last_ts)),
            });
        }
        Ok(plan)
    }
}

/// Wall-clock offset from the anchor at which a message recorded at `ts` is due, on loop
/// pass `pass` (0-based) of a span `(first, last)`.
pub fn due(ts: f64, pass: u32, span: (f64, f64), speed: f64, loop_gap: f64) -> Duration {
    let period = span.1 - span.0 + loop_gap;
    let offset = (ts - span.0 + f64::from(pass) * period) / speed;
    Duration::from_secs_f64(offset.max(0.0))
}

pub struct MemoryReplay {
    config: ReplayConfig,
    outputs: Vec<(String, Output<Vec<u8>>)>,
    /// Dropped to stop the player: the first ends its waits, the second a publish stuck on a full queue.
    stop: Option<(Sender<()>, tokio::sync::watch::Sender<()>)>,
    player: Option<JoinHandle<Result<()>>>,
    statics: JoinSet<()>,
}

impl Module for MemoryReplay {
    type Config = ReplayConfig;

    fn build(builder: &mut Builder, config: Self::Config) -> Self {
        let outputs = config
            .ports
            .iter()
            .map(|port| (port.clone(), builder.output(port, raw_bytes)))
            .collect();
        Self {
            config,
            outputs,
            stop: None,
            player: None,
            statics: JoinSet::new(),
        }
    }

    async fn setup(&mut self) {
        let config = self.config.clone();
        if config.db.is_empty() {
            info!("no db: nothing to replay");
            return;
        }
        let source = Source::open(Path::new(&config.db))
            .unwrap_or_else(|e| panic!("cannot open {}: {e:#}", config.db));
        let plan = Plan::new(&source, &config).unwrap_or_else(|e| panic!("{e:#}"));
        info!(timed = ?plan.timed, statics = ?plan.statics, start = plan.start, "replaying {}", config.db);

        let output_of = |name: &str| {
            self.outputs
                .iter()
                .find(|(port, _)| port == name)
                .map(|(_, out)| out.clone())
                .unwrap()
        };
        for name in &plan.statics {
            let data = source
                .first(name, plan.start, plan.end)
                .and_then(|d| source.storages()[name].to_wire(d.context("no row")?))
                .unwrap_or_else(|e| panic!("{name}: {e:#}"));
            let output = output_of(name);
            let period = Duration::from_secs_f64(config.static_period);
            self.statics.spawn(async move {
                let mut ticks =
                    tokio::time::interval_at(tokio::time::Instant::now() + period, period);
                loop {
                    ticks.tick().await;
                    if output.publish(&data).await.is_err() {
                        return;
                    }
                }
            });
        }

        let outputs: Vec<Output<Vec<u8>>> = plan.timed.iter().map(|n| output_of(n)).collect();
        let (stop_tx, stop_rx) = channel();
        let (cancel_tx, mut cancel_rx) = tokio::sync::watch::channel(());
        self.stop = Some((stop_tx, cancel_tx));
        let runtime = tokio::runtime::Handle::current();
        self.player = Some(tokio::task::spawn_blocking(move || {
            play(&source, &plan, &config, &stop_rx, &mut |index, data| {
                runtime.block_on(async {
                    tokio::select! {
                        published = outputs[index].publish(&data) => published.is_ok(),
                        _ = cancel_rx.changed() => false,
                    }
                })
            })
        }));
    }

    async fn handle(&mut self) {
        if let Some(player) = self.player.as_mut() {
            match player.await {
                Ok(Ok(())) => info!("replay finished"),
                Ok(Err(e)) => panic!("replay failed: {e:#}"),
                Err(e) => std::panic::resume_unwind(e.into_panic()),
            }
            self.player = None;
            if self.config.exit_on_complete && !self.config.r#loop {
                return;
            }
        }
        // Like the Python module, stay up (static streams keep publishing) until stopped.
        std::future::pending::<()>().await;
    }

    async fn teardown(&mut self) {
        self.stop = None;
        self.statics.abort_all();
        if let Some(player) = self.player.take() {
            if let Ok(Err(e)) = player.await {
                warn!("replay stopped with an error: {e:#}");
            }
        }
    }
}

/// Publishes the plan's timed streams at recorded timing until done or `stop` drops.
pub fn play(
    source: &Source,
    plan: &Plan,
    config: &ReplayConfig,
    stop: &Receiver<()>,
    publish: &mut dyn FnMut(usize, Vec<u8>) -> bool,
) -> Result<()> {
    let Some(span) = plan.span else { return Ok(()) };
    let storages: Vec<Storage> = plan
        .timed
        .iter()
        .map(|n| source.storages()[n].clone())
        .collect();
    let anchor = Instant::now();
    let mut stopped = false;
    for pass in 0.. {
        let mut emitted = false;
        source.for_each(&plan.timed, plan.start, plan.end, &mut |index, ts, data| {
            let wait = (anchor + due(ts, pass, span, config.speed, config.loop_gap))
                .saturating_duration_since(Instant::now());
            let stop_now = if wait.is_zero() {
                stop.try_recv() == Err(TryRecvError::Disconnected)
            } else {
                stop.recv_timeout(wait) != Err(RecvTimeoutError::Timeout)
            };
            if stop_now {
                stopped = true;
                return Ok(false);
            }
            let wire = storages[index]
                .to_wire(data)
                .with_context(|| plan.timed[index].clone())?;
            emitted = true;
            stopped = !publish(index, wire);
            Ok(!stopped)
        })?;
        if stopped || !config.r#loop || !emitted {
            break;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
