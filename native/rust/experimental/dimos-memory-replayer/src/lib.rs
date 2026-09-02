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

mod source;

use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::time::Duration;

use anyhow::{anyhow, Result};
use dimos_module::{Builder, Module, Output};
use serde::{Deserialize, Serialize};
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::{mpsc, oneshot};
use tokio::task::JoinHandle;
use tokio::time::Instant;
use tracing::{error, info};

use source::{Observation, SqliteReplaySource};

const LOOP_GAP: Duration = Duration::from_millis(50);

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayStoreConfig {
    pub path: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StreamConfig {
    pub port: String,
    pub name: String,
    pub payload_type: String,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(untagged)]
pub enum NullableF64 {
    Value(f64),
    Null,
}

impl NullableF64 {
    fn value(self) -> Option<f64> {
        match self {
            Self::Value(value) => Some(value),
            Self::Null => None,
        }
    }
}

#[dimos_module::native_config]
#[derive(Clone)]
pub struct ReplayConfig {
    pub store: ReplayStoreConfig,
    #[validate(range(min = 0.000000001))]
    pub speed: f64,
    pub seek: NullableF64,
    pub duration: NullableF64,
    pub from_timestamp: NullableF64,
    pub r#loop: bool,
    pub streams: Vec<StreamConfig>,
}

impl ReplayConfig {
    fn validate_replay(&self) -> Result<()> {
        if !self.speed.is_finite() || self.speed <= 0.0 {
            return Err(anyhow!("speed must be finite and greater than zero"));
        }
        if self.seek.value().is_some() && self.from_timestamp.value().is_some() {
            return Err(anyhow!("seek and from_timestamp are mutually exclusive"));
        }
        for (name, value) in [
            ("seek", self.seek.value()),
            ("duration", self.duration.value()),
        ] {
            if value.is_some_and(|value| !value.is_finite() || value < 0.0) {
                return Err(anyhow!("{name} must be finite and non-negative"));
            }
        }
        if self
            .from_timestamp
            .value()
            .is_some_and(|value| !value.is_finite())
        {
            return Err(anyhow!("from_timestamp must be finite"));
        }
        if self.streams.is_empty() {
            return Err(anyhow!("at least one replay output is required"));
        }
        let mut ports = HashSet::new();
        let mut names = HashSet::new();
        for stream in &self.streams {
            if !ports.insert(&stream.port) {
                return Err(anyhow!("duplicate replay port {:?}", stream.port));
            }
            if !names.insert(&stream.name) {
                return Err(anyhow!("duplicate replay stream {:?}", stream.name));
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
enum Control {
    Pause,
    Resume,
    SetSpeed(f64),
}

struct ControlRequest {
    control: Control,
    reply: oneshot::Sender<Result<(), String>>,
}

struct ReplayClock {
    replay_anchor: f64,
    wall_anchor: Instant,
    speed: f64,
    paused: bool,
}

impl ReplayClock {
    fn new(replay_anchor: f64, speed: f64) -> Self {
        Self {
            replay_anchor,
            wall_anchor: Instant::now(),
            speed,
            paused: false,
        }
    }

    fn position(&self) -> f64 {
        if self.paused {
            self.replay_anchor
        } else {
            self.replay_anchor + self.wall_anchor.elapsed().as_secs_f64() * self.speed
        }
    }

    fn deadline(&self, timestamp: f64) -> Instant {
        self.wall_anchor
            + Duration::from_secs_f64(((timestamp - self.replay_anchor) / self.speed).max(0.0))
    }

    fn restarted(&self, replay_anchor: f64) -> Self {
        Self {
            replay_anchor,
            wall_anchor: Instant::now(),
            speed: self.speed,
            paused: self.paused,
        }
    }

    fn apply(&mut self, control: Control) -> Result<(), String> {
        match control {
            Control::Pause => {
                if !self.paused {
                    self.replay_anchor = self.position();
                    self.paused = true;
                }
            }
            Control::Resume => {
                if self.paused {
                    self.wall_anchor = Instant::now();
                    self.paused = false;
                }
            }
            Control::SetSpeed(speed) => {
                if !speed.is_finite() || speed <= 0.0 {
                    return Err("speed must be finite and greater than zero".to_string());
                }
                self.replay_anchor = self.position();
                self.wall_anchor = Instant::now();
                self.speed = speed;
            }
        }
        Ok(())
    }
}

pub struct MemoryReplayer {
    config: ReplayConfig,
    outputs: HashMap<usize, Output<Vec<u8>>>,
    controls_tx: mpsc::UnboundedSender<ControlRequest>,
    controls_rx: mpsc::UnboundedReceiver<ControlRequest>,
    control_task: Option<JoinHandle<()>>,
}

impl Module for MemoryReplayer {
    type Config = ReplayConfig;

    fn build(builder: &mut Builder, config: Self::Config) -> Self {
        config
            .validate_replay()
            .expect("invalid memory replay configuration");
        let outputs = config
            .streams
            .iter()
            .enumerate()
            .map(|(index, stream)| {
                let output = builder.output(&stream.port, Vec::clone);
                (index, output)
            })
            .collect();
        let (controls_tx, controls_rx) = mpsc::unbounded_channel();
        Self {
            config,
            outputs,
            controls_tx,
            controls_rx,
            control_task: None,
        }
    }

    async fn setup(&mut self) {
        let sender = self.controls_tx.clone();
        self.control_task = Some(tokio::spawn(async move {
            control_loop(sender).await;
        }));
    }

    async fn handle(&mut self) {
        let config = self.config.clone();
        let source = tokio::task::spawn_blocking(move || SqliteReplaySource::prepare(&config))
            .await
            .expect("replay loader panicked")
            .expect("failed to prepare replay artifact");
        let (start, end) = source.window();
        info!(?start, ?end, "memory replayer ready");
        self.play(source).await;
    }

    async fn teardown(&mut self) {
        if let Some(task) = self.control_task.take() {
            task.abort();
        }
    }
}

impl MemoryReplayer {
    async fn play(&mut self, source: SqliteReplaySource) {
        let Some(first_timestamp) = source.first_timestamp() else {
            self.idle().await;
            return;
        };
        let mut clock = ReplayClock::new(first_timestamp, self.config.speed);
        loop {
            let mut observations = source.stream();
            while let Some(observation) = self.next_observation(&mut observations, &mut clock).await
            {
                loop {
                    while clock.paused {
                        self.apply_next_control(&mut clock).await;
                    }
                    let deadline = clock.deadline(observation.ts);
                    tokio::select! {
                        _ = tokio::time::sleep_until(deadline) => {
                            let output = self.outputs.get(&observation.stream_index)
                                .expect("validated replay output is present");
                            output.publish(&observation.data).await.expect("replay transport closed");
                            break;
                        }
                        request = self.controls_rx.recv() => {
                            if let Some(request) = request {
                                apply_control(&mut clock, request);
                            }
                        }
                    }
                }
            }

            if !self.config.r#loop {
                self.idle_with_clock(&mut clock).await;
                return;
            }
            self.loop_gap(&mut clock).await;
            clock = clock.restarted(first_timestamp);
        }
    }

    async fn next_observation(
        &mut self,
        observations: &mut mpsc::Receiver<Result<Observation>>,
        clock: &mut ReplayClock,
    ) -> Option<Observation> {
        loop {
            tokio::select! {
                observation = observations.recv() => {
                    return match observation {
                        Some(Ok(observation)) => Some(observation),
                        Some(Err(error)) => panic!("failed to read replay artifact: {error:#}"),
                        None => None,
                    };
                }
                request = self.controls_rx.recv() => {
                    if let Some(request) = request {
                        apply_control(clock, request);
                    }
                }
            }
        }
    }

    async fn apply_next_control(&mut self, clock: &mut ReplayClock) {
        if let Some(request) = self.controls_rx.recv().await {
            apply_control(clock, request);
        }
    }

    async fn loop_gap(&mut self, clock: &mut ReplayClock) {
        let sleep = tokio::time::sleep(LOOP_GAP);
        tokio::pin!(sleep);
        loop {
            tokio::select! {
                _ = &mut sleep => return,
                request = self.controls_rx.recv() => {
                    if let Some(request) = request {
                        apply_control(clock, request);
                    }
                }
            }
        }
    }

    async fn idle_with_clock(&mut self, clock: &mut ReplayClock) {
        loop {
            self.apply_next_control(clock).await;
        }
    }

    async fn idle(&mut self) {
        let mut clock = ReplayClock::new(0.0, self.config.speed);
        self.idle_with_clock(&mut clock).await;
    }
}

fn apply_control(clock: &mut ReplayClock, request: ControlRequest) {
    let result = clock.apply(request.control);
    let _ = request.reply.send(result);
}

#[derive(Deserialize)]
struct ControlEnvelope {
    dimos_control: WireControl,
}

#[derive(Deserialize)]
struct WireControl {
    id: u64,
    command: String,
    speed: Option<f64>,
}

async fn control_loop(sender: mpsc::UnboundedSender<ControlRequest>) {
    let mut lines = BufReader::new(tokio::io::stdin()).lines();
    while let Ok(Some(line)) = lines.next_line().await {
        let parsed = serde_json::from_str::<ControlEnvelope>(&line);
        let (id, result) = match parsed {
            Ok(envelope) => {
                let id = envelope.dimos_control.id;
                let control = match envelope.dimos_control.command.as_str() {
                    "pause" => Ok(Control::Pause),
                    "resume" => Ok(Control::Resume),
                    "set_speed" => envelope
                        .dimos_control
                        .speed
                        .map(Control::SetSpeed)
                        .ok_or_else(|| "set_speed requires speed".to_string()),
                    command => Err(format!("unknown replay command {command:?}")),
                };
                let result = match control {
                    Ok(control) => {
                        let (reply, response) = oneshot::channel();
                        if sender.send(ControlRequest { control, reply }).is_err() {
                            Err("replay scheduler stopped".to_string())
                        } else {
                            response
                                .await
                                .unwrap_or_else(|_| Err("replay scheduler stopped".to_string()))
                        }
                    }
                    Err(error) => Err(error),
                };
                (id, result)
            }
            Err(error) => {
                error!(%error, "invalid replay control message");
                continue;
            }
        };
        let payload = match result {
            Ok(()) => serde_json::json!({"dimos_control": {"id": id, "ok": true}}),
            Err(error) => {
                serde_json::json!({"dimos_control": {"id": id, "ok": false, "error": error}})
            }
        };
        let mut stdout = std::io::stdout().lock();
        if writeln!(stdout, "{payload}")
            .and_then(|_| stdout.flush())
            .is_err()
        {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(start_paused = true)]
    async fn speed_change_reanchors_without_jumping() {
        let mut clock = ReplayClock::new(10.0, 1.0);
        tokio::time::advance(Duration::from_secs(2)).await;
        clock.apply(Control::SetSpeed(2.0)).unwrap();
        assert!((clock.position() - 12.0).abs() < 1e-9);
        tokio::time::advance(Duration::from_secs(1)).await;
        assert!((clock.position() - 14.0).abs() < 1e-9);
    }

    #[tokio::test(start_paused = true)]
    async fn pause_and_resume_preserve_position() {
        let mut clock = ReplayClock::new(5.0, 1.0);
        tokio::time::advance(Duration::from_secs(1)).await;
        clock.apply(Control::Pause).unwrap();
        tokio::time::advance(Duration::from_secs(5)).await;
        assert!((clock.position() - 6.0).abs() < 1e-9);
        clock.apply(Control::Resume).unwrap();
        tokio::time::advance(Duration::from_secs(1)).await;
        assert!((clock.position() - 7.0).abs() < 1e-9);
    }

    #[tokio::test(start_paused = true)]
    async fn loop_restart_preserves_pause_and_speed() {
        let mut clock = ReplayClock::new(5.0, 1.0);
        clock.apply(Control::SetSpeed(3.0)).unwrap();
        clock.apply(Control::Pause).unwrap();

        let restarted = clock.restarted(10.0);

        assert!(restarted.paused);
        assert_eq!(restarted.speed, 3.0);
        assert_eq!(restarted.position(), 10.0);
    }
}
