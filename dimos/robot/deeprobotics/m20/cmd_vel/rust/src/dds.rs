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
//! rl_deploy's own DDS topics, from pure Rust (rustdds speaks RTPS to its Fast-DDS participant; no DrDDS process,
//! basic_server or bridge in between): a REAL_STEER writer, MOTION_INFO and HES_STATUS readers, and discovery, which
//! says when rl_deploy's REAL_STEER reader is matched and when another REAL_STEER writer (basic_server) is up.

use std::collections::HashSet;
use std::time::Instant;

use futures::StreamExt;
use rustdds::dds::statusevents::{DomainParticipantStatusEvent, StatusEvented};
use rustdds::no_key::{DataReaderStream, DataWriter};
use rustdds::policy::{Durability, History, Reliability};
use rustdds::*;
use tokio::sync::watch;
use tracing::{info, warn};

use crate::gate::{Gate, GateConfig, Velocity, Write};
use crate::wire::{Header, MotionInfo, StdMsgInt32, Steer};

#[derive(Clone, Debug)]
pub struct Topics {
    pub domain_id: u16,
    pub steer_topic: String,
    pub steer_type: String,
    pub motion_info_topic: String,
    pub motion_info_type: String,
    pub hes_status_topic: String,
    pub hes_status_type: String,
}

/// What the module publishes after each MOTION_INFO.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Status {
    pub ready: bool,
    pub motion_state: i32,
    pub gait: u32,
}

/// rl_deploy's REAL_STEER, as a writer of this process.
pub struct Steering {
    participant: DomainParticipant,
    writer: DataWriter<Steer>,
    sequence: u64,
    pub steer_topic: String,
}

/// The two readers, as async streams.
pub struct Readers {
    pub motion_info: DataReaderStream<MotionInfo>,
    pub hes_status: DataReaderStream<StdMsgInt32>,
}

fn qos(durability: Durability) -> QosPolicies {
    // as rl_deploy's endpoints (discovery): reliable, MOTION_INFO and REAL_STEER volatile, HES_STATUS transient-local
    QosPolicyBuilder::new()
        .reliability(Reliability::Reliable {
            max_blocking_time: rustdds::Duration::from_millis(100),
        })
        .durability(durability)
        .history(History::KeepLast { depth: 1 })
        .build()
}

impl Steering {
    pub fn open(topics: &Topics) -> Result<(Self, Readers), String> {
        let participant =
            DomainParticipant::new(topics.domain_id).map_err(|e| format!("participant: {e:?}"))?;
        let topic = |name: &str, type_name: &str, durability| {
            participant
                .create_topic(
                    name.into(),
                    type_name.into(),
                    &qos(durability),
                    TopicKind::NoKey,
                )
                .map_err(|e| format!("topic {name}: {e:?}"))
        };
        let steer = topic(
            &topics.steer_topic,
            &topics.steer_type,
            Durability::Volatile,
        )?;
        let motion_info = topic(
            &topics.motion_info_topic,
            &topics.motion_info_type,
            Durability::Volatile,
        )?;
        let hes_status = topic(
            &topics.hes_status_topic,
            &topics.hes_status_type,
            Durability::TransientLocal,
        )?;
        let publisher = participant
            .create_publisher(&qos(Durability::Volatile))
            .map_err(|e| format!("publisher: {e:?}"))?;
        let subscriber = participant
            .create_subscriber(&qos(Durability::Volatile))
            .map_err(|e| format!("subscriber: {e:?}"))?;
        let writer = publisher
            .create_datawriter_no_key_cdr::<Steer>(&steer, None)
            .map_err(|e| format!("REAL_STEER writer: {e:?}"))?;
        let motion_info = subscriber
            .create_datareader_no_key_cdr::<MotionInfo>(&motion_info, None)
            .map_err(|e| format!("MOTION_INFO reader: {e:?}"))?
            .async_sample_stream();
        let hes_status = subscriber
            .create_datareader_no_key_cdr::<StdMsgInt32>(&hes_status, None)
            .map_err(|e| format!("HES_STATUS reader: {e:?}"))?
            .async_sample_stream();
        Ok((
            Self {
                participant,
                writer,
                sequence: 0,
                steer_topic: topics.steer_topic.clone(),
            },
            Readers {
                motion_info,
                hes_status,
            },
        ))
    }

    pub fn participant(&self) -> &DomainParticipant {
        &self.participant
    }

    /// This participant's GUID; its prefix is every local endpoint's.
    pub fn own(&self) -> GUID {
        self.participant.guid()
    }

    pub fn writer_guid(&self) -> GUID {
        self.writer.guid()
    }

    pub fn write(&mut self, velocity: Velocity) {
        let steer = Steer {
            header: Header::now(self.sequence),
            x: velocity.x,
            y: velocity.y,
            yaw: velocity.yaw,
            ..Steer::default()
        };
        self.sequence += 1;
        if let Err(error) = self.writer.write(steer, None) {
            warn!(?error, "cannot write REAL_STEER");
        }
    }

    /// Wait until every matched reader acknowledged what was written.
    pub fn acknowledged(&self, max_wait: std::time::Duration) -> bool {
        self.writer
            .wait_for_acknowledgments(max_wait)
            .unwrap_or(false)
    }

    fn apply(&mut self, write: Write) {
        match write {
            Write::Nothing => {}
            Write::Velocity(velocity) => self.write(velocity),
            Write::FinalZero => {
                self.write(Velocity::ZERO);
                self.write(Velocity::ZERO);
            }
        }
    }
}

/// What discovery says about REAL_STEER: the remote readers matched with this writer (rl_deploy's, when it runs),
/// and the other writers (basic_server, a vendor bridge, another copy of this module).
#[derive(Default)]
pub struct Discovery {
    matched: HashSet<GUID>,
    writers: HashSet<GUID>,
}

impl Discovery {
    /// Track one discovery event; true when either set changed.
    pub fn see(
        &mut self,
        event: &DomainParticipantStatusEvent,
        topic: &str,
        own: GUID,
        writer: GUID,
    ) -> bool {
        match event {
            DomainParticipantStatusEvent::WriterDetected { writer: remote } => {
                remote.topic_name == topic
                    && remote.guid.prefix != own.prefix
                    && self.writers.insert(remote.guid)
            }
            DomainParticipantStatusEvent::RemoteReaderMatched {
                local_writer,
                remote_reader,
            } => *local_writer == writer && self.matched.insert(*remote_reader),
            DomainParticipantStatusEvent::WriterLost { guid, .. } => self.writers.remove(guid),
            DomainParticipantStatusEvent::ReaderLost { guid, .. } => self.matched.remove(guid),
            DomainParticipantStatusEvent::ParticipantLost { id, .. } => {
                let before = self.writers.len() + self.matched.len();
                self.writers.retain(|guid| guid.prefix != *id);
                self.matched.retain(|guid| guid.prefix != *id);
                before != self.writers.len() + self.matched.len()
            }
            _ => false,
        }
    }

    pub fn matched(&self) -> Vec<GUID> {
        self.matched.iter().copied().collect()
    }

    pub fn other_writers(&self) -> Vec<GUID> {
        self.writers.iter().copied().collect()
    }
}

/// The command loop: cmd_vel in, REAL_STEER out, until `stop` is set; a final zero if a command was live.
pub async fn run(
    topics: Topics,
    gate_config: GateConfig,
    exclusive: bool,
    release_ack_wait: std::time::Duration,
    mut commands: watch::Receiver<Option<(f64, f64, f64)>>,
    status: watch::Sender<Option<Status>>,
    mut stop: watch::Receiver<bool>,
) {
    let (mut steering, mut readers) = match Steering::open(&topics) {
        Ok(opened) => opened,
        Err(error) => {
            warn!(%error, "cannot join DDS; no velocity reaches rl_deploy");
            return;
        }
    };
    let (own, writer) = (steering.own(), steering.writer_guid());
    info!(writer = ?steering.writer_guid(), topic = %topics.steer_topic, "REAL_STEER writer up; waiting for rl_deploy's reader and MOTION_INFO");
    let listener = steering.participant().status_listener();
    let mut events = listener.as_async_status_stream();
    let mut gate = Gate::new(gate_config);
    let mut discovery = Discovery::default();
    let mut last_ready = None;
    loop {
        tokio::select! {
            changed = commands.changed() => {
                if changed.is_err() {
                    break;
                }
                let latest = *commands.borrow_and_update();
                if let Some((x, y, yaw)) = latest {
                    steering.apply(gate.on_command(x, y, yaw, Instant::now()));
                }
            }
            sample = readers.motion_info.next() => {
                let Some(Ok(sample)) = sample else { continue };
                let info = sample.into_value();
                let now = Instant::now();
                steering.apply(gate.on_motion_info(info.motion_state, now));
                let ready = gate.ready(now);
                if last_ready != Some(ready) {
                    info!(ready, motion_state = info.motion_state, gait = info.gait, "command path ready changed");
                    last_ready = Some(ready);
                }
                status.send_replace(Some(Status { ready, motion_state: info.motion_state, gait: info.gait }));
            }
            sample = readers.hes_status.next() => {
                let Some(Ok(sample)) = sample else { continue };
                let hes = sample.into_value().value;
                info!(hes, "HES_STATUS");
                gate.on_hes_status(hes);
            }
            event = events.next() => {
                let Some(event) = event else { continue };
                if !discovery.see(&event, &steering.steer_topic, own, writer) {
                    continue;
                }
                let matched = discovery.matched();
                gate.set_matched(!matched.is_empty());
                let others = discovery.other_writers();
                let blocked = exclusive && !others.is_empty();
                if blocked != gate.blocked() {
                    // released before blocking, so the last word before handing over is a zero
                    if blocked {
                        steering.apply(gate.release());
                    }
                    gate.set_blocked(blocked);
                    warn!(blocked, writers = ?others, "other REAL_STEER writers; writing nothing while one is up (exclusive)");
                }
                info!(readers = ?matched, other_writers = ?others, "REAL_STEER discovery changed");
            }
            _ = stop.changed() => break,
        }
    }
    steering.apply(gate.release());
    steering.acknowledged(release_ack_wait);
    info!("command path stopped");
}
