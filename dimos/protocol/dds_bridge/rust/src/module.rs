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

//! The bridge as a native module: one output port per bridged topic, named by its DimOS stream.

use std::borrow::Cow;
use std::collections::HashSet;
use std::time::Duration;

use dimos_module::{native_config, warn_throttled, Builder, Module, Output};
use futures::StreamExt;
use rustdds::{DomainParticipant, QosPolicyBuilder, TopicKind};
use serde::{Deserialize, Serialize};
use tracing::{error, info, warn};
use validator::ValidationError;

use crate::dds::{self, RawCdrAdapter, TopicQos};
use crate::translate::{self, DimosMsg, Translation};

/// Mirrors `DdsBridgeConfig` (module.py), which owns every default.
#[native_config]
#[validate(schema(function = "validate_bridges"))]
pub struct Config {
    pub domain_id: u16,
    /// Interface addresses DDS may use; empty means all.
    pub network_addresses: Vec<String>,
    pub dds_to_dimos: Vec<DdsToDimos>,
}

/// One DDS topic republished as one DimOS stream.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DdsToDimos {
    /// The DDS topic name as on the wire; ROS 2 prefixes its topics with `rt/`.
    pub dds_topic: String,
    /// The ROS 2 interface, a key of the translation table.
    pub dds_type: String,
    /// The registered DDS type name; null means the ROS 2 convention for `dds_type`.
    pub dds_type_name: Option<String>,
    /// The DimOS stream name; the port and channel this topic lands on.
    pub zenoh_topic: String,
    pub qos: TopicQos,
}

impl DdsToDimos {
    fn type_name(&self) -> String {
        self.dds_type_name
            .clone()
            .or_else(|| translate::ros2_dds_type_name(&self.dds_type))
            .expect("validated dds_type")
    }
}

fn rejection(code: &'static str, message: String) -> ValidationError {
    ValidationError::new(code).with_message(Cow::Owned(message))
}

fn validate_bridges(config: &Config) -> Result<(), ValidationError> {
    let mut streams = HashSet::new();
    for bridge in &config.dds_to_dimos {
        if translate::lookup(&bridge.dds_type).is_none() {
            return Err(rejection(
                "unknown_dds_type",
                format!(
                    "DDS topic {:?} has type {:?}, which the bridge cannot translate (supported: {})",
                    bridge.dds_topic,
                    bridge.dds_type,
                    translate::supported_types().join(", ")
                ),
            ));
        }
        if bridge.dds_topic.is_empty() || bridge.zenoh_topic.is_empty() {
            return Err(rejection(
                "empty_topic",
                format!("bridge {bridge:?} names an empty topic"),
            ));
        }
        if !streams.insert(bridge.zenoh_topic.as_str()) {
            return Err(rejection(
                "duplicate_stream",
                format!(
                    "two DDS topics publish onto the DimOS stream {:?}",
                    bridge.zenoh_topic
                ),
            ));
        }
        if bridge.qos.history_depth < 1 {
            return Err(rejection(
                "history_depth",
                format!(
                    "DDS topic {:?}: history_depth must be at least 1",
                    bridge.dds_topic
                ),
            ));
        }
    }
    Ok(())
}

/// How often a topic that keeps failing logs it again.
const WARN_EVERY: Duration = Duration::from_secs(5);

struct Route {
    bridge: DdsToDimos,
    translation: &'static Translation,
    output: Output<DimosMsg>,
}

pub struct DdsBridge {
    config: Config,
    routes: Vec<Route>,
    participant: Option<DomainParticipant>,
    readers: Vec<tokio::task::JoinHandle<()>>,
}

impl Module for DdsBridge {
    type Config = Config;

    fn build(builder: &mut Builder, config: Config) -> Self {
        let routes = config
            .dds_to_dimos
            .iter()
            .map(|bridge| {
                let translation = translate::lookup(&bridge.dds_type).expect("validated dds_type");
                let output = builder.output(&bridge.zenoh_topic, DimosMsg::encode);
                // The python side picks the channel's message type from its own copy of the table.
                let unmapped = output.topic == format!("/{}", bridge.zenoh_topic);
                if !unmapped && !output.topic.ends_with(translation.dimos_type) {
                    fatal(format!(
                        "stream {:?} is on channel {:?}, but the bridge publishes {} there",
                        bridge.zenoh_topic, output.topic, translation.dimos_type
                    ));
                }
                Route {
                    bridge: bridge.clone(),
                    translation,
                    output,
                }
            })
            .collect();
        Self {
            config,
            routes,
            participant: None,
            readers: Vec::new(),
        }
    }

    async fn setup(&mut self) {
        let participant = dds::join_domain(self.config.domain_id, &self.config.network_addresses)
            .unwrap_or_else(|e| fatal(e.to_string()));
        let subscriber = participant
            .create_subscriber(&QosPolicyBuilder::new().build())
            .unwrap_or_else(|e| fatal(format!("DDS subscriber: {e:?}")));
        for route in &self.routes {
            let bridge = &route.bridge;
            let qos = bridge.qos.to_dds();
            let topic = participant
                .create_topic(
                    bridge.dds_topic.clone(),
                    bridge.type_name(),
                    &qos,
                    TopicKind::NoKey,
                )
                .unwrap_or_else(|e| fatal(format!("DDS topic {:?}: {e:?}", bridge.dds_topic)));
            let reader = subscriber
                .create_datareader_no_key::<dds::RawCdr, RawCdrAdapter>(&topic, Some(qos))
                .unwrap_or_else(|e| fatal(format!("DDS reader for {:?}: {e:?}", bridge.dds_topic)));
            info!(
                dds_topic = %bridge.dds_topic,
                dds_type_name = %bridge.type_name(),
                stream = %route.output.topic,
                "bridging"
            );
            self.readers.push(tokio::spawn(forward(
                reader,
                bridge.dds_topic.clone(),
                route.translation,
                route.output.clone(),
            )));
        }
        self.participant = Some(participant);
    }

    async fn handle(&mut self) {
        // No inputs: the readers do the work until shutdown.
        std::future::pending::<()>().await
    }

    async fn teardown(&mut self) {
        for reader in self.readers.drain(..) {
            reader.abort();
        }
        self.participant = None;
    }
}

/// A startup failure: logged where the coordinator shows it, then the process exits.
fn fatal(message: String) -> ! {
    error!("{message}");
    std::process::exit(1);
}

/// Decode each sample of one DDS topic and publish it on its DimOS stream.
async fn forward(
    reader: rustdds::no_key::DataReader<dds::RawCdr, RawCdrAdapter>,
    dds_topic: String,
    translation: &'static Translation,
    output: Output<DimosMsg>,
) {
    let mut samples = reader.async_bare_sample_stream();
    let mut events = samples.async_event_stream();
    let mut first = true;
    loop {
        tokio::select! {
            sample = samples.next() => match sample {
                Some(Ok(raw)) => match (translation.decode)(&raw.body, raw.little_endian) {
                    Ok(msg) => {
                        if first {
                            first = false;
                            info!(dds_topic = %dds_topic, bytes = raw.body.len(), "first sample");
                        }
                        if let Err(e) = output.publish(&msg).await {
                            warn_throttled!(WARN_EVERY, dds_topic = %dds_topic, error = %e, "publish failed");
                        }
                    }
                    Err(e) => {
                        warn_throttled!(WARN_EVERY, dds_topic = %dds_topic, error = %e, "decode failed");
                    }
                },
                Some(Err(e)) => {
                    warn_throttled!(WARN_EVERY, dds_topic = %dds_topic, error = ?e, "DDS read failed");
                }
                None => break,
            },
            Some(event) = events.next() => {
                info!(dds_topic = %dds_topic, event = ?event, "DDS reader status");
            }
        }
    }
    warn!(dds_topic = %dds_topic, "DDS reader stream ended");
}

#[cfg(test)]
mod tests {
    use super::*;
    use validator::Validate;

    fn config(bridges: serde_json::Value) -> Config {
        serde_json::from_value(serde_json::json!({
            "domain_id": 0,
            "network_addresses": [],
            "dds_to_dimos": bridges,
        }))
        .expect("parses")
    }

    fn bridge(dds_topic: &str, dds_type: &str, stream: &str) -> serde_json::Value {
        serde_json::json!({
            "dds_topic": dds_topic,
            "dds_type": dds_type,
            "dds_type_name": null,
            "zenoh_topic": stream,
            "qos": {"reliability": "best_effort", "durability": "volatile", "history_depth": 10},
        })
    }

    #[test]
    fn an_unknown_type_names_its_topic_and_type() {
        let c = config(serde_json::json!([bridge(
            "rt/gait",
            "dr_msgs/msg/Gait",
            "gait"
        )]));
        let message = c.validate().expect_err("rejected").to_string();
        assert!(
            message.contains("rt/gait") && message.contains("dr_msgs/msg/Gait"),
            "{message}"
        );
        assert!(
            message.contains("nav_msgs/msg/Odometry"),
            "lists what is supported: {message}"
        );
    }

    #[test]
    fn two_topics_cannot_share_a_stream() {
        let c = config(serde_json::json!([
            bridge("rt/a", "std_msgs/msg/Int32", "same"),
            bridge("rt/b", "std_msgs/msg/Int32", "same"),
        ]));
        assert!(c
            .validate()
            .expect_err("rejected")
            .to_string()
            .contains("same"));
    }

    #[test]
    fn known_types_pass() {
        let c = config(serde_json::json!([bridge(
            "rt/odom",
            "nav_msgs/msg/Odometry",
            "odom"
        )]));
        c.validate().expect("valid");
        assert_eq!(
            c.dds_to_dimos[0].type_name(),
            "nav_msgs::msg::dds_::Odometry_"
        );
    }
}
