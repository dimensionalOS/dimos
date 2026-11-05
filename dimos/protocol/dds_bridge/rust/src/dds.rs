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

//! The DDS side: a participant, per-topic QoS, and readers that hand back raw CDR bodies.

use std::io;
use std::net::IpAddr;

use rustdds::no_key::{Decode, DefaultDecoder, DeserializerAdapter};
use rustdds::policy::{Durability, History, Reliability};
use rustdds::{
    DomainParticipant, DomainParticipantBuilder, QosPolicies, QosPolicyBuilder,
    RepresentationIdentifier,
};
use serde::{Deserialize, Serialize};

/// The reader QoS of one bridged topic. It must be compatible with the writer's or DDS never matches them.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TopicQos {
    pub reliability: ReliabilityKind,
    pub durability: DurabilityKind,
    pub history_depth: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReliabilityKind {
    Reliable,
    BestEffort,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DurabilityKind {
    Volatile,
    TransientLocal,
}

impl TopicQos {
    pub fn to_dds(&self) -> QosPolicies {
        let reliability = match self.reliability {
            ReliabilityKind::Reliable => Reliability::Reliable {
                max_blocking_time: rustdds::Duration::ZERO,
            },
            ReliabilityKind::BestEffort => Reliability::BestEffort,
        };
        let durability = match self.durability {
            DurabilityKind::Volatile => Durability::Volatile,
            DurabilityKind::TransientLocal => Durability::TransientLocal,
        };
        QosPolicyBuilder::new()
            .reliability(reliability)
            .durability(durability)
            .history(History::KeepLast {
                depth: self.history_depth,
            })
            .build()
    }
}

/// Join `domain_id`, advertising only `network_addresses` when any are given.
pub fn join_domain(domain_id: u16, network_addresses: &[String]) -> io::Result<DomainParticipant> {
    let mut builder = DomainParticipantBuilder::new(domain_id);
    if !network_addresses.is_empty() {
        let addresses = network_addresses
            .iter()
            .map(|a| {
                a.parse::<IpAddr>().map_err(|e| {
                    io::Error::new(
                        io::ErrorKind::InvalidInput,
                        format!("network address {a:?}: {e}"),
                    )
                })
            })
            .collect::<io::Result<Vec<IpAddr>>>()?;
        builder = builder.with_only_networks(addresses);
    }
    builder
        .build()
        .map_err(|e| io::Error::other(format!("joining DDS domain {domain_id}: {e:?}")))
}

/// One sample's CDR body, encapsulation header stripped, still undecoded.
pub struct RawCdr {
    pub body: Vec<u8>,
    pub little_endian: bool,
}

/// Hands the reader's payload back untouched so one reader type serves every table row.
pub struct RawCdrAdapter;

#[derive(Clone)]
pub struct RawCdrDecoder;

const CDR_ENCODINGS: [RepresentationIdentifier; 2] = [
    RepresentationIdentifier::CDR_LE,
    RepresentationIdentifier::CDR_BE,
];

impl DeserializerAdapter<RawCdr> for RawCdrAdapter {
    type Error = io::Error;
    type Decoded = RawCdr;

    fn supported_encodings() -> &'static [RepresentationIdentifier] {
        &CDR_ENCODINGS
    }

    fn transform_decoded(decoded: RawCdr) -> RawCdr {
        decoded
    }
}

impl DefaultDecoder<RawCdr> for RawCdrAdapter {
    type Decoder = RawCdrDecoder;
    const DECODER: Self::Decoder = RawCdrDecoder;
}

impl<'de> Decode<'de, RawCdr> for RawCdrDecoder {
    type Error = io::Error;

    fn decode_bytes(
        self,
        input_bytes: &'de [u8],
        encoding: RepresentationIdentifier,
    ) -> io::Result<RawCdr> {
        let little_endian = match encoding {
            RepresentationIdentifier::CDR_LE => true,
            RepresentationIdentifier::CDR_BE => false,
            other => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("unsupported encoding {other:?}"),
                ))
            }
        };
        Ok(RawCdr {
            body: input_bytes.to_vec(),
            little_endian,
        })
    }
}
