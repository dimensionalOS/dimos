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

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::BufWriter;

use anyhow::{Context, Result};
use mcap::records::MessageHeader;
use mcap::{Compression, WriteOptions, Writer};

use super::{Observation, RecordingStore};
use crate::{Codec, StreamConfig};

pub struct McapRecordingStore {
    writer: Writer<BufWriter<File>>,
    channels: HashMap<String, u16>,
    sequences: HashMap<String, u32>,
}

impl McapRecordingStore {
    pub fn open(path: &str, streams: &[StreamConfig], compression_threads: usize) -> Result<Self> {
        for stream in streams {
            anyhow::ensure!(
                stream.codec == Codec::Cdr,
                "MCAP requires cdr channels; use chunk compression instead of a payload wrapper"
            );
            anyhow::ensure!(
                !stream.schema_name.is_empty() && !stream.schema_definition.is_empty(),
                "MCAP requires a complete message schema"
            );
        }
        let file = File::create(path).with_context(|| format!("failed to create {path}"))?;
        let options = WriteOptions::new()
            .profile("ros2")
            .library("dimos-memory-recorder")
            .compression(Some(Compression::Zstd))
            .compression_threads(compression_threads.try_into().unwrap_or(u32::MAX));
        let mut writer = Writer::with_options(BufWriter::new(file), options)?;
        let mut channels = HashMap::new();
        for stream in streams {
            let metadata = BTreeMap::from([
                (
                    "dimos.payload_type".to_string(),
                    stream.payload_type.clone(),
                ),
                ("dimos.stream_name".to_string(), stream.name.clone()),
                ("dimos.port".to_string(), stream.port.clone()),
                (
                    "dimos.observation_time".to_string(),
                    "publish_time".to_string(),
                ),
            ]);
            let schema = writer.add_schema(
                &stream.schema_name,
                "ros2msg",
                stream.schema_definition.as_bytes(),
            )?;
            let channel = writer.add_channel(schema, &stream.name, "cdr", &metadata)?;
            channels.insert(stream.name.clone(), channel);
        }
        Ok(Self {
            writer,
            channels,
            sequences: HashMap::new(),
        })
    }
}

impl RecordingStore for McapRecordingStore {
    fn write_batch(&mut self, observations: &[Observation]) -> Result<()> {
        for observation in observations {
            let channel_id = self.channels[&observation.stream.name];
            let sequence = self
                .sequences
                .entry(observation.stream.name.clone())
                .or_default();
            self.writer.write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence: *sequence,
                    log_time: u64::try_from(observation.reception_ts)
                        .context("MCAP reception time precedes Unix epoch")?,
                    publish_time: u64::try_from(observation.source_ts)
                        .context("MCAP source time precedes Unix epoch")?,
                },
                &observation.data,
            )?;
            *sequence = sequence.wrapping_add(1);
        }
        Ok(())
    }

    fn finish(&mut self) -> Result<()> {
        self.writer.finish()?;
        Ok(())
    }
}
