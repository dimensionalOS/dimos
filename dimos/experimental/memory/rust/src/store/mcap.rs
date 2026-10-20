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
use std::time::Duration;

use anyhow::{Context, Result};
use mcap::records::MessageHeader;
use mcap::{Compression, WriteOptions, Writer};

use super::{Observation, RecordingStore};
use crate::StreamConfig;

pub struct McapRecordingStore {
    writer: Writer<BufWriter<File>>,
    channels: HashMap<String, u16>,
    sequences: HashMap<String, u32>,
}

impl McapRecordingStore {
    pub fn open(path: &str, streams: &[StreamConfig], compression_threads: usize) -> Result<Self> {
        let file = File::create(path).with_context(|| format!("failed to create {path}"))?;
        let options = WriteOptions::new()
            .profile("dimos")
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
            let schema_id = if let Some(schema) = &stream.json_schema {
                writer.add_schema(&stream.name, "jsonschema", &serde_json::to_vec(schema)?)?
            } else {
                0
            };
            let channel =
                writer.add_channel(schema_id, &stream.name, stream.codec.id(), &metadata)?;
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
            let log_time = timestamp_ns(observation.reception_ts).with_context(|| {
                format!(
                    "invalid MCAP reception time for stream {:?}",
                    observation.stream.name
                )
            })?;
            let publish_time = timestamp_ns(observation.source_ts).with_context(|| {
                format!(
                    "invalid MCAP source time for stream {:?}",
                    observation.stream.name
                )
            })?;
            let channel_id = self.channels[&observation.stream.name];
            let sequence = self
                .sequences
                .entry(observation.stream.name.clone())
                .or_default();
            self.writer.write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence: *sequence,
                    log_time,
                    publish_time,
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

fn timestamp_ns(timestamp: f64) -> Result<u64> {
    Duration::try_from_secs_f64(timestamp)
        .context("MCAP time must be finite and nonnegative")?
        .as_nanos()
        .try_into()
        .context("MCAP time exceeds the unsigned 64-bit nanosecond range")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Codec;
    use std::sync::Arc;
    use tempfile::NamedTempFile;

    #[test]
    fn timestamp_conversion_rejects_unrepresentable_times_without_panicking() {
        for timestamp in [
            -12.5,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::MAX,
            u64::MAX as f64 / 1_000_000_000.0,
        ] {
            assert!(timestamp_ns(timestamp).is_err(), "accepted {timestamp}");
        }
        assert_eq!(timestamp_ns(0.0).unwrap(), 0);
        assert_eq!(timestamp_ns(12.5).unwrap(), 12_500_000_000);
    }

    #[test]
    fn mcap_rejects_negative_source_time_instead_of_writing_zero() {
        let file = NamedTempFile::new().unwrap();
        let stream = Arc::new(StreamConfig {
            port: "events".into(),
            name: "events".into(),
            payload_type: "dimos.msgs.std_msgs.String.String".into(),
            codec: Codec::Json,
            timestamp_field: Some("ts".into()),
            json_schema: None,
        });
        let mut store =
            McapRecordingStore::open(file.path().to_str().unwrap(), &[(*stream).clone()], 1)
                .unwrap();
        let observation = Observation {
            stream,
            source_ts: -12.5,
            reception_ts: 100.0,
            data: br#"{"ts":-12.5}"#.to_vec(),
        };
        assert!(store
            .write_batch(std::slice::from_ref(&observation))
            .is_err());
        store.finish().unwrap();
        let bytes = std::fs::read(file.path()).unwrap();
        assert_eq!(mcap::MessageStream::new(&bytes).unwrap().count(), 0);

        let sqlite_file = NamedTempFile::new().unwrap();
        let mut sqlite = super::super::open(
            &super::super::RecordingStoreConfig::Sqlite {
                path: sqlite_file.path().to_str().unwrap().into(),
            },
            std::slice::from_ref(observation.stream.as_ref()),
            1,
        )
        .unwrap();
        sqlite.write_batch(&[observation]).unwrap();
        sqlite.finish().unwrap();
        let connection = rusqlite::Connection::open(sqlite_file.path()).unwrap();
        let source_ts: f64 = connection
            .query_row("SELECT ts FROM events", [], |row| row.get(0))
            .unwrap();
        assert_eq!(source_ts, -12.5);
    }

    #[test]
    fn mcap_preserves_zero_source_time_and_rejects_invalid_reception_time() {
        let file = NamedTempFile::new().unwrap();
        let stream = Arc::new(StreamConfig {
            port: "events".into(),
            name: "events".into(),
            payload_type: "dimos.msgs.std_msgs.String.String".into(),
            codec: Codec::Json,
            timestamp_field: Some("ts".into()),
            json_schema: None,
        });
        let mut store =
            McapRecordingStore::open(file.path().to_str().unwrap(), &[(*stream).clone()], 1)
                .unwrap();
        let mut observation = Observation {
            stream,
            source_ts: 0.0,
            reception_ts: -1.0,
            data: br#"{"ts":0}"#.to_vec(),
        };
        assert!(store
            .write_batch(std::slice::from_ref(&observation))
            .is_err());
        observation.reception_ts = 12.5;
        store.write_batch(&[observation]).unwrap();
        store.finish().unwrap();
        let bytes = std::fs::read(file.path()).unwrap();
        let messages = mcap::MessageStream::new(&bytes)
            .unwrap()
            .collect::<mcap::McapResult<Vec<_>>>()
            .unwrap();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].publish_time, 0);
        assert_eq!(messages[0].log_time, 12_500_000_000);
        assert_eq!(messages[0].sequence, 0);
    }
}
