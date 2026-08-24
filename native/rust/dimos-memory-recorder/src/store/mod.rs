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

mod mcap;
mod sqlite;

use std::sync::Arc;

use anyhow::Result;
use serde::Deserialize;

use crate::StreamConfig;

/// Durable artifact configuration for the recorder's writer thread.
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum RecordingStoreConfig {
    Sqlite { path: String },
    Mcap { path: String },
}

impl RecordingStoreConfig {
    pub fn path(&self) -> &str {
        match self {
            Self::Sqlite { path } | Self::Mcap { path } => path,
        }
    }

    pub fn stores_wire_bytes(&self) -> bool {
        matches!(self, Self::Mcap { .. })
    }
}

/// One ordered observation ready for durable storage.
pub struct Observation {
    pub stream: Arc<StreamConfig>,
    pub source_ts: f64,
    pub reception_ts: f64,
    pub data: Vec<u8>,
}

/// Record-only storage boundary shared by SQLite and MCAP artifacts.
pub trait RecordingStore: Send {
    /// Persist one arrival-ordered batch atomically where the format supports it.
    fn write_batch(&mut self, observations: &[Observation]) -> Result<()>;

    /// Finalize indexes, summaries, and buffered data before shutdown returns.
    fn finish(&mut self) -> Result<()>;
}

pub fn open(
    config: &RecordingStoreConfig,
    streams: &[StreamConfig],
    compression_threads: usize,
) -> Result<Box<dyn RecordingStore>> {
    match config {
        RecordingStoreConfig::Sqlite { path } => {
            Ok(Box::new(sqlite::SqliteRecordingStore::open(path, streams)?))
        }
        RecordingStoreConfig::Mcap { path } => Ok(Box::new(mcap::McapRecordingStore::open(
            path,
            streams,
            compression_threads,
        )?)),
    }
}
