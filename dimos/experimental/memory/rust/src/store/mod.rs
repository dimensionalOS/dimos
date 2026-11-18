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
use std::{fs, path::Path};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

use crate::StreamConfig;

/// Durable artifact configuration for the recorder's writer thread.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum RecordingStoreConfig {
    Sqlite {
        path: String,
    },
    Mcap {
        path: String,
    },
    #[serde(rename = "collection_mcap")]
    CollectionMcap {
        path: String,
    },
}

impl RecordingStoreConfig {
    pub fn path(&self) -> &str {
        match self {
            Self::Sqlite { path } | Self::Mcap { path } | Self::CollectionMcap { path } => path,
        }
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
        RecordingStoreConfig::CollectionMcap { path } => {
            // The collector exclusively creates the directory and closes the sidecar before launch.
            let schema_path = Path::new(path).with_file_name("schema.json");
            let schema = fs::read(&schema_path).with_context(|| {
                format!(
                    "failed to read collection schema: {}",
                    schema_path.display()
                )
            })?;
            let mut store = mcap::McapRecordingStore::open(path, streams, compression_threads)?;
            store.attach_collection_schema(&schema)?;
            Ok(Box::new(store))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collection_mcap_requires_the_sidecar_before_creating_an_artifact() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("recording.mcap");
        let config = RecordingStoreConfig::CollectionMcap {
            path: path.to_str().unwrap().into(),
        };
        let Err(error) = open(&config, &[], 1) else {
            panic!("collection accepted a missing schema.json");
        };
        assert!(error
            .to_string()
            .contains("failed to read collection schema"));
        assert!(!path.exists());
    }
}
