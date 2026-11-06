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

use std::collections::HashSet;

use anyhow::{Context, Result};
use rusqlite::{params, Connection};

use super::{Observation, RecordingStore};
use crate::StreamConfig;

pub struct SqliteRecordingStore {
    connection: Connection,
    initialized_streams: HashSet<String>,
}

impl SqliteRecordingStore {
    pub fn open(path: &str, streams: &[StreamConfig]) -> Result<Self> {
        let connection =
            Connection::open(path).with_context(|| format!("failed to open {path}"))?;
        connection.pragma_update(None, "journal_mode", "WAL")?;
        connection.pragma_update(None, "synchronous", "NORMAL")?;
        for stream in streams {
            validate_identifier(&stream.name)?;
        }
        connection.execute_batch(
            "CREATE TABLE IF NOT EXISTS _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)",
        )?;
        Ok(Self {
            connection,
            initialized_streams: HashSet::new(),
        })
    }
}

impl RecordingStore for SqliteRecordingStore {
    fn write_batch(&mut self, observations: &[Observation]) -> Result<()> {
        let transaction = self.connection.transaction()?;
        let mut new_streams = HashSet::new();
        for observation in observations {
            if !self.initialized_streams.contains(&observation.stream.name)
                && new_streams.insert(observation.stream.name.clone())
            {
                ensure_stream_tables(&transaction, &observation.stream)?;
            }
            insert_observation(&transaction, observation)?;
        }
        transaction.commit()?;
        self.initialized_streams.extend(new_streams);
        Ok(())
    }

    fn finish(&mut self) -> Result<()> {
        self.connection
            .execute_batch("PRAGMA wal_checkpoint(TRUNCATE)")?;
        Ok(())
    }
}

fn ensure_stream_tables(connection: &Connection, stream: &StreamConfig) -> Result<()> {
    let name = &stream.name;
    validate_identifier(name)?;
    connection.execute_batch(&format!(
        r#"
        CREATE TABLE IF NOT EXISTS "{name}" (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts REAL NOT NULL,
            value NUMERIC,
            pose_x REAL, pose_y REAL, pose_z REAL,
            pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
            tags BLOB DEFAULT (jsonb('{{}}'))
        );
        CREATE TABLE IF NOT EXISTS "{name}_blob" (
            id INTEGER PRIMARY KEY,
            data BLOB NOT NULL
        );
        CREATE VIRTUAL TABLE IF NOT EXISTS "{name}_rtree" USING rtree(
            id, x_min, x_max, y_min, y_max, z_min, z_max
        );
        "#
    ))?;
    if !stream.is_tf() {
        connection.execute(
            &format!(
                r#"CREATE INDEX IF NOT EXISTS "{name}_tag_reception_ts" ON "{name}"(json_extract(tags, '$.reception_ts'))"#
            ),
            [],
        )?;
    }
    let config = serde_json::json!({
        "payload_module": stream.payload_type,
        "codec_id": stream.codec.id(),
    });
    connection.execute(
        "INSERT OR IGNORE INTO _streams (name, config) VALUES (?1, ?2)",
        params![name, config.to_string()],
    )?;
    Ok(())
}

fn insert_observation(connection: &Connection, observation: &Observation) -> Result<()> {
    let name = &observation.stream.name;
    if observation.stream.is_tf() {
        connection.execute(
            &format!(r#"INSERT INTO "{name}" (ts) VALUES (?1)"#),
            params![observation.source_ts],
        )?;
    } else {
        let tags = serde_json::json!({"reception_ts": observation.reception_ts}).to_string();
        connection.execute(
            &format!(r#"INSERT INTO "{name}" (ts, tags) VALUES (?1, jsonb(?2))"#),
            params![observation.source_ts, tags],
        )?;
    }
    let id = connection.last_insert_rowid();
    connection.execute(
        &format!(r#"INSERT INTO "{name}_blob" (id, data) VALUES (?1, ?2)"#),
        params![id, observation.data],
    )?;
    Ok(())
}

fn validate_identifier(name: &str) -> Result<()> {
    let mut chars = name.chars();
    let first = chars
        .next()
        .ok_or_else(|| anyhow::anyhow!("stream name is empty"))?;
    if !(first == '_' || first.is_ascii_alphabetic())
        || !chars.all(|character| character == '_' || character.is_ascii_alphanumeric())
    {
        return Err(anyhow::anyhow!("invalid stream name {name:?}"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Codec;
    use std::sync::Arc;
    use tempfile::NamedTempFile;

    fn stream(name: &str) -> Arc<StreamConfig> {
        Arc::new(StreamConfig {
            port: name.into(),
            name: name.into(),
            payload_type: "dimos.msgs.std_msgs.String.String".into(),
            codec: Codec::Lcm,
            timestamp_field: None,
            json_schema: None,
        })
    }

    fn observation(stream: &Arc<StreamConfig>, source_ts: f64) -> Observation {
        Observation {
            stream: Arc::clone(stream),
            source_ts,
            reception_ts: 100.0,
            data: vec![1, 2, 3],
        }
    }

    fn registered_streams(connection: &Connection) -> Vec<String> {
        connection
            .prepare("SELECT name FROM _streams ORDER BY name")
            .unwrap()
            .query_map([], |row| row.get(0))
            .unwrap()
            .collect::<rusqlite::Result<Vec<_>>>()
            .unwrap()
    }

    #[test]
    fn streams_are_registered_with_tables_only_when_written() {
        let file = NamedTempFile::new().unwrap();
        let active = stream("active");
        let idle = stream("idle");
        let mut store = SqliteRecordingStore::open(
            file.path().to_str().unwrap(),
            &[(*active).clone(), (*idle).clone()],
        )
        .unwrap();
        assert!(registered_streams(&store.connection).is_empty());
        store.write_batch(&[observation(&active, 12.5)]).unwrap();
        assert_eq!(registered_streams(&store.connection), ["active"]);
        let idle_tables: i64 = store
            .connection
            .query_row(
                "SELECT count(*) FROM sqlite_master WHERE name GLOB 'idle*'",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!(idle_tables, 0);
        let config: String = store
            .connection
            .query_row(
                "SELECT config FROM _streams WHERE name='active'",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&config).unwrap(),
            serde_json::json!({
                "payload_module": active.payload_type,
                "codec_id": "lcm",
            })
        );
    }

    #[test]
    fn failed_first_batch_rolls_back_streams_and_can_be_retried() {
        let file = NamedTempFile::new().unwrap();
        let active = stream("active");
        let invalid = stream("invalid");
        let mut store = SqliteRecordingStore::open(file.path().to_str().unwrap(), &[]).unwrap();
        assert!(store
            .write_batch(&[observation(&active, 12.5), observation(&invalid, f64::NAN)])
            .is_err());
        assert!(registered_streams(&store.connection).is_empty());
        let tables: i64 = store
            .connection
            .query_row(
                "SELECT count(*) FROM sqlite_master WHERE name IN ('active', 'invalid')",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!(tables, 0);
        store.write_batch(&[observation(&active, 12.5)]).unwrap();
        assert_eq!(registered_streams(&store.connection), ["active"]);
    }

    #[test]
    fn append_preserves_existing_registered_configuration() {
        let file = NamedTempFile::new().unwrap();
        let active = stream("active");
        let original = r#"{"payload_module":"custom.Payload","codec_id":"lcm","extra":true}"#;
        let mut store = SqliteRecordingStore::open(file.path().to_str().unwrap(), &[]).unwrap();
        store
            .connection
            .execute(
                "INSERT INTO _streams (name, config) VALUES ('active', ?1)",
                [original],
            )
            .unwrap();
        store.write_batch(&[observation(&active, 12.5)]).unwrap();
        let actual: String = store
            .connection
            .query_row(
                "SELECT config FROM _streams WHERE name='active'",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!(actual, original);
    }

    #[test]
    fn unconfigured_stream_names_are_validated_before_sql() {
        let file = NamedTempFile::new().unwrap();
        let mut store = SqliteRecordingStore::open(file.path().to_str().unwrap(), &[]).unwrap();
        let error = store
            .write_batch(&[observation(&stream("bad\"name"), 12.5)])
            .unwrap_err();
        assert!(error.to_string().contains("invalid stream name"));
        assert!(registered_streams(&store.connection).is_empty());
    }
}
