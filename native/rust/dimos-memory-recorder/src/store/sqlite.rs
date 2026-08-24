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

use anyhow::{Context, Result};
use rusqlite::{params, Connection};

use super::{Observation, RecordingStore};
use crate::StreamConfig;

pub struct SqliteRecordingStore {
    connection: Connection,
}

impl SqliteRecordingStore {
    pub fn open(path: &str, streams: &[StreamConfig]) -> Result<Self> {
        let connection =
            Connection::open(path).with_context(|| format!("failed to open {path}"))?;
        connection.pragma_update(None, "journal_mode", "WAL")?;
        connection.pragma_update(None, "synchronous", "NORMAL")?;
        for stream in streams {
            ensure_stream_tables(&connection, stream)?;
        }
        Ok(Self { connection })
    }
}

impl RecordingStore for SqliteRecordingStore {
    fn write_batch(&mut self, observations: &[Observation]) -> Result<()> {
        let transaction = self.connection.transaction()?;
        for observation in observations {
            insert_observation(&transaction, observation)?;
        }
        transaction.commit()?;
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
