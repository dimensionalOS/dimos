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

//! Recovery of the window a crashed event-mode run left on disk.

use std::fs;
use std::path::Path;
use std::sync::Arc;
use std::thread;

use anyhow::{Context, Result};
use tracing::info;

use crate::store;
use crate::window::{segment_indexes, segment_path, DoneMarker, SegmentReader, WindowManifest};
use crate::WRITE_BATCH_SIZE;

/// Stores every segment the crashed run had not stored, in full, then removes the window.
/// Returns how many observations it added. Safe to rerun if it fails partway.
pub fn recover(window_dir: &Path) -> Result<u64> {
    let manifest = WindowManifest::read(window_dir)?;
    let done = DoneMarker::read(window_dir, manifest.streams.len())?;
    let stored = done.as_ref().map_or_else(
        || vec![0; manifest.streams.len()],
        |done| done.stored.clone(),
    );
    let threads = thread::available_parallelism().map_or(1, usize::from);
    let mut store = store::recover(&manifest.store, &manifest.streams, threads, &stored)?;
    let streams: Vec<_> = manifest.streams.into_iter().map(Arc::new).collect();

    let mut batch = Vec::with_capacity(WRITE_BATCH_SIZE);
    let mut written = 0;
    for index in segment_indexes(window_dir)? {
        if done.as_ref().is_some_and(|done| index <= done.segment) {
            continue;
        }
        for record in SegmentReader::open(&segment_path(window_dir, index))? {
            let record = record?;
            let stream = streams.get(usize::from(record.stream)).with_context(|| {
                format!("segment {index} names unknown stream {}", record.stream)
            })?;
            batch.push(record.into_observation(Arc::clone(stream)));
            if batch.len() >= WRITE_BATCH_SIZE {
                store.write_batch(&batch)?;
                written += batch.len() as u64;
                batch.clear();
            }
        }
    }
    store.write_batch(&batch)?;
    written += batch.len() as u64;
    store.finish()?;
    fs::remove_dir_all(window_dir)
        .with_context(|| format!("failed to remove {}", window_dir.display()))?;
    info!(window_dir = %window_dir.display(), written, "event window recovered");
    Ok(written)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::fs::OpenOptions;
    use std::io::Write;

    use rusqlite::Connection;

    use super::*;
    use crate::store::{Observation, RecordingStoreConfig};
    use crate::window::SegmentWriter;
    use crate::{Codec, StreamConfig};

    fn stream(name: &str) -> StreamConfig {
        StreamConfig {
            port: name.to_string(),
            name: name.to_string(),
            payload_type: "test.Raw".to_string(),
            codec: Codec::Lcm,
            timestamp_field: None,
            json_schema: None,
        }
    }

    fn append_garbage(path: &Path) {
        OpenOptions::new()
            .append(true)
            .open(path)
            .unwrap()
            .write_all(&[0xab; 37])
            .unwrap();
    }

    /// Reception times per stream, in stored order.
    fn stored_times(config: &RecordingStoreConfig) -> BTreeMap<String, Vec<f64>> {
        let mut times = BTreeMap::<String, Vec<f64>>::new();
        match config {
            RecordingStoreConfig::Sqlite { path } => {
                let connection = Connection::open(path).unwrap();
                for name in ["a", "b"] {
                    let values = connection
                        .prepare(&format!(
                            "SELECT json_extract(tags, '$.reception_ts') FROM {name} ORDER BY id"
                        ))
                        .unwrap()
                        .query_map([], |row| row.get(0))
                        .unwrap()
                        .collect::<rusqlite::Result<_>>()
                        .unwrap();
                    times.insert(name.to_string(), values);
                }
            }
            RecordingStoreConfig::Mcap { path, .. } => {
                let bytes = fs::read(path).unwrap();
                for message in mcap::MessageStream::new(&bytes).unwrap() {
                    let message = message.unwrap();
                    times
                        .entry(message.channel.topic.clone())
                        .or_default()
                        .push(message.log_time as f64 / 1e9);
                }
            }
        }
        times
    }

    #[test]
    fn recovery_appends_unstored_segments_without_duplicates() {
        let streams = vec![stream("a"), stream("b")];
        let shared: Vec<_> = streams.iter().cloned().map(Arc::new).collect();
        // Thirty observations at 1..=30 s, alternating streams, ten per segment.
        let observations: Vec<Observation> = (1..=30)
            .map(|second| Observation {
                stream: Arc::clone(&shared[second % 2]),
                source_ts: second as f64,
                reception_ts: second as f64,
                monotonic_ts: second as f64,
                data: vec![second as u8],
            })
            .collect();
        let segments: Vec<&[Observation]> = observations.chunks(10).collect();

        for sqlite in [true, false] {
            let dir = tempfile::tempdir().unwrap();
            let window = dir.path().join("window");
            let config = if sqlite {
                RecordingStoreConfig::Sqlite {
                    path: dir.path().join("memory.db").to_string_lossy().into_owned(),
                }
            } else {
                RecordingStoreConfig::Mcap {
                    path: dir
                        .path()
                        .join("memory.mcap")
                        .to_string_lossy()
                        .into_owned(),
                    attachments: vec![],
                }
            };

            // The crashed run stored segment 0, then wrote part of segment 1 before the crash.
            let mut recording = store::open(&config, &streams, 1).unwrap();
            recording.write_batch(segments[0]).unwrap();
            recording.sync().unwrap();
            recording.write_batch(&segments[1][..4]).unwrap();
            recording.sync().unwrap();
            std::mem::forget(recording);
            if !sqlite {
                append_garbage(Path::new(config.path()));
            }

            fs::create_dir(&window).unwrap();
            WindowManifest {
                store: config.clone(),
                streams: streams.clone(),
            }
            .write(&window)
            .unwrap();
            DoneMarker::create(&window)
                .unwrap()
                .write(0, &[5, 5])
                .unwrap();
            for (index, segment) in segments.iter().enumerate().skip(1) {
                let mut writer = SegmentWriter::create(&window, index as u64).unwrap();
                for observation in *segment {
                    let stream = if observation.stream.name == "a" { 0 } else { 1 };
                    writer.write(stream, observation).unwrap();
                }
                writer.seal().unwrap();
            }
            let last = segment_path(&window, 2);
            let length = fs::metadata(&last).unwrap().len();
            OpenOptions::new()
                .write(true)
                .open(&last)
                .unwrap()
                .set_len(length - 1)
                .unwrap();

            let written = recover(&window).unwrap();

            assert_eq!(written, 19, "sqlite: {sqlite}");
            assert!(!window.exists());
            let expected = |parity: usize| -> Vec<f64> {
                (1..30)
                    .filter(|second| second % 2 == parity)
                    .map(|second| second as f64)
                    .collect()
            };
            assert_eq!(
                stored_times(&config),
                BTreeMap::from([
                    ("a".to_string(), expected(0)),
                    ("b".to_string(), expected(1))
                ]),
                "sqlite: {sqlite}"
            );
        }
    }
}
