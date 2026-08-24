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

use std::collections::{BTreeMap, HashMap, HashSet};
use std::io;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{bounded, Receiver, RecvTimeoutError, Sender};
use dimos_module::transport::Dispatch;
use dimos_module::{LcmTransport, Transport, ZenohTransport};
use serde::Deserialize;
use tracing::{debug, info};

use crate::encoding::StoredObservation;
use crate::store::{Observation, RecordingStore};

mod encoding;
pub mod store;
mod timestamp;

const TF_PAYLOAD_TYPE: &str = "dimos.msgs.tf2_msgs.TFMessage.TFMessage";
const QUEUE_CAPACITY: usize = 256;
const WRITE_BATCH_SIZE: usize = 128;
const FLUSH_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Codec {
    Lcm,
    Jpeg,
    #[serde(rename = "lz4+lcm")]
    Lz4Lcm,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StreamConfig {
    pub port: String,
    pub name: String,
    pub payload_type: String,
    pub codec: Codec,
}

impl StreamConfig {
    pub fn is_tf(&self) -> bool {
        self.payload_type == TF_PAYLOAD_TYPE
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecorderConfig {
    pub store: store::RecordingStoreConfig,
    pub encoding_threads: usize,
    pub streams: Vec<StreamConfig>,
}

impl RecorderConfig {
    pub fn validate(&self) -> Result<()> {
        if self.encoding_threads == 0 {
            return Err(anyhow!("encoding_threads must be at least 1"));
        }
        let mut names = HashSet::new();
        for stream in &self.streams {
            if !names.insert(&stream.name) {
                return Err(anyhow!("duplicate recorded stream name {:?}", stream.name));
            }
        }
        Ok(())
    }
}

#[derive(Clone)]
pub struct RecorderHandle {
    sender: Sender<EncodeMessage>,
    permits: Sender<()>,
    sequence: Arc<AtomicU64>,
    accepting: Arc<AtomicBool>,
}

impl RecorderHandle {
    pub fn record(&self, stream: Arc<StreamConfig>, data: &[u8]) {
        if !self.accepting.load(Ordering::Acquire) {
            return;
        }
        if self.permits.send(()).is_err() {
            return;
        }
        let job = EncodeJob {
            sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
            stream,
            reception_ts: wall_time(),
            data: data.to_vec(),
        };
        if self.sender.send(EncodeMessage::Job(job)).is_err() {
            self.accepting.store(false, Ordering::Release);
        }
    }
}

pub struct RecorderEngine {
    handle: RecorderHandle,
    workers: Vec<JoinHandle<()>>,
    writer: Option<JoinHandle<Result<WriterStats>>>,
    failure: Receiver<String>,
}

impl RecorderEngine {
    pub fn start(config: RecorderConfig) -> Result<Self> {
        config.validate()?;
        let (encode_tx, encode_rx) = bounded(QUEUE_CAPACITY);
        let (write_tx, write_rx) = bounded(QUEUE_CAPACITY);
        let (permit_tx, permit_rx) = bounded(QUEUE_CAPACITY);
        let (failure_tx, failure_rx) = bounded(1);
        let accepting = Arc::new(AtomicBool::new(true));
        let handle = RecorderHandle {
            sender: encode_tx,
            permits: permit_tx,
            sequence: Arc::new(AtomicU64::new(0)),
            accepting: Arc::clone(&accepting),
        };

        let writer_config = config.clone();
        let writer = thread::Builder::new()
            .name("mem2-writer".to_string())
            .spawn(move || {
                let result = writer_loop(writer_config, write_rx, permit_rx);
                if let Err(error) = &result {
                    let _ = failure_tx.send(format!("{error:#}"));
                }
                result
            })
            .context("failed to start memory writer thread")?;

        let stores_wire_bytes = config.store.stores_wire_bytes();
        let worker_count = if stores_wire_bytes {
            1
        } else {
            config.encoding_threads
        };
        let mut workers = Vec::with_capacity(worker_count);
        for index in 0..worker_count {
            let receiver = encode_rx.clone();
            let sender = write_tx.clone();
            workers.push(
                thread::Builder::new()
                    .name(format!("mem2-encoder-{index}"))
                    .spawn(move || encode_loop(receiver, sender, stores_wire_bytes))
                    .context("failed to start encoding thread")?,
            );
        }
        drop(write_tx);

        Ok(Self {
            handle,
            workers,
            writer: Some(writer),
            failure: failure_rx,
        })
    }

    pub fn handle(&self) -> RecorderHandle {
        self.handle.clone()
    }

    pub fn failure_receiver(&self) -> Receiver<String> {
        self.failure.clone()
    }

    pub fn shutdown(mut self) -> Result<WriterStats> {
        self.handle.accepting.store(false, Ordering::Release);
        for _ in &self.workers {
            let _ = self.handle.sender.send(EncodeMessage::Shutdown);
        }
        for worker in self.workers.drain(..) {
            worker
                .join()
                .map_err(|_| anyhow!("encoding worker panicked"))?;
        }
        self.writer
            .take()
            .expect("writer handle is present")
            .join()
            .map_err(|_| anyhow!("writer thread panicked"))?
    }
}

#[derive(Debug)]
enum EncodeMessage {
    Job(EncodeJob),
    Shutdown,
}

#[derive(Debug)]
struct EncodeJob {
    sequence: u64,
    stream: Arc<StreamConfig>,
    reception_ts: f64,
    data: Vec<u8>,
}

#[derive(Debug)]
struct EncodedBatch {
    sequence: u64,
    stream: Arc<StreamConfig>,
    reception_ts: f64,
    observations: Result<Vec<StoredObservation>, String>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WriterStats {
    pub received: u64,
    pub written: u64,
    pub encode_errors: u64,
}

fn encode_loop(
    receiver: Receiver<EncodeMessage>,
    sender: Sender<EncodedBatch>,
    stores_wire_bytes: bool,
) {
    while let Ok(message) = receiver.recv() {
        match message {
            EncodeMessage::Job(job) => {
                let encoded =
                    encoding::encode(&job.stream, &job.data, job.reception_ts, stores_wire_bytes)
                        .map_err(|error| format!("{error:#}"));
                let batch = EncodedBatch {
                    sequence: job.sequence,
                    stream: job.stream,
                    reception_ts: job.reception_ts,
                    observations: encoded,
                };
                if sender.send(batch).is_err() {
                    return;
                }
            }
            EncodeMessage::Shutdown => return,
        }
    }
}

fn wall_time() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

#[derive(Deserialize)]
struct LaunchConfig {
    topics: HashMap<String, String>,
    config: RecorderConfig,
}

/// Run the stdin-configured native recorder until shutdown or pipeline failure.
pub async fn run() -> Result<()> {
    dimos_module::init_tracing();
    let launch_value = dimos_module::read_launch_config().await?;
    let launch =
        serde_json::from_value(launch_value.clone()).context("invalid memory recorder config")?;
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => run_with(LcmTransport::new().await?, launch).await,
        Ok("zenoh") => run_with(ZenohTransport::from_launch(&launch_value).await?, launch).await,
        other => Err(anyhow!(
            "DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got {other:?}"
        )),
    }
}

async fn run_with<T: Transport>(transport: T, launch: LaunchConfig) -> Result<()> {
    launch.config.validate()?;
    let engine = RecorderEngine::start(launch.config.clone())?;
    let handle = engine.handle();

    for stream in &launch.config.streams {
        let topic = launch
            .topics
            .get(&stream.port)
            .with_context(|| format!("no topic supplied for recorder port {:?}", stream.port))?;
        let stream = Arc::new(stream.clone());
        let callback_stream = Arc::clone(&stream);
        let callback_handle = handle.clone();
        let callback: Dispatch =
            Arc::new(move |data| callback_handle.record(Arc::clone(&callback_stream), data));
        transport
            .subscribe(topic, callback)
            .await
            .with_context(|| format!("failed to subscribe to {topic:?}"))?;
        info!(port = %stream.port, topic = %topic, store_stream = %stream.name, codec = ?stream.codec, "recording stream");
    }

    let failure = engine.failure_receiver();
    let pipeline_failed = tokio::task::spawn_blocking(move || failure.recv());
    tokio::pin!(pipeline_failed);
    tokio::select! {
        result = shutdown_signal() => result.context("failed to listen for shutdown")?,
        result = &mut pipeline_failed => {
            let message = result
                .context("pipeline failure monitor panicked")?
                .context("pipeline failure monitor disconnected")?;
            return engine.shutdown().and(Err(anyhow!(message)));
        }
    }
    engine.shutdown()?;
    Ok(())
}

#[cfg(unix)]
async fn shutdown_signal() -> io::Result<()> {
    use tokio::signal::unix::{signal, SignalKind};

    let mut terminate = signal(SignalKind::terminate())?;
    tokio::select! {
        result = tokio::signal::ctrl_c() => result,
        _ = terminate.recv() => Ok(()),
    }
}

#[cfg(not(unix))]
async fn shutdown_signal() -> io::Result<()> {
    tokio::signal::ctrl_c().await
}

fn writer_loop(
    config: RecorderConfig,
    receiver: Receiver<EncodedBatch>,
    permits: Receiver<()>,
) -> Result<WriterStats> {
    let mut store = store::open(&config.store, &config.streams, config.encoding_threads)?;

    let mut pending = BTreeMap::new();
    let mut ready = Vec::with_capacity(WRITE_BATCH_SIZE);
    let mut next_sequence = 0;
    let mut stats = WriterStats::default();

    loop {
        match receiver.recv_timeout(FLUSH_INTERVAL) {
            Ok(batch) => {
                stats.received += 1;
                pending.insert(batch.sequence, batch);
                while let Some(batch) = pending.remove(&next_sequence) {
                    ready.push(batch);
                    next_sequence += 1;
                }
                if ready.len() >= WRITE_BATCH_SIZE {
                    write_ready(store.as_mut(), &mut ready, &mut stats, &permits)?;
                }
            }
            Err(RecvTimeoutError::Timeout) => {
                write_ready(store.as_mut(), &mut ready, &mut stats, &permits)?;
            }
            Err(RecvTimeoutError::Disconnected) => {
                while let Some(batch) = pending.remove(&next_sequence) {
                    ready.push(batch);
                    next_sequence += 1;
                }
                if !pending.is_empty() {
                    return Err(anyhow!("encoder results ended with a sequence gap"));
                }
                write_ready(store.as_mut(), &mut ready, &mut stats, &permits)?;
                store.finish()?;
                info!(
                    received = stats.received,
                    written = stats.written,
                    encode_errors = stats.encode_errors,
                    "memory recorder flushed"
                );
                return Ok(stats);
            }
        }
    }
}

fn write_ready(
    store: &mut dyn RecordingStore,
    ready: &mut Vec<EncodedBatch>,
    stats: &mut WriterStats,
    permits: &Receiver<()>,
) -> Result<()> {
    if ready.is_empty() {
        return Ok(());
    }
    let completed_jobs = ready.len();
    let mut stored_observations = Vec::new();
    for batch in ready.drain(..) {
        match batch.observations {
            Ok(observations) => {
                for observation in observations {
                    stored_observations.push(Observation {
                        stream: Arc::clone(&batch.stream),
                        source_ts: observation.ts,
                        reception_ts: batch.reception_ts,
                        data: observation.data,
                    });
                }
            }
            Err(message) => {
                stats.encode_errors += 1;
                return Err(anyhow!(
                    "stream {:?} sequence {} encoding failed: {message}",
                    batch.stream.name,
                    batch.sequence
                ));
            }
        }
    }
    store.write_batch(&stored_observations)?;
    stats.written += stored_observations.len() as u64;
    debug!(
        jobs = completed_jobs,
        observations = stored_observations.len(),
        "memory recorder batch written"
    );
    for _ in 0..completed_jobs {
        permits
            .recv()
            .context("recorder permit accounting disconnected")?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::io::Read;

    use lcm_msgs::sensor_msgs::{Image, Imu};
    use lcm_msgs::std_msgs::{Header, Time};
    use lcm_msgs::tf2_msgs::TFMessage;
    use lz4_flex::frame::FrameDecoder;
    use rusqlite::Connection;
    use tempfile::NamedTempFile;

    use super::*;

    fn stream(name: &str, codec: Codec, is_tf: bool) -> Arc<StreamConfig> {
        Arc::new(StreamConfig {
            port: name.to_string(),
            name: name.to_string(),
            payload_type: if is_tf {
                TF_PAYLOAD_TYPE.to_string()
            } else {
                "test.Raw".to_string()
            },
            codec,
        })
    }

    #[test]
    fn lz4_codec_uses_the_frame_format_python_reads() {
        let compressed = encoding::lz4_frame(b"a payload worth compressing").unwrap();
        let mut decoded = Vec::new();
        FrameDecoder::new(compressed.as_slice())
            .read_to_end(&mut decoded)
            .unwrap();
        assert_eq!(decoded, b"a payload worth compressing");
    }

    #[test]
    fn jpeg_codec_preserves_the_lcm_envelope() {
        let image = Image {
            header: Header {
                seq: 9,
                stamp: Time { sec: 12, nsec: 34 },
                frame_id: "camera".to_string(),
            },
            height: 2,
            width: 2,
            encoding: "rgb8".to_string(),
            is_bigendian: 0,
            step: 6,
            data: vec![255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255],
        };

        let (ts, encoded) = encoding::encode_jpeg(&image.encode(), 100.0).unwrap();
        let decoded = Image::decode(&encoded).unwrap();

        assert_eq!(ts, 12.000_000_034);
        assert_eq!(decoded.header, image.header);
        assert_eq!(decoded.encoding, "jpeg");
        assert_eq!(decoded.step, 0);
        assert_eq!(&decoded.data[..2], &[0xff, 0xd8]);
    }

    #[test]
    fn tf_batches_become_individually_timestamped_observations() {
        let mut first = lcm_msgs::geometry_msgs::TransformStamped::default();
        first.header.stamp = Time { sec: 10, nsec: 5 };
        first.child_frame_id = "first".to_string();
        let mut second = lcm_msgs::geometry_msgs::TransformStamped::default();
        second.header.stamp = Time { sec: 20, nsec: 7 };
        second.child_frame_id = "second".to_string();
        let message = TFMessage {
            transforms: vec![first, second],
        };
        let job = EncodeJob {
            sequence: 0,
            stream: stream("tf", Codec::Lcm, true),
            reception_ts: 100.0,
            data: message.encode(),
        };

        let observations =
            encoding::encode(&job.stream, &job.data, job.reception_ts, false).unwrap();

        assert_eq!(observations.len(), 2);
        assert_eq!(observations[0].ts, 10.000_000_005);
        assert_eq!(observations[1].ts, 20.000_000_007);
        let decoded: Vec<TFMessage> = observations
            .iter()
            .map(|observation| TFMessage::decode(&observation.data).unwrap())
            .collect();
        assert_eq!(decoded[0].transforms.len(), 1);
        assert_eq!(decoded[0].transforms[0].child_frame_id, "first");
        assert_eq!(decoded[1].transforms.len(), 1);
        assert_eq!(decoded[1].transforms[0].child_frame_id, "second");
    }

    #[test]
    fn stamped_sensor_messages_preserve_their_source_timestamp() {
        let mut message = Imu::default();
        message.header.stamp = Time { sec: 42, nsec: 25 };
        let job = EncodeJob {
            sequence: 0,
            stream: Arc::new(StreamConfig {
                port: "imu".to_string(),
                name: "imu".to_string(),
                payload_type: "dimos.msgs.sensor_msgs.Imu.Imu".to_string(),
                codec: Codec::Lcm,
            }),
            reception_ts: 100.0,
            data: message.encode(),
        };

        assert_eq!(
            timestamp::timestamp_for(&job.stream.payload_type, &job.data, job.reception_ts)
                .unwrap(),
            42.000_000_025
        );
    }

    #[test]
    fn wire_store_encoding_preserves_the_original_packet() {
        let mut message = Imu::default();
        message.header.stamp = Time { sec: 42, nsec: 25 };
        let data = message.encode();
        let job = EncodeJob {
            sequence: 0,
            stream: stream("imu", Codec::Jpeg, false),
            reception_ts: 100.0,
            data: data.clone(),
        };

        let observations =
            encoding::encode(&job.stream, &job.data, job.reception_ts, true).unwrap();

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].ts, 100.0);
        assert_eq!(observations[0].data, data);
    }

    #[test]
    fn writer_restores_arrival_order_after_parallel_encoding() {
        let file = NamedTempFile::new().unwrap();
        let config = RecorderConfig {
            store: store::RecordingStoreConfig::Sqlite {
                path: file.path().to_string_lossy().into_owned(),
            },
            encoding_threads: 2,
            streams: vec![(*stream("samples", Codec::Lcm, false)).clone()],
        };
        let (sender, receiver) = bounded(8);
        let (permit_sender, permit_receiver) = bounded(8);
        let writer_config = config.clone();
        let writer = thread::spawn(move || writer_loop(writer_config, receiver, permit_receiver));
        for sequence in [1, 0, 2] {
            permit_sender.send(()).unwrap();
            sender
                .send(EncodedBatch {
                    sequence,
                    stream: stream("samples", Codec::Lcm, false),
                    reception_ts: sequence as f64,
                    observations: Ok(vec![StoredObservation {
                        ts: sequence as f64,
                        data: vec![sequence as u8],
                    }]),
                })
                .unwrap();
        }
        drop(sender);
        let stats = writer.join().unwrap().unwrap();

        let connection = Connection::open(file.path()).unwrap();
        let values = connection
            .prepare(
                "SELECT samples.ts, samples_blob.data, typeof(samples.tags), json_extract(samples.tags, '$.reception_ts') FROM samples JOIN samples_blob USING (id) ORDER BY samples.id",
            )
            .unwrap()
            .query_map([], |row| {
                Ok((
                    row.get::<_, f64>(0)?,
                    row.get::<_, Vec<u8>>(1)?,
                    row.get::<_, String>(2)?,
                    row.get::<_, f64>(3)?,
                ))
            })
            .unwrap()
            .collect::<rusqlite::Result<Vec<_>>>()
            .unwrap();
        assert_eq!(
            values,
            vec![
                (0.0, vec![0], "blob".to_string(), 0.0),
                (1.0, vec![1], "blob".to_string(), 1.0),
                (2.0, vec![2], "blob".to_string(), 2.0),
            ]
        );
        let reception_index: i64 = connection
            .query_row(
                "SELECT count(*) FROM sqlite_master WHERE type = 'index' AND name = 'samples_tag_reception_ts'",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!(reception_index, 1);
        assert_eq!(stats.written, 3);
    }

    #[test]
    fn tf_observations_use_empty_jsonb_tags_without_a_reception_index() {
        let file = NamedTempFile::new().unwrap();
        let connection = Connection::open(file.path()).unwrap();
        let tf = stream("tf", Codec::Lcm, true);
        drop(connection);
        let mut recording_store = store::open(
            &store::RecordingStoreConfig::Sqlite {
                path: file.path().to_string_lossy().into_owned(),
            },
            &[(*tf).clone()],
            1,
        )
        .unwrap();
        recording_store
            .write_batch(&[Observation {
                stream: Arc::clone(&tf),
                source_ts: 1.0,
                reception_ts: 100.0,
                data: vec![1, 2, 3],
            }])
            .unwrap();
        recording_store.finish().unwrap();
        drop(recording_store);
        let connection = Connection::open(file.path()).unwrap();

        let (tag_type, tags): (String, String) = connection
            .query_row("SELECT typeof(tags), json(tags) FROM tf", [], |row| {
                Ok((row.get(0)?, row.get(1)?))
            })
            .unwrap();
        assert_eq!(tag_type, "blob");
        assert_eq!(tags, "{}");
        let reception_index: i64 = connection
            .query_row(
                "SELECT count(*) FROM sqlite_master WHERE type = 'index' AND name = 'tf_tag_reception_ts'",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!(reception_index, 0);
    }

    #[test]
    fn mcap_store_writes_indexed_original_wire_messages() {
        let file = NamedTempFile::new().unwrap();
        let samples = stream("samples", Codec::Lcm, false);
        let mut recording_store = store::open(
            &store::RecordingStoreConfig::Mcap {
                path: file.path().to_string_lossy().into_owned(),
            },
            &[(*samples).clone()],
            1,
        )
        .unwrap();
        recording_store
            .write_batch(&[Observation {
                stream: Arc::clone(&samples),
                source_ts: 12.5,
                reception_ts: 13.0,
                data: vec![1, 2, 3],
            }])
            .unwrap();
        recording_store.finish().unwrap();
        drop(recording_store);

        let bytes = std::fs::read(file.path()).unwrap();
        let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
        assert_eq!(summary.chunk_indexes.len(), 1);
        assert_eq!(summary.chunk_indexes[0].compression, "zstd");
        let indexes = summary
            .read_message_indexes(&bytes, &summary.chunk_indexes[0])
            .unwrap();
        assert_eq!(indexes.values().map(Vec::len).sum::<usize>(), 1);

        let messages = mcap::MessageStream::new(&bytes)
            .unwrap()
            .collect::<mcap::McapResult<Vec<_>>>()
            .unwrap();
        assert_eq!(messages.len(), 1);
        let message = &messages[0];
        assert_eq!(message.channel.topic, "samples");
        assert_eq!(message.channel.message_encoding, "lcm");
        assert_eq!(message.channel.metadata["dimos.payload_type"], "test.Raw");
        assert_eq!(message.log_time, 13_000_000_000);
        assert_eq!(message.publish_time, 12_500_000_000);
        assert_eq!(message.data.as_ref(), &[1, 2, 3]);
    }

    #[test]
    fn engine_uses_the_configured_worker_count_and_flushes_on_shutdown() {
        let file = NamedTempFile::new().unwrap();
        let raw = stream("raw", Codec::Lcm, false);
        let config = RecorderConfig {
            store: store::RecordingStoreConfig::Sqlite {
                path: file.path().to_string_lossy().into_owned(),
            },
            encoding_threads: 3,
            streams: vec![(*raw).clone()],
        };
        let engine = RecorderEngine::start(config).unwrap();
        assert_eq!(engine.workers.len(), 3);
        let handle = engine.handle();
        handle.record(raw.clone(), b"one");
        handle.record(raw, b"two");
        let stats = engine.shutdown().unwrap();
        assert_eq!(stats.written, 2);
        assert_eq!(stats.encode_errors, 0);
    }

    #[test]
    fn encoding_failure_fails_the_recording_with_stream_and_sequence() {
        let file = NamedTempFile::new().unwrap();
        let image = stream("camera", Codec::Jpeg, false);
        let config = RecorderConfig {
            store: store::RecordingStoreConfig::Sqlite {
                path: file.path().to_string_lossy().into_owned(),
            },
            encoding_threads: 1,
            streams: vec![(*image).clone()],
        };
        let engine = RecorderEngine::start(config).unwrap();
        engine.handle().record(image, b"not an lcm image");

        let error = engine.shutdown().unwrap_err();

        let message = format!("{error:#}");
        assert!(message.contains("camera"));
        assert!(message.contains("sequence 0"));
        assert!(message.contains("invalid LCM Image"));
    }

    #[test]
    fn duplicate_stream_names_fail_validation() {
        let config = RecorderConfig {
            store: store::RecordingStoreConfig::Sqlite {
                path: "unused.db".to_string(),
            },
            encoding_threads: 1,
            streams: vec![
                (*stream("samples", Codec::Lcm, false)).clone(),
                StreamConfig {
                    port: "other".to_string(),
                    name: "samples".to_string(),
                    payload_type: "test.Raw".to_string(),
                    codec: Codec::Lcm,
                },
            ],
        };

        assert!(config
            .validate()
            .unwrap_err()
            .to_string()
            .contains("duplicate recorded stream name"));
    }
}
