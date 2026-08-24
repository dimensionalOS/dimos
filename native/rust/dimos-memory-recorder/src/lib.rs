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

use std::collections::BTreeMap;
use std::io::Write;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{bounded, Receiver, RecvTimeoutError, Sender};
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::geometry_msgs::{
    PointStamped, PoseStamped, PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped,
    WrenchStamped,
};
use lcm_msgs::nav_msgs::{OccupancyGrid, Odometry, Path};
use lcm_msgs::sensor_msgs::{
    CameraInfo, CompressedImage, Image, Imu, JointState, Joy, PointCloud2,
};
use lcm_msgs::tf2_msgs::TFMessage;
use lcm_msgs::vision_msgs::{Detection2D, Detection2DArray, Detection3D, Detection3DArray};
use lz4_flex::frame::FrameEncoder;
use serde::Deserialize;
use tracing::{error, info, warn};
use turbojpeg::{PixelFormat, Subsamp};

use crate::store::{Observation, RecordingStore};

pub mod store;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Codec {
    Lcm,
    Jpeg,
    #[serde(rename = "lz4+lcm")]
    Lz4Lcm,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum PayloadKind {
    Raw,
    Image,
    Tf,
}

#[derive(Clone, Debug, Deserialize)]
pub struct StreamConfig {
    pub port: String,
    pub name: String,
    pub payload_type: String,
    pub codec: Codec,
    pub payload_kind: PayloadKind,
}

#[derive(Clone, Debug, Deserialize)]
pub struct RecorderConfig {
    pub store: store::RecordingStoreConfig,
    pub encoding_threads: usize,
    pub queue_capacity: usize,
    pub write_batch_size: usize,
    pub flush_interval_ms: u64,
    pub jpeg_quality: i32,
    pub streams: Vec<StreamConfig>,
}

impl RecorderConfig {
    pub fn validate(&self) -> Result<()> {
        if self.encoding_threads == 0 {
            return Err(anyhow!("encoding_threads must be at least 1"));
        }
        if self.queue_capacity == 0 {
            return Err(anyhow!("queue_capacity must be at least 1"));
        }
        if self.write_batch_size == 0 {
            return Err(anyhow!("write_batch_size must be at least 1"));
        }
        if self.flush_interval_ms == 0 {
            return Err(anyhow!("flush_interval_ms must be at least 1"));
        }
        if !(0..=100).contains(&self.jpeg_quality) {
            return Err(anyhow!("jpeg_quality must be between 0 and 100"));
        }
        Ok(())
    }
}

#[derive(Clone)]
pub struct RecorderHandle {
    sender: Sender<EncodeMessage>,
    sequence: Arc<AtomicU64>,
    accepting: Arc<Mutex<bool>>,
}

impl RecorderHandle {
    pub fn record(&self, stream: Arc<StreamConfig>, data: &[u8]) {
        let accepting = self.accepting.lock().expect("accepting lock poisoned");
        if !*accepting {
            return;
        }
        let job = EncodeJob {
            sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
            stream,
            reception_ts: wall_time(),
            data: data.to_vec(),
        };
        if self.sender.send(EncodeMessage::Job(job)).is_err() {
            warn!("recorder queue closed; message dropped");
        }
    }
}

pub struct RecorderEngine {
    handle: RecorderHandle,
    workers: Vec<JoinHandle<()>>,
    writer: Option<JoinHandle<Result<WriterStats>>>,
}

impl RecorderEngine {
    pub fn start(config: RecorderConfig) -> Result<Self> {
        config.validate()?;
        let (encode_tx, encode_rx) = bounded(config.queue_capacity);
        let (write_tx, write_rx) = bounded(config.queue_capacity);
        let accepting = Arc::new(Mutex::new(true));
        let handle = RecorderHandle {
            sender: encode_tx,
            sequence: Arc::new(AtomicU64::new(0)),
            accepting: Arc::clone(&accepting),
        };

        let writer_config = config.clone();
        let writer = thread::Builder::new()
            .name("mem2-writer".to_string())
            .spawn(move || writer_loop(writer_config, write_rx))
            .context("failed to start memory writer thread")?;

        let mut workers = Vec::with_capacity(config.encoding_threads);
        let stores_wire_bytes = config.store.stores_wire_bytes();
        for index in 0..config.encoding_threads {
            let receiver = encode_rx.clone();
            let sender = write_tx.clone();
            let quality = config.jpeg_quality;
            workers.push(
                thread::Builder::new()
                    .name(format!("mem2-encoder-{index}"))
                    .spawn(move || encode_loop(receiver, sender, quality, stores_wire_bytes))
                    .context("failed to start encoding thread")?,
            );
        }
        drop(write_tx);

        Ok(Self {
            handle,
            workers,
            writer: Some(writer),
        })
    }

    pub fn handle(&self) -> RecorderHandle {
        self.handle.clone()
    }

    pub fn shutdown(mut self) -> Result<WriterStats> {
        *self
            .handle
            .accepting
            .lock()
            .expect("accepting lock poisoned") = false;
        for _ in &self.workers {
            self.handle
                .sender
                .send(EncodeMessage::Shutdown)
                .context("failed to stop encoding worker")?;
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
struct StoredObservation {
    ts: f64,
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
    quality: i32,
    stores_wire_bytes: bool,
) {
    while let Ok(message) = receiver.recv() {
        match message {
            EncodeMessage::Job(job) => {
                let encoded = encode_job(&job, quality, stores_wire_bytes)
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

fn encode_job(
    job: &EncodeJob,
    quality: i32,
    stores_wire_bytes: bool,
) -> Result<Vec<StoredObservation>> {
    if stores_wire_bytes {
        return Ok(vec![StoredObservation {
            ts: timestamp_for(job)?,
            data: job.data.clone(),
        }]);
    }
    if job.stream.payload_kind == PayloadKind::Tf {
        return encode_tf(job);
    }

    let (ts, data) = match job.stream.codec {
        Codec::Lcm => (timestamp_for(job)?, job.data.clone()),
        Codec::Lz4Lcm => (timestamp_for(job)?, lz4_frame(&job.data)?),
        Codec::Jpeg => encode_jpeg(&job.data, job.reception_ts, quality)?,
    };
    Ok(vec![StoredObservation { ts, data }])
}

fn timestamp_for(job: &EncodeJob) -> Result<f64> {
    macro_rules! stamped {
        ($message_type:ty) => {{
            let message = <$message_type>::decode(&job.data)
                .with_context(|| format!("invalid LCM {}", job.stream.payload_type))?;
            (message.header.stamp.sec, message.header.stamp.nsec)
        }};
    }

    let (sec, nsec) = match job.stream.payload_type.as_str() {
        "dimos.msgs.geometry_msgs.PointStamped.PointStamped" => stamped!(PointStamped),
        "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped" => stamped!(PoseStamped),
        "dimos.msgs.geometry_msgs.PoseWithCovarianceStamped.PoseWithCovarianceStamped" => {
            stamped!(PoseWithCovarianceStamped)
        }
        "dimos.msgs.geometry_msgs.TwistStamped.TwistStamped" => stamped!(TwistStamped),
        "dimos.msgs.geometry_msgs.TwistWithCovarianceStamped.TwistWithCovarianceStamped" => {
            stamped!(TwistWithCovarianceStamped)
        }
        "dimos.msgs.geometry_msgs.WrenchStamped.WrenchStamped" => stamped!(WrenchStamped),
        "dimos.msgs.nav_msgs.LineSegments3D.LineSegments3D" | "dimos.msgs.nav_msgs.Path.Path" => {
            stamped!(Path)
        }
        "dimos.msgs.nav_msgs.OccupancyGrid.OccupancyGrid" => stamped!(OccupancyGrid),
        "dimos.msgs.nav_msgs.Odometry.Odometry" => stamped!(Odometry),
        "dimos.msgs.sensor_msgs.CameraInfo.CameraInfo" => stamped!(CameraInfo),
        "dimos.msgs.sensor_msgs.CompressedImage.CompressedImage" => stamped!(CompressedImage),
        "dimos.msgs.sensor_msgs.Image.Image" => stamped!(Image),
        "dimos.msgs.sensor_msgs.Imu.Imu" => stamped!(Imu),
        "dimos.msgs.sensor_msgs.JointState.JointState" => stamped!(JointState),
        "dimos.msgs.sensor_msgs.Joy.Joy" => stamped!(Joy),
        "dimos.msgs.sensor_msgs.PointCloud2.PointCloud2" => stamped!(PointCloud2),
        "dimos.msgs.vision_msgs.Detection2D.Detection2D" => stamped!(Detection2D),
        "dimos.msgs.vision_msgs.Detection2DArray.Detection2DArray" => {
            stamped!(Detection2DArray)
        }
        "dimos.msgs.vision_msgs.Detection3D.Detection3D" => stamped!(Detection3D),
        "dimos.msgs.vision_msgs.Detection3DArray.Detection3DArray" => {
            stamped!(Detection3DArray)
        }
        "dimos.msgs.foxglove_msgs.CompressedVideo.CompressedVideo" => {
            let message = CompressedVideo::decode(&job.data)
                .context("invalid LCM foxglove_msgs.CompressedVideo")?;
            (message.timestamp.sec, message.timestamp.nanosec)
        }
        "dimos.msgs.tf2_msgs.TFMessage.TFMessage" => {
            let message = TFMessage::decode(&job.data).context("invalid LCM TFMessage")?;
            let Some(transform) = message.transforms.first() else {
                return Ok(job.reception_ts);
            };
            (transform.header.stamp.sec, transform.header.stamp.nsec)
        }
        "dimos.msgs.geometry_msgs.Transform.Transform" => {
            let message = TFMessage::decode(&job.data).context("invalid LCM Transform")?;
            let Some(transform) = message.transforms.first() else {
                return Ok(job.reception_ts);
            };
            (transform.header.stamp.sec, transform.header.stamp.nsec)
        }
        _ => return Ok(job.reception_ts),
    };
    Ok(header_timestamp(sec, nsec, job.reception_ts))
}

fn encode_tf(job: &EncodeJob) -> Result<Vec<StoredObservation>> {
    if job.stream.codec != Codec::Lcm {
        return Err(anyhow!("tf only supports the lcm codec"));
    }
    let message = TFMessage::decode(&job.data).context("invalid LCM TFMessage")?;
    Ok(message
        .transforms
        .into_iter()
        .map(|transform| StoredObservation {
            ts: header_timestamp(
                transform.header.stamp.sec,
                transform.header.stamp.nsec,
                job.reception_ts,
            ),
            data: TFMessage {
                transforms: vec![transform],
            }
            .encode(),
        })
        .collect())
}

fn encode_jpeg(data: &[u8], reception_ts: f64, quality: i32) -> Result<(f64, Vec<u8>)> {
    let mut image = Image::decode(data).context("invalid LCM Image")?;
    let ts = header_timestamp(
        image.header.stamp.sec,
        image.header.stamp.nsec,
        reception_ts,
    );
    if image.encoding == "jpeg" {
        return Ok((ts, data.to_vec()));
    }

    let format = pixel_format(&image.encoding)?;
    let pixels = normalize_pixels(&image)?;
    let pitch = image.width as usize * format.size();
    let source = turbojpeg::Image {
        pixels: pixels.as_slice(),
        width: image.width as usize,
        pitch,
        height: image.height as usize,
        format,
    };
    let jpeg = turbojpeg::compress(source, quality, Subsamp::Sub2x1)
        .context("TurboJPEG compression failed")?;
    image.encoding = "jpeg".to_string();
    image.is_bigendian = 0;
    image.step = 0;
    image.data = jpeg.to_vec();
    Ok((ts, image.encode()))
}

fn pixel_format(encoding: &str) -> Result<PixelFormat> {
    match encoding {
        "rgb8" => Ok(PixelFormat::RGB),
        "bgr8" => Ok(PixelFormat::BGR),
        "rgba8" => Ok(PixelFormat::RGBA),
        "bgra8" => Ok(PixelFormat::BGRA),
        "mono8" | "mono16" | "16UC1" | "16SC1" => Ok(PixelFormat::GRAY),
        other => Err(anyhow!(
            "JPEG codec does not support image encoding {other:?}"
        )),
    }
}

fn normalize_pixels(image: &Image) -> Result<Vec<u8>> {
    match image.encoding.as_str() {
        "mono16" | "16UC1" | "16SC1" => {
            if !image.data.len().is_multiple_of(2) {
                return Err(anyhow!("16-bit image has an odd byte count"));
            }
            let high_byte = usize::from(image.is_bigendian == 0);
            Ok(image
                .data
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pixel| pixel[high_byte])
                .collect())
        }
        _ => Ok(image.data.clone()),
    }
}

fn lz4_frame(data: &[u8]) -> Result<Vec<u8>> {
    let mut encoder = FrameEncoder::new(Vec::new());
    encoder.write_all(data).context("LZ4 compression failed")?;
    encoder.finish().context("LZ4 frame finalization failed")
}

fn header_timestamp(sec: i32, nsec: i32, fallback: f64) -> f64 {
    if sec > 0 {
        f64::from(sec) + f64::from(nsec) / 1_000_000_000.0
    } else {
        fallback
    }
}

fn wall_time() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

fn writer_loop(config: RecorderConfig, receiver: Receiver<EncodedBatch>) -> Result<WriterStats> {
    let mut store = store::open(&config.store, &config.streams, config.encoding_threads)?;

    let flush_interval = Duration::from_millis(config.flush_interval_ms);
    let mut pending = BTreeMap::new();
    let mut ready = Vec::with_capacity(config.write_batch_size);
    let mut next_sequence = 0;
    let mut stats = WriterStats::default();

    loop {
        match receiver.recv_timeout(flush_interval) {
            Ok(batch) => {
                stats.received += 1;
                pending.insert(batch.sequence, batch);
                while let Some(batch) = pending.remove(&next_sequence) {
                    ready.push(batch);
                    next_sequence += 1;
                }
                if ready.len() >= config.write_batch_size {
                    write_ready(store.as_mut(), &mut ready, &mut stats)?;
                }
            }
            Err(RecvTimeoutError::Timeout) => {
                write_ready(store.as_mut(), &mut ready, &mut stats)?;
            }
            Err(RecvTimeoutError::Disconnected) => {
                while let Some(batch) = pending.remove(&next_sequence) {
                    ready.push(batch);
                    next_sequence += 1;
                }
                if !pending.is_empty() {
                    return Err(anyhow!("encoder results ended with a sequence gap"));
                }
                write_ready(store.as_mut(), &mut ready, &mut stats)?;
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
) -> Result<()> {
    if ready.is_empty() {
        return Ok(());
    }
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
                error!(stream = %batch.stream.name, error = %message, "message encoding failed");
            }
        }
    }
    store.write_batch(&stored_observations)?;
    stats.written += stored_observations.len() as u64;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::io::Read;

    use lcm_msgs::std_msgs::{Header, Time};
    use lz4_flex::frame::FrameDecoder;
    use rusqlite::Connection;
    use tempfile::NamedTempFile;

    use super::*;

    fn stream(name: &str, codec: Codec, payload_kind: PayloadKind) -> Arc<StreamConfig> {
        Arc::new(StreamConfig {
            port: name.to_string(),
            name: name.to_string(),
            payload_type: "test.Raw".to_string(),
            codec,
            payload_kind,
        })
    }

    #[test]
    fn lz4_codec_uses_the_frame_format_python_reads() {
        let compressed = lz4_frame(b"a payload worth compressing").unwrap();
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

        let (ts, encoded) = encode_jpeg(&image.encode(), 100.0, 50).unwrap();
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
            stream: stream("tf", Codec::Lcm, PayloadKind::Tf),
            reception_ts: 100.0,
            data: message.encode(),
        };

        let observations = encode_job(&job, 50, false).unwrap();

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
                payload_kind: PayloadKind::Raw,
            }),
            reception_ts: 100.0,
            data: message.encode(),
        };

        assert_eq!(timestamp_for(&job).unwrap(), 42.000_000_025);
    }

    #[test]
    fn wire_store_encoding_preserves_the_original_packet() {
        let mut message = Imu::default();
        message.header.stamp = Time { sec: 42, nsec: 25 };
        let data = message.encode();
        let job = EncodeJob {
            sequence: 0,
            stream: stream("imu", Codec::Jpeg, PayloadKind::Image),
            reception_ts: 100.0,
            data: data.clone(),
        };

        let observations = encode_job(&job, 1, true).unwrap();

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
            queue_capacity: 8,
            write_batch_size: 8,
            flush_interval_ms: 10,
            jpeg_quality: 50,
            streams: vec![(*stream("samples", Codec::Lcm, PayloadKind::Raw)).clone()],
        };
        let (sender, receiver) = bounded(8);
        let writer_config = config.clone();
        let writer = thread::spawn(move || writer_loop(writer_config, receiver));
        for sequence in [1, 0, 2] {
            sender
                .send(EncodedBatch {
                    sequence,
                    stream: stream("samples", Codec::Lcm, PayloadKind::Raw),
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
        let tf = stream("tf", Codec::Lcm, PayloadKind::Tf);
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
        let samples = stream("samples", Codec::Lcm, PayloadKind::Raw);
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
        let raw = stream("raw", Codec::Lcm, PayloadKind::Raw);
        let config = RecorderConfig {
            store: store::RecordingStoreConfig::Sqlite {
                path: file.path().to_string_lossy().into_owned(),
            },
            encoding_threads: 3,
            queue_capacity: 8,
            write_batch_size: 64,
            flush_interval_ms: 10_000,
            jpeg_quality: 50,
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
}
