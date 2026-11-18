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

//! Event mode: thinned background recording with full-rate clips around triggers.

use std::collections::{HashMap, HashSet, VecDeque};
use std::fs;
use std::path::PathBuf;
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{anyhow, ensure, Context, Result};
use crossbeam_channel::{unbounded, Receiver, Sender};
use serde::{Deserialize, Serialize};
use tracing::info;

use crate::store::{Observation, RecordingStore};
use crate::window::{sync_dir, ClosedSegment, DoneMarker, SegmentReader, SegmentWriter, Syncer};
use crate::{StreamConfig, WRITE_BATCH_SIZE};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EventConfig {
    pub window_dir: PathBuf,
    pub pre_s: f64,
    pub post_s: f64,
    pub segment_s: f64,
    pub sync_s: f64,
    pub max_clip_s: f64,
    pub trigger: String,
    pub periods: HashMap<String, f64>,
}

impl EventConfig {
    pub(crate) fn validate(&self, streams: &[StreamConfig]) -> Result<()> {
        for (name, value) in [("pre_s", self.pre_s), ("post_s", self.post_s)] {
            ensure!(
                value.is_finite() && value >= 0.0,
                "{name} must be nonnegative"
            );
        }
        for (name, value) in [
            ("segment_s", self.segment_s),
            ("sync_s", self.sync_s),
            ("max_clip_s", self.max_clip_s),
        ] {
            ensure!(value.is_finite() && value > 0.0, "{name} must be positive");
        }
        ensure!(
            self.max_clip_s >= self.pre_s + self.post_s,
            "max_clip_s must cover pre_s + post_s"
        );
        ensure!(
            streams.len() <= usize::from(u16::MAX),
            "event mode supports at most {} streams",
            u16::MAX
        );
        ensure!(
            streams.iter().any(|stream| stream.name == self.trigger),
            "trigger stream {:?} is not recorded",
            self.trigger
        );
        let names: HashSet<&str> = streams.iter().map(|stream| stream.name.as_str()).collect();
        let periods: HashSet<&str> = self.periods.keys().map(String::as_str).collect();
        ensure!(
            names == periods,
            "periods must name exactly the recorded streams"
        );
        for (name, period) in &self.periods {
            ensure!(
                period.is_finite() && *period >= 0.0,
                "period for {name:?} must be nonnegative"
            );
        }
        Ok(())
    }
}

/// Messages from the writer thread to the store thread, in arrival order.
enum WindowEvent {
    /// A trigger observation arrived at this monotonic time.
    Trigger(f64),
    /// A finished segment is ready to age out.
    Segment(ClosedSegment),
    /// The aging clock moved to this monotonic time.
    Clock(f64),
}

/// Writer-side half of event mode: fills window segments and drives the store thread.
pub(crate) struct EventWriter {
    dir: PathBuf,
    segment_s: f64,
    trigger: String,
    stream_indexes: HashMap<String, u16>,
    segment: SegmentWriter,
    syncer: Syncer,
    events: Option<Sender<WindowEvent>>,
    store_thread: Option<JoinHandle<Result<u64>>>,
    clock: f64,
}

impl EventWriter {
    /// Creates the window directory and starts the sync and store threads.
    pub fn start(
        config: &EventConfig,
        streams: &[StreamConfig],
        store: Box<dyn RecordingStore>,
    ) -> Result<Self> {
        let dir = config.window_dir.clone();
        let parent = match dir.parent() {
            Some(parent) if !parent.as_os_str().is_empty() => parent.to_path_buf(),
            _ => PathBuf::from("."),
        };
        fs::create_dir_all(&parent)
            .with_context(|| format!("failed to create {}", parent.display()))?;
        fs::create_dir(&dir)
            .with_context(|| format!("failed to create window directory {}", dir.display()))?;
        sync_dir(&parent)?;
        let marker = DoneMarker::create(&dir)?;
        let segment = SegmentWriter::create(&dir, 0)?;
        let syncer = Syncer::start(
            segment.sync_handle()?,
            Duration::from_secs_f64(config.sync_s),
        )?;

        let (events, receiver) = unbounded();
        let store_writer = StoreWriter {
            store,
            streams: streams.iter().cloned().map(Arc::new).collect(),
            periods: streams
                .iter()
                .map(|stream| config.periods[&stream.name])
                .collect(),
            last_kept: vec![f64::NEG_INFINITY; streams.len()],
            clips: Clips::new(config.pre_s, config.post_s, config.max_clip_s),
            pre_s: config.pre_s,
            marker,
            dir: dir.clone(),
            written: 0,
        };
        let store_thread = thread::Builder::new()
            .name("mem2-store".to_string())
            .spawn(move || store_writer.run(receiver))
            .context("failed to start memory store thread")?;

        Ok(Self {
            dir,
            segment_s: config.segment_s,
            trigger: config.trigger.clone(),
            stream_indexes: streams
                .iter()
                .enumerate()
                .map(|(index, stream)| (stream.name.clone(), index as u16))
                .collect(),
            segment,
            syncer,
            events: Some(events),
            store_thread: Some(store_thread),
            clock: f64::NEG_INFINITY,
        })
    }

    /// Reports triggers to the store thread and appends every observation to the window.
    pub fn write_batch(&mut self, observations: &[Observation]) -> Result<()> {
        for observation in observations {
            if observation.stream.name == self.trigger {
                self.send_to_store(WindowEvent::Trigger(observation.monotonic_ts))?;
            }
            self.append(observation)?;
        }
        self.segment.flush()
    }

    /// Appends one observation to the open segment, handing the segment off first if it timed out.
    fn append(&mut self, observation: &Observation) -> Result<()> {
        if self.segment_timed_out(observation.monotonic_ts) {
            self.hand_off_segment()?;
        }
        let index = *self
            .stream_indexes
            .get(&observation.stream.name)
            .with_context(|| format!("unknown stream {:?}", observation.stream.name))?;
        self.segment.write(index, observation)?;
        self.clock = self.clock.max(observation.monotonic_ts);
        Ok(())
    }

    /// Runs after each write batch: checks the helper threads and advances the aging clock.
    /// The clock moves to `now` only when `idle`, meaning no observation is still being encoded.
    pub fn tick(&mut self, now: f64, idle: bool) -> Result<()> {
        if self.syncer.is_finished() {
            return Err(self
                .syncer
                .stop()
                .err()
                .unwrap_or_else(|| anyhow!("sync thread stopped")));
        }
        if self
            .store_thread
            .as_ref()
            .is_none_or(JoinHandle::is_finished)
        {
            return Err(self.store_thread_error());
        }
        if idle {
            self.clock = self.clock.max(now);
        }
        if self.segment_timed_out(self.clock) {
            self.hand_off_segment()?;
        }
        self.send_to_store(WindowEvent::Clock(self.clock))
    }

    /// Hands over the last segment and waits for the store thread to drain the window.
    /// Returns how many observations it stored.
    pub fn finish(mut self) -> Result<u64> {
        self.send_open_segment()?;
        self.syncer.stop()?;
        self.events.take();
        let written = join_store_thread(self.store_thread.take())?;
        info!(window_dir = %self.dir.display(), "event window drained");
        Ok(written)
    }

    /// Whether `segment_s` has passed between the open segment's first record and `time`.
    fn segment_timed_out(&self, time: f64) -> bool {
        !self.segment.is_empty() && time >= self.segment.first_ts() + self.segment_s
    }

    /// Hands the current segment to the store thread and opens the next one.
    fn hand_off_segment(&mut self) -> Result<()> {
        self.send_open_segment()?;
        self.segment = SegmentWriter::create(&self.dir, self.segment.index() + 1)?;
        self.syncer.switch(self.segment.sync_handle()?)
    }

    /// Seals the open segment and sends it to the store thread if it holds records.
    fn send_open_segment(&mut self) -> Result<()> {
        let segment = self.segment.seal()?;
        if segment.records > 0 {
            self.send_to_store(WindowEvent::Segment(segment))?;
        }
        Ok(())
    }

    /// Sends `event` to the store thread, or returns the store thread's error if it has stopped.
    fn send_to_store(&mut self, event: WindowEvent) -> Result<()> {
        let sent = self
            .events
            .as_ref()
            .is_some_and(|events| events.send(event).is_ok());
        if sent {
            Ok(())
        } else {
            Err(self.store_thread_error())
        }
    }

    /// Joins the stopped store thread and returns why it stopped.
    fn store_thread_error(&mut self) -> anyhow::Error {
        self.events.take();
        match join_store_thread(self.store_thread.take()) {
            Ok(_) => anyhow!("store thread stopped early"),
            Err(error) => error,
        }
    }
}

fn join_store_thread(thread: Option<JoinHandle<Result<u64>>>) -> Result<u64> {
    thread
        .context("store thread already stopped")?
        .join()
        .map_err(|_| anyhow!("store thread panicked"))?
        .context("store thread failed")
}

/// Merged full-rate time ranges on the monotonic clock, in start order.
struct Clips {
    pre_s: f64,
    post_s: f64,
    max_clip_s: f64,
    ranges: VecDeque<(f64, f64)>,
}

impl Clips {
    fn new(pre_s: f64, post_s: f64, max_clip_s: f64) -> Self {
        Self {
            pre_s,
            post_s,
            max_clip_s,
            ranges: VecDeque::new(),
        }
    }

    /// Adds `[time - pre_s, time + post_s]`, extending an overlapping clip up to `max_clip_s`.
    fn add_trigger(&mut self, time: f64) {
        let (start, end) = (time - self.pre_s, time + self.post_s);
        match self.ranges.back_mut() {
            Some(last) if start <= last.1 => {
                last.1 = last.1.max(end).min(last.0 + self.max_clip_s);
            }
            _ => self.ranges.push_back((start, end)),
        }
    }

    /// Keeps everything from `start` on, for the shutdown clip.
    fn keep_all_from(&mut self, start: f64) {
        match self.ranges.back_mut() {
            Some(last) if start <= last.1 => last.1 = f64::INFINITY,
            _ => self.ranges.push_back((start, f64::INFINITY)),
        }
    }

    /// Whether `time` falls in a clip. Times must not decrease between calls.
    fn contains(&mut self, time: f64) -> bool {
        while self.ranges.front().is_some_and(|range| range.1 < time) {
            self.ranges.pop_front();
        }
        self.ranges.front().is_some_and(|range| range.0 <= time)
    }
}

/// Store-thread half of event mode: ages segments out of the window into the store.
struct StoreWriter {
    store: Box<dyn RecordingStore>,
    streams: Vec<Arc<StreamConfig>>,
    periods: Vec<f64>,
    last_kept: Vec<f64>,
    clips: Clips,
    pre_s: f64,
    marker: DoneMarker,
    dir: PathBuf,
    written: u64,
}

impl StoreWriter {
    /// Stores each segment once no trigger can still reach it, then drains the rest on shutdown.
    fn run(mut self, events: Receiver<WindowEvent>) -> Result<u64> {
        let mut segments = VecDeque::new();
        let mut clock = f64::NEG_INFINITY;
        for event in events {
            match event {
                WindowEvent::Trigger(time) => self.clips.add_trigger(time),
                WindowEvent::Segment(segment) => segments.push_back(segment),
                WindowEvent::Clock(time) => clock = time,
            }
            while segments
                .front()
                .is_some_and(|segment: &ClosedSegment| clock > segment.newest_ts + self.pre_s)
            {
                let segment = segments.pop_front().expect("front segment exists");
                self.store_segment(&segment)?;
            }
        }
        self.clips.keep_all_from(clock - self.pre_s);
        for segment in segments {
            self.store_segment(&segment)?;
        }
        self.store.finish()?;
        fs::remove_dir_all(&self.dir)
            .with_context(|| format!("failed to remove {}", self.dir.display()))?;
        Ok(self.written)
    }

    /// Writes the segment's kept observations, makes them durable, then deletes the segment.
    fn store_segment(&mut self, segment: &ClosedSegment) -> Result<()> {
        let mut batch = Vec::with_capacity(WRITE_BATCH_SIZE);
        let mut records = 0;
        for record in SegmentReader::open(&segment.path)? {
            let record = record?;
            records += 1;
            let stream = usize::from(record.stream);
            let time = record.monotonic_ts;
            let in_clip = self.clips.contains(time);
            if !in_clip && time - self.last_kept[stream] < self.periods[stream] {
                continue;
            }
            self.last_kept[stream] = time;
            batch.push(Observation {
                stream: Arc::clone(&self.streams[stream]),
                source_ts: record.source_ts,
                reception_ts: record.reception_ts,
                monotonic_ts: time,
                data: record.data,
            });
            if batch.len() >= WRITE_BATCH_SIZE {
                self.flush_batch(&mut batch)?;
            }
        }
        ensure!(
            records == segment.records,
            "segment {} holds {records} of {} records",
            segment.index,
            segment.records
        );
        self.flush_batch(&mut batch)?;
        self.store.sync()?;
        self.marker.write(segment.index)?;
        fs::remove_file(&segment.path)
            .with_context(|| format!("failed to delete {}", segment.path.display()))
    }

    fn flush_batch(&mut self, batch: &mut Vec<Observation>) -> Result<()> {
        if batch.is_empty() {
            return Ok(());
        }
        self.store.write_batch(batch)?;
        self.written += batch.len() as u64;
        batch.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use crossbeam_channel::bounded;
    use rusqlite::Connection;

    use super::*;
    use crate::encoding::StoredObservation;
    use crate::store::RecordingStoreConfig;
    use crate::{writer_loop, Codec, EncodedBatch, RecorderConfig, RecordingMode};

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

    fn segment_count(dir: &std::path::Path) -> usize {
        fs::read_dir(dir)
            .unwrap()
            .filter(|entry| entry.as_ref().unwrap().path().extension() == Some("seg".as_ref()))
            .count()
    }

    #[test]
    fn background_is_thinned_and_a_trigger_keeps_exactly_its_clip() {
        let dir = tempfile::tempdir().unwrap();
        let window = dir.path().join("window");
        let database = dir.path().join("memory.db");
        let streams = vec![stream("cloud"), stream("odom"), stream("clip")];
        let config = RecorderConfig {
            store: RecordingStoreConfig::Sqlite {
                path: database.to_string_lossy().into_owned(),
            },
            encoding_threads: 1,
            streams: streams.clone(),
            mode: RecordingMode::Events(EventConfig {
                window_dir: window.clone(),
                pre_s: 60.0,
                post_s: 60.0,
                segment_s: 10.0,
                sync_s: 1.0,
                max_clip_s: 600.0,
                trigger: "clip".to_string(),
                periods: HashMap::from([
                    ("cloud".to_string(), 1.0),
                    ("odom".to_string(), 0.0),
                    ("clip".to_string(), 0.0),
                ]),
            }),
        };
        let (sender, receiver) = bounded(256);
        let (permit_sender, permit_receiver) = bounded(256);
        let writer =
            thread::spawn(move || writer_loop(config, receiver, permit_receiver, Instant::now()));

        let streams: Vec<Arc<StreamConfig>> = streams.into_iter().map(Arc::new).collect();
        let times: Vec<f64> = (0..3200).map(|step| f64::from(step) * 0.125).collect();
        let mut sequence = 0;
        let mut send = |stream: &Arc<StreamConfig>, time: f64| {
            permit_sender.send(()).unwrap();
            sender
                .send(EncodedBatch {
                    sequence,
                    stream: Arc::clone(stream),
                    reception_ts: time,
                    monotonic_ts: time,
                    observations: Ok(vec![StoredObservation {
                        ts: time,
                        data: vec![0],
                    }]),
                })
                .unwrap();
            sequence += 1;
        };
        for &time in &times {
            send(&streams[0], time);
            send(&streams[1], time);
            if time == 200.0 {
                send(&streams[2], time);
            }
        }

        // Segments 0 to 32 age out once the clock reaches 399.875. 33 to 38 wait, 39 is open.
        let deadline = Instant::now() + Duration::from_secs(10);
        while segment_count(&window) != 7 {
            assert!(Instant::now() < deadline, "segments did not age out");
            thread::sleep(Duration::from_millis(10));
        }

        drop(sender);
        let stats = writer.join().unwrap().unwrap();
        assert!(!window.exists());

        let connection = Connection::open(&database).unwrap();
        let cloud: Vec<f64> = connection
            .prepare("SELECT json_extract(tags, '$.reception_ts') FROM cloud ORDER BY id")
            .unwrap()
            .query_map([], |row| row.get(0))
            .unwrap()
            .collect::<rusqlite::Result<_>>()
            .unwrap();
        let expected: Vec<f64> = times
            .iter()
            .copied()
            .filter(|&time| {
                (140.0..=260.0).contains(&time) || time >= 339.875 || time.fract() == 0.0
            })
            .collect();
        assert_eq!(cloud, expected);
        let count = |table: &str| -> i64 {
            connection
                .query_row(&format!("SELECT count(*) FROM {table}"), [], |row| {
                    row.get(0)
                })
                .unwrap()
        };
        assert_eq!(count("odom"), 3200);
        assert_eq!(count("clip"), 1);
        assert_eq!(stats.written as usize, expected.len() + 3201);
    }
}
