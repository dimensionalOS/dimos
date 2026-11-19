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

use std::borrow::Cow;
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
use validator::{Validate, ValidationError};

use crate::store::{Observation, RecordingStore, RecordingStoreConfig};
use crate::window::{
    remove_window, sync_dir, ClosedSegment, Done, SegmentReader, SegmentWriter, Syncer,
    WindowManifest,
};
use crate::{StreamConfig, WRITE_BATCH_SIZE};

#[derive(Clone, Debug, Deserialize, Serialize, Validate)]
#[serde(deny_unknown_fields)]
#[validate(schema(function = "clip_covers_window"))]
pub struct EventConfig {
    pub window_dir: PathBuf,
    #[validate(range(min = 0.0))]
    pub pre_s: f64,
    #[validate(range(min = 0.0))]
    pub post_s: f64,
    #[validate(range(exclusive_min = 0.0))]
    pub segment_s: f64,
    #[validate(range(exclusive_min = 0.0))]
    pub sync_s: f64,
    #[validate(range(exclusive_min = 0.0))]
    pub max_clip_s: f64,
    pub trigger: String,
    #[validate(custom(function = "nonnegative_periods"))]
    pub periods: HashMap<String, f64>,
}

impl EventConfig {
    /// The trigger must be a recorded stream, and `periods` must name exactly the recorded streams.
    pub(crate) fn check_streams(&self, streams: &[StreamConfig]) -> Result<(), ValidationError> {
        if !streams.iter().any(|stream| stream.name == self.trigger) {
            return Err(invalid(format!(
                "trigger stream {:?} is not recorded",
                self.trigger
            )));
        }
        let names: HashSet<&str> = streams.iter().map(|stream| stream.name.as_str()).collect();
        if names != self.periods.keys().map(String::as_str).collect() {
            return Err(invalid("periods must name exactly the recorded streams"));
        }
        Ok(())
    }
}

fn clip_covers_window(config: &EventConfig) -> Result<(), ValidationError> {
    if config.max_clip_s < config.pre_s + config.post_s {
        return Err(invalid("max_clip_s must cover pre_s + post_s"));
    }
    Ok(())
}

fn nonnegative_periods(periods: &HashMap<String, f64>) -> Result<(), ValidationError> {
    if periods.values().any(|period| *period < 0.0) {
        return Err(invalid("periods must be nonnegative"));
    }
    Ok(())
}

fn invalid(message: impl Into<Cow<'static, str>>) -> ValidationError {
    ValidationError::new("event_config").with_message(message.into())
}

/// Messages from the writer thread to the store thread, in arrival order.
enum WindowEvent {
    /// A trigger observation arrived at this monotonic time.
    Trigger(f64),
    /// A finished segment is ready to age out.
    Segment(ClosedSegment),
    /// The aging clock moved to this monotonic time.
    Clock(f64),
    /// The recording stopped cleanly, so drain the window into the store.
    Finish,
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
    store_thread: Option<JoinHandle<Result<()>>>,
    clock: f64,
}

impl EventWriter {
    /// Creates the window directory and starts the sync and store threads.
    pub fn start(
        config: &EventConfig,
        store_config: &RecordingStoreConfig,
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
        WindowManifest {
            store: store_config.clone(),
            streams: streams.to_vec(),
        }
        .write(&dir)?;
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
            stored: vec![0; streams.len()],
            clips: Clips::new(config.pre_s, config.post_s, config.max_clip_s),
            pre_s: config.pre_s,
            dir: dir.clone(),
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

    /// Whether `segment_s` has passed between the open segment's first record and `time`.
    fn segment_timed_out(&self, time: f64) -> bool {
        self.segment.records > 0 && time >= self.segment.first_ts + self.segment_s
    }

    /// Hands the current segment to the store thread and opens the next one.
    fn hand_off_segment(&mut self) -> Result<()> {
        self.send_open_segment()?;
        self.segment = SegmentWriter::create(&self.dir, self.segment.index + 1)?;
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
        match &self.events {
            Some(events) if events.send(event).is_ok() => Ok(()),
            _ => self
                .join_store_thread()
                .and(Err(anyhow!("store thread stopped early"))),
        }
    }

    fn join_store_thread(&mut self) -> Result<()> {
        self.events.take();
        self.store_thread
            .take()
            .context("store thread already stopped")?
            .join()
            .map_err(|_| anyhow!("store thread panicked"))?
            .context("store thread failed")
    }
}

impl RecordingStore for EventWriter {
    /// Reports triggers to the store thread and appends every observation to the window.
    fn write_batch(&mut self, observations: &[Observation]) -> Result<()> {
        for observation in observations {
            if observation.stream.name == self.trigger {
                self.send_to_store(WindowEvent::Trigger(observation.monotonic_ts))?;
            }
            self.append(observation)?;
        }
        self.segment.flush()
    }

    fn sync(&mut self) -> Result<()> {
        self.segment.flush()?;
        self.segment.sync_handle()?.sync_data()?;
        Ok(())
    }

    /// Advances the aging clock, to `now` only when `idle`, and hands off a timed-out segment.
    fn tick(&mut self, now: f64, idle: bool) -> Result<()> {
        if idle {
            self.clock = self.clock.max(now);
        }
        if self.segment_timed_out(self.clock) {
            self.hand_off_segment()?;
        }
        self.send_to_store(WindowEvent::Clock(self.clock))
    }

    /// Hands over the last segment and waits for the store thread to drain the window.
    fn finish(&mut self) -> Result<()> {
        self.send_open_segment()?;
        self.syncer.stop()?;
        self.send_to_store(WindowEvent::Finish)?;
        self.join_store_thread()?;
        info!(window_dir = %self.dir.display(), "event window drained");
        Ok(())
    }
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
    stored: Vec<u64>,
    clips: Clips,
    pre_s: f64,
    dir: PathBuf,
}

impl StoreWriter {
    /// Stores each segment once no trigger can still reach it, then drains the rest on `Finish`.
    /// A writer that stops without `Finish` leaves the window for `dimos-memory-recover`.
    fn run(mut self, events: Receiver<WindowEvent>) -> Result<()> {
        let mut segments = VecDeque::new();
        let mut clock = f64::NEG_INFINITY;
        loop {
            match events.recv() {
                Ok(WindowEvent::Trigger(time)) => self.clips.add_trigger(time),
                Ok(WindowEvent::Segment(segment)) => segments.push_back(segment),
                Ok(WindowEvent::Clock(time)) => clock = time,
                Ok(WindowEvent::Finish) => break,
                Err(_) => return Ok(()),
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
        remove_window(&self.dir)
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
            if !self.clips.contains(time) && time - self.last_kept[stream] < self.periods[stream] {
                continue;
            }
            self.last_kept[stream] = time;
            self.stored[stream] += 1;
            batch.push(record.into_observation(Arc::clone(&self.streams[stream])));
            if batch.len() >= WRITE_BATCH_SIZE {
                self.store.write_batch(&batch)?;
                batch.clear();
            }
        }
        ensure!(
            records == segment.records,
            "segment {} holds {records} of {} records",
            segment.index,
            segment.records
        );
        self.store.write_batch(&batch)?;
        self.store.sync()?;
        Done {
            segment: segment.index,
            stored: self.stored.clone(),
        }
        .write(&self.dir)?;
        fs::remove_file(&segment.path)
            .with_context(|| format!("failed to delete {}", segment.path.display()))
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
        let streams = vec![stream("cloud"), stream("odom"), stream("clip_that")];
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
                trigger: "clip_that".to_string(),
                periods: HashMap::from([
                    ("cloud".to_string(), 1.0),
                    ("odom".to_string(), 0.0),
                    ("clip_that".to_string(), 0.0),
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
        writer.join().unwrap().unwrap();
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
        assert_eq!(count("clip_that"), 1);
    }
}
