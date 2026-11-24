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
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{anyhow, ensure, Context, Result};
use crossbeam_channel::{unbounded, Receiver, Sender};
use serde::{Deserialize, Serialize};
use tracing::{info, warn};
use validator::{Validate, ValidationError};

use crate::store::{self, Observation, RecordingStore, RecordingStoreConfig};
use crate::window::{
    remove_window, sync_dir, ClosedSegment, Done, SegmentReader, SegmentWriter, Syncer,
    WindowManifest,
};
use crate::{StreamConfig, WRITE_BATCH_SIZE};

const BYTES_PER_MIB: f64 = (1 << 20) as f64;

#[derive(Clone, Debug, Deserialize, Serialize, Validate)]
#[serde(deny_unknown_fields)]
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
    pub max_window_mib: f64,
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
    gate: WindowGate,
    syncer: Syncer,
    events: Option<Sender<WindowEvent>>,
    store_thread: Option<JoinHandle<Result<()>>>,
    clock: f64,
}

impl EventWriter {
    /// Creates the window directory, then opens the store and starts the sync and store threads.
    /// An existing window fails the start before the store is opened, so its recording is untouched.
    pub fn start(
        config: &EventConfig,
        store_config: &RecordingStoreConfig,
        streams: &[StreamConfig],
        compression_threads: usize,
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
        let store = store::open(store_config, streams, compression_threads).or_else(|error| {
            remove_window(&dir)?;
            Err(error)
        })?;
        let segment = SegmentWriter::create(&dir, 0)?;
        let syncer = Syncer::start(
            segment.sync_handle()?,
            Duration::from_secs_f64(config.sync_s),
        )?;

        let periods: Vec<f64> = streams
            .iter()
            .map(|stream| config.periods[&stream.name])
            .collect();
        let max_window_bytes = (config.max_window_mib * BYTES_PER_MIB) as u64;
        let window_bytes = Arc::new(AtomicU64::new(0));
        let (events, receiver) = unbounded();
        let store_writer = StoreWriter {
            store,
            streams: streams.iter().cloned().map(Arc::new).collect(),
            periods: periods.clone(),
            last_kept: vec![f64::NEG_INFINITY; streams.len()],
            stored: vec![0; streams.len()],
            clips: Clips::new(config.pre_s, config.post_s),
            pre_s: config.pre_s,
            max_window_bytes,
            window_bytes: Arc::clone(&window_bytes),
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
            gate: WindowGate::new(periods, max_window_bytes, window_bytes),
            syncer,
            events: Some(events),
            store_thread: Some(store_thread),
            clock: f64::NEG_INFINITY,
        })
    }

    /// Appends one observation to the open segment, handing the segment off first if it timed out.
    /// A full window only takes what the stream's period would keep outside a clip.
    fn append(&mut self, observation: &Observation) -> Result<()> {
        if self.segment_timed_out(observation.monotonic_ts) {
            self.hand_off_segment()?;
        }
        let index = *self
            .stream_indexes
            .get(&observation.stream.name)
            .with_context(|| format!("unknown stream {:?}", observation.stream.name))?;
        self.clock = self.clock.max(observation.monotonic_ts);
        if self
            .gate
            .admits(index, observation.monotonic_ts, self.segment.bytes)
        {
            self.segment.write(index, observation)?;
        }
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
            self.gate
                .window_bytes
                .fetch_add(segment.bytes, Ordering::SeqCst);
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

/// Holds the window to `max_bytes`: closed segments waiting for the store thread plus the open
/// one. Over it, each stream is thinned to its period before anything reaches the window.
struct WindowGate {
    periods: Vec<f64>,
    last_admitted: Vec<f64>,
    max_bytes: u64,
    window_bytes: Arc<AtomicU64>,
    full: bool,
}

impl WindowGate {
    fn new(periods: Vec<f64>, max_bytes: u64, window_bytes: Arc<AtomicU64>) -> Self {
        Self {
            last_admitted: vec![f64::NEG_INFINITY; periods.len()],
            periods,
            max_bytes,
            window_bytes,
            full: false,
        }
    }

    /// Whether an observation of `stream` at `time` goes into the window, given the open
    /// segment's `open_bytes`.
    fn admits(&mut self, stream: u16, time: f64, open_bytes: u64) -> bool {
        let stream = usize::from(stream);
        let bytes = self.window_bytes.load(Ordering::SeqCst) + open_bytes;
        let full = bytes >= self.max_bytes;
        if full != self.full {
            self.full = full;
            if full {
                warn!(
                    window_mib = bytes as f64 / BYTES_PER_MIB,
                    "event window full; thinning new data until the store catches up"
                );
            } else {
                info!(
                    window_mib = bytes as f64 / BYTES_PER_MIB,
                    "event window has room again; recording at full rate"
                );
            }
        }
        if full && time - self.last_admitted[stream] < self.periods[stream] {
            return false;
        }
        self.last_admitted[stream] = time;
        true
    }
}

/// Merged full-rate time ranges on the monotonic clock, in start order.
struct Clips {
    pre_s: f64,
    post_s: f64,
    ranges: VecDeque<(f64, f64)>,
}

impl Clips {
    fn new(pre_s: f64, post_s: f64) -> Self {
        Self {
            pre_s,
            post_s,
            ranges: VecDeque::new(),
        }
    }

    /// Adds `[time - pre_s, time + post_s]`, extending an overlapping clip.
    fn add_trigger(&mut self, time: f64) {
        let (start, end) = (time - self.pre_s, time + self.post_s);
        match self.ranges.back_mut() {
            Some(last) if start <= last.1 => last.1 = last.1.max(end),
            _ => self.ranges.push_back((start, end)),
        }
    }

    /// Whether `time` falls in a clip, dropping clips that end before it. Times must not decrease.
    fn covers(&mut self, time: f64) -> bool {
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
    max_window_bytes: u64,
    window_bytes: Arc<AtomicU64>,
    dir: PathBuf,
}

impl StoreWriter {
    /// Stores each segment once no trigger can still reach it, then drains the rest on `Finish`.
    /// Over `max_window_bytes`, the oldest segments are stored right away, before later triggers
    /// can reach them.
    /// A writer that stops without `Finish` leaves the window behind.
    fn run(mut self, events: Receiver<WindowEvent>) -> Result<()> {
        let mut segments: VecDeque<ClosedSegment> = VecDeque::new();
        let mut clock = f64::NEG_INFINITY;
        'events: loop {
            let Ok(next) = events.recv() else {
                return Ok(());
            };
            for event in std::iter::once(next).chain(events.try_iter()) {
                match event {
                    WindowEvent::Trigger(time) => self.clips.add_trigger(time),
                    WindowEvent::Segment(segment) => segments.push_back(segment),
                    WindowEvent::Clock(time) => clock = time,
                    WindowEvent::Finish => break 'events,
                }
            }
            while let Some(segment) = segments.front() {
                let window_bytes = self.window_bytes.load(Ordering::SeqCst);
                let over_cap = window_bytes > self.max_window_bytes;
                if !over_cap && clock <= segment.newest_ts + self.pre_s {
                    break;
                }
                let segment = segments.pop_front().expect("front segment exists");
                if over_cap {
                    warn!(
                        segment = segment.index,
                        window_mib = window_bytes as f64 / BYTES_PER_MIB,
                        "event window over its cap; storing the oldest segment early"
                    );
                }
                self.store_segment(&segment)?;
            }
        }
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
            if !self.clips.covers(time) && time - self.last_kept[stream] < self.periods[stream] {
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
            .with_context(|| format!("failed to delete {}", segment.path.display()))?;
        self.window_bytes.fetch_sub(segment.bytes, Ordering::SeqCst);
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

    fn event_config(window_dir: PathBuf, max_window_mib: f64) -> EventConfig {
        EventConfig {
            window_dir,
            pre_s: 60.0,
            post_s: 60.0,
            segment_s: 10.0,
            sync_s: 1.0,
            max_window_mib,
            trigger: "clip_that".to_string(),
            periods: HashMap::from([
                ("cloud".to_string(), 1.0),
                ("odom".to_string(), 0.0),
                ("clip_that".to_string(), 0.0),
            ]),
        }
    }

    /// Records 400 s of `cloud` and `odom` at 8 Hz with one trigger at 200 s, then returns the
    /// stored cloud times and the odom and trigger counts. Waits for `waiting` segments first.
    fn record_events(max_window_mib: f64, waiting: usize) -> (Vec<f64>, i64, i64) {
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
            mode: RecordingMode::Events(event_config(window.clone(), max_window_mib)),
        };
        let (sender, receiver) = bounded(256);
        let (permit_sender, permit_receiver) = bounded(256);
        let writer =
            thread::spawn(move || writer_loop(config, receiver, permit_receiver, Instant::now()));

        let streams: Vec<Arc<StreamConfig>> = streams.into_iter().map(Arc::new).collect();
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
        for &time in &times() {
            send(&streams[0], time);
            send(&streams[1], time);
            if time == 200.0 {
                send(&streams[2], time);
            }
        }

        let deadline = Instant::now() + Duration::from_secs(10);
        while segment_count(&window) != waiting {
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
        let count = |table: &str| -> i64 {
            connection
                .query_row(&format!("SELECT count(*) FROM {table}"), [], |row| {
                    row.get(0)
                })
                .unwrap()
        };
        (cloud, count("odom"), count("clip_that"))
    }

    fn times() -> Vec<f64> {
        (0..3200).map(|step| f64::from(step) * 0.125).collect()
    }

    #[test]
    fn background_is_thinned_and_a_trigger_keeps_exactly_its_clip() {
        // Segments 0 to 32 age out once the clock reaches 399.875. 33 to 38 wait, 39 is open.
        let (cloud, odom, clip_that) = record_events(1000.0, 7);
        let expected: Vec<f64> = times()
            .into_iter()
            .filter(|&time| (140.0..=260.0).contains(&time) || time.fract() == 0.0)
            .collect();
        assert_eq!(cloud, expected);
        assert_eq!(odom, 3200);
        assert_eq!(clip_that, 1);
    }

    #[test]
    fn a_window_over_its_cap_stores_segments_early_and_thins_new_data() {
        // A cap below one segment stores every closed segment at once, so only the open one waits.
        // The window is full from the first few records on, so the gate thins the clip too.
        let (cloud, odom, clip_that) = record_events(0.0001, 1);
        assert_eq!(cloud.len(), 400);
        assert!(cloud.windows(2).all(|pair| pair[1] - pair[0] >= 1.0));
        assert_eq!(odom, 3200);
        assert_eq!(clip_that, 1);
    }

    #[test]
    fn a_segment_stored_early_over_the_cap_keeps_its_clip() {
        let dir = tempfile::tempdir().unwrap();
        let database = dir.path().join("memory.db");
        let streams = vec![stream("cloud")];
        let store_config = RecordingStoreConfig::Sqlite {
            path: database.to_string_lossy().into_owned(),
        };
        let mut segment = SegmentWriter::create(dir.path(), 0).unwrap();
        let cloud = Arc::new(streams[0].clone());
        for &time in &times()[..80] {
            let observation = Observation {
                stream: Arc::clone(&cloud),
                source_ts: time,
                reception_ts: time,
                monotonic_ts: time,
                data: vec![0],
            };
            segment.write(0, &observation).unwrap();
        }
        let segment = segment.seal().unwrap();
        let store_writer = StoreWriter {
            store: store::open(&store_config, &streams, 1).unwrap(),
            streams: vec![cloud],
            periods: vec![1.0],
            last_kept: vec![f64::NEG_INFINITY],
            stored: vec![0],
            clips: Clips::new(60.0, 60.0),
            pre_s: 60.0,
            max_window_bytes: 0,
            window_bytes: Arc::new(AtomicU64::new(segment.bytes)),
            dir: dir.path().to_path_buf(),
        };
        let (events, receiver) = unbounded();
        // The trigger's clip starts at 5 s, and without `Finish` only the cap stores the segment.
        events.send(WindowEvent::Trigger(65.0)).unwrap();
        events.send(WindowEvent::Segment(segment)).unwrap();
        drop(events);
        store_writer.run(receiver).unwrap();

        let stored: Vec<f64> = Connection::open(&database)
            .unwrap()
            .prepare("SELECT json_extract(tags, '$.reception_ts') FROM cloud ORDER BY id")
            .unwrap()
            .query_map([], |row| row.get(0))
            .unwrap()
            .collect::<rusqlite::Result<_>>()
            .unwrap();
        let expected: Vec<f64> = times()[..80]
            .iter()
            .copied()
            .filter(|&time| time >= 5.0 || time.fract() == 0.0)
            .collect();
        assert_eq!(stored, expected);
    }

    #[test]
    fn a_full_window_takes_only_what_each_period_keeps_until_the_store_catches_up() {
        let window_bytes = Arc::new(AtomicU64::new(0));
        let mut gate = WindowGate::new(vec![1.0, 0.0], 100, Arc::clone(&window_bytes));
        let admitted = |gate: &mut WindowGate, stream: u16| -> Vec<f64> {
            times()[..16]
                .iter()
                .copied()
                .filter(|&time| gate.admits(stream, time, 0))
                .collect()
        };

        assert_eq!(admitted(&mut gate, 0).len(), 16);

        // Segments waiting for the store fill the window, so cloud drops to one per second.
        window_bytes.store(100, Ordering::SeqCst);
        let mut gate = WindowGate::new(vec![1.0, 0.0], 100, Arc::clone(&window_bytes));
        assert_eq!(admitted(&mut gate, 0), vec![0.0, 1.0]);
        assert_eq!(admitted(&mut gate, 1).len(), 16);

        // The open segment counts toward the window too.
        window_bytes.store(60, Ordering::SeqCst);
        let mut gate = WindowGate::new(vec![1.0, 0.0], 100, Arc::clone(&window_bytes));
        assert!(gate.admits(0, 0.0, 40));
        assert!(!gate.admits(0, 0.5, 40));
        assert!(gate.admits(0, 0.75, 39));
    }

    #[test]
    fn an_existing_window_fails_the_start_before_touching_its_recording() {
        let dir = tempfile::tempdir().unwrap();
        let window = dir.path().join("window");
        let recording = dir.path().join("memory.mcap");
        fs::write(&recording, b"crashed recording").unwrap();
        fs::create_dir(&window).unwrap();
        let store = RecordingStoreConfig::Mcap {
            path: recording.to_string_lossy().into_owned(),
            attachments: Vec::new(),
        };
        let streams = [stream("cloud"), stream("odom"), stream("clip_that")];

        assert!(EventWriter::start(&event_config(window, 1000.0), &store, &streams, 1).is_err());
        assert_eq!(fs::read(&recording).unwrap(), b"crashed recording");
    }
}
