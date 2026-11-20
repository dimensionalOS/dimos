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

//! On-disk window segments that hold recent observations before the main file.

use std::fs::{self, File, OpenOptions};
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{unbounded, RecvTimeoutError, Sender};
use serde::Serialize;

use crate::store::{Observation, RecordingStoreConfig};
use crate::StreamConfig;

const HEADER_SIZE: usize = 4 + 2 + 3 * 8;
const CRC_SIZE: usize = 4;
const MANIFEST_FILE: &str = "recording.json";
const DONE_FILE: &str = "done.json";

/// One observation read back from a segment.
pub(crate) struct Record {
    pub stream: u16,
    pub source_ts: f64,
    pub reception_ts: f64,
    pub monotonic_ts: f64,
    pub data: Vec<u8>,
}

impl Record {
    pub fn into_observation(self, stream: Arc<StreamConfig>) -> Observation {
        Observation {
            stream,
            source_ts: self.source_ts,
            reception_ts: self.reception_ts,
            monotonic_ts: self.monotonic_ts,
            data: self.data,
        }
    }
}

/// A finished segment handed to the store thread.
#[derive(Debug)]
pub(crate) struct ClosedSegment {
    pub index: u64,
    pub path: PathBuf,
    pub newest_ts: f64,
    pub records: u64,
    pub bytes: u64,
}

pub(crate) fn segment_path(dir: &Path, index: u64) -> PathBuf {
    dir.join(format!("{index}.seg"))
}

/// Removes the manifest first, so a removal cut short leaves no window that names a recording.
pub(crate) fn remove_window(dir: &Path) -> Result<()> {
    let manifest = dir.join(MANIFEST_FILE);
    fs::remove_file(&manifest)
        .with_context(|| format!("failed to remove {}", manifest.display()))?;
    sync_dir(dir)?;
    fs::remove_dir_all(dir).with_context(|| format!("failed to remove {}", dir.display()))
}

pub(crate) fn sync_dir(dir: &Path) -> Result<()> {
    File::open(dir)
        .and_then(|directory| directory.sync_all())
        .with_context(|| format!("failed to sync directory {}", dir.display()))
}

/// Appends framed records to one segment file.
pub(crate) struct SegmentWriter {
    pub index: u64,
    pub records: u64,
    pub first_ts: f64,
    newest_ts: f64,
    bytes: u64,
    path: PathBuf,
    file: BufWriter<File>,
}

impl SegmentWriter {
    /// Creates segment `index` in `dir` and syncs the directory so its name survives power loss.
    pub fn create(dir: &Path, index: u64) -> Result<Self> {
        let path = segment_path(dir, index);
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .with_context(|| format!("failed to create segment {}", path.display()))?;
        sync_dir(dir)?;
        Ok(Self {
            index,
            records: 0,
            first_ts: f64::NEG_INFINITY,
            newest_ts: f64::NEG_INFINITY,
            bytes: 0,
            path,
            file: BufWriter::new(file),
        })
    }

    /// A second handle to the file, for syncing it off the writer thread.
    pub fn sync_handle(&self) -> Result<File> {
        self.file
            .get_ref()
            .try_clone()
            .context("failed to clone the segment handle")
    }

    /// Buffers one framed record. Call `flush` to hand it to the kernel.
    pub fn write(&mut self, stream: u16, observation: &Observation) -> Result<()> {
        let length = u32::try_from(observation.data.len())
            .context("observation exceeds the segment record size")?;
        let mut header = [0; HEADER_SIZE];
        header[..4].copy_from_slice(&length.to_le_bytes());
        header[4..6].copy_from_slice(&stream.to_le_bytes());
        header[6..14].copy_from_slice(&observation.source_ts.to_le_bytes());
        header[14..22].copy_from_slice(&observation.reception_ts.to_le_bytes());
        header[22..30].copy_from_slice(&observation.monotonic_ts.to_le_bytes());
        let mut crc = crc32fast::Hasher::new();
        crc.update(&header);
        crc.update(&observation.data);
        self.file.write_all(&header)?;
        self.file.write_all(&observation.data)?;
        self.file.write_all(&crc.finalize().to_le_bytes())?;
        if self.records == 0 {
            self.first_ts = observation.monotonic_ts;
        }
        self.records += 1;
        self.bytes += (HEADER_SIZE + observation.data.len() + CRC_SIZE) as u64;
        self.newest_ts = self.newest_ts.max(observation.monotonic_ts);
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.file
            .flush()
            .with_context(|| format!("failed to flush segment {}", self.path.display()))
    }

    /// Flushes the segment and describes it for the store thread. Nothing more may be written.
    pub fn seal(&mut self) -> Result<ClosedSegment> {
        self.flush()?;
        Ok(ClosedSegment {
            index: self.index,
            path: self.path.clone(),
            newest_ts: self.newest_ts,
            records: self.records,
            bytes: self.bytes,
        })
    }
}

/// Reads records in order and stops at the first truncated or corrupt one.
pub(crate) struct SegmentReader {
    file: BufReader<File>,
    remaining: u64,
}

impl SegmentReader {
    pub fn open(path: &Path) -> Result<Self> {
        let file =
            File::open(path).with_context(|| format!("failed to open {}", path.display()))?;
        let remaining = file.metadata()?.len();
        Ok(Self {
            file: BufReader::new(file),
            remaining,
        })
    }

    fn read_record(&mut self) -> Result<Option<Record>> {
        let mut header = [0; HEADER_SIZE];
        if self.remaining < HEADER_SIZE as u64 {
            return self.end();
        }
        self.file.read_exact(&mut header)?;
        let length = u32::from_le_bytes(header[..4].try_into()?) as usize;
        let record_size = (HEADER_SIZE + length + CRC_SIZE) as u64;
        if self.remaining < record_size {
            return self.end();
        }
        let mut data = vec![0; length];
        self.file.read_exact(&mut data)?;
        let mut stored_crc = [0; CRC_SIZE];
        self.file.read_exact(&mut stored_crc)?;
        let mut crc = crc32fast::Hasher::new();
        crc.update(&header);
        crc.update(&data);
        if crc.finalize() != u32::from_le_bytes(stored_crc) {
            return self.end();
        }
        self.remaining -= record_size;
        Ok(Some(Record {
            stream: u16::from_le_bytes(header[4..6].try_into()?),
            source_ts: f64::from_le_bytes(header[6..14].try_into()?),
            reception_ts: f64::from_le_bytes(header[14..22].try_into()?),
            monotonic_ts: f64::from_le_bytes(header[22..30].try_into()?),
            data,
        }))
    }

    /// Ends iteration at a torn or corrupt record.
    fn end(&mut self) -> Result<Option<Record>> {
        self.remaining = 0;
        Ok(None)
    }
}

impl Iterator for SegmentReader {
    type Item = Result<Record>;

    fn next(&mut self) -> Option<Self::Item> {
        self.read_record().transpose()
    }
}

/// The recording a window belongs to: its store and its streams in segment index order.
#[derive(Serialize)]
pub(crate) struct WindowManifest {
    pub store: RecordingStoreConfig,
    pub streams: Vec<StreamConfig>,
}

impl WindowManifest {
    pub fn write(&self, dir: &Path) -> Result<()> {
        let path = dir.join(MANIFEST_FILE);
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .with_context(|| format!("failed to create {}", path.display()))?;
        serde_json::to_writer(&file, self)?;
        file.sync_data()
            .with_context(|| format!("failed to sync {}", path.display()))?;
        sync_dir(dir)
    }
}

/// The newest segment durable in the store file, and how many observations of each stream it holds.
#[derive(Serialize)]
pub(crate) struct Done {
    pub segment: u64,
    pub stored: Vec<u64>,
}

impl Done {
    /// Replaces the marker through a rename, so a crash leaves either the old or the new one.
    pub fn write(&self, dir: &Path) -> Result<()> {
        let temporary = dir.join(format!("{DONE_FILE}.tmp"));
        let file = File::create(&temporary)
            .with_context(|| format!("failed to create {}", temporary.display()))?;
        serde_json::to_writer(&file, self)?;
        file.sync_data().context("failed to sync the done marker")?;
        fs::rename(&temporary, dir.join(DONE_FILE)).context("failed to replace the done marker")?;
        sync_dir(dir)
    }
}

/// Background thread that syncs the open segment every interval.
pub(crate) struct Syncer {
    handles: Option<Sender<File>>,
    thread: Option<JoinHandle<Result<()>>>,
}

impl Syncer {
    /// Starts syncing `first` every `interval`.
    pub fn start(first: File, interval: Duration) -> Result<Self> {
        let (handles, receiver) = unbounded::<File>();
        let thread = thread::Builder::new()
            .name("mem2-syncer".to_string())
            .spawn(move || {
                let mut current = first;
                loop {
                    match receiver.recv_timeout(interval) {
                        Ok(next) => {
                            current.sync_data()?;
                            current = next;
                        }
                        Err(RecvTimeoutError::Timeout) => current.sync_data()?,
                        Err(RecvTimeoutError::Disconnected) => {
                            current.sync_data()?;
                            return Ok(());
                        }
                    }
                }
            })
            .context("failed to start memory sync thread")?;
        Ok(Self {
            handles: Some(handles),
            thread: Some(thread),
        })
    }

    /// Syncs the previous segment once more, then follows `next`.
    pub fn switch(&mut self, next: File) -> Result<()> {
        match &self.handles {
            Some(handles) if handles.send(next).is_ok() => Ok(()),
            _ => self.stop().and(Err(anyhow!("sync thread stopped"))),
        }
    }

    /// Syncs the current segment one last time and joins the thread.
    pub fn stop(&mut self) -> Result<()> {
        self.handles.take();
        match self.thread.take() {
            Some(thread) => thread
                .join()
                .map_err(|_| anyhow!("sync thread panicked"))?
                .context("window segment sync failed"),
            None => Ok(()),
        }
    }
}
