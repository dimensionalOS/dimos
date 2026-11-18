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

use std::fs::{File, OpenOptions};
use std::io::{BufReader, BufWriter, Read, Write};
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{unbounded, RecvTimeoutError, Sender};

use crate::store::Observation;

const LENGTH_SIZE: usize = 4;
const HEADER_SIZE: usize = LENGTH_SIZE + 2 + 3 * 8;
const CRC_SIZE: usize = 4;

/// One observation read back from a segment.
pub(crate) struct Record {
    pub stream: u16,
    pub source_ts: f64,
    pub reception_ts: f64,
    pub monotonic_ts: f64,
    pub data: Vec<u8>,
}

/// A finished segment handed to the store thread.
#[derive(Debug)]
pub(crate) struct ClosedSegment {
    pub index: u64,
    pub path: PathBuf,
    pub newest_ts: f64,
    pub records: u64,
}

pub(crate) fn segment_path(dir: &Path, index: u64) -> PathBuf {
    dir.join(format!("{index}.seg"))
}

pub(crate) fn sync_dir(dir: &Path) -> Result<()> {
    File::open(dir)
        .and_then(|directory| directory.sync_all())
        .with_context(|| format!("failed to sync directory {}", dir.display()))
}

/// Appends framed records to one segment file.
pub(crate) struct SegmentWriter {
    index: u64,
    path: PathBuf,
    file: BufWriter<File>,
    records: u64,
    first_ts: f64,
    newest_ts: f64,
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
            path,
            file: BufWriter::new(file),
            records: 0,
            first_ts: f64::NEG_INFINITY,
            newest_ts: f64::NEG_INFINITY,
        })
    }

    pub fn index(&self) -> u64 {
        self.index
    }

    pub fn is_empty(&self) -> bool {
        self.records == 0
    }

    pub fn first_ts(&self) -> f64 {
        self.first_ts
    }

    /// A second handle to the file, for the sync thread.
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
        if self.remaining < HEADER_SIZE as u64 {
            return Ok(None);
        }
        let mut header = [0; HEADER_SIZE];
        self.file.read_exact(&mut header)?;
        let length = u32::from_le_bytes(header[..4].try_into()?) as usize;
        let record_size = (HEADER_SIZE + length + CRC_SIZE) as u64;
        if self.remaining < record_size {
            return Ok(None);
        }
        let mut data = vec![0; length];
        self.file.read_exact(&mut data)?;
        let mut stored_crc = [0; CRC_SIZE];
        self.file.read_exact(&mut stored_crc)?;
        let mut crc = crc32fast::Hasher::new();
        crc.update(&header);
        crc.update(&data);
        if crc.finalize() != u32::from_le_bytes(stored_crc) {
            return Ok(None);
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
}

impl Iterator for SegmentReader {
    type Item = Result<Record>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.read_record() {
            Ok(Some(record)) => Some(Ok(record)),
            Ok(None) => {
                self.remaining = 0;
                None
            }
            Err(error) => {
                self.remaining = 0;
                Some(Err(error))
            }
        }
    }
}

/// Index of the newest segment already durable in the main file.
pub(crate) struct DoneMarker {
    file: File,
}

impl DoneMarker {
    /// Creates the empty marker file and syncs the directory.
    pub fn create(dir: &Path) -> Result<Self> {
        let path = dir.join("done");
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .with_context(|| format!("failed to create {}", path.display()))?;
        sync_dir(dir)?;
        Ok(Self { file })
    }

    /// Records `index` as done, overwriting the previous index in place.
    pub fn write(&mut self, index: u64) -> Result<()> {
        self.file.write_all_at(&index.to_le_bytes(), 0)?;
        self.file
            .sync_data()
            .context("failed to sync the window done marker")
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
        let sent = self
            .handles
            .as_ref()
            .is_some_and(|handles| handles.send(next).is_ok());
        if sent {
            Ok(())
        } else {
            Err(self
                .stop()
                .err()
                .unwrap_or_else(|| anyhow!("sync thread stopped")))
        }
    }

    pub fn is_finished(&self) -> bool {
        self.thread.as_ref().is_none_or(JoinHandle::is_finished)
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
