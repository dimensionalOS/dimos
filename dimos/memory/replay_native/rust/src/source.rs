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

//! Reading a recording's streams as LCM wire bytes, from a memory `.db` or a dimos `.mcap`.

use std::collections::{BTreeMap, HashMap};
use std::io::Read;
use std::path::Path;

use anyhow::{bail, Context, Result};
use rusqlite::{Connection, OpenFlags};

/// How a stream's stored bytes relate to its LCM wire bytes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Storage {
    /// Already the wire bytes: `lcm`, and `jpeg` (an LCM Image whose `encoding` is `jpeg`).
    Wire,
    /// An LZ4 frame around the wire bytes.
    Lz4,
    /// Anything else (pickle, json, cdr, ...), named by its codec.
    Unsupported(String),
}

impl Storage {
    pub fn from_codec(codec: &str) -> Self {
        match codec {
            "lcm" | "jpeg" => Storage::Wire,
            "lz4+lcm" | "lz4+jpeg" => Storage::Lz4,
            other => Storage::Unsupported(other.to_string()),
        }
    }

    pub fn to_wire(&self, data: Vec<u8>) -> Result<Vec<u8>> {
        match self {
            Storage::Wire => Ok(data),
            Storage::Lz4 => {
                let mut out = Vec::new();
                lz4_flex::frame::FrameDecoder::new(data.as_slice())
                    .read_to_end(&mut out)
                    .context("lz4 decompression failed")?;
                Ok(out)
            }
            Storage::Unsupported(codec) => bail!("{codec} is not an LCM-compatible codec"),
        }
    }
}

/// One stream's rows inside a `[start, end]` window.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WindowStats {
    pub count: u64,
    pub first_ts: f64,
    pub last_ts: f64,
}

/// Which requested stream (and at what time) an mcap message belongs to, if any.
type Wanted<'a> =
    dyn Fn(&mcap::Channel, &mcap::records::MessageHeader) -> Option<(usize, f64)> + 'a;

/// A stored message: index into the requested streams, timestamp, stored bytes.
pub type Visit<'a> = dyn FnMut(usize, f64, Vec<u8>) -> Result<bool> + 'a;

pub enum Source {
    Db(DbSource),
    Mcap(Box<McapSource>),
}

impl Source {
    pub fn open(path: &Path) -> Result<Self> {
        let is_mcap = path
            .extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| e.eq_ignore_ascii_case("mcap"));
        Ok(if is_mcap {
            Source::Mcap(Box::new(McapSource::open(path)?))
        } else {
            Source::Db(DbSource::open(path)?)
        })
    }

    /// Every stream in the file and its storage.
    pub fn storages(&self) -> &BTreeMap<String, Storage> {
        match self {
            Source::Db(s) => &s.storages,
            Source::Mcap(s) => &s.index.storages,
        }
    }

    /// Earliest timestamp of any stream in the file: what `seek` counts from.
    pub fn recording_first(&self) -> Result<Option<f64>> {
        match self {
            Source::Db(s) => s.recording_first(),
            Source::Mcap(s) => Ok(s.recording_first()),
        }
    }

    pub fn stats(&self, name: &str, start: f64, end: f64) -> Result<WindowStats> {
        match self {
            Source::Db(s) => s.stats(name, start, end),
            Source::Mcap(s) => Ok(s.stats(name, start, end)),
        }
    }

    /// Calls `visit` with each stored message of `names` inside `[start, end]`, in timestamp
    /// order (file order for an mcap), until it returns false.
    pub fn for_each(
        &self,
        names: &[String],
        start: f64,
        end: f64,
        visit: &mut Visit,
    ) -> Result<()> {
        match self {
            Source::Db(s) => s.for_each(names, start, end, visit),
            Source::Mcap(s) => s.for_each(names, start, end, visit),
        }
    }

    /// The first stored message of `name` inside `[start, end]`.
    pub fn first(&self, name: &str, start: f64, end: f64) -> Result<Option<Vec<u8>>> {
        let mut found = None;
        self.for_each(&[name.to_string()], start, end, &mut |_, _, data| {
            found = Some(data);
            Ok(false)
        })?;
        Ok(found)
    }
}

// ---------------------------------------------------------------- memory .db

pub struct DbSource {
    connection: Connection,
    storages: BTreeMap<String, Storage>,
}

/// SQLite identifiers are quoted, not bound, so a stream name has to be escaped.
fn quote(identifier: &str) -> String {
    format!("\"{}\"", identifier.replace('"', "\"\""))
}

impl DbSource {
    fn open(path: &Path) -> Result<Self> {
        let connection = Connection::open_with_flags(path, OpenFlags::SQLITE_OPEN_READ_ONLY)
            .with_context(|| format!("failed to open {}", path.display()))?;
        let rows = connection
            .prepare("SELECT name, config FROM _streams")
            .context("no _streams table: is this a dimos memory recording?")?
            .query_map([], |row| {
                Ok((row.get::<_, String>(0)?, row.get::<_, String>(1)?))
            })?
            .collect::<rusqlite::Result<Vec<_>>>()?;
        let mut storages = BTreeMap::new();
        for (name, config) in rows {
            let config: serde_json::Value =
                serde_json::from_str(&config).with_context(|| format!("bad config for {name}"))?;
            let codec = config["codec_id"].as_str().unwrap_or_default();
            storages.insert(name, Storage::from_codec(codec));
        }
        Ok(Self {
            connection,
            storages,
        })
    }

    fn recording_first(&self) -> Result<Option<f64>> {
        let mut first: Option<f64> = None;
        for name in self.storages.keys() {
            let ts: Option<f64> = self.connection.query_row(
                &format!("SELECT MIN(ts) FROM {}", quote(name)),
                [],
                |r| r.get(0),
            )?;
            if let Some(ts) = ts {
                first = Some(first.map_or(ts, |f| f.min(ts)));
            }
        }
        Ok(first)
    }

    fn stats(&self, name: &str, start: f64, end: f64) -> Result<WindowStats> {
        let sql = format!(
            "SELECT COUNT(*), MIN(ts), MAX(ts) FROM {} WHERE ts >= ?1 AND ts <= ?2",
            quote(name)
        );
        let (count, first, last): (i64, Option<f64>, Option<f64>) =
            self.connection.query_row(&sql, [start, end], |r| {
                Ok((r.get(0)?, r.get(1)?, r.get(2)?))
            })?;
        Ok(WindowStats {
            count: count as u64,
            first_ts: first.unwrap_or(0.0),
            last_ts: last.unwrap_or(0.0),
        })
    }

    fn for_each(&self, names: &[String], start: f64, end: f64, visit: &mut Visit) -> Result<()> {
        if names.is_empty() {
            return Ok(());
        }
        // Each arm walks one stream's ts index; SQLite merges the ordered arms as it steps.
        let arms: Vec<String> = names
            .iter()
            .enumerate()
            .map(|(index, name)| {
                format!(
                    "SELECT {index} AS s, t.ts AS ts, b.data AS data FROM {} t JOIN {} b ON b.id = t.id \
                     WHERE t.ts >= ?1 AND t.ts <= ?2",
                    quote(name),
                    quote(&format!("{name}_blob"))
                )
            })
            .collect();
        let sql = format!("{} ORDER BY ts", arms.join(" UNION ALL "));
        let mut statement = self.connection.prepare(&sql)?;
        let mut rows = statement.query([start, end])?;
        while let Some(row) = rows.next()? {
            let index: i64 = row.get(0)?;
            if !visit(index as usize, row.get(1)?, row.get(2)?)? {
                break;
            }
        }
        Ok(())
    }
}

// --------------------------------------------------------------------- .mcap

pub struct McapSource {
    mapped: memmap2::Mmap,
    index: McapIndex,
    /// Present when the file is indexed, which lets playback go in log-time order.
    summary: Option<mcap::Summary>,
}

/// What one scan of an mcap learns about its channels.
#[derive(Default)]
struct McapIndex {
    storages: BTreeMap<String, Storage>,
    /// Channel topic -> stream name.
    stream_of: HashMap<String, String>,
    /// Stream name -> every message timestamp, sorted.
    timestamps: HashMap<String, Vec<f64>>,
    /// Channel topic -> whether its observation time is `publish_time` (else `log_time`).
    uses_publish_time: HashMap<String, bool>,
}

/// The stream name `McapStore` gives a channel: a dimos wire topic `dimos/<port>/<msg_name>`
/// is named by its port, anything else by its topic, `rt/` dropped, `/` -> `_`.
pub fn stream_name(topic: &str) -> String {
    if let Some(rest) = topic.strip_prefix("dimos/") {
        if let Some((port, _msg_name)) = rest.rsplit_once('/') {
            if !port.is_empty() {
                return port.replace('/', "_");
            }
        }
    }
    topic.strip_prefix("rt/").unwrap_or(topic).replace('/', "_")
}

fn observation_ns(uses_publish_time: bool, message: &mcap::Message) -> u64 {
    if uses_publish_time {
        message.publish_time
    } else {
        message.log_time
    }
}

impl McapIndex {
    fn add_channel(&mut self, channel: &mcap::Channel) {
        if self.stream_of.contains_key(&channel.topic) {
            return;
        }
        let name = stream_name(&channel.topic);
        self.storages
            .entry(name.clone())
            .or_insert_with(|| Storage::from_codec(&channel.message_encoding));
        self.stream_of.insert(channel.topic.clone(), name);
        let publish = channel
            .metadata
            .get("dimos.observation_time")
            .map(String::as_str)
            == Some("publish_time");
        self.uses_publish_time
            .insert(channel.topic.clone(), publish);
    }
}

impl McapSource {
    fn open(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to open {}", path.display()))?;
        // Safety: read-only; recordings are finalized before they are replayed.
        let mapped = unsafe { memmap2::Mmap::map(&file) }
            .with_context(|| format!("failed to mmap {}", path.display()))?;
        let mut index = McapIndex::default();
        // The summary lists channels that never got a message; the scan finds the rest.
        let summary = mcap::Summary::read(&mapped)?;
        for channel in summary.iter().flat_map(|s| s.channels.values()) {
            index.add_channel(channel);
        }
        let mut timestamps: HashMap<String, Vec<f64>> = HashMap::new();
        for message in mcap::MessageStream::new(&mapped)? {
            let message = message?;
            index.add_channel(&message.channel);
            let topic = &message.channel.topic;
            let ns = observation_ns(index.uses_publish_time[topic], &message);
            timestamps
                .entry(index.stream_of[topic].clone())
                .or_default()
                .push(ns as f64 / 1e9);
        }
        for ts in timestamps.values_mut() {
            ts.sort_by(f64::total_cmp);
        }
        index.timestamps = timestamps;
        Ok(Self {
            mapped,
            index,
            summary,
        })
    }

    fn recording_first(&self) -> Option<f64> {
        self.index
            .timestamps
            .values()
            .filter_map(|ts| ts.first().copied())
            .reduce(f64::min)
    }

    fn stats(&self, name: &str, start: f64, end: f64) -> WindowStats {
        let Some(ts) = self.index.timestamps.get(name) else {
            return WindowStats::default();
        };
        let lo = ts.partition_point(|&t| t < start);
        let hi = ts.partition_point(|&t| t <= end);
        if lo >= hi {
            return WindowStats::default();
        }
        WindowStats {
            count: (hi - lo) as u64,
            first_ts: ts[lo],
            last_ts: ts[hi - 1],
        }
    }

    fn for_each(&self, names: &[String], start: f64, end: f64, visit: &mut Visit) -> Result<()> {
        let index_of: HashMap<&str, usize> = names
            .iter()
            .enumerate()
            .map(|(i, n)| (n.as_str(), i))
            .collect();
        // Index into `names` of a message's channel, and its observation time, if it is wanted.
        let wanted = |channel: &mcap::Channel, header: &mcap::records::MessageHeader| {
            let index = *index_of.get(self.index.stream_of[&channel.topic].as_str())?;
            let ns = match self.index.uses_publish_time[&channel.topic] {
                true => header.publish_time,
                false => header.log_time,
            };
            let ts = ns as f64 / 1e9;
            (start..=end).contains(&ts).then_some((index, ts))
        };
        match &self.summary {
            Some(summary) if !summary.chunk_indexes.is_empty() => {
                self.for_each_indexed(summary, &wanted, visit)
            }
            // An unindexed file plays in file order.
            _ => {
                for message in mcap::MessageStream::new(&self.mapped)? {
                    let message = message?;
                    let header = mcap::records::MessageHeader {
                        channel_id: message.channel.id,
                        sequence: message.sequence,
                        log_time: message.log_time,
                        publish_time: message.publish_time,
                    };
                    if let Some((index, ts)) = wanted(&message.channel, &header) {
                        if !visit(index, ts, message.data.into_owned())? {
                            break;
                        }
                    }
                }
                Ok(())
            }
        }
    }

    /// Visits messages in log-time order: a recorder may write one channel after another, so file
    /// order can be minutes away from time order.
    fn for_each_indexed(
        &self,
        summary: &mcap::Summary,
        wanted: &Wanted,
        visit: &mut Visit,
    ) -> Result<()> {
        use mcap::sans_io::{IndexedReadEvent, IndexedReader, IndexedReaderOptions};
        let options = IndexedReaderOptions::new()
            .with_order(mcap::sans_io::indexed_reader::ReadOrder::LogTime);
        let mut reader = IndexedReader::new_with_options(summary, options)?;
        let mut pending: Option<(u64, usize)> = None;
        loop {
            if let Some((offset, length)) = pending.take() {
                let start = offset as usize;
                let chunk = self
                    .mapped
                    .get(start..start + length)
                    .context("chunk past end of file")?;
                reader.insert_chunk_record_data(offset, chunk)?;
            }
            let Some(event) = reader.next_event() else {
                return Ok(());
            };
            match event? {
                IndexedReadEvent::ReadChunkRequest { offset, length } => {
                    pending = Some((offset, length))
                }
                IndexedReadEvent::Message { header, data } => {
                    let channel = summary
                        .channels
                        .get(&header.channel_id)
                        .context("message on a channel the summary does not list")?;
                    if let Some((index, ts)) = wanted(channel, &header) {
                        if !visit(index, ts, data.to_vec())? {
                            return Ok(());
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stream_names_match_the_python_mcap_store() {
        assert_eq!(stream_name("dimos/odom/geometry_msgs.PoseStamped"), "odom");
        assert_eq!(stream_name("dimos/a/b/nav_msgs.Path"), "a_b");
        assert_eq!(stream_name("rt/utlidar/cloud"), "utlidar_cloud");
        assert_eq!(stream_name("control_log"), "control_log");
    }

    #[test]
    fn storage_follows_the_codec() {
        assert_eq!(Storage::from_codec("jpeg"), Storage::Wire);
        assert_eq!(Storage::from_codec("lz4+lcm"), Storage::Lz4);
        assert!(Storage::Unsupported("pickle".into())
            .to_wire(vec![1])
            .is_err());
    }

    #[test]
    fn lz4_unwraps_to_the_wire_bytes() {
        use std::io::Write;
        let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
        encoder.write_all(b"lcm wire bytes").unwrap();
        let compressed = encoder.finish().unwrap();
        assert_eq!(Storage::Lz4.to_wire(compressed).unwrap(), b"lcm wire bytes");
    }

    #[test]
    fn quote_escapes_embedded_quotes() {
        assert_eq!(quote(r#"we"ird"#), r#""we""ird""#);
    }
}
