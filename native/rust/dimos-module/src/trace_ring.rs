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

//! Compressed trace chunks. Called only by the diagnostics worker, never control callbacks.
use std::{
    collections::VecDeque,
    io::{self, Write},
    time::{Duration, Instant},
};
struct Chunk {
    first: Instant,
    last: Instant,
    bytes: Vec<u8>,
    raw: usize,
}
pub(crate) struct TraceRing {
    chunks: VecDeque<Chunk>,
    pending: Vec<u8>,
    first: Instant,
    last: Instant,
    compressed: usize,
    pub(crate) raw_bytes: usize,
    limit: usize,
    pub(crate) evictions: u64,
    pub(crate) capacity_lost_until: Option<Instant>,
}
impl TraceRing {
    pub(crate) fn new(limit: usize) -> Self {
        Self {
            chunks: VecDeque::new(),
            pending: Vec::new(),
            first: Instant::now(),
            last: Instant::now(),
            compressed: 0,
            raw_bytes: 0,
            limit,
            evictions: 0,
            capacity_lost_until: None,
        }
    }
    pub(crate) fn size(&self) -> usize {
        self.compressed
            + self.pending.capacity()
            + self.chunks.capacity() * std::mem::size_of::<Chunk>()
    }
    fn flush(&mut self) -> io::Result<()> {
        if !self.pending.is_empty() {
            let mut bytes = zstd::bulk::compress(&self.pending, 1)?;
            bytes.shrink_to_fit();
            self.compressed += bytes.capacity();
            self.chunks.push_back(Chunk {
                first: self.first,
                last: self.last,
                bytes,
                raw: self.pending.len(),
            });
            self.pending.clear();
        }
        Ok(())
    }
    pub(crate) fn push(&mut self, row: &[u8], at: Instant) -> io::Result<()> {
        if row.len() > 65536 {
            return Err(io::Error::other("trace row exceeds bound"));
        }
        if !self.pending.is_empty()
            && (self.pending.len() + row.len() > 65536
                || at.saturating_duration_since(self.first) >= Duration::from_millis(100))
        {
            self.flush()?;
        }
        if self.pending.is_empty() {
            self.first = at;
        }
        self.last = at;
        self.pending.extend_from_slice(row);
        self.raw_bytes += row.len();
        Ok(())
    }
    pub(crate) fn trim(&mut self, cutoff: Instant) -> io::Result<()> {
        if self.size() > self.limit || self.last < cutoff {
            self.flush()?;
        }
        while !self.chunks.is_empty()
            && (self.size() > self.limit || self.chunks.front().is_some_and(|c| c.last < cutoff))
        {
            let capacity = self.size() > self.limit;
            let chunk = self.chunks.pop_front().unwrap();
            self.compressed -= chunk.bytes.capacity();
            self.raw_bytes -= chunk.raw;
            self.evictions += 1;
            if capacity {
                self.capacity_lost_until = Some(chunk.last);
            }
        }
        Ok(())
    }
    pub(crate) fn write_to(&self, output: &mut impl Write) -> io::Result<()> {
        for chunk in &self.chunks {
            output.write_all(&zstd::bulk::decompress(&chunk.bytes, 65536)?)?;
        }
        output.write_all(&self.pending)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn retains_ninety_seconds_of_full_rate_metadata_and_roundtrips() {
        let mut ring = TraceRing::new(4 * 1024 * 1024);
        let start = Instant::now();
        let mut expected = Vec::new();
        // 1,500 changing boundary events/sec; repeated schema/context metadata, no image payloads.
        for n in 0..135_000u64 {
            let row=format!("{{\"event\":\"gateway.command_received\",\"mono_ns\":{},\"context\":{{\"run_id\":\"qualification\",\"publisher\":\"operator\",\"stream\":\"pose\",\"sequence\":{},\"clock_domain\":\"operator/clock\",\"parents\":[],\"lineage\":\"complete\"}}}}\n",n*666667,n);
            ring.push(row.as_bytes(), start + Duration::from_nanos(n * 666667))
                .unwrap();
            ring.trim(start).unwrap();
            expected.extend_from_slice(row.as_bytes());
        }
        assert!(ring.size() <= 4 * 1024 * 1024);
        assert!(ring.capacity_lost_until.is_none());
        let mut actual = Vec::new();
        ring.write_to(&mut actual).unwrap();
        assert_eq!(actual, expected);
        assert!(ring.chunks.front().unwrap().first == start);
    }
}
