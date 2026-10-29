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

//! A cache of every cloud computation a closure search makes (ICP, its scores, degeneracy, the fit's covariance, each
//! extended-ICP round), keyed by source keyframe, target keyframe and what was computed, so a replay can rerun the
//! whole downstream pipeline (gates, weighting, GNC, the noise learner, the solve) without the clouds. Recording stores
//! each result as computed; replaying returns it, and computes (and counts) any the recording never made.

use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

use crate::connection_fit::IcpResult;

/// Whether the cache stores what is computed or answers from what was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Record,
    Replay,
}

pub struct IcpCache {
    pub mode: Mode,
    entries: Mutex<HashMap<String, Vec<f64>>>,
    pub hits: AtomicUsize,
    pub misses: AtomicUsize,
}

impl IcpCache {
    pub fn new(mode: Mode, entries: HashMap<String, Vec<f64>>) -> IcpCache {
        IcpCache {
            mode,
            entries: Mutex::new(entries),
            hits: AtomicUsize::new(0),
            misses: AtomicUsize::new(0),
        }
    }

    /// The cached value for `key`, else `compute()` (stored when recording, counted as a miss when replaying).
    pub fn get_or(&self, key: String, compute: impl FnOnce() -> Vec<f64>) -> Vec<f64> {
        if self.mode == Mode::Replay {
            if let Some(value) = self.entries.lock().unwrap().get(&key) {
                self.hits.fetch_add(1, Ordering::Relaxed);
                return value.clone();
            }
            self.misses.fetch_add(1, Ordering::Relaxed);
        }
        let value = compute();
        if self.mode == Mode::Record {
            self.entries.lock().unwrap().insert(key, value.clone());
        }
        value
    }

    /// Every entry, for writing out.
    pub fn entries(&self) -> HashMap<String, Vec<f64>> {
        self.entries.lock().unwrap().clone()
    }
}

/// An ICP result as cached: converged, rotation (row-major), translation, fitness, overlap.
pub fn encode_icp(icp: &IcpResult) -> Vec<f64> {
    let mut v = vec![icp.converged as u8 as f64];
    v.extend(icp.rotation.iter().flatten());
    v.extend(icp.translation);
    v.push(icp.fitness);
    v.push(icp.overlap);
    v
}

pub fn decode_icp(v: &[f64]) -> IcpResult {
    IcpResult {
        converged: v[0] != 0.0,
        rotation: std::array::from_fn(|i| std::array::from_fn(|j| v[1 + 3 * i + j])),
        translation: [v[10], v[11], v[12]],
        fitness: v[13],
        overlap: v[14],
    }
}

/// The cache as bytes, exact for every f64 (NaN and infinities included, which JSON cannot hold): per entry, its key's
/// length (u32) and bytes, then its value's length (u32) and f64s, all little-endian.
pub fn to_bytes(entries: &HashMap<String, Vec<f64>>) -> Vec<u8> {
    let mut keys: Vec<&String> = entries.keys().collect();
    keys.sort();
    let mut out = Vec::new();
    for key in keys {
        out.extend((key.len() as u32).to_le_bytes());
        out.extend(key.as_bytes());
        let value = &entries[key];
        out.extend((value.len() as u32).to_le_bytes());
        for v in value {
            out.extend(v.to_le_bytes());
        }
    }
    out
}

pub fn from_bytes(bytes: &[u8]) -> Result<HashMap<String, Vec<f64>>, String> {
    let mut entries = HashMap::new();
    let mut at = 0;
    let take = |at: &mut usize, n: usize| -> Result<&[u8], String> {
        let slice = bytes
            .get(*at..*at + n)
            .ok_or_else(|| format!("icp cache truncated at byte {at}"))?;
        *at += n;
        Ok(slice)
    };
    let u32_at = |slice: &[u8]| u32::from_le_bytes(slice.try_into().unwrap()) as usize;
    while at < bytes.len() {
        let key_len = u32_at(take(&mut at, 4)?);
        let key = String::from_utf8(take(&mut at, key_len)?.to_vec()).map_err(|e| e.to_string())?;
        let count = u32_at(take(&mut at, 4)?);
        let value = take(&mut at, 8 * count)?
            .chunks_exact(8)
            .map(|c| f64::from_le_bytes(c.try_into().unwrap()))
            .collect();
        entries.insert(key, value);
    }
    Ok(entries)
}
