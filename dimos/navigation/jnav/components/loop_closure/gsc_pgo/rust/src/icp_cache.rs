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
use crate::mat3::{self, Mat3, Vec3};

/// An encoded alignment's correction (`encode_icp`'s rotation and translation, world axes, moving the source cloud) turned
/// into the source keyframe's pose relative to the target keyframe's (`forward`), or back. The cache stores the
/// relative pose: the correction alone depends on where the graph had put the two keyframes when it was computed, so a
/// replay under any other noise or verdict (a differently corrected graph) read it back tens of metres off (huge).
pub fn relative_icp(
    encoded: &mut [f64],
    source: (&Mat3, &Vec3),
    target: (&Mat3, &Vec3),
    forward: bool,
) {
    let rotation: Mat3 = std::array::from_fn(|i| std::array::from_fn(|j| encoded[1 + 3 * i + j]));
    let translation: Vec3 = [encoded[10], encoded[11], encoded[12]];
    let ((rs, ps), (rt, pt)) = (source, target);
    let back = mat3::transpose(rt);
    let (rotation, translation) = if forward {
        // R_rel = Rt^T R Rs, t_rel = Rt^T (R ps + t - pt)
        (
            mat3::mat_mul(&back, &mat3::mat_mul(&rotation, rs)),
            mat3::mat_vec(
                &back,
                &mat3::sub(&mat3::add(&mat3::mat_vec(&rotation, ps), &translation), pt),
            ),
        )
    } else {
        // R = Rt R_rel Rs^T, t = pt + Rt t_rel - R ps
        let r = mat3::mat_mul(&mat3::mat_mul(rt, &rotation), &mat3::transpose(rs));
        let t = mat3::sub(
            &mat3::add(pt, &mat3::mat_vec(rt, &translation)),
            &mat3::mat_vec(&r, ps),
        );
        (r, t)
    };
    for i in 0..3 {
        for j in 0..3 {
            encoded[1 + 3 * i + j] = rotation[i][j];
        }
        encoded[10 + i] = translation[i];
    }
}

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

/// `encode_icp` with the iterations and stop reason after it (`decode_icp` reads either).
pub fn encode_icp_logged(icp: &IcpResult) -> Vec<f64> {
    let mut v = encode_icp(icp);
    v.push(icp.iterations as f64);
    v.push(icp.stop.code() as f64);
    v
}

pub fn decode_icp(v: &[f64]) -> IcpResult {
    IcpResult {
        converged: v[0] != 0.0,
        rotation: std::array::from_fn(|i| std::array::from_fn(|j| v[1 + 3 * i + j])),
        translation: [v[10], v[11], v[12]],
        fitness: v[13],
        overlap: v[14],
        // entries cached before these were logged read as 0 iterations, stop by `converged`
        iterations: v.get(15).map_or(0, |&n| n as u32),
        stop: v.get(16).map_or(
            if v[0] != 0.0 {
                crate::connection_fit::IcpStop::Converged
            } else {
                crate::connection_fit::IcpStop::Diverged
            },
            |&c| crate::connection_fit::IcpStop::from_code(c as u8),
        ),
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

#[cfg(test)]
mod relative_icp_tests {
    use super::relative_icp;
    use crate::mat3::{self, Mat3, Vec3};

    fn rot(axis: Vec3, angle: f64) -> Mat3 {
        let n = mat3::norm(&axis);
        let k = mat3::skew(&mat3::scale(&axis, 1.0 / n));
        let k2 = mat3::mat_mul(&k, &k);
        std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                (i == j) as u8 as f64 + angle.sin() * k[i][j] + (1.0 - angle.cos()) * k2[i][j]
            })
        })
    }

    /// The same alignment read back under a differently corrected graph (both keyframes moved by one rigid motion, as a
    /// solve moves them) gives the same source pose relative to the target, where replaying the raw correction would not.
    #[test]
    fn a_cached_alignment_keeps_its_relative_pose_under_a_moved_graph() {
        let (rs, ps) = (rot([0.2, 0.1, 1.0], 0.7), [12.0, -3.0, 1.5]);
        let (rt, pt) = (rot([0.0, 0.3, 1.0], -1.2), [2.0, 4.0, 0.2]);
        let (rc, tc) = (rot([1.0, -0.4, 0.2], 0.05), [0.8, -0.3, 0.1]);
        let mut encoded = vec![1.0];
        encoded.extend(rc.iter().flatten());
        encoded.extend(tc);
        encoded.extend([0.02, 0.9]);
        let mut stored = encoded.clone();
        relative_icp(&mut stored, (&rs, &ps), (&rt, &pt), true);
        // back at the same graph: the correction itself
        let mut same = stored.clone();
        relative_icp(&mut same, (&rs, &ps), (&rt, &pt), false);
        for (a, b) in same.iter().zip(&encoded) {
            assert!((a - b).abs() < 1e-12, "{same:?} vs {encoded:?}");
        }
        // under a graph moved by g (rotation rg, then translation pg): the source moved relative to the target the same
        let (rg, pg) = (rot([0.1, 1.0, 0.3], 0.9), [40.0, -25.0, 7.0]);
        let moved = |r: &Mat3, p: &Vec3| {
            (
                mat3::mat_mul(&rg, r),
                mat3::add(&mat3::mat_vec(&rg, p), &pg),
            )
        };
        let ((rs2, ps2), (rt2, pt2)) = (moved(&rs, &ps), moved(&rt, &pt));
        let mut replayed = stored.clone();
        relative_icp(&mut replayed, (&rs2, &ps2), (&rt2, &pt2), false);
        let relative = |r: &Mat3, t: &Vec3, rs: &Mat3, ps: &Vec3, rt: &Mat3, pt: &Vec3| {
            let back = mat3::transpose(rt);
            (
                mat3::mat_mul(&back, &mat3::mat_mul(r, rs)),
                mat3::mat_vec(&back, &mat3::sub(&mat3::add(&mat3::mat_vec(r, ps), t), pt)),
            )
        };
        let r2: Mat3 = std::array::from_fn(|i| std::array::from_fn(|j| replayed[1 + 3 * i + j]));
        let t2 = [replayed[10], replayed[11], replayed[12]];
        let (want_r, want_t) = relative(&rc, &tc, &rs, &ps, &rt, &pt);
        let (got_r, got_t) = relative(&r2, &t2, &rs2, &ps2, &rt2, &pt2);
        assert!(
            mat3::norm(&mat3::sub(&want_t, &got_t)) < 1e-9,
            "{want_t:?} vs {got_t:?}"
        );
        assert!(mat3::angular_distance(&want_r, &got_r) < 1e-9);
        assert_eq!(replayed[13..], encoded[13..]);
    }
}
