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

//! O(1)-snapshot stores for the off-scan-thread jobs: `Appended<T>` (append-only chunked arena read without locks),
//! `KeyFrames`/`Closures` (fixed data appended; the adopted solve's poses/weights swapped in as an `Arc`).

use crate::gsc_pgo::{CommittedClosure, KeyPoseWithCloud};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::PointCloud;
use std::sync::{Arc, OnceLock};

const CHUNK: usize = 1024;
const CHUNKS: usize = 1 << 15;

/// One arena chunk of `CHUNK` slots.
type Chunk<T> = Box<[OnceLock<T>]>;

/// A chunked arena, each slot set once; readers only read slots below a length they were handed.
struct Arena<T> {
    chunks: Box<[OnceLock<Chunk<T>>]>,
}

impl<T> Arena<T> {
    fn new() -> Arena<T> {
        Arena {
            chunks: (0..CHUNKS).map(|_| OnceLock::new()).collect(),
        }
    }
    fn set(&self, index: usize, value: T) {
        let chunk = self.chunks[index / CHUNK]
            .get_or_init(|| (0..CHUNK).map(|_| OnceLock::new()).collect());
        if chunk[index % CHUNK].set(value).is_err() {
            panic!("arena slot {index} set twice");
        }
    }
    fn get(&self, index: usize) -> &T {
        self.chunks[index / CHUNK]
            .get()
            .and_then(|chunk| chunk[index % CHUNK].get())
            .expect("arena slot read before it was set")
    }
}

/// An append-only sequence: the shared arena and how much of it this handle sees. Cloning it is a snapshot.
pub struct Appended<T> {
    arena: Arc<Arena<T>>,
    len: usize,
}

impl<T> Clone for Appended<T> {
    fn clone(&self) -> Self {
        Appended {
            arena: self.arena.clone(),
            len: self.len,
        }
    }
}

impl<T> Default for Appended<T> {
    fn default() -> Self {
        Appended {
            arena: Arc::new(Arena::new()),
            len: 0,
        }
    }
}

impl<T> Appended<T> {
    pub fn len(&self) -> usize {
        self.len
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub fn get(&self, index: usize) -> &T {
        assert!(index < self.len, "index {index} past the {} appended", self.len);
        self.arena.get(index)
    }
    pub fn last(&self) -> Option<&T> {
        self.len.checked_sub(1).map(|k| self.arena.get(k))
    }
    /// Append (only through the one handle that owns the sequence; snapshots never see past their length).
    pub(crate) fn push(&mut self, value: T) {
        self.arena.set(self.len, value);
        self.len += 1;
    }
}

impl<T> std::ops::Index<usize> for Appended<T> {
    type Output = T;
    fn index(&self, index: usize) -> &T {
        self.get(index)
    }
}

/// What a keyframe never changes after it is made.
pub struct KeyFixed {
    pub rotation_local: Mat3,
    pub translation_local: Vec3,
    pub time: f64,
    pub body_cloud: Option<Arc<PointCloud>>,
}

/// The newest adopted solve's global poses (the first `rotations.len()` keyframes), built on its thread.
#[derive(Default)]
pub struct SolvedPoses {
    pub rotations: Vec<Mat3>,
    pub positions: Vec<Vec3>,
}

/// The keyframes (see the module). Cloning is a snapshot.
#[derive(Clone)]
pub struct KeyFrames {
    fixed: Appended<KeyFixed>,
    solved: Arc<SolvedPoses>,
    ride_rotation: Mat3,
    ride_translation: Vec3,
}

impl Default for KeyFrames {
    fn default() -> Self {
        KeyFrames {
            fixed: Appended::default(),
            solved: Arc::new(SolvedPoses::default()),
            ride_rotation: mat3::identity(),
            ride_translation: [0.0; 3],
        }
    }
}

impl KeyFrames {
    pub fn len(&self) -> usize {
        self.fixed.len()
    }
    pub fn is_empty(&self) -> bool {
        self.fixed.is_empty()
    }
    pub fn fixed(&self, k: usize) -> &KeyFixed {
        self.fixed.get(k)
    }
    /// Keyframe `k`'s global pose: the newest solve's, or its odometry pose ridden on that solve's newest correction.
    pub fn global(&self, k: usize) -> (Mat3, Vec3) {
        if k < self.solved.rotations.len() {
            return (self.solved.rotations[k], self.solved.positions[k]);
        }
        let fixed = self.fixed.get(k);
        (
            mat3::mat_mul(&self.ride_rotation, &fixed.rotation_local),
            mat3::add(
                &mat3::mat_vec(&self.ride_rotation, &fixed.translation_local),
                &self.ride_translation,
            ),
        )
    }
    pub fn position(&self, k: usize) -> Vec3 {
        self.global(k).1
    }
    pub fn rotation(&self, k: usize) -> Mat3 {
        self.global(k).0
    }
    /// Keyframe `k` whole (its cloud shared).
    pub fn get(&self, k: usize) -> KeyPoseWithCloud {
        let fixed = self.fixed.get(k);
        let (rotation_global, translation_global) = self.global(k);
        KeyPoseWithCloud {
            rotation_local: fixed.rotation_local,
            translation_local: fixed.translation_local,
            rotation_global,
            translation_global,
            time: fixed.time,
            body_cloud: fixed.body_cloud.clone(),
        }
    }
    pub fn last(&self) -> Option<KeyPoseWithCloud> {
        self.len().checked_sub(1).map(|k| self.get(k))
    }
    pub fn iter(&self) -> impl Iterator<Item = KeyPoseWithCloud> + '_ {
        (0..self.len()).map(|k| self.get(k))
    }
    pub fn range(&self, range: std::ops::Range<usize>) -> impl Iterator<Item = KeyPoseWithCloud> + '_ {
        range.map(|k| self.get(k))
    }
    pub(crate) fn push(&mut self, fixed: KeyFixed) {
        self.fixed.push(fixed);
    }
    /// Adopt a solve's poses (its first keyframes) and the correction the rest ride on: an `Arc` swap.
    pub(crate) fn adopt(&mut self, solved: Arc<SolvedPoses>, ride_rotation: Mat3, ride_translation: Vec3) {
        self.solved = solved;
        self.ride_rotation = ride_rotation;
        self.ride_translation = ride_translation;
    }
}

/// What a committed closure never changes: its pair, relative pose and fitness, and the search's confidence and yank.
#[derive(Clone, Copy)]
pub struct ClosureFixed {
    pub source_id: usize,
    pub target_id: usize,
    pub rotation_offset: Mat3,
    pub translation_offset: Vec3,
    pub score: f64,
    pub fit_confidence: f64,
    pub yank_stdevs: f64,
}

/// The committed closures (see the module). Cloning is a snapshot.
#[derive(Clone, Default)]
pub struct Closures {
    fixed: Appended<ClosureFixed>,
    weights: Arc<Vec<f64>>,
}

impl Closures {
    pub fn len(&self) -> usize {
        self.fixed.len()
    }
    pub fn is_empty(&self) -> bool {
        self.fixed.is_empty()
    }
    pub fn fixed(&self, i: usize) -> &ClosureFixed {
        self.fixed.get(i)
    }
    /// Closure `i`'s weight from the newest adopted solve (NaN until a solve has judged it).
    pub fn weight(&self, i: usize) -> f64 {
        self.weights.get(i).copied().unwrap_or(f64::NAN)
    }
    /// How many closures the newest adopted solve judged (the first ones; the rest are unjudged).
    pub fn judged(&self) -> usize {
        self.weights.len()
    }
    pub fn get(&self, i: usize) -> CommittedClosure {
        let fixed = self.fixed.get(i);
        CommittedClosure {
            source_id: fixed.source_id,
            target_id: fixed.target_id,
            rotation_offset: fixed.rotation_offset,
            translation_offset: fixed.translation_offset,
            score: fixed.score,
            gnc_weight: self.weight(i),
        }
    }
    pub fn iter(&self) -> impl Iterator<Item = CommittedClosure> + '_ {
        (0..self.len()).map(|i| self.get(i))
    }
    pub(crate) fn push(&mut self, fixed: ClosureFixed) {
        self.fixed.push(fixed);
    }
    /// Adopt a solve's weights (the closures dispatched to it): an `Arc` swap.
    pub(crate) fn adopt(&mut self, weights: Arc<Vec<f64>>) {
        self.weights = weights;
    }
}
