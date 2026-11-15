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

//! Hash maps and sets with fixed hasher keys, so the same inserts iterate in the same order every run.

use std::hash::BuildHasher;

/// Builds aHash hashers from constant keys (ahash's own RandomState and AHasher::default draw per-process random keys).
#[derive(Clone, Copy, Default, Debug)]
pub struct FixedState;

const KEYS: ahash::RandomState = ahash::RandomState::with_seeds(
    0x243f_6a88_85a3_08d3,
    0x1319_8a2e_0370_7344,
    0xa409_3822_299f_31d0,
    0x082e_fa98_ec4e_6c89,
);

impl BuildHasher for FixedState {
    type Hasher = ahash::AHasher;

    #[inline]
    fn build_hasher(&self) -> ahash::AHasher {
        KEYS.build_hasher()
    }
}

pub type AHashMap<K, V> = std::collections::HashMap<K, V, FixedState>;
pub type AHashSet<K> = std::collections::HashSet<K, FixedState>;
