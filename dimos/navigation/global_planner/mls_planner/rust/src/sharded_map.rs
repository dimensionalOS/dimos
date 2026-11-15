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

//! A hash map split into fixed shards by a seeded hash, so a map of the whole world grows (and rehashes) one shard
//! at a time: no single insert pays for rehashing every entry.

use std::hash::Hash;

use crate::fixed_hash::AHashMap;

const SHARDS: usize = 64;

pub struct ShardedMap<K, V> {
    hasher: ahash::RandomState,
    shards: Vec<AHashMap<K, V>>,
}

impl<K: Hash + Eq, V> Default for ShardedMap<K, V> {
    fn default() -> Self {
        Self {
            hasher: ahash::RandomState::with_seeds(5, 6, 7, 8),
            shards: (0..SHARDS).map(|_| AHashMap::default()).collect(),
        }
    }
}

impl<K: Hash + Eq, V> ShardedMap<K, V> {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    fn shard(&self, k: &K) -> usize {
        (self.hasher.hash_one(k) >> 58) as usize
    }

    #[inline]
    pub fn get(&self, k: &K) -> Option<&V> {
        self.shards[self.shard(k)].get(k)
    }

    pub fn get_mut(&mut self, k: &K) -> Option<&mut V> {
        let s = self.shard(k);
        self.shards[s].get_mut(k)
    }

    pub fn contains_key(&self, k: &K) -> bool {
        self.shards[self.shard(k)].contains_key(k)
    }

    pub fn insert(&mut self, k: K, v: V) -> Option<V> {
        let s = self.shard(&k);
        self.shards[s].insert(k, v)
    }

    pub fn remove(&mut self, k: &K) -> Option<V> {
        let s = self.shard(k);
        self.shards[s].remove(k)
    }

    pub fn len(&self) -> usize {
        self.shards.iter().map(|s| s.len()).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.shards.iter().all(|s| s.is_empty())
    }

    /// Entries the shards hold room for.
    pub fn capacity(&self) -> usize {
        self.shards.iter().map(|s| s.capacity()).sum()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&K, &V)> + '_ {
        self.shards.iter().flat_map(|s| s.iter())
    }
}

impl<K: Hash + Eq, V> FromIterator<(K, V)> for ShardedMap<K, V> {
    fn from_iter<I: IntoIterator<Item = (K, V)>>(iter: I) -> Self {
        let mut map = Self::new();
        for (k, v) in iter {
            map.insert(k, v);
        }
        map
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sharded_map_keeps_what_a_map_keeps() {
        let mut m: ShardedMap<(i32, i32, i32), u32> = ShardedMap::new();
        for i in 0..10_000 {
            assert_eq!(m.insert((i, -i, i % 7), i as u32), None);
        }
        assert_eq!(m.insert((3, -3, 3), 99), Some(3));
        assert_eq!(m.remove(&(4, -4, 4)), Some(4));
        assert_eq!(m.len(), 9_999);
        assert_eq!(m.get(&(3, -3, 3)), Some(&99));
        assert!(!m.contains_key(&(4, -4, 4)));
        assert_eq!(m.iter().count(), 9_999);
        assert!(m.capacity() >= m.len());
    }
}
