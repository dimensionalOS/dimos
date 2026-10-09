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

//! Voxel storage in fixed 8x8x8 chunks: an occupancy mask plus the chunk's
//! voxels packed in mask order.

use ahash::AHashMap;

use super::{Voxel, VoxelHealth, VoxelKey};

pub type ChunkKey = (i32, i32, i32);

const CHUNK_BITS: i32 = 3;
/// Voxels per chunk edge.
pub const CHUNK_EDGE: i32 = 1 << CHUNK_BITS;
const LOCAL_MASK: i32 = CHUNK_EDGE - 1;
const SLOTS: usize = 1 << (3 * CHUNK_BITS);
const WORD_BITS: usize = 64;
const WORDS: usize = SLOTS / WORD_BITS;

/// Chunks a neighborhood may overlap along one axis.
const NEAR_SPAN: usize = 2;
/// Largest neighborhood radius: a window of `2r + 1` voxels has to fit in
/// `NEAR_SPAN` chunks along each axis.
pub const NEAR_RADIUS_MAX: i32 = CHUNK_EDGE / 2;

#[inline]
pub fn chunk_of(key: VoxelKey) -> ChunkKey {
    (
        key.0 >> CHUNK_BITS,
        key.1 >> CHUNK_BITS,
        key.2 >> CHUNK_BITS,
    )
}

#[inline]
fn slot_of(key: VoxelKey) -> usize {
    (((key.0 & LOCAL_MASK) << (2 * CHUNK_BITS))
        | ((key.1 & LOCAL_MASK) << CHUNK_BITS)
        | (key.2 & LOCAL_MASK)) as usize
}

#[inline]
fn key_of(chunk: ChunkKey, slot: usize) -> VoxelKey {
    let s = slot as i32;
    (
        (chunk.0 << CHUNK_BITS) | (s >> (2 * CHUNK_BITS)),
        (chunk.1 << CHUNK_BITS) | ((s >> CHUNK_BITS) & LOCAL_MASK),
        (chunk.2 << CHUNK_BITS) | (s & LOCAL_MASK),
    )
}

/// Positions of the set bits of one word, ascending.
#[inline]
fn set_bits(mut bits: u64) -> impl Iterator<Item = usize> {
    std::iter::from_fn(move || {
        (bits != 0).then(|| {
            let b = bits.trailing_zeros() as usize;
            bits &= bits - 1;
            b
        })
    })
}

/// A bitset over a chunk's slots.
#[derive(Clone, Copy, Default)]
struct Mask([u64; WORDS]);

impl Mask {
    #[inline]
    fn get(&self, slot: usize) -> bool {
        (self.0[slot / WORD_BITS] >> (slot % WORD_BITS)) & 1 != 0
    }

    #[inline]
    fn set(&mut self, slot: usize, on: bool) {
        let bit = 1u64 << (slot % WORD_BITS);
        let word = &mut self.0[slot / WORD_BITS];
        if on {
            *word |= bit;
        } else {
            *word &= !bit;
        }
    }

    /// Set bits below `slot`.
    #[inline]
    fn rank(&self, slot: usize) -> usize {
        let word = slot / WORD_BITS;
        let below: u32 = self.0[..word].iter().map(|w| w.count_ones()).sum();
        let low = self.0[word] & ((1u64 << (slot % WORD_BITS)) - 1);
        (below + low.count_ones()) as usize
    }

    fn count(&self) -> usize {
        self.0.iter().map(|w| w.count_ones() as usize).sum()
    }

    /// Set slots, ascending.
    fn ones(&self) -> impl Iterator<Item = usize> + '_ {
        self.0
            .iter()
            .enumerate()
            .flat_map(|(w, &bits)| set_bits(bits).map(move |b| w * WORD_BITS + b))
    }

    /// Set slots, ascending, each with its rank in `within`, a mask this one
    /// is a subset of.
    fn ones_ranked_in<'a>(&'a self, within: &'a Mask) -> impl Iterator<Item = (usize, usize)> + 'a {
        let mut below = 0;
        self.0
            .iter()
            .zip(&within.0)
            .enumerate()
            .flat_map(move |(w, (&bits, &outer))| {
                let base = below;
                below += outer.count_ones() as usize;
                set_bits(bits).map(move |b| {
                    let rank = base + (outer & ((1u64 << b) - 1)).count_ones() as usize;
                    (w * WORD_BITS + b, rank)
                })
            })
    }
}

#[derive(Clone, Default)]
struct Chunk {
    occupied: Mask,
    /// Occupied slots whose voxel is healthy.
    healthy: Mask,
    /// Healthy-neighbor count per voxel, packed like `voxels`.
    support: Vec<u8>,
    voxels: Vec<Voxel>,
}

impl Chunk {
    /// Packed index of the voxel at `slot`, if occupied.
    #[inline]
    fn index(&self, slot: usize) -> Option<usize> {
        self.occupied.get(slot).then(|| self.occupied.rank(slot))
    }

    #[inline]
    fn get(&self, key: VoxelKey) -> Option<&Voxel> {
        self.index(slot_of(key)).map(|i| &self.voxels[i])
    }

    #[inline]
    fn is_healthy(&self, key: VoxelKey) -> bool {
        self.healthy.get(slot_of(key))
    }
}

/// A chunk with its key, for scans over its voxels.
#[derive(Clone, Copy)]
pub struct ChunkRef<'a> {
    pub key: ChunkKey,
    chunk: &'a Chunk,
}

/// One healthy voxel from a chunk scan.
#[derive(Clone, Copy)]
pub struct HealthyVoxel<'a> {
    pub key: VoxelKey,
    pub support: u8,
    pub voxel: &'a Voxel,
}

impl<'a> ChunkRef<'a> {
    pub fn healthy_len(&self) -> usize {
        self.chunk.healthy.count()
    }

    /// Voxels with their keys, in slot order.
    pub fn iter(&self) -> impl Iterator<Item = (VoxelKey, &'a Voxel)> + 'a {
        let (key, chunk) = (self.key, self.chunk);
        chunk
            .occupied
            .ones()
            .zip(&chunk.voxels)
            .map(move |(slot, v)| (key_of(key, slot), v))
    }

    /// Healthy voxels in slot order.
    pub fn healthy(&self) -> impl Iterator<Item = HealthyVoxel<'a>> + 'a {
        let (key, chunk) = (self.key, self.chunk);
        chunk
            .healthy
            .ones_ranked_in(&chunk.occupied)
            .map(move |(slot, i)| HealthyVoxel {
                key: key_of(key, slot),
                support: chunk.support[i],
                voxel: &chunk.voxels[i],
            })
    }
}

/// Voxels keyed by grid position, stored chunk by chunk. Health is written
/// through `update_health` so the healthy mask always matches it.
#[derive(Clone, Default)]
pub struct ChunkMap {
    chunks: AHashMap<ChunkKey, Chunk>,
    len: usize,
}

impl ChunkMap {
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn clear(&mut self) {
        self.chunks.clear();
        self.len = 0;
    }

    pub fn reserve_chunks(&mut self, additional: usize) {
        self.chunks.reserve(additional);
    }

    #[inline]
    pub fn get(&self, key: &VoxelKey) -> Option<&Voxel> {
        self.chunks.get(&chunk_of(*key))?.get(*key)
    }

    /// Mutable access to a voxel's moments, fine cells and normal. Its health
    /// changes through `update_health`.
    #[inline]
    pub fn get_mut(&mut self, key: &VoxelKey) -> Option<&mut Voxel> {
        let chunk = self.chunks.get_mut(&chunk_of(*key))?;
        let i = chunk.index(slot_of(*key))?;
        Some(&mut chunk.voxels[i])
    }

    #[inline]
    pub fn contains_key(&self, key: &VoxelKey) -> bool {
        self.chunks
            .get(&chunk_of(*key))
            .is_some_and(|c| c.occupied.get(slot_of(*key)))
    }

    #[inline]
    pub fn is_healthy(&self, key: &VoxelKey) -> bool {
        self.chunks
            .get(&chunk_of(*key))
            .is_some_and(|c| c.is_healthy(*key))
    }

    /// Store `voxel` at `key` with its healthy-neighbor count, returning the
    /// voxel it replaced.
    pub fn insert(&mut self, key: VoxelKey, voxel: Voxel, support: u8) -> Option<Voxel> {
        let chunk = self.chunks.entry(chunk_of(key)).or_default();
        let slot = slot_of(key);
        let i = chunk.occupied.rank(slot);
        chunk.healthy.set(slot, voxel.health > 0);
        if chunk.occupied.get(slot) {
            chunk.support[i] = support;
            return Some(std::mem::replace(&mut chunk.voxels[i], voxel));
        }
        chunk.occupied.set(slot, true);
        chunk.voxels.insert(i, voxel);
        chunk.support.insert(i, support);
        self.len += 1;
        None
    }

    /// Rewrite the health of the voxel at `key` through `f`, keeping the
    /// healthy mask in step. Returns the old and new health.
    pub fn update_health(
        &mut self,
        key: VoxelKey,
        f: impl FnOnce(VoxelHealth) -> VoxelHealth,
    ) -> Option<(VoxelHealth, VoxelHealth)> {
        let chunk = self.chunks.get_mut(&chunk_of(key))?;
        let slot = slot_of(key);
        let i = chunk.index(slot)?;
        let voxel = &mut chunk.voxels[i];
        let was = voxel.health;
        voxel.health = f(was);
        chunk.healthy.set(slot, voxel.health > 0);
        Some((was, voxel.health))
    }

    /// Healthy-neighbor count of the voxel at `key`.
    #[cfg(test)]
    pub fn support(&self, key: &VoxelKey) -> Option<u8> {
        let chunk = self.chunks.get(&chunk_of(*key))?;
        chunk.index(slot_of(*key)).map(|i| chunk.support[i])
    }

    pub fn healthy_len(&self) -> usize {
        self.chunks.values().map(|c| c.healthy.count()).sum()
    }

    /// Remove the voxel at `key`, dropping its chunk once empty.
    pub fn remove(&mut self, key: &VoxelKey) -> Option<Voxel> {
        let ck = chunk_of(*key);
        let chunk = self.chunks.get_mut(&ck)?;
        let slot = slot_of(*key);
        let i = chunk.index(slot)?;
        chunk.occupied.set(slot, false);
        chunk.healthy.set(slot, false);
        chunk.support.remove(i);
        let voxel = chunk.voxels.remove(i);
        if chunk.voxels.is_empty() {
            self.chunks.remove(&ck);
        }
        self.len -= 1;
        Some(voxel)
    }

    /// Read access to the voxels within `r` of `center`, resolving each
    /// overlapped chunk once. `r` must not exceed `NEAR_RADIUS_MAX`.
    pub fn neighborhood(&self, center: VoxelKey, r: i32) -> Neighborhood<'_> {
        Neighborhood::new(self, center, r)
    }

    /// Calls `f` with each voxel within `r` of `center` and its support count,
    /// resolving each overlapped chunk once.
    pub fn for_each_near_mut(
        &mut self,
        center: VoxelKey,
        r: i32,
        mut f: impl FnMut(VoxelKey, &mut Voxel, &mut u8),
    ) {
        let lo = (center.0 - r, center.1 - r, center.2 - r);
        let hi = (center.0 + r, center.1 + r, center.2 + r);
        let (clo, chi) = (chunk_of(lo), chunk_of(hi));
        for cx in clo.0..=chi.0 {
            for cy in clo.1..=chi.1 {
                for cz in clo.2..=chi.2 {
                    let Some(chunk) = self.chunks.get_mut(&(cx, cy, cz)) else {
                        continue;
                    };
                    let span = |c: i32, lo: i32, hi: i32| {
                        lo.max(c << CHUNK_BITS)..=hi.min((c << CHUNK_BITS) + LOCAL_MASK)
                    };
                    for x in span(cx, lo.0, hi.0) {
                        for y in span(cy, lo.1, hi.1) {
                            for z in span(cz, lo.2, hi.2) {
                                if let Some(i) = chunk.index(slot_of((x, y, z))) {
                                    f((x, y, z), &mut chunk.voxels[i], &mut chunk.support[i]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn chunk(&self, key: &ChunkKey) -> Option<ChunkRef<'_>> {
        self.chunks
            .get(key)
            .map(|chunk| ChunkRef { key: *key, chunk })
    }

    pub fn chunks(&self) -> impl Iterator<Item = ChunkRef<'_>> {
        self.chunks
            .iter()
            .map(|(&key, chunk)| ChunkRef { key, chunk })
    }

    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    pub fn iter(&self) -> impl Iterator<Item = (VoxelKey, &Voxel)> {
        self.chunks().flat_map(|c| c.iter())
    }

    #[cfg(test)]
    pub fn keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.iter().map(|(k, _)| k)
    }

    #[cfg(test)]
    pub fn values(&self) -> impl Iterator<Item = &Voxel> {
        self.chunks.values().flat_map(|c| c.voxels.iter())
    }
}

/// Where the chunk `ck` sits in a neighborhood whose lowest chunk is `lo`.
#[inline]
fn near_index(lo: ChunkKey, ck: ChunkKey) -> usize {
    (((ck.0 - lo.0) << 2) | ((ck.1 - lo.1) << 1) | (ck.2 - lo.2)) as usize
}

/// The chunks around one voxel, for repeated lookups near it.
pub struct Neighborhood<'a> {
    lo: ChunkKey,
    chunks: [Option<&'a Chunk>; NEAR_SPAN * NEAR_SPAN * NEAR_SPAN],
}

impl<'a> Neighborhood<'a> {
    fn new(map: &'a ChunkMap, center: VoxelKey, r: i32) -> Self {
        assert!(
            (0..=NEAR_RADIUS_MAX).contains(&r),
            "neighborhood radius {r} outside 0..={NEAR_RADIUS_MAX}"
        );
        let lo = chunk_of((center.0 - r, center.1 - r, center.2 - r));
        let hi = chunk_of((center.0 + r, center.1 + r, center.2 + r));
        let mut chunks = [None; NEAR_SPAN * NEAR_SPAN * NEAR_SPAN];
        for cx in lo.0..=hi.0 {
            for cy in lo.1..=hi.1 {
                for cz in lo.2..=hi.2 {
                    chunks[near_index(lo, (cx, cy, cz))] = map.chunks.get(&(cx, cy, cz));
                }
            }
        }
        Self { lo, chunks }
    }

    #[inline]
    fn chunk(&self, key: VoxelKey) -> Option<&'a Chunk> {
        self.chunks[near_index(self.lo, chunk_of(key))]
    }

    #[inline]
    pub fn get(&self, key: VoxelKey) -> Option<&'a Voxel> {
        self.chunk(key)?.get(key)
    }

    #[inline]
    pub fn is_healthy(&self, key: VoxelKey) -> bool {
        self.chunk(key).is_some_and(|c| c.is_healthy(key))
    }
}

#[cfg(test)]
impl std::ops::Index<&VoxelKey> for ChunkMap {
    type Output = Voxel;

    fn index(&self, key: &VoxelKey) -> &Voxel {
        self.get(key).expect("no voxel at key")
    }
}

/// Voxel lookups along a path, resolving the chunk only when the path enters a
/// new one.
pub struct ChunkCursor<'a> {
    map: &'a ChunkMap,
    key: Option<ChunkKey>,
    chunk: Option<&'a Chunk>,
}

impl<'a> ChunkCursor<'a> {
    pub fn new(map: &'a ChunkMap) -> Self {
        Self {
            map,
            key: None,
            chunk: None,
        }
    }

    #[inline]
    pub fn get(&mut self, key: VoxelKey) -> Option<&'a Voxel> {
        let ck = chunk_of(key);
        if self.key != Some(ck) {
            self.key = Some(ck);
            self.chunk = self.map.chunks.get(&ck);
        }
        self.chunk?.get(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slot_and_key_round_trip_across_negative_chunks() {
        for key in [
            (0, 0, 0),
            (-1, -1, -1),
            (7, -8, 9),
            (-9, 15, -16),
            (123, -456, 789),
        ] {
            assert_eq!(key_of(chunk_of(key), slot_of(key)), key);
        }
        assert_eq!(chunk_of((-1, 0, 7)), (-1, 0, 0));
        assert_eq!(chunk_of((-8, -9, 8)), (-1, -2, 1));
    }

    #[test]
    fn packed_order_survives_inserts_and_removes() {
        let mut map = ChunkMap::default();
        let keys = [
            (3, 1, 2),
            (0, 0, 0),
            (7, 7, 7),
            (-1, 2, 3),
            (1, 0, 0),
            (0, 0, 1),
        ];
        for (i, &k) in keys.iter().enumerate() {
            assert!(map.insert(k, Voxel::with_health(i as i32), 0).is_none());
        }
        assert_eq!(map.len(), keys.len());
        assert_eq!(map.healthy_len(), 5);
        for (i, k) in keys.iter().enumerate() {
            assert_eq!(map[k].health, i as i32);
        }

        assert_eq!(map.remove(&(0, 0, 0)).map(|v| v.health), Some(1));
        assert!(map.remove(&(0, 0, 0)).is_none());
        assert_eq!(map.update_health((7, 7, 7), |_| 70), Some((2, 70)));
        assert_eq!(
            map.insert((1, 0, 0), Voxel::with_health(40), 0)
                .map(|v| v.health),
            Some(4)
        );
        assert_eq!(
            map.insert((0, 0, 1), Voxel::with_health(-5), 0)
                .map(|v| v.health),
            Some(5)
        );
        assert_eq!(map.healthy_len(), 3, "replace path updates the mask");
        assert!(map.insert((2, 2, 2), Voxel::with_health(22), 0).is_none());

        let mut got: Vec<(VoxelKey, i32)> = map.iter().map(|(k, v)| (k, v.health)).collect();
        got.sort();
        assert_eq!(
            got,
            vec![
                ((-1, 2, 3), 3),
                ((0, 0, 1), -5),
                ((1, 0, 0), 40),
                ((2, 2, 2), 22),
                ((3, 1, 2), 0),
                ((7, 7, 7), 70),
            ]
        );
        assert_eq!(map.len(), 6);
    }

    fn random_map() -> ChunkMap {
        let mut map = ChunkMap::default();
        let mut n = 0;
        for x in -10..10 {
            for y in -10..10 {
                for z in -10..10 {
                    if (x * 7 + y * 3 + z * 5) % 4 != 0 {
                        n += 1;
                        map.insert((x, y, z), Voxel::with_health(n % 3 - 1), 0);
                    }
                }
            }
        }
        map
    }

    #[test]
    fn neighborhood_access_matches_map_lookups() {
        let map = random_map();
        for center in [(0, 0, 0), (-1, 7, 8), (-8, -9, 7), (3, 3, 3), (-10, 9, 0)] {
            let near = map.neighborhood(center, 1);
            let mut expected = Vec::new();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let k = (center.0 + dx, center.1 + dy, center.2 + dz);
                        let want = map.get(&k).map(|v| v.health);
                        assert_eq!(near.get(k).map(|v| v.health), want);
                        assert_eq!(near.is_healthy(k), want.is_some_and(|h| h > 0));
                        if let Some(h) = want {
                            expected.push((k, h));
                        }
                    }
                }
            }
            let mut seen = Vec::new();
            let mut map = map.clone();
            map.for_each_near_mut(center, 1, |k, v, _| seen.push((k, v.health)));
            seen.sort();
            expected.sort();
            assert_eq!(seen, expected);
        }
    }

    /// The largest allowed radius still fits every overlapped chunk in the
    /// neighborhood's table, whichever way the window straddles chunks.
    #[test]
    fn neighborhood_at_max_radius_matches_map_lookups() {
        let map = random_map();
        let r = NEAR_RADIUS_MAX;
        for center in [(0, 0, 0), (3, 3, 3), (4, 4, 4), (-4, 7, -5), (-9, 9, 1)] {
            let near = map.neighborhood(center, r);
            for dx in -r..=r {
                for dy in -r..=r {
                    for dz in -r..=r {
                        let k = (center.0 + dx, center.1 + dy, center.2 + dz);
                        let want = map.get(&k).map(|v| v.health);
                        assert_eq!(near.get(k).map(|v| v.health), want, "{center:?} {k:?}");
                        assert_eq!(near.is_healthy(k), want.is_some_and(|h| h > 0));
                    }
                }
            }
        }
    }

    #[test]
    #[should_panic(expected = "neighborhood radius")]
    fn neighborhood_rejects_radius_past_max() {
        ChunkMap::default().neighborhood((0, 0, 0), NEAR_RADIUS_MAX + 1);
    }

    #[test]
    fn healthy_scan_follows_health_and_support() {
        let mut map = ChunkMap::default();
        for (i, key) in [(0, 0, 0), (1, 0, 0), (7, 7, 7), (3, 4, 5)]
            .into_iter()
            .enumerate()
        {
            map.insert(key, Voxel::with_health(i as i32 - 1), 10 + i as u8);
        }
        let scan = |map: &ChunkMap| {
            let mut got: Vec<(VoxelKey, u8)> = map
                .chunks()
                .flat_map(|c| c.healthy().map(|hv| (hv.key, hv.support)))
                .collect();
            got.sort();
            got
        };
        assert_eq!(scan(&map), vec![((3, 4, 5), 13), ((7, 7, 7), 12)]);
        assert_eq!(map.healthy_len(), 2);
        assert!(map.is_healthy(&(7, 7, 7)));
        assert!(!map.is_healthy(&(0, 0, 0)));
        assert!(!map.is_healthy(&(9, 9, 9)));

        assert_eq!(map.update_health((0, 0, 0), |h| h + 2), Some((-1, 1)));
        assert_eq!(map.update_health((9, 9, 9), |h| h + 2), None);
        map.remove(&(7, 7, 7));
        assert_eq!(scan(&map), vec![((0, 0, 0), 10), ((3, 4, 5), 13)]);
        assert!(map.is_healthy(&(0, 0, 0)));
        assert_eq!(map.support(&(1, 0, 0)), Some(11));
        assert_eq!(map.support(&(7, 7, 7)), None);
    }

    #[test]
    fn emptied_chunk_is_dropped() {
        let mut map = ChunkMap::default();
        map.insert((-3, 4, 5), Voxel::default(), 0);
        assert_eq!(map.chunk_count(), 1);
        map.remove(&(-3, 4, 5));
        assert_eq!(map.chunk_count(), 0);
        assert!(map.is_empty());
    }

    #[test]
    fn cursor_matches_map_lookups() {
        let mut map = ChunkMap::default();
        for x in -20..20 {
            if x % 3 != 0 {
                map.insert((x, 1, -1), Voxel::with_health(x), 0);
            }
        }
        let mut cursor = ChunkCursor::new(&map);
        for x in -20..20 {
            let key = (x, 1, -1);
            assert_eq!(
                cursor.get(key).map(|v| v.health),
                map.get(&key).map(|v| v.health)
            );
        }
    }
}
