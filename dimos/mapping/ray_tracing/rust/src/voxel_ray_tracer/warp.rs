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

//! Move voxels to the keys a warp lands them in, carrying their health, point
//! moments and fine cells, and keeping support counts, the healthy mask and
//! the viz in step.

use super::*;

/// One voxel's move: its key, the key its warped center lands in, and how far
/// the warped center sits from that voxel's center.
#[derive(Clone, Copy, Debug)]
pub struct VoxelMove {
    pub from: VoxelKey,
    pub to: VoxelKey,
    pub offset: Vector3<f32>,
}

impl Voxel {
    /// Re-center the point moments by `offset`: each point moves with its
    /// voxel's center, which now sits `offset` from the new voxel's center.
    fn shift(&mut self, offset: Vector3<f32>) {
        let n = self.num_pts as f32;
        self.m2 += self.sum * offset.transpose()
            + offset * self.sum.transpose()
            + n * offset * offset.transpose();
        self.sum += n * offset;
        self.normal = NormalFit::Stale;
    }

    /// Fold another voxel that landed in the same key into this one. Health
    /// is left to the caller, which owns the healthy mask.
    fn absorb(&mut self, other: &Voxel) {
        self.fine |= other.fine;
        self.num_pts += other.num_pts;
        self.next_fit_pts = self.next_fit_pts.min(other.next_fit_pts);
        self.sum += other.sum;
        self.m2 += other.m2;
        self.normal = NormalFit::Stale;
    }
}

impl VoxelMap {
    /// Move each voxel to its new key, merging into whatever is there. Every
    /// mover leaves before any lands, so support counts stay exact.
    pub fn move_voxels(&mut self, moves: &[VoxelMove]) {
        let mut changed: AHashSet<VoxelKey> = AHashSet::with_capacity(moves.len());
        let mut removed: Vec<VoxelKey> = Vec::new();
        let mut taken: Vec<(VoxelKey, Voxel)> = Vec::new();
        for m in moves {
            if m.from == m.to {
                if let Some(v) = self.voxels.get_mut(&m.from) {
                    v.shift(m.offset);
                    changed.insert(m.from);
                }
                continue;
            }
            let Some(mut v) = self.voxels.remove(&m.from) else {
                continue;
            };
            if v.health > 0 {
                self.health_crossed(m.from, false);
            }
            self.changed_chunks.insert(chunk_of(m.from));
            removed.push(m.from);
            v.shift(m.offset);
            taken.push((m.to, v));
        }
        for (to, v) in taken {
            self.land(to, v);
            changed.insert(to);
        }
        mark_stale(self, &changed, &removed);
    }

    /// Put a moved voxel at `to`, merging into one already there.
    fn land(&mut self, to: VoxelKey, v: Voxel) {
        self.changed_chunks.insert(chunk_of(to));
        let health = v.health;
        if let Some(existing) = self.voxels.get_mut(&to) {
            existing.absorb(&v);
            let (was, now) = self
                .voxels
                .update_health(to, |h| h.max(health))
                .expect("present above");
            if (was > 0) != (now > 0) {
                self.health_crossed(to, now > 0);
            }
            return;
        }
        let support = self.count_healthy_neighbors(to);
        self.voxels.insert(to, v, support);
        if health > 0 {
            self.health_crossed(to, true);
        }
    }

    /// Rebuild the map with `moves` applied: every voxel taken out and put
    /// back at its key, support counted once per voxel instead of updated
    /// around each mover, for when most of the map moves.
    pub fn rebuild_moved(&mut self, moves: &[VoxelMove]) {
        let moving: AHashMap<VoxelKey, (VoxelKey, Vector3<f32>)> =
            moves.iter().map(|m| (m.from, (m.to, m.offset))).collect();
        let mut landed: AHashMap<VoxelKey, Voxel> = AHashMap::with_capacity(self.voxels.len());
        for chunk in self.voxels.chunks() {
            for (key, v) in chunk.iter() {
                let mut v = v.clone();
                let to = match moving.get(&key) {
                    Some(&(to, offset)) => {
                        v.shift(offset);
                        to
                    }
                    None => key,
                };
                match landed.get_mut(&to) {
                    Some(existing) => {
                        existing.absorb(&v);
                        existing.health = existing.health.max(v.health);
                    }
                    None => {
                        landed.insert(to, v);
                    }
                }
            }
        }
        let old_chunks: Vec<ChunkKey> = self.voxels.chunks().map(|c| c.key).collect();
        self.changed_chunks.extend(old_chunks);
        self.voxels.clear();
        let keys: Vec<VoxelKey> = landed.keys().copied().collect();
        for (key, mut v) in landed {
            v.normal = NormalFit::Stale;
            self.voxels.insert(key, v, 0);
        }
        for &key in &keys {
            let support = self.count_healthy_neighbors(key);
            self.voxels
                .for_each_near_mut(key, 0, |_, _, s| *s = support);
            self.changed_chunks.insert(chunk_of(key));
        }
    }
}

/// Each voxel's center, the point a warp moves it by.
pub fn voxel_centers(keys: &[VoxelKey], voxel_size: f32) -> Vec<[f32; 3]> {
    keys.par_iter()
        .map(|&k| {
            let (x, y, z) = voxel_center(k, voxel_size);
            [x, y, z]
        })
        .collect()
}

/// Where each voxel lands given its warped center.
pub fn voxel_moves(keys: &[VoxelKey], warped: &[[f32; 3]], voxel_size: f32) -> Vec<VoxelMove> {
    let inv = 1.0 / voxel_size;
    keys.par_iter()
        .zip(warped.par_iter())
        .map(|(&from, w)| {
            let to = world_to_voxel(w[0], w[1], w[2], inv);
            let (cx, cy, cz) = voxel_center(to, voxel_size);
            VoxelMove {
                from,
                to,
                offset: Vector3::new(w[0] - cx, w[1] - cy, w[2] - cz),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests;
