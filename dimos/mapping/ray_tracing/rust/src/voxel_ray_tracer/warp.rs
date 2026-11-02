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
    /// back at its key in bulk, chunks built and supports counted in parallel,
    /// for when most of the map moves.
    pub fn rebuild_moved(&mut self, moves: &[VoxelMove]) {
        let moving: AHashMap<VoxelKey, (VoxelKey, Vector3<f32>)> =
            moves.iter().map(|m| (m.from, (m.to, m.offset))).collect();
        let chunks: Vec<ChunkRef> = self.voxels.chunks().collect();
        let mut landed: Vec<(VoxelKey, Voxel)> = chunks
            .par_iter()
            .flat_map_iter(|chunk| {
                let moving = &moving;
                chunk.iter().map(move |(key, v)| {
                    let mut v = v.clone();
                    v.normal = NormalFit::Stale;
                    match moving.get(&key) {
                        Some(&(to, offset)) => {
                            v.shift(offset);
                            (to, v)
                        }
                        None => (key, v),
                    }
                })
            })
            .collect();
        let old_chunks: Vec<ChunkKey> = chunks.iter().map(|c| c.key).collect();
        drop(chunks);
        landed.par_sort_unstable_by_key(|(k, _)| (chunk_of(*k), *k));
        // Voxels that landed on one key merge into the first.
        let mut merged: Vec<(VoxelKey, Voxel)> = Vec::with_capacity(landed.len());
        for (key, v) in landed {
            match merged.last_mut() {
                Some((last, existing)) if *last == key => {
                    existing.absorb(&v);
                    existing.health = existing.health.max(v.health);
                }
                _ => merged.push((key, v)),
            }
        }
        self.changed_chunks.extend(old_chunks);
        self.changed_chunks
            .extend(merged.iter().map(|(k, _)| chunk_of(*k)));
        self.voxels = ChunkMap::from_sorted(merged);
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

impl VoxelMap {
    /// Each voxel's mean point, its center when it holds none. Warping the mean
    /// rather than the center keeps a sub-voxel shift: it accumulates in the
    /// moments over closures until the mean crosses into another voxel.
    pub fn voxel_means(&self, keys: &[VoxelKey], voxel_size: f32) -> Vec<[f32; 3]> {
        keys.par_iter()
            .map(|&k| {
                let (x, y, z) = voxel_center(k, voxel_size);
                let mean = self
                    .voxels
                    .get(&k)
                    .filter(|v| v.num_pts > 0)
                    .map_or(Vector3::zeros(), |v| v.sum / v.num_pts as f32);
                [x + mean.x, y + mean.y, z + mean.z]
            })
            .collect()
    }
}

/// Where each voxel lands when its point `from_points[i]` warps to `warped[i]`:
/// the key of the warped point, and how far the old center lands from the new
/// one, which re-centers the moments.
pub fn voxel_moves(
    keys: &[VoxelKey],
    from_points: &[[f32; 3]],
    warped: &[[f32; 3]],
    voxel_size: f32,
) -> Vec<VoxelMove> {
    let inv = 1.0 / voxel_size;
    keys.par_iter()
        .zip(from_points.par_iter().zip(warped.par_iter()))
        .map(|(&from, (p, w))| {
            let to = world_to_voxel(w[0], w[1], w[2], inv);
            let (ox, oy, oz) = voxel_center(from, voxel_size);
            let (cx, cy, cz) = voxel_center(to, voxel_size);
            VoxelMove {
                from,
                to,
                offset: Vector3::new(
                    w[0] - p[0] + ox - cx,
                    w[1] - p[1] + oy - cy,
                    w[2] - p[2] + oz - cz,
                ),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests;
