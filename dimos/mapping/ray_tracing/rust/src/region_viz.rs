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

//! The map as viewer regions. Square cells of the region grid publish on their
//! own when a chunk in them changed, plus a round robin slice, so a viewer
//! takes the map in messages a lossy link can carry and heals what it lost.

use std::collections::{BTreeMap, BTreeSet};

use crate::voxel_ray_tracer::{ChunkKey, CHUNK_SIZE};

pub type Cell = (i32, i32);

/// The region grid cell a chunk's center falls in. Cells never run smaller
/// than a chunk, so a chunk lies in exactly one.
pub fn region_of(chunk: ChunkKey, voxel_size: f32, region_m: f32) -> Cell {
    let edge = CHUNK_SIZE as f32 * voxel_size;
    let cell_m = region_m.max(edge);
    (
        (((chunk.0 as f32) + 0.5) * edge / cell_m).floor() as i32,
        (((chunk.1 as f32) + 0.5) * edge / cell_m).floor() as i32,
    )
}

/// A cell packed into a message seq so the viewer keys entities by it.
pub fn pack_cell((i, j): Cell) -> i32 {
    debug_assert!(i16::try_from(i).is_ok() && i16::try_from(j).is_ok());
    (i << 16) | (j & 0xffff)
}

/// Which cells a viewer has, so a tick can send the changed ones, an empty
/// message for the ones that vanished, and the next slice of the sweep.
#[derive(Default)]
pub struct RegionSweep {
    known: BTreeSet<Cell>,
    cursor: Option<Cell>,
}

impl RegionSweep {
    /// The cells due this tick. A cell absent from `present` is due empty
    /// once, when a viewer had it.
    pub fn tick<T>(
        &mut self,
        changed: impl IntoIterator<Item = Cell>,
        present: &BTreeMap<Cell, T>,
        sweep: usize,
    ) -> Vec<Cell> {
        let mut due: BTreeSet<Cell> = changed
            .into_iter()
            .filter(|cell| present.contains_key(cell))
            .collect();
        due.extend(self.known.iter().filter(|c| !present.contains_key(c)));
        due.extend(self.sweep_cells(present, sweep));
        self.known = present.keys().copied().collect();
        due.into_iter().collect()
    }

    fn sweep_cells<T>(&mut self, present: &BTreeMap<Cell, T>, sweep: usize) -> Vec<Cell> {
        let keys: Vec<Cell> = present.keys().copied().collect();
        if keys.is_empty() || sweep == 0 {
            return Vec::new();
        }
        let start = match self.cursor {
            Some(cursor) => keys.partition_point(|k| *k <= cursor),
            None => 0,
        };
        let swept: Vec<Cell> = (0..sweep.min(keys.len()))
            .map(|n| keys[(start + n) % keys.len()])
            .collect();
        self.cursor = swept.last().copied();
        swept
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn present(cells: &[Cell]) -> BTreeMap<Cell, ()> {
        cells.iter().map(|&c| (c, ())).collect()
    }

    #[test]
    fn regions_group_chunks_by_center_and_never_split_a_chunk() {
        // 16 voxels of 0.25 m make 4 m chunks, on a 4 m grid: one chunk per cell.
        assert_eq!(region_of((0, 0, 3), 0.25, 4.0), (0, 0));
        assert_eq!(region_of((-1, 2, 0), 0.25, 4.0), (-1, 2));
        // 1.28 m chunks on a 4 m grid: chunk 3 is centered at 4.48 m, cell 1.
        assert_eq!(region_of((2, 0, 0), 0.08, 4.0), (0, 0));
        assert_eq!(region_of((3, 0, 0), 0.08, 4.0), (1, 0));
        // A grid finer than a chunk widens to the chunk.
        assert_eq!(region_of((5, -1, 0), 0.25, 1.0), (5, -1));
    }

    #[test]
    fn packed_cells_keep_sign_in_both_halves() {
        let seq = pack_cell((-3, 5));
        assert_eq!((seq >> 16, ((seq & 0xffff) ^ 0x8000) - 0x8000), (-3, 5));
        let seq = pack_cell((7, -2));
        assert_eq!((seq >> 16, ((seq & 0xffff) ^ 0x8000) - 0x8000), (7, -2));
    }

    #[test]
    fn changed_cells_are_due_and_a_vanished_cell_is_due_once() {
        let mut sweep = RegionSweep::default();
        let map = present(&[(0, 0), (1, 0), (0, 1)]);
        assert_eq!(sweep.tick([(1, 0), (9, 9)], &map, 0), vec![(1, 0)]);

        let smaller = present(&[(0, 0), (0, 1)]);
        assert_eq!(sweep.tick([], &smaller, 0), vec![(1, 0)]);
        assert!(sweep.tick([], &smaller, 0).is_empty());
    }

    #[test]
    fn the_sweep_walks_every_cell_and_wraps() {
        let mut sweep = RegionSweep::default();
        let map = present(&[(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]);
        let mut seen = Vec::new();
        for _ in 0..3 {
            seen.extend(sweep.tick([], &map, 2));
        }
        assert_eq!(seen.len(), 6);
        seen.sort();
        seen.dedup();
        assert_eq!(seen, vec![(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]);
    }
}
