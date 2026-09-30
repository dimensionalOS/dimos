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

//! Viz artifacts as regions. The surface and the edge corridors are bucketed
//! into square cells and each cell publishes on its own, so a viewer takes the
//! map in messages a lossy link can carry, and a change costs its cell only.

use std::collections::BTreeMap;
use std::hash::{Hash, Hasher};

use ahash::AHasher;

use crate::voxel::VoxelKey;

pub type Cell = (i32, i32);
pub type Segment = (VoxelKey, VoxelKey, f32);

/// One cell's surface voxels with their clearance, and its corridor segments.
#[derive(Default)]
pub struct RegionContent {
    pub surface: Vec<(VoxelKey, f32)>,
    pub segments: Vec<Segment>,
}

impl RegionContent {
    fn fingerprint(&self) -> u64 {
        // Order free: the graph rebuilds reorder cells whose content did not change.
        let mut acc = (self.surface.len() as u64) << 32 | self.segments.len() as u64;
        for (key, clearance) in &self.surface {
            acc = acc.wrapping_add(hash_item((key, clearance.to_bits())));
        }
        for (a, b, cost) in &self.segments {
            acc = acc.wrapping_add(hash_item((a, b, cost.to_bits())));
        }
        acc
    }
}

fn hash_item<T: Hash>(item: T) -> u64 {
    let mut h = AHasher::default();
    item.hash(&mut h);
    h.finish()
}

/// Cell of a voxel column at a pitch in voxels.
pub fn cell_of((ix, iy, _): VoxelKey, pitch: i32) -> Cell {
    (ix.div_euclid(pitch), iy.div_euclid(pitch))
}

/// A cell packed into a message seq so the viewer keys entities by it.
pub fn pack_cell((i, j): Cell) -> i32 {
    debug_assert!(i16::try_from(i).is_ok() && i16::try_from(j).is_ok());
    (i << 16) | (j & 0xffff)
}

/// What each cell last published, so a tick republishes only the cells whose
/// content changed plus a slice of the sweep that heals a viewer's losses.
pub struct RegionViz {
    pitch: i32,
    sweep: usize,
    published: BTreeMap<Cell, u64>,
    cursor: Option<Cell>,
}

impl RegionViz {
    pub fn new(pitch_voxels: i32, sweep: usize) -> Self {
        Self {
            pitch: pitch_voxels.max(1),
            sweep,
            published: BTreeMap::new(),
            cursor: None,
        }
    }

    /// The cells due this tick with their current content. A cell that
    /// emptied since its last publish is due once more, empty, then forgotten.
    pub fn tick(
        &mut self,
        surface: Vec<(VoxelKey, f32)>,
        segments: Vec<Segment>,
    ) -> Vec<(Cell, RegionContent)> {
        let mut cells: BTreeMap<Cell, RegionContent> = BTreeMap::new();
        for entry in surface {
            cells
                .entry(cell_of(entry.0, self.pitch))
                .or_default()
                .surface
                .push(entry);
        }
        for segment in segments {
            cells
                .entry(cell_of(segment.0, self.pitch))
                .or_default()
                .segments
                .push(segment);
        }

        let mut due: BTreeMap<Cell, RegionContent> = BTreeMap::new();
        let cleared: Vec<Cell> = self
            .published
            .keys()
            .filter(|cell| !cells.contains_key(cell))
            .copied()
            .collect();
        for cell in cleared {
            self.published.remove(&cell);
            due.insert(cell, RegionContent::default());
        }

        let swept = self.sweep_cells(&cells);
        for (cell, content) in cells {
            let fp = content.fingerprint();
            let changed = self.published.insert(cell, fp) != Some(fp);
            if changed || swept.contains(&cell) {
                due.insert(cell, content);
            }
        }
        due.into_iter().collect()
    }

    /// The next slice of cells past the cursor, wrapping around.
    fn sweep_cells(&mut self, cells: &BTreeMap<Cell, RegionContent>) -> Vec<Cell> {
        let keys: Vec<Cell> = cells.keys().copied().collect();
        if keys.is_empty() || self.sweep == 0 {
            return Vec::new();
        }
        let start = match self.cursor {
            Some(cursor) => keys.partition_point(|k| *k <= cursor),
            None => 0,
        };
        let swept: Vec<Cell> = (0..self.sweep.min(keys.len()))
            .map(|n| keys[(start + n) % keys.len()])
            .collect();
        self.cursor = swept.last().copied();
        swept
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn surface(cells: &[(i32, i32)]) -> Vec<(VoxelKey, f32)> {
        cells.iter().map(|&(x, y)| ((x, y, 0), 1.0)).collect()
    }

    fn due_cells(due: &[(Cell, RegionContent)]) -> Vec<Cell> {
        due.iter().map(|(c, _)| *c).collect()
    }

    #[test]
    fn cells_bucket_columns_at_the_pitch_with_negative_floors() {
        assert_eq!(cell_of((0, 0, 3), 50), (0, 0));
        assert_eq!(cell_of((49, 50, 0), 50), (0, 1));
        assert_eq!(cell_of((-1, -50, 0), 50), (-1, -1));
        assert_eq!(cell_of((-51, 0, 0), 50), (-2, 0));
    }

    #[test]
    fn packed_cells_round_trip_through_the_seq_layout() {
        let seq = pack_cell((-3, 5));
        assert_eq!(seq >> 16, -3);
        assert_eq!(((seq & 0xffff) ^ 0x8000) - 0x8000, 5);
        let seq = pack_cell((7, -2));
        assert_eq!(seq >> 16, 7);
        assert_eq!(((seq & 0xffff) ^ 0x8000) - 0x8000, -2);
    }

    #[test]
    fn first_tick_publishes_every_cell_and_a_quiet_tick_only_the_sweep() {
        let mut viz = RegionViz::new(10, 1);
        let map = || surface(&[(0, 0), (1, 1), (15, 0), (0, 25)]);
        let first = viz.tick(map(), Vec::new());
        assert_eq!(due_cells(&first), vec![(0, 0), (0, 2), (1, 0)]);
        assert_eq!(first[0].1.surface.len(), 2);

        let quiet = viz.tick(map(), Vec::new());
        assert_eq!(due_cells(&quiet), vec![(0, 2)]);
        let quiet = viz.tick(map(), Vec::new());
        assert_eq!(due_cells(&quiet), vec![(1, 0)]);
        let quiet = viz.tick(map(), Vec::new());
        assert_eq!(due_cells(&quiet), vec![(0, 0)]);
    }

    #[test]
    fn a_changed_cell_is_due_and_reordering_is_not_a_change() {
        let mut viz = RegionViz::new(10, 0);
        viz.tick(surface(&[(0, 0), (1, 1), (15, 0)]), Vec::new());

        let reordered = viz.tick(surface(&[(15, 0), (1, 1), (0, 0)]), Vec::new());
        assert!(reordered.is_empty());

        let mut changed = surface(&[(0, 0), (1, 1), (15, 0)]);
        changed[2].1 = 0.5;
        assert_eq!(due_cells(&viz.tick(changed, Vec::new())), vec![(1, 0)]);

        let edge = vec![((15, 0, 0), (16, 0, 0), 2.0)];
        assert_eq!(
            due_cells(&viz.tick(surface(&[(0, 0), (1, 1), (15, 0)]), edge)),
            vec![(1, 0)]
        );
    }

    #[test]
    fn an_emptied_cell_is_due_empty_once_then_forgotten() {
        let mut viz = RegionViz::new(10, 0);
        viz.tick(surface(&[(0, 0), (15, 0)]), Vec::new());

        let due = viz.tick(surface(&[(0, 0)]), Vec::new());
        assert_eq!(due_cells(&due), vec![(1, 0)]);
        assert!(due[0].1.surface.is_empty() && due[0].1.segments.is_empty());

        assert!(viz.tick(surface(&[(0, 0)]), Vec::new()).is_empty());
    }

    #[test]
    fn the_sweep_wraps_and_covers_every_cell() {
        let mut viz = RegionViz::new(10, 2);
        let map = || surface(&[(0, 0), (15, 0), (0, 15), (15, 15), (25, 0)]);
        viz.tick(map(), Vec::new());
        let mut seen = Vec::new();
        for _ in 0..3 {
            seen.extend(due_cells(&viz.tick(map(), Vec::new())));
        }
        assert_eq!(seen.len(), 6);
        let mut unique = seen.clone();
        unique.sort();
        unique.dedup();
        assert_eq!(unique, vec![(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]);
    }
}
