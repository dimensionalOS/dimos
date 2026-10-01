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

use std::collections::{BTreeMap, BTreeSet};
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

/// Cells an update may have changed since the last tick.
enum Dirty {
    All,
    Cells(BTreeSet<Cell>),
}

/// What each cell last published, so a tick republishes only the cells whose
/// content changed plus a slice of the sweep that heals a viewer's losses.
/// Only the cells an update touched are reread, so a tick costs what changed.
pub struct RegionViz {
    pitch: i32,
    /// Columns past a rewritten window the graph repair can still reach.
    reach: i32,
    sweep: usize,
    dirty: Dirty,
    published: BTreeMap<Cell, u64>,
    cursor: Option<Cell>,
}

impl RegionViz {
    pub fn new(pitch_voxels: i32, reach_voxels: i32, sweep: usize) -> Self {
        Self {
            pitch: pitch_voxels.max(1),
            reach: reach_voxels.max(0),
            sweep,
            dirty: Dirty::Cells(BTreeSet::new()),
            published: BTreeMap::new(),
            cursor: None,
        }
    }

    /// Every cell may have changed.
    pub fn mark_all(&mut self) {
        self.dirty = Dirty::All;
    }

    /// The cells covering an inclusive column window, widened by the reach.
    pub fn mark_window(&mut self, (x0, x1, y0, y1): (i32, i32, i32, i32)) {
        let Dirty::Cells(cells) = &mut self.dirty else {
            return;
        };
        let (cx0, cx1) = (
            (x0 - self.reach).div_euclid(self.pitch),
            (x1 + self.reach).div_euclid(self.pitch),
        );
        let (cy0, cy1) = (
            (y0 - self.reach).div_euclid(self.pitch),
            (y1 + self.reach).div_euclid(self.pitch),
        );
        for i in cx0..=cx1 {
            for j in cy0..=cy1 {
                cells.insert((i, j));
            }
        }
    }

    /// The cells due this tick with their current content: the changed ones
    /// first, then the ones that emptied (due once more, empty, then
    /// forgotten), then the sweep slice. A tick whose changes already
    /// outnumber the sweep skips it and leaves the cursor where it was.
    pub fn tick(
        &mut self,
        surface: impl Iterator<Item = (VoxelKey, f32)>,
        segments: impl Iterator<Item = Segment>,
    ) -> Vec<(Cell, RegionContent)> {
        let dirty = std::mem::replace(&mut self.dirty, Dirty::Cells(BTreeSet::new()));
        let swept = self.sweep_cells();
        let wanted: Option<BTreeSet<Cell>> = match dirty {
            Dirty::All => None,
            Dirty::Cells(mut cells) => {
                cells.extend(swept.iter().copied());
                Some(cells)
            }
        };
        let pitch = self.pitch;
        let want = |cell: &Cell| wanted.as_ref().is_none_or(|w| w.contains(cell));

        let mut cells: BTreeMap<Cell, RegionContent> = BTreeMap::new();
        for entry in surface {
            let cell = cell_of(entry.0, pitch);
            if want(&cell) {
                cells.entry(cell).or_default().surface.push(entry);
            }
        }
        for segment in segments {
            let cell = cell_of(segment.0, pitch);
            if want(&cell) {
                cells.entry(cell).or_default().segments.push(segment);
            }
        }

        let candidates: Vec<Cell> = match &wanted {
            None => self.published.keys().copied().collect(),
            Some(w) => w.iter().copied().collect(),
        };
        let mut vanished: Vec<(Cell, RegionContent)> = Vec::new();
        for cell in candidates {
            if !cells.contains_key(&cell) && self.published.remove(&cell).is_some() {
                vanished.push((cell, RegionContent::default()));
            }
        }
        let mut changed: Vec<(Cell, RegionContent)> = Vec::new();
        let mut unchanged: Vec<(Cell, RegionContent)> = Vec::new();
        for (cell, content) in cells {
            let fp = content.fingerprint();
            if self.published.insert(cell, fp) != Some(fp) {
                changed.push((cell, content));
            } else if swept.contains(&cell) {
                unchanged.push((cell, content));
            }
        }
        let mut due = changed;
        let changes = due.len();
        due.extend(vanished);
        if changes <= self.sweep {
            due.extend(unchanged);
            self.cursor = swept.last().copied().or(self.cursor);
        }
        due
    }

    /// The next slice of published cells past the cursor, wrapping around.
    /// The cursor moves only once the slice is sent.
    fn sweep_cells(&self) -> Vec<Cell> {
        let keys: Vec<Cell> = self.published.keys().copied().collect();
        if keys.is_empty() || self.sweep == 0 {
            return Vec::new();
        }
        let start = match self.cursor {
            Some(cursor) => keys.partition_point(|k| *k <= cursor),
            None => 0,
        };
        (0..self.sweep.min(keys.len()))
            .map(|n| keys[(start + n) % keys.len()])
            .collect()
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

    fn tick(viz: &mut RegionViz, s: &[(i32, i32)], e: &[Segment]) -> Vec<(Cell, RegionContent)> {
        viz.tick(surface(s).into_iter(), e.iter().copied())
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
        assert_eq!((seq >> 16, ((seq & 0xffff) ^ 0x8000) - 0x8000), (-3, 5));
        let seq = pack_cell((7, -2));
        assert_eq!((seq >> 16, ((seq & 0xffff) ^ 0x8000) - 0x8000), (7, -2));
    }

    #[test]
    fn a_full_mark_publishes_every_cell_and_a_quiet_tick_only_the_sweep() {
        let mut viz = RegionViz::new(10, 0, 1);
        let map = [(0, 0), (1, 1), (15, 0), (0, 25)];
        viz.mark_all();
        let first = tick(&mut viz, &map, &[]);
        assert_eq!(due_cells(&first), vec![(0, 0), (0, 2), (1, 0)]);
        assert_eq!(first[0].1.surface.len(), 2);

        assert_eq!(due_cells(&tick(&mut viz, &map, &[])), vec![(0, 0)]);
        assert_eq!(due_cells(&tick(&mut viz, &map, &[])), vec![(0, 2)]);
        assert_eq!(due_cells(&tick(&mut viz, &map, &[])), vec![(1, 0)]);
        assert_eq!(due_cells(&tick(&mut viz, &map, &[])), vec![(0, 0)]);
    }

    #[test]
    fn only_marked_cells_are_reread_and_reordering_is_not_a_change() {
        let mut viz = RegionViz::new(10, 0, 0);
        viz.mark_all();
        tick(&mut viz, &[(0, 0), (1, 1), (15, 0)], &[]);

        // Reordered, with every cell marked: nothing changed.
        viz.mark_all();
        assert!(tick(&mut viz, &[(15, 0), (1, 1), (0, 0)], &[]).is_empty());

        // A change in a cell nothing marked is not seen until a mark covers it.
        let mut changed = surface(&[(0, 0), (1, 1), (15, 0)]);
        changed[2].1 = 0.5;
        viz.mark_window((0, 0, 0, 0));
        assert!(viz
            .tick(changed.clone().into_iter(), std::iter::empty())
            .is_empty());
        viz.mark_window((15, 15, 0, 0));
        assert_eq!(
            due_cells(&viz.tick(changed.into_iter(), std::iter::empty())),
            vec![(1, 0)]
        );

        let edge = [((15, 0, 0), (16, 0, 0), 2.0)];
        viz.mark_window((15, 16, 0, 0));
        assert_eq!(
            due_cells(&tick(&mut viz, &[(0, 0), (1, 1), (15, 0)], &edge)),
            vec![(1, 0)]
        );
    }

    #[test]
    fn a_window_marks_the_cells_within_reach_of_it() {
        let mut viz = RegionViz::new(10, 3, 0);
        viz.mark_all();
        tick(&mut viz, &[(0, 0), (12, 0), (25, 0)], &[]);
        // Columns 8..=9 reach into cell 1 but not cell 2.
        viz.mark_window((8, 9, 0, 0));
        let mut changed = surface(&[(0, 0), (12, 0), (25, 0)]);
        changed[1].1 = 0.5;
        changed[2].1 = 0.5;
        assert_eq!(
            due_cells(&viz.tick(changed.into_iter(), std::iter::empty())),
            vec![(1, 0)]
        );
    }

    #[test]
    fn a_change_at_the_far_edge_of_the_reach_is_picked_up() {
        let mut viz = RegionViz::new(10, 5, 0);
        viz.mark_all();
        tick(&mut viz, &[(3, 0), (8, 0), (12, 0)], &[]);
        // A window at column 3 reaches column 8, in the same cell, and not 12.
        let mut changed = surface(&[(3, 0), (8, 0), (12, 0)]);
        changed[1].1 = 0.5;
        changed[2].1 = 0.5;
        viz.mark_window((3, 3, 0, 0));
        let due = viz.tick(changed.clone().into_iter(), std::iter::empty());
        assert_eq!(due_cells(&due), vec![(0, 0)]);
        // Column 12's change waits for a window that reaches its cell.
        viz.mark_window((7, 7, 0, 0));
        assert_eq!(
            due_cells(&viz.tick(changed.into_iter(), std::iter::empty())),
            vec![(1, 0)]
        );
    }

    #[test]
    fn an_emptied_cell_is_due_empty_once_then_forgotten() {
        let mut viz = RegionViz::new(10, 0, 0);
        viz.mark_all();
        tick(&mut viz, &[(0, 0), (15, 0)], &[]);

        viz.mark_window((15, 15, 0, 0));
        let due = tick(&mut viz, &[(0, 0)], &[]);
        assert_eq!(due_cells(&due), vec![(1, 0)]);
        assert!(due[0].1.surface.is_empty() && due[0].1.segments.is_empty());

        viz.mark_window((15, 15, 0, 0));
        assert!(tick(&mut viz, &[(0, 0)], &[]).is_empty());
    }

    #[test]
    fn changed_cells_come_first_and_a_full_tick_skips_the_sweep() {
        let mut viz = RegionViz::new(10, 0, 2);
        let map = [(0, 0), (15, 0), (0, 15), (15, 15), (25, 0)];
        viz.mark_all();
        tick(&mut viz, &map, &[]);

        // Cell (1,1) changes and (2,0) empties: changed, then vanished, then
        // the sweep slice, which starts at the first published cell.
        let mut changed = surface(&[(0, 0), (15, 0), (0, 15), (15, 15)]);
        changed[3].1 = 0.5;
        viz.mark_window((15, 25, 0, 15));
        assert_eq!(
            due_cells(&viz.tick(changed.clone().into_iter(), std::iter::empty())),
            vec![(1, 1), (2, 0), (0, 0), (0, 1)]
        );

        // Three changes exceed a sweep of 2: no sweep, cursor unmoved.
        for item in changed.iter_mut().take(3) {
            item.1 = 0.25;
        }
        viz.mark_all();
        assert_eq!(
            due_cells(&viz.tick(changed.clone().into_iter(), std::iter::empty())),
            vec![(0, 0), (0, 1), (1, 0)]
        );
        assert_eq!(
            due_cells(&viz.tick(changed.into_iter(), std::iter::empty())),
            vec![(1, 0), (1, 1)]
        );
    }

    #[test]
    fn the_sweep_wraps_and_covers_every_published_cell() {
        let mut viz = RegionViz::new(10, 0, 2);
        let map = [(0, 0), (15, 0), (0, 15), (15, 15), (25, 0)];
        viz.mark_all();
        tick(&mut viz, &map, &[]);
        let mut seen = Vec::new();
        for _ in 0..3 {
            seen.extend(due_cells(&tick(&mut viz, &map, &[])));
        }
        assert_eq!(seen.len(), 6);
        seen.sort();
        seen.dedup();
        assert_eq!(seen, vec![(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]);
    }
}
