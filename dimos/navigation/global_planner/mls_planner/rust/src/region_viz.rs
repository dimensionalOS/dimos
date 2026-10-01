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

use crate::mls_planner::ColumnWindow;
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

/// What each cell last published, so a tick republishes the cells whose
/// content changed plus a fixed slice of the sweep that heals a viewer's
/// losses. Only the cells an update touched are hashed, the scan over the
/// surface still walks every item.
pub struct RegionViz {
    pitch: i32,
    /// Columns past a rewritten window the graph repair can still reach.
    reach: i32,
    sweep: usize,
    dirty: Dirty,
    /// Fingerprint of what each cell last published. None once the cell
    /// emptied, until the sweep has sent it empty once more.
    published: BTreeMap<Cell, Option<u64>>,
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
    pub fn mark_window(&mut self, (x0, x1, y0, y1): ColumnWindow) {
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
    /// first, then the ones that just emptied, then the sweep slice. A cell
    /// that emptied goes out empty now, stays in the sweep until it has been
    /// sent empty once more, and is then forgotten.
    pub fn tick(
        &mut self,
        surface: impl Iterator<Item = (VoxelKey, f32)>,
        segments: impl Iterator<Item = Segment>,
    ) -> Vec<(Cell, RegionContent)> {
        let dirty = std::mem::replace(&mut self.dirty, Dirty::Cells(BTreeSet::new()));
        // Only dirty cells can turn out changed, so this many extra candidates
        // leave a full sweep slice of unchanged ones.
        let candidates = self.sweep_candidates(match &dirty {
            Dirty::All => self.published.len(),
            Dirty::Cells(cells) => cells.len(),
        });
        let wanted: Option<BTreeSet<Cell>> = match dirty {
            Dirty::All => None,
            Dirty::Cells(mut cells) => {
                cells.extend(candidates.iter().copied());
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

        let checked: Vec<Cell> = match &wanted {
            None => self.published.keys().copied().collect(),
            Some(w) => w.iter().copied().collect(),
        };
        let mut due: Vec<(Cell, RegionContent)> = Vec::new();
        let mut vanished: Vec<(Cell, RegionContent)> = Vec::new();
        let mut unchanged: BTreeMap<Cell, RegionContent> = BTreeMap::new();
        for cell in checked {
            if cells.contains_key(&cell) {
                continue;
            }
            if let Some(slot @ Some(_)) = self.published.get_mut(&cell) {
                *slot = None;
                vanished.push((cell, RegionContent::default()));
            }
        }
        for (cell, content) in cells {
            let fp = content.fingerprint();
            if self.published.insert(cell, Some(fp)) != Some(Some(fp)) {
                due.push((cell, content));
            } else {
                unchanged.insert(cell, content);
            }
        }
        let busy: BTreeSet<Cell> = due.iter().chain(&vanished).map(|(c, _)| *c).collect();
        due.extend(vanished);

        let mut swept = 0;
        for cell in candidates {
            if swept == self.sweep {
                break;
            }
            if busy.contains(&cell) {
                continue;
            }
            match self.published.get(&cell) {
                Some(None) => {
                    self.published.remove(&cell);
                    due.push((cell, RegionContent::default()));
                }
                Some(Some(_)) => {
                    due.push((cell, unchanged.remove(&cell).unwrap_or_default()));
                }
                None => continue,
            }
            swept += 1;
            self.cursor = Some(cell);
        }
        due
    }

    /// The next `sweep + extra` published cells past the cursor, wrapping.
    fn sweep_candidates(&self, extra: usize) -> Vec<Cell> {
        let keys: Vec<Cell> = self.published.keys().copied().collect();
        if keys.is_empty() || self.sweep == 0 {
            return Vec::new();
        }
        let start = match self.cursor {
            Some(cursor) => keys.partition_point(|k| *k <= cursor),
            None => 0,
        };
        (0..(self.sweep + extra).min(keys.len()))
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

    fn tick(
        viz: &mut RegionViz,
        columns: &[(i32, i32)],
        segments: &[Segment],
    ) -> Vec<(Cell, RegionContent)> {
        viz.tick(surface(columns).into_iter(), segments.iter().copied())
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
        // Nothing was published before, so there is nothing to sweep yet.
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
    fn an_emptied_cell_goes_out_empty_now_and_once_more_from_the_sweep() {
        let mut viz = RegionViz::new(10, 0, 1);
        viz.mark_all();
        tick(&mut viz, &[(0, 0), (15, 0)], &[]);

        viz.mark_window((15, 15, 0, 0));
        let due = tick(&mut viz, &[(0, 0)], &[]);
        // Empty now, then the sweep slice starting at the first key.
        assert_eq!(due_cells(&due), vec![(1, 0), (0, 0)]);
        assert!(due[0].1.surface.is_empty() && due[0].1.segments.is_empty());

        // The sweep reaches the emptied cell, sends it empty once more, forgets it.
        let due = tick(&mut viz, &[(0, 0)], &[]);
        assert_eq!(due_cells(&due), vec![(1, 0)]);
        assert!(due[0].1.surface.is_empty());
        assert_eq!(due_cells(&tick(&mut viz, &[(0, 0)], &[])), vec![(0, 0)]);
        assert_eq!(due_cells(&tick(&mut viz, &[(0, 0)], &[])), vec![(0, 0)]);
    }

    #[test]
    fn changed_cells_come_first_and_the_sweep_runs_on_a_busy_tick() {
        let mut viz = RegionViz::new(10, 0, 2);
        let map = [(0, 0), (15, 0), (0, 15), (15, 15), (25, 0)];
        viz.mark_all();
        tick(&mut viz, &map, &[]);

        // Three changes still leave a full slice of two unchanged cells.
        let mut changed = surface(&map);
        for item in changed.iter_mut().take(3) {
            item.1 = 0.25;
        }
        viz.mark_all();
        assert_eq!(
            due_cells(&viz.tick(changed.into_iter(), std::iter::empty())),
            vec![(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]
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
