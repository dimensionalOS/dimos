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
use std::ops::Bound::{Excluded, Unbounded};

use ahash::{AHashMap, AHashSet, AHasher};

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

/// A cell packed into a message seq so the viewer keys entities by it: i in
/// the high 16 bits, j in the low 16, both signed.
pub fn pack_cell((i, j): Cell) -> i32 {
    debug_assert!(i16::try_from(i).is_ok() && i16::try_from(j).is_ok());
    (i << 16) | (j & 0xffff)
}

/// Cells an update may have changed since the last tick.
enum Dirty {
    All,
    Cells(AHashSet<Cell>),
}

impl Dirty {
    fn contains(&self, cell: &Cell) -> bool {
        match self {
            Dirty::All => true,
            Dirty::Cells(cells) => cells.contains(cell),
        }
    }
}

/// The surface and segments bucketed into their cells, keeping the wanted
/// cells only, or every cell when none are named.
fn bucket(
    surface: impl Iterator<Item = (VoxelKey, f32)>,
    segments: impl Iterator<Item = Segment>,
    pitch: i32,
    wanted: Option<&AHashSet<Cell>>,
) -> AHashMap<Cell, RegionContent> {
    let want = |cell: &Cell| wanted.is_none_or(|w| w.contains(cell));
    let mut cells: AHashMap<Cell, RegionContent> = AHashMap::new();
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
    cells
}

/// What each cell last published, so a tick sends the cells whose content
/// changed plus a slice of the sweep. Only the cells an update touched are
/// hashed.
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
            dirty: Dirty::Cells(AHashSet::new()),
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

    /// The cells due this tick with their content, in send order: the sweep
    /// slice, then the changed cells, then the ones that just emptied. The
    /// sweep goes first since congestion drops the tail of a burst.
    pub fn tick(
        &mut self,
        surface: impl Iterator<Item = (VoxelKey, f32)>,
        segments: impl Iterator<Item = Segment>,
    ) -> Vec<(Cell, RegionContent)> {
        let dirty = std::mem::replace(&mut self.dirty, Dirty::Cells(AHashSet::new()));
        let candidates = self.sweep_candidates(&dirty);
        let wanted = match dirty {
            Dirty::All => None,
            Dirty::Cells(mut cells) => {
                cells.extend(candidates.iter().copied());
                Some(cells)
            }
        };
        let cells = bucket(surface, segments, self.pitch, wanted.as_ref());
        let (mut due, unchanged) = self.classify(cells, wanted.as_ref());
        let swept = self.sweep_into(&mut due, candidates, unchanged);
        due.rotate_right(swept);
        due
    }

    /// Record what the checked cells now hold. The changed cells then the
    /// ones that just emptied, in cell order, and the content of the rest.
    fn classify(
        &mut self,
        cells: AHashMap<Cell, RegionContent>,
        wanted: Option<&AHashSet<Cell>>,
    ) -> (Vec<(Cell, RegionContent)>, AHashMap<Cell, RegionContent>) {
        let mut vanished: Vec<(Cell, RegionContent)> = Vec::new();
        for (cell, slot) in &mut self.published {
            let checked = wanted.is_none_or(|w| w.contains(cell));
            if checked && slot.is_some() && !cells.contains_key(cell) {
                *slot = None;
                vanished.push((*cell, RegionContent::default()));
            }
        }
        let mut cells: Vec<(Cell, RegionContent)> = cells.into_iter().collect();
        cells.sort_unstable_by_key(|(cell, _)| *cell);
        let mut due: Vec<(Cell, RegionContent)> = Vec::new();
        let mut unchanged: AHashMap<Cell, RegionContent> = AHashMap::new();
        for (cell, content) in cells {
            let fp = content.fingerprint();
            if self.published.insert(cell, Some(fp)) != Some(Some(fp)) {
                due.push((cell, content));
            } else {
                unchanged.insert(cell, content);
            }
        }
        due.extend(vanished);
        (due, unchanged)
    }

    /// Append the sweep slice: the first candidates not already due. Returns
    /// the number of cells appended.
    fn sweep_into(
        &mut self,
        due: &mut Vec<(Cell, RegionContent)>,
        candidates: Vec<Cell>,
        mut unchanged: AHashMap<Cell, RegionContent>,
    ) -> usize {
        let busy: AHashSet<Cell> = due.iter().map(|(cell, _)| *cell).collect();
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
        swept
    }

    /// Published cells past the cursor, wrapping, up to the one that makes a
    /// full sweep slice of cells not dirty. Only dirty cells can turn out
    /// changed, so the rest are free for the sweep.
    fn sweep_candidates(&self, dirty: &Dirty) -> Vec<Cell> {
        let (past, wrapped) = match self.cursor {
            Some(cursor) => (
                self.published.range((Excluded(cursor), Unbounded)),
                Some(self.published.range(..=cursor)),
            ),
            None => (self.published.range(..), None),
        };
        let mut candidates: Vec<Cell> = Vec::new();
        let mut clean = 0;
        for (cell, _) in past.chain(wrapped.into_iter().flatten()) {
            if clean == self.sweep {
                break;
            }
            if !dirty.contains(cell) {
                clean += 1;
            }
            candidates.push(*cell);
        }
        candidates
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
        // The sweep slice starting at the first key, then the emptied cell, empty.
        assert_eq!(due_cells(&due), vec![(0, 0), (1, 0)]);
        assert!(due[1].1.surface.is_empty() && due[1].1.segments.is_empty());

        // The sweep reaches the emptied cell, sends it empty once more, forgets it.
        let due = tick(&mut viz, &[(0, 0)], &[]);
        assert_eq!(due_cells(&due), vec![(1, 0)]);
        assert!(due[0].1.surface.is_empty());
        assert_eq!(due_cells(&tick(&mut viz, &[(0, 0)], &[])), vec![(0, 0)]);
        assert_eq!(due_cells(&tick(&mut viz, &[(0, 0)], &[])), vec![(0, 0)]);
    }

    #[test]
    fn the_sweep_slice_goes_first_and_runs_on_a_busy_tick() {
        let mut viz = RegionViz::new(10, 0, 2);
        let map = [(0, 0), (15, 0), (0, 15), (15, 15), (25, 0)];
        viz.mark_all();
        tick(&mut viz, &map, &[]);

        // Three changes still leave a full slice of two unchanged cells, sent first.
        let mut changed = surface(&map);
        for item in changed.iter_mut().take(3) {
            item.1 = 0.25;
        }
        viz.mark_all();
        assert_eq!(
            due_cells(&viz.tick(changed.into_iter(), std::iter::empty())),
            vec![(1, 1), (2, 0), (0, 0), (0, 1), (1, 0)]
        );
    }

    #[test]
    fn a_busy_tick_of_marked_cells_still_carries_a_full_sweep_slice() {
        let mut viz = RegionViz::new(10, 0, 2);
        let map = [(0, 0), (0, 15), (15, 0), (15, 15), (25, 0)];
        viz.mark_all();
        tick(&mut viz, &map, &[]);

        // The three cells the sweep would reach first all changed.
        let mut changed = surface(&map);
        for item in changed.iter_mut().take(3) {
            item.1 = 0.25;
        }
        viz.mark_window((0, 0, 0, 15));
        viz.mark_window((15, 15, 0, 0));
        let due = viz.tick(changed.into_iter(), std::iter::empty());
        assert_eq!(
            due_cells(&due),
            vec![(1, 1), (2, 0), (0, 0), (0, 1), (1, 0)]
        );
        assert!(due.iter().all(|(_, content)| content.surface.len() == 1));
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
