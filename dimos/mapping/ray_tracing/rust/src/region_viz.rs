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
//! own when a chunk in them changed, plus a fixed slice every tick, so a viewer
//! takes the map in messages a lossy link can carry and heals what it lost.

use std::collections::{BTreeMap, BTreeSet};

use crate::voxel_ray_tracer::Cell;

/// A cell packed into a message seq so the viewer keys entities by it: i in
/// the high 16 bits, j in the low 16, both signed.
pub fn pack_cell((i, j): Cell) -> i32 {
    debug_assert!(i16::try_from(i).is_ok() && i16::try_from(j).is_ok());
    (i << 16) | (j & 0xffff)
}

/// Which cells a viewer has, so a tick sends the next slice of the sweep, an
/// empty message for the cells that vanished, and the changed cells.
#[derive(Default)]
pub struct RegionSweep {
    /// Cells a viewer holds, and whether the map still has them. A vanished
    /// cell stays until the sweep has sent it empty once more.
    known: BTreeMap<Cell, bool>,
    cursor: Option<Cell>,
}

impl RegionSweep {
    /// The cells due this tick, in send order: `sweep` cells from the cursor,
    /// then the vanished cells, then the changed ones. The sweep goes first
    /// since congestion drops the tail of a burst.
    pub fn tick<T>(
        &mut self,
        changed: impl IntoIterator<Item = Cell>,
        present: &BTreeMap<Cell, T>,
        sweep: usize,
    ) -> Vec<Cell> {
        let changed: BTreeSet<Cell> = changed
            .into_iter()
            .filter(|cell| present.contains_key(cell))
            .collect();
        let mut vanished: Vec<Cell> = Vec::new();
        for (cell, still) in self.known.iter_mut() {
            if *still && !present.contains_key(cell) {
                *still = false;
                vanished.push(*cell);
            }
        }
        for cell in present.keys() {
            self.known.insert(*cell, true);
        }

        let busy: BTreeSet<Cell> = vanished.iter().chain(&changed).copied().collect();
        let keys: Vec<Cell> = self.known.keys().copied().collect();
        let start = match self.cursor {
            Some(cursor) => keys.partition_point(|k| *k <= cursor),
            None => 0,
        };
        let mut due: Vec<Cell> = Vec::new();
        for n in 0..keys.len() {
            if due.len() == sweep {
                break;
            }
            let cell = keys[(start + n) % keys.len()];
            if busy.contains(&cell) {
                continue;
            }
            due.push(cell);
            self.cursor = Some(cell);
        }
        for cell in &due {
            if self.known.get(cell) == Some(&false) {
                self.known.remove(cell);
            }
        }
        due.extend(vanished);
        due.extend(changed);
        due
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn present(cells: &[Cell]) -> BTreeMap<Cell, ()> {
        cells.iter().map(|&c| (c, ())).collect()
    }

    #[test]
    fn packed_cells_keep_sign_in_both_halves() {
        let seq = pack_cell((-3, 5));
        assert_eq!((seq >> 16, ((seq & 0xffff) ^ 0x8000) - 0x8000), (-3, 5));
        let seq = pack_cell((7, -2));
        assert_eq!((seq >> 16, ((seq & 0xffff) ^ 0x8000) - 0x8000), (7, -2));
    }

    #[test]
    fn changed_cells_are_due_and_a_vanished_cell_is_due_now_and_once_from_the_sweep() {
        let mut sweep = RegionSweep::default();
        let map = present(&[(0, 0), (1, 0), (0, 1)]);
        assert_eq!(sweep.tick([(1, 0), (9, 9)], &map, 0), vec![(1, 0)]);

        let smaller = present(&[(0, 0), (0, 1)]);
        assert_eq!(sweep.tick([], &smaller, 0), vec![(1, 0)]);
        assert!(sweep.tick([], &smaller, 0).is_empty());
        // The sweep walks (0,0), (0,1), then the vanished (1,0) once more.
        assert_eq!(sweep.tick([], &smaller, 1), vec![(0, 0)]);
        assert_eq!(sweep.tick([], &smaller, 1), vec![(0, 1)]);
        assert_eq!(sweep.tick([], &smaller, 1), vec![(1, 0)]);
        assert_eq!(sweep.tick([], &smaller, 1), vec![(0, 0)]);
        assert_eq!(sweep.tick([], &smaller, 1), vec![(0, 1)]);
    }

    #[test]
    fn the_sweep_slice_goes_first_then_the_vanished_then_the_changed() {
        let mut sweep = RegionSweep::default();
        let map = present(&[(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]);
        sweep.tick([], &map, 0);
        let smaller = present(&[(0, 0), (0, 1), (1, 0), (1, 1)]);
        // Two swept cells skipping the busy ones, the vanished one, then the changed.
        assert_eq!(
            sweep.tick([(1, 1), (0, 1)], &smaller, 2),
            vec![(0, 0), (1, 0), (2, 0), (0, 1), (1, 1)]
        );
        // Three changes still leave a full slice of two, which retires the vanished cell.
        assert_eq!(
            sweep.tick([(0, 0), (0, 1), (1, 0)], &smaller, 2),
            vec![(1, 1), (2, 0), (0, 0), (0, 1), (1, 0)]
        );
    }

    #[test]
    fn the_sweep_walks_every_cell_and_wraps() {
        let mut sweep = RegionSweep::default();
        let map = present(&[(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]);
        sweep.tick([], &map, 0);
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
