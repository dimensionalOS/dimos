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

//! Dense row-major bit grid with word-parallel morphology.

const WORD_BITS: usize = u64::BITS as usize;

/// Positions of the set bits of one word, ascending.
struct SetBits(u64);

impl Iterator for SetBits {
    type Item = usize;

    fn next(&mut self) -> Option<usize> {
        if self.0 == 0 {
            return None;
        }
        let bit = self.0.trailing_zeros() as usize;
        self.0 &= self.0 - 1;
        Some(bit)
    }
}

/// A word's left and right neighbors along its row, with the carries from
/// the words beside it.
#[inline]
fn shifted(row: &[u64], i: usize) -> (u64, u64) {
    let from_prev = if i > 0 {
        row[i - 1] >> (WORD_BITS - 1)
    } else {
        0
    };
    let from_next = if i + 1 < row.len() {
        row[i + 1] << (WORD_BITS - 1)
    } else {
        0
    };
    ((row[i] << 1) | from_prev, (row[i] >> 1) | from_next)
}

/// Dense bit grid over a box, one word per WORD_BITS columns. Everything
/// outside the grid is empty.
#[derive(Clone)]
pub struct BitGrid {
    width: usize,
    height: usize,
    words: usize,
    bits: Vec<u64>,
}

impl BitGrid {
    pub fn new(width: usize, height: usize) -> Self {
        let words = width.div_ceil(WORD_BITS);
        Self {
            width,
            height,
            words,
            bits: vec![0; words * height],
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn set(&mut self, x: usize, y: usize) {
        self.bits[y * self.words + x / WORD_BITS] |= 1u64 << (x % WORD_BITS);
    }

    pub fn get(&self, x: usize, y: usize) -> bool {
        self.bits[y * self.words + x / WORD_BITS] & (1u64 << (x % WORD_BITS)) != 0
    }

    fn row(&self, y: usize) -> &[u64] {
        &self.bits[y * self.words..(y + 1) * self.words]
    }

    /// Keeps the bits past the width in a row's last word clear.
    fn tail_mask(&self) -> u64 {
        if self.width.is_multiple_of(WORD_BITS) {
            u64::MAX
        } else {
            (1u64 << (self.width % WORD_BITS)) - 1
        }
    }

    /// Grow the set by one cell in the four directions, an L1 dilation step.
    pub fn dilate(&mut self, scratch: &mut BitGrid) {
        self.step(scratch, |a, b| a | b);
    }

    /// Shrink the set by one cell in the four directions, an L1 erosion step.
    pub fn erode(&mut self, scratch: &mut BitGrid) {
        self.step(scratch, |a, b| a & b);
    }

    fn step(&mut self, scratch: &mut BitGrid, combine: impl Fn(u64, u64) -> u64) {
        let (words, height) = (self.words, self.height);
        let tail_mask = self.tail_mask();
        for y in 0..height {
            let row = self.row(y);
            let up = (y > 0).then(|| self.row(y - 1));
            let down = (y + 1 < height).then(|| self.row(y + 1));
            for i in 0..words {
                let (left, right) = shifted(row, i);
                let mut v = combine(combine(row[i], left), right);
                v = combine(v, up.map_or(0, |r| r[i]));
                v = combine(v, down.map_or(0, |r| r[i]));
                if i + 1 == words {
                    v &= tail_mask;
                }
                scratch.bits[y * words + i] = v;
            }
        }
        std::mem::swap(&mut self.bits, &mut scratch.bits);
    }

    /// Every cell within `r` of a set cell along both axes, a square
    /// dilation. `rows` is the inclusive row range holding the set cells.
    pub fn square_dilated(&self, r: usize, (lo, hi): (usize, usize)) -> BitGrid {
        let words = self.words;
        let tail_mask = self.tail_mask();
        let mut out = self.clone();
        let mut row = vec![0u64; words];
        let mut next = vec![0u64; words];
        for y in lo..=hi {
            row.copy_from_slice(self.row(y));
            for _ in 0..r {
                for i in 0..words {
                    let (left, right) = shifted(&row, i);
                    next[i] = row[i] | left | right;
                }
                if let Some(last) = next.last_mut() {
                    *last &= tail_mask;
                }
                std::mem::swap(&mut row, &mut next);
            }
            out.bits[y * words..(y + 1) * words].copy_from_slice(&row);
        }
        let last_row = self.height.saturating_sub(1);
        let widened = out.bits.clone();
        for y in lo.saturating_sub(r)..=(hi + r).min(last_row) {
            let dst = &mut out.bits[y * words..(y + 1) * words];
            for sy in y.saturating_sub(r)..=(y + r).min(last_row) {
                if sy == y {
                    continue;
                }
                let src = &widened[sy * words..(sy + 1) * words];
                for (d, &word) in dst.iter_mut().zip(src) {
                    *d |= word;
                }
            }
        }
        out
    }

    /// The set columns of one row, ascending.
    pub fn row_cells(&self, y: usize) -> impl Iterator<Item = usize> + '_ {
        self.row(y)
            .iter()
            .enumerate()
            .flat_map(|(i, &word)| SetBits(word).map(move |bit| i * WORD_BITS + bit))
    }

    /// The first and last set column of one row, None when it is empty.
    pub fn row_span(&self, y: usize) -> Option<(usize, usize)> {
        let row = self.row(y);
        let first = row.iter().position(|&word| word != 0)?;
        let last = row.iter().rposition(|&word| word != 0)?;
        Some((
            first * WORD_BITS + row[first].trailing_zeros() as usize,
            last * WORD_BITS + WORD_BITS - 1 - row[last].leading_zeros() as usize,
        ))
    }

    pub fn set_cells(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        (0..self.height).flat_map(move |y| self.row_cells(y).map(move |x| (x, y)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two-pass L1 distance transform, the reference the bit grid must match.
    fn chamfer(dist: &mut [u16], w: usize, h: usize, edge: u16) {
        for y in 0..h {
            for x in 0..w {
                let left = if x > 0 { dist[y * w + x - 1] } else { edge };
                let up = if y > 0 { dist[(y - 1) * w + x] } else { edge };
                let best = left.min(up).saturating_add(1);
                dist[y * w + x] = dist[y * w + x].min(best);
            }
        }
        for y in (0..h).rev() {
            for x in (0..w).rev() {
                let right = if x + 1 < w { dist[y * w + x + 1] } else { edge };
                let down = if y + 1 < h {
                    dist[(y + 1) * w + x]
                } else {
                    edge
                };
                let best = right.min(down).saturating_add(1);
                dist[y * w + x] = dist[y * w + x].min(best);
            }
        }
    }

    /// L1 closing by r through the distance transform: dilate against an
    /// empty border, then erode against a source border.
    fn chamfer_closing(set: &[(usize, usize)], w: usize, h: usize, r: u16) -> Vec<(usize, usize)> {
        const INF: u16 = u16::MAX - 1;
        let mut dist = vec![INF; w * h];
        for &(x, y) in set {
            dist[y * w + x] = 0;
        }
        chamfer(&mut dist, w, h, INF);
        for v in dist.iter_mut() {
            *v = if *v <= r { INF } else { 0 };
        }
        chamfer(&mut dist, w, h, 0);
        (0..h)
            .flat_map(|y| (0..w).map(move |x| (x, y)))
            .filter(|&(x, y)| dist[y * w + x] > r)
            .collect()
    }

    #[test]
    fn bit_grid_closing_matches_the_distance_transform() {
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for case in 0..40 {
            let (w, h) = (3 + (next() % 150) as usize, 3 + (next() % 40) as usize);
            let r = 1 + (case % 5) as u32;
            let fill = 10 + next() % 60;
            let set: Vec<(usize, usize)> = (0..h)
                .flat_map(|y| (0..w).map(move |x| (x, y)))
                .filter(|_| next() % 100 < fill)
                .collect();
            let mut grid = BitGrid::new(w, h);
            for &(x, y) in &set {
                grid.set(x, y);
            }
            let mut scratch = BitGrid::new(w, h);
            for _ in 0..r {
                grid.dilate(&mut scratch);
            }
            for _ in 0..r {
                grid.erode(&mut scratch);
            }
            let mut got: Vec<(usize, usize)> = grid.set_cells().collect();
            got.sort_unstable();
            let mut want = chamfer_closing(&set, w, h, r as u16);
            want.sort_unstable();
            assert_eq!(got, want, "case {case}: {w}x{h} r={r}");
        }
    }

    #[test]
    fn square_dilation_matches_brute_force() {
        let mut seed = 0x2545f4914f6cdd1du64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for case in 0..40 {
            let (w, h) = (1 + (next() % 150) as usize, 1 + (next() % 40) as usize);
            let r = case % 4;
            let mut grid = BitGrid::new(w, h);
            let mut set: Vec<(usize, usize)> = vec![(0, 0), (w - 1, h - 1), (w.min(64) - 1, 0)];
            if w > 64 {
                set.push((64, h - 1));
            }
            for _ in 0..(next() % 12) {
                set.push(((next() % w as u64) as usize, (next() % h as u64) as usize));
            }
            for &(x, y) in &set {
                grid.set(x, y);
            }
            let (lo, hi) = set
                .iter()
                .fold((h, 0), |(lo, hi), &(_, y)| (lo.min(y), hi.max(y)));
            let mut got: Vec<(usize, usize)> =
                grid.square_dilated(r, (lo, hi)).set_cells().collect();
            got.sort_unstable();
            let mut want: Vec<(usize, usize)> = (0..h)
                .flat_map(|y| (0..w).map(move |x| (x, y)))
                .filter(|&(x, y)| {
                    set.iter()
                        .any(|&(sx, sy)| x.abs_diff(sx) <= r && y.abs_diff(sy) <= r)
                })
                .collect();
            want.sort_unstable();
            assert_eq!(got, want, "case {case}: {w}x{h} r={r}");
        }
    }
}
