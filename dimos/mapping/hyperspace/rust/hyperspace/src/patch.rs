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

//! Per-frame patch embedding grids, stored as f16 and dotted in f32.

use half::f16;
use serde::{Deserialize, Serialize};

/// `rows × cols` L2-normalized patch embeddings of one image, row-major.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PatchGrid {
    pub rows: usize,
    pub cols: usize,
    pub dim: usize,
    pub data: Vec<f16>,
}

impl PatchGrid {
    pub fn from_f32(rows: usize, cols: usize, dim: usize, data: &[f32]) -> Self {
        assert_eq!(data.len(), rows * cols * dim, "patch grid size mismatch");
        PatchGrid {
            rows,
            cols,
            dim,
            data: data.iter().map(|v| f16::from_f32(*v)).collect(),
        }
    }

    pub fn patch_count(&self) -> usize {
        self.rows * self.cols
    }

    pub fn patch(&self, index: usize) -> &[f16] {
        &self.data[index * self.dim..(index + 1) * self.dim]
    }

    /// Dot product of every patch with `query` (which must be `dim` long).
    pub fn scores(&self, query: &[f32]) -> Vec<f32> {
        assert_eq!(query.len(), self.dim);
        (0..self.patch_count())
            .map(|index| dot_f16(self.patch(index), query))
            .collect()
    }

    /// Mean over patches of (1 - cosine) against another grid of the same shape:
    /// 0 for an identical frame, ~1 for unrelated content.
    pub fn distance(&self, other: &PatchGrid) -> f32 {
        assert_eq!(
            (self.rows, self.cols, self.dim),
            (other.rows, other.cols, other.dim)
        );
        let total: f32 = (0..self.patch_count())
            .map(|index| 1.0 - dot_f16_f16(self.patch(index), other.patch(index)))
            .sum();
        total / self.patch_count() as f32
    }

    /// Largest per-patch (1 - cosine) against another grid: catches one new
    /// object entering an otherwise unchanged view.
    pub fn max_patch_distance(&self, other: &PatchGrid) -> f32 {
        (0..self.patch_count())
            .map(|index| 1.0 - dot_f16_f16(self.patch(index), other.patch(index)))
            .fold(0.0, f32::max)
    }
}

pub fn dot_f16(a: &[f16], b: &[f32]) -> f32 {
    a.iter().zip(b).map(|(x, y)| x.to_f32() * y).sum()
}

pub fn dot_f16_f16(a: &[f16], b: &[f16]) -> f32 {
    a.iter().zip(b).map(|(x, y)| x.to_f32() * y.to_f32()).sum()
}

pub fn normalize(vector: &mut [f32]) {
    let norm = vector.iter().map(|v| v * v).sum::<f32>().sqrt();
    if norm > 0.0 {
        for v in vector.iter_mut() {
            *v /= norm;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scores_and_distance() {
        let grid = PatchGrid::from_f32(1, 2, 3, &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
        let scores = grid.scores(&[1.0, 0.0, 0.0]);
        assert!((scores[0] - 1.0).abs() < 1e-3 && scores[1].abs() < 1e-3);
        assert!(grid.distance(&grid) < 1e-3);
        let other = PatchGrid::from_f32(1, 2, 3, &[0.0, 1.0, 0.0, 1.0, 0.0, 0.0]);
        assert!((grid.distance(&other) - 1.0).abs() < 1e-3);
        assert!((grid.max_patch_distance(&other) - 1.0).abs() < 1e-3);
    }
}
