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

//! Neighborhood-pooled surface-normal fitting and its refresh policy.

use ahash::{AHashMap, AHashSet};
use arrayvec::ArrayVec;
use nalgebra::linalg::SymmetricEigen;
use nalgebra::{Matrix3, Vector3, U3};
use rayon::prelude::*;

use super::{Voxel, VoxelKey, VoxelMap};

pub(super) const NORMAL_MIN_POINTS: u32 = 3;
const NORMAL_NEIGHBOR_RADIUS: i32 = 1;
const NEIGHBORHOOD_CAP: usize = (2 * NORMAL_NEIGHBOR_RADIUS as usize + 1).pow(3);
const NORMAL_REWEIGHT_ITERS: u32 = 3;
/// Neighbor weight falloff with plane distance, as a fraction of voxel size.
const NORMAL_PLANE_SIGMA_FRAC: f32 = 0.5;
/// Fraction of points that must survive the IRLS to count as a real plane.
const NORMAL_MIN_SUPPORT: f32 = 0.5;

/// The surface normal of a covariance, or None unless it is clearly planar.
#[cfg(test)]
pub(super) fn fit_normal(cov: Matrix3<f32>) -> Option<(Vector3<f32>, f32)> {
    classify(&cov.symmetric_eigen())
}

/// fit_normal on an already-computed eigendecomposition. Pairs the normal
/// with the smallest eigenvalue, the fit's out-of-plane variance.
fn classify(eig: &SymmetricEigen<f32, U3>) -> Option<(Vector3<f32>, f32)> {
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]));
    let e2 = eig.eigenvalues[idx[2]].max(0.0);
    if e2 < 1e-12 {
        return None;
    }
    let e0 = eig.eigenvalues[idx[0]].max(0.0);
    let l0 = e0.sqrt();
    let l1 = eig.eigenvalues[idx[1]].max(0.0).sqrt();
    let l2 = e2.sqrt();
    let linearity = (l2 - l1) / l2;
    let planarity = (l1 - l0) / l2;
    let scattering = l0 / l2;
    if planarity < linearity || planarity < scattering {
        return None;
    }
    Some((eig.eigenvectors.column(idx[0]).into_owned(), e0))
}

/// Moments of one neighbor voxel: count, sum, sum of outer products, centroid.
struct Neighbor {
    n: f32,
    s: Vector3<f32>,
    t: Matrix3<f32>,
    centroid: Vector3<f32>,
}

/// Fit a voxel's normal from one scan of its neighborhood.
pub(super) fn pooled_normal(
    voxels: &AHashMap<VoxelKey, Voxel>,
    key: VoxelKey,
    voxel_size: f32,
) -> Option<(Vector3<f32>, f32)> {
    let r = NORMAL_NEIGHBOR_RADIUS;
    let mut nbs: ArrayVec<Neighbor, NEIGHBORHOOD_CAP> = ArrayVec::new();
    let mut n_raw: u32 = 0;
    for dx in -r..=r {
        for dy in -r..=r {
            for dz in -r..=r {
                let nk = (key.0 + dx, key.1 + dy, key.2 + dz);
                let Some(v) = voxels.get(&nk) else {
                    continue;
                };
                if v.num_pts == 0 {
                    continue;
                }
                let ni = v.num_pts as f32;
                // Shift this voxel's center-relative moments to the target center.
                let d = Vector3::new(dx as f32, dy as f32, dz as f32) * voxel_size;
                let s = v.sum + d * ni;
                let t =
                    v.m2 + v.sum * d.transpose() + d * v.sum.transpose() + d * d.transpose() * ni;
                n_raw += v.num_pts;
                nbs.push(Neighbor {
                    n: ni,
                    s,
                    t,
                    centroid: s / ni,
                });
            }
        }
    }
    if n_raw < NORMAL_MIN_POINTS {
        return None;
    }

    let sigma = NORMAL_PLANE_SIGMA_FRAC * voxel_size;
    let two_sig2 = 2.0 * sigma * sigma;
    let mut weights = [1.0_f32; NEIGHBORHOOD_CAP];
    // The last iteration's decomposition doubles as the final fit input.
    let mut last_eig: Option<SymmetricEigen<f32, U3>> = None;
    for _ in 0..NORMAL_REWEIGHT_ITERS {
        let (mut wn, mut s, mut t) = (0.0_f32, Vector3::zeros(), Matrix3::zeros());
        for (nb, &w) in nbs.iter().zip(&weights) {
            wn += w * nb.n;
            s += nb.s * w;
            t += nb.t * w;
        }
        if wn < 1e-6 {
            break;
        }
        let mean = s / wn;
        let cov = t / wn - mean * mean.transpose();
        let eig = cov.symmetric_eigen();
        let smallest = eig
            .eigenvalues
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|(i, _)| i)
            .unwrap();
        let normal = eig.eigenvectors.column(smallest).into_owned();
        for (nb, w) in nbs.iter().zip(&mut weights) {
            let dist = normal.dot(&(nb.centroid - mean)).abs();
            *w = (-(dist * dist) / two_sig2).exp();
        }
        last_eig = Some(eig);
    }
    // Reject the plane if too many points had to be discarded to fit it.
    let kept: f32 = nbs.iter().zip(&weights).map(|(nb, &w)| w * nb.n).sum();
    if kept < NORMAL_MIN_SUPPORT * n_raw as f32 {
        return None;
    }
    classify(&last_eig?)
}

/// Refit the cached normal of every voxel whose neighborhood changed
/// materially this frame: refit-milestone crossings and removals, dilated by
/// the pooled-fit radius.
pub(super) fn refresh_voxels(
    map: &mut VoxelMap,
    changed: &AHashSet<VoxelKey>,
    removed: &[VoxelKey],
    voxel_size: f32,
) {
    let r = NORMAL_NEIGHBOR_RADIUS;
    // Sized for the dilation's typical overlap so the serial loop never rehashes.
    let mut dirty: AHashSet<VoxelKey> =
        AHashSet::with_capacity(8 * (changed.len() + removed.len()));
    for &c in changed.iter().chain(removed.iter()) {
        for dx in -r..=r {
            for dy in -r..=r {
                for dz in -r..=r {
                    dirty.insert((c.0 + dx, c.1 + dy, c.2 + dz));
                }
            }
        }
    }
    let updates: Vec<(VoxelKey, Option<Vector3<f32>>)> = dirty
        .par_iter()
        .filter(|k| map.voxels.contains_key(k))
        .map(|&k| (k, pooled_normal(&map.voxels, k, voxel_size).map(|(n, _)| n)))
        .collect();
    for (k, n) in updates {
        if let Some(c) = map.voxels.get_mut(&k) {
            c.normal = n;
        }
    }
}

/// Spare a clearing miss when a grazing ray skims a planar surface.
pub(super) fn should_spare(c: &Voxel, ray_unit: Vector3<f32>, graze_cos: f32) -> bool {
    match c.normal {
        Some(n) => ray_unit.dot(&n).abs() < graze_cos,
        None => false,
    }
}
