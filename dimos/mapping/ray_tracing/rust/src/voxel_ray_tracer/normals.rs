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
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;

use super::{ChunkMap, VoxelKey, VoxelMap};

pub(super) const NORMAL_MIN_POINTS: u32 = 3;
const NORMAL_NEIGHBOR_RADIUS: i32 = 1;
const NEIGHBORHOOD_CAP: usize = (2 * NORMAL_NEIGHBOR_RADIUS as usize + 1).pow(3);
const NORMAL_REWEIGHT_ITERS: u32 = 3;
/// Neighbor weight falloff with plane distance, as a fraction of voxel size.
const NORMAL_PLANE_SIGMA_FRAC: f32 = 0.5;
/// Fraction of points that must survive the IRLS to count as a real plane.
const NORMAL_MIN_SUPPORT: f32 = 0.5;
/// Eigenvalue spread below this fraction of the matrix scale is isotropic.
const EIGEN_ISOTROPIC_TOL: f64 = 1e-12;
/// Null-space cross product below this fraction of the squared row scale
/// means a repeated eigenvalue.
const EIGEN_NULL_TOL: f64 = 1e-18;
/// Largest eigenvalue below this is a degenerate fit with no plane.
const EIGEN_DEGENERATE: f32 = 1e-12;

/// A voxel's cached pooled fit. Stale until a clearing ray needs it.
#[derive(Clone, Copy, Debug)]
pub(super) enum NormalFit {
    Stale,
    Fitted(Option<Vector3<f32>>),
}

/// The surface normal of a covariance, or None unless it is clearly planar.
#[cfg(test)]
pub(super) fn fit_normal(cov: Matrix3<f32>) -> Option<(Vector3<f32>, f32)> {
    classify(&sym3_eigen(&cov))
}

/// Eigenvalues of a symmetric 3x3, ascending, and the eigenvector of the smallest.
struct Sym3Eigen {
    values: [f32; 3],
    smallest: Vector3<f32>,
}

/// Closed-form eigendecomposition of a symmetric 3x3, computed in f64. An
/// iterative solver is too slow for the per-voxel fit.
fn sym3_eigen(m: &Matrix3<f32>) -> Sym3Eigen {
    let at = |r: usize, c: usize| 0.5 * (m[(r, c)] as f64 + m[(c, r)] as f64);
    let (a, b, c) = (at(0, 0), at(1, 1), at(2, 2));
    let (d, e, f) = (at(0, 1), at(1, 2), at(0, 2));
    let q = (a + b + c) / 3.0;
    let off = d * d + e * e + f * f;
    let spread = (a - q).powi(2) + (b - q).powi(2) + (c - q).powi(2) + 2.0 * off;
    let scale = a.abs().max(b.abs()).max(c.abs()).max(off.sqrt());
    if spread <= (EIGEN_ISOTROPIC_TOL * scale).powi(2) {
        // Isotropic or zero. Every direction is an eigenvector.
        return Sym3Eigen {
            values: [q as f32; 3],
            smallest: Vector3::z(),
        };
    }
    let p = (spread / 6.0).sqrt();
    let (ba, bb, bc) = ((a - q) / p, (b - q) / p, (c - q) / p);
    let (bd, be, bf) = (d / p, e / p, f / p);
    let det = ba * (bb * bc - be * be) - bd * (bd * bc - be * bf) + bf * (bd * be - bb * bf);
    let phi = (det / 2.0).clamp(-1.0, 1.0).acos() / 3.0;
    let largest = q + 2.0 * p * phi.cos();
    let smallest = q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos();
    let middle = 3.0 * q - largest - smallest;
    let rows = |lambda: f64| [[a - lambda, d, f], [d, b - lambda, e], [f, e, c - lambda]];
    Sym3Eigen {
        values: [smallest as f32, middle as f32, largest as f32],
        smallest: null_vector(rows(smallest))
            .or_else(|| null_vector(rows(largest)).map(any_perpendicular))
            .unwrap_or_else(Vector3::z),
    }
}

/// Unit vector spanning the null space of a rank-2 symmetric matrix.
fn null_vector(rows: [[f64; 3]; 3]) -> Option<Vector3<f32>> {
    let cross = |u: [f64; 3], v: [f64; 3]| {
        [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ]
    };
    let norm2 = |v: [f64; 3]| v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    let best = [
        cross(rows[0], rows[1]),
        cross(rows[0], rows[2]),
        cross(rows[1], rows[2]),
    ]
    .into_iter()
    .max_by(|x, y| norm2(*x).total_cmp(&norm2(*y)))?;
    let row_scale = rows.iter().map(|r| norm2(*r)).fold(0.0, f64::max);
    let n2 = norm2(best);
    if n2 <= EIGEN_NULL_TOL * row_scale * row_scale {
        return None;
    }
    let n = n2.sqrt();
    Some(Vector3::new(
        (best[0] / n) as f32,
        (best[1] / n) as f32,
        (best[2] / n) as f32,
    ))
}

/// Some unit vector perpendicular to `v`, for the repeated-smallest-eigenvalue case.
fn any_perpendicular(v: Vector3<f32>) -> Vector3<f32> {
    // Cross with whichever axis is further from v.
    let axis = if v.x.abs() < v.y.abs() {
        Vector3::x()
    } else {
        Vector3::y()
    };
    v.cross(&axis).normalize()
}

/// fit_normal on an already-computed eigendecomposition. Pairs the normal
/// with the smallest eigenvalue, the fit's out-of-plane variance.
fn classify(eig: &Sym3Eigen) -> Option<(Vector3<f32>, f32)> {
    let e2 = eig.values[2].max(0.0);
    if e2 < EIGEN_DEGENERATE {
        return None;
    }
    let e0 = eig.values[0].max(0.0);
    let l0 = e0.sqrt();
    let l1 = eig.values[1].max(0.0).sqrt();
    let l2 = e2.sqrt();
    let linearity = (l2 - l1) / l2;
    let planarity = (l1 - l0) / l2;
    let scattering = l0 / l2;
    if planarity < linearity || planarity < scattering {
        return None;
    }
    Some((eig.smallest, e0))
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
    voxels: &ChunkMap,
    key: VoxelKey,
    voxel_size: f32,
) -> Option<(Vector3<f32>, f32)> {
    let r = NORMAL_NEIGHBOR_RADIUS;
    let mut nbs: ArrayVec<Neighbor, NEIGHBORHOOD_CAP> = ArrayVec::new();
    let mut n_raw: u32 = 0;
    let near = voxels.neighborhood(key, r);
    for dx in -r..=r {
        for dy in -r..=r {
            for dz in -r..=r {
                let Some(v) = near.get((key.0 + dx, key.1 + dy, key.2 + dz)) else {
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
    let mut last_eig: Option<Sym3Eigen> = None;
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
        let eig = sym3_eigen(&cov);
        let normal = eig.smallest;
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

/// Mark stale every voxel whose neighborhood changed materially this frame:
/// refit-milestone crossings and removals, dilated by the pooled-fit radius.
pub(super) fn mark_stale(map: &mut VoxelMap, changed: &AHashSet<VoxelKey>, removed: &[VoxelKey]) {
    let r = NORMAL_NEIGHBOR_RADIUS;
    for &c in changed.iter().chain(removed.iter()) {
        map.voxels
            .for_each_near_mut(c, r, |_, v, _| v.normal = NormalFit::Stale);
    }
}

/// Fit each stale voxel the rays crossed once, in parallel, then settle the
/// spare decisions that waited on those fits.
pub(super) fn resolve_deferred(
    map: &mut VoxelMap,
    misses: &mut AHashSet<VoxelKey>,
    deferred: &[(VoxelKey, Vector3<f32>)],
    voxel_size: f32,
    graze_cos: f32,
) {
    let stale: AHashSet<VoxelKey> = deferred.iter().map(|&(k, _)| k).collect();
    let fits: AHashMap<VoxelKey, Option<Vector3<f32>>> = stale
        .par_iter()
        .map(|&k| (k, pooled_normal(&map.voxels, k, voxel_size).map(|(n, _)| n)))
        .collect::<Vec<_>>()
        .into_iter()
        .collect();
    for &(k, ray_unit) in deferred {
        if !should_spare(fits[&k], ray_unit, graze_cos) {
            misses.insert(k);
        }
    }
    for (k, n) in fits {
        if let Some(v) = map.voxels.get_mut(&k) {
            v.normal = NormalFit::Fitted(n);
        }
    }
}

/// Spare a clearing miss when a grazing ray skims a planar surface.
pub(super) fn should_spare(
    normal: Option<Vector3<f32>>,
    ray_unit: Vector3<f32>,
    graze_cos: f32,
) -> bool {
    normal.is_some_and(|n| ray_unit.dot(&n).abs() < graze_cos)
}

#[cfg(test)]
mod sym3_tests {
    use super::*;

    /// Deterministic pseudo-random floats, so the comparison is reproducible.
    fn lcg(state: &mut u64) -> f32 {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*state >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
    }

    #[test]
    fn closed_form_matches_nalgebra_on_planar_and_random_covariances() {
        let mut state = 7u64;
        for case in 0..20_000 {
            // Even cases are flat voxel-sized planes with thin noise. Odd
            // cases are arbitrary SPD matrices.
            let spread = if case % 2 == 0 {
                Vector3::new(2.5e-3, 1.5e-3, 1e-6)
            } else {
                Vector3::new(1e-3, 1e-3, 1e-3)
            };
            let axes = Matrix3::from_fn(|_, _| lcg(&mut state)).qr().q();
            let jitter = Vector3::new(
                lcg(&mut state).abs(),
                lcg(&mut state).abs(),
                lcg(&mut state).abs(),
            );
            let cov = axes
                * Matrix3::from_diagonal(&spread.component_mul(&(jitter + Vector3::repeat(0.1))))
                * axes.transpose();

            let ours = sym3_eigen(&cov);
            let reference = cov.symmetric_eigen();
            let mut expected: Vec<f32> = reference.eigenvalues.iter().copied().collect();
            expected.sort_by(f32::total_cmp);
            let scale = expected[2].abs().max(1e-12);
            for (got, want) in ours.values.iter().zip(&expected) {
                assert!(
                    (got - want).abs() <= 1e-4 * scale,
                    "case {case}: {got} vs {want}"
                );
            }
            // The smallest eigenvector is only unique when the smallest eigenvalue is separated.
            if expected[1] - expected[0] > 1e-2 * scale {
                let smallest = reference
                    .eigenvalues
                    .iter()
                    .enumerate()
                    .min_by(|a, b| a.1.total_cmp(b.1))
                    .unwrap()
                    .0;
                let want = reference.eigenvectors.column(smallest);
                assert!(
                    ours.smallest.dot(&want).abs() > 0.9999,
                    "case {case}: normal disagrees"
                );
            }
        }
    }

    #[test]
    fn isotropic_and_zero_covariances_do_not_panic() {
        assert_eq!(sym3_eigen(&Matrix3::zeros()).values, [0.0; 3]);
        let iso = sym3_eigen(&(Matrix3::identity() * 2.0));
        assert!(iso.values.iter().all(|v| (v - 2.0).abs() < 1e-6));
        assert!((iso.smallest.norm() - 1.0).abs() < 1e-6);
    }

    /// A repeated smallest eigenvalue has no unique eigenvector. Any unit
    /// vector in its plane, so perpendicular to the largest's, will do.
    #[test]
    fn repeated_smallest_eigenvalue_yields_a_vector_in_its_plane() {
        let eig = sym3_eigen(&Matrix3::from_diagonal(&Vector3::new(2.0, 2.0, 3.0)));
        assert!((eig.values[0] - 2.0).abs() < 1e-6);
        assert!((eig.values[2] - 3.0).abs() < 1e-6);
        assert!((eig.smallest.norm() - 1.0).abs() < 1e-6);
        assert!(eig.smallest.z.abs() < 1e-6, "{:?}", eig.smallest);
    }
}
