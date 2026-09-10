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

// Point/measurement types and the plane fit from common_lib.h.
use std::collections::VecDeque;

use nalgebra::Vector3;

pub const NUM_MATCH_POINTS: usize = 5;

/// The pcl::PointXYZINormal fields Point-LIO reads; `curvature` is the offset time in ms.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub curvature: f32,
}

/// One IMU sample (acc in g, gyro in rad/s, stamp in s).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    pub stamp: f64,
    pub gyro: Vector3<f64>,
    pub acc: Vector3<f64>,
}

/// One lidar frame plus the IMU samples that cover it.
#[derive(Clone, Debug, Default)]
pub struct MeasureGroup {
    pub lidar_beg_time: f64,
    pub lidar_last_time: f64,
    pub lidar: Vec<PointXYZI>,
    pub imu: VecDeque<ImuSample>,
}

/// Run lengths of points sharing a curvature (time); `[1]` for an empty cloud, like the C++.
pub fn time_compressing(points: &[PointXYZI]) -> Vec<usize> {
    let mut seq = Vec::with_capacity(points.len());
    let mut j = 0;
    for i in 0..points.len().saturating_sub(1) {
        j += 1;
        if points[i + 1].curvature > points[i].curvature {
            seq.push(j);
            j = 0;
        }
    }
    seq.push(j + 1);
    seq
}

/// Least-squares plane `[a b c d]` (unit normal) through the first 5 points; `None` when any
/// of them sits farther than `threshold` from it.
pub fn esti_plane(points: &[PointXYZI], threshold: f32) -> Option<[f32; 4]> {
    let mut a = [[0f32; 5]; 3];
    for (j, p) in points[..NUM_MATCH_POINTS].iter().enumerate() {
        a[0][j] = p.x;
        a[1][j] = p.y;
        a[2][j] = p.z;
    }
    let nv = colpiv_qr_solve(&mut a, [-1.0; 5]);
    let n = (nv[0] * nv[0] + (nv[1] * nv[1] + nv[2] * nv[2])).sqrt();
    let pca = [nv[0] / n, nv[1] / n, nv[2] / n, (1.0f64 / n as f64) as f32];
    for p in &points[..NUM_MATCH_POINTS] {
        if (pca[0] * p.x + pca[1] * p.y + pca[2] * p.z + pca[3]).abs() > threshold {
            return None;
        }
    }
    Some(pca)
}

// Eigen 3.4 ColPivHouseholderQR (computeInPlace + _solve_impl) on a 5x3 float system, `a` is
// column-major. Reductions follow Eigen's SSE order for fixed sizes; dynamic-size ones are
// sequential (assumes a 16-aligned matrix; the packet path is address dependent).
fn colpiv_qr_solve(a: &mut [[f32; 5]; 3], mut c: [f32; 5]) -> [f32; 3] {
    const ROWS: usize = 5;
    const COLS: usize = 3;
    let eps = f32::EPSILON;
    let mut hcoeffs = [0f32; COLS];
    let mut trans = [0usize; COLS];
    let mut updated = [0f32; COLS];
    let mut direct = [0f32; COLS];
    for k in 0..COLS {
        direct[k] = norm5(&a[k]);
        updated[k] = direct[k];
    }
    let max_norm = updated
        .iter()
        .fold(updated[0], |m, &v| if m < v { v } else { m });
    let threshold_helper = abs2(max_norm * eps) / ROWS as f32;
    let norm_downdate_threshold = eps.sqrt();
    let mut nonzero_pivots = COLS;
    for k in 0..COLS {
        let (mut bi, mut bv) = (k, updated[k]);
        for (j, &u) in updated.iter().enumerate().skip(k + 1) {
            if u > bv {
                bv = u;
                bi = j;
            }
        }
        if nonzero_pivots == COLS && abs2(bv) < threshold_helper * (ROWS - k) as f32 {
            nonzero_pivots = k;
        }
        trans[k] = bi;
        if k != bi {
            a.swap(k, bi);
            updated.swap(k, bi);
            direct.swap(k, bi);
        }
        let (tau, beta) = make_householder(&mut a[k][k..]);
        hcoeffs[k] = tau;
        a[k][k] = beta;
        let mut essential = [0f32; ROWS];
        essential[..ROWS - k - 1].copy_from_slice(&a[k][k + 1..]);
        let essential = &essential[..ROWS - k - 1];
        for col in a.iter_mut().skip(k + 1) {
            apply_householder_left(&mut col[k..], essential, tau);
        }
        for (j, col) in a.iter().enumerate().skip(k + 1) {
            if updated[j] != 0.0 {
                let mut temp = col[k].abs() / updated[j];
                temp = (1.0 + temp) * (1.0 - temp);
                if temp < 0.0 {
                    temp = 0.0;
                }
                let temp2 = temp * abs2(updated[j] / direct[j]);
                if temp2 <= norm_downdate_threshold {
                    direct[j] = sq_norm_dyn(&col[k + 1..]).sqrt();
                    updated[j] = direct[j];
                } else {
                    updated[j] *= temp.sqrt();
                }
            }
        }
    }
    let mut perm = [0usize, 1, 2];
    for (k, &t) in trans.iter().enumerate() {
        perm.swap(k, t);
    }
    let mut dst = [0f32; COLS];
    for k in 0..nonzero_pivots {
        apply_householder_left(&mut c[k..], &a[k][k + 1..], hcoeffs[k]);
    }
    for kk in 0..nonzero_pivots {
        let i = nonzero_pivots - kk - 1;
        if c[i] != 0.0 {
            c[i] /= a[i][i];
            for t in 0..i {
                c[t] -= c[i] * a[i][t];
            }
        }
    }
    for i in 0..nonzero_pivots {
        dst[perm[i]] = c[i];
    }
    dst
}

fn abs2(x: f32) -> f32 {
    x * x
}

// Fixed-size 5-vector norm: one SSE packet reduced as (a0+a2)+(a1+a3), then the tail.
fn norm5(v: &[f32; 5]) -> f32 {
    ((v[0] * v[0] + v[2] * v[2]) + (v[1] * v[1] + v[3] * v[3]) + v[4] * v[4]).sqrt()
}

// Dynamic-size reductions (tails, inner products): one SSE packet for length 4, else sequential.
fn sq_norm_dyn(v: &[f32]) -> f32 {
    if v.len() == 4 {
        (v[0] * v[0] + v[2] * v[2]) + (v[1] * v[1] + v[3] * v[3])
    } else {
        v.iter().fold(0.0, |s, &x| s + x * x)
    }
}

fn dot_eigen(a: &[f32], b: &[f32]) -> f32 {
    if a.len() == 4 {
        (a[0] * b[0] + a[2] * b[2]) + (a[1] * b[1] + a[3] * b[3])
    } else {
        a.iter().zip(b).fold(0.0, |s, (&x, &y)| s + x * y)
    }
}

// MatrixBase::makeHouseholderInPlace: returns (tau, beta), leaves the essential part in v[1..].
fn make_householder(v: &mut [f32]) -> (f32, f32) {
    let tail_sq = if v.len() == 1 {
        0.0
    } else {
        sq_norm_dyn(&v[1..])
    };
    let c0 = v[0];
    if tail_sq <= f32::MIN_POSITIVE {
        v[1..].fill(0.0);
        return (0.0, c0);
    }
    let mut beta = (abs2(c0) + tail_sq).sqrt();
    if c0 >= 0.0 {
        beta = -beta;
    }
    for e in &mut v[1..] {
        *e /= c0 - beta;
    }
    ((beta - c0) / beta, beta)
}

// MatrixBase::applyHouseholderOnTheLeft on one column.
fn apply_householder_left(col: &mut [f32], essential: &[f32], tau: f32) {
    if col.len() == 1 {
        col[0] *= 1.0 - tau;
    } else if tau != 0.0 {
        let mut tmp = dot_eigen(essential, &col[1..]);
        tmp += col[0];
        col[0] -= tau * tmp;
        for (b, e) in col[1..].iter_mut().zip(essential) {
            *b -= tau * *e * tmp;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f32, y: f32, z: f32, t: f32) -> PointXYZI {
        PointXYZI {
            x,
            y,
            z,
            intensity: 0.0,
            curvature: t,
        }
    }

    #[test]
    fn plane_fit_and_time_runs() {
        // z = 0.5x + 0.25y + 1  ->  0.5x + 0.25y - z + 1 = 0
        let f = |x: f32, y: f32| 0.5 * x + 0.25 * y + 1.0;
        let pts: Vec<_> = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (2.0, 0.5)]
            .iter()
            .map(|&(x, y)| pt(x, y, f(x, y), 0.0))
            .collect();
        let p = esti_plane(&pts, 0.1).unwrap();
        let s = 1.0 / (0.5f32 * 0.5 + 0.25 * 0.25 + 1.0).sqrt();
        for (got, want) in p.iter().zip([0.5 * s, 0.25 * s, -s, s]) {
            assert!((got - want).abs() < 1e-5, "{p:?}");
        }
        let mut bent = pts.clone();
        bent[4].z += 1.0;
        assert!(esti_plane(&bent, 0.1).is_none());
        assert!(esti_plane(&bent, 10.0).is_some());

        let cloud: Vec<_> = [0.0, 0.0, 1.0, 2.0, 2.0, 2.0]
            .iter()
            .map(|&t| pt(0.0, 0.0, 0.0, t))
            .collect();
        assert_eq!(time_compressing(&cloud), vec![2, 1, 3]);
        assert_eq!(time_compressing(&[]), vec![1]);
    }
}
