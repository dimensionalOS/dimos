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

//! Small dense linear algebra for the PGO core: 3x3 rotation / vector
//! helpers (the Eigen `M3D`/`V3D` operations simple_pgo.cpp uses) and a
//! cyclic-Jacobi symmetric eigensolver (stand-in for Eigen's
//! `SelfAdjointEigenSolver`, also used for Horn's 4x4 quaternion matrix in
//! the ICP transformation estimation).

pub type Mat3 = [[f64; 3]; 3];
pub type Vec3 = [f64; 3];

pub fn identity() -> Mat3 {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

pub fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for (k, b_row) in b.iter().enumerate() {
                out[i][j] += a[i][k] * b_row[j];
            }
        }
    }
    out
}

pub fn transpose(a: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[j][i];
        }
    }
    out
}

pub fn mat_vec(a: &Mat3, v: &Vec3) -> Vec3 {
    let mut out = [0.0; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i] += a[i][j] * v[j];
        }
    }
    out
}

pub fn add(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn norm(v: &Vec3) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Rotation about +z by `yaw` radians (Eigen `AngleAxisd(yaw, UnitZ())`).
pub fn rot_z(yaw: f64) -> Mat3 {
    let (s, c) = yaw.sin_cos();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

/// Rotation about +y by `pitch` radians (Eigen `AngleAxisd(pitch, UnitY())`).
pub fn rot_y(pitch: f64) -> Mat3 {
    let (s, c) = pitch.sin_cos();
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}

/// Rotation matrix -> quaternion `[w, x, y, z]` (Shepperd's method, same
/// branch structure as Eigen's converting constructor).
pub fn quat_from_mat(m: &Mat3) -> [f64; 4] {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            0.25 * s,
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
        ]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [
            (m[2][1] - m[1][2]) / s,
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
        ]
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [
            (m[0][2] - m[2][0]) / s,
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
        ]
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        [
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
        ]
    }
}

/// Quaternion `[w, x, y, z]` -> rotation matrix.
pub fn mat_from_quat(q: &[f64; 4]) -> Mat3 {
    let [w, x, y, z] = *q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// Eigen `Quaterniond(r1).angularDistance(Quaterniond(r2))`:
/// `2 * atan2(|vec(q1 * conj(q2))|, |w(q1 * conj(q2))|)` — the geodesic
/// rotation angle between the two orientations, in radians.
pub fn angular_distance(r1: &Mat3, r2: &Mat3) -> f64 {
    let q1 = quat_from_mat(r1);
    let q2 = quat_from_mat(r2);
    // d = q1 * conj(q2)
    let (w1, x1, y1, z1) = (q1[0], q1[1], q1[2], q1[3]);
    let (w2, x2, y2, z2) = (q2[0], -q2[1], -q2[2], -q2[3]);
    let dw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    let dx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    let dy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    let dz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    2.0 * (dx * dx + dy * dy + dz * dz).sqrt().atan2(dw.abs())
}

/// Cyclic-Jacobi eigendecomposition of a symmetric N x N matrix. Returns
/// (eigenvalues ascending, eigenvectors as columns of the returned matrix,
/// i.e. `vectors[row][col]` with column `col` the eigenvector of
/// `values[col]`). Robust for the 3x3/4x4 sizes used here.
#[allow(clippy::needless_range_loop)] // index-pair sweeps read clearer than iterators here
pub fn jacobi_eigen<const N: usize>(mut a: [[f64; N]; N]) -> ([f64; N], [[f64; N]; N]) {
    let mut v = [[0.0; N]; N];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for _sweep in 0..64 {
        let mut off = 0.0;
        for p in 0..N {
            for q in (p + 1)..N {
                off += a[p][q] * a[p][q];
            }
        }
        if off < 1e-30 {
            break;
        }
        for p in 0..(N - 1) {
            for q in (p + 1)..N {
                if a[p][q].abs() < 1e-300 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = if theta >= 0.0 {
                    1.0 / (theta + (theta * theta + 1.0).sqrt())
                } else {
                    -1.0 / (-theta + (theta * theta + 1.0).sqrt())
                };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                let apq = a[p][q];
                a[p][p] -= t * apq;
                a[q][q] += t * apq;
                a[p][q] = 0.0;
                a[q][p] = 0.0;
                for i in 0..N {
                    if i == p || i == q {
                        continue;
                    }
                    let aip = a[i][p];
                    let aiq = a[i][q];
                    a[i][p] = c * aip - s * aiq;
                    a[p][i] = a[i][p];
                    a[i][q] = s * aip + c * aiq;
                    a[q][i] = a[i][q];
                }
                for row in v.iter_mut() {
                    let vip = row[p];
                    let viq = row[q];
                    row[p] = c * vip - s * viq;
                    row[q] = s * vip + c * viq;
                }
            }
        }
    }
    // Sort ascending by eigenvalue, permuting eigenvector columns to match.
    let mut order: [usize; N] = [0; N];
    for (i, slot) in order.iter_mut().enumerate() {
        *slot = i;
    }
    order.sort_by(|&i, &j| {
        a[i][i]
            .partial_cmp(&a[j][j])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut values = [0.0; N];
    let mut vectors = [[0.0; N]; N];
    for (dst, &src) in order.iter().enumerate() {
        values[dst] = a[src][src];
        for row in 0..N {
            vectors[row][dst] = v[row][src];
        }
    }
    (values, vectors)
}
