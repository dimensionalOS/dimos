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

//! Landmark factors: keyframe -> landmark between factors with a 6x6 information (any dimension may be unknown); a landmark never sighted with a rotation is a 3-entry point in the solve, else a 6-entry pose.

use crate::big_picture_fit::{edge_residual, linearize_between, Edge, State};
use crate::mat3::{self, Mat3, Vec3};

/// Landmark `landmark` (dense index) seen from `keyframe`: body-frame pose and information (`Edge` order; 0 = unknown).
#[derive(Clone, Copy, Debug)]
pub struct LandmarkSighting {
    pub keyframe: usize,
    pub landmark: usize,
    pub rotation: Mat3,
    pub position: Vec3,
    pub information: [[f64; 6]; 6],
}

impl LandmarkSighting {
    /// A sighting of a point: `position` with its 3x3 `covariance` (m²), the rotation unknown.
    pub fn point(keyframe: usize, landmark: usize, position: Vec3, covariance: Mat3) -> Self {
        let mut full = [[0.0; 6]; 6];
        for i in 0..3 {
            for j in 0..3 {
                full[3 + i][3 + j] = covariance[i][j];
            }
        }
        LandmarkSighting {
            keyframe,
            landmark,
            rotation: mat3::identity(),
            position,
            information: known_inverse(&full),
        }
    }
}

/// A sighting as the solve uses it: a keyframe -> landmark edge, its information and how many dims it knows.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Sighting {
    pub(crate) keyframe: usize,
    pub(crate) landmark: usize,
    pub(crate) measured: Edge,
    pub(crate) information: [[f64; 6]; 6],
    pub(crate) known: usize,
}

impl Sighting {
    pub(crate) fn new(s: &LandmarkSighting) -> Self {
        Sighting {
            keyframe: s.keyframe,
            landmark: s.landmark,
            measured: Edge {
                from: s.keyframe,
                to: s.keyframe,
                rotation: s.rotation,
                translation: s.position,
                variances: [1.0; 6],
            },
            information: s.information,
            known: (0..6).filter(|&i| s.information[i][i] > 0.0).count(),
        }
    }

    /// Whether it says anything about the landmark's rotation.
    pub(crate) fn knows_rotation(&self) -> bool {
        self.information[..3].iter().flatten().any(|&v| v != 0.0)
    }

    fn residual(&self, state: &State) -> [f64; 6] {
        let k = self.keyframe;
        edge_residual(
            &self.measured,
            (&state.rotation[k], &state.position[k]),
            (
                &state.landmark_rotation[self.landmark],
                &state.landmark[self.landmark],
            ),
        )
    }

    /// r^T I r.
    pub(crate) fn whitened(&self, state: &State) -> f64 {
        let r = self.residual(state);
        let ir = mat_vec6(&self.information, &r);
        (0..6).map(|i| r[i] * ir[i]).sum()
    }

    /// The residual and its 6 x (6 + dims) Jacobian over [keyframe rotation, position, the landmark's last `dims` entries].
    pub(crate) fn linearize(&self, state: &State, dims: usize) -> ([f64; 6], Vec<[f64; 12]>) {
        let k = self.keyframe;
        let (residual, jf, jt) = linearize_between(
            &self.measured,
            (&state.rotation[k], &state.position[k]),
            (
                &state.landmark_rotation[self.landmark],
                &state.landmark[self.landmark],
            ),
        );
        let jacobian = (0..6)
            .map(|row| {
                let mut out = [0.0; 12];
                out[..6].copy_from_slice(&jf[row]);
                for c in 0..dims {
                    out[6 + c] = jt[row][c + 6 - dims];
                }
                out
            })
            .collect();
        (residual, jacobian)
    }

    /// Fill entries (J column, J column, matrix row, matrix column): keyframe block (if free), landmark block (at `at`), cross.
    pub(crate) fn entries(
        &self,
        slot: &[Option<usize>],
        at: usize,
        dims: usize,
    ) -> impl Iterator<Item = (usize, usize, usize, usize)> {
        let pose = slot[self.keyframe];
        let own = pose.into_iter().flat_map(|a| {
            (0..6).flat_map(move |i| (0..=i).map(move |j| (i, j, 6 * a + i, 6 * a + j)))
        });
        let landmark =
            (0..dims).flat_map(move |i| (0..=i).map(move |j| (6 + i, 6 + j, at + i, at + j)));
        let cross = pose.into_iter().flat_map(move |a| {
            (0..dims).flat_map(move |i| (0..6).map(move |j| (6 + i, j, at + i, 6 * a + j)))
        });
        own.chain(landmark).chain(cross)
    }
}

/// `planar_pose_graph`: a body-frame sighting in the level frame `tilt` turns the body into; its translation (and the translation rows of its information) turn, its rotation turns on the body side (the rotation error is the landmark's).
pub fn leveled(
    tilt: &Mat3,
    rotation: &Mat3,
    position: &Vec3,
    information: &[[f64; 6]; 6],
) -> (Mat3, Vec3, [[f64; 6]; 6]) {
    // information' = B information B^T, B = diag(I, tilt)
    let b = |i: usize, k: usize| match (i < 3, k < 3) {
        (true, true) => (i == k) as u8 as f64,
        (false, false) => tilt[i - 3][k - 3],
        _ => 0.0,
    };
    let turned = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            (0..6)
                .flat_map(|k| (0..6).map(move |l| (k, l)))
                .map(|(k, l)| b(i, k) * information[k][l] * b(j, l))
                .sum()
        })
    });
    (
        mat3::mat_mul(tilt, rotation),
        mat3::mat_vec(tilt, position),
        turned,
    )
}

/// Inverse of `m` over its positive finite diagonal dims, zero elsewhere (ROS -1, inf, 0 = unknown); all zero if singular.
pub fn known_inverse(m: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let known: Vec<usize> = (0..6)
        .filter(|&i| m[i][i].is_finite() && m[i][i] > 0.0)
        .collect();
    let block: Vec<Vec<f64>> = known
        .iter()
        .map(|&i| known.iter().map(|&j| m[i][j]).collect())
        .collect();
    let mut out = [[0.0; 6]; 6];
    if let Some(inverse) = invert(block) {
        for (a, &i) in known.iter().enumerate() {
            for (b, &j) in known.iter().enumerate() {
                out[i][j] = inverse[a][b];
            }
        }
    }
    out
}

pub(crate) fn mat_vec6(m: &[[f64; 6]; 6], v: &[f64; 6]) -> [f64; 6] {
    std::array::from_fn(|i| (0..6).map(|j| m[i][j] * v[j]).sum())
}

/// The inverse of a small symmetric positive definite matrix (Gauss-Jordan with partial pivoting); None if singular.
fn invert(mut a: Vec<Vec<f64>>) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    let scale = (0..n).map(|i| a[i][i].abs()).fold(0.0, f64::max);
    let mut inverse: Vec<Vec<f64>> = (0..n)
        .map(|i| (0..n).map(|j| (i == j) as u8 as f64).collect())
        .collect();
    for col in 0..n {
        let pivot = (col..n).max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))?;
        if a[pivot][col].abs() <= 1e-15 * scale {
            return None;
        }
        a.swap(col, pivot);
        inverse.swap(col, pivot);
        let p = a[col][col];
        for j in 0..n {
            a[col][j] /= p;
            inverse[col][j] /= p;
        }
        for row in 0..n {
            if row != col {
                let f = a[row][col];
                for j in 0..n {
                    a[row][j] -= f * a[col][j];
                    inverse[row][j] -= f * inverse[col][j];
                }
            }
        }
    }
    Some(inverse)
}

/// The chi-square value a `dof`-DoF residual stays under with probability `p`.
pub(crate) fn chi2_inverse(p: f64, dof: usize) -> f64 {
    crate::loop_overlap::chi2_inverse(p, |x| chi2_cdf(x, dof))
}

/// The regularized lower incomplete gamma P(dof / 2, x / 2), by its series.
fn chi2_cdf(x: f64, dof: usize) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    let (a, half) = (dof as f64 / 2.0, x / 2.0);
    // Γ(a + 1) from Γ(1) = 1 or Γ(3/2) = √π / 2
    let mut gamma = if dof.is_multiple_of(2) {
        1.0
    } else {
        std::f64::consts::PI.sqrt() / 2.0
    };
    let mut g = if dof.is_multiple_of(2) { 1.0 } else { 1.5 };
    while g < a + 1.0 - 1e-9 {
        gamma *= g;
        g += 1.0;
    }
    let mut term = (a * half.ln() - half).exp() / gamma;
    let mut sum = term;
    for n in 1..1000 {
        term *= half / (a + n as f64);
        sum += term;
        if term < 1e-15 * sum {
            break;
        }
    }
    sum.min(1.0)
}
