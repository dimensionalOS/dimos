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

// so3_math.h and the MTK::SO3 helpers Point-LIO calls; rotations are plain 3x3 matrices.
use nalgebra::{Matrix3, Vector3};

pub fn hat(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0)
}

/// Rodrigues exp map (`Exp(vec)` / `SO3::exp`), identity below 1e-7 rad.
pub fn exp(ang: &Vector3<f64>) -> Matrix3<f64> {
    let ang_norm = norm3(ang);
    if ang_norm > 0.0000001 {
        rodrigues(&hat(&(ang / ang_norm)), ang_norm)
    } else {
        Matrix3::identity()
    }
}

/// `Exp(v1, v2, v3)`: same map with the looser 1e-5 small-angle cutoff.
pub fn exp_xyz(v1: f64, v2: f64, v3: f64) -> Matrix3<f64> {
    let norm = (v1 * v1 + v2 * v2 + v3 * v3).sqrt();
    if norm > 0.00001 {
        rodrigues(&hat(&Vector3::new(v1 / norm, v2 / norm, v3 / norm)), norm)
    } else {
        Matrix3::identity()
    }
}

/// Log map; trace summed as Eigen does (`r00 + (r11 + r22)`).
pub fn log(r: &Matrix3<f64>) -> Vector3<f64> {
    let trace = r[(0, 0)] + (r[(1, 1)] + r[(2, 2)]);
    let theta = if trace > 3.0 - 1e-6 {
        0.0
    } else {
        (0.5 * (trace - 1.0)).acos()
    };
    let k = Vector3::new(
        r[(2, 1)] - r[(1, 2)],
        r[(0, 2)] - r[(2, 0)],
        r[(1, 0)] - r[(0, 1)],
    );
    if theta.abs() < 0.001 {
        0.5 * k
    } else {
        (0.5 * theta / theta.sin()) * k
    }
}

/// `Vector3d::norm()`: Eigen's fixed-size redux sums as `a0 + (a1 + a2)`.
pub fn norm3(v: &Vector3<f64>) -> f64 {
    (v[0] * v[0] + (v[1] * v[1] + v[2] * v[2])).sqrt()
}

/// `MTK::A_matrix` (the left Jacobian of SO(3)), identity below 1e-11.
pub fn a_matrix(v: &Vector3<f64>) -> Matrix3<f64> {
    let sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    let norm = sq.sqrt();
    if norm < 1e-11 {
        return Matrix3::identity();
    }
    let h = hat(v);
    let c1 = (1.0 - norm.cos()) / sq;
    let c2 = (1.0 - norm.sin() / norm) / sq;
    let hh = mul3(&(c2 * h), &h);
    let mut r = Matrix3::identity();
    for i in 0..3 {
        for j in 0..3 {
            r[(i, j)] = r[(i, j)] + c1 * h[(i, j)] + hh[(i, j)];
        }
    }
    r
}

/// Roll/pitch/yaw of a rotation matrix (`SO3ToEuler` / `RotMtoEuler`).
pub fn so3_to_euler(r: &Matrix3<f64>) -> Vector3<f64> {
    let sy = (r[(0, 0)] * r[(0, 0)] + r[(1, 0)] * r[(1, 0)]).sqrt();
    if sy < 1e-6 {
        Vector3::new((-r[(1, 2)]).atan2(r[(1, 1)]), (-r[(2, 0)]).atan2(sy), 0.0)
    } else {
        Vector3::new(
            r[(2, 1)].atan2(r[(2, 2)]),
            (-r[(2, 0)]).atan2(sy),
            r[(1, 0)].atan2(r[(0, 0)]),
        )
    }
}

// Eye3 + sin*K + ((1-cos)*K)*K, coefficient-wise in that order.
fn rodrigues(k: &Matrix3<f64>, ang: f64) -> Matrix3<f64> {
    let s = ang.sin();
    let kk = mul3(&((1.0 - ang.cos()) * k), k);
    let mut r = Matrix3::identity();
    for i in 0..3 {
        for j in 0..3 {
            r[(i, j)] = r[(i, j)] + s * k[(i, j)] + kk[(i, j)];
        }
    }
    r
}

// Explicit product so the accumulation order is fixed, not nalgebra's.
fn mul3(a: &Matrix3<f64>, b: &Matrix3<f64>) -> Matrix3<f64> {
    let mut r = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..3 {
            r[(i, j)] = a[(i, 0)] * b[(0, j)] + a[(i, 1)] * b[(1, j)] + a[(i, 2)] * b[(2, j)];
        }
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exp_log_roundtrip() {
        let v = Vector3::new(0.3, -0.7, 1.1);
        let r = exp(&v);
        assert!((r.transpose() * r - Matrix3::identity()).norm() < 1e-12);
        assert!((log(&r) - v).norm() < 1e-12);
        assert!((exp_xyz(v[0], v[1], v[2]) - r).norm() < 1e-15);
        assert!((exp(&Vector3::new(1e-8, 0.0, 0.0)) - Matrix3::identity()).norm() == 0.0);
        assert!((exp(&Vector3::zeros()) - a_matrix(&Vector3::zeros())).norm() == 0.0);
        // A(v) is the left Jacobian: exp(v + d) ~ exp(A(v) d) exp(v).
        let d = Vector3::new(1e-6, -2e-6, 3e-6);
        let lhs = exp(&(v + d));
        let rhs = exp(&(a_matrix(&v) * d)) * exp(&v);
        assert!((lhs - rhs).norm() < 1e-10);
        let e = so3_to_euler(&r);
        let back = exp(&Vector3::new(0.0, 0.0, e[2]))
            * exp(&Vector3::new(0.0, e[1], 0.0))
            * exp(&Vector3::new(e[0], 0.0, 0.0));
        assert!((back - r).norm() < 1e-12);
    }
}
