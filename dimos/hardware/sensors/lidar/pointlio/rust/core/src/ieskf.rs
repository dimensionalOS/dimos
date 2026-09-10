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

//! Iterated error-state KF: the two Point-LIO instantiations of MTK `esekfom::esekf`,
//! hand-written (`state_input` 24-d, `state_output` 30-d). Expression order mirrors the C++.

use nalgebra::{DMatrix, DVector, Dyn, Matrix3, OMatrix, SMatrix, SVector, Vector3, Vector6, U1};

use crate::so3::{a_matrix, exp, log};

pub type V3 = Vector3<f64>;
pub type M3 = Matrix3<f64>;

/// MTK `input_ikfom`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InputIkfom {
    pub acc: V3,
    pub gyro: V3,
}

/// MTK `state_input` (24-d): pos 0, rot 3, offset_R_L_I 6, offset_T_L_I 9, vel 12, bg 15, ba 18, gravity 21.
#[allow(non_snake_case)]
#[derive(Clone, Debug, PartialEq)]
pub struct StateInput {
    pub pos: V3,
    pub rot: M3,
    pub offset_R_L_I: M3,
    pub offset_T_L_I: V3,
    pub vel: V3,
    pub bg: V3,
    pub ba: V3,
    pub gravity: V3,
}

/// MTK `state_output` (30-d): pos 0, rot 3, offset_R_L_I 6, offset_T_L_I 9, vel 12, omg 15, acc 18, gravity 21, bg 24, ba 27.
#[allow(non_snake_case)]
#[derive(Clone, Debug, PartialEq)]
pub struct StateOutput {
    pub pos: V3,
    pub rot: M3,
    pub offset_R_L_I: M3,
    pub offset_T_L_I: V3,
    pub vel: V3,
    pub omg: V3,
    pub acc: V3,
    pub gravity: V3,
    pub bg: V3,
    pub ba: V3,
}

impl Default for StateInput {
    fn default() -> Self {
        Self {
            pos: V3::zeros(),
            rot: M3::identity(),
            offset_R_L_I: M3::identity(),
            offset_T_L_I: V3::zeros(),
            vel: V3::zeros(),
            bg: V3::zeros(),
            ba: V3::zeros(),
            gravity: V3::zeros(),
        }
    }
}

impl Default for StateOutput {
    fn default() -> Self {
        Self {
            pos: V3::zeros(),
            rot: M3::identity(),
            offset_R_L_I: M3::identity(),
            offset_T_L_I: V3::zeros(),
            vel: V3::zeros(),
            omg: V3::zeros(),
            acc: V3::zeros(),
            gravity: V3::zeros(),
            bg: V3::zeros(),
            ba: V3::zeros(),
        }
    }
}

/// What `esekf` needs from an MTK manifold; DIM == DOF for both Point-LIO states.
pub trait Manifold<const N: usize>: Clone {
    /// Start index of every SO3 block (MTK `SO3_state`).
    const SO3_IDX: &'static [usize];
    /// MTK `oplus`: vect `+= scale*v`, SO3 `*= exp(v, scale)`.
    fn oplus(&mut self, vec: &SVector<f64, N>, scale: f64);
    /// MTK `boxminus`: vect `self - other`, SO3 `log(other^T * self)`.
    fn boxminus(&self, other: &Self) -> SVector<f64, N>;
    /// MTK `boxplus` with scale 1 (bit-identical to `oplus(v, 1)`).
    fn boxplus(&mut self, vec: &SVector<f64, N>) {
        self.oplus(vec, 1.0);
    }
}

fn seg<const N: usize>(v: &SVector<f64, N>, i: usize) -> V3 {
    v.fixed_rows::<3>(i).into_owned()
}

fn put<const N: usize>(r: &mut SVector<f64, N>, i: usize, v: &V3) {
    r.fixed_rows_mut::<3>(i).copy_from(v);
}

impl Manifold<24> for StateInput {
    const SO3_IDX: &'static [usize] = &[3, 6];

    fn oplus(&mut self, v: &SVector<f64, 24>, scale: f64) {
        self.pos += scale * seg(v, 0);
        self.rot *= exp(&(seg(v, 3) * scale));
        self.offset_R_L_I *= exp(&(seg(v, 6) * scale));
        self.offset_T_L_I += scale * seg(v, 9);
        self.vel += scale * seg(v, 12);
        self.bg += scale * seg(v, 15);
        self.ba += scale * seg(v, 18);
        self.gravity += scale * seg(v, 21);
    }

    fn boxminus(&self, o: &Self) -> SVector<f64, 24> {
        let mut r = SVector::zeros();
        put(&mut r, 0, &(self.pos - o.pos));
        put(&mut r, 3, &log(&(o.rot.transpose() * self.rot)));
        put(
            &mut r,
            6,
            &log(&(o.offset_R_L_I.transpose() * self.offset_R_L_I)),
        );
        put(&mut r, 9, &(self.offset_T_L_I - o.offset_T_L_I));
        put(&mut r, 12, &(self.vel - o.vel));
        put(&mut r, 15, &(self.bg - o.bg));
        put(&mut r, 18, &(self.ba - o.ba));
        put(&mut r, 21, &(self.gravity - o.gravity));
        r
    }
}

impl Manifold<30> for StateOutput {
    const SO3_IDX: &'static [usize] = &[3, 6];

    fn oplus(&mut self, v: &SVector<f64, 30>, scale: f64) {
        self.pos += scale * seg(v, 0);
        self.rot *= exp(&(seg(v, 3) * scale));
        self.offset_R_L_I *= exp(&(seg(v, 6) * scale));
        self.offset_T_L_I += scale * seg(v, 9);
        self.vel += scale * seg(v, 12);
        self.omg += scale * seg(v, 15);
        self.acc += scale * seg(v, 18);
        self.gravity += scale * seg(v, 21);
        self.bg += scale * seg(v, 24);
        self.ba += scale * seg(v, 27);
    }

    fn boxminus(&self, o: &Self) -> SVector<f64, 30> {
        let mut r = SVector::zeros();
        put(&mut r, 0, &(self.pos - o.pos));
        put(&mut r, 3, &log(&(o.rot.transpose() * self.rot)));
        put(
            &mut r,
            6,
            &log(&(o.offset_R_L_I.transpose() * self.offset_R_L_I)),
        );
        put(&mut r, 9, &(self.offset_T_L_I - o.offset_T_L_I));
        put(&mut r, 12, &(self.vel - o.vel));
        put(&mut r, 15, &(self.omg - o.omg));
        put(&mut r, 18, &(self.acc - o.acc));
        put(&mut r, 21, &(self.gravity - o.gravity));
        put(&mut r, 24, &(self.bg - o.bg));
        put(&mut r, 27, &(self.ba - o.ba));
        r
    }
}

/// `esekfom::dyn_share_modified<double>`.
#[allow(non_snake_case)]
#[derive(Clone, Debug)]
pub struct DynShareModified {
    pub valid: bool,
    pub converge: bool,
    pub M_Noise: f64,
    pub z: DVector<f64>,
    pub h_x: DMatrix<f64>,
    pub z_IMU: Vector6<f64>,
    pub R_IMU: Vector6<f64>,
    pub satu_check: [bool; 6],
}

impl Default for DynShareModified {
    fn default() -> Self {
        Self {
            valid: false,
            converge: false,
            M_Noise: 0.0,
            z: DVector::zeros(0),
            h_x: DMatrix::zeros(0, 0),
            z_IMU: Vector6::zeros(),
            R_IMU: Vector6::zeros(),
            satu_check: [false; 6],
        }
    }
}

pub type ProcessModel<S, const N: usize> = fn(&S, &InputIkfom) -> SVector<f64, N>;
pub type ProcessMatrix<S, const N: usize> = fn(&S, &InputIkfom) -> SMatrix<f64, N, N>;

/// `esekfom::esekf<state, N, input_ikfom>`; `x`/`P` are the C++ `x_`/`P_`, set them directly to inject a state.
#[allow(non_snake_case)]
pub struct Esekf<S, const N: usize> {
    pub x: S,
    pub P: SMatrix<f64, N, N>,
    f: ProcessModel<S, N>,
    f_x: ProcessMatrix<S, N>,
    maximum_iter: usize,
}

impl<S: Manifold<N> + Default, const N: usize> Esekf<S, N> {
    /// `init_dyn_share_modified_2h/3h` minus the h-models (those are passed per update).
    pub fn new(f: ProcessModel<S, N>, f_x: ProcessMatrix<S, N>) -> Self {
        Self {
            x: S::default(),
            P: SMatrix::identity(),
            f,
            f_x,
            maximum_iter: 1,
        }
    }

    /// `predict(dt, Q, i_in, predict_state, prop_cov)`.
    #[allow(non_snake_case)]
    pub fn predict(
        &mut self,
        dt: f64,
        q: &SMatrix<f64, N, N>,
        i_in: &InputIkfom,
        predict_state: bool,
        prop_cov: bool,
    ) {
        if predict_state {
            let f_ = (self.f)(&self.x, i_in);
            self.x.oplus(&f_, dt);
        }
        if prop_cov {
            let f_ = (self.f)(&self.x, i_in);
            let f_x_ = (self.f_x)(&self.x, i_in);
            // vect rows copy through unchanged (idx == dim); SO3 rows get the A-matrix below.
            let mut f_x_final = f_x_;
            let mut F_x1: SMatrix<f64, N, N> = SMatrix::identity();
            for &idx in S::SO3_IDX {
                let mut seg_SO3 = V3::zeros();
                for i in 0..3 {
                    // C++: -1 * f_(dim+i) * dt; negation is exact so this is bit-identical.
                    seg_SO3[i] = -f_[idx + i] * dt;
                }
                F_x1.fixed_view_mut::<3, 3>(idx, idx)
                    .copy_from(&exp(&seg_SO3));
                let res_temp_SO3 = a_matrix(&seg_SO3);
                for i in 0..N {
                    let col = res_temp_SO3 * f_x_.fixed_view::<3, 1>(idx, i);
                    f_x_final.fixed_view_mut::<3, 1>(idx, i).copy_from(&col);
                }
            }
            F_x1 += f_x_final * dt;
            self.P = F_x1 * self.P * F_x1.transpose() + q * (dt * dt);
        }
    }

    /// `update_iterated_dyn_share_modified` with the lidar h-model `h(x, P[0:3,0:3], P[3:6,3:6], dyn_share)`.
    pub fn update_iterated_dyn_share_modified(
        &mut self,
        mut h: impl FnMut(&S, M3, M3, &mut DynShareModified),
    ) -> bool {
        let mut dyn_share = DynShareModified::default();
        for _ in 0..self.maximum_iter {
            dyn_share.valid = true;
            h(
                &self.x,
                self.P.fixed_view::<3, 3>(0, 0).into_owned(),
                self.P.fixed_view::<3, 3>(3, 3).into_owned(),
                &mut dyn_share,
            );
            if !dyn_share.valid {
                return false;
            }
            let z = &dyn_share.z;
            let h_x = &dyn_share.h_x;
            let dof_measurement = h_x.nrows();
            let m_noise = dyn_share.M_Noise;
            let k_: OMatrix<f64, nalgebra::Const<N>, Dyn> = if N > dof_measurement {
                let pht = self.P.fixed_view::<N, 12>(0, 0) * h_x.transpose();
                let mut hpht = h_x * pht.rows(0, 12);
                for m in 0..dof_measurement {
                    hpht[(m, m)] += m_noise;
                }
                pht * hpht.try_inverse().expect("HPHT singular")
            } else {
                let hth = m_noise * h_x.transpose() * h_x;
                let mut p_inv = self.P.try_inverse().expect("P singular");
                let mut p_inv_00 = p_inv.fixed_view_mut::<12, 12>(0, 0);
                p_inv_00 += hth.fixed_view::<12, 12>(0, 0);
                p_inv = p_inv.try_inverse().expect("P_inv singular");
                p_inv.fixed_view::<N, 12>(0, 0) * h_x.transpose() * m_noise
            };
            let dx_: OMatrix<f64, nalgebra::Const<N>, U1> = &k_ * z;
            self.x.boxplus(&dx_);
            self.P -= k_ * h_x * self.P.fixed_view::<12, N>(0, 0);
        }
        true
    }
}

impl Esekf<StateOutput, 30> {
    /// `update_iterated_dyn_share_IMU` with the IMU h-model (columns omg/acc 15.. and bg/ba 24..).
    #[allow(non_snake_case)]
    pub fn update_iterated_dyn_share_IMU(
        &mut self,
        mut h: impl FnMut(&StateOutput, &mut DynShareModified),
    ) {
        let mut dyn_share = DynShareModified::default();
        for _ in 0..self.maximum_iter {
            dyn_share.valid = true;
            h(&self.x, &mut dyn_share);
            let z = dyn_share.z_IMU;
            let mut PHT = SMatrix::<f64, 30, 6>::zeros();
            let mut HP = SMatrix::<f64, 6, 30>::zeros();
            let mut HPHT = SMatrix::<f64, 6, 6>::zeros();
            for l_ in 0..6 {
                if !dyn_share.satu_check[l_] {
                    PHT.set_column(l_, &(self.P.column(15 + l_) + self.P.column(24 + l_)));
                    HP.set_row(l_, &(self.P.row(15 + l_) + self.P.row(24 + l_)));
                }
            }
            for l_ in 0..6 {
                if !dyn_share.satu_check[l_] {
                    HPHT.set_column(l_, &(HP.column(15 + l_) + HP.column(24 + l_)));
                }
                HPHT[(l_, l_)] += dyn_share.R_IMU[l_];
            }
            let K = PHT * HPHT.try_inverse().expect("HPHT singular");
            let dx_ = K * z;
            self.P -= K * HP;
            self.x.boxplus(&dx_);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
    }

    pub fn rand_v3(seed: &mut u64, s: f64) -> V3 {
        V3::new(lcg(seed) * s, lcg(seed) * s, lcg(seed) * s)
    }

    pub fn rand_rot(seed: &mut u64) -> M3 {
        exp(&rand_v3(seed, 2.0))
    }

    pub fn rand_input_state(seed: &mut u64) -> StateInput {
        StateInput {
            pos: rand_v3(seed, 5.0),
            rot: rand_rot(seed),
            offset_R_L_I: rand_rot(seed),
            offset_T_L_I: rand_v3(seed, 0.2),
            vel: rand_v3(seed, 2.0),
            bg: rand_v3(seed, 0.01),
            ba: rand_v3(seed, 0.1),
            gravity: rand_v3(seed, 9.8),
        }
    }

    pub fn rand_output_state(seed: &mut u64) -> StateOutput {
        StateOutput {
            pos: rand_v3(seed, 5.0),
            rot: rand_rot(seed),
            offset_R_L_I: rand_rot(seed),
            offset_T_L_I: rand_v3(seed, 0.2),
            vel: rand_v3(seed, 2.0),
            omg: rand_v3(seed, 1.0),
            acc: rand_v3(seed, 9.8),
            gravity: rand_v3(seed, 9.8),
            bg: rand_v3(seed, 0.01),
            ba: rand_v3(seed, 0.1),
        }
    }

    #[test]
    fn boxplus_boxminus_roundtrip_input() {
        let mut seed = 1;
        for _ in 0..10 {
            let x = rand_input_state(&mut seed);
            let dx = SVector::<f64, 24>::from_fn(|_, _| lcg(&mut seed) * 0.5);
            let mut y = x.clone();
            y.boxplus(&dx);
            assert!((y.boxminus(&x) - dx).norm() < 1e-10);
            let mut z = x.clone();
            z.boxplus(&SVector::zeros());
            assert_eq!(z, x);
        }
    }

    #[test]
    fn boxplus_boxminus_roundtrip_output() {
        let mut seed = 2;
        for _ in 0..10 {
            let x = rand_output_state(&mut seed);
            let dx = SVector::<f64, 30>::from_fn(|_, _| lcg(&mut seed) * 0.5);
            let mut y = x.clone();
            y.boxplus(&dx);
            assert!((y.boxminus(&x) - dx).norm() < 1e-10);
        }
    }
}
