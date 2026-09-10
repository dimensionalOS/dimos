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

//! Port of `Estimator.cpp`: process models, Jacobians, lidar/IMU h-models. The C++ globals the
//! h-models read are an explicit context struct with the same field names.

use nalgebra::{DMatrix, DVector, SMatrix, SVector};

use crate::common::{esti_plane, PointXYZI, NUM_MATCH_POINTS};
use crate::ieskf::{DynShareModified, InputIkfom, StateInput, StateOutput, M3, V3};
use crate::ivox::IVox;
use crate::so3::hat;

pub fn process_noise_cov_input(
    gyr_cov_input: f64,
    acc_cov_input: f64,
    b_gyr_cov: f64,
    b_acc_cov: f64,
) -> SMatrix<f64, 24, 24> {
    let mut cov = SMatrix::zeros();
    for i in 0..3 {
        cov[(3 + i, 3 + i)] = gyr_cov_input;
        cov[(12 + i, 12 + i)] = acc_cov_input;
        cov[(15 + i, 15 + i)] = b_gyr_cov;
        cov[(18 + i, 18 + i)] = b_acc_cov;
    }
    cov
}

pub fn process_noise_cov_output(
    vel_cov: f64,
    gyr_cov_output: f64,
    acc_cov_output: f64,
    b_gyr_cov: f64,
    b_acc_cov: f64,
) -> SMatrix<f64, 30, 30> {
    let mut cov = SMatrix::zeros();
    for i in 0..3 {
        cov[(12 + i, 12 + i)] = vel_cov;
        cov[(15 + i, 15 + i)] = gyr_cov_output;
        cov[(18 + i, 18 + i)] = acc_cov_output;
        cov[(24 + i, 24 + i)] = b_gyr_cov;
        cov[(27 + i, 27 + i)] = b_acc_cov;
    }
    cov
}

/// `reset_cov` (parameters.cpp): initial P for the input filter.
pub fn reset_cov() -> SMatrix<f64, 24, 24> {
    let mut p = SMatrix::identity() * 0.1;
    for i in 0..3 {
        p[(21 + i, 21 + i)] = 0.0001;
    }
    for i in 0..6 {
        p[(15 + i, 15 + i)] = 0.001;
    }
    p
}

/// `reset_cov_output` (parameters.cpp): initial P for the output filter.
pub fn reset_cov_output() -> SMatrix<f64, 30, 30> {
    let mut p = SMatrix::identity() * 0.01;
    for i in 0..3 {
        p[(21 + i, 21 + i)] = 0.0001;
    }
    for i in 0..6 {
        p[(24 + i, 24 + i)] = 0.001;
    }
    p
}

pub fn get_f_input(s: &StateInput, input: &InputIkfom) -> SVector<f64, 24> {
    let mut res = SVector::zeros();
    let omega = input.gyro - s.bg;
    let a_inertial = s.rot * (input.acc - s.ba);
    for i in 0..3 {
        res[i] = s.vel[i];
        res[i + 3] = omega[i];
        res[i + 12] = a_inertial[i] + s.gravity[i];
    }
    res
}

pub fn get_f_output(s: &StateOutput, _input: &InputIkfom) -> SVector<f64, 30> {
    let mut res = SVector::zeros();
    let a_inertial = s.rot * s.acc;
    for i in 0..3 {
        res[i] = s.vel[i];
        res[i + 3] = s.omg[i];
        res[i + 12] = a_inertial[i] + s.gravity[i];
    }
    res
}

pub fn df_dx_input(s: &StateInput, input: &InputIkfom) -> SMatrix<f64, 24, 24> {
    let mut cov = SMatrix::<f64, 24, 24>::zeros();
    cov.fixed_view_mut::<3, 3>(0, 12).copy_from(&M3::identity());
    let acc_ = input.acc - s.ba;
    cov.fixed_view_mut::<3, 3>(12, 3)
        .copy_from(&(-s.rot * hat(&acc_)));
    cov.fixed_view_mut::<3, 3>(12, 18).copy_from(&(-s.rot));
    cov.fixed_view_mut::<3, 3>(12, 21)
        .copy_from(&M3::identity());
    cov.fixed_view_mut::<3, 3>(3, 15)
        .copy_from(&(-M3::identity()));
    cov
}

pub fn df_dx_output(s: &StateOutput, _input: &InputIkfom) -> SMatrix<f64, 30, 30> {
    let mut cov = SMatrix::<f64, 30, 30>::zeros();
    cov.fixed_view_mut::<3, 3>(0, 12).copy_from(&M3::identity());
    cov.fixed_view_mut::<3, 3>(12, 3)
        .copy_from(&(-s.rot * hat(&s.acc)));
    cov.fixed_view_mut::<3, 3>(12, 18).copy_from(&s.rot);
    cov.fixed_view_mut::<3, 3>(12, 21)
        .copy_from(&M3::identity());
    cov.fixed_view_mut::<3, 3>(3, 15).copy_from(&M3::identity());
    cov
}

/// Per-frame buffers + config the lidar h-models read (C++ globals, same names).
#[allow(non_snake_case)]
pub struct HModelCtx<'a> {
    pub feats_down_body: &'a [PointXYZI],
    pub feats_down_world: &'a mut [PointXYZI],
    pub pbody_list: &'a [V3],
    pub crossmat_list: &'a [M3],
    pub Nearest_Points: &'a mut [Vec<PointXYZI>],
    pub point_selected_surf: &'a mut [bool],
    pub normvec: &'a mut Vec<PointXYZI>,
    pub ivox_: &'a mut IVox,
    pub effct_feat_num: &'a mut i32,
    pub time_seq: &'a [usize],
    pub k: usize,
    /// Starts at -1; the h-model touches points `idx+1 ..= idx+time_seq[k]`.
    pub idx: isize,
    pub plane_thr: f32,
    pub match_s: f64,
    pub extrinsic_est_en: bool,
    pub laser_point_cov: f64,
    pub Lidar_R_wrt_IMU: M3,
    pub Lidar_T_wrt_IMU: V3,
}

/// The pose part shared by both states (what `pointBodyToWorld` reads off `kf_*.x_`).
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
pub struct Pose<'a> {
    pub pos: &'a V3,
    pub rot: &'a M3,
    pub offset_R_L_I: &'a M3,
    pub offset_T_L_I: &'a V3,
}

impl<'a> From<&'a StateInput> for Pose<'a> {
    fn from(s: &'a StateInput) -> Self {
        Pose {
            pos: &s.pos,
            rot: &s.rot,
            offset_R_L_I: &s.offset_R_L_I,
            offset_T_L_I: &s.offset_T_L_I,
        }
    }
}

impl<'a> From<&'a StateOutput> for Pose<'a> {
    fn from(s: &'a StateOutput) -> Self {
        Pose {
            pos: &s.pos,
            rot: &s.rot,
            offset_R_L_I: &s.offset_R_L_I,
            offset_T_L_I: &s.offset_T_L_I,
        }
    }
}

/// `pointBodyToWorld`: writes x/y/z/intensity of `po`, other fields untouched.
#[allow(non_snake_case)]
pub fn point_body_to_world(
    s: Pose,
    extrinsic_est_en: bool,
    Lidar_R_wrt_IMU: &M3,
    Lidar_T_wrt_IMU: &V3,
    pi: &PointXYZI,
    po: &mut PointXYZI,
) {
    let p_body = V3::new(pi.x as f64, pi.y as f64, pi.z as f64);
    let p_global = if extrinsic_est_en {
        s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos
    } else {
        s.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + s.pos
    };
    po.x = p_global[0] as f32;
    po.y = p_global[1] as f32;
    po.z = p_global[2] as f32;
    po.intensity = pi.intensity;
}

pub fn h_model_input(
    s: &StateInput,
    cov_p: M3,
    cov_r: M3,
    ctx: &mut HModelCtx,
    ekfom_data: &mut DynShareModified,
) {
    h_model_lidar(s.into(), cov_p, cov_r, ctx, ekfom_data)
}

pub fn h_model_output(
    s: &StateOutput,
    cov_p: M3,
    cov_r: M3,
    ctx: &mut HModelCtx,
    ekfom_data: &mut DynShareModified,
) {
    h_model_lidar(s.into(), cov_p, cov_r, ctx, ekfom_data)
}

/// Shared body of `h_model_input` / `h_model_output` (identical in the C++). cov_p/cov_R are unused there too.
fn h_model_lidar(
    s: Pose,
    _cov_p: M3,
    _cov_r: M3,
    ctx: &mut HModelCtx,
    ekfom_data: &mut DynShareModified,
) {
    let ts = ctx.time_seq[ctx.k];
    ctx.normvec.resize(ts, PointXYZI::default());
    let base = (ctx.idx + 1) as usize;
    let mut effect_num_k = 0usize;
    for j in 0..ts {
        let i = base + j;
        point_body_to_world(
            s,
            ctx.extrinsic_est_en,
            &ctx.Lidar_R_wrt_IMU,
            &ctx.Lidar_T_wrt_IMU,
            &ctx.feats_down_body[i],
            &mut ctx.feats_down_world[i],
        );
        let point_world_j = ctx.feats_down_world[i];
        let p_body = ctx.pbody_list[i];
        let p_norm = crate::so3::norm3(&p_body);
        let points_near = &mut ctx.Nearest_Points[i];
        ctx.ivox_
            .closest_points(&point_world_j, NUM_MATCH_POINTS, 5.0, points_near);
        if points_near.len() < NUM_MATCH_POINTS {
            ctx.point_selected_surf[i] = false;
        } else {
            ctx.point_selected_surf[i] = false;
            if let Some(pabcd) = esti_plane(points_near, ctx.plane_thr) {
                // esti_plane<float>: everything below is float until the match_s test.
                let pd2 = (pabcd[0] * point_world_j.x
                    + pabcd[1] * point_world_j.y
                    + pabcd[2] * point_world_j.z
                    + pabcd[3])
                    .abs();
                if p_norm > ctx.match_s * pd2 as f64 * pd2 as f64 {
                    ctx.point_selected_surf[i] = true;
                    let nv = &mut ctx.normvec[j];
                    nv.x = pabcd[0];
                    nv.y = pabcd[1];
                    nv.z = pabcd[2];
                    nv.intensity = pabcd[3];
                    effect_num_k += 1;
                }
            }
        }
    }
    if effect_num_k == 0 {
        ekfom_data.valid = false;
        return;
    }
    ekfom_data.M_Noise = ctx.laser_point_cov;
    ekfom_data.h_x = DMatrix::zeros(effect_num_k, 12);
    ekfom_data.z = DVector::zeros(effect_num_k);
    let mut m = 0;
    for j in 0..ts {
        let i = base + j;
        if ctx.point_selected_surf[i] {
            let nv = ctx.normvec[j];
            let norm_vec = V3::new(nv.x as f64, nv.y as f64, nv.z as f64);
            let mut row = SVector::<f64, 12>::zeros();
            row.fixed_rows_mut::<3>(0).copy_from(&norm_vec);
            if ctx.extrinsic_est_en {
                let p_body = ctx.pbody_list[i];
                let p_crossmat = hat(&p_body);
                let point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
                let p_imu_crossmat = hat(&point_imu);
                let c = s.rot.transpose() * norm_vec;
                let a = p_imu_crossmat * c;
                let b = p_crossmat * s.offset_R_L_I.transpose() * c;
                row.fixed_rows_mut::<3>(3).copy_from(&a);
                row.fixed_rows_mut::<3>(6).copy_from(&b);
                row.fixed_rows_mut::<3>(9).copy_from(&c);
            } else {
                let point_crossmat = ctx.crossmat_list[i];
                let c = s.rot.transpose() * norm_vec;
                let a = point_crossmat * c;
                row.fixed_rows_mut::<3>(3).copy_from(&a);
            }
            ekfom_data.h_x.row_mut(m).copy_from(&row.transpose());
            let fw = ctx.feats_down_world[i];
            ekfom_data.z[m] = -norm_vec[0] * fw.x as f64
                - norm_vec[1] * fw.y as f64
                - norm_vec[2] * fw.z as f64
                - nv.intensity as f64;
            m += 1;
        }
    }
    *ctx.effct_feat_num += effect_num_k as i32;
}

/// Globals read by `h_model_IMU_output` (same names).
#[allow(non_snake_case)]
#[derive(Clone, Debug)]
pub struct ImuHCtx {
    pub angvel_avr: V3,
    pub acc_avr: V3,
    pub G_m_s2: f64,
    pub acc_norm: f64,
    pub imu_meas_omg_cov: f64,
    pub imu_meas_acc_cov: f64,
    pub check_satu: bool,
    pub satu_gyro: f64,
    pub satu_acc: f64,
}

#[allow(non_snake_case)]
pub fn h_model_IMU_output(s: &StateOutput, ctx: &ImuHCtx, ekfom_data: &mut DynShareModified) {
    ekfom_data.satu_check = [false; 6];
    let z_omg = ctx.angvel_avr - s.omg - s.bg;
    let z_acc = ctx.acc_avr * ctx.G_m_s2 / ctx.acc_norm - s.acc - s.ba;
    ekfom_data.z_IMU.fixed_rows_mut::<3>(0).copy_from(&z_omg);
    ekfom_data.z_IMU.fixed_rows_mut::<3>(3).copy_from(&z_acc);
    for i in 0..3 {
        ekfom_data.R_IMU[i] = ctx.imu_meas_omg_cov;
        ekfom_data.R_IMU[3 + i] = ctx.imu_meas_acc_cov;
    }
    if ctx.check_satu {
        for i in 0..3 {
            if ctx.angvel_avr[i].abs() >= 0.99 * ctx.satu_gyro {
                ekfom_data.satu_check[i] = true;
                ekfom_data.z_IMU[i] = 0.0;
            }
        }
        for i in 0..3 {
            if ctx.acc_avr[i].abs() >= 0.99 * ctx.satu_acc {
                ekfom_data.satu_check[3 + i] = true;
                ekfom_data.z_IMU[3 + i] = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ieskf::{Esekf, Manifold};

    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
    }

    fn rv(seed: &mut u64, s: f64) -> V3 {
        V3::new(lcg(seed) * s, lcg(seed) * s, lcg(seed) * s)
    }

    fn rot(seed: &mut u64) -> M3 {
        crate::so3::exp(&rv(seed, 2.0))
    }

    /// Central finite difference of `f` along every boxplus direction vs. the analytic Jacobian.
    fn check_jacobian<S: Manifold<N>, const N: usize>(
        x: &S,
        input: &InputIkfom,
        f: fn(&S, &InputIkfom) -> SVector<f64, N>,
        df: fn(&S, &InputIkfom) -> SMatrix<f64, N, N>,
    ) {
        let eps = 1e-6;
        let jac = df(x, input);
        for i in 0..N {
            let mut e = SVector::<f64, N>::zeros();
            e[i] = eps;
            let mut xp = x.clone();
            xp.boxplus(&e);
            let mut xm = x.clone();
            xm.boxplus(&(-e));
            let fd = (f(&xp, input) - f(&xm, input)) / (2.0 * eps);
            let err = (fd - jac.column(i)).norm();
            assert!(err < 1e-5, "column {i}: fd/analytic mismatch {err}");
        }
    }

    #[test]
    fn df_dx_input_matches_finite_difference() {
        let mut seed = 11;
        let x = StateInput {
            pos: rv(&mut seed, 5.0),
            rot: rot(&mut seed),
            offset_R_L_I: rot(&mut seed),
            offset_T_L_I: rv(&mut seed, 0.2),
            vel: rv(&mut seed, 2.0),
            bg: rv(&mut seed, 0.01),
            ba: rv(&mut seed, 0.1),
            gravity: rv(&mut seed, 9.8),
        };
        let input = InputIkfom {
            acc: rv(&mut seed, 9.8),
            gyro: rv(&mut seed, 1.0),
        };
        check_jacobian(&x, &input, get_f_input, df_dx_input);
    }

    #[test]
    fn df_dx_output_matches_finite_difference() {
        let mut seed = 12;
        let x = StateOutput {
            pos: rv(&mut seed, 5.0),
            rot: rot(&mut seed),
            offset_R_L_I: rot(&mut seed),
            offset_T_L_I: rv(&mut seed, 0.2),
            vel: rv(&mut seed, 2.0),
            omg: rv(&mut seed, 1.0),
            acc: rv(&mut seed, 9.8),
            gravity: rv(&mut seed, 9.8),
            bg: rv(&mut seed, 0.01),
            ba: rv(&mut seed, 0.1),
        };
        let input = InputIkfom::default();
        check_jacobian(&x, &input, get_f_output, df_dx_output);
    }

    #[test]
    fn predict_zero_input_keeps_static_state() {
        let q_in = process_noise_cov_input(0.1, 0.1, 1e-4, 1e-4);
        let mut kf = Esekf::<StateInput, 24>::new(get_f_input, df_dx_input);
        kf.x.gravity = V3::new(0.0, 0.0, -9.81);
        kf.P = reset_cov();
        let x0 = kf.x.clone();
        let input = InputIkfom {
            acc: V3::new(0.0, 0.0, 9.81),
            gyro: V3::zeros(),
        };
        kf.predict(0.01, &q_in, &input, true, true);
        assert_eq!(kf.x, x0);
        assert!(kf.P[(0, 0)] > 0.1, "covariance must grow");

        let q_out = process_noise_cov_output(20.0, 0.1, 0.1, 1e-4, 1e-4);
        let mut kf = Esekf::<StateOutput, 30>::new(get_f_output, df_dx_output);
        kf.x.gravity = V3::new(0.0, 0.0, -9.81);
        kf.x.acc = V3::new(0.0, 0.0, 9.81);
        kf.P = reset_cov_output();
        let x0 = kf.x.clone();
        kf.predict(0.01, &q_out, &InputIkfom::default(), true, true);
        assert_eq!(kf.x, x0);
    }

    /// Map: five points on the plane z = -1 around the query.
    fn floor_plane(pt: &PointXYZI) -> IVox {
        let mut ivox = IVox::new(crate::ivox::IVoxOptions::default());
        let pts: Vec<_> = [(0.0, 0.0), (0.1, 0.0), (-0.1, 0.0), (0.0, 0.1), (0.0, -0.1)]
            .iter()
            .map(|&(dx, dy)| PointXYZI {
                x: pt.x + dx,
                y: pt.y + dy,
                z: -1.0,
                ..Default::default()
            })
            .collect();
        ivox.add_points(&pts);
        ivox
    }

    #[test]
    fn h_model_output_one_point_above_floor() {
        let body = [PointXYZI {
            x: 1.0,
            y: 0.0,
            z: -0.9,
            intensity: 7.0,
            curvature: 0.0,
        }];
        let mut world = [PointXYZI::default()];
        let pbody = [V3::new(1.0, 0.0, -0.9)];
        let crossmat = [hat(&pbody[0])];
        let mut nearest = [Vec::new()];
        let mut selected = [false];
        let mut normvec = Vec::new();
        let mut effct = 0;
        let mut ivox = floor_plane(&body[0]);
        let mut ctx = HModelCtx {
            feats_down_body: &body,
            feats_down_world: &mut world,
            pbody_list: &pbody,
            crossmat_list: &crossmat,
            Nearest_Points: &mut nearest,
            point_selected_surf: &mut selected,
            normvec: &mut normvec,
            ivox_: &mut ivox,
            effct_feat_num: &mut effct,
            time_seq: &[1],
            k: 0,
            idx: -1,
            plane_thr: 0.1,
            match_s: 81.0,
            extrinsic_est_en: false,
            laser_point_cov: 0.01,
            Lidar_R_wrt_IMU: M3::identity(),
            Lidar_T_wrt_IMU: V3::zeros(),
        };
        let mut kf = Esekf::<StateOutput, 30>::new(get_f_output, df_dx_output);
        kf.P = reset_cov_output();
        // The filter sets `valid` before every h-model call; the h-model only clears it.
        let mut d = DynShareModified {
            valid: true,
            ..Default::default()
        };
        h_model_output(&kf.x, M3::zeros(), M3::zeros(), &mut ctx, &mut d);
        assert!(d.valid);
        assert_eq!(d.h_x.nrows(), 1);
        assert_eq!(d.z.len(), 1);
        assert!(d.h_x[(0, 2)].abs() > 0.999, "normal is +-z");
        assert!((d.z[0].abs() - 0.1).abs() < 1e-6, "residual is the height");
        assert_eq!(d.M_Noise, 0.01);

        // One filter step must pull the pose down toward the floor.
        let ok = kf.update_iterated_dyn_share_modified(|s, cp, cr, dyn_share| {
            h_model_output(s, cp, cr, &mut ctx, dyn_share)
        });
        assert!(ok);
        assert!(
            kf.x.pos[2] < -1e-3 && kf.x.pos[2] > -0.1,
            "pos.z = {}",
            kf.x.pos[2]
        );
        assert!(kf.P[(2, 2)] < 0.01, "P shrinks");
        assert_eq!(effct, 2, "both h-model calls counted");
        assert_eq!(world[0].intensity, 7.0);
    }

    #[test]
    fn h_model_imu_saturation() {
        let s = StateOutput {
            omg: V3::new(0.1, 0.0, 0.0),
            ..Default::default()
        };
        let ctx = ImuHCtx {
            angvel_avr: V3::new(0.3, 40.0, 0.0),
            acc_avr: V3::new(0.0, 0.0, 1.0),
            G_m_s2: 9.81,
            acc_norm: 1.0,
            imu_meas_omg_cov: 0.1,
            imu_meas_acc_cov: 0.2,
            check_satu: true,
            satu_gyro: 35.0,
            satu_acc: 3.0,
        };
        let mut d = DynShareModified::default();
        h_model_IMU_output(&s, &ctx, &mut d);
        assert!((d.z_IMU[0] - 0.2).abs() < 1e-12);
        assert!(d.satu_check[1] && d.z_IMU[1] == 0.0);
        assert!((d.z_IMU[5] - 9.81).abs() < 1e-12);
        assert_eq!(d.R_IMU[0], 0.1);
        assert_eq!(d.R_IMU[5], 0.2);

        let mut kf = Esekf::<StateOutput, 30>::new(get_f_output, df_dx_output);
        kf.x = s.clone();
        kf.P = reset_cov_output();
        kf.update_iterated_dyn_share_IMU(|s, dyn_share| h_model_IMU_output(s, &ctx, dyn_share));
        assert!(
            kf.x.omg[0] > 0.1,
            "omg.x moves toward the gyro: {}",
            kf.x.omg[0]
        );
        assert_eq!(kf.x.omg[1], 0.0, "saturated axis untouched");
    }
}
