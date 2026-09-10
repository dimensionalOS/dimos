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

//! Port of `laserMapping.hpp`: input buffering, lidar/IMU sync, the per-frame point-by-point
//! IESKF loop (both `use_imu_as_input` branches), map init/increment, odometry. The C++ globals
//! are fields of `LaserMapping`; plots, log files, ROS publishing, PCD saving and the dead
//! ikd-Tree FOV code are not ported.

use std::collections::VecDeque;

use nalgebra::SMatrix;

use crate::common::{time_compressing, ImuSample, MeasureGroup, PointXYZI};
use crate::estimator::{
    df_dx_input, df_dx_output, get_f_input, get_f_output, h_model_IMU_output, h_model_input,
    h_model_output, point_body_to_world, process_noise_cov_input, process_noise_cov_output,
    reset_cov, reset_cov_output, HModelCtx, ImuHCtx, Pose,
};
use crate::ieskf::{Esekf, InputIkfom, StateInput, StateOutput, M3, V3};
use crate::imu_processing::ImuProcess;
use crate::ivox::{IVox, IVoxOptions};
use crate::preprocess::{LivoxPoint, Preprocess};
use crate::so3::hat;
use crate::sort::std_sort;
use crate::voxel_grid::voxel_grid;
use crate::Config;

/// `G_m_s2` in Estimator.cpp.
pub const G_M_S2: f64 = 9.81;

/// `custom_messages::Time::fromSec(t).toSec()`: the sec/nsec split every stamp goes through.
pub fn ros_time(t: f64) -> f64 {
    let mut sec = t.floor() as u64;
    let mut nsecs = ((t - sec as f64) * 1e9).round() as u64;
    sec += nsecs / 1_000_000_000;
    nsecs %= 1_000_000_000;
    sec as f64 + 1e-9 * nsecs as f64
}

/// `Eigen::Quaterniond(Matrix3d)` (Shoemake), coefficients `[x, y, z, w]`.
pub fn quat_from_rot(m: &M3) -> [f64; 4] {
    let mut q = [0.0; 4];
    let mut t = m[(0, 0)] + (m[(1, 1)] + m[(2, 2)]);
    if t > 0.0 {
        t = (t + 1.0).sqrt();
        q[3] = 0.5 * t;
        t = 0.5 / t;
        q[0] = (m[(2, 1)] - m[(1, 2)]) * t;
        q[1] = (m[(0, 2)] - m[(2, 0)]) * t;
        q[2] = (m[(1, 0)] - m[(0, 1)]) * t;
    } else {
        let mut i = 0;
        if m[(1, 1)] > m[(0, 0)] {
            i = 1;
        }
        if m[(2, 2)] > m[(i, i)] {
            i = 2;
        }
        let j = (i + 1) % 3;
        let k = (j + 1) % 3;
        t = (m[(i, i)] - m[(j, j)] - m[(k, k)] + 1.0).sqrt();
        q[i] = 0.5 * t;
        t = 0.5 / t;
        q[3] = (m[(k, j)] - m[(j, k)]) * t;
        q[j] = (m[(j, i)] + m[(i, j)]) * t;
        q[k] = (m[(k, i)] + m[(i, k)]) * t;
    }
    q
}

// `Matrix3d::normalized()` as the C++ calls it on offset_R_L_I: Frobenius, not orthonormal
// (a C++ bug, reachable only with extrinsic_est_en). Sum order is Eigen's 9-element redux tree.
fn frob_normalized(m: &M3) -> M3 {
    let a = m.as_slice();
    let sq = |i: usize| a[i] * a[i];
    let z = ((sq(0) + sq(1)) + (sq(2) + sq(3))) + ((sq(4) + sq(5)) + (sq(6) + (sq(7) + sq(8))));
    if z > 0.0 {
        m / z.sqrt()
    } else {
        *m
    }
}

fn time_list(a: &PointXYZI, b: &PointXYZI) -> bool {
    a.curvature < b.curvature
}

/// What `publish_odometry` fills (`odomAftMapped`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Odom {
    /// Sensor seconds after the ROS-time round trip; `lidar_end_time` unless
    /// `publish_odometry_without_downsample`.
    pub ts: f64,
    pub pos: [f64; 3],
    /// `[x, y, z, w]`.
    pub quat: [f64; 4],
    pub vel: [f64; 3],
    pub omg: [f64; 3],
}

pub struct LaserMapping {
    cfg: Config,
    pre: Preprocess,
    p_imu: ImuProcess,
    pub kf_input: Esekf<StateInput, 24>,
    pub kf_output: Esekf<StateOutput, 30>,
    q_input: SMatrix<f64, 24, 24>,
    q_output: SMatrix<f64, 30, 30>,
    pub ivox: IVox,
    lidar_r_wrt_imu: M3,
    lidar_t_wrt_imu: V3,
    state_in: StateInput,
    state_out: StateOutput,
    // callbacks
    pub lidar_buffer: VecDeque<Vec<PointXYZI>>,
    time_buffer: VecDeque<f64>,
    imu_deque: VecDeque<ImuSample>,
    last_timestamp_lidar: f64,
    last_timestamp_imu: f64,
    frame_ct: i32,
    time_con: f64,
    ptr_con: Vec<PointXYZI>,
    // sync
    measures: MeasureGroup,
    lidar_pushed: bool,
    pub lidar_end_time: f64,
    pub first_lidar_time: f64,
    flg_first_scan: bool,
    imu_last: ImuSample,
    imu_next: ImuSample,
    imu_last_ptr: Option<ImuSample>,
    init_map: bool,
    init_feats_world: Vec<PointXYZI>,
    is_first_frame: bool,
    time_update_last: f64,
    time_current: f64,
    time_predict_last_const: f64,
    t_last: f64,
    // per frame
    pub feats_undistort: Vec<PointXYZI>,
    pub feats_down_body: Vec<PointXYZI>,
    pub feats_down_world: Vec<PointXYZI>,
    pub time_seq: Vec<usize>,
    pub nearest_points: Vec<Vec<PointXYZI>>,
    pub point_selected_surf: Vec<bool>,
    normvec: Vec<PointXYZI>,
    pbody_list: Vec<V3>,
    crossmat_list: Vec<M3>,
    pub effct_feat_num: i32,
    pub feats_down_size: usize,
    angvel_avr: V3,
    acc_avr: V3,
    input_in: InputIkfom,
    odom: Odom,
}

impl LaserMapping {
    /// `LaserMapping::setup` + `readParameters`.
    pub fn new(cfg: &Config) -> Self {
        let cfg = cfg.clone();
        let v3 = |v: &[f64], name: &str| {
            assert!(v.len() == 3, "{name} must have 3 entries");
            V3::new(v[0], v[1], v[2])
        };
        let lidar_t_wrt_imu = v3(&cfg.extrinsic_t, "extrinsic_t");
        assert!(
            cfg.extrinsic_r.len() == 9,
            "extrinsic_r must have 9 entries"
        );
        let r = &cfg.extrinsic_r;
        let lidar_r_wrt_imu = M3::new(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
        let mut kf_input = Esekf::new(get_f_input, df_dx_input);
        let mut kf_output = Esekf::new(get_f_output, df_dx_output);
        if cfg.extrinsic_est_en {
            if !cfg.use_imu_as_input {
                kf_output.x.offset_R_L_I = lidar_r_wrt_imu;
                kf_output.x.offset_T_L_I = lidar_t_wrt_imu;
            } else {
                kf_input.x.offset_R_L_I = lidar_r_wrt_imu;
                kf_input.x.offset_T_L_I = lidar_t_wrt_imu;
            }
        }
        kf_input.P = reset_cov();
        kf_output.P = reset_cov_output();
        Self {
            pre: Preprocess {
                blind: cfg.blind,
                point_filter_num: cfg.point_filter_num as u32,
                n_scans: cfg.scan_line as u16,
                // readParameters never forwards det_range to Preprocess; it keeps the ctor's 1000.
                ..Default::default()
            },
            p_imu: {
                let mut p = ImuProcess::default();
                p.imu_en = cfg.imu_en;
                p
            },
            kf_input,
            kf_output,
            q_input: process_noise_cov_input(
                cfg.gyr_cov_input,
                cfg.acc_cov_input,
                cfg.b_gyr_cov,
                cfg.b_acc_cov,
            ),
            q_output: process_noise_cov_output(
                cfg.vel_cov,
                cfg.gyr_cov_output,
                cfg.acc_cov_output,
                cfg.b_gyr_cov,
                cfg.b_acc_cov,
            ),
            ivox: IVox::new(IVoxOptions {
                resolution: cfg.ivox_grid_resolution as f32,
                nearby_type: cfg.ivox_nearby_type,
                ..Default::default()
            }),
            lidar_r_wrt_imu,
            lidar_t_wrt_imu,
            state_in: StateInput::default(),
            state_out: StateOutput::default(),
            lidar_buffer: VecDeque::new(),
            time_buffer: VecDeque::new(),
            imu_deque: VecDeque::new(),
            last_timestamp_lidar: -1.0,
            last_timestamp_imu: -1.0,
            frame_ct: 0,
            time_con: 0.0,
            ptr_con: Vec::new(),
            measures: MeasureGroup::default(),
            lidar_pushed: false,
            lidar_end_time: 0.0,
            first_lidar_time: 0.0,
            flg_first_scan: true,
            imu_last: ImuSample::default(),
            imu_next: ImuSample::default(),
            imu_last_ptr: None,
            init_map: false,
            init_feats_world: Vec::new(),
            is_first_frame: true,
            time_update_last: 0.0,
            time_current: 0.0,
            time_predict_last_const: 0.0,
            t_last: 0.0,
            feats_undistort: Vec::new(),
            feats_down_body: Vec::new(),
            feats_down_world: Vec::new(),
            time_seq: Vec::new(),
            nearest_points: Vec::new(),
            point_selected_surf: Vec::new(),
            normvec: Vec::new(),
            pbody_list: Vec::new(),
            crossmat_list: Vec::new(),
            effct_feat_num: 0,
            feats_down_size: 0,
            angvel_avr: V3::zeros(),
            acc_avr: V3::zeros(),
            input_in: InputIkfom::default(),
            odom: Odom::default(),
            cfg,
        }
    }

    /// `livox_pcl_cbk`; `stamp` is the header stamp in seconds (already ROS-time rounded).
    pub fn livox_pcl_cbk(&mut self, stamp: f64, points: &[LivoxPoint]) {
        if stamp < self.last_timestamp_lidar {
            return; // lidar loop back
        }
        self.last_timestamp_lidar = stamp;
        let mut ptr = self.pre.process(points);
        let cfg = &self.cfg;
        if cfg.cut_frame {
            std_sort(&mut ptr, &time_list);
            let mut time_div = stamp;
            let mut ptr_div = Vec::new();
            for p in &ptr {
                ptr_div.push(*p);
                if p.curvature as f64 / 1000.0 + stamp - time_div > cfg.cut_frame_time_interval {
                    self.lidar_buffer.push_back(ptr_div.clone());
                    self.time_buffer.push_back(time_div);
                    time_div += p.curvature as f64 / 1000.0;
                    ptr_div.clear();
                }
            }
            if !ptr_div.is_empty() {
                self.lidar_buffer.push_back(ptr_div);
                self.time_buffer.push_back(time_div);
            }
        } else if cfg.con_frame {
            if self.frame_ct == 0 {
                self.time_con = self.last_timestamp_lidar;
            }
            if self.frame_ct < cfg.con_frame_num {
                for mut p in ptr {
                    p.curvature = (p.curvature as f64
                        + (self.last_timestamp_lidar - self.time_con) * 1000.0)
                        as f32;
                    self.ptr_con.push(p);
                }
                self.frame_ct += 1;
            } else {
                // The frame that overflows con_frame_num is dropped, as in the C++.
                self.lidar_buffer
                    .push_back(std::mem::take(&mut self.ptr_con));
                self.time_buffer.push_back(self.time_con);
                self.frame_ct = 0;
            }
        } else {
            self.lidar_buffer.push_back(ptr);
            self.time_buffer.push_back(stamp);
        }
    }

    /// `imu_cbk`; `stamp` is the message stamp in seconds (already ROS-time rounded).
    pub fn imu_cbk(&mut self, stamp: f64, gyro: V3, acc: V3) {
        let timestamp = ros_time(stamp - self.cfg.time_lag_imu_to_lidar);
        if timestamp < self.last_timestamp_imu {
            return; // imu loop back
        }
        self.imu_deque.push_back(ImuSample {
            stamp: timestamp,
            gyro,
            acc,
        });
        self.last_timestamp_imu = timestamp;
    }

    fn lidar_end(lidar: &[PointXYZI], beg: f64) -> f64 {
        let end_time = lidar.iter().fold(lidar[lidar.len() - 1].curvature, |m, p| {
            if p.curvature > m {
                p.curvature
            } else {
                m
            }
        });
        beg + end_time as f64 / 1000.0
    }

    fn sync_packages(&mut self) -> bool {
        if !self.cfg.imu_en {
            if let Some(lidar) = self.lidar_buffer.pop_front() {
                self.measures.lidar_beg_time = self.time_buffer.pop_front().unwrap();
                self.measures.lidar = lidar;
                if self.measures.lidar.is_empty() {
                    return false; // lose lidar
                }
                self.lidar_end_time =
                    Self::lidar_end(&self.measures.lidar, self.measures.lidar_beg_time);
                self.measures.lidar_last_time = self.lidar_end_time;
                return true;
            }
            return false;
        }
        if self.lidar_buffer.is_empty() || self.imu_deque.is_empty() {
            return false;
        }
        if !self.lidar_pushed {
            let lidar = &self.lidar_buffer[0];
            if lidar.is_empty() {
                self.lidar_buffer.pop_front();
                self.time_buffer.pop_front();
                return false; // lose lidar
            }
            self.measures.lidar = lidar.clone();
            self.measures.lidar_beg_time = self.time_buffer[0];
            self.lidar_end_time =
                Self::lidar_end(&self.measures.lidar, self.measures.lidar_beg_time);
            self.measures.lidar_last_time = self.lidar_end_time;
            self.lidar_pushed = true;
        }
        if self.last_timestamp_imu < self.lidar_end_time {
            return false;
        }
        // meas.imu is never cleared in the C++ (only shrink_to_fit), so it accumulates.
        if self.p_imu.imu_need_init_ {
            self.drain_imu_into_measures();
        } else if !self.init_map {
            let last = self.imu_last_ptr.expect("no IMU consumed before map init");
            self.measures.imu.push_back(last);
            self.drain_imu_into_measures();
        }
        self.lidar_buffer.pop_front();
        self.time_buffer.pop_front();
        self.lidar_pushed = false;
        true
    }

    fn drain_imu_into_measures(&mut self) {
        let mut imu_time = self.imu_deque[0].stamp;
        while !self.imu_deque.is_empty() && imu_time < self.lidar_end_time {
            imu_time = self.imu_deque[0].stamp;
            if imu_time > self.lidar_end_time {
                break;
            }
            let s = self.imu_deque.pop_front().unwrap();
            self.measures.imu.push_back(s);
            self.imu_last = self.imu_next;
            self.imu_last_ptr = Some(s);
            self.imu_next = s;
        }
    }

    /// `run_once`: true when a scan was processed and `odometry()` refreshed.
    pub fn run_once(&mut self) -> bool {
        if !self.sync_packages() {
            return false;
        }
        if self.flg_first_scan {
            self.first_lidar_time = self.measures.lidar_beg_time;
            self.flg_first_scan = false;
        }
        if self.p_imu.process(self.measures.imu.make_contiguous()) {
            self.feats_undistort = self.measures.lidar.clone();
        }
        if self.feats_undistort.is_empty() {
            return false;
        }
        let cfg = &self.cfg;
        let gravity_init = V3::new(
            cfg.gravity_init[0],
            cfg.gravity_init[1],
            cfg.gravity_init[2],
        );
        if cfg.imu_en {
            if !self.p_imu.gravity_align_ {
                while self.measures.lidar_beg_time > self.imu_next.stamp {
                    let Some(next) = self.imu_deque.pop_front() else {
                        break;
                    };
                    self.imu_last = self.imu_next;
                    self.imu_next = next;
                }
                if cfg.start_in_aggressive_motion {
                    self.state_in.gravity = gravity_init;
                    self.state_out.gravity = gravity_init;
                    self.state_out.acc = gravity_init * -1.0;
                } else {
                    let m = self.p_imu.mean_acc;
                    self.state_in.gravity = (-1.0 * m) * G_M_S2 / cfg.acc_norm;
                    self.state_out.gravity = (-1.0 * m) * G_M_S2 / cfg.acc_norm;
                    self.state_out.acc = m * G_M_S2 / cfg.acc_norm;
                }
                if cfg.gravity_align {
                    self.p_imu.gravity_ = V3::new(cfg.gravity[0], cfg.gravity[1], cfg.gravity[2]);
                    let rot_init = self.p_imu.set_init(&self.state_in.gravity);
                    self.state_in.gravity = self.p_imu.gravity_;
                    self.state_out.gravity = self.p_imu.gravity_;
                    self.state_in.rot = rot_init;
                    self.state_out.rot = rot_init;
                    self.state_out.acc = -rot_init.transpose() * self.state_out.gravity;
                }
                // change_x: replaces the whole state, incl. any extrinsic seeded in setup.
                self.kf_input.x = self.state_in.clone();
                self.kf_output.x = self.state_out.clone();
            }
        } else if !self.p_imu.gravity_align_ {
            self.state_in.gravity = gravity_init;
            self.state_out.gravity = gravity_init;
            self.state_out.acc = gravity_init * -1.0;
        }

        // downsample
        if cfg.space_down_sample {
            self.feats_down_body = voxel_grid(&self.feats_undistort, cfg.filter_size_surf as f32);
        } else {
            self.feats_down_body = self.measures.lidar.clone();
        }
        std_sort(&mut self.feats_down_body, &time_list);
        self.time_seq = time_compressing(&self.feats_down_body);
        let n = self.feats_down_body.len();
        self.feats_down_size = n;

        let pose_of =
            |ki: &Esekf<StateInput, 24>, ko: &Esekf<StateOutput, 30>| -> (V3, M3, M3, V3) {
                if !cfg.use_imu_as_input {
                    (ko.x.pos, ko.x.rot, ko.x.offset_R_L_I, ko.x.offset_T_L_I)
                } else {
                    (ki.x.pos, ki.x.rot, ki.x.offset_R_L_I, ki.x.offset_T_L_I)
                }
            };

        // map init
        if !self.init_map {
            self.feats_down_world.resize(n, PointXYZI::default());
            let (pos, rot, orli, otli) = pose_of(&self.kf_input, &self.kf_output);
            let pose = Pose {
                pos: &pos,
                rot: &rot,
                offset_R_L_I: &orli,
                offset_T_L_I: &otli,
            };
            for i in 0..n {
                point_body_to_world(
                    pose,
                    cfg.extrinsic_est_en,
                    &self.lidar_r_wrt_imu,
                    &self.lidar_t_wrt_imu,
                    &self.feats_down_body[i],
                    &mut self.feats_down_world[i],
                );
            }
            self.init_feats_world
                .extend_from_slice(&self.feats_down_world[..n]);
            if self.init_feats_world.len() < cfg.init_map_size.max(0) as usize {
                return false;
            }
            self.ivox.add_points(&self.init_feats_world);
            self.init_feats_world = Vec::new();
            self.init_map = true;
            return false;
        }

        self.normvec.resize(n, PointXYZI::default());
        self.feats_down_world.resize(n, PointXYZI::default());
        self.nearest_points.resize(n, Vec::new());
        if self.point_selected_surf.len() < n {
            self.point_selected_surf.resize(n, true);
        }
        self.crossmat_list.resize(n, M3::zeros());
        self.pbody_list.resize(n, V3::zeros());
        for i in 0..n {
            let p = self.feats_down_body[i];
            let mut point_this = V3::new(p.x as f64, p.y as f64, p.z as f64);
            self.pbody_list[i] = point_this;
            if cfg.extrinsic_est_en {
                let (_, _, orli, otli) = pose_of(&self.kf_input, &self.kf_output);
                point_this = frob_normalized(&orli) * point_this + otli;
            } else {
                point_this = self.lidar_r_wrt_imu * point_this + self.lidar_t_wrt_imu;
            }
            self.crossmat_list[i] = hat(&point_this);
        }

        self.point_by_point_update(n);
        if n > 4 {
            map_incremental(
                &mut self.ivox,
                &self.feats_down_world,
                &self.nearest_points,
                self.cfg.filter_size_map,
            );
        }
        true
    }

    /// The `/*** iterated state estimation ***/` loop of run_once for either filter.
    fn point_by_point_update(&mut self, n: usize) {
        let Self {
            cfg,
            kf_input,
            kf_output,
            q_input,
            q_output,
            ivox,
            lidar_r_wrt_imu,
            lidar_t_wrt_imu,
            imu_deque,
            measures,
            lidar_end_time,
            imu_last,
            imu_next,
            is_first_frame,
            time_update_last,
            time_current,
            time_predict_last_const,
            t_last,
            feats_down_body,
            feats_down_world,
            time_seq,
            nearest_points,
            point_selected_surf,
            normvec,
            pbody_list,
            crossmat_list,
            effct_feat_num,
            angvel_avr,
            acc_avr,
            input_in,
            odom,
            ..
        } = self;
        *effct_feat_num = 0;
        let pcl_beg_time = measures.lidar_beg_time;
        let mut idx: isize = -1;
        let mut ctx = HModelCtx {
            feats_down_body: feats_down_body.as_slice(),
            feats_down_world: feats_down_world.as_mut_slice(),
            pbody_list: pbody_list.as_slice(),
            crossmat_list: crossmat_list.as_slice(),
            Nearest_Points: nearest_points.as_mut_slice(),
            point_selected_surf: point_selected_surf.as_mut_slice(),
            normvec,
            ivox_: ivox,
            effct_feat_num,
            time_seq: time_seq.as_slice(),
            k: 0,
            idx: -1,
            plane_thr: cfg.plane_thr as f32,
            match_s: cfg.match_s,
            extrinsic_est_en: cfg.extrinsic_est_en,
            laser_point_cov: cfg.lidar_meas_cov,
            Lidar_R_wrt_IMU: *lidar_r_wrt_imu,
            Lidar_T_wrt_IMU: *lidar_t_wrt_imu,
        };
        let publish = |odom: &mut Odom,
                       ki: &Esekf<StateInput, 24>,
                       ko: &Esekf<StateOutput, 30>,
                       stamp: f64,
                       imu_last: &ImuSample| {
            let arr = |v: &V3| [v[0], v[1], v[2]];
            odom.ts = ros_time(stamp);
            if !cfg.use_imu_as_input {
                odom.pos = arr(&ko.x.pos);
                odom.quat = quat_from_rot(&ko.x.rot);
                odom.vel = arr(&ko.x.vel);
                odom.omg = arr(&ko.x.omg);
            } else {
                odom.pos = arr(&ki.x.pos);
                odom.quat = quat_from_rot(&ki.x.rot);
                odom.vel = arr(&ki.x.vel);
                odom.omg = arr(&imu_last.gyro);
            }
        };
        let to_world = |ctx: &mut HModelCtx, pose: Pose, idx: isize, len: usize| {
            for j in 0..len {
                let i = (idx + j as isize + 1) as usize;
                point_body_to_world(
                    pose,
                    cfg.extrinsic_est_en,
                    &ctx.Lidar_R_wrt_IMU,
                    &ctx.Lidar_T_wrt_IMU,
                    &ctx.feats_down_body[i],
                    &mut ctx.feats_down_world[i],
                );
            }
        };

        if !cfg.use_imu_as_input {
            let imu_ctx = |angvel_avr: V3, acc_avr: V3| ImuHCtx {
                angvel_avr,
                acc_avr,
                G_m_s2: G_M_S2,
                acc_norm: cfg.acc_norm,
                imu_meas_omg_cov: cfg.imu_meas_omg_cov,
                imu_meas_acc_cov: cfg.imu_meas_acc_cov,
                check_satu: cfg.check_satu,
                satu_gyro: cfg.satu_gyro,
                satu_acc: cfg.satu_acc,
            };
            for k in 0..ctx.time_seq.len() {
                let point_body = ctx.feats_down_body[(idx + ctx.time_seq[k] as isize) as usize];
                *time_current = point_body.curvature as f64 / 1000.0 + pcl_beg_time;
                if *is_first_frame {
                    if cfg.imu_en {
                        while *time_current > imu_next.stamp {
                            let Some(next) = imu_deque.pop_front() else {
                                break;
                            };
                            *imu_last = *imu_next;
                            *imu_next = next;
                        }
                        *angvel_avr = imu_last.gyro;
                        *acc_avr = imu_last.acc;
                    }
                    *is_first_frame = false;
                    *time_update_last = *time_current;
                    *time_predict_last_const = *time_current;
                }
                if cfg.imu_en && !imu_deque.is_empty() {
                    let mut imu_comes = *time_current > imu_next.stamp;
                    while imu_comes {
                        *angvel_avr = imu_next.gyro;
                        *acc_avr = imu_next.acc;
                        let dt = imu_next.stamp - *time_predict_last_const;
                        kf_output.predict(dt, q_output, input_in, true, false);
                        *time_predict_last_const = imu_next.stamp;
                        let dt_cov = imu_next.stamp - *time_update_last;
                        if dt_cov > 0.0 {
                            *time_update_last = imu_next.stamp;
                            kf_output.predict(dt_cov, q_output, input_in, false, true);
                            let ic = imu_ctx(*angvel_avr, *acc_avr);
                            kf_output.update_iterated_dyn_share_IMU(|s, d| {
                                h_model_IMU_output(s, &ic, d)
                            });
                        }
                        let Some(next) = imu_deque.pop_front() else {
                            break;
                        };
                        *imu_last = *imu_next;
                        *imu_next = next;
                        imu_comes = *time_current > imu_next.stamp;
                    }
                }
                let dt = *time_current - *time_predict_last_const;
                if !cfg.prop_at_freq_of_imu {
                    let dt_cov = *time_current - *time_update_last;
                    if dt_cov > 0.0 {
                        kf_output.predict(dt_cov, q_output, input_in, false, true);
                        *time_update_last = *time_current;
                    }
                }
                kf_output.predict(dt, q_output, input_in, true, false);
                *time_predict_last_const = *time_current;
                if n < 1 {
                    idx += ctx.time_seq[k] as isize;
                    continue;
                }
                ctx.k = k;
                ctx.idx = idx;
                let ok = kf_output.update_iterated_dyn_share_modified(|s, cp, cr, d| {
                    h_model_output(s, cp, cr, &mut ctx, d)
                });
                if !ok {
                    idx += ctx.time_seq[k] as isize;
                    continue;
                }
                if cfg.prop_at_freq_of_imu {
                    let dt_cov = *time_current - *time_update_last;
                    if !cfg.imu_en && dt_cov >= cfg.imu_time_inte {
                        kf_output.predict(dt_cov, q_output, input_in, false, true);
                        *time_update_last = *time_current;
                    }
                }
                if cfg.publish_odometry_without_downsample {
                    publish(odom, kf_input, kf_output, *time_current, imu_last);
                }
                let len = ctx.time_seq[k];
                to_world(&mut ctx, Pose::from(&kf_output.x), idx, len);
                idx += len as isize;
            }
        } else {
            for k in 0..ctx.time_seq.len() {
                let point_body = ctx.feats_down_body[(idx + ctx.time_seq[k] as isize) as usize];
                *time_current = point_body.curvature as f64 / 1000.0 + pcl_beg_time;
                if *is_first_frame {
                    while *time_current > imu_next.stamp {
                        *imu_last = *imu_next;
                        *imu_next = imu_deque.pop_front().expect("imu_deque ran dry");
                    }
                    *is_first_frame = false;
                    *t_last = *time_current;
                    *time_update_last = *time_current;
                    input_in.gyro = imu_last.gyro;
                    input_in.acc = imu_last.acc * G_M_S2 / cfg.acc_norm;
                }
                while *time_current > imu_next.stamp {
                    *imu_last = *imu_next;
                    *imu_next = imu_deque.pop_front().expect("imu_deque ran dry");
                    input_in.gyro = imu_last.gyro;
                    input_in.acc = imu_last.acc * G_M_S2 / cfg.acc_norm;
                    let dt = imu_last.stamp - *t_last;
                    let dt_cov = imu_last.stamp - *time_update_last;
                    if dt_cov > 0.0 {
                        kf_input.predict(dt_cov, q_input, input_in, false, true);
                        *time_update_last = imu_last.stamp;
                    }
                    kf_input.predict(dt, q_input, input_in, true, false);
                    *t_last = imu_last.stamp;
                }
                let dt = *time_current - *t_last;
                *t_last = *time_current;
                if !cfg.prop_at_freq_of_imu {
                    let dt_cov = *time_current - *time_update_last;
                    if dt_cov > 0.0 {
                        kf_input.predict(dt_cov, q_input, input_in, false, true);
                        *time_update_last = *time_current;
                    }
                }
                kf_input.predict(dt, q_input, input_in, true, false);
                if n < 1 {
                    idx += ctx.time_seq[k] as isize;
                    continue;
                }
                ctx.k = k;
                ctx.idx = idx;
                let ok = kf_input.update_iterated_dyn_share_modified(|s, cp, cr, d| {
                    h_model_input(s, cp, cr, &mut ctx, d)
                });
                if !ok {
                    idx += ctx.time_seq[k] as isize;
                    continue;
                }
                if cfg.publish_odometry_without_downsample {
                    publish(odom, kf_input, kf_output, *time_current, imu_last);
                }
                let len = ctx.time_seq[k];
                to_world(&mut ctx, Pose::from(&kf_input.x), idx, len);
                idx += len as isize;
            }
        }
        if !cfg.publish_odometry_without_downsample {
            publish(odom, kf_input, kf_output, *lidar_end_time, imu_last);
        }
    }

    pub fn odometry(&self) -> Odom {
        self.odom
    }

    /// `get_body_cloud`: the undistorted scan in the IMU frame.
    pub fn body_cloud(&self) -> Vec<PointXYZI> {
        let (r, t) = if self.cfg.extrinsic_est_en {
            let (_, _, orli, otli) = if !self.cfg.use_imu_as_input {
                let x = &self.kf_output.x;
                (x.pos, x.rot, x.offset_R_L_I, x.offset_T_L_I)
            } else {
                let x = &self.kf_input.x;
                (x.pos, x.rot, x.offset_R_L_I, x.offset_T_L_I)
            };
            (frob_normalized(&orli), otli)
        } else {
            (self.lidar_r_wrt_imu, self.lidar_t_wrt_imu)
        };
        self.feats_undistort
            .iter()
            .map(|p| {
                let b = r * V3::new(p.x as f64, p.y as f64, p.z as f64) + t;
                PointXYZI {
                    x: b[0] as f32,
                    y: b[1] as f32,
                    z: b[2] as f32,
                    intensity: p.intensity,
                    curvature: 0.0,
                }
            })
            .collect()
    }

    /// `get_world_cloud`: the undistorted scan registered with `kf_output` (whatever the mode).
    pub fn world_cloud(&self) -> Vec<PointXYZI> {
        let x = &self.kf_output.x;
        self.feats_undistort
            .iter()
            .map(|p| {
                let pb = V3::new(p.x as f64, p.y as f64, p.z as f64);
                let w = x.rot * (x.offset_R_L_I * pb + x.offset_T_L_I) + x.pos;
                PointXYZI {
                    x: w[0] as f32,
                    y: w[1] as f32,
                    z: w[2] as f32,
                    intensity: p.intensity,
                    curvature: 0.0,
                }
            })
            .collect()
    }
}

/// `map_incremental`: add the frame's world points not already covered by a map neighbour.
fn map_incremental(
    ivox: &mut IVox,
    feats_down_world: &[PointXYZI],
    nearest_points: &[Vec<PointXYZI>],
    filter_size_map: f64,
) {
    let fs = filter_size_map as f32;
    let mut points_to_add = Vec::with_capacity(feats_down_world.len());
    for (pw, near) in feats_down_world.iter().zip(nearest_points) {
        if near.is_empty() {
            points_to_add.push(*pw);
            continue;
        }
        let center = [pw.x, pw.y, pw.z].map(|v| ((v / fs).floor() + 0.5) * fs);
        let covered = near.iter().any(|q| {
            [q.x - center[0], q.y - center[1], q.z - center[2]]
                .iter()
                .all(|d| (d.abs() as f64) < 0.5 * filter_size_map)
        });
        if !covered {
            points_to_add.push(*pw);
        }
    }
    ivox.add_points(&points_to_add);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helpers() {
        // ROS time round trip keeps ns resolution and normalises a rounded-up nsec.
        assert_eq!(ros_time(103.141736208), 103.141736208);
        assert!((ros_time(5.9999999996) - 6.0).abs() < 1e-12);
        // Quaternion of a 90 deg yaw and of a 180 deg roll (trace <= 0 branch).
        let q = quat_from_rot(&crate::so3::exp(&V3::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
        )));
        let s = std::f64::consts::FRAC_1_SQRT_2;
        assert!((q[2] - s).abs() < 1e-12 && (q[3] - s).abs() < 1e-12);
        let q = quat_from_rot(&M3::from_diagonal(&V3::new(1.0, -1.0, -1.0)));
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
        let f = frob_normalized(&M3::identity());
        assert!((f[(0, 0)] - 1.0 / 3f64.sqrt()).abs() < 1e-15);
    }

    #[test]
    fn static_scene_stays_put() {
        // A box room seen from a fixed sensor with a resting IMU: after init the pose must
        // stay near the origin and every frame after map init must produce odometry.
        let cfg = Config {
            point_filter_num: 1,
            blind: 0.1,
            ..Default::default()
        };
        let mut lm = LaserMapping::new(&cfg);
        let mut frame = Vec::new();
        for i in 0..1500u64 {
            let a = i as f32 * 0.0037;
            let b = (i % 97) as f32 * 0.03 - 1.4;
            // Rays hitting walls at x=+-4, y=+-4, z=+-2 (a crude box).
            let dir = [a.cos() * b.cos(), a.sin() * b.cos(), b.sin()];
            let t = [4.0 / dir[0].abs(), 4.0 / dir[1].abs(), 2.0 / dir[2].abs()]
                .into_iter()
                .fold(f32::MAX, f32::min);
            frame.push(LivoxPoint {
                x: dir[0] * t,
                y: dir[1] * t,
                z: dir[2] * t,
                reflectivity: 50,
                tag: 0,
                line: 0,
                offset_time: i * 66_000,
            });
        }
        let mut processed = 0;
        for f in 0..10u64 {
            let t0 = 100.0 + f as f64 * 0.1;
            for j in 0..20 {
                lm.imu_cbk(t0 + j as f64 * 0.005, V3::zeros(), V3::new(0.0, 0.0, 1.0));
            }
            lm.imu_cbk(t0 + 0.1, V3::zeros(), V3::new(0.0, 0.0, 1.0));
            lm.livox_pcl_cbk(t0, &frame);
            loop {
                let before = lm.lidar_buffer.len();
                if lm.run_once() {
                    processed += 1;
                    let o = lm.odometry();
                    assert!(o.pos.iter().all(|v| v.abs() < 0.05), "{o:?}");
                    assert!((o.quat[3].abs() - 1.0).abs() < 1e-3, "{o:?}");
                }
                if lm.lidar_buffer.len() == before {
                    break;
                }
            }
        }
        assert!(processed >= 5, "processed {processed}");
        assert_eq!(lm.body_cloud().len(), lm.world_cloud().len());
        assert!(lm.ivox.num_valid_grids() > 10);
    }
}
