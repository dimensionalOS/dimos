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

// Point-LIO estimator core. Port of dimensionalOS/dimos-module-pointlio (GPL-2.0).
// Pure f64 math, no I/O, no threads: given the same feed sequence it is deterministic.
pub mod common;
pub mod estimator;
pub mod ieskf;
pub mod imu_processing;
pub mod ivox;
pub mod laser_mapping;
pub mod preprocess;
pub mod so3;
pub mod sort;
pub mod voxel_grid;

use nalgebra::Vector3;
use serde::Deserialize;

use crate::common::esti_plane;
pub use crate::common::PointXYZI;
pub use crate::ivox::NearbyType;
pub use crate::laser_mapping::{LaserMapping, Odom};
pub use crate::preprocess::LivoxPoint;

/// The tuning fields of the Python `PointLioConfig`, same names and defaults; unknown keys
/// (ports, IPs, publish rates) are ignored so the coordinator JSON loads as-is.
#[derive(Clone, Debug, Deserialize)]
#[serde(default)]
pub struct Config {
    pub msr_freq: f64,
    pub main_freq: f64,
    pub con_frame: bool,
    pub con_frame_num: i32,
    pub cut_frame: bool,
    pub cut_frame_time_interval: f64,
    pub time_lag_imu_to_lidar: f64,
    pub scan_line: i32,
    pub scan_rate: i32,
    pub blind: f64,
    pub point_filter_num: i32,
    pub use_imu_as_input: bool,
    pub prop_at_freq_of_imu: bool,
    pub check_satu: bool,
    pub init_map_size: i32,
    pub space_down_sample: bool,
    pub satu_acc: f64,
    pub satu_gyro: f64,
    pub acc_norm: f64,
    pub plane_thr: f64,
    pub filter_size_surf: f64,
    pub filter_size_map: f64,
    pub ivox_grid_resolution: f64,
    pub ivox_nearby_type: NearbyType,
    pub cube_side_length: f64,
    pub det_range: f64,
    pub fov_degree: f64,
    pub imu_en: bool,
    pub start_in_aggressive_motion: bool,
    pub extrinsic_est_en: bool,
    pub imu_time_inte: f64,
    pub lidar_meas_cov: f64,
    pub acc_cov_input: f64,
    pub vel_cov: f64,
    pub gyr_cov_input: f64,
    pub gyr_cov_output: f64,
    pub acc_cov_output: f64,
    pub b_gyr_cov: f64,
    pub b_acc_cov: f64,
    pub imu_meas_acc_cov: f64,
    pub imu_meas_omg_cov: f64,
    pub match_s: f64,
    pub gravity_align: bool,
    pub gravity: Vec<f64>,
    pub gravity_init: Vec<f64>,
    pub extrinsic_t: Vec<f64>,
    pub extrinsic_r: Vec<f64>,
    pub publish_odometry_without_downsample: bool,
    pub odom_only: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            msr_freq: 50.0,
            main_freq: 5000.0,
            con_frame: false,
            con_frame_num: 1,
            cut_frame: false,
            cut_frame_time_interval: 0.1,
            time_lag_imu_to_lidar: 0.0,
            scan_line: 4,
            scan_rate: 10,
            blind: 0.5,
            point_filter_num: 3,
            use_imu_as_input: false,
            prop_at_freq_of_imu: true,
            check_satu: true,
            init_map_size: 10,
            space_down_sample: true,
            satu_acc: 3.0,
            satu_gyro: 35.0,
            acc_norm: 1.0,
            plane_thr: 0.1,
            filter_size_surf: 0.2,
            filter_size_map: 0.5,
            ivox_grid_resolution: 2.0,
            ivox_nearby_type: NearbyType::Nearby6,
            cube_side_length: 1000.0,
            det_range: 100.0,
            fov_degree: 360.0,
            imu_en: true,
            start_in_aggressive_motion: false,
            extrinsic_est_en: false,
            imu_time_inte: 0.005,
            lidar_meas_cov: 0.01,
            acc_cov_input: 0.1,
            vel_cov: 20.0,
            gyr_cov_input: 0.01,
            gyr_cov_output: 1000.0,
            acc_cov_output: 500.0,
            b_gyr_cov: 0.0001,
            b_acc_cov: 0.0001,
            imu_meas_acc_cov: 0.01,
            imu_meas_omg_cov: 0.01,
            match_s: 81.0,
            gravity_align: true,
            gravity: vec![0.0, 0.0, -9.81],
            gravity_init: vec![0.0, 0.0, -9.81],
            extrinsic_t: vec![-0.011, -0.02329, 0.04412],
            extrinsic_r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            publish_odometry_without_downsample: false,
            odom_only: false,
        }
    }
}

/// Per-point intermediates of one processed frame (REPLAY.md `frames.bin`).
#[derive(Clone, Debug, PartialEq)]
pub struct PointDump {
    /// `point_selected_surf[i]`.
    pub selected: bool,
    /// `Nearest_Points[i]` xyz.
    pub neighbours: Vec<[f32; 3]>,
    /// `esti_plane(Nearest_Points[i], plane_thr)` when selected, else zeros.
    pub plane: [f32; 4],
}

/// One processed frame's intermediates, read straight off the core after `process()`.
#[derive(Clone, Debug, PartialEq)]
pub struct FrameDump {
    /// Same stamp as `odometry().ts`.
    pub lidar_ts: f64,
    pub feats_down_body: Vec<[f32; 3]>,
    /// `effct_feat_num`.
    pub n_eff: u32,
    /// `kf_output.x_`: pos, rot (3x3 column-major), offset_R_L_I (3x3), offset_T_L_I, vel, omg,
    /// acc, gravity, bg, ba.
    pub state: [f64; 42],
    /// `kf_output.P_`, 30x30 row-major.
    pub p: Vec<f64>,
    pub points: Vec<PointDump>,
}

/// Point-LIO with the `harness.cpp` feed contract: stamps in sensor seconds, accel in g.
pub struct PointLio {
    lm: LaserMapping,
    plane_thr: f32,
}

impl PointLio {
    pub fn new(cfg: &Config) -> Self {
        Self {
            lm: LaserMapping::new(cfg),
            plane_thr: cfg.plane_thr as f32,
        }
    }

    /// `imu_cbk` of a message stamped `fromSec(ts_s)`; `acc_g` in g (raw m/s^2 / 9.80665).
    pub fn feed_imu(&mut self, ts_s: f64, gyro: [f64; 3], acc_g: [f64; 3]) {
        self.lm.imu_cbk(
            laser_mapping::ros_time(ts_s),
            Vector3::from(gyro),
            Vector3::from(acc_g),
        );
    }

    /// `livox_pcl_cbk` of a CustomMsg stamped `fromSec(start_ns / 1e9)`; `offset_time` is
    /// relative to `start_ns`.
    pub fn feed_lidar(&mut self, start_ns: u64, points: &[LivoxPoint]) {
        let stamp = laser_mapping::ros_time(start_ns as f64 / 1e9);
        self.lm.livox_pcl_cbk(stamp, points);
    }

    /// One `run_once`; true when a scan was consumed and `odometry()` is fresh. Drain rule
    /// (harness): loop until `lidar_buffer_len()` stops shrinking.
    pub fn process(&mut self) -> bool {
        self.lm.run_once()
    }

    pub fn lidar_buffer_len(&self) -> usize {
        self.lm.lidar_buffer.len()
    }

    pub fn odometry(&self) -> Odom {
        self.lm.odometry()
    }

    /// Undistorted scan in the IMU frame.
    pub fn body_cloud(&self) -> Vec<PointXYZI> {
        self.lm.body_cloud()
    }

    /// Undistorted scan in the world frame.
    pub fn world_cloud(&self) -> Vec<PointXYZI> {
        self.lm.world_cloud()
    }

    pub fn inner(&self) -> &LaserMapping {
        &self.lm
    }

    /// `dump_frame` of harness.cpp for the frame `process()` just consumed.
    pub fn frame_dump(&self) -> FrameDump {
        let lm = &self.lm;
        let n = lm.feats_down_size;
        let x = &lm.kf_output.x;
        let mut state = Vec::with_capacity(42);
        state.extend_from_slice(x.pos.as_slice());
        state.extend_from_slice(x.rot.as_slice());
        state.extend_from_slice(x.offset_R_L_I.as_slice());
        for v in [
            &x.offset_T_L_I,
            &x.vel,
            &x.omg,
            &x.acc,
            &x.gravity,
            &x.bg,
            &x.ba,
        ] {
            state.extend_from_slice(v.as_slice());
        }
        let mut p = Vec::with_capacity(900);
        for r in 0..30 {
            for c in 0..30 {
                p.push(lm.kf_output.P[(r, c)]);
            }
        }
        let xyz = |q: &PointXYZI| [q.x, q.y, q.z];
        let points = (0..n)
            .map(|i| {
                let nbr = &lm.nearest_points[i];
                let selected = lm.point_selected_surf[i];
                let plane = if selected && nbr.len() >= common::NUM_MATCH_POINTS {
                    esti_plane(nbr, self.plane_thr).unwrap_or([0.0; 4])
                } else {
                    [0.0; 4]
                };
                PointDump {
                    selected,
                    neighbours: nbr.iter().map(xyz).collect(),
                    plane,
                }
            })
            .collect();
        FrameDump {
            lidar_ts: lm.odometry().ts,
            feats_down_body: lm.feats_down_body[..n].iter().map(xyz).collect(),
            n_eff: lm.effct_feat_num as u32,
            state: state.try_into().unwrap(),
            p,
            points,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_loads_coordinator_json() {
        let c: Config = serde_json::from_str(
            r#"{"host_ip":"0.0.0.0","frequency":10.0,"ivox_nearby_type":"nearby18","blind":0.7}"#,
        )
        .unwrap();
        assert_eq!(c.ivox_nearby_type, NearbyType::Nearby18);
        assert_eq!(c.blind, 0.7);
        assert_eq!(c.point_filter_num, 3);
        assert!(serde_json::from_str::<Config>(r#"{"ivox_nearby_type":"x"}"#).is_err());
        let lio = PointLio::new(&c);
        assert!(!lio.odometry().ts.is_nan());
        assert_eq!(lio.lidar_buffer_len(), 0);
    }
}
