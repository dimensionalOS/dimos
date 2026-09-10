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

//! Port of `IMU_Processing.hpp`: IMU init statistics + gravity alignment (`ImuProcess`).

use crate::common::ImuSample;
use crate::ieskf::{M3, V3};
use crate::so3::exp_xyz;

pub const MAX_INI_COUNT: i32 = 100;

#[allow(non_snake_case)]
#[derive(Clone, Debug)]
pub struct ImuProcess {
    pub imu_en: bool,
    pub mean_acc: V3,
    pub gravity_: V3,
    pub imu_need_init_: bool,
    pub b_first_frame_: bool,
    pub gravity_align_: bool,
    mean_gyr: V3,
    init_iter_num: i32,
}

impl Default for ImuProcess {
    fn default() -> Self {
        Self {
            imu_en: true,
            mean_acc: V3::new(0.0, 0.0, -1.0),
            gravity_: V3::zeros(),
            imu_need_init_: true,
            b_first_frame_: true,
            gravity_align_: false,
            mean_gyr: V3::zeros(),
            init_iter_num: 1,
        }
    }
}

impl ImuProcess {
    pub fn reset(&mut self) {
        self.mean_acc = V3::new(0.0, 0.0, -1.0);
        self.mean_gyr = V3::zeros();
        self.imu_need_init_ = true;
        self.init_iter_num = 1;
    }

    /// `IMU_init`: running mean of acc/gyro over `meas.imu` (N is `init_iter_num`).
    fn imu_init(&mut self, imu: &[ImuSample]) {
        if self.b_first_frame_ {
            self.reset();
            self.init_iter_num = 1;
            self.b_first_frame_ = false;
            self.mean_acc = imu[0].acc;
            self.mean_gyr = imu[0].gyro;
        }
        for s in imu {
            let n = self.init_iter_num as f64;
            self.mean_acc += (s.acc - self.mean_acc) / n;
            self.mean_gyr += (s.gyro - self.mean_gyr) / n;
            self.init_iter_num += 1;
        }
    }

    /// `Process(meas, cur_pcl_un_)` with `imu = meas.imu`. Returns true when the C++ copied
    /// `meas.lidar` into `cur_pcl_un_`; false means it returned early and left the old cloud.
    pub fn process(&mut self, imu: &[ImuSample]) -> bool {
        if self.imu_en {
            if imu.is_empty() {
                return false;
            }
            if self.imu_need_init_ {
                self.imu_init(imu);
                self.imu_need_init_ = true;
                if self.init_iter_num > MAX_INI_COUNT {
                    self.imu_need_init_ = false;
                    return true;
                }
                return false;
            }
            if !self.gravity_align_ {
                self.gravity_align_ = true;
            }
            true
        } else {
            if !self.b_first_frame_ {
                if !self.gravity_align_ {
                    self.gravity_align_ = true;
                }
            } else {
                self.b_first_frame_ = false;
                return false;
            }
            true
        }
    }

    /// `Set_init`: rotation taking `tmp_gravity` (measured) onto `gravity_` (configured).
    pub fn set_init(&self, tmp_gravity: &V3) -> M3 {
        let g = &self.gravity_;
        let hat_grav = M3::new(0.0, g[2], -g[1], -g[2], 0.0, g[0], g[1], -g[0], 0.0);
        let align_norm = (hat_grav * tmp_gravity).norm() / tmp_gravity.norm() / g.norm();
        let mut align_cos = g.dot(tmp_gravity);
        align_cos = align_cos / g.norm() / tmp_gravity.norm();
        if align_norm < 1e-6 {
            if align_cos > 1e-6 {
                M3::identity()
            } else {
                -M3::identity()
            }
        } else {
            let align_angle =
                hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * align_cos.acos();
            exp_xyz(align_angle[0], align_angle[1], align_angle[2])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_needs_max_ini_count_samples_then_passes_clouds() {
        let mut p = ImuProcess::default();
        assert!(!p.process(&[]));
        let s = ImuSample {
            stamp: 0.0,
            acc: V3::new(0.0, 0.0, 1.0),
            gyro: V3::new(0.01, 0.0, 0.0),
        };
        let batch = vec![s; 10];
        for _ in 0..9 {
            assert!(!p.process(&batch));
            assert!(p.imu_need_init_);
        }
        assert!(p.process(&batch));
        assert!(!p.imu_need_init_);
        assert!(!p.gravity_align_);
        assert!((p.mean_acc - V3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
        assert!(p.process(&batch));
        assert!(p.gravity_align_);
    }

    #[test]
    fn set_init_rotates_measured_gravity_onto_configured() {
        let p = ImuProcess {
            gravity_: V3::new(0.0, 0.0, -9.81),
            ..Default::default()
        };
        let tmp = V3::new(1.0, 0.5, -9.0);
        let rot = p.set_init(&tmp);
        let aligned = rot * tmp;
        let dir = aligned / aligned.norm();
        assert!(
            (dir - V3::new(0.0, 0.0, -1.0)).norm() < 1e-9,
            "aligned = {aligned}"
        );
        assert_eq!(p.set_init(&p.gravity_.clone()), M3::identity());
        assert_eq!(p.set_init(&-p.gravity_), -M3::identity());
    }
}
