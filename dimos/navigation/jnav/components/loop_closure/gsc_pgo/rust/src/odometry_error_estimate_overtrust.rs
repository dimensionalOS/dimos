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

//! The learned odometry error estimate is only logged: the solve and the gates use the stated odometry error estimate.

use crate::gsc_pgo::{Config, GscPgo};

/// The learned odometry error estimate is only logged: the solve and the gates use the stated odometry error estimate.
#[test]
fn the_solve_and_gates_use_the_stated_odometry_error_estimate() {
    let pgo = GscPgo::new(Config {
        learned_odometry_error_estimate_from_closures: true,
        learned_odometry_error_estimate_prior_ln_sd_xy: 1.72,
        learned_odometry_error_estimate_prior_ln_sd_z: 2.05,
        learned_odometry_error_estimate_prior_ln_sd_yaw: 1.15,
        learned_odometry_error_estimate_prior_ln_sd_tilt: 0.96,
        ..Config::default()
    });
    assert_eq!(pgo.odometry_error_estimate(), [1.0; 4]);
    assert_eq!(pgo.odometry_error_estimate(), [1.0; 4]);
}
