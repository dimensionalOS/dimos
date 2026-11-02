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

//! The odometry drift model (`big_picture_fit::motion_variances`) and the next run's starting drift.

use dimos_gsc_pgo::gsc_pgo::{Config, GscPgo};

#[test]
fn odometry_drift_is_a_random_walk_in_distance_and_turn_and_starts_the_next_run_from_the_config() {
    let config = Config {
        odom_xy_drift_per_sqrt_m: 0.01,
        odom_xy_drift_per_sqrt_rad: 0.02,
        ..Config::default()
    };
    // 4 m travelled while turning 0.25 rad: (1 cm)^2 x 4 + (2 cm)^2 x 0.25
    let [xy, ..] = dimos_gsc_pgo::big_picture_fit::motion_variances(&config, 4.0, 0.25);
    assert!((xy - (1e-4 * 4.0 + 4e-4 * 0.25)).abs() < 1e-12, "{xy}");
    // before any loop the next run's drift is the configured one, measured nowhere
    let (drift, measured) = GscPgo::new(config.clone()).odometry_drift_for_next_run();
    assert_eq!(drift[0], [0.01, 0.02]);
    assert_eq!(measured, [false; 4]);
}
