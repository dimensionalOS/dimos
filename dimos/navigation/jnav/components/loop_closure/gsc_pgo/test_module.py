# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import hashlib
import json

from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import DEFAULTS, PGOConfig


def test_the_config_sent_to_rust_keeps_its_previous_defaults():
    sent = json.dumps(PGOConfig().to_config_dict(), sort_keys=True)
    # sha256 of the effective config before defaults.json (Python fields, Rust lidar_range defaults, blends_per_window = the old fixed 5)
    assert (
        hashlib.sha256(sent.encode()).hexdigest()
        == "a5e58d3e12d0f57db7f9a9899753a8756e12b06f6753c7a8fd5aaa8056231e6f"
    )


def test_partial_nested_overrides_keep_the_other_defaults():
    config = PGOConfig(
        lidar_range={"window_scans": 30}, deterministic_config={"solve_keyframe_power": 2.0}
    )
    assert config.lidar_range == {**DEFAULTS["lidar_range"], "window_scans": 30}
    assert config.deterministic_config == {
        **DEFAULTS["deterministic_config"],
        "solve_keyframe_power": 2.0,
    }


def test_a_stated_odometry_error_estimate_turns_the_lidar_range_estimate_off():
    assert PGOConfig().odometry_error_estimate_from_lidar_range
    config = PGOConfig(odometry_error_estimate_xy_per_sqrt_m=1e-3)
    assert not config.odometry_error_estimate_from_lidar_range
    assert (
        config.odometry_error_estimate_z_per_sqrt_m
        == DEFAULTS["lidar_range"]["mid360_odometry_error_estimate"][1][0]
    )
