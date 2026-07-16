# Copyright 2025-2026 Dimensional Inc.
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

from pathlib import Path

from dimos.robot.get_all_blueprints import get_by_name
from dimos.utils.config import load_config_mapping

M20_SIMPLE_NAV_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config/m20_simple_nav.yaml"

EXPECTED_PLANNER_CONFIG = {
    "path_length_weight": 1.0,
    "path_cell_cost_weight": 3.0,
    "publish_raw_path": True,
    "constrained_path_smoothing_enabled": True,
    "path_smoothing_iterations": 40,
    "path_smoothing_data_weight": 0.02,
    "path_smoothing_smoothness_weight": 0.45,
    "path_smoothing_max_deviation_m": 0.1,
    "path_smoothing_collision_sample_spacing_m": 0.05,
    "path_smoothing_max_cost_increase": 2.0,
    "path_smoothing_backtracking_factor": 0.5,
    "path_smoothing_max_backtracking_steps": 3,
    "path_resample_spacing_m": 0.1,
}


def test_m20_simple_nav_profile_matches_blueprint_config() -> None:
    payload = load_config_mapping(M20_SIMPLE_NAV_CONFIG_PATH)

    assert set(payload) == {"replanningastarplanner"}
    assert payload["replanningastarplanner"] == EXPECTED_PLANNER_CONFIG

    blueprint_config = get_by_name("m20-simple-nav").config()
    validated = blueprint_config.model_validate(payload)
    planner = validated.replanningastarplanner

    assert planner is not None
    assert planner.model_dump(include=set(EXPECTED_PLANNER_CONFIG)) == EXPECTED_PLANNER_CONFIG


def test_m20_simple_nav_profile_enables_diagnostics_and_smoothing() -> None:
    payload = load_config_mapping(M20_SIMPLE_NAV_CONFIG_PATH)
    planner = payload["replanningastarplanner"]

    assert planner["publish_raw_path"] is True
    assert planner["constrained_path_smoothing_enabled"] is True
