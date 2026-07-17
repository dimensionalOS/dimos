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

"""Tests for the Galaxea A1Z planning model configuration."""

from dimos.robot.manipulators.a1z.config import (
    A1Z_G1Z_MODEL_PATH,
    A1Z_MEASURED_STATE_LIMIT_TOLERANCE,
    make_a1z_model_config,
)


def test_a1z_model_allows_bounded_home_encoder_overshoot() -> None:
    config = make_a1z_model_config(has_gripper=False)

    assert A1Z_MEASURED_STATE_LIMIT_TOLERANCE == 0.01
    assert config.measured_state_limit_tolerance == A1Z_MEASURED_STATE_LIMIT_TOLERANCE


def test_a1z_model_can_include_gripper_without_enabling_gripper_control() -> None:
    config = make_a1z_model_config(has_gripper=True, enable_gripper_control=False)

    assert config.model_path == A1Z_G1Z_MODEL_PATH
    assert config.end_effector_link == "gripper_eef_link"
    assert config.gripper_hardware_id is None
