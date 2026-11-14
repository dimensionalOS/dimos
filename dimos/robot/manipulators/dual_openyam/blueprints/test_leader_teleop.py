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
import pytest

from dimos.robot.manipulators.dual_openyam.blueprints import leader_teleop
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_LEADERS,
    dual_openyam_model_config,
)

_MODEL = dual_openyam_model_config(robot="arm")


def test_the_leaders_drive_every_planning_model_joint_in_order() -> None:
    assert leader_teleop.DUAL_OPENYAM_LEADER_JOINTS == list(_MODEL.joint_names)


@pytest.mark.self_hosted
def test_leader_limits_match_the_urdf() -> None:
    loaded = _MODEL.model.load()
    for bus in DUAL_OPENYAM_LEADERS.buses:
        for motor in bus.motors:
            joint = loaded.get_joint(f"arm/{motor.joint}")
            assert joint is not None
            assert joint.lower == pytest.approx(motor.lower, abs=1e-4)
            assert joint.upper == pytest.approx(motor.upper, abs=1e-4)
