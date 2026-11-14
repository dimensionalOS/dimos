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

from pathlib import Path

import numpy as np
import pytest

from dimos.robot.manipulators.openyam.policy.reference import ReferenceRow
from dimos.robot.manipulators.openyam.policy.tcp import MjcfTcpFk, quat_angle_deg


@pytest.mark.parametrize("angle", [0.0, 0.73])
def test_tcp_native_wxyz_goal_and_copy(tmp_path: Path, angle: float) -> None:
    mujoco = pytest.importorskip("mujoco")
    plant = tmp_path / "plant.xml"
    plant.write_text(
        '<mujoco><worldbody><body name="arm_tcp">'
        '<joint name="arm_joint_1" axis="0 0 1"/>'
        '<geom type="sphere" size="0.01" mass="1"/>'
        "</body></worldbody></mujoco>"
    )
    fk = MjcfTcpFk(plant, joint_names=("arm_joint_1",))
    q = np.array([angle], dtype=np.float32)
    position, quaternion = fk(q)
    expected = np.array([np.cos(float(q[0]) / 2), 0, 0, np.sin(float(q[0]) / 2)])
    np.testing.assert_allclose(quaternion, expected, atol=1e-14)
    np.testing.assert_array_equal(quaternion, fk.data.xquat[fk._body])
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, quaternion)
    np.testing.assert_allclose(matrix, fk.data.xmat[fk._body], atol=1e-14)
    row = ReferenceRow(0, 0.0, q, np.zeros_like(q), position, quaternion)
    assert quat_angle_deg(row.goal_motion[3:7], expected) < 1e-5
    assert quat_angle_deg(row.goal_motion[3:7], -expected) < 1e-5
    saved_position, saved_quaternion = position.copy(), quaternion.copy()
    assert not np.shares_memory(quaternion, fk.data.xquat)
    assert not np.shares_memory(position, fk.data.xpos)
    fk(np.array([angle + 0.4]))
    np.testing.assert_array_equal(position, saved_position)
    np.testing.assert_array_equal(quaternion, saved_quaternion)
