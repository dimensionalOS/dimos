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

import math

import pytest

from dimos.hardware.manipulators.mock.adapter import MockAdapter
from dimos.hardware.spec import JointLimits


@pytest.mark.parametrize("dof", [6, 7])
def test_reads_and_limits_cover_all_configured_joints(dof: int) -> None:
    adapter = MockAdapter(dof=dof)

    assert adapter.get_dof() == dof
    assert adapter.read_joint_positions() == [0.0] * dof
    assert adapter.read_joint_velocities() == [0.0] * dof
    assert adapter.read_joint_efforts() == [0.0] * dof
    assert adapter.get_limits() == JointLimits(
        position_lower=[-math.pi] * dof,
        position_upper=[math.pi] * dof,
        velocity_max=[1.0] * dof,
    )


def test_stop_clears_every_joint_velocity() -> None:
    adapter = MockAdapter(dof=7)
    assert adapter.write_joint_velocities([float(i) for i in range(7)])

    assert adapter.write_stop()

    assert adapter.read_joint_velocities() == [0.0] * 7


def test_deactivate_stops_and_disables() -> None:
    adapter = MockAdapter(dof=7)
    assert adapter.activate()
    assert adapter.write_joint_velocities([1.0] * 7)

    assert adapter.deactivate()

    assert adapter.read_joint_velocities() == [0.0] * 7
    assert adapter.read_state()["state"] == 1
