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
from types import SimpleNamespace

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.robot.unitree.g1 import tool_pour_reach_map as reach_tool
from dimos.robot.unitree.g1.manip_stance import (
    POUR_Z,
    palm_position_for_spout,
)


@pytest.mark.parametrize(
    ("spout_offset", "expected_tip"),
    [
        ((0.0, 0.20, 0.0), -np.pi / 2),
        ((0.0, -0.20, 0.0), np.pi / 2),
    ],
)
def test_build_samples_the_runtime_spout_tcp_and_records_it(
    mocker: MockerFixture,
    spout_offset: tuple[float, float, float],
    expected_tip: float,
) -> None:
    capability = SimpleNamespace(robot="g1-right", model_id="test-map")
    mocker.patch.object(reach_tool.CapabilityMap, "load", return_value=capability)
    mocker.patch.object(reach_tool, "capability_scores", return_value=np.ones(1))
    solver = mocker.patch.object(reach_tool, "_Solver").return_value
    solver.solves.return_value = True

    data = reach_tool.build(
        Path("unused-capability-map.npz"),
        x_range=(0.4, 0.4),
        y_range=(-0.2, -0.2),
        spout_offset_in_palm=spout_offset,
    )

    tool_yaw = float(np.arctan2(-0.2, 0.4))
    tipped_rotation = reach_tool._rotation(tool_yaw, expected_tip)
    expected_palm = palm_position_for_spout(
        np.array([0.4, -0.2, POUR_Z]),
        tipped_rotation,
        spout_offset,
    )
    upright_call, tipped_call = solver.solves.call_args_list
    assert upright_call.args[0] == pytest.approx(expected_palm)
    assert upright_call.args[1] == pytest.approx(reach_tool._rotation(tool_yaw, 0.0))
    assert tipped_call.args[0] == pytest.approx(expected_palm)
    assert tipped_call.args[1] == pytest.approx(tipped_rotation)
    assert data["spout_offset_in_palm"] == list(spout_offset)
    assert data["tip_radians"] == pytest.approx(expected_tip)
    assert data["ik_upright"] == [[1]]
    assert data["ik_tipped"] == [[1]]
