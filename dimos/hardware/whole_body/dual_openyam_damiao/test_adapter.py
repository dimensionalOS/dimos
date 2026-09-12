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

from collections.abc import Iterator

import can_motor_control
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.dual_openyam_damiao.adapter import (
    DualOpenYamDamiaoAdapter,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_JOINTS

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def adapter(mocker: MockerFixture) -> Iterator[DualOpenYamDamiaoAdapter]:
    mocker.patch.object(
        DualOpenYamDamiaoAdapter,
        "_make_can_bus",
        side_effect=lambda name: can_motor_control.MockCanBus(name),
    )
    result = DualOpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            bus_devices={"left": "can8", "right": "can9"},
            gravity_comp=False,
        )
    )
    yield result
    result.disconnect()


def test_adapter_connects_complete_dual_yam_topology(
    adapter: DualOpenYamDamiaoAdapter,
) -> None:
    assert adapter.connect()

    robot = adapter._robot
    assert robot.bus_names() == ["left", "right"]
    assert robot.group_names() == ["left_arm", "right_arm", "left_gripper", "right_gripper"]
    assert len(robot["left_arm"]) == 6
    assert len(robot["right_arm"]) == 6
    assert list(adapter.joint_names) == DUAL_OPENYAM_JOINTS
    assert adapter._pin_model.nq == 12
    assert adapter._pin_model.nv == 12
    assert tuple(str(name) for name in adapter._pin_model.names[1:]) == (
        *[f"left_joint{index}" for index in range(1, 7)],
        *[f"right_joint{index}" for index in range(1, 7)],
    )
