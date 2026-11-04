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


from types import SimpleNamespace

import numpy as np
import pytest

from dimos.imitation.policy.dreamlabs.backend import DreamlabsBackend
from dimos.imitation.policy.dreamlabs.config import DreamlabsArmConfig, DreamlabsBackendConfig
from dimos.robot.assets.model import RobotModel


@pytest.fixture
def policy_config_fixture(tmp_path):
    model_path = tmp_path / "arm.urdf"
    model_path.write_text("""<robot name="test"><link name="robot_root"/><link name="base"/><link name="tool"/>
    <joint name="mount" type="fixed"><parent link="robot_root"/><child link="base"/><origin xyz="1 2 3"/></joint>
    <joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" velocity="1" effort="1"/></joint></robot>""")
    return SimpleNamespace(
        fps=None,
        execution_steps=None,
        policy_path="dreamlabs/dream-1-beta",
        joint_names=["grip", "slide"],
        policy_joint_names=None,
        robot_type="fixture_arm",
        dreamlabs=DreamlabsBackendConfig(
            mode="eef",
            model=RobotModel.from_file(model_path),
            arms={
                "left": DreamlabsArmConfig(
                    joint_names=["slide"],
                    base_frame="base",
                    target_frame="tool",
                    gripper_joint="grip",
                    target_to_sdk_xyz=(0.1, 0.0, 0.0),
                    target_to_sdk_xyzw=(0.0, 0.0, 0.0, 1.0),
                )
            },
        ),
    )


@pytest.fixture
def images():
    return {
        role: np.full((32, 64, 3), color, dtype=np.uint8)
        for role, color in (
            ("overhead", [210, 0, 0]),
            ("wrist_l", [0, 220, 0]),
            ("wrist_r", [0, 0, 230]),
        )
    }


@pytest.fixture
def backend(policy_config_fixture, mocker):
    client = mocker.MagicMock()
    mocker.patch("dimos.imitation.policy.dreamlabs.backend.create_client", return_value=client)
    client.get_action.return_value = {
        "per_key": {
            "action.left_eef_xyz": [[0.4, 0.1, 0.2]] * 48,
            "action.left_eef_rot6": [[1.0, 0.0, 0.0, 0.0, 1.0, 0.0]] * 48,
            "action.left_grip": [[0.75]] * 48,
            "action.left_arm": None,
        }
    }
    instance = DreamlabsBackend(policy_config_fixture)
    try:
        yield instance, client
    finally:
        instance.close()
