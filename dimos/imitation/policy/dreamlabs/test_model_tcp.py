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

"""Rigid model TCP frames and legacy tool transforms use the same SDK convention."""

import math

import numpy as np
import pytest

from dimos.imitation.policy.dreamlabs.config import DreamlabsArmConfig
from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics
from dimos.robot.assets.model import RobotModel


@pytest.fixture
def frame_config(policy_config_fixture):
    config = policy_config_fixture.dreamlabs
    model = config.model.with_fixed_frame("grasp", "tool", xyz=(0.0, 0.0, 0.08))
    model = model.with_fixed_frame(
        "sdk_tcp", "tool", xyz=(0.2, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2)
    )
    arm = DreamlabsArmConfig(
        joint_names=["slide"],
        base_frame="base",
        target_frame="grasp",
        gripper_joint="grip",
        sdk_tcp_frame="sdk_tcp",
    )
    return config.model_copy(update={"model": model, "arms": {"left": arm}})


@pytest.mark.parametrize("position", [0.0, 0.4, 0.8])
def test_sibling_tcp_frame_converts_translation_rotation_and_mount(frame_config, position):
    fk = DreamlabsKinematics(frame_config)
    positions = {"slide": position, "grip": 0.25}
    observation = fk.observation(positions, "move")
    np.testing.assert_allclose(observation["state.left_eef_xyz"], [position + 0.2, 0, 0], atol=1e-7)
    np.testing.assert_allclose(observation["state.left_eef_rot6"], [0, 1, 0, -1, 0, 0], atol=1e-7)
    pose = fk.canonical_pose("left", fk.sdk_pose("left", positions))
    np.testing.assert_allclose([pose.x, pose.y, pose.z], [1 + position, 2, 3.08])
    np.testing.assert_allclose(pose.orientation.to_numpy(), [0, 0, 0, 1], atol=1e-7)


def test_named_tcp_agrees_with_existing_explicit_transform(frame_config):
    legacy_arm = frame_config.arms["left"].model_dump()
    legacy_arm.update(
        sdk_tcp_frame=None,
        target_to_sdk_xyz=(0.2, 0.0, -0.08),
        target_to_sdk_xyzw=(0.0, 0.0, 2**-0.5, 2**-0.5),
    )
    legacy = DreamlabsKinematics(
        frame_config.model_copy(update={"arms": {"left": DreamlabsArmConfig(**legacy_arm)}})
    )
    framed = DreamlabsKinematics(frame_config)
    positions = {"slide": 0.35}
    np.testing.assert_allclose(
        framed.sdk_pose("left", positions), legacy.sdk_pose("left", positions), atol=1e-12
    )


@pytest.mark.parametrize("tcp,error", [("missing", "Unknown"), ("base", "rigidly connected")])
def test_tcp_frame_must_exist_and_be_rigidly_connected(frame_config, tcp, error):
    arm = frame_config.arms["left"].model_copy(update={"sdk_tcp_frame": tcp})
    with pytest.raises(ValueError, match=error):
        DreamlabsKinematics(frame_config.model_copy(update={"arms": {"left": arm}}))


def test_target_itself_can_define_the_protocol_tcp(frame_config):
    arm = frame_config.arms["left"].model_copy(update={"sdk_tcp_frame": "grasp"})
    fk = DreamlabsKinematics(frame_config.model_copy(update={"arms": {"left": arm}}))
    np.testing.assert_allclose(fk.sdk_pose("left", {"slide": 0.4})[:3, 3], [0.4, 0, 0.08])


@pytest.mark.parametrize(
    "override",
    [{"target_to_sdk_xyz": (0, 0, 0)}, {"target_to_sdk_xyzw": (0, 0, 0, 1)}],
)
def test_tcp_frame_cannot_be_combined_with_numeric_override(frame_config, override):
    values = frame_config.arms["left"].model_dump() | override
    with pytest.raises(ValueError, match="not both"):
        DreamlabsArmConfig(**values)


def test_missing_tcp_definition_fails_before_loading_assets(frame_config, mocker):
    load = mocker.patch.object(RobotModel, "load")
    arm = frame_config.arms["left"].model_copy(update={"sdk_tcp_frame": None})
    with pytest.raises(ValueError, match="model TCP frame or calibrated"):
        DreamlabsKinematics(frame_config.model_copy(update={"arms": {"left": arm}}))
    load.assert_not_called()


@pytest.mark.parametrize("offset", [float("nan"), float("inf")])
def test_model_tcp_rejects_nonfinite_geometry(frame_config, offset):
    model = frame_config.model.with_fixed_frame("invalid_tcp", "tool", xyz=(offset, 0.0, 0.0))
    arm = frame_config.arms["left"].model_copy(update={"sdk_tcp_frame": "invalid_tcp"})
    with pytest.raises(ValueError, match="finite"):
        DreamlabsKinematics(frame_config.model_copy(update={"model": model, "arms": {"left": arm}}))
