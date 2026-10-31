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

from dimos.control.coordinator import ControlCoordinator
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.imitation.policy.dreamlabs.camera import DreamlabsCameraModule
from dimos.imitation.policy.module import PolicyModule, policy_class
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.manipulators.dual_openyam.blueprints.dreamlabs import build_dual_openyam_dreamlabs
from dimos.robot.manipulators.dual_openyam.blueprints.learning_collection import (
    build_dual_openyam_collection,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS


@pytest.mark.parametrize("mode", ["joint", "eef"])
def test_dreamlabs_blueprint_routes_policy_and_retains_manual_planning(mode):
    blueprint = build_dual_openyam_dreamlabs(mode=mode)
    coordinator = next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )
    policy = next(atom for atom in blueprint.blueprints if issubclass(atom.module, PolicyModule))
    tasks = {task.name: task for task in coordinator["tasks"]}
    assert "joint_trajectory" in tasks
    assert ("eef_trajectory" in tasks) is (mode == "eef")
    assert policy.kwargs["dreamlabs"].mode == mode
    assert policy.kwargs["joint_names"] == list(DUAL_OPENYAM_JOINTS)
    assert policy.kwargs["image_mapping"] == {
        "overhead_image": "overhead",
        "left_wrist_image": "wrist_l",
        "right_wrist_image": "wrist_r",
    }
    assert policy.module is policy_class(tuple(sorted(policy.kwargs["image_mapping"])))


def test_eef_blueprint_keeps_manual_planning_above_policy_priority():
    blueprint = build_dual_openyam_dreamlabs(mode="eef")
    coordinator = next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )
    tasks = {task.name: task for task in coordinator["tasks"]}
    assert tasks["eef_trajectory"].params["target_frames"] == [
        "left_grasp_frame",
        "right_grasp_frame",
    ]
    assert tasks["joint_trajectory"].priority > tasks["eef_trajectory"].priority


def test_camera_device_override_preserves_existing_capture_configuration(mocker):
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    for method in ("__init__", "serve_module_rpc", "start", "stop"):
        mocker.patch.object(LCMRPC, method, return_value=None)
    start = mocker.patch.object(WebcamModule, "start")
    module = DreamlabsCameraModule(
        rpc_transport=LCMRPC,
        device="/dev/camera-fixture",
        hardware={"width": 800, "height": 600, "fps": 30.0},
    )
    try:
        module.start()
        start.assert_called_once_with()
        assert module.config.hardware.camera_index == "/dev/camera-fixture"
        assert (
            module.config.hardware.width,
            module.config.hardware.height,
            module.config.hardware.fps,
        ) == (800, 600, 30.0)
    finally:
        module.stop()


def test_three_camera_collection_matches_rollout_inputs():
    collection = build_dual_openyam_collection(overhead=True)
    schema = next(
        atom.kwargs["recording_schema"]
        for atom in collection.blueprints
        if "recording_schema" in atom.kwargs
    )
    assert {
        feature.stream for feature in schema.observation.values() if feature.dtype == "video"
    } == {"overhead_image", "left_wrist_image", "right_wrist_image"}
    assert schema.action["action"].stream == "coordinator_joint_command"
    assert schema.action["action"].source_kind == "joint_position_updates"
    assert schema.observation["observation.state"].names == list(DUAL_OPENYAM_JOINTS)
