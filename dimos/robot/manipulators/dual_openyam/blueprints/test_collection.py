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

from dimos.core.coordination.module_coordinator import _is_name_unique
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionModule
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.collection import (
    DUAL_OPENYAM_CAMERAS,
    DUAL_OPENYAM_COLLECTION_JOINTS,
    dual_openyam_collection_camera,
    dual_openyam_collection_cameras,
    dual_openyam_collection_profile,
)
from dimos.robot.manipulators.dual_openyam.blueprints.leader_collection import (
    dual_openyam_leader_collection,
)
from dimos.robot.manipulators.dual_openyam.blueprints.leader_teleop import (
    teleop_dual_openyam_leader,
)
from dimos.teleop.yam_leader.teleop_module import YamLeaderTeleopModule


def test_profile_records_three_images_and_the_coordinator_joint_targets() -> None:
    profile = dual_openyam_collection_profile("dual-openyam-leader")

    assert set(profile.observations) == {
        "observation.images.overhead",
        "observation.images.left_wrist",
        "observation.images.right_wrist",
        "observation.state",
    }
    for camera in DUAL_OPENYAM_CAMERAS:
        image = profile.observations[f"observation.images.{camera}"]
        assert (image.stream, image.dtype, image.shape) == (
            f"{camera}_color_image",
            "video",
            (480, 640, 3),
        )
    state = profile.observations["observation.state"]
    assert (state.stream, state.shape) == ("coordinator_joint_state", (14,))
    assert state.names == list(DUAL_OPENYAM_COLLECTION_JOINTS)
    assert state.names[0] == "arm/left_joint1"
    assert state.names[-2:] == ["arm/left_gripper", "arm/right_gripper"]
    (action,) = profile.actions.values()
    assert (action.stream, action.source_kind) == (
        "coordinator_joint_command",
        "joint_position_updates",
    )
    assert (action.shape, action.names) == (state.shape, state.names)
    assert profile.metadata == {"name": "dual-openyam-leader", "robot_type": "dual_openyam"}


def test_camera_streams_carry_the_camera_name_and_serials_come_from_the_run_config() -> None:
    camera = dual_openyam_collection_camera("left_wrist")
    (atom,) = camera.blueprints

    assert atom.module is RealSenseCamera
    assert atom.kwargs["instance_name"] == "left_wrist_camera"
    assert atom.kwargs["enable_depth"] is False
    assert "serial_number" not in atom.kwargs
    assert camera.remapping_map[("left_wrist_camera", "color_image")] == "left_wrist_color_image"
    assert camera.remapping_map[("left_wrist_camera", "camera_info")] == "left_wrist_camera_info"
    assert [c.blueprints[0].kwargs["instance_name"] for c in dual_openyam_collection_cameras()] == [
        f"{name}_camera" for name in DUAL_OPENYAM_CAMERAS
    ]
    with pytest.raises(ValueError):
        dual_openyam_collection_camera("chest")


def test_leader_collection_composes_the_recorder_episodes_leader_and_cameras() -> None:
    blueprint = dual_openyam_leader_collection
    modules = [atom.module for atom in blueprint.blueprints]

    assert modules[:2] == [CollectionModule, EpisodeMonitorModule]
    assert modules.count(RealSenseCamera) == len(DUAL_OPENYAM_CAMERAS)
    assert set(modules) >= {YamLeaderTeleopModule, DamiaoCoordinator}
    assert [atom.module for atom in teleop_dual_openyam_leader.blueprints] == [
        module
        for module in modules
        if module not in (CollectionModule, EpisodeMonitorModule, RealSenseCamera)
    ]
    (episodes,) = [atom for atom in blueprint.blueprints if atom.module is EpisodeMonitorModule]
    assert episodes.kwargs["instance_name"] == "episodes"
    for name in DUAL_OPENYAM_CAMERAS:
        assert blueprint.remapping_map[(f"{name}_camera", "color_image")] == f"{name}_color_image"
        assert _is_name_unique(blueprint, f"{name}_color_image")
    profile = dual_openyam_collection_profile("dual-openyam-leader")
    (recorder,) = [atom for atom in blueprint.blueprints if atom.module is CollectionModule]
    assert recorder.kwargs["recording_schema"] == profile.to_schema()
    assert set(recorder.kwargs["input_types"]) == set(profile.input_types())
