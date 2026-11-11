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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import _is_name_unique
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionModule
from dimos.imitation.dataprep.core import resolve_field
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.dual_openyam.blueprints.leader_collection import (
    DUAL_OPENYAM_CAMERAS,
    DUAL_OPENYAM_COLLECTION_JOINTS,
    dual_openyam_collection_camera,
    dual_openyam_leader_collection_profile,
    learning_collect_leader_dual_openyam,
)
from dimos.robot.manipulators.dual_openyam.blueprints.leader_teleop import (
    dual_openyam_leader_teleop,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.teleop.leader.module import LeaderTeleopModule


def test_profile_records_three_images_and_the_coordinator_joints_by_name() -> None:
    profile = dual_openyam_leader_collection_profile()
    assert set(profile.observations) == {
        *[f"observation.images.{camera}" for camera in DUAL_OPENYAM_CAMERAS],
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
    assert state.names == [f"arm/{name}" for name in DUAL_OPENYAM.joint_names]
    (action,) = profile.actions.values()
    assert (action.stream, action.source_kind) == (
        "arm_position_command",
        "joint_position_updates",
    )
    assert (action.shape, action.names) == (state.shape, state.names)
    assert profile.metadata == {"name": "dual-openyam-leader", "robot_type": "dual_openyam"}

    schema = profile.to_schema()
    message = JointState(
        name=list(reversed(DUAL_OPENYAM_COLLECTION_JOINTS)), position=list(reversed(range(14)))
    )
    np.testing.assert_array_equal(resolve_field(message, schema.action["action"]), np.arange(14))


def test_camera_streams_carry_the_rig_name_and_serials_come_from_the_run_config() -> None:
    camera = dual_openyam_collection_camera("left_wrist")
    (atom,) = camera.blueprints
    assert atom.module is RealSenseCamera
    assert atom.kwargs["instance_name"] == "left_wrist_camera"
    assert atom.kwargs["enable_depth"] is False
    assert "serial_number" not in atom.kwargs
    assert camera.remapping_map[("left_wrist_camera", "color_image")] == "left_wrist_color_image"
    with pytest.raises(ValueError):
        dual_openyam_collection_camera("chest")


def test_collection_composes_the_recorder_episodes_leader_teleop_and_cameras(
    tmp_path: Path,
) -> None:
    blueprint = learning_collect_leader_dual_openyam
    modules = [atom.module for atom in blueprint.blueprints]
    assert modules[:2] == [CollectionModule, EpisodeMonitorModule]
    assert modules.count(RealSenseCamera) == len(DUAL_OPENYAM_CAMERAS)
    assert [atom.module for atom in dual_openyam_leader_teleop.blueprints] == [
        module
        for module in modules
        if module not in (CollectionModule, EpisodeMonitorModule, RealSenseCamera)
    ]
    for name in DUAL_OPENYAM_CAMERAS:
        assert blueprint.remapping_map[(f"{name}_camera", "color_image")] == f"{name}_color_image"
        assert _is_name_unique(blueprint, f"{name}_color_image")
    (recorder,) = [atom for atom in blueprint.blueprints if atom.module is CollectionModule]
    profile = dual_openyam_leader_collection_profile()
    assert recorder.kwargs["recording_schema"] == profile.to_schema()
    assert set(recorder.kwargs["input_types"]) == set(profile.input_types())

    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--recorder.recording",
            str(tmp_path / "leader"),
            "--episodes.task",
            "stack the cups",
            "--left-can-port",
            "follower_l",
            "--right-can-port",
            "follower_r",
            "--leaderteleopmodule.port-left",
            "/dev/ttyACM1",
            "--left-wrist-camera.serial-number",
            "000000000001",
        ],
        environ={},
    )
    assert parsed.module_kwargs("episodes")["task"] == "stack the cups"
    assert parsed.module_kwargs(LeaderTeleopModule.name)["port_left"] == "/dev/ttyACM1"
    assert parsed.module_configs["left_wrist_camera"]["serial_number"] == "000000000001"


def test_blueprint_is_registered() -> None:
    assert all_blueprints["learning-collect-leader-dual-openyam"].endswith(
        "leader_collection:learning_collect_leader_dual_openyam"
    )
