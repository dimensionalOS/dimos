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

from __future__ import annotations

import pickle

import pytest

from dimos.core.global_config import global_config

if global_config.simulation:
    pytest.skip("water demo is hardware-only", allow_module_level=True)

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1GrootWBCTaskParams
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import _verify_no_name_conflicts
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_water_demo import unitree_g1_water_demo

CAMERA = "realsensecamera"
BRIDGE = "rerunbridgemodule"


def _kwargs(instance: str) -> dict:
    return BlueprintConfigParser(unitree_g1_water_demo).parse().module_kwargs(instance)


def _stream(instance: str, port: str) -> str:
    return unitree_g1_water_demo.remapping_map.get((instance, port), port)


def camera_streams() -> list:
    (camera,) = [a for a in unitree_g1_water_demo.active_blueprints if a.name == CAMERA]
    return list(camera.streams)


def test_demo_module_set_is_exactly_what_the_robot_needs() -> None:
    names = {atom.name for atom in unitree_g1_water_demo.active_blueprints}
    assert names == {
        "g1wholebodyconnection",
        "ControlCoordinator",
        "realsensecamera",
        "manipulationmodule",
        "g1headcameratf",
        "markerdetectionstreammodule",
        "markertfmodule",
        "markerlatchmodule",
        "rerunbridgemodule",
        "rerunwebsocketserver",
        "websocketvismodule",
        "pointlio",
        "g1liobasepose",
        "posetargetobservationmodule",
        "wateringtaskmodule",
    }


def test_plant_pose_reaches_the_object_pose_contract() -> None:
    # Perception must publish the same message sim publishes from SimBodyPose,
    # so nothing downstream can tell the two apart.
    assert _stream("markerlatchmodule", "object_pose") == "object_pose"
    assert _stream("markerdetectionstreammodule", "color_image") == "color_image"
    assert _stream("markerlatchmodule", "detections") == "detections"
    assert _stream("posetargetobservationmodule", "object_pose") == "object_pose"
    assert _stream("wateringtaskmodule", "target_observation") == "target_observation"


def test_marker_pipeline_matches_the_printed_tags() -> None:
    detector = _kwargs("markerdetectionstreammodule")
    assert detector["marker_length_m"] == 0.15
    latch = _kwargs("markerlatchmodule")
    assert latch["marker_ids"] == [0, 1, 2]


def test_perception_is_world_relative_through_the_live_pelvis_tf() -> None:
    assert _kwargs("pointlio")["frame_id"] == "world"
    assert _kwargs("markerdetectionstreammodule")["world_frame"] == "world"
    assert _kwargs("markertfmodule")["world_frame"] == "world"
    assert _kwargs("markerlatchmodule")["frame_id"] == "world"
    assert _kwargs("g1headcameratf")["base_frame"] == "pelvis"
    assert _kwargs("g1liobasepose")["world_frame"] == "world"
    assert _kwargs("g1liobasepose")["base_frame"] == "pelvis"
    assert _stream("manipulationmodule", "odom") == "base_pose"


def test_camera_tf_chain_connects_to_the_urdf_mount() -> None:
    # G1HeadCameraTf publishes pelvis->d435_link; the camera hangs its own
    # optical chain off that same link. A mismatch drops every frame silently.
    assert _kwargs("g1headcameratf")["camera_frame"] == "d435_link"
    assert _kwargs(CAMERA)["base_frame_id"] == "d435_link"


def test_arm_trajectory_task_sits_above_the_servo_hold() -> None:
    (coordinator,) = [
        a for a in unitree_g1_water_demo.active_blueprints if a.name == "ControlCoordinator"
    ]
    priorities = {t.name: t.priority for t in coordinator.kwargs["tasks"]}
    assert priorities["groot_wbc"] == 50
    assert priorities["traj_arms"] == 30
    assert priorities["servo_arms"] == 10


def test_policy_comes_up_unarmed_and_dry_run() -> None:
    (coordinator,) = [
        a for a in unitree_g1_water_demo.active_blueprints if a.name == "ControlCoordinator"
    ]
    (groot,) = [t for t in coordinator.kwargs["tasks"] if t.name == "groot_wbc"]
    assert groot.params["auto_arm"] is False
    assert groot.params["auto_dry_run"] is True
    assert groot.params["default_ramp_seconds"] == 10.0
    assert groot.params["timeout"] == 0.25
    assert G1GrootWBCTaskParams.model_validate(groot.params).timeout == 0.25


def test_camera_publishes_compressed_color() -> None:
    (camera,) = [a for a in unitree_g1_water_demo.active_blueprints if a.name == CAMERA]
    compressed = [s for s in camera.streams if s.type is CompressedImage]
    assert [s.name for s in compressed] == ["color_compressed"]
    # Depth and pointcloud would swamp the Jetson; the demo only needs color.
    camera = _kwargs(CAMERA)
    assert camera["compress_color"] is True
    assert camera["enable_depth"] is False
    assert camera["enable_pointcloud"] is False
    # Compression is additive: marker detection still needs the raw pixels.
    assert [s.name for s in camera_streams() if s.type is Image] == [
        "color_image",
        "depth_image",
    ]


def test_camera_has_a_2d_view_to_render_into() -> None:
    # The inherited layout is 3D-only, so an image logged to world/color_compressed
    # arrives but is never displayed.
    bridge = _kwargs(BRIDGE)
    views = bridge["blueprint"]().root_container.contents
    assert [str(v.origin) for v in views] == ["world", "world/color_compressed"]


def test_rerun_caps_survive_config_parsing() -> None:
    # model_dump() used to turn the URDF factories into plain dicts here, which
    # crashed the bridge on worker deploy.
    bridge = _kwargs(BRIDGE)
    assert bridge["memory_limit"] == "5%"
    assert bridge["max_hz"]["world/color_compressed"] == 5.0
    assert callable(bridge["visual_override"]["world/g1/joints"])
    assert callable(next(iter(bridge["static"].values())))


def test_viewer_teleop_reaches_the_single_hardware_policy() -> None:
    assert _stream("rerunwebsocketserver", "tele_cmd_vel") == "tele_cmd_vel"
    assert _stream("websocketvismodule", "tele_cmd_vel") == "tele_cmd_vel"
    assert _stream("wateringtaskmodule", "operator_command") == "tele_cmd_vel"
    assert _stream("wateringtaskmodule", "base_command") == "cmd_vel"
    assert _stream("ControlCoordinator", "twist_command") == "cmd_vel"

    (coordinator,) = [
        atom
        for atom in unitree_g1_water_demo.active_blueprints
        if atom.name == "ControlCoordinator"
    ]
    tasks = {task.name: task for task in coordinator.kwargs["tasks"]}
    assert "teleop_groot_wbc" not in tasks

    # Hardware pins /g1/cmd_vel; sim uses bare /cmd_vel.
    transports = {
        name: transport.topic.topic
        for (name, message_type), transport in unitree_g1_water_demo.transport_map.items()
        if message_type is Twist
    }
    assert transports == {
        "cmd_vel": "/g1/cmd_vel",
        "tele_cmd_vel": "/g1/tele_cmd_vel",
    }


def test_no_nav_stack() -> None:
    # The coordinator is the only arbitration authority, so base motion must
    # not route through a second mux. Point-LIO is observer-only, not nav.
    names = {atom.name for atom in unitree_g1_water_demo.active_blueprints}
    assert not names & {
        "movementmanager",
        "costmapper",
        "replanningastarplanner",
        "raytracingvoxelmap",
    }


def test_pointlio_is_visible_and_separate_hardware_motion_steps_are_enabled() -> None:
    assert callable(_kwargs(BRIDGE)["visual_override"]["world/lidar"])
    watering = _kwargs("wateringtaskmodule")
    assert watering["motion_enabled"] is False
    assert watering["approach_motion_enabled"] is True
    assert watering["pour_motion_enabled"] is True
    assert watering["approach_holonomic"] is True
    assert watering["approach_max_linear"] == 0.15
    assert watering["approach_max_lateral"] == 0.12
    assert watering["approach_max_angular"] == 0.25
    assert _stream("wateringtaskmodule", "base_command") == "cmd_vel"


def test_demo_passes_runtime_stream_conflict_validation() -> None:
    # This is the exact preflight check used by ``dimos run`` before workers
    # start. Metadata-only blueprint tests do not catch cross-type name reuse.
    _verify_no_name_conflicts(unitree_g1_water_demo)


def test_worker_configuration_is_picklable() -> None:
    # Worker deployment sends module kwargs through multiprocessing.Pipe.
    # Lambdas in viewer overrides pass metadata tests but fail here at runtime.
    pickle.dumps(_kwargs(BRIDGE))


def test_viser_is_reachable_from_the_operator_laptop() -> None:
    # Viser defaults to 127.0.0.1, which is invisible from off the robot.
    (manip,) = [
        a for a in unitree_g1_water_demo.active_blueprints if a.name == "manipulationmodule"
    ]
    assert manip.kwargs["visualization"].backend == "viser"
    assert manip.kwargs["visualization"].host == "0.0.0.0"
    assert manip.kwargs["visualization"].port == 8095
    assert manip.module.dedicated_worker is True
