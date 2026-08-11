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

import pytest

from dimos.core.global_config import global_config

if global_config.simulation:
    pytest.skip("water demo is hardware-only", allow_module_level=True)

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_water_demo import unitree_g1_water_demo

CAMERA = "realsensecamera"
BRIDGE = "rerunbridgemodule"


def _kwargs(instance: str) -> dict:
    return BlueprintConfigParser(unitree_g1_water_demo).parse().module_kwargs(instance)


def _stream(instance: str, port: str) -> str:
    return unitree_g1_water_demo.remapping_map.get((instance, port), port)


def test_demo_module_set_is_exactly_what_the_robot_needs() -> None:
    names = {atom.name for atom in unitree_g1_water_demo.active_blueprints}
    assert names == {
        "g1wholebodyconnection",
        "ControlCoordinator",
        "realsensecamera",
        "manipulationmodule",
        "rerunbridgemodule",
        "rerunwebsocketserver",
        "websocketvismodule",
    }


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


def test_camera_publishes_compressed_color() -> None:
    (camera,) = [a for a in unitree_g1_water_demo.active_blueprints if a.name == CAMERA]
    compressed = [s for s in camera.streams if s.type is CompressedImage]
    assert [s.name for s in compressed] == ["color_compressed"]
    # Depth and pointcloud would swamp the Jetson; the demo only needs color.
    assert _kwargs(CAMERA) == {
        "enable_depth": False,
        "enable_pointcloud": False,
        "compress_color": True,
    }


def test_rerun_caps_survive_config_parsing() -> None:
    # model_dump() used to turn the URDF factories into plain dicts here, which
    # crashed the bridge on worker deploy.
    bridge = _kwargs(BRIDGE)
    assert bridge["memory_limit"] == "5%"
    assert bridge["max_hz"]["world/color_compressed"] == 5.0
    assert callable(bridge["visual_override"]["world/g1/joints"])
    assert callable(next(iter(bridge["static"].values())))


def test_viewer_teleop_reaches_the_policy_directly() -> None:
    # Viewer -> cmd_vel -> coordinator, with nothing muxing in between.
    assert _stream("rerunwebsocketserver", "tele_cmd_vel") == "cmd_vel"
    assert _stream("websocketvismodule", "tele_cmd_vel") == "cmd_vel"
    assert _stream("ControlCoordinator", "twist_command") == "cmd_vel"

    # Hardware pins /g1/cmd_vel; sim uses bare /cmd_vel.
    (transport,) = [t for (_n, ty), t in unitree_g1_water_demo.transport_map.items() if ty is Twist]
    assert transport.topic.topic == "/g1/cmd_vel"


def test_no_nav_stack() -> None:
    # The coordinator is the only arbitration authority, so base motion must
    # not route through a second mux; and the demo carries no lidar/nav.
    names = {atom.name for atom in unitree_g1_water_demo.active_blueprints}
    assert not names & {
        "movementmanager",
        "costmapper",
        "replanningastarplanner",
        "pointlio",
        "raytracingvoxelmap",
    }


def test_viser_is_off_on_the_robot() -> None:
    (manip,) = [
        a for a in unitree_g1_water_demo.active_blueprints if a.name == "manipulationmodule"
    ]
    assert "visualization" not in manip.kwargs
