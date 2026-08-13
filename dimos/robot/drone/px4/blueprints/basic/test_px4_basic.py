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

from collections import Counter

import pytest

pytest.importorskip("gi", reason="PX4 camera tests require system GStreamer bindings")
pytest.importorskip("mavsdk", reason="PX4 blueprint tests require the drone extra")

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.web_human_input import WebInput
from dimos.core.module import ModuleBase
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.robot.drone.px4.blueprints.agentic.px4_agentic import px4_agentic
from dimos.robot.drone.px4.blueprints.basic.px4_basic import (
    gazebo_rerun_config,
    px4_basic,
    px4_gazebo_harmonic,
    rerun_config,
)
from dimos.robot.drone.px4.flight_control import FlightController
from dimos.robot.drone.px4.gstreamer_tee_camera import GsTeeCamera, GstInputFormat


def _module_counts(blueprint) -> Counter[type[ModuleBase]]:
    return Counter(atom.module for atom in blueprint.active_blueprints)


def test_px4_basic_contains_hardware_stack() -> None:
    counts = _module_counts(px4_basic)

    assert counts[PointLio] == 1
    assert counts[FlightController] == 1
    assert counts[GsTeeCamera] == 1


def test_px4_basic_rerun_config_visualizes_maps_and_drone_body() -> None:
    overrides = rerun_config["visual_override"]

    assert overrides["world/color_image"] is None
    assert callable(overrides["world/video_h264"])
    assert overrides["world/region_bounds"] is None
    assert rerun_config["max_hz"] == {
        "world/global_map": 0,
        "world/local_map": 0,
        "world/lidar": 5.0,
    }
    assert callable(rerun_config["static"]["world/drone/body"])


def test_px4_gazebo_uses_sitl_mavlink_and_rtp_h264() -> None:
    controller = next(
        atom for atom in px4_gazebo_harmonic.active_blueprints if atom.module is FlightController
    )
    camera_module = next(
        atom for atom in px4_gazebo_harmonic.active_blueprints if atom.module is GsTeeCamera
    )

    assert controller.kwargs["connection_url"] == "udpin://0.0.0.0:14540"
    assert camera_module.kwargs["input_format"] is GstInputFormat.H264
    assert "udpsrc port=5600" in camera_module.kwargs["input_pipeline"]


def test_px4_gazebo_rerun_config_is_video_only() -> None:
    assert gazebo_rerun_config["blueprint"].__name__ == "_gazebo_rerun_layout"
    assert gazebo_rerun_config["visual_override"]["world/color_image"] is None
    assert callable(gazebo_rerun_config["visual_override"]["world/video_h264"])
    assert "max_hz" not in gazebo_rerun_config
    assert "static" not in gazebo_rerun_config


def test_px4_basic_uses_sensor_frame_for_pointlio_odometry() -> None:
    pointlio = next(atom for atom in px4_basic.active_blueprints if atom.module is PointLio)

    assert pointlio.kwargs["sensor_frame_id"] == "mid360_link"


def test_px4_agentic_adds_mcp_modules_to_hardware_stack() -> None:
    counts = _module_counts(px4_agentic)

    assert counts[PointLio] == 1
    assert counts[FlightController] == 1
    assert counts[McpServer] == 1
    assert counts[McpClient] == 1
    assert counts[WebInput] == 1
