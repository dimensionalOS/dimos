# Copyright 2025-2026 Dimensional Inc.
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
from typing import Any, TypeGuard

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import Blueprint, TransportSpec
from dimos.core.module import ModuleBase
from dimos.core.stream import Transport
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.drone.px4.blueprints.basic.debug import (
    px4_flight_debug,
    px4_gazebo_harmonic,
    px4_mapping_debug,
    px4_video_debug,
)
from dimos.robot.drone.px4.flight_control import FlightController
from dimos.robot.drone.px4.gstreamer.gstreamer_api import GstSource, GstSourceConfig
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import Px4GstTeeCamera
from dimos.robot.drone.px4.mid360_mount_tf import Mid360MountStaticTf
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def _module_counts(blueprint: Blueprint) -> Counter[type[ModuleBase]]:
    return Counter(atom.module for atom in blueprint.active_blueprints)


def test_video_debug_contains_camera_and_remote_viewer() -> None:
    counts = _module_counts(px4_video_debug)

    assert counts[Px4GstTeeCamera] == 1
    assert counts[RerunBridgeModule] == 1
    raw_transport = px4_video_debug.transport_map[("color_image", Image)]
    h264_transport = px4_video_debug.transport_map[("video_h264", CompressedVideo)]
    assert _is_transport_spec(raw_transport)
    assert _is_transport_spec(h264_transport)
    assert raw_transport.cls is pSHMTransport
    assert raw_transport.args == ("/color_image",)
    assert raw_transport.kwargs == {"default_capacity": DEFAULT_CAPACITY_COLOR_IMAGE}
    assert h264_transport.cls is LCMTransport
    assert h264_transport.args == ("/video_h264", CompressedVideo)


def test_flight_debug_contains_controller_and_viewer_teleop_route() -> None:
    counts = _module_counts(px4_flight_debug)

    assert counts[FlightController] == 1
    assert counts[RerunBridgeModule] == 1
    assert px4_flight_debug.remapping_map[(RerunWebSocketServer.name, "tele_cmd_vel")] == "cmd_vel"


def test_gazebo_harmonic_contains_sitl_controller_and_udp_camera() -> None:
    counts = _module_counts(px4_gazebo_harmonic)

    assert counts[FlightController] == 1
    assert counts[Px4GstTeeCamera] == 1
    controller_atom = next(
        atom for atom in px4_gazebo_harmonic.active_blueprints if atom.module is FlightController
    )
    camera_atom = next(
        atom for atom in px4_gazebo_harmonic.active_blueprints if atom.module is Px4GstTeeCamera
    )
    assert controller_atom.kwargs["mavsdk_config"].connection_url == "udpin://0.0.0.0:14540"
    assert camera_atom.kwargs["source_config"] == GstSourceConfig(
        source=GstSource.UDP_RTP_H264,
        udp_port=5600,
    )


def test_mapping_debug_contains_pointlio_mount_map_and_remote_viewer() -> None:
    counts = _module_counts(px4_mapping_debug)

    assert counts[PointLio] == 1
    assert counts[Mid360MountStaticTf] == 1
    assert counts[RayTracingVoxelMap] == 1
    assert counts[RerunBridgeModule] == 1


def _is_transport_spec(value: TransportSpec | Transport[Any]) -> TypeGuard[TransportSpec]:
    return isinstance(value, TransportSpec)
