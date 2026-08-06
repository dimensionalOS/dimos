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
from typing import Any, TypeGuard

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import TransportSpec
from dimos.core.module import ModuleBase
from dimos.core.stream import Transport
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.drone.px4 import camera
from dimos.robot.drone.px4.blueprints.basic.px4_basic import px4_basic
from dimos.robot.drone.px4.flight_control import FlightController
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import Px4GstTeeCamera
from dimos.robot.drone.px4.mid360_mount_tf import Mid360MountStaticTf
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def test_px4_basic_contains_all_required_modules() -> None:
    module_counts: Counter[type[ModuleBase]] = Counter(
        atom.module for atom in px4_basic.active_blueprints
    )

    assert module_counts[PointLio] == 1
    assert module_counts[Mid360MountStaticTf] == 1
    assert module_counts[FlightController] == 1
    assert module_counts[Px4GstTeeCamera] == 1
    assert module_counts[RayTracingVoxelMap] == 1
    assert module_counts[RerunBridgeModule] == 1
    camera_atom = next(
        atom for atom in px4_basic.active_blueprints if atom.module is Px4GstTeeCamera
    )
    assert "h264_sink" not in camera_atom.kwargs


def test_px4_basic_suppresses_bridged_raw_color_image_in_h264_mode() -> None:
    bridge_atom = next(
        atom for atom in px4_basic.active_blueprints if atom.module is RerunBridgeModule
    )

    assert bridge_atom.kwargs["blueprint"] is camera.px4_camera_layout
    overrides = bridge_atom.kwargs["visual_override"]
    assert overrides["world/region_bounds"] is None
    assert overrides["world/color_image"] is None
    assert callable(overrides["world/camera_info"])
    assert callable(overrides["world/video_h264"])


def test_px4_basic_pins_raw_pshm_and_typed_h264_lcm_transports() -> None:
    raw_transport = px4_basic.transport_map[("color_image", Image)]
    h264_transport = px4_basic.transport_map[("video_h264", CompressedVideo)]

    assert _is_transport_spec(raw_transport)
    assert _is_transport_spec(h264_transport)
    assert raw_transport.cls is pSHMTransport
    assert raw_transport.args == ("/color_image",)
    assert raw_transport.kwargs == {"default_capacity": DEFAULT_CAPACITY_COLOR_IMAGE}
    assert h264_transport.cls is LCMTransport
    assert h264_transport.args == ("/video_h264", CompressedVideo)


def test_px4_basic_routes_rerun_teleop_to_flight_control() -> None:
    assert px4_basic.remapping_map[(RerunWebSocketServer.name, "tele_cmd_vel")] == "cmd_vel"


def _is_transport_spec(value: TransportSpec | Transport[Any]) -> TypeGuard[TransportSpec]:
    return isinstance(value, TransportSpec)
