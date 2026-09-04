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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.module_coordinator import _materialize_transports
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.core.transport_factory import make_transport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_teleop import r1pro_teleop
from dimos.robot.galaxea.r1pro.blueprints.manipulation.r1pro_manipulation import (
    r1pro_manipulation,
)
from dimos.robot.galaxea.r1pro.connection import R1ProConnection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def test_color_streams_are_compressed() -> None:
    atom = R1ProConnection.blueprint().active_blueprints[0]
    color_streams = [stream for stream in atom.streams if stream.name.endswith("_color")]

    assert color_streams
    assert all(stream.type is CompressedImage for stream in color_streams)


def test_control_adapters_follow_the_active_transport() -> None:
    blueprint = r1pro_control()
    atom = next(bp for bp in blueprint.active_blueprints if bp.module is ControlCoordinator)

    assert blueprint.global_config_overrides["transport"] == "zenoh"
    assert all(
        component.adapter_kwargs["transport_cls"] is make_transport
        for component in atom.kwargs["hardware"]
    )


def test_control_streams_use_zenoh() -> None:
    previous = global_config.transport
    try:
        global_config.update(transport="zenoh")
        transports = _materialize_transports(r1pro_control(), {})
    finally:
        global_config.update(transport=previous)

    assert type(transports[("chassis_cmd_vel", Twist)]) is ZenohTransport
    assert type(transports[("head_left_color", CompressedImage)]) is ZenohTransport


def test_teleop_routes_directly_to_the_coordinator() -> None:
    modules = {atom.module for atom in r1pro_teleop.active_blueprints}

    assert MovementManager not in modules
    assert (
        r1pro_teleop.remapping_map[(RerunWebSocketServer.name, "tele_cmd_vel")] == "twist_command"
    )


def test_manipulation_uses_remote_viser() -> None:
    atom = next(
        bp for bp in r1pro_manipulation.active_blueprints if bp.module is ManipulationModule
    )

    visualization = atom.kwargs["visualization"]
    assert isinstance(visualization, ViserVisualizationConfig)
    assert visualization.host == "0.0.0.0"
