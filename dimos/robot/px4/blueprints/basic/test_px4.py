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

"""Wiring of the basic PX4 blueprints."""

from __future__ import annotations

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, TransportSpec
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.px4.blueprints.basic.px4 import px4, px4_control
from dimos.robot.px4.blueprints.basic.px4_a8 import px4_a8
from dimos.robot.px4.blueprints.basic.px4_sitl import px4_sitl
from dimos.robot.px4.config import ROUTER_MAV_URL, SITL_MAV_URL
from dimos.robot.px4.connection import Px4DroneConnection, Px4DroneConnectionConfig


def _connection_config(blueprint: Blueprint) -> Px4DroneConnectionConfig:
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    atom = next(a for a in blueprint.active_blueprints if issubclass(a.module, Px4DroneConnection))
    return Px4DroneConnectionConfig.model_validate(parsed.module_kwargs(atom.name))


def _qos(blueprint: Blueprint, key: tuple[str, type]) -> object:
    spec = blueprint.transport_map[key]
    assert isinstance(spec, TransportSpec)
    return spec.args[0].qos


def test_the_viewer_keyboard_drives_cmd_vel() -> None:
    assert "cmd_vel" in px4.remapping_map.values()
    assert _qos(px4_control(), ("cmd_vel", Twist)) is None  # reliable: no key press is dropped


def test_vehicle_streams_are_latest_wins() -> None:
    control = px4_control()
    assert _qos(control, ("odometry", Odometry)) is not None
    assert _qos(control, ("tf", TFMessage)) is not None


def test_sitl_points_at_px4_sitl_and_the_aircraft_at_the_router() -> None:
    sitl, aircraft = _connection_config(px4_sitl), _connection_config(px4)
    assert (sitl.mav_url, sitl.sitl) == (SITL_MAV_URL, True)
    assert (aircraft.mav_url, aircraft.sitl, aircraft.publish_odom_tf) == (
        ROUTER_MAV_URL,
        False,
        True,
    )


def test_a8_video_is_reliable() -> None:
    # A dropped H.265 access unit stalls decode until the next keyframe.
    assert _qos(px4_a8, ("video", CompressedVideo)) is None
