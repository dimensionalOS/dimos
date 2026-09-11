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

"""Project cockpit composition; transport and panel implementations remain in DimOS."""

import os
from dataclasses import dataclass
from typing import Any, ClassVar

from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.pollen.microduck import web_codecs  # noqa: F401 -- register existing encoders
from dimos.web.cockpit import (
    Channel,
    ChannelRequest,
    Chat,
    Col,
    Control,
    NavMap,
    Panel,
    Row,
    Teleop,
    cockpit,
)
from langchain_core.messages.base import BaseMessage
from microduck_world.ball_detection import BALL_CAMERA_CHANNELS, BALL_CAMERA_ENCODING
from microduck_world.comparison import COMPARE_CHANNEL, COMPARE_FPS
from microduck_world.relay import WorldBridge, relay_settings
from microduck_world.roster import ROSTER
from microduck_world.world_sim import WORLD_ENCODING, WORLD_FPS

HEAD_SIZE = (640, 360)
HEAD_FPS = 6
JPEG_QUALITY = 40

AGENT_ENABLED = bool(os.environ.get("OPENAI_API_KEY"))


@dataclass(frozen=True)
class World3D(Panel):
    kind: ClassVar[str] = "world3d"
    title: str = "Microduck World"
    view: str = "world"
    robot: str = "duck1"

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        return (ChannelRequest("world_state", "rx", WORLD_ENCODING, WORLD_FPS, delivery="latest"),)

    def _panel_params(self) -> dict[str, Any]:
        jpeg = "color_image" if self.view == "pov" else COMPARE_CHANNEL
        return {"view": self.view, "jpeg": jpeg, "robot": self.robot}


@dataclass(frozen=True)
class BallCamera(Panel):
    kind: ClassVar[str] = "ball-camera"
    title: str = "Football camera"
    robot: str = "duck1"

    def _channel_requests(self) -> tuple[ChannelRequest, ...]:
        return (
            ChannelRequest(
                f"{self.robot}_ball_camera", "rx", BALL_CAMERA_ENCODING, 3.0, delivery="latest"
            ),
        )

    def _panel_params(self) -> dict[str, Any]:
        return {"robot": self.robot}


BALL_CHANNELS = tuple(
    Channel(name, str, encoding=BALL_CAMERA_ENCODING, delivery="latest", max_hz=3.0)
    for name in BALL_CAMERA_CHANNELS
)

CHANNELS = (
    # Deliberately not bound in panel.channels: the SDK subscribes only while JPEG is selected.
    Channel(
        COMPARE_CHANNEL,
        Image,
        encoding="jpeg.v1",
        delivery="latest",
        max_hz=COMPARE_FPS,
        rate_gate=False,
        params={"quality": JPEG_QUALITY},
    ),
    Channel(
        "color_image",
        Image,
        encoding="jpeg.v1",
        delivery="latest",
        max_hz=HEAD_FPS,
        rate_gate=False,
        params={"quality": JPEG_QUALITY},
    ),
    Channel(
        "agent",
        BaseMessage,
        encoding="chat.json.v1",
        max_hz=30.0,
        resend_on_subscribe=True,
        rate_gate=False,
        replay_depth=200,
    ),
    Channel(
        "agent_idle",
        bool,
        encoding="flag.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "mode",
        str,
        encoding="mode.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "policy_state",
        str,
        encoding="policy.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "world_state",
        str,
        encoding=WORLD_ENCODING,
        delivery="latest",
        max_hz=WORLD_FPS,
        resend_on_subscribe=True,
    ),
    Channel(
        "path",
        Path,
        encoding="path.json.v1",
        delivery="latest",
        max_hz=5.0,
        resend_on_subscribe=True,
    ),
    Channel(
        "nav_state",
        str,
        encoding="navstate.json.v1",
        max_hz=10.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
    Channel(
        "places",
        str,
        encoding="places.json.v1",
        max_hz=2.0,
        resend_on_subscribe=True,
        rate_gate=False,
    ),
)


def world_cockpit(robot: str = "duck1", generation: str = "") -> Blueprint:
    """One full cockpit per robot; the world bridge only serves spectators."""
    observer = robot == "world"
    layout = (
        World3D(robot="duck1")
        if observer
        else Col(
            Control(),
            Row(
                World3D(robot=robot),
                Col(
                    World3D(title="Duck camera", view="pov", robot=robot),
                    BallCamera(robot=robot),
                    NavMap(fit_places=True),
                    Teleop(title="Drive Microduck", mode="mode", max_linear=0.15, max_angular=0.6),
                    shares=[3, 3, 3, 2],
                ),
                Chat(
                    title="Agent" if AGENT_ENABLED else "Agent (API key required)",
                    read_only=not AGENT_ENABLED,
                ),
                shares=[6, 3, 3],
            ),
        )
    )
    channels = (
        tuple(c for c in CHANNELS if c.stream in ("world_state", COMPARE_CHANNEL))
        if observer
        else CHANNELS
    ) + tuple(c for c in BALL_CHANNELS if observer or c.stream == f"{robot}_ball_camera")
    compiled = cockpit(layout=layout, channels=channels).blueprints[0]
    result = WorldBridge.blueprint(
        **compiled.kwargs,
        robot_id="world" if observer else f"{robot}-{generation}",
        robot_name="Football world" if observer else ROSTER[robot]["name"],
        relay_url=None if observer else relay_settings().upstream_url,
        local_port=int(relay_settings().upstream_url.rsplit(":", 1)[1]),
        open_browser=False,
        web_build=False,
    )
    return result.namespace(robot, expose=("world_state", COMPARE_CHANNEL, *BALL_CAMERA_CHANNELS))
