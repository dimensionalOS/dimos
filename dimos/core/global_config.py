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

import os
import platform
import re
from typing import Literal, TypeAlias

from pydantic import AliasChoices, Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.constants import DEFAULT_BUILD_NATIVE
from dimos.models.vl.types import VlModelName
from dimos.visualization.rerun.constants import (
    RERUN_ENABLE_WEB,
    RERUN_OPEN_DEFAULT,
    RerunOpenOption,
    ViewerBackend,
)

TransportBackend: TypeAlias = Literal["lcm", "zenoh"]
ZenohMode: TypeAlias = Literal["peer", "client", "router"]


def _get_all_numbers(s: str) -> list[float]:
    return [float(x) for x in re.findall(r"-?\d+\.?\d*", s)]


def _default_transport() -> TransportBackend:
    if platform.system() == "Darwin":
        return "zenoh"
    return "lcm"


class GlobalConfig(BaseSettings):
    robot_ip: str | None = None
    robot_ips: str | None = None
    # Per-device AES-128 key for new Unitree firmware (G1 >=1.5.1, Go2 >=1.1.15, data2=3
    # handshake). Fetch: unitree-fetch-aes-key --email YOU --sn <serial>
    unitree_aes_128_key: str | None = None
    xarm7_ip: str | None = None
    xarm6_ip: str | None = None
    can_port: str | None = None
    device_path: str | None = None  # device path for real robot (e.g. /dev/ttyUSB0)
    simulation: str = ""
    replay: bool = False
    replay_db: str = "go2_short"
    new_memory: bool = False
    # Discover zenoh peers across the network.
    # Toggling off drops back to loopback-only discovery:
    # Sibling worker processes still find each other,
    # remote peers come solely from the connect endpoints derived from --robot-ip
    zenoh_scouting: bool = False
    # Interface multicast scouting binds to, overriding the choice above. Name a
    # link (`--zenoh-interface wlan0`) to scout exactly that one -- how robots on
    # a shared LAN find each other without scouting every interface they own.
    zenoh_interface: str = ""
    # Whether multicast scouting runs at all -- unlike the two knobs above, which
    # only decide how far it reaches. Off leaves discovery to gossip and the
    # explicit connect endpoints, which is what a router deployment wants.
    zenoh_multicast: bool = True
    # Gossip discovery: a peer hands back the peers it already knows, turning one
    # dialled endpoint into the whole mesh. Off means a session sees only what it
    # dialled itself, so every peer needs a fixed listen port and its own endpoint.
    zenoh_gossip: bool = True
    # Endpoints to dial, comma separated, overriding the ones derived from
    # --robot-ip. Set it to watch a robot without claiming it as *the* robot of
    # this process (`dimos --zenoh-connect tcp/go2:7447 spy`), or to reach a
    # router that is not the robot. A bare host gets `tcp/` and the robot port.
    zenoh_connect: str = ""
    # Seconds ZenohService.start() blocks for the configured connect endpoints to
    # link before giving up and continuing. 0 disables the wait.
    zenoh_connect_timeout: float = 1.0
    # Session mode every zenoh session opens in. `client` routes everything
    # through a single router instead of gossip-meshing peer to peer -- one copy
    # of a heavy stream over the wifi link no matter how many local subscribers.
    zenoh_mode: ZenohMode = "peer"
    viewer: ViewerBackend = "rerun"
    rerun_open: RerunOpenOption = RERUN_OPEN_DEFAULT
    rerun_web: bool = RERUN_ENABLE_WEB
    rerun_host: str | None = None
    rerun_websocket_server_port: int = 3030
    n_workers: int = 2
    memory_limit: str = "auto"
    mujoco_camera_position: str | None = None
    mujoco_room: str | None = None
    mujoco_room_from_occupancy: str | None = None
    mujoco_global_costmap_from_occupancy: str | None = None
    mujoco_global_map_from_pointcloud: str | None = None
    mujoco_start_pos: str = "-1.0, 1.0"
    mujoco_steps_per_frame: int = 7
    scene_package: str | None = None
    robot_model: str | None = None
    robot_id: str | None = None
    robot_width: float = 0.3
    robot_rotation_diameter: float = 0.6
    nerf_speed: float = 1.0
    mcp_port: int = 9990
    # `DIMOS_TRANSPORT` (or `.env`) is the single switch read by every process
    # (dimos, humancli, agentspy, dtop). The `transport` alias keeps the bare
    # env name and the `--transport` CLI flag (which sets the field by name) working.
    transport: TransportBackend = Field(
        default_factory=_default_transport,
        validation_alias=AliasChoices("DIMOS_TRANSPORT", "transport"),
    )
    build_native: bool = DEFAULT_BUILD_NATIVE
    dtop: bool = False
    obstacle_avoidance: bool = True
    detection_model: VlModelName = "moondream"
    listen_host: str = "127.0.0.1"
    dimsim_scene: str = "apartment"
    dimsim_port: int = 8090
    dimsim_headless: bool = True
    local_relay: bool = False
    relay_url: str | None = None

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        validate_assignment=True,
    )

    def update(self, **kwargs: object) -> None:
        """Update config fields in place."""
        for key, value in kwargs.items():
            if key not in type(self).model_fields:
                raise AttributeError(f"GlobalConfig has no field '{key}'")
            setattr(self, key, value)

    @property
    def unitree_connection_type(self) -> str:
        if self.replay:
            return "replay"
        if self.simulation:
            return self.simulation
        return "webrtc"

    @property
    def mujoco_start_pos_float(self) -> tuple[float, float]:
        x, y = _get_all_numbers(self.mujoco_start_pos)
        return (x, y)

    @property
    def mujoco_camera_position_float(self) -> tuple[float, ...]:
        if self.mujoco_camera_position is None:
            return (-0.906, 0.008, 1.101, 4.931, 89.749, -46.378)
        return tuple(_get_all_numbers(self.mujoco_camera_position))

    @property
    def processed_robot_ips(self) -> tuple[str, ...]:
        ips = [x.strip() for x in (self.robot_ips or "").split(",") if x.strip()]
        is_running_tests = "PYTEST_CURRENT_TEST" in os.environ
        if not ips and not is_running_tests:
            raise ValueError("No robot IPs specified. Must have at least one IP.")
        return tuple(ips)


global_config = GlobalConfig()
