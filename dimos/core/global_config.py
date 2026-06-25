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

from collections.abc import Iterable
import re
from typing import Any

from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.constants import DEFAULT_BUILD_NATIVE
from dimos.models.vl.types import VlModelName
from dimos.visualization.rerun.constants import (
    RERUN_ENABLE_WEB,
    RERUN_OPEN_DEFAULT,
    RerunOpenOption,
    ViewerBackend,
)


def _get_all_numbers(s: str) -> list[float]:
    return [float(x) for x in re.findall(r"-?\d+\.?\d*", s)]


# Field names whose values must never be printed/serialized in the clear:
# anything containing "_secret" or "token", or ending in "_key".
_SECRET_KEY_RE = re.compile(r"_secret|token|_key$", re.IGNORECASE)
_REDACTED = "***"


def _is_secret_key(name: str) -> bool:
    return _SECRET_KEY_RE.search(name) is not None


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
    robot_model: str | None = None
    robot_id: str | None = None
    robot_width: float = 0.3
    robot_rotation_diameter: float = 0.6
    nerf_speed: float = 1.0
    planner_robot_speed: float | None = None
    mcp_port: int = 9990
    build_native: bool = DEFAULT_BUILD_NATIVE
    dtop: bool = False
    obstacle_avoidance: bool = True
    detection_model: VlModelName = "moondream"
    listen_host: str = "127.0.0.1"
    dimsim_scene: str = "apt"
    dimsim_port: int = 8090

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    def update(self, **kwargs: object) -> None:
        """Update config fields in place."""
        for key, value in kwargs.items():
            if not hasattr(self, key):
                raise AttributeError(f"GlobalConfig has no field '{key}'")
            setattr(self, key, value)

    def model_dump(self, **kwargs: Any) -> dict[str, Any]:
        """Dump fields with secret-looking values redacted."""
        return {
            k: (_REDACTED if v is not None and _is_secret_key(k) else v)
            for k, v in super().model_dump(**kwargs).items()
        }

    def __repr_args__(self) -> Iterable[tuple[str | None, Any]]:
        return [
            (k, _REDACTED if v is not None and k and _is_secret_key(k) else v)
            for k, v in super().__repr_args__()
        ]

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


global_config = GlobalConfig()
