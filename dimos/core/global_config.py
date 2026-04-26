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

import re
from typing import Literal, TypeAlias

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.models.vl.types import VlModelName

ViewerBackend: TypeAlias = Literal["rerun", "rerun-web", "rerun-connect", "foxglove", "none"]


def _get_all_numbers(s: str) -> list[float]:
    return [float(x) for x in re.findall(r"-?\d+\.?\d*", s)]


class GlobalConfig(BaseSettings):
    robot_ip: str | None = None
    robot_ips: str | None = None
    xarm7_ip: str | None = None
    xarm6_ip: str | None = None
    can_port: str | None = None
    simulation: bool = False
    replay: bool = False
    replay_dir: str = "go2_sf_office"
    new_memory: bool = False
    viewer: ViewerBackend = "rerun"
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
    robot_width: float = 0.3
    robot_rotation_diameter: float = 0.6
    nerf_speed: float = 1.0
    # Local path follower in ``replanning_a_star.LocalPlanner`` (issue 921 / P3-3).
    local_planner_path_controller: Literal["differential", "holonomic"] = "holonomic"
    local_planner_holonomic_kp: float = 2.0
    local_planner_holonomic_ky: float = 1.5
    local_planner_max_tangent_accel_m_s2: float = Field(default=1.0, gt=0.0)
    local_planner_max_normal_accel_m_s2: float = Field(default=0.6, gt=0.0)
    local_planner_goal_decel_m_s2: float = Field(default=1.0, gt=0.0)
    local_planner_max_planar_cmd_accel_m_s2: float = Field(default=5.0, gt=0.0)
    local_planner_max_yaw_accel_rad_s2: float = Field(default=5.0, gt=0.0)
    local_planner_max_yaw_rate_rad_s: float | None = Field(default=None, gt=0.0)
    # Issue 921 P4-1: one knob for LocalPlanner sleep pacing and controller dt (e.g. PD).
    local_planner_control_rate_hz: float = Field(default=10.0, ge=0.1, le=60.0)
    # Optional issue 921 JSONL telemetry export. Set to a file path for live speed-vs-divergence logs.
    local_planner_trajectory_tick_log_path: str | None = None
    planner_robot_speed: float | None = None
    mcp_port: int = 9990
    dtop: bool = False
    obstacle_avoidance: bool = True
    detection_model: VlModelName = "moondream"
    listen_host: str = "127.0.0.1"

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
            return "mujoco"
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
