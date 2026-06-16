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

from __future__ import annotations

from typing import Protocol

from dimos.core.module import ModuleConfig
from dimos.simulation.scenes.office import SceneAsset
from dimos.spec.utils import Spec


class BabylonSceneViewerConfig(ModuleConfig):
    """Configuration for the in-process Babylon viewer / browser-physics sim."""

    mjcf_path: str = ""
    port: int = 8091

    scene: SceneAsset | None = None
    disable_scene: bool = False
    browser_collision_path: str | None = None

    broadcast_hz: float = 20.0
    pointcloud_hz: float = 2.0
    pointcloud_max_points: int = 70000
    camera_hz: float = 15.0
    camera_jpeg_quality: int = 75
    camera_name: str = "camera"

    enable_sim: bool = True
    sim_rate: float = 200.0
    vehicle_height: float = 0.75
    init_x: float = 0.0
    init_y: float = 0.0
    init_z: float = 0.0
    init_yaw: float = 0.0
    lock_z: bool = False

    lidar_hz: float = 10.0
    lidar_n_azimuth: int = 360
    lidar_n_elevation: int = 16
    lidar_elevation_min_deg: float = -22.5
    lidar_elevation_max_deg: float = 22.5
    lidar_max_range: float = 30.0
    lidar_z_offset: float = 1.2


class MujocoRespawnSpec(Spec, Protocol):
    def respawn(self) -> bool: ...
    def respawn_at(
        self,
        x: float,
        y: float,
        z: float | None = None,
        yaw: float | None = None,
    ) -> bool: ...


class HumanoidControlSpec(Spec, Protocol):
    """Optional RPC surface for humanoid arm controls in the viewer HUD."""

    def set_arm_joint(self, name: str, position: float) -> bool: ...
    def release_arms(self) -> bool: ...
    def arm_joint_limits(self) -> list[tuple[str, float, float]]: ...


class CoordinatorControlSpec(Spec, Protocol):
    """Optional RPC surface for policy arm/dry-run controls."""

    def set_activated(self, engaged: bool) -> None: ...
    def set_dry_run(self, enabled: bool) -> None: ...
