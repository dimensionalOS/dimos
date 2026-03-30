#!/usr/bin/env python3
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

"""
M20 ROSNav Docker configuration.

dimos manages the nav container lifecycle via DockerModuleProxy. The container
runs ARISE SLAM + nav planner (started by the entrypoint) alongside the dimos
RPC server (started by docker_module run). The M20-specific Dockerfile.nav
bakes in ros2_pub, configs, and the M20 wrapper entrypoint.
"""

from __future__ import annotations

from dataclasses import field
from pathlib import Path

from dimos.navigation.rosnav.rosnav_module import ROSNav, ROSNavConfig


class M20ROSNavConfig(ROSNavConfig):
    """M20 nav config — dimos manages the container lifecycle."""

    # Use the M20 nav image (includes ros2_pub + ARISE configs)
    docker_image: str = "ghcr.io/aphexcx/m20-nav:latest"
    docker_container_name: str = "dimos-nav"

    # Don't try to build from Dockerfile — image is pre-built via deploy.sh push
    docker_file: None = None

    # M20 entrypoint starts ros2_pub (drdds bridge) before the base entrypoint
    docker_entrypoint: str = "/usr/local/bin/m20_entrypoint.sh"

    # NOS hardware constraints
    docker_gpus: str | None = None  # RK3588 has no NVIDIA GPU
    docker_shm_size: str = "1g"
    docker_privileged: bool = True  # Required for --ipc host + /dev/shm access
    docker_startup_timeout: float = 600.0  # ARM64: pip install + pybind11 build on first start

    docker_extra_args: list[str] = field(
        default_factory=lambda: [
            "--ipc=host",  # Required for drdds bridge POSIX SHM
        ]
    )

    docker_env: dict[str, str] = field(
        default_factory=lambda: {
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "0",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "LOCALIZATION_METHOD": "arise_slam",
            "MODE": "hardware",
        }
    )

    docker_devices: list[str] = field(default_factory=list)

    # M20 navigation settings
    localization_method: str = "arise_slam"
    mode: str = "hardware"
    vehicle_height: float = 0.47  # 47cm lidar height in agile stance
    use_rviz: bool = False

    def model_post_init(self, __context: object) -> None:
        # Minimal volumes — configs are baked into the image via Dockerfile.nav
        # parents: [0]=m20/ [1]=deeprobotics/ [2]=robot/ [3]=dimos/ [4]=repo_root/
        repo_root = Path(__file__).parents[4]
        entrypoint_sh = repo_root / "dimos" / "navigation" / "rosnav" / "entrypoint.sh"
        self.docker_volumes = [
            # Live dimos source for RPC module + editable install
            (str(repo_root), "/workspace/dimos", "rw"),
            # Mount updated entrypoint (image has stale docker_runner ref until rebuilt)
            (str(entrypoint_sh), "/usr/local/bin/entrypoint.sh", "ro"),
        ]


class M20ROSNav(ROSNav):
    """ROSNav module for M20 — runs in dimos-managed Docker container."""

    default_config = M20ROSNavConfig


m20_ros_nav = M20ROSNav.blueprint
