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
M20 ROSNav configuration.

The nav container (dimos-nav) is started independently via deploy.sh start.
dimos reconnects to the already-running container rather than managing its lifecycle.
"""

from dataclasses import field

from dimos.navigation.rosnav.rosnav_module import ROSNav, ROSNavConfig


class M20ROSNavConfig(ROSNavConfig):
    """M20 nav config — reconnects to pre-started container (no build/pull)."""

    # Container is started by deploy.sh, not by dimos
    docker_image: str = "ghcr.io/aphexcx/m20-nav:latest"
    docker_container_name: str = "dimos-nav"
    docker_file: None = None  # Don't try to build
    docker_reconnect_container: bool = True  # Reconnect to existing container

    # NOS constraints
    docker_gpus: str | None = None
    docker_privileged: bool = False
    docker_extra_args: list[str] = field(
        default_factory=lambda: ["--ipc=host"]
    )

    # M20 navigation settings
    localization_method: str = "arise_slam"
    mode: str = "hardware"
    vehicle_height: float = 0.47  # 47cm lidar height in agile stance
    use_rviz: bool = False


class M20ROSNav(ROSNav):
    """ROSNav module that reconnects to the pre-started M20 nav container."""

    default_config = M20ROSNavConfig


m20_ros_nav = M20ROSNav.blueprint
