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
M20ROSNavConfig: Docker configuration for running the CMU navigation stack
(FASTLIO2 + FAR planner + base_autonomy) on a Deep Robotics M20.

The container runs on the M20 NOS (Ubuntu 20.04 / arm64 / RK3588). DDS domain 0
matches the NOS rsdriver and lio_perception stack. Volumes are minimal — no X11,
Unity, or simulation assets.
"""

from dataclasses import dataclass, field
from pathlib import Path

from dimos.core.module import ModuleConfig


@dataclass
class M20ROSNavConfig(ModuleConfig):
    """Docker-oriented config for the M20 navigation container.

    Acceptance criteria:
      - M20ROSNavConfig() instantiates without error
      - docker_env["ROS_DOMAIN_ID"] == "0"
      - docker_extra_args contains "--memory=1.5g"
      - docker_volumes does NOT contain X11 or Unity paths
      - docker_env["LOCALIZATION_METHOD"] == "fastlio"
    """

    # --- Docker image ---
    docker_image: str = "dimos_m20_nav:humble"
    docker_file: Path | None = field(
        default_factory=lambda: Path(__file__).parent / "docker" / "Dockerfile"
    )
    docker_build_context: Path | None = field(
        default_factory=lambda: Path(__file__).parents[3]  # repo root
    )

    # --- Docker runtime ---
    docker_network_mode: str = "host"
    docker_shm_size: str = "512m"
    docker_gpus: str | None = None  # M20 NOS (RK3588) has no NVIDIA GPU
    docker_privileged: bool = False
    docker_startup_timeout: float = 180.0

    docker_entrypoint: str = field(
        default_factory=lambda: str(
            Path(__file__).parent / "docker" / "entrypoint.sh"
        )
    )

    docker_extra_args: list[str] = field(
        default_factory=lambda: [
            "--memory=1.5g",
            "--cap-add=NET_ADMIN",
        ]
    )

    docker_env: dict[str, str] = field(
        default_factory=lambda: {
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "0",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/ros2_ws/config/fastdds.xml",
            "LOCALIZATION_METHOD": "fastlio",
        }
    )

    docker_volumes: list[tuple[str, str, str]] = field(
        default_factory=lambda: []
    )

    docker_devices: list[str] = field(default_factory=list)

    # --- Navigation settings ---
    mode: str = "hardware"
    robot_config_path: str = "deeprobotics/m20"

    def __post_init__(self) -> None:
        super().__post_init__()

        self.docker_env["MODE"] = self.mode
        self.docker_env["ROBOT_CONFIG_PATH"] = self.robot_config_path

        repo_root = Path(__file__).parents[3]

        # Minimal NOS volumes — no X11, no Unity sim assets
        self.docker_volumes = [
            # Live dimos source so the module is always up-to-date
            (str(repo_root), "/workspace/dimos", "rw"),
            # M20-specific DDS config — unicast to AOS only (excludes GOS)
            (
                str(Path(__file__).parent / "docker" / "fastdds_m20.xml"),
                "/ros2_ws/config/fastdds.xml",
                "ro",
            ),
            # M20-specific entrypoint
            (
                str(Path(__file__).parent / "docker" / "entrypoint.sh"),
                "/usr/local/bin/entrypoint.sh",
                "ro",
            ),
        ]
