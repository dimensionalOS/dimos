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

Extends the base ROSNavConfig with M20-specific overrides:
- NOS (RK3588, arm64) has no NVIDIA GPU → docker_gpus=None
- 15GB total RAM but shared with host dimos + bridge → 512m SHM, 1.5g memory limit for nav container
- DDS domain 0 (matches NOS rsdriver/yesense)
- No X11, Unity, or simulation assets
- IPC host for drdds bridge shared memory
"""

from dataclasses import field
from pathlib import Path

from dimos.navigation.rosnav.rosnav_module import ROSNav, ROSNavConfig


class M20ROSNavConfig(ROSNavConfig):
    # --- Docker image ---
    docker_image: str = "ghcr.io/aphexcx/m20-nav:latest"
    docker_container_name: str = "dimos-nav"

    # --- NOS hardware constraints ---
    docker_gpus: str | None = None  # RK3588 has no NVIDIA GPU
    docker_shm_size: str = "1g"  # NOS has 15GB but shared with host dimos (was 8g for G1)
    docker_privileged: bool = False
    docker_startup_timeout: float = 180.0

    docker_extra_args: list[str] = field(
        default_factory=lambda: [
            "--memory=1.5g",  # Fail predictably before OOM-killing host
            "--cap-add=NET_ADMIN",
            "--ipc=host",  # Required for drdds bridge POSIX SHM
        ]
    )

    docker_env: dict[str, str] = field(
        default_factory=lambda: {
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "0",  # Match NOS rsdriver (not 42 like G1)
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/ros2_ws/config/fastdds.xml",
            "LOCALIZATION_METHOD": "arise_slam",
        }
    )

    docker_volumes: list[tuple[str, str, str]] = field(default_factory=list)
    docker_devices: list[str] = field(default_factory=list)

    # --- M20 navigation settings ---
    mode: str = "hardware"
    localization_method: str = "arise_slam"
    robot_config_path: str = "deeprobotics/m20"
    vehicle_height: float = 0.47  # 47cm lidar height in agile stance
    use_rviz: bool = False

    def model_post_init(self, __context: object) -> None:
        # Skip ROSNavConfig.model_post_init — it adds G1-specific volumes
        # (X11, Unity, Unitree hardware env vars). We set M20-specific ones here.

        self.docker_env["MODE"] = self.mode
        self.docker_env["LOCALIZATION_METHOD"] = self.localization_method
        self.docker_env["ROBOT_CONFIG_PATH"] = self.robot_config_path
        self.docker_env["VEHICLE_HEIGHT"] = str(self.vehicle_height)
        self.docker_env["USE_RVIZ"] = "true" if self.use_rviz else "false"

        repo_root = Path(__file__).parents[3]
        m20_docker_dir = Path(__file__).parent / "docker"

        arise_share = "/ros2_ws/install/arise_slam_mid360/share/arise_slam_mid360"
        planner_share = "/ros2_ws/install/local_planner/share/local_planner"

        # Minimal NOS volumes — no X11, no Unity sim assets
        self.docker_volumes = [
            # Live dimos source so the module is always up-to-date
            (str(repo_root), "/workspace/dimos", "rw"),
            # M20-specific DDS config (large SHM segments for bridge)
            (
                str(m20_docker_dir / "fastdds_m20.xml"),
                "/ros2_ws/config/fastdds.xml",
                "ro",
            ),
            # M20-specific entrypoint (hardware mode, no Unity)
            (
                str(m20_docker_dir / "entrypoint.sh"),
                "/usr/local/bin/entrypoint.sh",
                "ro",
            ),
            # ARISE SLAM M20 config (overrides default livox_mid360.yaml)
            (
                str(m20_docker_dir / "arise_slam_m20.yaml"),
                f"{arise_share}/config/livox_mid360.yaml",
                "ro",
            ),
            # Patched ARISE launch with longer lifecycle timers for ARM64
            (
                str(m20_docker_dir / "arize_slam_m20.launch.py"),
                f"{arise_share}/launch/arize_slam.launch.py",
                "ro",
            ),
            # M20 robot config for local_planner (vehicle dimensions, speeds)
            (
                str(m20_docker_dir / "local_planner_m20.yaml"),
                f"{planner_share}/config/deeprobotics/m20.yaml",
                "ro",
            ),
        ]


class M20ROSNav(ROSNav):
    """ROSNav module with M20-specific Docker configuration."""

    default_config = M20ROSNavConfig


m20_ros_nav = M20ROSNav.blueprint
