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

import logging

from dimos.core.stream import Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.rosnav.rosnav_module import ROSNav, ROSNavConfig

logger = logging.getLogger(__name__)


class M20ROSNavConfig(ROSNavConfig):
    """M20 nav config — dimos manages the container lifecycle."""

    # Use the M20 nav image (includes ros2_pub + ARISE configs)
    docker_image: str = "ghcr.io/aphexcx/m20-nav:latest"
    docker_container_name: str = "dimos-nav"

    # Don't try to build from Dockerfile — image is pre-built via deploy.sh push
    docker_file: None = None
    docker_reconnect_container: bool = True  # Reconnect to existing container on restart

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
            "LOCAL_PLANNER_CONFIG": "m20",
            "ROBOT_CONFIG_PATH": "m20",
        }
    )

    docker_devices: list[str] = field(default_factory=list)

    # M20 navigation settings
    localization_method: str = "arise_slam"
    mode: str = "hardware"
    vehicle_height: float = 0.47  # 47cm lidar height in agile stance
    use_rviz: bool = False

    def model_post_init(self, __context: object) -> None:
        # parents: [0]=m20/ [1]=deeprobotics/ [2]=robot/ [3]=dimos/ [4]=repo_root/
        repo_root = Path(__file__).parents[4]
        self.docker_volumes = [
            # Live dimos source so container-side code stays up-to-date
            (str(repo_root), "/workspace/dimos", "rw"),
        ]


class M20ROSNav(ROSNav):
    """ROSNav module for M20 — runs in dimos-managed Docker container.

    Adds an ``odometry`` output (full Odometry with twist) alongside the
    inherited ``odom`` (PoseStamped-only). SmartNav modules subscribe to
    ``odometry: In[Odometry]``, so this bridge is needed.
    """

    default_config = M20ROSNavConfig

    # SmartNav expects Odometry type — parent only outputs PoseStamped
    odometry: Out[Odometry]

    def _on_ros_odom(self, msg) -> None:  # type: ignore[override]
        """Extend parent to also publish full Odometry."""
        super()._on_ros_odom(msg)

        # Build full Odometry from the ROS message (preserves twist + covariance)
        from dimos.msgs.geometry_msgs.Pose import Pose
        from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular

        odom = Odometry(
            ts=ts,
            frame_id=msg.header.frame_id,
            child_frame_id=getattr(msg, "child_frame_id", ""),
            pose=PoseWithCovariance(
                Pose(position=Vector3(p.x, p.y, p.z), orientation=Quaternion(o.x, o.y, o.z, o.w)),
                list(msg.pose.covariance),
            ),
            twist=TwistWithCovariance(
                Twist(linear=Vector3(lv.x, lv.y, lv.z), angular=Vector3(av.x, av.y, av.z)),
                list(msg.twist.covariance),
            ),
        )
        self.odometry.publish(odom)


m20_ros_nav = M20ROSNav.blueprint
