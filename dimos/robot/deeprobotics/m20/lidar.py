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

"""The M20's two RoboSense Airys as dimos streams: front_lidar / rear_lidar clouds (each in its own lidar frame),
front_lidar_imu / rear_lidar_imu, and the base_link -> lidar mount transforms.

Runs on AOS (eth0 is on the lidars' network) next to the vendor rslidar, which keeps receiving the same multicast.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.hardware.sensors.lidar.robosense.module import Airy
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, frames_to_edge_transforms
from dimos.robot.deeprobotics.m20.constants import (
    FRONT_LIDAR_FRAME,
    FRONT_LIDAR_GROUP,
    FRONT_LIDAR_IP,
    FRONT_LIDAR_PORTS,
    FRONT_LIDAR_RPY,
    FRONT_LIDAR_XYZ_M,
    REAR_LIDAR_FRAME,
    REAR_LIDAR_GROUP,
    REAR_LIDAR_IP,
    REAR_LIDAR_PORTS,
    REAR_LIDAR_RPY,
    REAR_LIDAR_XYZ_M,
)

FRONT_AIRY = "front_airy"
REAR_AIRY = "rear_airy"


class M20LidarMountTf(StaticTfPublisher):
    """base_link -> front_lidar and base_link -> rear_lidar."""

    def transforms(self) -> list[Transform]:
        return frames_to_edge_transforms(
            [
                (FRONT_LIDAR_FRAME, "base_link", FRONT_LIDAR_XYZ_M, FRONT_LIDAR_RPY),
                (REAR_LIDAR_FRAME, "base_link", REAR_LIDAR_XYZ_M, REAR_LIDAR_RPY),
            ]
        )


def _airy(
    instance_name: str,
    frame: str,
    group: str,
    lidar_ip: str,
    ports: tuple[int, int, int],
    overrides: dict[str, Any],
) -> Blueprint:
    msop_port, difop_port, imu_port = ports
    return Airy.blueprint(
        **{
            "instance_name": instance_name,
            "group": group,
            "lidar_ip": lidar_ip,
            "msop_port": msop_port,
            "difop_port": difop_port,
            "imu_port": imu_port,
            "frame_id": frame,
            "imu_frame_id": frame,
            # the vendor driver's config.yaml: PTP-synced lidar clock, 0.2..60 m
            "use_lidar_clock": True,
            "min_distance": 0.2,
            "max_distance": 60.0,
            **overrides,
        }
    )


def m20_airy_lidars(**overrides: Any) -> MetaBlueprint:
    """Both Airys plus their mount tf; `overrides` (e.g. voxel=0.1) go to both lidars."""
    return autoconnect(
        _airy(
            FRONT_AIRY,
            FRONT_LIDAR_FRAME,
            FRONT_LIDAR_GROUP,
            FRONT_LIDAR_IP,
            FRONT_LIDAR_PORTS,
            overrides,
        ),
        _airy(
            REAR_AIRY,
            REAR_LIDAR_FRAME,
            REAR_LIDAR_GROUP,
            REAR_LIDAR_IP,
            REAR_LIDAR_PORTS,
            overrides,
        ),
        M20LidarMountTf.blueprint(),
    ).remappings(
        [
            (FRONT_AIRY, "lidar", "front_lidar"),
            (FRONT_AIRY, "imu", "front_lidar_imu"),
            (REAR_AIRY, "lidar", "rear_lidar"),
            (REAR_AIRY, "imu", "rear_lidar_imu"),
        ]
    )
