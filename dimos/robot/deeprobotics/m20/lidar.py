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

"""The M20's two RoboSense Airys: an ``M20Lidar`` module per lidar (front or rear), both onto one ``lidar`` stream and
their IMUs onto ``lidar_imu``, told apart by frame_id, each publishing its own mount onto the one global tf.

Runs on AOS (eth0 is on the lidars' network) next to the vendor rslidar, which keeps receiving the same multicast.
"""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field, model_validator

from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.hardware.sensors.lidar.robosense.module import Airy, AiryConfig
from dimos.robot.deeprobotics.m20 import constants

# Each Airy's network (multicast group, address, MSOP/DIFOP/IMU ports) and mount in base_link.
_HARDWARE = {
    "front": {
        "group": constants.FRONT_LIDAR_GROUP,
        "lidar_ip": constants.FRONT_LIDAR_IP,
        "ports": constants.FRONT_LIDAR_PORTS,
        "mount_xyz": constants.FRONT_LIDAR_XYZ_M,
        "mount_rpy": constants.FRONT_LIDAR_RPY,
    },
    "rear": {
        "group": constants.REAR_LIDAR_GROUP,
        "lidar_ip": constants.REAR_LIDAR_IP,
        "ports": constants.REAR_LIDAR_PORTS,
        "mount_xyz": constants.REAR_LIDAR_XYZ_M,
        "mount_rpy": constants.REAR_LIDAR_RPY,
    },
}


class M20LidarConfig(AiryConfig):
    # Which of the M20's Airys: its network and mount follow (any of those set explicitly wins).
    lidar: Literal["front", "rear"]
    # The scans' frame, the IMU samples' frame, and the frame the mount is published from.
    frame_id: str
    imu_frame_id: str
    mount_parent_frame_id: str
    # Returns nearer than this (m) are dropped.
    min_distance: float = Field(default=0.1, ge=0)
    # The vendor driver's config.yaml: up to 60 m, PTP-synced lidar clock.
    max_distance: float = Field(default=60.0, ge=0)
    use_lidar_clock: bool = True

    @model_validator(mode="after")
    def _hardware_of_the_chosen_lidar(self) -> M20LidarConfig:
        hardware = _HARDWARE[self.lidar]
        msop_port, difop_port, imu_port = hardware["ports"]
        chosen = {
            "group": hardware["group"],
            "lidar_ip": hardware["lidar_ip"],
            "msop_port": msop_port,
            "difop_port": difop_port,
            "imu_port": imu_port,
            "mount_xyz": hardware["mount_xyz"],
            "mount_rpy": hardware["mount_rpy"],
        }
        for key, value in chosen.items():
            if key not in self.model_fields_set:
                object.__setattr__(self, key, value)
        return self

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # the driver only knows the Airy's own fields
        config.pop("lidar")
        return config


class M20Lidar(Airy):
    """One of the M20's two Airys (front or rear), with its mount on tf."""

    config: M20LidarConfig


def m20_lidars(lidar: str = "lidar", imu: str = "lidar_imu", **overrides: Any) -> MetaBlueprint:
    """Both M20Lidars, each namespaced, onto `lidar` and `imu`; tf stays global. `overrides` (e.g. voxel=0.1) go to both."""
    fragments: list[Blueprint] = []
    for which, frame in (
        ("front", constants.FRONT_LIDAR_FRAME),
        ("rear", constants.REAR_LIDAR_FRAME),
    ):
        fragments.append(
            M20Lidar.blueprint(
                lidar=which,
                frame_id=frame,
                # the samples are rotated into the lidar's axes, so they share its frame
                imu_frame_id=frame,
                mount_parent_frame_id="base_link",
                **overrides,
            )
            .remappings([(M20Lidar, "lidar", lidar), (M20Lidar, "imu", imu)])
            .namespace(frame, expose={lidar, imu, "tf"})
        )
    return autoconnect(*fragments)
