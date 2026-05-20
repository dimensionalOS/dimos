#!/usr/bin/env python3
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

from __future__ import annotations

import math
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec


class _Go2RobotBodyState(Module):
    """Layer 6 robot body and local-policy state facade for Go2.

    This module observes already-existing low-level streams. It does not move
    the robot, publish commands, or replace `GO2Connection`, mapping,
    navigation, or local-control modules.
    """

    _connection: GO2ConnectionSpec | None = None
    _latest_odom: PoseStamped | None = None
    _latest_odom_seen_at: float | None = None
    _latest_image_seen_at: float | None = None
    _latest_lidar_seen_at: float | None = None
    _odom_count: int = 0
    _image_count: int = 0
    _lidar_count: int = 0

    odom: In[PoseStamped]
    color_image: In[Image]
    lidar: In[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom
        self._latest_odom_seen_at = time.time()
        self._odom_count += 1

    def _on_color_image(self, _image: Image) -> None:
        self._latest_image_seen_at = time.time()
        self._image_count += 1

    def _on_lidar(self, _pointcloud: PointCloud2) -> None:
        self._latest_lidar_seen_at = time.time()
        self._lidar_count += 1

    @rpc
    def get_robot_body_snapshot(self) -> dict[str, Any]:
        """Return the current Layer 6 robot-body and local-policy snapshot."""
        return _to_jsonable(
            {
                "available": True,
                "version": "v1",
                "connection": self.get_connection_state(),
                "sensors": self.get_sensor_state(),
                "local_policy": self.get_local_policy_state(),
                "safety": self.get_safety_state(),
            }
        )

    @rpc
    def get_connection_state(self) -> dict[str, Any]:
        """Return connection-mode and connection-spec state."""
        simulation = bool(getattr(global_config, "simulation", False))
        replay = bool(getattr(global_config, "replay", False))
        mode = "simulation" if simulation else "replay" if replay else "hardware"
        return {
            "available": self._connection is not None,
            "mode": mode,
            "robot_ip": getattr(global_config, "robot_ip", None),
            "unitree_connection_type": getattr(global_config, "unitree_connection_type", None),
            "robot_model": getattr(global_config, "robot_model", None),
            "replay_db": getattr(global_config, "replay_db", None),
        }

    @rpc
    def get_sensor_state(self) -> dict[str, Any]:
        """Return latest observed body sensor stream state."""
        return {
            "odom": self._stream_state(
                count=self._odom_count,
                seen_at=self._latest_odom_seen_at,
                latest=self._pose_to_dict(self._latest_odom),
            ),
            "color_image": self._stream_state(
                count=self._image_count,
                seen_at=self._latest_image_seen_at,
            ),
            "lidar": self._stream_state(
                count=self._lidar_count,
                seen_at=self._latest_lidar_seen_at,
            ),
        }

    @rpc
    def get_local_policy_state(self) -> dict[str, Any]:
        """Return local-control policy settings visible from runtime config."""
        return {
            "available": True,
            "obstacle_avoidance": bool(getattr(global_config, "obstacle_avoidance", False)),
            "go2_mode": getattr(global_config, "go2_mode", None),
            "unitree_connection_type": getattr(global_config, "unitree_connection_type", None),
            "local_control_owner": "GO2Connection and navigation/movement modules",
            "command_interface": "cmd_vel stream and GO2Connection RPCs",
        }

    @rpc
    def get_safety_state(self) -> dict[str, Any]:
        """Return conservative Layer 6 safety state visible to upper layers."""
        return {
            "available": True,
            "stop_interfaces": [
                "stop_navigation",
                "stop_following",
                "stop_security_patrol",
                "GO2Connection.liedown",
            ],
            "obstacle_avoidance_configured": bool(
                getattr(global_config, "obstacle_avoidance", False)
            ),
            "body_pose_available": self._latest_odom is not None,
            "notes": [
                "Layer 6 state is observational in V1.",
                "Physical safety checks still belong to local robot control.",
            ],
        }

    def _stream_state(
        self,
        count: int,
        seen_at: float | None,
        latest: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        now = time.time()
        state: dict[str, Any] = {
            "available": count > 0,
            "count": count,
            "last_seen_age_sec": round(now - seen_at, 3) if seen_at else None,
        }
        if latest is not None:
            state["latest"] = latest
        return state

    def _pose_to_dict(self, pose: PoseStamped | None) -> dict[str, Any] | None:
        if pose is None:
            return None
        return {
            "frame_id": pose.frame_id,
            "timestamp": pose.ts,
            "position": {
                "x": round(pose.position.x, 3),
                "y": round(pose.position.y, 3),
                "z": round(pose.position.z, 3),
            },
            "yaw_degrees": round(math.degrees(pose.yaw), 1),
        }


def _to_jsonable(value: Any, max_string_length: int = 500) -> Any:
    if value is None or isinstance(value, bool | int | float):
        return value
    if isinstance(value, str):
        return value[:max_string_length]
    if isinstance(value, dict):
        return {
            str(key): _to_jsonable(item, max_string_length)
            for key, item in value.items()
            if not str(key).startswith("_")
        }
    if isinstance(value, list | tuple):
        return [_to_jsonable(item, max_string_length) for item in value[:10]]
    return str(value)[:max_string_length]


__all__ = ["_Go2RobotBodyState"]
