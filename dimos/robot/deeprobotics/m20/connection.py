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

"""UDP-only connection module for Deep Robotics M20 quadruped.

Receives cmd_vel commands and forwards them to the M20 via UDP.
Designed for the ROSNav architecture where the navigation stack runs in a
Docker container and publishes velocity commands over DDS/ROS2.
"""

from typing import Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, rpc
from dimos.msgs.geometry_msgs import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class M20Connection(Module):
    """UDP connection to the Deep Robotics M20.

    Parameters
    ----------
    enable_ros : bool
        If True, start an internal ROS bridge. Set to False when the
        ROSNav Docker container already handles ROS communication.
    enable_lidar : bool
        If True, manage lidar data streams. Set to False when the
        navigation container owns the lidar pipeline.
    """

    cmd_vel: In[Twist]

    def __init__(
        self,
        enable_ros: bool = False,
        enable_lidar: bool = False,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self._enable_ros = enable_ros
        self._enable_lidar = enable_lidar
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        logger.info(
            "M20Connection started (enable_ros=%s, enable_lidar=%s)",
            self._enable_ros,
            self._enable_lidar,
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_cmd_vel(self, twist: Twist) -> None:
        # TODO: forward twist to M20 via UDP
        pass


m20_connection = M20Connection.blueprint
