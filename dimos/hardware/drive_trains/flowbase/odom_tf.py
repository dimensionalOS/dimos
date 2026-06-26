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

"""Publishes ``world ← base_link`` TF from FlowBase wheel odometry.

``ControlCoordinator`` reads ``[x, y, theta]`` from ``FlowBaseAdapter.read_odometry``
and stores it as *positions* on the base joints (``{hw_id}/vx``, ``{hw_id}/vy``,
``{hw_id}/wz``).  This module subscribes to ``coordinator_joint_state`` and
re-publishes those values as a proper TF transform so downstream modules (cameras,
depth estimation, costmappers) can place observations in the world frame.

Wire it alongside ``ControlCoordinator`` in any FlowBase blueprint::

    autoconnect(
        coordinator_flowbase,
        FlowBaseOdomModule.blueprint(),
        ...
    )

``coordinator_joint_state`` autoconnects by name.
"""

from __future__ import annotations

from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Config(ModuleConfig):
    """Configuration for FlowBaseOdomModule.

    Attributes:
        hw_id:       Hardware ID used in joint names (matches ``ControlCoordinator`` hw_id).
        world_frame: TF parent frame.
        base_frame:  TF child frame published by this module.
    """

    hw_id: str = "base"
    world_frame: str = "world"
    base_frame: str = "base_link"


class FlowBaseOdomModule(Module):
    """Converts FlowBase wheel-odometry joint positions into ``world ← base_link`` TF.

    Subscribes to the ``coordinator_joint_state`` stream (published by
    ``ControlCoordinator``) and extracts the ``{hw_id}/vx``, ``{hw_id}/vy``,
    and ``{hw_id}/wz`` joint positions, which the coordinator populates from
    ``FlowBaseAdapter.read_odometry()``.  These are (x, y, theta) in metres
    and radians relative to the start pose.

    Ports
    -----
    Inputs
        coordinator_joint_state : Joint state from ``ControlCoordinator``.
    """

    config: Config

    coordinator_joint_state: In[JointState]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_joint_state(self, js: JointState) -> None:
        hw = self.config.hw_id
        try:
            names = js.name
            pos = js.position
            x = pos[names.index(f"{hw}/vx")]
            y = pos[names.index(f"{hw}/vy")]
            theta = pos[names.index(f"{hw}/wz")]
        except (ValueError, IndexError):
            # Message doesn't contain base joints — skip quietly.
            return

        tf = Transform(
            translation=Vector3(x, y, 0.0),
            rotation=Quaternion.from_euler(Vector3(0.0, 0.0, theta)),
            frame_id=self.config.world_frame,
            child_frame_id=self.config.base_frame,
            ts=js.ts,
        )
        self.tf.publish(tf)
