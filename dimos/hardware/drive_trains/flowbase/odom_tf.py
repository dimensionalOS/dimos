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

"""Converts FlowBase wheel odometry into world←base_link TF.

ControlCoordinator stores (x, y, theta) from FlowBaseAdapter.read_odometry()
as *positions* on the base joints ({hw_id}/vx, {hw_id}/vy, {hw_id}/wz).
This module re-publishes those as a TF transform so downstream modules can
place observations in the world frame.
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
    hw_id: str = "base"
    world_frame: str = "world"
    base_frame: str = "base_link"


class FlowBaseOdomModule(Module):
    """Publishes world←base_link TF from ControlCoordinator joint positions."""

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
            return

        self.tf.publish(Transform(
            translation=Vector3(x, y, 0.0),
            rotation=Quaternion.from_euler(Vector3(0.0, 0.0, theta)),
            frame_id=self.config.world_frame,
            child_frame_id=self.config.base_frame,
            ts=js.ts,
        ))
