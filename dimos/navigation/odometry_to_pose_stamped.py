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

"""OdometryToPoseStamped — thin shim that republishes nav_msgs/Odometry as
geometry_msgs/PoseStamped.

Needed when a SLAM source (e.g. FastLio2) emits ``Odometry`` but downstream
consumers want ``PoseStamped`` — notably the ``transport_lcm`` twist-base
adapter (``ControlCoordinator``) and ``ReplanningAStarPlanner.odom``, both of
which require ``PoseStamped``. Publishing the *same* pose to both keeps the
planner/map and the precision follower in one consistent frame.
"""

from __future__ import annotations

from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OdometryToPoseStampedConfig(ModuleConfig):
    """``frame_id`` override for the published PoseStamped. When empty, the
    incoming Odometry's ``frame_id`` is passed through (falling back to
    ``"odom"`` if that is also empty)."""

    frame_id: str = ""


class OdometryToPoseStamped(Module):
    """Subscribe ``odometry: In[Odometry]`` → publish ``pose: Out[PoseStamped]``."""

    config: OdometryToPoseStampedConfig
    odometry: In[Odometry]
    pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._count = 0

    @rpc
    def start(self) -> None:
        super().start()
        unsub = self.odometry.subscribe(self._on_odometry)
        self.register_disposable(Disposable(unsub))
        logger.info("OdometryToPoseStamped started")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_odometry(self, msg: Odometry) -> None:
        inner = msg.pose.pose  # Pose (position: Vector3, orientation: Quaternion)
        frame_id = self.config.frame_id or msg.frame_id or "odom"
        self.pose.publish(
            PoseStamped(
                ts=msg.ts,
                frame_id=frame_id,
                position=inner.position,
                orientation=inner.orientation,
            )
        )
        self._count += 1
        if self._count == 1:
            logger.info(f"OdometryToPoseStamped: first pose at frame_id='{frame_id}'")


__all__ = ["OdometryToPoseStamped", "OdometryToPoseStampedConfig"]
