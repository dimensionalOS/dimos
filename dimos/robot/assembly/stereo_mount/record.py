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

"""Record the stereo_mount rig (ZED eyes + Point-LIO odom/lidar) into a memory2 db.

Extends :class:`~dimos.hardware.sensors.lidar.pointlio.recorder.PointlioRecorder`
(``pointlio_odometry`` / ``pointlio_lidar`` with odometry poses baked in) with the
SDK-free ZED module's ``color_image_left`` / ``color_image_right`` — names already
match, so autoconnect wires them straight in. Point-LIO publishes the moving
``world -> lidar_link`` edge onto tf and the rig's static urdf frames tie the
cameras into that tree, so every stream lands world-anchored.
"""

from __future__ import annotations

import asyncio
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.memory2.module import pose_setter_for
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# How long pointlio_lidar may be silent before the loss banner starts repeating.
LIDAR_LOSS_TIMEOUT = 5.0


class StereoMountRecorder(PointlioRecorder):
    # pointlio_odometry / pointlio_lidar are inherited from PointlioRecorder.
    color_image_left: In[Image]
    color_image_right: In[Image]

    _watchdog_running: bool = False
    _last_lidar_at: float | None = None
    _started_at: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._started_at = time.time()
        self._last_lidar_at = None
        self._watchdog_running = True
        self.spawn(self._lidar_loss_watchdog())

    @pose_setter_for("pointlio_lidar")
    async def _lidar_pose(self, msg: Any) -> Any:
        # Piggyback on the recording path itself: this runs once per stored
        # lidar message, so it doubles as the loss-watchdog liveness signal.
        self._last_lidar_at = time.time()
        return await super()._lidar_pose(msg)

    async def _lidar_loss_watchdog(self) -> None:
        """Loudly and repeatedly complain while no lidar data is arriving.

        Without pointlio_lidar there is no odometry, no ``world`` tf edge, and
        every recorded stream is missing its world pose — a silent, ruined
        recording. Make it impossible to miss.
        """
        while self._watchdog_running:
            await asyncio.sleep(LIDAR_LOSS_TIMEOUT)
            if not self._watchdog_running:
                return
            silent_for = time.time() - (self._last_lidar_at or self._started_at)
            if silent_for <= LIDAR_LOSS_TIMEOUT:
                continue
            never = self._last_lidar_at is None
            logger.error("█" * 70)
            logger.error(
                "██ LIDAR DATA %s — %.0fs without pointlio_lidar messages!",
                "NEVER RECEIVED" if never else "LOST",
                silent_for,
            )
            logger.error("██ No lidar -> no odometry -> no world poses in this recording.")
            logger.error(
                "██ Check the Mid-360 power + ethernet link (ethtool <nic>) and"
                " DIMOS_POINTLIO_LIDAR_IP."
            )
            logger.error("█" * 70)

    @rpc
    def stop(self) -> None:
        self._watchdog_running = False
        super().stop()
