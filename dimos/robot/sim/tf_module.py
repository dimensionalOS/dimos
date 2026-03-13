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

"""Lightweight TF publisher for DimSim.

Subscribes to odometry from the DimSim bridge (via LCM, wired by autoconnect)
and publishes the transform chain: world -> base_link -> {camera_link ->
camera_optical, lidar_link}.  Also publishes CameraInfo at 1 Hz, forwards
cmd_vel to the bridge, and exposes a ``move()`` RPC.

This module replaces the TF / camera_info / cmd_vel parts of the old
DimSimConnection while the NativeModule bridge handles sensor data directly.
"""

from __future__ import annotations

import math
from threading import Thread
import time

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.sensor_msgs import CameraInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# DimSim captures at 960x432 with 80-degree horizontal FOV.
_DIMSIM_WIDTH = 960
_DIMSIM_HEIGHT = 432
_DIMSIM_FOV_DEG = 80


def _camera_info_static() -> CameraInfo:
    """Build CameraInfo for DimSim's virtual camera."""
    fov_rad = math.radians(_DIMSIM_FOV_DEG)
    fx = (_DIMSIM_WIDTH / 2) / math.tan(fov_rad / 2)
    fy = fx  # square pixels
    cx = _DIMSIM_WIDTH / 2.0
    cy = _DIMSIM_HEIGHT / 2.0

    return CameraInfo(
        frame_id="camera_optical",
        height=_DIMSIM_HEIGHT,
        width=_DIMSIM_WIDTH,
        distortion_model="plumb_bob",
        D=[0.0, 0.0, 0.0, 0.0, 0.0],
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x=0,
        binning_y=0,
    )


class DimSimTF(Module):
    """Lightweight TF publisher for the DimSim simulator.

    Wired by autoconnect to receive odom from the bridge's LCM output.
    Publishes TF transforms and camera intrinsics.  Exposes ``move()`` RPC
    for sending cmd_vel to the bridge.
    """

    # Odom input — autoconnect wires this to DimSimBridge.odom via LCM
    odom: In[PoseStamped]

    # Outputs
    camera_info: Out[CameraInfo]
    cmd_vel: Out[Twist]

    _camera_info_thread: Thread | None = None
    _latest_odom: PoseStamped | None = None
    _odom_last_ts: float = 0.0
    _odom_count: int = 0

    @classmethod
    def _odom_to_tf(cls, odom: PoseStamped) -> list[Transform]:
        """Build transform chain from odometry pose.

        Transform tree: world -> base_link -> {camera_link -> camera_optical, lidar_link}
        """
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),  # camera 30cm forward
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=odom.ts,
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=odom.ts,
        )

        lidar_link = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="lidar_link",
            ts=odom.ts,
        )

        return [
            Transform.from_pose("base_link", odom),
            camera_link,
            camera_optical,
            lidar_link,
        ]

    def _on_odom(self, pose: PoseStamped) -> None:
        """Handle incoming odometry — publish TF transforms."""
        self._latest_odom = pose
        self._odom_count += 1

        transforms = self._odom_to_tf(pose)
        self.tf.publish(*transforms)

    def _publish_camera_info_loop(self) -> None:
        """Publish camera intrinsics at 1 Hz."""
        while self._camera_info_thread is not None:
            self.camera_info.publish(_camera_info_static())
            time.sleep(1.0)

    @rpc
    def start(self) -> None:
        super().start()

        from reactivex.disposable import Disposable

        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))

        self._camera_info_thread = Thread(target=self._publish_camera_info_loop, daemon=True)
        self._camera_info_thread.start()

        logger.info("DimSimTF started — listening for odom, publishing TF + camera_info")

    @rpc
    def stop(self) -> None:
        thread = self._camera_info_thread
        self._camera_info_thread = None
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to the simulator via cmd_vel."""
        self.cmd_vel.publish(twist)
        return True


sim_tf = DimSimTF.blueprint

__all__ = ["DimSimTF", "sim_tf"]
