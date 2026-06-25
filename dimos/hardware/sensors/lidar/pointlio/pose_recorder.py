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

"""Memory2 recorder base that anchors Point-LIO frames with the live odometry pose.

Subclass with whatever companion ``In`` ports a given rig wants recorded (camera,
robot odom/lidar, etc.). Point-LIO's ``odometry`` / ``lidar`` outputs are wired to
``pointlio_odometry`` / ``pointlio_lidar`` (via ``.remappings()``), and each lidar
frame is stamped with the latest odometry pose (``@pose_setter_for``) so
``pointlio_lidar`` carries the trajectory and ``dimos map global`` can register the
body-frame cloud directly — no separate ``dimos map pose-fill`` pass.

This is distinct from :mod:`dimos.hardware.sensors.lidar.pointlio.recorder`, the
standalone time-aligning recorder used by the pcap-replay tooling.
"""

from __future__ import annotations

import time

from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Max sensor-ts gap to stamp a lidar frame with the latest odometry pose. Past
# this the odometry is considered stale (Point-LIO dropout/lag) and the frame is
# left unposed -> map-skipped rather than registered at a wrong location. Matches
# PointlioRecorder._POSE_MATCH_TOL / pose_fill's nearest-match window.
_POSE_MATCH_TOL = 0.1


class PointlioPoseRecorderConfig(RecorderConfig):
    # Append into a populated db (keep other streams); replace only our own.
    on_existing: OnExisting = OnExisting.APPEND


class PointlioPoseRecorder(Recorder):
    config: PointlioPoseRecorderConfig

    pointlio_odometry: In[Odometry]
    pointlio_lidar: In[PointCloud2]

    _last_odom_pose: Pose | None = None
    _last_odom_raw_ts: float = 0.0

    @pose_setter_for("pointlio_odometry")
    def _odom_pose(self, msg: Odometry) -> Pose | None:
        pose = getattr(msg, "pose", None)
        self._last_odom_pose = getattr(pose, "pose", None) if pose is not None else None
        raw_ts = getattr(msg, "ts", None)
        self._last_odom_raw_ts = raw_ts if raw_ts is not None else time.time()
        return self._last_odom_pose

    @pose_setter_for("pointlio_lidar")
    def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        # Most-recent odometry pose, stamped directly (no tf) — but only if it's
        # fresh. Stale odometry (older than _POSE_MATCH_TOL) or no odometry yet
        # returns None -> frame stored unposed, map-skipped.
        if self._last_odom_pose is None:
            return None
        raw_ts = getattr(msg, "ts", None)
        raw_ts = raw_ts if raw_ts is not None else time.time()
        if abs(raw_ts - self._last_odom_raw_ts) > _POSE_MATCH_TOL:
            return None
        return self._last_odom_pose
