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

"""Shared fixtures for rosbag-based nav_stack module validation tests.

Loads recorded data from the OG ROS navigation autonomy stack and provides
helpers for feeding it to DimOS native modules via LCM at original timing,
then capturing and comparing outputs with deviation scores.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

import lcm as lcmlib
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

DATA_DIR = Path(__file__).parent / "data"
ROSBAG_FIXTURE_60S = DATA_DIR / "og_nav_60s.npz"


@dataclass
class RosbagWindow:
    """A time window of recorded nav stack data with original timestamps."""

    odom: np.ndarray  # (N, 8): t, px, py, pz, qx, qy, qz, qw
    way_point: np.ndarray  # (N, 4): t, x, y, z
    cmd_vel: np.ndarray  # (N, 7): t, lx, ly, lz, ax, ay, az
    goal: np.ndarray  # (N, 4): t, x, y, z
    path_endpoints: np.ndarray  # (N, 5): t, n_poses, last_x, last_y, arc_length
    scans: list[tuple[float, np.ndarray]]  # [(t, points_Nx3), ...]
    terrain_maps: list[tuple[float, np.ndarray]]
    terrain_maps_ext: list[tuple[float, np.ndarray]]
    paths: list[tuple[float, np.ndarray]]  # [(t, poses_Nx7: x,y,z,qx,qy,qz,qw), ...]


def load_rosbag_window(path: Path = ROSBAG_FIXTURE_60S) -> RosbagWindow:
    """Load a pre-extracted rosbag fixture."""
    if not path.exists():
        pytest.skip(f"Rosbag fixture not found: {path}")

    data = np.load(str(path), allow_pickle=False)

    def load_indexed(prefix: str, data_suffix: str = "pts") -> list[tuple[float, np.ndarray]]:
        result = []
        for i in range(500):
            t_key = f"{prefix}_{i}_t"
            d_key = f"{prefix}_{i}_{data_suffix}"
            if t_key not in data:
                break
            result.append((float(data[t_key][0]), data[d_key]))
        return result

    return RosbagWindow(
        odom=data["odom"],
        way_point=data["way_point"],
        cmd_vel=data["cmd_vel"],
        goal=data.get("goal", np.zeros((0, 4))),
        path_endpoints=data.get("path_endpoints", np.zeros((0, 5))),
        scans=load_indexed("scan"),
        terrain_maps=load_indexed("tmap"),
        terrain_maps_ext=load_indexed("tmap_ext"),
        paths=load_indexed("path", "poses"),
    )


def make_odometry_msg(
    pos: np.ndarray, quat: np.ndarray, ts: float, frame_id: str = "map", child_frame: str = "sensor"
) -> Odometry:
    """Build an Odometry message from position + quaternion arrays."""
    pose = Pose()
    pose.position.x = float(pos[0])
    pose.position.y = float(pos[1])
    pose.position.z = float(pos[2])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return Odometry(ts=ts, frame_id=frame_id, child_frame_id=child_frame, pose=pose)


def make_pointcloud_msg(points: np.ndarray, ts: float, frame_id: str = "map") -> PointCloud2:
    """Build a PointCloud2 message from an Nx3 numpy array."""
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id=frame_id, timestamp=ts)


def make_waypoint_msg(
    x: float, y: float, z: float, ts: float, frame_id: str = "map"
) -> PointStamped:
    """Build a PointStamped message."""
    return PointStamped(ts=ts, frame_id=frame_id, x=x, y=y, z=z)


def publish_lcm(lc: lcmlib.LCM, topic: str, msg: Any) -> None:
    """Encode and publish a DimOS message over LCM."""
    lc.publish(topic, msg.lcm_encode())


@dataclass
class LcmCollector:
    """Subscribes to an LCM topic and collects decoded messages with timestamps."""

    topic: str
    msg_type: type
    messages: list[Any] = field(default_factory=list)
    timestamps: list[float] = field(default_factory=list)
    _sub: Any = field(default=None, repr=False)

    def start(self, lc: lcmlib.LCM) -> None:
        msg_cls = self.msg_type

        def handler(_channel: str, data: bytes) -> None:
            try:
                msg = msg_cls.lcm_decode(data)  # type: ignore[attr-defined]
                self.messages.append(msg)
                self.timestamps.append(time.monotonic())
            except Exception as exc:
                import sys

                print(f"LcmCollector decode error on {self.topic}: {exc}", file=sys.stderr)

        self._sub = lc.subscribe(self.topic, handler)

    def stop(self, lc: lcmlib.LCM) -> None:
        if self._sub is not None:
            lc.unsubscribe(self._sub)
            self._sub = None


def lcm_handle_loop(lc: lcmlib.LCM, stop_event: threading.Event, timeout_ms: int = 50) -> None:
    """Run LCM handle loop until stop_event is set."""
    while not stop_event.is_set():
        lc.handle_timeout(timeout_ms)


@dataclass
class NativeProcessRunner:
    """Start and manage a native module C++ process for testing."""

    binary_path: str
    args: list[str]
    process: subprocess.Popen[bytes] | None = field(default=None, repr=False)

    def start(self) -> None:
        self.process = subprocess.Popen(
            [self.binary_path, *self.args],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    def stop(self, timeout: float = 3.0) -> None:
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            self.process = None

    @property
    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None


def feed_at_original_timing(
    lc: lcmlib.LCM,
    window: RosbagWindow,
    topic_map: dict[str, str],
    odom_subsample: int = 4,
) -> None:
    """Replay recorded data over LCM at the original inter-message timing.

    Args:
        lc: LCM instance.
        window: Loaded rosbag data.
        topic_map: Maps data key to LCM topic string. Keys:
            "odom", "scan", "terrain", "terrain_ext", "waypoint", "goal"
        odom_subsample: Keep every Nth odom message (200Hz is excessive for testing).
            Use 4 for ~50Hz, 1 for full rate.
    """
    timeline: list[tuple[float, str, Any]] = []

    # Odom at subsampled rate
    for i in range(0, len(window.odom), odom_subsample):
        row = window.odom[i]
        msg = make_odometry_msg(row[1:4], row[4:8], ts=row[0])
        timeline.append((row[0], topic_map.get("odom", ""), msg))

    for t, pts in window.scans:
        if "scan" in topic_map:
            timeline.append((t, topic_map["scan"], make_pointcloud_msg(pts, ts=t)))

    for t, pts in window.terrain_maps:
        if "terrain" in topic_map:
            timeline.append((t, topic_map["terrain"], make_pointcloud_msg(pts, ts=t)))

    for t, pts in window.terrain_maps_ext:
        if "terrain_ext" in topic_map:
            timeline.append((t, topic_map["terrain_ext"], make_pointcloud_msg(pts, ts=t)))

    for row in window.way_point:
        if "waypoint" in topic_map:
            msg = make_waypoint_msg(float(row[1]), float(row[2]), float(row[3]), ts=float(row[0]))
            timeline.append((float(row[0]), topic_map["waypoint"], msg))

    for row in window.goal:
        if "goal" in topic_map:
            msg = make_waypoint_msg(float(row[1]), float(row[2]), float(row[3]), ts=float(row[0]))
            timeline.append((float(row[0]), topic_map["goal"], msg))

    # Path messages (for PathFollower testing)
    for t, pose_arr in window.paths:
        if "path" not in topic_map or len(pose_arr) == 0:
            continue
        poses = []
        for row in pose_arr:
            poses.append(
                PoseStamped(
                    ts=t,
                    frame_id="map",
                    position=[float(row[0]), float(row[1]), float(row[2])],
                    orientation=[float(row[3]), float(row[4]), float(row[5]), float(row[6])],
                )
            )
        path_msg = NavPath(ts=t, frame_id="map", poses=poses)
        timeline.append((t, topic_map["path"], path_msg))

    timeline.sort(key=lambda x: x[0])
    timeline = [(t, topic, msg) for t, topic, msg in timeline if topic]

    if not timeline:
        return

    t_start = timeline[0][0]
    real_start = time.monotonic()
    for t, topic, msg in timeline:
        target_dt = t - t_start
        elapsed = time.monotonic() - real_start
        if target_dt > elapsed:
            time.sleep(target_dt - elapsed)
        publish_lcm(lc, topic, msg)
