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

import asyncio
from collections.abc import AsyncIterator
from dataclasses import dataclass, field
import itertools
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

import lcm as lcmlib
import numpy as np
import pytest

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Upper bound on the number of indexed entries (scan_0..scan_N, tmap_0..tmap_N,
# path_0..path_N) inside a rosbag npz. Loop breaks early when keys run out;
# this is just the safety ceiling.
MAX_INDEXED_ENTRIES = 500


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


def load_rosbag_window(path: Path | None = None) -> RosbagWindow:
    """Load a pre-extracted rosbag fixture (defaults to the 60s OG-nav recording)."""
    if path is None:
        path = get_data("og_nav_60s.npz")
    if not path.exists():
        pytest.skip(f"Rosbag fixture not found: {path}")

    data = np.load(str(path), allow_pickle=False)

    def load_indexed(prefix: str, data_suffix: str = "pts") -> list[tuple[float, np.ndarray]]:
        result = []
        for index in range(MAX_INDEXED_ENTRIES):
            time_key = f"{prefix}_{index}_t"
            data_key = f"{prefix}_{index}_{data_suffix}"
            if time_key not in data:
                break
            result.append((float(data[time_key][0]), data[data_key]))
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
    position: np.ndarray,
    quaternion: np.ndarray,
    ts: float,
    frame_id: str = "map",
    child_frame: str = "sensor",
) -> Odometry:
    """Build an Odometry message from position + quaternion arrays."""
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])
    return Odometry(ts=ts, frame_id=frame_id, child_frame_id=child_frame, pose=pose)


def make_pointcloud_msg(points: np.ndarray, ts: float, frame_id: str = "map") -> PointCloud2:
    points_f32 = points.astype(np.float32)
    if points_f32.ndim == 2 and points_f32.shape[1] >= 4:
        return PointCloud2.from_numpy(
            points_f32[:, :3], frame_id=frame_id, timestamp=ts, intensities=points_f32[:, 3]
        )
    return PointCloud2.from_numpy(points_f32, frame_id=frame_id, timestamp=ts)


def make_waypoint_msg(
    x: float, y: float, z: float, ts: float, frame_id: str = "map"
) -> PointStamped:
    return PointStamped(ts=ts, frame_id=frame_id, x=x, y=y, z=z)


@dataclass
class LcmCollector:
    """Subscribes to an LCM topic and collects decoded messages with timestamps."""

    topic: str
    message_type: type
    messages: list[Any] = field(default_factory=list)
    timestamps: list[float] = field(default_factory=list)
    _subscription: Any = field(default=None, repr=False)

    def start(self, lcm: lcmlib.LCM) -> None:
        message_class = self.message_type

        def handler(_channel: str, data: bytes) -> None:
            try:
                message = message_class.lcm_decode(data)  # type: ignore[attr-defined]
                self.messages.append(message)
                self.timestamps.append(time.monotonic())
            except Exception as exc:
                logger.error(f"LcmCollector decode error on {self.topic}: {exc}")

        self._subscription = lcm.subscribe(self.topic, handler)

    def stop(self, lcm: lcmlib.LCM) -> None:
        if self._subscription is not None:
            lcm.unsubscribe(self._subscription)
            self._subscription = None


def lcm_handle_loop(lcm: lcmlib.LCM, stop_event: threading.Event, timeout_ms: int = 50) -> None:
    """Run LCM handle loop until stop_event is set."""
    while not stop_event.is_set():
        lcm.handle_timeout(timeout_ms)


_isolated_lcm_url_counter = itertools.count()


def make_isolated_lcm_url() -> str:
    """Return an LCM URL that should not collide with concurrent runs.

    Uses the standard multicast group with TTL=0 (so traffic never escapes
    the local host) and a port picked from the high ephemeral range. The
    monotonic counter guarantees back-to-back calls in the same process get
    distinct ports; mixing in the PID disjoint-ifies concurrent CI workers.
    """
    # ports 49152..65535 are the IANA "dynamic/private" range
    span = 16000
    pid_offset = os.getpid() % span
    call_offset = next(_isolated_lcm_url_counter)
    port = 49152 + ((pid_offset + call_offset) % span)
    return f"udpm://239.255.76.67:{port}?ttl=0"


@dataclass
class NativeProcessRunner:
    """Start and manage a native module C++ process for testing."""

    binary_path: str
    args: list[str]
    process: subprocess.Popen[bytes] | None = field(default=None, repr=False)

    def start(
        self,
        capture_stderr: bool = False,
        env: dict[str, str] | None = None,
    ) -> None:
        process_env: dict[str, str] | None
        if env is None:
            process_env = None
        else:
            process_env = {**os.environ, **env}
        self.process = subprocess.Popen(
            [self.binary_path, *self.args],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE if capture_stderr else subprocess.DEVNULL,
            start_new_session=True,
            env=process_env,
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
    lcm: lcmlib.LCM,
    window: RosbagWindow,
    topic_map: dict[str, str],
    odom_subsample: int = 4,
) -> None:
    """Replay recorded data over LCM at the original inter-message timing.

    Args:
        topic_map: key to LCM topic string. Keys:
            "odom", "scan", "terrain", "terrain_ext", "waypoint", "goal"
        odom_subsample: Keep every Nth odom message (200Hz is excessive for testing).
            Use 4 for ~50Hz, 1 for full rate.
    """
    timeline: list[tuple[float, str, Any]] = []

    # Odom at subsampled rate
    for odom_index in range(0, len(window.odom), odom_subsample):
        row = window.odom[odom_index]
        msg = make_odometry_msg(row[1:4], row[4:8], ts=row[0])
        timeline.append((row[0], topic_map.get("odom", ""), msg))

    for timestamp, points in window.scans:
        if "scan" in topic_map:
            timeline.append(
                (timestamp, topic_map["scan"], make_pointcloud_msg(points, ts=timestamp))
            )

    for timestamp, points in window.terrain_maps:
        if "terrain" in topic_map:
            timeline.append(
                (timestamp, topic_map["terrain"], make_pointcloud_msg(points, ts=timestamp))
            )

    for timestamp, points in window.terrain_maps_ext:
        if "terrain_ext" in topic_map:
            timeline.append(
                (timestamp, topic_map["terrain_ext"], make_pointcloud_msg(points, ts=timestamp))
            )

    for row in window.way_point:
        if "waypoint" in topic_map:
            msg = make_waypoint_msg(float(row[1]), float(row[2]), float(row[3]), ts=float(row[0]))
            timeline.append((float(row[0]), topic_map["waypoint"], msg))

    for row in window.goal:
        if "goal" in topic_map:
            msg = make_waypoint_msg(float(row[1]), float(row[2]), float(row[3]), ts=float(row[0]))
            timeline.append((float(row[0]), topic_map["goal"], msg))

    # Path messages (for PathFollower testing)
    for timestamp, pose_array in window.paths:
        if "path" not in topic_map or len(pose_array) == 0:
            continue
        poses = []
        for row in pose_array:
            poses.append(
                PoseStamped(
                    ts=timestamp,
                    frame_id="map",
                    position=[float(row[0]), float(row[1]), float(row[2])],
                    orientation=[float(row[3]), float(row[4]), float(row[5]), float(row[6])],
                )
            )
        path_msg = NavPath(ts=timestamp, frame_id="map", poses=poses)
        timeline.append((timestamp, topic_map["path"], path_msg))

    timeline.sort(key=lambda entry: entry[0])
    timeline = [(timestamp, topic, msg) for timestamp, topic, msg in timeline if topic]

    if not timeline:
        return

    start_timestamp = timeline[0][0]
    real_start = time.monotonic()
    for timestamp, topic, msg in timeline:
        target_offset = timestamp - start_timestamp
        elapsed = time.monotonic() - real_start
        if target_offset > elapsed:
            time.sleep(target_offset - elapsed)
        lcm.publish(topic, msg.lcm_encode())


PLAYBACK_STARTUP_DELAY_SEC = 2.0


class RosbagScanOdomPlaybackConfig(ModuleConfig):
    rosbag_path: str | None = None
    odom_subsample: int = 4
    startup_delay_sec: float = PLAYBACK_STARTUP_DELAY_SEC


class RosbagScanOdomPlaybackModule(Module):
    """Replays scan + odom from a rosbag fixture at original inter-message timing."""

    config: RosbagScanOdomPlaybackConfig

    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._frames_published: int = 0
        self._playback_finished: bool = False
        self._playback_error: str | None = None

    async def main(self) -> AsyncIterator[None]:
        rosbag_path = Path(self.config.rosbag_path) if self.config.rosbag_path else None
        self._window = load_rosbag_window(rosbag_path)
        self._playback_task = asyncio.create_task(self._run_playback())
        yield
        self._playback_task.cancel()

    async def _run_playback(self) -> None:
        # finally guarantees is_finished() flips to True even if a frame
        # raises. Without it, callers polling is_finished() would spin
        # forever and the coordinator would never tear down.
        try:
            if self.config.startup_delay_sec > 0:
                await asyncio.sleep(self.config.startup_delay_sec)
            timeline: list[tuple[str, float, Any]] = []
            for odom_index in range(0, len(self._window.odom), self.config.odom_subsample):
                row = self._window.odom[odom_index]
                timeline.append(
                    ("odom", float(row[0]), make_odometry_msg(row[1:4], row[4:8], ts=row[0]))
                )
            for timestamp, points in self._window.scans:
                timeline.append(
                    ("scan", float(timestamp), make_pointcloud_msg(points, ts=timestamp))
                )
            timeline.sort(key=lambda entry: entry[1])
            if not timeline:
                return

            bag_start_time = timeline[0][1]
            wallclock_start = time.monotonic()
            for kind, bag_timestamp, message in timeline:
                target_wallclock = wallclock_start + (bag_timestamp - bag_start_time)
                now = time.monotonic()
                if target_wallclock > now:
                    await asyncio.sleep(target_wallclock - now)
                if kind == "odom":
                    self.odometry.publish(message)
                else:
                    self.registered_scan.publish(message)
                self._frames_published += 1
        except Exception as exc:
            self._playback_error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self._playback_finished = True

    @rpc
    def is_finished(self) -> bool:
        return self._playback_finished

    @rpc
    def playback_error(self) -> str | None:
        return self._playback_error

    @rpc
    def frames_published(self) -> int:
        return self._frames_published
