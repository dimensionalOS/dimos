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

"""Shared fixtures for the binary-driven rtab_map tests.

Each test spins up the real ``rtab_map`` native binary on dedicated LCM
channels, sends synthetic ``Odometry`` + ``PointCloud2`` messages over LCM,
and reads back what the binary publishes. Skips if the binary isn't built.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
import threading
import time
import uuid

import lcm as lcmlib
import numpy as np
import pytest

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    LcmCollector,
    NativeProcessRunner,
    lcm_handle_loop,
    make_odometry_msg,
    make_pointcloud_msg,
)

RTAB_BIN = Path(__file__).resolve().parent.parent / "cpp" / "result" / "bin" / "rtab_map"

_BINARY_STARTUP_SEC = 1.5
_DRAIN_SEC = 2.0


@dataclass
class RtabHarness:
    """One running rtab_map binary plus collectors and a publisher LCM handle."""

    runner: NativeProcessRunner
    publisher: lcmlib.LCM
    receiver: lcmlib.LCM
    handle_thread: threading.Thread
    stop_event: threading.Event
    scan_topic: str
    odom_topic: str
    corrected: LcmCollector
    global_map: LcmCollector
    rtab_tf: LcmCollector
    octomap: LcmCollector
    proj2d: LcmCollector
    _topic_prefix: str = field(default="")

    def publish_scan(self, points: np.ndarray, ts: float) -> None:
        msg = make_pointcloud_msg(points, ts, frame_id="map")
        # The C++ wrapper's lcm_encode helper is not directly exposed for
        # PointCloud2 in the same way as PGO's tests — encode via the msg's
        # lcm_encode method to match what the binary expects.
        self.publisher.publish(self.scan_topic, msg.lcm_encode(frame_id="map"))

    def publish_odom(self, position: np.ndarray, quaternion: np.ndarray, ts: float) -> None:
        msg = make_odometry_msg(position=position, quaternion=quaternion, ts=ts, frame_id="map")
        self.publisher.publish(self.odom_topic, msg.lcm_encode())

    def drain(self, seconds: float = _DRAIN_SEC) -> None:
        time.sleep(seconds)


def _topic(prefix: str, port: str, type_name: str) -> str:
    return f"/{prefix}_{port}#{type_name}"


@pytest.fixture()
def rtab_harness() -> Iterator[RtabHarness]:
    """Spawn the binary on a unique topic prefix; tear it down at the end."""
    if not RTAB_BIN.exists():
        pytest.skip(f"rtab_map binary not found at {RTAB_BIN} — run nix build first")

    prefix = f"rt{uuid.uuid4().hex[:6]}"
    scan_t = _topic(prefix, "scan", "sensor_msgs.PointCloud2")
    odom_t = _topic(prefix, "odom", "nav_msgs.Odometry")
    corr_t = _topic(prefix, "corr", "nav_msgs.Odometry")
    gmap_t = _topic(prefix, "gmap", "sensor_msgs.PointCloud2")
    tf_t = _topic(prefix, "tf", "nav_msgs.Odometry")
    octo_t = _topic(prefix, "octo", "sensor_msgs.PointCloud2")
    proj_t = _topic(prefix, "proj", "sensor_msgs.PointCloud2")

    runner = NativeProcessRunner(
        binary_path=str(RTAB_BIN),
        args=[
            "--registered_scan",
            scan_t,
            "--odometry",
            odom_t,
            "--corrected_odometry",
            corr_t,
            "--global_map",
            gmap_t,
            "--rtab_tf",
            tf_t,
            "--octomap",
            octo_t,
            "--projected_2d_grid",
            proj_t,
            "--octomap_publish_period",
            "0.1",
            "--global_map_publish_period",
            "0.2",
        ],
    )

    receiver = lcmlib.LCM()
    publisher = lcmlib.LCM()

    corrected = LcmCollector(topic=corr_t, msg_type=Odometry)
    global_map = LcmCollector(topic=gmap_t, msg_type=PointCloud2)
    rtab_tf = LcmCollector(topic=tf_t, msg_type=Odometry)
    octomap = LcmCollector(topic=octo_t, msg_type=PointCloud2)
    proj2d = LcmCollector(topic=proj_t, msg_type=PointCloud2)

    for c in (corrected, global_map, rtab_tf, octomap, proj2d):
        c.start(receiver)

    stop_event = threading.Event()
    handle_thread = threading.Thread(
        target=lcm_handle_loop, args=(receiver, stop_event), daemon=True
    )
    handle_thread.start()

    runner.start(capture_stderr=True)
    time.sleep(_BINARY_STARTUP_SEC)
    assert runner.is_running, "rtab_map binary failed to start"

    harness = RtabHarness(
        runner=runner,
        publisher=publisher,
        receiver=receiver,
        handle_thread=handle_thread,
        stop_event=stop_event,
        scan_topic=scan_t,
        odom_topic=odom_t,
        corrected=corrected,
        global_map=global_map,
        rtab_tf=rtab_tf,
        octomap=octomap,
        proj2d=proj2d,
        _topic_prefix=prefix,
    )
    try:
        yield harness
    finally:
        runner.stop()
        stop_event.set()
        handle_thread.join(timeout=2.0)


def square_room_scan(centered_at_origin: bool = True) -> np.ndarray:
    """Body-frame scan of four walls of a 3m x 3m room, with floor points."""
    grid = np.linspace(-1.5, 1.5, 24)
    walls = np.concatenate(
        [
            np.stack([np.full_like(grid, 1.5), grid, np.zeros_like(grid)], axis=1),
            np.stack([np.full_like(grid, -1.5), grid, np.zeros_like(grid)], axis=1),
            np.stack([grid, np.full_like(grid, 1.5), np.zeros_like(grid)], axis=1),
            np.stack([grid, np.full_like(grid, -1.5), np.zeros_like(grid)], axis=1),
        ]
    )
    floor_xy = np.linspace(-1.0, 1.0, 8)
    xx, yy = np.meshgrid(floor_xy, floor_xy)
    floor = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, -0.5)], axis=1)
    cloud = np.concatenate([walls, floor], axis=0)
    intensities = np.ones(len(cloud), dtype=np.float32)
    if not centered_at_origin:
        cloud = cloud + np.array([0.1, 0.0, 0.0])
    return np.column_stack([cloud.astype(np.float32), intensities])


def identity_quat() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0])
