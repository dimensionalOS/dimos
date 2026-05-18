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

"""Module that streams a KITTI-360 sequence as scan + odometry messages.

Pairs with any module satisfying ``LoopClosure``
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.benchmarks.pose_graph_kitti360.kitti360_loader import (
    load_kitti360_sequence,
)
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    make_odometry_msg,
    make_pointcloud_msg,
)

FIRST_RESPONSE_TIMEOUT_SEC = 120.0


class Kitti360PlaybackConfig(ModuleConfig):
    kitti360_root: str
    sequence_id: int = 2
    max_scans: int | None = None
    publish_interval_sec: float = 0.02
    first_response_timeout_sec: float = FIRST_RESPONSE_TIMEOUT_SEC


class Kitti360PlaybackModule(Module):
    """Replays a KITTI-360 sequence at a controlled rate."""

    config: Kitti360PlaybackConfig

    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]
    corrected_odometry: In[Odometry]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._frame_ids: list[int] = []
        self._send_timestamps: list[float] = []
        self._frames_published: int = 0
        self._playback_finished: bool = False
        self._playback_error: str | None = None
        self._first_response_event: asyncio.Event | None = None

    async def handle_corrected_odometry(self, value: Odometry) -> None:
        if self._first_response_event is not None:
            self._first_response_event.set()

    async def main(self) -> AsyncIterator[None]:
        self._sequence = load_kitti360_sequence(
            Path(self.config.kitti360_root), self.config.sequence_id
        )
        frame_ids = self._sequence.frame_ids
        if self.config.max_scans is not None:
            frame_ids = frame_ids[: self.config.max_scans]
        self._frame_ids = frame_ids
        self._send_timestamps = compute_send_timestamps(self._sequence.timestamps, frame_ids)
        # Event lives on self._loop, the same loop _run_playback and
        # handle_corrected_odometry run on.
        self._first_response_event = asyncio.Event()
        self._playback_task = asyncio.create_task(self._run_playback())
        yield
        self._playback_task.cancel()

    async def _run_playback(self) -> None:
        try:
            assert self._first_response_event is not None
            for index, frame_id in enumerate(self._frame_ids):
                # scan_xyz is a blocking np.fromfile — push it to a thread so
                # the event loop (and any concurrent RPC) keeps spinning.
                scan_xyz = await asyncio.to_thread(self._sequence.scan_xyz, frame_id)
                pose = self._sequence.lidar_pose(frame_id)
                position = pose[:3, 3]
                quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
                timestamp = self._send_timestamps[index]

                odometry_message = make_odometry_msg(position, quaternion, ts=timestamp)
                world_xyz = (pose[:3, :3] @ scan_xyz[:, :3].T).T + position
                cloud_array = np.column_stack([world_xyz, scan_xyz[:, 3:4]]).astype(np.float32)
                cloud_message = make_pointcloud_msg(cloud_array, ts=timestamp)

                # Odometry first so the receiver can stash the latest pose
                # before the matching scan arrives.
                self.odometry.publish(odometry_message)
                self.registered_scan.publish(cloud_message)

                self._frames_published = index + 1
                if index == 0:
                    try:
                        await asyncio.wait_for(
                            self._first_response_event.wait(),
                            timeout=self.config.first_response_timeout_sec,
                        )
                    except asyncio.TimeoutError:
                        raise RuntimeError(
                            "No corrected_odometry within "
                            f"{self.config.first_response_timeout_sec:.1f}s of "
                            "the first scan — playback aborted"
                        ) from None
                if self.config.publish_interval_sec > 0:
                    await asyncio.sleep(self.config.publish_interval_sec)
        except Exception as exc:
            self._playback_error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self._playback_finished = True

    @rpc
    def frames_published(self) -> int:
        return self._frames_published

    @rpc
    def is_finished(self) -> bool:
        return self._playback_finished

    @rpc
    def playback_error(self) -> str | None:
        return self._playback_error

    @rpc
    def send_timestamps(self) -> list[float]:
        return list(self._send_timestamps)

    @rpc
    def frame_ids(self) -> list[int]:
        return list(self._frame_ids)


def compute_send_timestamps(
    raw_timestamps: dict[int, float], frame_ids_in_order: list[int]
) -> list[float]:
    """Compute strictly-monotonic publish timestamps from raw KITTI ones.

    PGO's Odometry constructor treats ``ts==0`` as "now", so clamp the first
    ts away from zero; subsequent values inherit at least a 1 ms floor.
    """
    if not frame_ids_in_order:
        return []
    first_timestamp = max(raw_timestamps.get(frame_ids_in_order[0], 1.0), 1.0)
    send_timestamps: list[float] = []
    for index, frame_id in enumerate(frame_ids_in_order):
        raw_timestamp = raw_timestamps.get(frame_id, float(index))
        send_timestamps.append(max(raw_timestamp, first_timestamp + index * 0.001))
    return send_timestamps
