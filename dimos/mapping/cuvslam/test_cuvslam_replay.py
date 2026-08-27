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

import asyncio
from collections.abc import AsyncIterator, Iterator
from concurrent.futures import Future
import heapq
import math
import time
from typing import Any

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import get_data

# 12 s of alfred (D455) driving ~2.5 m: stereo IR pairs at 15 fps, camera infos, tf.
SNIPPET = "alfred_stereo_short.db"
SNIPPET_STEREO_PAIRS = 179
LOCKSTEP_FAILURE = "Either the machine is too slow or something with cuVSLAM is broken"

# (db stream, out port), in publish order at equal timestamps.
STREAM_PORTS = (
    ("tf", "tf"),
    ("infrared_left_camera_info", "camera_info"),
    ("infrared_right_camera_info", "camera_info"),
    ("infrared_left", "image"),
    ("infrared_right", "image"),
)
# The right image completes a stereo pair, so it is the lockstep trigger.
PAIR_TRIGGER_STREAM = "infrared_right"
CAMERA_FRAMES = ("camera_infra1_optical_frame", "camera_infra2_optical_frame")


class StereoReplayConfig(ModuleConfig):
    db_path: str
    odometry_timeout: float = 10.0


class StereoReplayModule(Module):
    """Workaround until we get pure modules: Lockstep replay: publish one stereo pair, wait for cuvslam's odometry, repeat.

    Pacing input by cuvslam's own output keeps slow and fast machines on the
    same input sequence, so the run stays deterministic.
    """

    config: StereoReplayConfig

    image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]
    odometry: In[Odometry]

    async def main(self) -> AsyncIterator[None]:
        self._positions: list[tuple[float, float, float]] = []
        self._pairs_sent = 0
        self._replay_done = False
        self._error: str | None = None
        self._odometry_arrived = asyncio.Event()
        self._replay_future: Future[Any] | None = None
        yield
        if self._replay_future is not None:
            self._replay_future.cancel()

    async def handle_odometry(self, value: Odometry) -> None:
        point = value.pose.pose.position
        self._positions.append((point.x, point.y, point.z))
        self._odometry_arrived.set()

    @rpc
    def start_replay(self) -> None:
        self._replay_future = self.spawn(self._replay())

    @rpc
    def progress(self) -> dict[str, Any]:
        return {
            "done": self._replay_done,
            "error": self._error,
            "pairs_sent": self._pairs_sent,
            "positions": list(self._positions),
        }

    async def _replay(self) -> None:
        store = SqliteStore(path=self.config.db_path, must_exist=True)
        store.start()
        try:
            replay = store.replay()
            # The intrinsics and the base_link->camera tf chain are static but
            # trickle in after the first image; send them up front so the rig is
            # constructible at the first stereo pair.
            for stream_name, port_name in STREAM_PORTS:
                if port_name == "camera_info":
                    self.camera_info.publish(replay.stream(stream_name).first())
            missing_frames = set(CAMERA_FRAMES)
            for _ts, message in replay.stream("tf").iterate_ts():
                self.tf.publish(message)
                missing_frames -= {t.child_frame_id for t in message.transforms}
                if not missing_frames:
                    break

            def tagged(
                priority: int, stream_name: str, port_name: str
            ) -> Iterator[tuple[Any, ...]]:
                for ts, message in replay.stream(stream_name).iterate_ts():
                    yield ts, priority, stream_name, port_name, message

            merged = heapq.merge(
                *(tagged(priority, *names) for priority, names in enumerate(STREAM_PORTS)),
                key=lambda item: item[:2],
            )
            for _ts, _priority, stream_name, port_name, message in merged:
                if stream_name != PAIR_TRIGGER_STREAM:
                    getattr(self, port_name).publish(message)
                    continue
                self._odometry_arrived.clear()
                self.image.publish(message)
                self._pairs_sent += 1
                try:
                    await asyncio.wait_for(
                        self._odometry_arrived.wait(), self.config.odometry_timeout
                    )
                except asyncio.TimeoutError:
                    self._error = (
                        f"no odometry for stereo pair {self._pairs_sent}. {LOCKSTEP_FAILURE}"
                    )
                    return
        finally:
            store.stop()
            self._replay_done = True


@pytest.mark.self_hosted
def test_cuvslam_stereo_replay() -> None:
    """Replay a short alfred stereo recording through CuvslamOdometry"""
    coordinator = ModuleCoordinator.build(
        autoconnect(
            CuvslamOdometry.blueprint(camera_mode="stereo"),
            StereoReplayModule.blueprint(db_path=str(get_data(SNIPPET))),
        ).global_config(transport="lcm", viewer="none")
    )
    try:
        replay = coordinator.get_instance(StereoReplayModule)
        assert replay is not None
        replay.start_replay()
        while not (progress := replay.progress())["done"]:
            time.sleep(1.0)
    finally:
        coordinator.stop()

    positions = progress["positions"]
    assert progress["error"] is None, progress["error"]
    assert len(positions) >= SNIPPET_STEREO_PAIRS // 2, (
        f"expected odometry for most stereo pairs, got {len(positions)}"
    )
    displacement = math.dist(positions[0], positions[-1])
    assert 1.0 < displacement < 5.0, (
        f"net displacement {displacement:.2f} m outside the ~2.5 m recorded drive"
    )
