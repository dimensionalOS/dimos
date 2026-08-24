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

"""Replay an alfred recording db onto the stream names the live drivers publish."""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any

import reactivex as rx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, Out
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PROGRESS_INTERVAL_SECONDS = 10.0

STEREO_PAIR_TOLERANCE = 0.001
"""Both imagers in a frameset carry the same stamp; cuVSLAM rejects pairs past 1 ms."""


def _stamp_matched_pairs(left: Any, right: Any) -> Any:
    """Pair two image observables by timestamp instead of arrival order.

    Each stream skips past whatever is already behind the shared clock when it
    subscribes, and the two imagers subscribe an instant apart, so they can enter on
    different frames and an ordinal zip would pair mismatched stamps from then on.
    """

    def subscribe(observer: Any, scheduler: Any = None) -> Any:
        lock = threading.Lock()
        pending: dict[str, Any] = {"left": None, "right": None}
        completed: set[str] = set()

        def on_frame(side: str, other: str, frame: Any) -> None:
            emit = None
            with lock:
                held = pending[other]
                if held is not None and abs(held.ts - frame.ts) <= STEREO_PAIR_TOLERANCE:
                    pending[other] = None
                    emit = (held, frame) if side == "right" else (frame, held)
                elif held is not None and held.ts > frame.ts:
                    pass  # this frame's partner was dropped; the held one still waits
                else:
                    # Any held older frame lost its partner; keep only the newest.
                    pending[other] = None
                    pending[side] = frame
            if emit is not None:
                observer.on_next(emit)

        def on_done(side: str) -> None:
            with lock:
                completed.add(side)
                done = len(completed) == 2
            if done:
                observer.on_completed()

        subscriptions = [
            left.subscribe(
                on_next=lambda f: on_frame("left", "right", f),
                on_completed=lambda: on_done("left"),
            ),
            right.subscribe(
                on_next=lambda f: on_frame("right", "left", f),
                on_completed=lambda: on_done("right"),
            ),
        ]

        def dispose() -> None:
            for subscription in subscriptions:
                subscription.dispose()

        return Disposable(dispose)

    return rx.create(subscribe)


LIVE_PARENT_FRAMES = {"odom", "map", "visual_odom"}
"""Recorded tf edges under these parents (and any edge onto base_link) belong to the
filter that ran at record time; the replayed stack publishes its own, and replaying
both would give base_link two parents."""


class AlfredReplayConfig(ModuleConfig):
    db_path: str = ""
    tf_stream: str = "tf"
    # Faster than real time outruns the CPU cuVSLAM tracker, which then falls behind
    # the other streams and fuses stale visual deltas: the trajectory changes.
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    publish_wheel_odometry: bool = True
    # The vision-only stack never consumes the lidar; it costs decode CPU that the
    # tracker needs, so it stays off unless a comparison wants it.
    publish_lidar: bool = False
    done_file: str = ""
    """Touched once every stream has emitted its last message."""


class AlfredReplay(Module):
    """Publish a recording's sensor streams back onto live streams, in recorded order.

    Both infrared imagers and the colour image go onto the one ``image`` stream and are
    told apart downstream by ``frame_id``, exactly as the live camera driver does.
    """

    config: AlfredReplayConfig

    image: Out[Image]
    color_image: Out[Image]
    color_camera_info: Out[CameraInfo]
    camera_info: Out[CameraInfo]
    depth_image: Out[Image]
    depth_camera_info: Out[CameraInfo]
    imu: Out[Imu]
    imu_info: Out[ImuInfo]
    lidar: Out[PointCloud2]
    tf: IO[TFMessage]
    source_odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._subscriptions: list[Any] = []
        self._lock = threading.Lock()
        self._latest_ts: dict[str, float] = {}
        self._running: set[str] = set()
        self._started_at = 0.0
        self._first_ts = 0.0
        self._progress_timer: threading.Timer | None = None

    def _track(self, name: str, observable: Any, on_next: Any, ts_of: Any = None) -> None:
        """Subscribe while recording how far this stream has got, and when it ends."""
        self._running.add(name)
        read_ts = ts_of or (lambda value: getattr(value, "ts", None))

        def next_(value: Any) -> None:
            ts = read_ts(value)
            if ts is not None:
                with self._lock:
                    self._latest_ts[name] = float(ts)
                    if not self._first_ts:
                        self._first_ts = float(ts)
            on_next(value)

        self._subscriptions.append(
            observable.subscribe(on_next=next_, on_completed=lambda: self._stream_finished(name))
        )

    def _stream_finished(self, name: str) -> None:
        with self._lock:
            self._running.discard(name)
            remaining = len(self._running)
        logger.info("replay stream %s drained, %d still running", name, remaining)
        if remaining:
            return
        self._report_progress(schedule_next=False)
        if self.config.done_file:
            Path(self.config.done_file).write_text(str(time.time()))

    def _report_progress(self, schedule_next: bool = True) -> None:
        with self._lock:
            latest = dict(self._latest_ts)
            running = set(self._running)
        if latest:
            leader = max(latest, key=lambda name: latest[name])
            trailer = min(latest, key=lambda name: latest[name])
            elapsed = time.time() - self._started_at
            covered = latest[leader] - self._first_ts
            logger.info(
                "replay %.0fs of recording in %.0fs wall (%.2fx), spread %.2fs "
                "(%s ahead of %s), %d streams running",
                covered,
                elapsed,
                covered / elapsed if elapsed > 0 else 0.0,
                latest[leader] - latest[trailer],
                leader,
                trailer,
                len(running),
            )
        if schedule_next and running:
            self._progress_timer = threading.Timer(PROGRESS_INTERVAL_SECONDS, self._report_progress)
            self._progress_timer.daemon = True
            self._progress_timer.start()

    @rpc
    def start(self) -> None:
        super().start()
        store = SqliteStore(path=self.config.db_path, must_exist=True)
        replay_kwargs: dict[str, Any] = {"speed": self.config.speed}
        if self.config.seek is not None:
            replay_kwargs["seek"] = self.config.seek
        if self.config.duration is not None:
            replay_kwargs["duration"] = self.config.duration
        replay = store.replay(**replay_kwargs)

        stereo = _stamp_matched_pairs(
            replay.stream("infrared_left").observable(),
            replay.stream("infrared_right").observable(),
        )
        self._started_at = time.time()

        def publish_stereo(pair: tuple[Image, Image]) -> None:
            for frame in pair:
                self.image.publish(frame)

        self._track("infrared", stereo, publish_stereo, ts_of=lambda pair: pair[0].ts)

        outputs: list[tuple[str, Out[Any]]] = [
            ("infrared_left_camera_info", self.camera_info),
            ("infrared_right_camera_info", self.camera_info),
            # The recorded camera_info stream is the colour intrinsics, published live
            # as color_camera_info by the alfred-mls-nav remapping.
            ("camera_info", self.color_camera_info),
            ("depth_image", self.depth_image),
            ("depth_camera_info", self.depth_camera_info),
            ("imu", self.imu),
            ("imu_info", self.imu_info),
        ]
        if self.config.publish_wheel_odometry:
            outputs.append(("wheel_odometry", self.source_odometry))
        if self.config.publish_lidar:
            outputs.append(("lidar", self.lidar))
        for name, port in outputs:
            self._track(name, replay.stream(name).observable(), port.publish)

        def publish_colour(frame: Image) -> None:
            self.image.publish(frame)
            self.color_image.publish(frame)

        self._track("color_image", replay.stream("color_image").observable(), publish_colour)
        self._track(
            self.config.tf_stream,
            replay.stream(self.config.tf_stream).observable(),
            self._publish_tf,
        )
        self._report_progress()

    def _publish_tf(self, message: TFMessage) -> None:
        kept = [
            transform
            for transform in message.transforms
            if transform.frame_id not in LIVE_PARENT_FRAMES
            and transform.child_frame_id != "base_link"
        ]
        if kept:
            message.transforms = kept
            self.tf.publish(message)

    @rpc
    def stop(self) -> None:
        if self._progress_timer is not None:
            self._progress_timer.cancel()
        for subscription in self._subscriptions:
            subscription.dispose()
        super().stop()
