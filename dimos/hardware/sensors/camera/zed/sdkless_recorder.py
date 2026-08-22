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

"""Record the SDK-free ZED modules (stereo h264 + camera_info + ~800 Hz IMU).

In port names match the producers' Out names (``ZedUvcCamera``'s
``color_image_left`` / ``color_image_right`` / ``camera_info_*``, ``ZedImu``'s
``zed_imu``), so autoconnect wires everything with no remappings. Subclass to
record additional rig streams (see ``ZedMid360Recorder``).
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder
from dimos.memory2.stream import Stream
from dimos.memory2.type.observation import Observation
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The high-rate IMU port that gets the batched, tf-free recording path below.
IMU_PORT = "zed_imu"
# How often to flush buffered IMU samples as one transaction (~20 samples/flush
# at 800 Hz) — far under the recorder's per-message drain rate, so nothing is
# dropped, unlike the default LATEST-coalescing single-message dispatch.
IMU_FLUSH_INTERVAL = 0.05


class ZedRecorder(Recorder):
    color_image_left: In[CompressedVideo]  # h264; decode with H264Decoder
    color_image_right: In[CompressedVideo]
    camera_info_left: In[CameraInfo]
    camera_info_right: In[CameraInfo]
    zed_imu: In[Imu]  # ZED-M onboard IMU, ~800 Hz

    _flush_running: bool = False

    @rpc
    def start(self) -> None:
        self._imu_buf: list[tuple[float, Any]] = []
        self._imu_lock = threading.Lock()
        self._imu_backend: Any = None
        super().start()
        self._flush_running = True
        if self._imu_backend is not None:
            self.spawn(self._imu_flush_loop())

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Record most ports the normal way; give the ~800 Hz IMU a custom path.

        The base recorder dispatches every message through a LATEST-coalescing
        mailbox (drops intermediate messages when the handler can't keep up) and
        does a tf pose-lookup + a committed insert per message — which caps the
        IMU at ~70 Hz. For ``zed_imu`` instead: subscribe raw (no coalescing, so
        no drops), skip the tf lookup entirely (IMU carries no world pose), and
        buffer off the event loop; :meth:`_imu_flush_loop` writes each batch as a
        single transaction (N inserts, one commit).
        """
        if name != IMU_PORT:
            super()._port_to_stream(name, input_topic, stream)
            return
        self._imu_backend = stream._source

        def _buffer(msg: Any) -> None:
            with self._imu_lock:
                self._imu_buf.append((time.time(), msg))

        self.register_disposable(input_topic.pure_observable().subscribe(_buffer))
        logger.info(
            "Recording %s -> %s (%s) [batched, tf-free]",
            name,
            stream.name,
            input_topic.type.__name__,
        )

    def _flush_imu(self) -> int:
        """Write all buffered IMU samples as one transaction. Runs on the event
        loop, so it is serialised against the other streams' writes."""
        backend = self._imu_backend
        if backend is None:
            return 0
        with self._imu_lock:
            batch = self._imu_buf
            self._imu_buf = []
        if not batch:
            return 0
        for recv_ts, msg in batch:
            ts = getattr(msg, "ts", None) or recv_ts
            obs = Observation(id=-1, ts=ts, pose=None, tags={"reception_ts": recv_ts}, _data=msg)
            encoded = backend.codec.encode(msg)
            row_id = backend.metadata_store.insert(obs)  # no commit
            if backend.blob_store is not None:
                backend.blob_store.put(backend.name, row_id, encoded)  # no commit
        backend.metadata_store.commit()  # single commit for the whole batch
        return len(batch)

    async def _imu_flush_loop(self) -> None:
        while self._flush_running:
            await asyncio.sleep(IMU_FLUSH_INTERVAL)
            try:
                self._flush_imu()
            except Exception:
                logger.exception("ZED IMU batch flush failed")

    @rpc
    def stop(self) -> None:
        self._flush_running = False  # ends _imu_flush_loop
        try:
            flushed = self._flush_imu()  # drain the last buffered IMU samples
            if flushed:
                logger.info("ZED IMU: flushed final %d samples on stop", flushed)
        except Exception:
            logger.exception("ZED IMU final flush failed")
        # Detach the backend so a straggling _imu_flush_loop iteration can't
        # write after super().stop() closes the db.
        self._imu_backend = None
        super().stop()
