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

"""Replay a raw PointCloud2+Imu recording (scripts/pcap_to_raw_db.py output)
onto the transport, timed like the live Mid-360 driver would publish it.

The message-based counterpart of VirtualMid360: no virtual NIC, no sudo, no
SDK — the sensor is replaced at the message boundary instead of on the wire.
Output ports are named for direct autoconnect with PointLioRust
(``raw_lidar``/``imu``).
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RawSensorReplayConfig(ModuleConfig):
    # Dataset name or path of a db with `lidar` + `imu` streams.
    db: str = "data/mid360_shake_stairs/mid360_shake_stairs_raw.db"
    speed: float = 1.0
    loop: bool = False


class RawSensorReplay(Module):
    """Publishes a recorded raw sensor db as raw_lidar + imu topics."""

    dedicated_worker = True

    config: RawSensorReplayConfig

    raw_lidar: Out[PointCloud2]
    imu: Out[Imu]

    _store: SqliteStore | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._store = SqliteStore(path=str(resolve_db_path(self.config.db)), must_exist=True)
        self._store.start()
        replay = self._store.replay(speed=self.config.speed, loop=self.config.loop)

        available = replay.list_streams()
        for name in ("lidar", "imu"):
            if name not in available:
                raise RuntimeError(
                    f"RawSensorReplay: stream {name!r} not in {self.config.db!r} "
                    f"(available: {available}); generate the db with "
                    "dimos.hardware.sensors.lidar.livox.scripts.pcap_to_raw_db"
                )

        logger.info(
            "Raw sensor replay started",
            db=self.config.db,
            lidar_frames=replay.stream("lidar").count(),
            imu_msgs=replay.stream("imu").count(),
            speed=self.config.speed,
        )
        # One shared Replay anchor: both streams advance on the same clock.
        self.register_disposable(
            replay.stream("lidar").observable().subscribe(self.raw_lidar.publish)
        )
        self.register_disposable(replay.stream("imu").observable().subscribe(self.imu.publish))

    @rpc
    def stop(self) -> None:
        if self._store is not None:
            self._store.stop()
            self._store = None
        super().stop()
