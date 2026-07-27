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

"""Go2 memory2 recorder with continuous hosted snapshot publication."""

from __future__ import annotations

from pathlib import Path
from threading import Event, Thread
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorderConfig
from dimos.hosted_data.integrations.memory2_sync import ContinuousMemory2Publisher
from dimos.robot.unitree.go2.go2_mid360_recorder import Go2Mid360Recorder
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Go2HostedRecorderConfig(PointlioRecorderConfig):
    """Local recording and remote publication settings."""

    hosted_server_url: str = "http://127.0.0.1:8765"
    hosted_owner: str = "local"
    hosted_repository: str = "go2"
    hosted_token: str | None = None
    hosted_dataset: str | None = None
    hosted_interval_seconds: float = Field(default=5.0, ge=0.1)


class Go2HostedRecorder(Go2Mid360Recorder):
    """Record Go2 streams locally and continuously publish replayable snapshots."""

    config: Go2HostedRecorderConfig
    _hosted_stop: Event | None = None
    _hosted_thread: Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.g.replay:
            return
        publisher = ContinuousMemory2Publisher(
            path=Path(self.config.db_path),
            server_url=self.config.hosted_server_url,
            owner=self.config.hosted_owner,
            repository=self.config.hosted_repository,
            token=self.config.hosted_token,
            dataset=self.config.hosted_dataset,
            interval_seconds=self.config.hosted_interval_seconds,
        )
        self._hosted_stop = Event()
        self._hosted_thread = Thread(
            target=publisher.run,
            args=(self._hosted_stop,),
            name="go2-hosted-memory2-sync",
            daemon=True,
        )
        self._hosted_thread.start()
        logger.info(
            "Publishing live Go2 memory2 snapshots to %s/%s/%s",
            self.config.hosted_server_url,
            self.config.hosted_owner,
            self.config.hosted_repository,
        )

    @rpc
    def stop(self) -> None:
        if self._hosted_stop is not None:
            self._hosted_stop.set()
        if self._hosted_thread is not None:
            self._hosted_thread.join(timeout=self.config.hosted_interval_seconds + 5.0)
        self._hosted_stop = None
        self._hosted_thread = None
        super().stop()
