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

from __future__ import annotations

from dataclasses import asdict
import os
import threading
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.core.resource import Resource
from dimos.core.resource_monitor.stats import (
    WorkerStats,
    collect_process_stats,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Sequence

    from dimos.core.resource_monitor.logger import ResourceLogger

logger = setup_logger()


@runtime_checkable
class WorkerInfo(Protocol):
    @property
    def pid(self) -> int | None: ...

    @property
    def worker_id(self) -> int: ...

    @property
    def module_names(self) -> list[str]: ...


@runtime_checkable
class WorkerSource(Protocol):
    @property
    def workers(self) -> Sequence[WorkerInfo]: ...


class StatsMonitor(Resource):
    """Self-contained resource monitor that runs in a daemon thread.

    Collects stats for the coordinator process and all workers, then
    forwards them to a ``ResourceLogger``.
    """

    def __init__(
        self,
        worker_source: WorkerSource,
        resource_logger: ResourceLogger | None = None,
        interval: float = 1.0,
        coordinator_pid: int | None = None,
    ) -> None:
        self._worker_source = worker_source
        self._interval = interval
        self._coordinator_pid = coordinator_pid or os.getpid()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        if resource_logger is not None:
            self._logger = resource_logger
        else:
            from dimos.core.resource_monitor.logger import LCMResourceLogger

            self._logger = LCMResourceLogger()

    def start(self) -> None:
        """Start the monitoring daemon thread."""
        # Prime cpu_percent so the first real reading isn't 0.0.
        collect_process_stats(self._coordinator_pid)

        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the monitor thread to stop and wait for it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def _loop(self) -> None:
        while not self._stop.wait(self._interval):
            try:
                self._collect_and_log()
            except Exception:
                logger.error("StatsMonitor collection failed", exc_info=True)

    def _collect_and_log(self) -> None:
        coordinator = collect_process_stats(self._coordinator_pid)

        worker_stats: list[WorkerStats] = []
        for w in self._worker_source.workers:
            pid = w.pid
            if pid is not None:
                ps = collect_process_stats(pid)
                worker_stats.append(
                    WorkerStats(**asdict(ps), worker_id=w.worker_id, modules=w.module_names)
                )
            else:
                worker_stats.append(
                    WorkerStats(
                        pid=0,
                        alive=False,
                        worker_id=w.worker_id,
                        modules=w.module_names,
                    )
                )

        self._logger.log_stats(coordinator, worker_stats)
