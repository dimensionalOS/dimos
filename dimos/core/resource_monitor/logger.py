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
from typing import TYPE_CHECKING, Any, Protocol

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.resource_monitor.stats import ProcessStats, WorkerStats

logger = setup_logger()


class ResourceLogger(Protocol):
    def log_stats(self, coordinator: ProcessStats, workers: list[WorkerStats]) -> None: ...


class StructlogResourceLogger:
    """Default implementation — logs resource stats via structlog info."""

    def log_stats(self, coordinator: ProcessStats, workers: list[WorkerStats]) -> None:
        logger.info(
            "coordinator",
            pid=coordinator.pid,
            cpu_pct=coordinator.cpu_percent,
            pss_mb=round(coordinator.pss / 1048576, 1),
            threads=coordinator.num_threads,
        )
        for w in workers:
            logger.info(
                "worker",
                worker_id=w.worker_id,
                pid=w.pid,
                alive=w.alive,
                cpu_pct=w.cpu_percent,
                pss_mb=round(w.pss / 1048576, 1),
                threads=w.num_threads,
                children=w.num_children,
                fds=w.num_fds,
                io_r_mb=round(w.io_read_bytes / 1048576, 1),
                io_w_mb=round(w.io_write_bytes / 1048576, 1),
                modules=w.modules,
            )


class LCMResourceLogger:
    """Publishes resource stats as dicts over a pickle LCM channel."""

    def __init__(self, topic: str = "/dimos/resource_stats") -> None:
        from dimos.core.transport import pLCMTransport

        self._transport: pLCMTransport[dict[str, Any]] = pLCMTransport(topic)

    def log_stats(self, coordinator: ProcessStats, workers: list[WorkerStats]) -> None:
        self._transport.broadcast(
            None,
            {
                "coordinator": asdict(coordinator),
                "workers": [asdict(w) for w in workers],
            },
        )
