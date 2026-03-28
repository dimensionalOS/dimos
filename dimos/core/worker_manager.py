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

from collections.abc import Iterable
from typing import TYPE_CHECKING, Any

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.rpc_client import RPCClient
from dimos.core.worker import Worker
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    from dimos.core.resource_monitor.monitor import StatsMonitor

logger = setup_logger()


class WorkerManager:
    deployment_identifier: str = "python"

    def __init__(self, g: GlobalConfig) -> None:
        self._cfg = g
        self._n_workers = g.n_workers
        self._workers: list[Worker] = []
        self._closed = False
        self._started = False
        self._stats_monitor: StatsMonitor | None = None

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        for _ in range(self._n_workers):
            worker = Worker()
            worker.start_process()
            self._workers.append(worker)
        logger.info("Worker pool started.", n_workers=self._n_workers)

        if self._cfg.dtop:
            from dimos.core.resource_monitor.monitor import StatsMonitor

            self._stats_monitor = StatsMonitor(self)
            self._stats_monitor.start()

    def _select_worker(self) -> Worker:
        return min(self._workers, key=lambda w: w.module_count)

    def deploy(
        self, module_class: type[ModuleBase], global_config: GlobalConfig, kwargs: dict[str, Any]
    ) -> RPCClient:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        if not self._started:
            self.start()

        worker = self._select_worker()
        actor = worker.deploy_module(module_class, global_config, kwargs=kwargs)
        return RPCClient(actor, module_class)

    def deploy_parallel(self, module_specs: Iterable[ModuleSpec]) -> list[RPCClient]:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        module_specs = list(module_specs)
        if len(module_specs) == 0:
            return []

        if not self._started:
            self.start()

        # Pre-assign workers sequentially (so least-loaded accounting is
        # correct), then deploy concurrently via threads. The per-worker lock
        # serializes deploys that land on the same worker process.
        assignments: list[tuple[Worker, type[ModuleBase], GlobalConfig, dict[str, Any]]] = []
        for module_class, global_config, kwargs in module_specs:
            worker = self._select_worker()
            worker.reserve_slot()
            assignments.append((worker, module_class, global_config, kwargs))

        try:
            # item: (worker, module_class, global_config, kwargs)
            return safe_thread_map(
                assignments,
                lambda item: RPCClient(item[0].deploy_module(item[1], item[2], item[3]), item[1]),
            )
        except:
            self.stop()
            raise

    def health_check(self) -> bool:
        if len(self._workers) == 0:
            logger.error("health_check: no workers found")
            return False
        for w in self._workers:
            if w.pid is None:
                logger.error("health_check: worker died", worker_id=w.worker_id)
                return False
        return True

    def suppress_console(self) -> None:
        for worker in self._workers:
            worker.suppress_console()

    @property
    def workers(self) -> list[Worker]:
        return list(self._workers)

    def stop(self) -> None:
        if self._closed:
            return
        self._closed = True

        if self._stats_monitor is not None:
            self._stats_monitor.stop()
            self._stats_monitor = None

        logger.info("Shutting down all workers...")

        for worker in reversed(self._workers):
            try:
                worker.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down worker: {e}", exc_info=True)

        self._workers.clear()

        logger.info("All workers shut down")
