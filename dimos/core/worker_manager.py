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
from concurrent.futures import ThreadPoolExecutor
from typing import Any

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.rpc_client import RPCClient
from dimos.core.worker import Worker
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WorkerManager:
    def __init__(self, n_workers: int = 2) -> None:
        self._n_workers = n_workers
        self._workers: list[Worker] = []
        self._closed = False
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        for _ in range(self._n_workers):
            worker = Worker()
            worker.start_process()
            self._workers.append(worker)
        logger.info("Worker pool started.", n_workers=self._n_workers)

    def add_worker(self) -> Worker:
        """Spawn one additional worker process and add it to the pool."""
        worker = Worker()
        worker.start_process()
        self._workers.append(worker)
        self._n_workers = len(self._workers)
        return worker

    def ensure_capacity(self, n: int) -> None:
        """Grow the pool to at least *n* workers if needed."""
        while len(self._workers) < n:
            self.add_worker()
        if len(self._workers) > self._n_workers:
            logger.info("Worker pool grew.", n_workers=len(self._workers))

    def _select_worker(self) -> Worker:
        return min(self._workers, key=lambda w: w.module_count)

    def deploy(
        self, module_class: type[ModuleBase], global_config: GlobalConfig, kwargs: dict[str, Any]
    ) -> RPCClient:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        # Auto-start for backward compatibility
        if not self._started:
            self.start()

        worker = self._select_worker()
        actor = worker.deploy_module(module_class, global_config, kwargs=kwargs)
        return RPCClient(actor, module_class)

    def deploy_parallel(self, module_specs: Iterable[ModuleSpec]) -> list[RPCClient]:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        # Auto-start for backward compatibility
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

        def _deploy(
            item: tuple[Worker, type[ModuleBase], GlobalConfig, dict[str, Any]],
        ) -> RPCClient:
            worker, module_class, global_config, kwargs = item
            actor = worker.deploy_module(module_class, global_config=global_config, kwargs=kwargs)
            return RPCClient(actor, module_class)

        with ThreadPoolExecutor(max_workers=len(assignments)) as pool:
            results = list(pool.map(_deploy, assignments))

        return results

    def suppress_console(self) -> None:
        """Tell all workers to redirect stdout/stderr to /dev/null."""
        for worker in self._workers:
            worker.suppress_console()

    @property
    def workers(self) -> list[Worker]:
        return list(self._workers)

    def close_all(self) -> None:
        if self._closed:
            return
        self._closed = True

        logger.info("Shutting down all workers...")

        for worker in reversed(self._workers):
            try:
                worker.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down worker: {e}", exc_info=True)

        self._workers.clear()

        logger.info("All workers shut down")
