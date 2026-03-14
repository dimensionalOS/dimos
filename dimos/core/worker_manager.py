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
from contextlib import suppress
from typing import Any

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.rpc_client import RPCClient
from dimos.core.worker import Worker
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map

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

        module_specs = list(module_specs)
        if len(module_specs) == 0:
            return []

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

        def _on_errors(
            _outcomes: list[Any], successes: list[RPCClient], errors: list[Exception]
        ) -> None:
            for rpc_client in successes:
                with suppress(Exception):
                    rpc_client.stop_rpc_client()
            raise ExceptionGroup("worker deploy_parallel failed", errors)  # type: ignore[name-defined]

        return safe_thread_map(
            assignments,
            lambda item: RPCClient(item[0].deploy_module(item[1], item[2], item[3]), item[1]),
            _on_errors,
        )

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
