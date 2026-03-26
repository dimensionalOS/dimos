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
from typing import TYPE_CHECKING, Any

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.rpc_client import RPCClient
from dimos.core.worker import Worker
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map

if TYPE_CHECKING:
    from dimos.core.resource_monitor.monitor import StatsMonitor

logger = setup_logger()


_MIN_WORKERS = 2


class WorkerManager:
    def __init__(self, g: GlobalConfig) -> None:
        self._cfg = g
        self._max_workers = g.n_workers
        self._worker_to_module_ratio = g.worker_to_module_ratio
        self._workers: list[Worker] = []
        self._n_modules = 0
        self._closed = False
        self._started = False
        self._stats_monitor: StatsMonitor | None = None

    def _desired_workers(self, n_modules: int) -> int:
        """Target worker count: ratio * modules, clamped to [_MIN_WORKERS, max_workers]."""
        from_ratio = int(n_modules * self._worker_to_module_ratio + 0.5)
        return max(_MIN_WORKERS, min(from_ratio, self._max_workers))

    def _ensure_workers(self, n_modules: int) -> None:
        """Grow the worker pool to match the desired count for *n_modules*."""
        target = self._desired_workers(n_modules)
        while len(self._workers) < target:
            worker = Worker()
            worker.start_process()
            self._workers.append(worker)

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._ensure_workers(self._n_modules)
        logger.info("Worker pool started.", n_workers=len(self._workers))

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

        self._n_modules += 1
        self._ensure_workers(self._n_modules)
        try:
            worker = self._select_worker()
            actor = worker.deploy_module(module_class, global_config, kwargs=kwargs)
            return RPCClient(actor, module_class)
        except Exception:
            self._n_modules -= 1
            raise

    def deploy_parallel(self, module_specs: Iterable[ModuleSpec]) -> list[RPCClient]:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        module_specs = list(module_specs)
        if len(module_specs) == 0:
            return []

        if not self._started:
            self.start()

        self._n_modules += len(module_specs)
        self._ensure_workers(self._n_modules)

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
            self._n_modules -= len(errors)
            for rpc_client in successes:
                with suppress(Exception):
                    rpc_client.stop_rpc_client()
            raise ExceptionGroup("worker deploy_parallel failed", errors)

        return safe_thread_map(
            assignments,
            # item = [worker, module_class, global_config, kwargs]
            lambda item: RPCClient(item[0].deploy_module(item[1], item[2], item[3]), item[1]),
            _on_errors,
        )

    def should_manage(self, module_class: type) -> bool:
        """Catch-all — accepts any module not claimed by another manager."""
        return True

    def health_check(self) -> bool:
        """Verify all worker processes are alive."""
        if len(self._workers) == 0:
            logger.error("health_check: no workers found")
            return False
        for w in self._workers:
            if w.pid is None:
                logger.error("health_check: worker died", worker_id=w.worker_id)
                return False
        return True

    def suppress_console(self) -> None:
        """Tell all workers to redirect stdout/stderr to /dev/null."""
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
