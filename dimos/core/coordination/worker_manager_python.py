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

from collections.abc import Iterable, Mapping
from dataclasses import asdict, is_dataclass
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.python_worker import PythonWorker
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.rpc_client import ModuleProxyProtocol, RPCClient
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map

if TYPE_CHECKING:
    from dimos.core.resource_monitor.monitor import StatsMonitor

logger = setup_logger()


def _normalize_config_value(value: Any) -> Any:
    if isinstance(value, BaseConfig):
        return value.model_dump()
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    return value


def _merge_config_args(defaults: Mapping[str, Any], overrides: Mapping[str, Any]) -> dict[str, Any]:
    """Recursively merge CLI/module overrides into blueprint defaults."""
    merged = {key: _normalize_config_value(value) for key, value in defaults.items()}
    for key, override_value in overrides.items():
        normalized_override = _normalize_config_value(override_value)
        default_value = merged.get(key)
        if isinstance(default_value, Mapping) and isinstance(normalized_override, Mapping):
            merged[key] = _merge_config_args(default_value, normalized_override)
        else:
            merged[key] = normalized_override
    return merged


class WorkerManagerPython:
    deployment_identifier: str = "python"

    def __init__(self, g: GlobalConfig) -> None:
        self._cfg = g
        self._n_workers = g.n_workers
        self._workers: list[PythonWorker] = []
        self._closed = False
        self._started = False
        self._stats_monitor: StatsMonitor | None = None

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        for _ in range(self._n_workers):
            worker = PythonWorker()
            worker.start_process()
            self._workers.append(worker)
        logger.info("Worker pool started.", n_workers=self._n_workers)

        if self._cfg.dtop:
            from dimos.core.resource_monitor.monitor import StatsMonitor

            self._stats_monitor = StatsMonitor(self)
            self._stats_monitor.start()

    def add_workers(self, n: int) -> None:
        """Spawn *n* additional worker processes into the pool."""
        if self._closed:
            raise RuntimeError("WorkerManager is closed")
        if not self._started:
            raise RuntimeError("WorkerManager not started; call start() first")
        for _ in range(n):
            worker = PythonWorker()
            worker.start_process()
            self._workers.append(worker)
        self._n_workers += n
        logger.info("Added workers to pool.", added=n, total=self._n_workers)

    def deploy(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        if not self._started:
            self.start()

        self._ensure_capacity_for_dedicated([(module_class, global_config, kwargs)])
        worker = self._select_worker(dedicated=module_class.dedicated_worker)
        actor = worker.deploy_module(module_class, global_config, kwargs=kwargs)
        return RPCClient(actor, module_class)

    def deploy_fresh(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol:
        """Spawn a brand-new worker process and deploy *module_class* on it.

        Used by restart so the new module is imported by a Python process with
        a clean ``sys.modules`` — existing workers would reuse the old class
        object even after ``importlib.reload`` in the parent.
        """
        if self._closed:
            raise RuntimeError("WorkerManager is closed")
        if not self._started:
            self.start()

        worker = PythonWorker()
        worker.start_process()
        self._workers.append(worker)
        self._n_workers += 1
        if module_class.dedicated_worker:
            worker.dedicated = True
        actor = worker.deploy_module(module_class, global_config, kwargs=kwargs)
        return RPCClient(actor, module_class)

    def undeploy(self, proxy: ModuleProxyProtocol) -> None:
        """Undeploy a module and shut down its worker if it is now empty."""
        actor = getattr(proxy, "actor_instance", None)
        if actor is None:
            raise ValueError("Proxy has no actor_instance. Cannot undeploy.")

        module_id = actor._module_id
        target: PythonWorker | None = None
        for worker in self._workers:
            if module_id in worker._modules:
                target = worker
                break
        if target is None:
            raise ValueError(f"No worker holds module_id={module_id}")

        target.undeploy_module(module_id)

        if not target._modules:
            target.shutdown()
            self._workers.remove(target)
            self._n_workers = max(0, self._n_workers - 1)

    def deploy_parallel(
        self, specs: Iterable[ModuleSpec], blueprint_args: Mapping[str, Mapping[str, Any]]
    ) -> list[ModuleProxyProtocol]:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        specs = list(specs)
        if len(specs) == 0:
            return []

        if not self._started:
            self.start()

        self._ensure_capacity_for_dedicated(specs)

        # Pre-assign workers sequentially (so least-loaded accounting is
        # correct), then deploy concurrently via threads. The per-worker lock
        # serializes deploys that land on the same worker process.
        # Process dedicated specs first so they claim empty workers before
        # non-dedicated specs land on them; preserve input order in output.
        workers_by_index: dict[int, PythonWorker] = {}
        order = sorted(range(len(specs)), key=lambda i: not specs[i][0].dedicated_worker)
        for i in order:
            module_class, global_config, kwargs = specs[i]
            worker = self._select_worker(dedicated=module_class.dedicated_worker)
            worker.reserve_slot()
            specs[i] = (
                module_class,
                global_config,
                _merge_config_args(kwargs, blueprint_args.get(module_class.name, {})),
            )
            workers_by_index[i] = worker

        assignments = [(workers_by_index[i], specs[i]) for i in range(len(specs))]

        def _deploy(item: tuple[PythonWorker, ModuleSpec]) -> ModuleProxyProtocol:
            worker, (module_class, global_config, kwargs) = item
            return RPCClient(
                worker.deploy_module(module_class, global_config, kwargs), module_class
            )

        try:
            return safe_thread_map(assignments, _deploy)
        except ExceptionGroup:
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
    def workers(self) -> list[PythonWorker]:
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

    def _select_worker(self, dedicated: bool = False) -> PythonWorker:
        """Pick a worker for a new module and mark it dedicated if needed."""
        if dedicated:
            for w in self._workers:
                if not w.dedicated and w.module_count == 0:
                    w.dedicated = True
                    return w
            self.add_workers(1)
            w = self._workers[-1]
            w.dedicated = True
            return w

        candidates = [w for w in self._workers if not w.dedicated]
        if not candidates:
            self.add_workers(1)
            return self._workers[-1]
        return min(candidates, key=lambda w: w.module_count)

    def _ensure_capacity_for_dedicated(self, specs: Iterable[ModuleSpec]) -> None:
        """Grow the pool so non-dedicated workers >= dedicated workers.

        If the total number of dedicated modules (already deployed + about to be)
        exceeds half the worker pool, scale up to `2 * total_dedicated` workers.
        """
        new_dedicated = sum(1 for spec in specs if spec[0].dedicated_worker)
        already_dedicated = sum(1 for w in self._workers if w.dedicated)
        total_dedicated = already_dedicated + new_dedicated
        if total_dedicated == 0:
            return
        total_workers = len(self._workers)
        if total_dedicated * 2 > total_workers:
            n_to_add = total_dedicated * 2 - total_workers
            logger.warning(
                "Auto-scaling worker pool for dedicated modules.",
                dedicated=total_dedicated,
                before=total_workers,
                added=n_to_add,
            )
            self.add_workers(n_to_add)
