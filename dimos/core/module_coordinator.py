# Copyright 2025-2026 Dimensional Inc.
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

import threading
from typing import TYPE_CHECKING, Any

from dimos.core.docker_worker_manager import DockerWorkerManager
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.resource import Resource
from dimos.core.worker_manager import WorkerManager
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map

if TYPE_CHECKING:
    from dimos.core.resource_monitor.monitor import StatsMonitor
    from dimos.core.rpc_client import ModuleProxy, ModuleProxyProtocol
    from dimos.core.worker import Worker

logger = setup_logger()


class ModuleCoordinator(Resource):  # type: ignore[misc]
    _client: WorkerManager | None = None
    _global_config: GlobalConfig
    _n: int | None = None
    _memory_limit: str = "auto"
    _deployed_modules: dict[type[ModuleBase], ModuleProxyProtocol]
    _stats_monitor: StatsMonitor | None = None

    def __init__(
        self,
        n: int | None = None,
        cfg: GlobalConfig = global_config,
    ) -> None:
        self._n = n if n is not None else cfg.n_workers
        self._memory_limit = cfg.memory_limit
        self._global_config = cfg
        self._deployed_modules = {}

    @property
    def workers(self) -> list[Worker]:
        """Active worker processes."""
        if self._client is None:
            return []
        return self._client.workers

    @property
    def n_workers(self) -> int:
        """Number of active workers."""
        return len(self.workers)

    def health_check(self) -> bool:
        """Verify all workers are alive after build.

        Since ``blueprint.build()`` is synchronous, every module should be
        started by the time this runs.  We just confirm no worker has died.
        """
        if self.n_workers == 0:
            logger.error("health_check: no workers found")
            return False

        for w in self.workers:
            if w.pid is None:
                logger.error("health_check: worker died", worker_id=w.worker_id)
                return False

        return True

    @property
    def n_modules(self) -> int:
        """Number of deployed modules."""
        return len(self._deployed_modules)

    def suppress_console(self) -> None:
        """Silence console output in all worker processes."""
        if self._client is not None:
            self._client.suppress_console()

    def start(self) -> None:
        n = self._n if self._n is not None else 2
        self._client = WorkerManager(n_workers=n)
        self._client.start()

        if self._global_config.dtop:
            from dimos.core.resource_monitor.monitor import StatsMonitor

            self._stats_monitor = StatsMonitor(self._client)
            self._stats_monitor.start()

    def stop(self) -> None:
        if self._stats_monitor is not None:
            self._stats_monitor.stop()
            self._stats_monitor = None

        for module_class, module in reversed(self._deployed_modules.items()):
            logger.info("Stopping module...", module=module_class.__name__)
            try:
                module.stop()
            except Exception:
                logger.error("Error stopping module", module=module_class.__name__, exc_info=True)
            logger.info("Module stopped.", module=module_class.__name__)

        if self._client is not None:
            self._client.close_all()

    def deploy(
        self,
        module_class: type[ModuleBase[Any]],
        global_config: GlobalConfig = global_config,
        **kwargs: Any,
    ) -> ModuleProxy:
        # Inline to avoid circular import: module_coordinator → docker_runner → module → blueprints → module_coordinator
        from dimos.core.docker_runner import DockerModule, is_docker_module

        if not self._client:
            raise ValueError("Trying to dimos.deploy before the client has started")

        deployed_module: ModuleProxyProtocol
        if is_docker_module(module_class):
            deployed_module = DockerModule(module_class, global_config=global_config, **kwargs)  # type: ignore[arg-type]
        else:
            deployed_module = self._client.deploy(module_class, global_config, kwargs)
        self._deployed_modules[module_class] = deployed_module  # type: ignore[assignment]
        return deployed_module  # type: ignore[return-value]

    def deploy_parallel(self, module_specs: list[ModuleSpec]) -> list[ModuleProxy]:
        # Inline to avoid circular import: module_coordinator → docker_runner → module → blueprints → module_coordinator
        from dimos.core.docker_runner import is_docker_module

        if not self._client:
            raise ValueError("Not started")

        # Split by type, tracking original indices for reassembly
        docker_indices: list[int] = []
        worker_indices: list[int] = []
        docker_specs: list[ModuleSpec] = []
        worker_specs: list[ModuleSpec] = []
        for i, spec in enumerate(module_specs):
            if is_docker_module(spec[0]):
                docker_indices.append(i)
                docker_specs.append(spec)
            else:
                worker_indices.append(i)
                worker_specs.append(spec)

        # Deploy worker and docker modules in parallel.
        results: list[Any] = [None] * len(module_specs)

        def _deploy_workers() -> None:
            if not worker_specs:
                return
            assert self._client is not None
            for index, module in zip(
                worker_indices, self._client.deploy_parallel(worker_specs), strict=False
            ):
                results[index] = module

        def _deploy_docker() -> None:
            if not docker_specs:
                return
            for index, module in zip(
                docker_indices, DockerWorkerManager.deploy_parallel(docker_specs), strict=False
            ):
                results[index] = module

        def _register() -> None:
            for (module_class, _, _), module in zip(module_specs, results, strict=False):
                if module is not None:
                    self._deployed_modules[module_class] = module

        def _on_errors(
            _outcomes: list[Any], _successes: list[Any], errors: list[Exception]
        ) -> None:
            _register()
            raise ExceptionGroup("deploy_parallel failed", errors)

        safe_thread_map([_deploy_workers, _deploy_docker], lambda fn: fn(), _on_errors)
        _register()
        return results

    def start_all_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        if not modules:
            raise ValueError("No modules deployed. Call deploy() before start_all_modules().")

        def _on_start_errors(
            _outcomes: list[Any], _successes: list[Any], errors: list[Exception]
        ) -> None:
            raise ExceptionGroup("start_all_modules failed", errors)  # type: ignore[name-defined]

        safe_thread_map(modules, lambda m: m.start(), _on_start_errors)

        for module in modules:
            if hasattr(module, "on_system_modules"):
                module.on_system_modules(modules)

    def get_instance(self, module: type[ModuleBase]) -> ModuleProxy:
        return self._deployed_modules.get(module)  # type: ignore[return-value, no-any-return]

    def loop(self) -> None:
        stop = threading.Event()
        try:
            stop.wait()
        except KeyboardInterrupt:
            return
        finally:
            self.stop()
