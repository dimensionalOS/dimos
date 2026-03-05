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

from concurrent.futures import ThreadPoolExecutor
import threading
from typing import TYPE_CHECKING, Any

from dimos.core.docker_runner import DockerModule, is_docker_module
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.resource import Resource
from dimos.core.worker_manager import WorkerManager
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.module import Module, ModuleT
    from dimos.core.resource_monitor.monitor import StatsMonitor
    from dimos.core.rpc_client import ModuleProxy

logger = setup_logger()


class ModuleCoordinator(Resource):  # type: ignore[misc]
    _client: WorkerManager | None = None
    _global_config: GlobalConfig
    _n: int | None = None
    _memory_limit: str = "auto"
    _deployed_modules: dict[type[Module], ModuleProxy]
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

        self._client.close_all()  # type: ignore[union-attr]

    def deploy(self, module_class: type[ModuleT], *args, **kwargs) -> ModuleProxy:  # type: ignore[no-untyped-def]
        if not self._client:
            raise ValueError("Trying to dimos.deploy before the client has started")
        
        deployed_module : ModuleProxy
        if is_docker_module(module_class):
            deployed_module = DockerModule(module_class, *args, **kwargs)
        else:
            deployed_module = self._client.deploy(module_class, *args, **kwargs)
        self._deployed_modules[module_class] = deployed_module
        return deployed_module

    def deploy_parallel(
        self, module_specs: list[tuple[type[ModuleT], tuple[Any, ...], dict[str, Any]]]
    ) -> list[ModuleProxy]:
        if not self._client:
            raise ValueError("Not started")

        # Separate docker modules from regular modules
        docker_specs: list[tuple[type[ModuleT], tuple[Any, ...], dict[str, Any]]] = []
        worker_specs: list[tuple[type[ModuleT], tuple[Any, ...], dict[str, Any]]] = []
        spec_indices: list[tuple[str, int]] = []  # ("docker"|"worker", index_in_sublist)

        for module_class, args, kwargs in module_specs:
            if is_docker_module(module_class):
                docker_specs.append(spec)
            else:
                worker_specs.append(spec)

        worker_results = self._client.deploy_parallel(worker_specs) if worker_specs else []
        docker_results = []
        if docker_specs:
            with ThreadPoolExecutor(max_workers=len(docker_specs)) as executor:
                docker_results = list(
                    executor.map(
                        lambda spec: DockerModule(spec[0], *spec[1], **spec[2]), docker_specs
                    )
                )
        specs = worker_specs+docker_specs
        results = worker_results+docker_results
        
        for (module_class, _, _), module in zip(specs, results, strict=True):
            self._deployed_modules[module_class] = module
        return results  # type: ignore[return-value]

    def start_all_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        with ThreadPoolExecutor(max_workers=len(modules)) as executor:
            list(executor.map(lambda m: m.start(), modules))

        for module in modules:
            if hasattr(module, "on_system_modules"):
                module.on_system_modules(modules)

    def get_instance(self, module: type[ModuleT]) -> ModuleProxy:
        return self._deployed_modules.get(module)  # type: ignore[return-value, no-any-return]

    def loop(self) -> None:
        stop = threading.Event()
        try:
            stop.wait()
        except KeyboardInterrupt:
            return
        finally:
            self.stop()
