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

from contextlib import suppress
import threading
from typing import TYPE_CHECKING, Any

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.resource import Resource
from dimos.core.worker_manager import WorkerManager
from dimos.core.worker_manager_docker import WorkerManagerDocker
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxy, ModuleProxyProtocol

logger = setup_logger()


class ModuleCoordinator(Resource):  # type: ignore[misc]
    _managers: list[WorkerManagerDocker | WorkerManager]
    _global_config: GlobalConfig
    _deployed_modules: dict[type[ModuleBase], ModuleProxyProtocol]

    def __init__(
        self,
        g: GlobalConfig = global_config,
    ) -> None:
        self._global_config = g
        self._managers = []
        self._deployed_modules = {}

    def start(self) -> None:
        self._managers = [
            WorkerManagerDocker(g=self._global_config),
            WorkerManager(g=self._global_config),
        ]
        for m in self._managers:
            m.start()

    def _find_manager(
        self, module_class: type[ModuleBase[Any]]
    ) -> WorkerManagerDocker | WorkerManager:
        for m in self._managers:
            if m.should_manage(module_class):
                return m
        raise ValueError(f"No manager found for {module_class.__name__}")

    def health_check(self) -> bool:
        return all(m.health_check() for m in self._managers)

    @property
    def n_modules(self) -> int:
        return len(self._deployed_modules)

    def suppress_console(self) -> None:
        for m in self._managers:
            m.suppress_console()

    def stop(self) -> None:
        for module_class, module in reversed(self._deployed_modules.items()):
            logger.info("Stopping module...", module=module_class.__name__)
            with suppress(Exception):
                module.stop()
            logger.info("Module stopped.", module=module_class.__name__)

        def _stop_manager(m: WorkerManagerDocker | WorkerManager) -> None:
            try:
                m.stop()
            except Exception:
                logger.error("Error stopping manager", manager=type(m).__name__, exc_info=True)

        safe_thread_map(self._managers, _stop_manager)

    def deploy(
        self,
        module_class: type[ModuleBase[Any]],
        global_config: GlobalConfig = global_config,
        **kwargs: Any,
    ) -> ModuleProxy:
        if not self._managers:
            raise ValueError("Trying to dimos.deploy before the client has started")

        manager = self._find_manager(module_class)
        deployed_module = manager.deploy(module_class, global_config, kwargs)
        self._deployed_modules[module_class] = deployed_module  # type: ignore[assignment]
        return deployed_module  # type: ignore[return-value]

    def deploy_parallel(self, module_specs: list[ModuleSpec]) -> list[ModuleProxy]:
        if not self._managers:
            raise ValueError("Not started")

        # Group specs by manager, tracking original indices for reassembly
        groups: dict[int, WorkerManagerDocker | WorkerManager] = {}
        indices_by_manager: dict[int, list[int]] = {}
        specs_by_manager: dict[int, list[ModuleSpec]] = {}
        for index, spec in enumerate(module_specs):
            manager = self._find_manager(spec[0])
            mid = id(manager)
            groups.setdefault(mid, manager)
            indices_by_manager.setdefault(mid, []).append(index)
            specs_by_manager.setdefault(mid, []).append(spec)

        results: list[Any] = [None] * len(module_specs)

        def _deploy_group(mid: int) -> None:
            deployed = groups[mid].deploy_parallel(specs_by_manager[mid])
            for index, module in zip(indices_by_manager[mid], deployed, strict=True):
                results[index] = module

        safe_thread_map(list(groups.keys()), _deploy_group)
        self._deployed_modules.update(
            {
                cls: mod
                for (cls, _, _), mod in zip(module_specs, results, strict=True)
                if mod is not None
            }
        )
        return results

    def build_all_modules(self) -> None:
        """Call build() on all deployed modules in parallel.

        build() handles heavy one-time work (docker builds, LFS downloads, etc.)
        with a very long timeout. Must be called after deploy and stream wiring
        but before start_all_modules().
        """
        modules = list(self._deployed_modules.values())
        if not modules:
            raise ValueError("No modules deployed. Call deploy() before build_all_modules().")

        def _on_build_errors(
            _outcomes: list[Any], successes: list[Any], errors: list[Exception]
        ) -> None:
            for mod in modules:
                with suppress(Exception):
                    mod.stop()
            raise ExceptionGroup("build_all_modules failed", errors)

        safe_thread_map(modules, lambda m: m.build(), _on_build_errors)

    def start_all_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        if not modules:
            raise ValueError("No modules deployed. Call deploy() before start_all_modules().")

        safe_thread_map(modules, lambda m: m.start())

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
