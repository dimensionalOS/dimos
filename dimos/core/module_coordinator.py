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
from typing import TYPE_CHECKING, Any, TypeAlias

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.resource import Resource
from dimos.core.worker_manager import WorkerManager
from dimos.core.worker_manager_docker import WorkerManagerDocker
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxy, ModuleProxyProtocol

logger = setup_logger()

DeploymentManager: TypeAlias = WorkerManagerDocker | WorkerManager


class ModuleCoordinator(Resource):  # type: ignore[misc]
    _managers: dict[str, DeploymentManager]
    _global_config: GlobalConfig
    _deployed_modules: dict[type[ModuleBase], ModuleProxyProtocol]

    def __init__(
        self,
        g: GlobalConfig = global_config,
    ) -> None:
        self._global_config = g
        manager_types: list[type[DeploymentManager]] = [WorkerManagerDocker, WorkerManager]
        self._managers: dict[str, DeploymentManager] = {
            cls.deployment_identifier: cls(g=g) for cls in manager_types
        }
        self._deployed_modules = {}

    def start(self) -> None:
        from dimos.core.o3dpickle import register_picklers

        register_picklers()
        for m in self._managers.values():
            m.start()

    def health_check(self) -> bool:
        return all(m.health_check() for m in self._managers.values())

    @property
    def n_modules(self) -> int:
        return len(self._deployed_modules)

    def suppress_console(self) -> None:
        for m in self._managers.values():
            m.suppress_console()

    def stop(self) -> None:
        for module_class, module in reversed(self._deployed_modules.items()):
            logger.info("Stopping module...", module=module_class.__name__)
            try:
                module.stop()
            except Exception:
                logger.error("Error stopping module", module=module_class.__name__, exc_info=True)
            logger.info("Module stopped.", module=module_class.__name__)

        def _stop_manager(m: DeploymentManager) -> None:
            try:
                m.stop()
            except Exception:
                logger.error("Error stopping manager", manager=type(m).__name__, exc_info=True)

        safe_thread_map(tuple(self._managers.values()), _stop_manager)

    def deploy(
        self,
        module_class: type[ModuleBase[Any]],
        global_config: GlobalConfig = global_config,
        **kwargs: Any,
    ) -> ModuleProxy:
        if not self._managers:
            raise ValueError("Trying to dimos.deploy before the client has started")

        deployed_module = self._managers[module_class.deployment].deploy(
            module_class, global_config, kwargs
        )
        self._deployed_modules[module_class] = deployed_module  # type: ignore[assignment]
        return deployed_module  # type: ignore[return-value]

    def deploy_parallel(self, module_specs: list[ModuleSpec]) -> list[ModuleProxy]:
        if not self._managers:
            raise ValueError("Not started")

        # Group specs by deployment type, tracking original indices for reassembly
        indices_by_deployment: dict[str, list[int]] = {}
        specs_by_deployment: dict[str, list[ModuleSpec]] = {}
        for index, spec in enumerate(module_specs):
            # spec = (module_class, global_config, kwargs)
            dep = spec[0].deployment
            indices_by_deployment.setdefault(dep, []).append(index)
            specs_by_deployment.setdefault(dep, []).append(spec)

        results: list[Any] = [None] * len(module_specs)

        def _deploy_group(dep: str) -> None:
            deployed = self._managers[dep].deploy_parallel(specs_by_deployment[dep])
            for index, module in zip(indices_by_deployment[dep], deployed, strict=True):
                results[index] = module

        try:
            safe_thread_map(list(specs_by_deployment.keys()), _deploy_group)
        except:
            self.stop()
            raise

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

        try:
            safe_thread_map(modules, lambda m: m.build())
        except:
            self.stop()
            raise

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
