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

from concurrent.futures import ThreadPoolExecutor
import time
from typing import TYPE_CHECKING, Any

from dimos import core
from dimos.core import DimosCluster
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleT
from dimos.core.resource import Resource
from dimos.core.worker_manager import WorkerManager

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxy


class ModuleCoordinator(Resource):  # type: ignore[misc]
    _client: DimosCluster | WorkerManager | None = None
    _global_config: GlobalConfig
    _n: int | None = None
    _memory_limit: str = "auto"
    _deployed_modules: dict[type[Module], "ModuleProxy"] = {}

    def __init__(
        self,
        n: int | None = None,
        global_config: GlobalConfig | None = None,
    ) -> None:
        cfg = global_config or GlobalConfig()
        self._n = n if n is not None else cfg.n_dask_workers
        self._memory_limit = cfg.memory_limit
        self._global_config = cfg

    def start(self) -> None:
        if self._global_config.dask:
            self._client = core.start(self._n, self._memory_limit)
        else:
            self._client = WorkerManager()

    def stop(self) -> None:
        for module in reversed(self._deployed_modules.values()):
            module.stop()

        self._client.close_all()  # type: ignore[union-attr]

    def deploy(self, module_class: type[ModuleT], *args, **kwargs) -> "ModuleProxy":  # type: ignore[no-untyped-def]
        if not self._client:
            raise ValueError("Trying to dimos.deploy before dask client has started")

        module: ModuleProxy = self._client.deploy(module_class, *args, **kwargs)  # type: ignore[union-attr, attr-defined, assignment]
        self._deployed_modules[module_class] = module
        return module

    def deploy_parallel(
        self, module_specs: list[tuple[type[ModuleT], tuple[Any, ...], dict[str, Any]]]
    ) -> list["ModuleProxy"]:
        if not self._client:
            raise ValueError("Not started")

        if isinstance(self._client, WorkerManager):
            modules = self._client.deploy_parallel(module_specs)
            for (module_class, _, _), module in zip(module_specs, modules, strict=True):
                self._deployed_modules[module_class] = module  # type: ignore[assignment]
            return modules  # type: ignore[return-value]
        else:
            return [
                self.deploy(module_class, *args, **kwargs)
                for module_class, args, kwargs in module_specs
            ]

    def start_all_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        if isinstance(self._client, WorkerManager):
            with ThreadPoolExecutor(max_workers=len(modules)) as executor:
                list(executor.map(lambda m: m.start(), modules))
        else:
            for module in modules:
                module.start()

    def get_instance(self, module: type[ModuleT]) -> "ModuleProxy":
        return self._deployed_modules.get(module)  # type: ignore[return-value, no-any-return]

    def loop(self) -> None:
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            return
        finally:
            self.stop()
