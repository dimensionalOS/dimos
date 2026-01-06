# Copyright 2025 Dimensional Inc.
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

import time
from typing import TypeVar

from dimos import core
from dimos.core import DimosCluster, Module
from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource

T = TypeVar("T", bound="Module")


class ModuleCoordinator(Resource):
    _client: DimosCluster | None = None
    _n: int | None = None
    _memory_limit: str = "auto"
    _deployed_modules: dict[type[Module], Module] = {}

    def __init__(
        self,
        n: int | None = None,
        global_config: GlobalConfig | None = None,
    ) -> None:
        cfg = global_config or GlobalConfig()
        self._n = n if n is not None else cfg.n_dask_workers
        self._memory_limit = cfg.memory_limit

    def start(self) -> None:
        self._client = core.start(self._n, self._memory_limit)

    def stop(self) -> None:
        for module in reversed(self._deployed_modules.values()):
            module.stop()

        self._client.close_all()  # type: ignore[union-attr]

    def deploy(self, module_class: type[T], *args, **kwargs) -> T:  # type: ignore[no-untyped-def]
        if not self._client:
            raise ValueError("Not started")

        module = self._client.deploy(module_class, *args, **kwargs)  # type: ignore[attr-defined]
        self._deployed_modules[module_class] = module
        return module  # type: ignore[no-any-return]

    def start_all_modules(self) -> None:
        for module in self._deployed_modules.values():
            module.start()

    def get_instance(self, module: type[T]) -> T | None:
        return self._deployed_modules.get(module)  # type: ignore[return-value]

    def loop(self) -> None:
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            return
        finally:
            self.stop()
