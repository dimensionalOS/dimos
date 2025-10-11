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

from typing import Optional, Type, TypeVar

from dimos import core
from dimos.core import DimosCluster, Module
from dimos.core.resource import Resource

T = TypeVar("T", bound="Module")


class Dimos(Resource):
    _client: Optional[DimosCluster] = None
    _n: Optional[int] = None
    _memory_limit: str = "auto"
    _deployed_modules: dict[Type[Module], Module] = {}

    def __init__(self, n: Optional[int] = None, memory_limit: str = "auto"):
        self._n = n
        self._memory_limit = memory_limit

    def start(self) -> None:
        self._client = core.start(self._n, self._memory_limit)

    def stop(self) -> None:
        for module in reversed(self._deployed_modules.values()):
            module.stop()

        self._client.close_all()

    def deploy(self, module_class: Type[T], *args, **kwargs) -> T:
        if not self._client:
            raise ValueError("Not started")

        module = self._client.deploy(module_class, *args, **kwargs)
        self._deployed_modules[module_class] = module
        return module

    def start_all_modules(self) -> None:
        for module in self._deployed_modules.values():
            module.start()

    def get_instance(self, module: Type[T]) -> T | None:
        return self._deployed_modules.get(module)
