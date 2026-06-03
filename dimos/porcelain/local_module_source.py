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

from typing import TYPE_CHECKING, Any

from dimos.porcelain.module_source import ModuleSource
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

logger = setup_logger()


class LocalModuleSource(ModuleSource):
    """Module source backed by an in-process `ModuleCoordinator`.

    Returns the per-module `RPCClient` proxies the coordinator already
    maintains for inter-module calls. Method calls flow over the same LCM
    bus the modules use to talk to each other.
    """

    is_remote = False

    def __init__(self, coordinator: ModuleCoordinator) -> None:
        self._coordinator = coordinator

    def list_module_names(self) -> list[str]:
        return self._coordinator.list_module_names()

    def get_module(self, name: str) -> Any:
        for cls, proxy in self._coordinator._deployed_modules.items():
            if cls.__name__ == name:
                return proxy
        raise KeyError(name)

    def invalidate(self, name: str) -> None:
        return None

    def close(self) -> None:
        return None
