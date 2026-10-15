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

from typing import TYPE_CHECKING

from dimos.porcelain.module_handle import ModuleHandle
from dimos.porcelain.module_source import ModuleSource
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import (
        ModuleCoordinator,
        ModuleDescriptor,
    )
    from dimos.core.introspection.module.info import StreamDescriptor

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

    def list_module_descriptors(self) -> list[ModuleDescriptor]:
        return self._coordinator.list_modules()

    def list_stream_descriptors(self) -> list[StreamDescriptor]:
        return self._coordinator.list_streams()

    def get_module(self, name: str) -> ModuleHandle:
        return self._coordinator._deployed_modules[self.instance_key(name)]

    def instance_key(self, name: str) -> str:
        """The deployed instance that an instance name or class name refers to."""
        if name in self._coordinator._deployed_modules:
            return name

        matches = [
            instance_key
            for instance_key, cls in self._coordinator._instance_classes.items()
            if cls.__name__ == name
        ]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError(
                f"Multiple instances of {name!r} are deployed "
                f"({', '.join(sorted(matches))}); use the instance name."
            )
        raise KeyError(name)

    def invalidate(self, name: str) -> None:
        return None

    def close(self) -> None:
        return None
