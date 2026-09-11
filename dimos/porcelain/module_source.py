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

from typing import TYPE_CHECKING, Protocol

from dimos.porcelain.module_handle import ModuleHandle

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleDescriptor


class ModuleSource(Protocol):
    """Common interface for owned and coordinator-connected module sources.

    "Local" is backed by the coordinator owned by ``Dimos.run()``. "Remote" is
    relative to the client: it connects to a separately running coordinator,
    which may still be on the same host.
    """

    is_remote: bool

    def list_module_names(self) -> list[str]: ...

    def list_module_descriptors(self) -> list[ModuleDescriptor]: ...

    def get_module(self, name: str) -> ModuleHandle: ...

    def close(self) -> None: ...
