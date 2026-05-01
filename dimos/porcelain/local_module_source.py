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

import threading
from typing import TYPE_CHECKING, Any

import rpyc

from dimos.porcelain.module_source import ModuleSource
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

logger = setup_logger()


class LocalModuleSource(ModuleSource):
    """Module source backed by an in-process `ModuleCoordinator`.

    Uses per-worker RPyC servers for both attribute access and skill calls,
    so the wire path is identical to `RemoteModuleSource`.
    """

    is_remote = False

    def __init__(self, coordinator: ModuleCoordinator) -> None:
        self._coordinator = coordinator
        self._cache: dict[str, tuple[rpyc.Connection, Any]] = {}
        self._lock = threading.RLock()

    def list_module_names(self) -> list[str]:
        return self._coordinator.list_module_names()

    def get_rpyc_module(self, name: str) -> Any:
        with self._lock:
            cached = self._cache.get(name)
            if cached is not None and not cached[0].closed:
                return cached[1]

            host, port, module_id = self._coordinator.get_module_endpoint(name)

            conn = rpyc.connect(
                host, port, config={"sync_request_timeout": 30, "allow_pickle": True}
            )
            module = conn.root.get_module(module_id)
            self._cache[name] = (conn, module)
            return module

    def invalidate(self, name: str) -> None:
        with self._lock:
            entry = self._cache.pop(name, None)
        if entry is not None:
            try:
                entry[0].close()
            except Exception:
                logger.warning("Failed to close RPyC connection for module %s", name, exc_info=True)

    def close(self) -> None:
        with self._lock:
            for conn, _ in self._cache.values():
                try:
                    conn.close()
                except Exception:
                    logger.warning("Failed to close RPyC connection during shutdown", exc_info=True)
            self._cache.clear()
