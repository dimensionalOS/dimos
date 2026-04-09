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
from typing import Any

import rpyc

from dimos.porcelain.module_source import ModuleSource


class RemoteModuleSource(ModuleSource):
    """Module source backed by a remote `CoordinatorService` RPyC endpoint."""

    is_remote = True

    def __init__(self, host: str, port: int) -> None:
        self._coord_conn = rpyc.connect(host, port, config={"sync_request_timeout": 30})
        self._cache: dict[str, tuple[rpyc.Connection, Any]] = {}
        self._lock = threading.RLock()

    def list_module_names(self) -> list[str]:
        return list(self._coord_conn.root.list_modules())

    def get_rpyc_module(self, name: str) -> Any:
        with self._lock:
            cached = self._cache.get(name)
            if cached is not None and not cached[0].closed:
                return cached[1]

            endpoint = self._coord_conn.root.get_module_endpoint(name)
            host, port, module_id = endpoint[0], int(endpoint[1]), int(endpoint[2])
            conn = rpyc.connect(host, port, config={"sync_request_timeout": 30})
            module = conn.root.get_module(module_id)
            self._cache[name] = (conn, module)
            return module

    def close(self) -> None:
        with self._lock:
            for conn, _ in self._cache.values():
                try:
                    conn.close()
                except Exception:
                    pass
            self._cache.clear()
        try:
            self._coord_conn.close()
        except Exception:
            pass
