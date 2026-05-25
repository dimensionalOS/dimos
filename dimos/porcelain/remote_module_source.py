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

import importlib
import threading
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.coordination.module_coordinator import ModuleDescriptor
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.porcelain.module_source import ModuleSource
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint
    from dimos.protocol.rpc.pubsubrpc import LCMRPC

logger = setup_logger()


class _RemoteProxy:
    """Names-only proxy for a remote module whose class can't be imported.

    Exposes only the @rpc methods the remote daemon advertised; any other
    attribute access raises `AttributeError`.
    """

    def __init__(self, rpc: LCMRPC, remote_name: str, rpc_names: set[str]) -> None:
        self._rpc = rpc
        self._remote_name = remote_name
        self._rpc_names = rpc_names
        self._unsub_fns: list = []  # type: ignore[type-arg]

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        if name not in self._rpc_names:
            raise AttributeError(f"{self._remote_name!r} has no @rpc method named {name!r}")
        return RpcCall(None, self._rpc, name, self._remote_name, self._unsub_fns, None)

    def __dir__(self) -> list[str]:
        return sorted(self._rpc_names)


class RemoteModuleSource(ModuleSource):
    """Module source that drives a remote daemon over the Coordinator @rpc service."""

    is_remote = True

    def __init__(self, *, timeout: float = 5.0) -> None:
        self._timeout = timeout
        self._cache: dict[str, RPCClient | _RemoteProxy] = {}
        self._descriptors: dict[str, ModuleDescriptor] | None = None
        self._lock = threading.RLock()

        try:
            self._coord = CoordinatorRPC.connect(timeout=timeout)
        except TimeoutError:
            raise RuntimeError(
                "No running DimOS instance. Start one with `dimos run <blueprint>`."
            ) from None

    def _refresh_descriptors(self) -> dict[str, ModuleDescriptor]:
        descriptors = self._coord.call("list_modules")
        self._descriptors = {d.class_name: d for d in descriptors}
        return self._descriptors

    def _get_descriptor(self, name: str) -> ModuleDescriptor:
        with self._lock:
            if self._descriptors is None or name not in self._descriptors:
                self._refresh_descriptors()
            assert self._descriptors is not None
            if name not in self._descriptors:
                raise KeyError(name)
            return self._descriptors[name]

    def list_module_names(self) -> list[str]:
        with self._lock:
            descriptors = self._refresh_descriptors()
            return list(descriptors.keys())

    def get_module(self, name: str) -> Any:
        with self._lock:
            cached = self._cache.get(name)
            if cached is not None:
                return cached

            descriptor = self._get_descriptor(name)
            proxy: RPCClient | _RemoteProxy
            try:
                module_path, class_name = descriptor.qualified_path.rsplit(".", 1)
                cls = getattr(importlib.import_module(module_path), class_name)
                proxy = RPCClient(None, cls, rpc=self._coord.rpc)
            except (ImportError, AttributeError):
                proxy = _RemoteProxy(self._coord.rpc, name, set(descriptor.rpc_names))
            self._cache[name] = proxy
            return proxy

    def invalidate(self, name: str) -> None:
        with self._lock:
            entry = self._cache.pop(name, None)
            self._descriptors = None
        if isinstance(entry, RPCClient):
            try:
                entry.stop_rpc_client()
            except Exception:
                logger.warning("Failed to release proxy for %s", name, exc_info=True)

    def load_blueprint_by_name(self, name: str) -> None:
        self._coord.call("load_blueprint_by_name", name)

    def load_blueprint(self, blueprint: Blueprint) -> None:
        self._coord.call("load_blueprint", blueprint)

    def restart_module_by_class_name(self, class_name: str, *, reload_source: bool) -> None:
        self._coord.call("restart_module_by_class_name", class_name, reload_source=reload_source)
        self.invalidate(class_name)

    def close(self) -> None:
        with self._lock:
            for entry in self._cache.values():
                if isinstance(entry, RPCClient):
                    try:
                        entry.stop_rpc_client()
                    except Exception:
                        pass
            self._cache.clear()
            self._descriptors = None
        try:
            self._coord.stop()
        except Exception:
            pass
