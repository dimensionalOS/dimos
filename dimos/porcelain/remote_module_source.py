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

import functools
import importlib
import pickle
import threading
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.coordination.module_coordinator import ModuleDescriptor
from dimos.core.introspection.module.info import StreamDescriptor
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import RemoteOut
from dimos.core.transport import WebRTCAudioTransport, WebRTCTransport, WebRTCVideoTransport
from dimos.porcelain.module_handle import ModuleHandle, RemoteModuleProxy
from dimos.porcelain.module_source import ModuleSource
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

logger = setup_logger()


class RemoteModuleSource(ModuleSource):
    """Module source connected to a separately running coordinator over RPC.

    The coordinator may be on the same host; "remote" distinguishes this from
    the in-process coordinator owned by ``Dimos.run()``.
    """

    is_remote = True

    def __init__(self, *, timeout: float = 5.0) -> None:
        self._timeout = timeout
        self._cache: dict[str, RPCClient | RemoteModuleProxy] = {}
        self._descriptors: dict[str, ModuleDescriptor] | None = None
        self._streams: dict[tuple[str, str], RemoteOut[Any]] = {}
        self._lock = threading.RLock()
        self._closed = False

        try:
            self._coord = CoordinatorRPC.connect(timeout=timeout)
        except TimeoutError:
            raise RuntimeError(
                "No running DimOS coordinator found on the configured transport bus. "
                "Start one with `dimos run <blueprint>` or enter a coordinator loop "
                "from Python."
            ) from None

    def _refresh_descriptors(self) -> dict[str, ModuleDescriptor]:
        if self._closed:
            raise RuntimeError("This module source has been stopped")
        descriptors = self._coord.call("list_modules")
        # rpc_name is empty when talking to an older daemon.
        self._descriptors = {(d.rpc_name or d.class_name): d for d in descriptors}
        return self._descriptors

    def _get_descriptor(self, name: str) -> ModuleDescriptor:
        with self._lock:
            if self._descriptors is None or name not in self._descriptors:
                self._refresh_descriptors()
            assert self._descriptors is not None
            if name in self._descriptors:
                return self._descriptors[name]

            matches = [d for d in self._descriptors.values() if d.class_name == name]
            if len(matches) == 1:
                return matches[0]
            if len(matches) > 1:
                instance_names = ", ".join(sorted(d.rpc_name for d in matches))
                raise ValueError(
                    f"Multiple instances of {name!r} are deployed "
                    f"({instance_names}); use the instance name."
                )
            raise KeyError(name)

    def list_module_names(self) -> list[str]:
        with self._lock:
            descriptors = self._refresh_descriptors()
            return list(descriptors.keys())

    def list_module_descriptors(self) -> list[ModuleDescriptor]:
        with self._lock:
            return list(self._refresh_descriptors().values())

    def list_stream_descriptors(self) -> list[StreamDescriptor]:
        with self._lock:
            if self._closed:
                raise RuntimeError("This module source has been stopped")
            return list(self._coord.call("list_streams", rpc_timeout=self._timeout))

    def get_module(self, name: str) -> ModuleHandle:
        with self._lock:
            descriptor = self._get_descriptor(name)
            remote_name = descriptor.rpc_name or descriptor.class_name
            cached = self._cache.get(remote_name)
            if cached is not None:
                return cached

            proxy: RPCClient | RemoteModuleProxy
            try:
                module_path, class_name = descriptor.qualified_path.rsplit(".", 1)
                cls = getattr(importlib.import_module(module_path), class_name)
                proxy = RPCClient(
                    None,
                    cls,
                    remote_name,
                    rpc=self._coord.rpc,
                    stream_lookup=functools.partial(self._get_stream, remote_name),
                )
            except (ImportError, AttributeError):
                proxy = RemoteModuleProxy(
                    self._coord.rpc,
                    remote_name,
                    set(descriptor.rpc_names),
                    functools.partial(self._get_stream, remote_name),
                )
            self._cache[remote_name] = proxy
            return proxy

    def _get_stream(self, remote_name: str, name: str) -> RemoteOut[Any]:
        with self._lock:
            if self._closed:
                raise RuntimeError("This module source has been stopped")
            key = (remote_name, name)
            if key not in self._streams:
                descriptor = next(
                    (d for d in self.list_stream_descriptors() if (d.module, d.name) == key), None
                )
                if descriptor is None:
                    raise AttributeError(
                        f"{remote_name!r} has no @rpc method or stream named {name!r}"
                    )
                stream_type, transport = pickle.loads(descriptor.transport)
                if isinstance(
                    transport, (WebRTCTransport, WebRTCVideoTransport, WebRTCAudioTransport)
                ):
                    raise NotImplementedError(
                        "WebRTC stream subscriptions require an operator client; "
                        "use the hosted teleop client."
                    )
                # Expose a subscription view for inputs, outputs and IO ports.
                self._streams[key] = RemoteOut(stream_type, name, remote_name, transport)
            return self._streams[key]

    def invalidate(self, name: str) -> None:
        with self._lock:
            entry = self._cache.pop(name, None)
            self._descriptors = None
            streams = [self._streams.pop(key) for key in list(self._streams) if key[0] == name]
        for stream in streams:
            stream.stop()
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
            if self._closed:
                return
            self._closed = True
            entries = list(self._cache.values())
            streams = list(self._streams.values())
            self._cache.clear()
            self._descriptors = None
            self._streams.clear()
        # Never hold the source lock while waiting for dispatch threads.
        for entry in entries:
            if isinstance(entry, RPCClient):
                try:
                    entry.stop_rpc_client()
                except Exception:
                    pass
        for stream in streams:
            stream.stop()
        try:
            self._coord.stop()
        except Exception:
            pass
