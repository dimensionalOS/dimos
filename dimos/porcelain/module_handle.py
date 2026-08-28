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

from collections.abc import Callable
from typing import TypeAlias

from dimos.core.rpc_client import ModuleProxyProtocol, RpcCall, RPCClient
from dimos.protocol.rpc.spec import RPCSpec


class RemoteModuleProxy:
    """Names-only handle for a module whose Python class is unavailable.

    Only RPC names advertised by the coordinator are accessible.
    """

    def __init__(self, rpc: RPCSpec, remote_name: str, rpc_names: set[str]) -> None:
        self._rpc = rpc
        self._remote_name = remote_name
        self._rpc_names = rpc_names
        self._unsub_fns: list[Callable[[], None]] = []

    def __getattr__(self, name: str) -> RpcCall:
        if name.startswith("_"):
            raise AttributeError(name)
        if name not in self._rpc_names:
            raise AttributeError(f"{self._remote_name!r} has no @rpc method named {name!r}")
        return RpcCall(None, self._rpc, name, self._remote_name, self._unsub_fns, None)

    def __dir__(self) -> list[str]:
        return sorted(self._rpc_names)

    @property
    def remote_name(self) -> str:
        """The exact deployed module-instance name."""
        return self._remote_name


ModuleHandle: TypeAlias = ModuleProxyProtocol | RPCClient | RemoteModuleProxy
