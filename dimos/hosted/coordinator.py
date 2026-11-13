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

"""The hosted run's aggregate introspection service for desktop and shell clients."""

from collections.abc import Callable, Sequence
from typing import Any

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.coordination.module_coordinator import ModuleDescriptor
from dimos.core.introspection.module.info import StreamDescriptor
from dimos.hosted.protocol import run_coordinator_rpc_name
from dimos.protocol.rpc.spec import RPCSpec


class HostedCoordinator:
    def __init__(
        self, rpc: RPCSpec, run_id: str, host_ids: Sequence[str], stop: Callable[[], None]
    ) -> None:
        self._coordinators = [
            CoordinatorRPC(rpc, run_coordinator_rpc_name(run_id, host_id))
            for host_id in sorted(set(host_ids))
        ]
        self._stop = stop

    @property
    def rpcs(self) -> dict[str, Callable[..., Any]]:
        return {
            "ping": self.ping,
            "list_modules": self.list_modules,
            "list_streams": self.list_streams,
            "shutdown": self._stop,
        }

    def ping(self) -> str:
        return "pong"

    def list_modules(self) -> list[ModuleDescriptor]:
        return [
            module
            for coordinator in self._coordinators
            for module in coordinator.call("list_modules")
        ]

    def list_streams(self) -> list[StreamDescriptor]:
        return [
            stream
            for coordinator in self._coordinators
            for stream in coordinator.call("list_streams")
        ]
