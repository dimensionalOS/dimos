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

"""The ``Coordinator`` RPC service for a hosted run: one view over every fragment's coordinator.

A hosted run has no single coordinator; each Host's fragment serves its own under
``runs/<run>/hosts/<host>/coordinator``. Clients that look for ``Coordinator`` (``Dimos.connect()``,
Desktop's gateway, the Controller's RPC buttons) get this instead: ``list_modules`` and ``list_streams``
gathered from every fragment. Module RPCs need nothing more, their names are global.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleDescriptor
    from dimos.core.introspection.module.info import StreamDescriptor
    from dimos.protocol.rpc.spec import RPCSpec

logger = setup_logger()

# a fragment that doesn't answer within this is left out of a listing rather than stalling it
FRAGMENT_TIMEOUT_S = 5.0


class HostedRunCoordinator:
    """``Coordinator`` for a hosted run: ping, and the fragments' modules and streams together."""

    def __init__(self, rpc: RPCSpec, fragment_names: Iterable[str]) -> None:
        self._fragments = [CoordinatorRPC(rpc, name) for name in fragment_names]

    @property
    def rpcs(self) -> dict[str, Callable[..., Any]]:
        return {
            "ping": self.ping,
            "list_modules": self.list_modules,
            "list_streams": self.list_streams,
        }

    def ping(self) -> str:
        return "pong"

    def list_modules(self) -> list[ModuleDescriptor]:
        return self._gather("list_modules")

    def list_streams(self) -> list[StreamDescriptor]:
        return self._gather("list_streams")

    def _gather(self, method: str) -> list[Any]:
        found: list[Any] = []
        for fragment in self._fragments:
            try:
                found.extend(fragment.call(method, rpc_timeout=FRAGMENT_TIMEOUT_S))
            except TimeoutError:
                logger.warning(f"hosted run: {fragment.name} did not answer {method}")
        return found


def serve_hosted_run_coordinator(fragment_names: Iterable[str]) -> CoordinatorRPC | None:
    """Publish a ``Coordinator`` over the given fragment coordinators; None (with a warning) if one already answers."""
    try:
        served = CoordinatorRPC.serve(HostedRunCoordinator(_client_rpc(), fragment_names))
    except RuntimeError as error:
        logger.warning(f"hosted run: not serving Coordinator: {error}")
        return None
    return served


def _client_rpc() -> RPCSpec:
    from dimos.core.transport_factory import rpc_backend

    rpc = rpc_backend()()
    rpc.start()
    return rpc
