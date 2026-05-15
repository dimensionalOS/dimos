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

from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.protocol.rpc.spec import RPCInspectable

logger = setup_logger()


class CoordinatorRPC:
    """Owns the LCM RPC connection to the singleton Coordinator service."""

    NAME = "Coordinator"

    def __init__(self, rpc: LCMRPC) -> None:
        self._rpc = rpc

    @classmethod
    def serve(cls, coordinator: RPCInspectable) -> CoordinatorRPC:
        """Publish `coordinator`'s @rpc methods under the `Coordinator/` prefix."""
        cls._ensure_no_existing_service()
        rpc = LCMRPC()
        rpc.serve_module_rpc(coordinator, name=cls.NAME)
        rpc.start()
        return cls(rpc)

    @classmethod
    def connect(cls, *, timeout: float) -> CoordinatorRPC:
        """Attach to a running Coordinator, raising `TimeoutError` if none answers."""
        rpc = LCMRPC()
        rpc.start()
        client = cls(rpc)
        try:
            client.call("ping", rpc_timeout=timeout)
        except BaseException:
            rpc.stop()
            raise
        return client

    def call(self, method: str, *args: Any, rpc_timeout: float | None = None, **kwargs: Any) -> Any:
        """Invoke `Coordinator/<method>` and return its result."""
        result, _unsub = self._rpc.call_sync(
            f"{self.NAME}/{method}",
            ([*args], kwargs),
            rpc_timeout=rpc_timeout,
        )
        return result

    @property
    def rpc(self) -> LCMRPC:
        return self._rpc

    def stop(self) -> None:
        try:
            self._rpc.stop()
        except Exception:
            logger.error("Error closing Coordinator RPC service", exc_info=True)

    @classmethod
    def _ensure_no_existing_service(cls) -> None:
        probe = LCMRPC()
        probe.start()
        try:
            try:
                probe.call_sync(f"{cls.NAME}/ping", ([], {}), rpc_timeout=0.5)
            except TimeoutError:
                return
            raise RuntimeError(
                f"another {cls.NAME} service is already running on this LCM bus. "
                "Run `dimos stop` first."
            )
        finally:
            probe.stop()
