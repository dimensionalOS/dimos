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
import time
from typing import TYPE_CHECKING, Any

from filelock import FileLock

from dimos.constants import RUNTIME_DIR
from dimos.core.global_config import global_config
from dimos.core.transport_factory import rpc_backend
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.protocol.rpc.spec import RPCInspectable, RPCSpec

logger = setup_logger()

_CONNECT_ATTEMPT_TIMEOUT = 0.25


class CoordinatorAlreadyRunningError(RuntimeError):
    """Another run owns the requested Coordinator address."""


class CoordinatorRPC:
    """Owns the RPC connection to the singleton Coordinator service."""

    NAME = "Coordinator"

    def __init__(self, rpc: RPCSpec, name: str = NAME) -> None:
        self._rpc = rpc
        self._name = name
        self._published = False

    @classmethod
    def reserve(cls, name: str) -> CoordinatorRPC:
        """Claim the name before deployment. The public RPCs appear after publish()."""
        rpc = rpc_backend()()
        rpc.start()
        try:
            # Serialize the short probe-and-register window for simultaneous local
            # launches. The bus, including remote services, determines occupancy.
            RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
            with FileLock(RUNTIME_DIR / "coordinator-admission.lock"):
                cls._ensure_no_existing_service(rpc, name)
                rpc.serve_rpc(lambda: True, f"{name}/reserved")
        except BaseException:
            rpc.stop()
            raise
        return cls(rpc, name)

    def publish(self, coordinator: RPCInspectable) -> None:
        """Make the ready Coordinator available at the reserved address."""
        if not self._published:
            self._rpc.serve_module_rpc(coordinator, name=self._name)
            self._published = True

    @classmethod
    def serve(
        cls,
        coordinator: RPCInspectable,
        *,
        name: str | None = None,
    ) -> CoordinatorRPC:
        """Publish ``coordinator`` under the selected RPC prefix."""
        service = cls.reserve(name or cls.NAME)
        try:
            service.publish(coordinator)
        except BaseException:
            service.stop()
            raise
        return service

    @classmethod
    def connect(
        cls,
        *,
        timeout: float,
        name: str | None = None,
        before_retry: Callable[[], None] | None = None,
    ) -> CoordinatorRPC:
        """Attach to a running Coordinator, raising `TimeoutError` if none answers.

        ``before_retry`` runs after every unanswered ping; raising from it gives
        up early, e.g. once the process meant to serve the Coordinator has exited.
        """
        selected_name = name or cls.NAME
        rpc = rpc_backend()()
        rpc.start()
        client = cls(rpc, selected_name)
        deadline = time.monotonic() + timeout
        last_timeout: TimeoutError | None = None
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Coordinator did not respond within {timeout} seconds"
                    ) from last_timeout
                try:
                    client.call(
                        "ping",
                        rpc_timeout=min(_CONNECT_ATTEMPT_TIMEOUT, remaining),
                    )
                    break
                except TimeoutError as exc:
                    last_timeout = exc
                    if before_retry is not None:
                        before_retry()
        except BaseException:
            rpc.stop()
            raise
        return client

    def call(self, method: str, *args: Any, rpc_timeout: float | None = None, **kwargs: Any) -> Any:
        """Invoke `Coordinator/<method>` and return its result."""
        result, _unsub = self._rpc.call_sync(
            f"{self._name}/{method}",
            ([*args], kwargs),
            rpc_timeout=rpc_timeout,
        )
        return result

    @property
    def rpc(self) -> RPCSpec:
        return self._rpc

    @property
    def name(self) -> str:
        return self._name

    def stop(self) -> None:
        try:
            self._rpc.stop()
        except Exception:
            logger.error("Error closing Coordinator RPC service", exc_info=True)

    @staticmethod
    def _ensure_no_existing_service(rpc: RPCSpec, name: str) -> None:
        # Older Coordinators answer only ping. A Coordinator still starting
        # answers reserved and leaves ping unavailable to readiness probes.
        for method in ("ping", "reserved"):
            try:
                rpc.call_sync(f"{name}/{method}", ([], {}), rpc_timeout=0.5)
            except TimeoutError:
                continue
            raise CoordinatorAlreadyRunningError(
                f"another {name} service is already running on the "
                f"{global_config.transport} bus. Run `dimos stop` first."
            )
