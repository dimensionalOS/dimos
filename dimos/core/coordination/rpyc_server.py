# Copyright 2025-2026 Dimensional Inc.
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

from threading import Thread
from typing import TYPE_CHECKING

from rpyc.utils.server import ThreadedServer

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.coordination.rpyc_services import CoordinatorService
from dimos.core.global_config import global_config
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

logger = setup_logger()


class RpycServer:
    def __init__(self, coordinator: ModuleCoordinator) -> None:
        self._coordinator = coordinator
        self._server: ThreadedServer | None = None
        self._thread: Thread | None = None

    def start(self) -> int:
        """Start the discovery service and return the bound port."""
        # Create a class at runtime because RPyC takes a class, not an object.
        bound_service = type(
            "_BoundCoordinatorService",
            (CoordinatorService,),
            {"_coordinator": self._coordinator},
        )

        self._server = ThreadedServer(
            bound_service,
            port=0,
            hostname=global_config.listen_host,
            protocol_config={
                "allow_all_attrs": True,
                "allow_public_attrs": True,
            },
        )
        self._thread = Thread(target=self._server.start, daemon=True, name="coordinator-rpyc")
        self._thread.start()
        return int(self._server.port)

    def stop(self) -> None:
        if self._server is not None:
            try:
                self._server.close()
            except Exception:
                logger.error("Error closing coordinator RPyC server", exc_info=True)
            self._server = None

        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                logger.warning("Coordinator RPyC thread did not exit within timeout")
            self._thread = None

    @property
    def port(self) -> int:
        if self._server is None:
            return 0
        return int(self._server.port)
