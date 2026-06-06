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

from collections.abc import Callable
import threading
from typing import Any

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class PanelBackend:
    def __init__(
        self,
        *,
        module_ref: Any | None,
        module_rpc: Any | None,
        remote_factory: Callable[..., Any],
        timeout_seconds: Callable[[], float],
    ) -> None:
        self._module_ref = module_ref
        self._module_rpc = module_rpc
        self._remote_factory = remote_factory
        self._timeout_seconds = timeout_seconds
        self.client: Any | None = None

    def reset_client(self) -> Any | None:
        if self.client is not None and self.client is not self._module_ref:
            self.client.stop_rpc_client()
        if self._module_ref is not None:
            self.client = self._module_ref
            return self.client
        self.client = self._remote_factory(
            ManipulationModule,
            rpc=self._module_rpc if isinstance(self._module_rpc, LCMRPC) else None,
        )
        return self.client

    def close(self) -> None:
        if self.client is not None and self.client is not self._module_ref:
            self.client.stop_rpc_client()
        self.client = None

    def ensure_client(self) -> Any:
        if self.client is None:
            self.reset_client()
        if self.client is None:
            raise RuntimeError("Manipulation RPC client is not connected")
        return self.client

    def list_robots(self) -> list[str]:
        client = self.ensure_client()
        try:
            return list(client.list_robots())
        except TimeoutError:
            self.reset_client()
            return list(self.ensure_client().list_robots())

    def call_preview_with_timeout(
        self, call: Callable[[], dict[str, Any]], timeout_status: str
    ) -> dict[str, Any]:
        result: dict[str, Any] | None = None
        error: Exception | None = None

        def run() -> None:
            nonlocal result, error
            try:
                result = call()
            except Exception as e:
                error = e

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        timeout = max(self._timeout_seconds(), 0.0)
        thread.join(timeout=timeout)
        if thread.is_alive():
            return {
                "success": False,
                "status": timeout_status,
                "message": f"Preview request timed out after {timeout:.1f}s",
                "collision_free": False,
            }
        if error is not None:
            raise error
        return result or {
            "success": False,
            "status": "EMPTY_RESULT",
            "message": "Preview request returned no result",
            "collision_free": False,
        }

    def solve_ik_preview(self, pose: Any, robot_name: str) -> dict[str, Any]:
        client = self.ensure_client()
        return self.call_preview_with_timeout(
            lambda: dict(client.solve_ik_preview(pose, robot_name)), "IK_TIMEOUT"
        )

    def solve_fk_preview(self, joints: Any, robot_name: str) -> dict[str, Any]:
        client = self.ensure_client()
        return self.call_preview_with_timeout(
            lambda: dict(client.solve_fk_preview(joints, robot_name)), "FK_TIMEOUT"
        )
