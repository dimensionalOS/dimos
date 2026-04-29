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

import pickle
from typing import TYPE_CHECKING, Any

import rpyc

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

_CONFIG = {
    "allow_all_attrs": True,
    "allow_public_attrs": True,
    "allow_setattr": True,
}


class CoordinatorService(rpyc.Service):  # type: ignore[misc]
    """RPyC discovery service exposed by the coordinator process.

    Lets an out-of-process `Dimos.connect()` client enumerate deployed
    modules and look up the worker RPyC endpoint for each one.
    """

    _coordinator: ModuleCoordinator | None = None

    def on_connect(self, conn: Any) -> None:
        conn._config.update(_CONFIG)

    def exposed_list_modules(self) -> list[str]:
        assert self._coordinator is not None
        return self._coordinator.list_module_names()

    def exposed_get_module_endpoint(self, class_name: str) -> tuple[str, int, int]:
        assert self._coordinator is not None
        return self._coordinator.get_module_endpoint(class_name)

    def exposed_load_blueprint_by_name(self, name: str) -> None:
        # Avoid circular import.
        from dimos.robot.get_all_blueprints import get_by_name

        assert self._coordinator is not None
        self._coordinator.load_blueprint(get_by_name(name))

    def exposed_load_blueprint_pickled(self, data: bytes) -> None:
        assert self._coordinator is not None
        try:
            blueprint = pickle.loads(data)
        except Exception as e:
            raise RuntimeError(
                f"Failed to unpickle Blueprint on daemon ({type(e).__name__}: {e}). "
                "The blueprint's module classes must be importable on the daemon "
                "and all kwargs must be picklable."
            ) from e
        self._coordinator.load_blueprint(blueprint)

    def exposed_restart_module_by_class_name(
        self, class_name: str, reload_source: bool = True
    ) -> None:
        assert self._coordinator is not None
        self._coordinator.restart_module_by_class_name(class_name, reload_source=reload_source)


class WorkerRpycService(rpyc.Service):  # type: ignore[misc]
    _instances: dict[int, Any] = {}

    def on_connect(self, conn: Any) -> None:
        conn._config.update(_CONFIG)

    def exposed_get_module(self, module_id: int) -> Any:
        return self._instances[module_id]
