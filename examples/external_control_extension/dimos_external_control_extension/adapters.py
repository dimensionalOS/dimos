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

"""Hardware adapter implemented by an external package."""

from __future__ import annotations

import logging
from pathlib import Path
import threading

LOGGER = logging.getLogger("dimos_external_control_extension")


class ExternalTestBaseAdapter:
    """In-memory twist-base adapter used by the external package demo."""

    def __init__(
        self,
        dof: int = 3,
        address: str | Path | None = None,
        hardware_id: str = "external_test_base",
        **adapter_kwargs: object,
    ) -> None:
        self._dof = dof
        self._hardware_id = hardware_id
        self._connected = False
        self._enabled = False
        self._last_velocities = [0.0] * dof
        self._write_count = 0
        self.command_event = threading.Event()
        LOGGER.info(
            "[external_test_robot] ExternalTestBaseAdapter.__init__(dof=%s, hardware_id=%s, address=%s, kwargs=%s)",
            dof,
            hardware_id,
            address,
            adapter_kwargs,
        )

    @property
    def write_count(self) -> int:
        return self._write_count

    @property
    def last_velocities(self) -> list[float]:
        return list(self._last_velocities)

    def connect(self) -> bool:
        self._connected = True
        LOGGER.info("[external_test_robot] ExternalTestBaseAdapter.connect()")
        return True

    def disconnect(self) -> None:
        self._connected = False
        LOGGER.info("[external_test_robot] ExternalTestBaseAdapter.disconnect()")

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return self._dof

    def read_velocities(self) -> list[float]:
        return list(self._last_velocities)

    def read_odometry(self) -> list[float] | None:
        return [0.0] * self._dof

    def write_velocities(self, velocities: list[float]) -> bool:
        self._last_velocities = list(velocities)
        self._write_count += 1
        self.command_event.set()
        LOGGER.info(
            "[external_test_robot] ExternalTestBaseAdapter.write_velocities(%s)",
            velocities,
        )
        return True

    def write_stop(self) -> bool:
        return self.write_velocities([0.0] * self._dof)

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        LOGGER.info("[external_test_robot] ExternalTestBaseAdapter.write_enable(%s)", enable)
        return True

    def read_enabled(self) -> bool:
        return self._enabled


__all__ = ["ExternalTestBaseAdapter"]
