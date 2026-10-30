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

"""Device lifecycle shared by hardware and simulated connection backends."""

from collections.abc import Callable
from typing import Any, Protocol

Feedback = Callable[[str, Any], None]
"""Publish a typed reading on one of the connection's declared output ports."""


class ConnectionDevice(Protocol):
    """A connection's internal device; construction must not open hardware.

    Devices receive a feedback callback when constructed. ``start`` opens the
    device and starts readings without commanding motion. ``write`` receives
    named values already validated by the connection. ``halt`` must be safe
    before the first command, and ``stop`` must release partially opened devices.
    """

    def start(self) -> None: ...

    def write(self, values: dict[str, float]) -> None: ...

    def halt(self) -> None: ...

    def stop(self) -> None: ...


DeviceFactory = Callable[[Feedback], ConnectionDevice]
