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

"""Stock firmware IO owned by the single G1 connection module."""

from typing import Any, Protocol, runtime_checkable

from dimos.control.connection.device import ConnectionDevice


@runtime_checkable
class G1StockDevice(ConnectionDevice, Protocol):
    def get_state(self) -> str: ...

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]: ...

    def stand_up(self) -> bool: ...

    def lie_down(self) -> bool: ...
