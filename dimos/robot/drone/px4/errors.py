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

"""PX4 configuration and runtime errors."""

from dataclasses import dataclass

from typing_extensions import override


@dataclass(frozen=True, slots=True)
class InvalidMavsdkConfigError(ValueError):
    field: str
    detail: str

    @override
    def __str__(self) -> str:
        return f"{self.field}: {self.detail}"


@dataclass(frozen=True, slots=True)
class MavsdkUnavailableError(RuntimeError):
    """Raised when flight control is started without the optional MAVSDK package."""

    @override
    def __str__(self) -> str:
        return "MAVSDK is required to connect PX4 flight control"


@dataclass(frozen=True, slots=True)
class MavsdkConnectionTimeoutError(TimeoutError):
    connection_url: str
    timeout_s: float

    @override
    def __str__(self) -> str:
        return f"MAVSDK did not connect to {self.connection_url} within {self.timeout_s:g} seconds"
