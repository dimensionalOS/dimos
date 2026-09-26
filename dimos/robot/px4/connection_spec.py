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

from typing import Any, Protocol

from dimos.spec.utils import Spec


class Px4DroneConnectionSpec(Spec, Protocol):
    """The operator commands of ``Px4DroneConnection`` that ``Px4SkillContainer`` calls."""

    def takeoff(self, altitude_m: float | None = None) -> dict[str, Any]: ...
    def go_to(
        self,
        north_m: float = 0.0,
        east_m: float = 0.0,
        altitude_m: float | None = None,
        heading_deg: float | None = None,
        relative: bool = True,
    ) -> dict[str, Any]: ...
    def land(self) -> dict[str, Any]: ...
    def set_guidance_mode(self, mode: str) -> dict[str, Any]: ...
    def status(self) -> dict[str, Any]: ...
