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

"""RobotModel as a module config field, picked by name.

    GO2_RIGS = {"sf": GO2_SF, "athens": GO2_ATHENS}
    Go2Robot = Annotated[RobotModel, Named(GO2_RIGS)]

    class Go2ConnectionConfig(RobotAwareConfig):
        robot: Go2Robot = "sf"

The field takes a model, a name from the mapping, or a URDF path. Its schema is a string
enum of the names, so env/CLI can switch rigs (`--go2connection.robot athens`) and
`--help` lists them. It dumps as the name while the value equals a named model, else as
`<RobotModel 23 links, 12 joints>`. Edits happen in Python, not through config paths.

SKETCH: signatures only.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from pydantic import GetCoreSchemaHandler
from pydantic_core import core_schema

from dimos.protocol.robot.model import RobotModel


@dataclass(frozen=True)
class Named:
    choices: Mapping[str, RobotModel]

    def __get_pydantic_core_schema__(
        self, source: Any, handler: GetCoreSchemaHandler
    ) -> core_schema.CoreSchema:
        raise NotImplementedError
