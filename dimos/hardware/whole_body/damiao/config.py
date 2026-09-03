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

from typing import Annotated

from pydantic import ConfigDict, Field
from pydantic.dataclasses import dataclass as pydantic_dataclass

_NonEmptyString = Annotated[str, Field(min_length=1, strict=True)]


@pydantic_dataclass(
    frozen=True,
    config=ConfigDict(extra="forbid", validate_default=True),
)
class DamiaoRuntimeConfig:
    """Deployment values that may vary without changing robot topology."""

    bus_devices: dict[_NonEmptyString, _NonEmptyString] = Field(default_factory=dict)
    gravity_comp: bool = Field(default=True, strict=True)
    passive_grippers: tuple[_NonEmptyString, ...] = ()
    tick_deadline_us: int = Field(default=1_000, ge=1, strict=True)
