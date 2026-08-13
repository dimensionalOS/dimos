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

"""Generic configuration envelope for coordinator-owned control tasks."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class TaskConfig:
    """Configuration for a registered control task."""

    name: str
    type: str = "trajectory"
    joint_names: list[str] = field(default_factory=list)
    priority: int = 10
    auto_start: bool = False
    params: dict[str, Any] = field(default_factory=dict)
    stream_bind: dict[str, str] = field(default_factory=dict)
