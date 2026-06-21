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

"""xArm blueprint compatibility exports."""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS = {
    "coordinator_xarm6": ".basic",
    "coordinator_xarm7": ".basic",
    "dual_xarm6_planner": ".basic",
    "keyboard_teleop_xarm6": ".teleop",
    "keyboard_teleop_xarm7": ".teleop",
    "xarm6_planner_only": ".basic",
    "xarm7_planner_coordinator": ".basic",
    "xarm7_planner_coordinator_agent": ".agentic",
    "xarm_perception": ".perception",
    "xarm_perception_agent": ".agentic",
    "xarm_perception_sim": ".simulation",
    "xarm_perception_sim_agent": ".agentic",
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(_EXPORTS[name], __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted([*globals(), *_EXPORTS])
