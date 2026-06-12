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

"""Lazy RoboPlan imports for the optional manipulation backend."""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from types import ModuleType

ROBOPLAN_INSTALL_HINT = (
    "RoboPlan is not installed. Install the optional backend with `uv sync --extra roboplan` "
    "or install the RoboPlan Git packages documented for the manipulation planning backend."
)


class RoboPlanImportError(ImportError):
    """Raised when optional RoboPlan packages are requested but unavailable."""


@dataclass(frozen=True)
class RoboPlanModules:
    """Namespace-package modules used by the RoboPlan backend."""

    core: ModuleType
    rrt: ModuleType | None = None
    simple_ik: ModuleType | None = None
    toppra: ModuleType | None = None
    optimal_ik: ModuleType | None = None
    example_models: ModuleType | None = None


def import_roboplan_core() -> ModuleType:
    """Import `roboplan.core` with an actionable optional-extra hint."""
    try:
        return import_module("roboplan.core")
    except ImportError as exc:
        raise RoboPlanImportError(f"{ROBOPLAN_INSTALL_HINT} Missing module: roboplan.core") from exc


def import_roboplan_rrt() -> ModuleType:
    """Import `roboplan.rrt` with an actionable optional-extra hint."""
    try:
        return import_module("roboplan.rrt")
    except ImportError as exc:
        raise RoboPlanImportError(f"{ROBOPLAN_INSTALL_HINT} Missing module: roboplan.rrt") from exc


def import_roboplan_modules(include_planner: bool = False) -> RoboPlanModules:
    """Import the RoboPlan modules needed for a requested backend path.

    Args:
        include_planner: Also require native RRT planner bindings.
    """
    core = import_roboplan_core()
    rrt = import_roboplan_rrt() if include_planner else None
    return RoboPlanModules(core=core, rrt=rrt)
