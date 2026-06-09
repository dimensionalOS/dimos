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

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from types import ModuleType


class RoboPlanBackendUnavailableError(ImportError):
    pass


@dataclass(frozen=True)
class RoboPlanComponents:
    core: ModuleType
    rrt: ModuleType
    simple_ik: ModuleType
    toppra: ModuleType | None = None


def load_roboplan_components(*, require_toppra: bool = False) -> RoboPlanComponents:
    core = _import_roboplan_module("roboplan.core")
    rrt = _import_roboplan_module("roboplan.rrt")
    simple_ik = _import_roboplan_module("roboplan.simple_ik")
    toppra = _import_roboplan_module("roboplan.toppra") if require_toppra else None
    return RoboPlanComponents(core=core, rrt=rrt, simple_ik=simple_ik, toppra=toppra)


def probe_optional_toppra() -> ModuleType | None:
    try:
        return import_module("roboplan.toppra")
    except (ImportError, ModuleNotFoundError):
        return None


def _import_roboplan_module(module_name: str) -> ModuleType:
    try:
        return import_module(module_name)
    except (ImportError, ModuleNotFoundError) as exc:
        missing = getattr(exc, "name", None) or module_name
        raise RoboPlanBackendUnavailableError(
            "RoboPlan planning backend was selected, but required module "
            f"'{module_name}' could not be imported (missing: '{missing}'). "
            "Install RoboPlan Python bindings and verify with: "
            'python -c "import roboplan.core, roboplan.rrt, roboplan.simple_ik"'
        ) from exc
