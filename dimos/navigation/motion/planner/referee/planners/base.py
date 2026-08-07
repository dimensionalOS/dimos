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

"""Pluggable planner interface for the motion2 environment.

A candidate is a factory `make(scenario, cfg) -> PlannerEpisode`. The episode
is stateful across plan() calls (warm starts, hysteresis live inside it) and
nothing survives reset(). Honest candidates read only what plan() receives —
cloud, pose, goal; the scenario handed to the factory is for judge-side
references (gold) and world-free setup, not for peeking at truth.
"""

from __future__ import annotations

from collections.abc import Callable
from importlib import import_module
from typing import TYPE_CHECKING, Any, Protocol

if TYPE_CHECKING:
    from ..types import Path, PointCloud2


class PlannerEpisode(Protocol):
    def reset(self) -> None: ...

    def plan(
        self, cloud: PointCloud2, pose: tuple[float, float, float], goal: tuple[float, float]
    ) -> Path: ...


PlannerFactory = Callable[..., PlannerEpisode]

# name -> "module:factory"; arbitrary "module:factory" strings load too, so
# generated candidates plug in without registering. Registry entries are
# package-relative so the package works wherever it is copied.
REGISTRY = {
    "gold": ".gold:make",  # judge-side reference
    "target-py": ".target:make_py",  # port spec (python)
    "target": ".target:make",  # rust candidate
}


def load(name: str) -> PlannerFactory:
    """Resolve a registry name or a dotted "module:factory" string."""
    target = REGISTRY.get(name, name)
    mod, _, attr = target.partition(":")
    module = import_module(mod, package=__package__) if mod.startswith(".") else import_module(mod)
    factory: Any = getattr(module, attr or "make")
    return factory  # type: ignore[no-any-return]
