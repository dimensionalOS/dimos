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

"""A navmesh for scenes that ship none (HSSD), computed once and cached.

Runs under python 3.9 in the Habitat env: no dimos imports, loaded by path from
``server.py`` and ``misc/habitat/nav_cases.py``.
"""

import importlib
from pathlib import Path
from typing import Any

# Go2-ish: half-width plus margin, standing height, one step.
AGENT_RADIUS_M = 0.3
AGENT_HEIGHT_M = 0.5
AGENT_MAX_CLIMB_M = 0.16


def ensure_navmesh(sim: Any, scene_id: str, cache_dir: Path) -> str:
    """Make ``sim.pathfinder`` usable; returns where the navmesh came from."""
    habitat_sim = importlib.import_module("habitat_sim")  # python 3.9 env only

    if sim.pathfinder.is_loaded:
        return "dataset"
    cached = Path(cache_dir) / f"{scene_id.replace('/', '_')}.navmesh"
    if cached.is_file() and sim.pathfinder.load_nav_mesh(str(cached)):
        return str(cached)
    settings = habitat_sim.NavMeshSettings()
    settings.set_defaults()
    settings.agent_radius = AGENT_RADIUS_M
    settings.agent_height = AGENT_HEIGHT_M
    settings.agent_max_climb = AGENT_MAX_CLIMB_M
    settings.include_static_objects = True
    if not sim.recompute_navmesh(sim.pathfinder, settings):
        raise RuntimeError(f"navmesh computation failed for {scene_id}")
    cached.parent.mkdir(parents=True, exist_ok=True)
    sim.pathfinder.save_nav_mesh(str(cached))
    return f"computed -> {cached}"
