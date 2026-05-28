#!/usr/bin/env python3
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

"""All-Gemini agentic Go2 with multi-room maps + relocalization.

Extends `unitree_go2_agentic_gemini` with:
  - RelocalizationModule: loads a named premap, scan-matches the live lidar into
    it (FPFH+ICP), publishes a world→map TF, and emits `merged_map` — which the
    existing CostMapper already prefers, so the costmap + explorer/planner run in
    the saved map's frame. Started "mapless"; a map is loaded at runtime.
  - RoomManager: agent skills to be TOLD the room (`set_room`) or GUESS it
    (`guess_room`, by scoring the live scan against each registered room map).
  - MapSaverSkill: `save_map` to persist the map built in a live session to a
    reusable .pc2.lcm (the live-session counterpart to `dimos export-premap`),
    plus `list_maps`. Lets a room map be created on-device, then registered in
    rooms.yaml for later relocalization.

So every run in a room shares that room's coordinate frame instead of a fresh
origin per boot. Per-room maps are built once (with `dimos export-premap` or the
`save_map` skill) and listed in `data/rooms.yaml`.
"""

from dimos.agents.skills.map_saver_skill import MapSaverSkill
from dimos.agents.skills.room_manager import RoomManager
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic_gemini import (
    unitree_go2_agentic_gemini,
)

unitree_go2_agentic_gemini_rooms = autoconnect(
    unitree_go2_agentic_gemini,
    RelocalizationModule.blueprint(),
    RoomManager.blueprint(),
    MapSaverSkill.blueprint(),
).global_config(n_workers=12)

__all__ = ["unitree_go2_agentic_gemini_rooms"]
