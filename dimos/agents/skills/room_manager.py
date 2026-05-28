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

"""Per-room map selection for the Go2.

A thin agent-facing layer over RelocalizationModule: keep a registry of named
rooms → premap files, and let the robot be TOLD which room it's in
(`set_room`) or GUESS it (`guess_room`, by scan-matching the live lidar against
each candidate map and picking the best fit). Loading a room's map makes
RelocalizationModule relocalize into that map's saved frame, so every run in a
room shares one coordinate frame (instead of a fresh origin per boot).
"""

from pathlib import Path
from typing import Any

import yaml

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.mapping.relocalization.relocalization_spec import RelocalizationSpec
from dimos.utils.data import get_data_dir
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def load_room_registry(registry_file: str) -> dict[str, dict[str, Any]]:
    """Read the room→map registry YAML. Returns {room_name: {map, marker_id?}}."""
    path = Path(registry_file)
    if not path.is_absolute() and not path.exists():
        path = get_data_dir() / registry_file
    if not path.exists():
        logger.warning("room registry not found: %s", path)
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    rooms = data.get("rooms", {}) if isinstance(data, dict) else {}
    return {str(name): dict(cfg or {}) for name, cfg in rooms.items()}


class RoomManagerConfig(ModuleConfig):
    registry_file: str = "rooms.yaml"  # resolved in the data/ dir if not a path
    fitness_threshold: float = 0.45  # min scan-match fitness to accept a guess
    default_room: str | None = None  # if set, load this room's map on start


class RoomManager(Module):
    config: RoomManagerConfig
    # Structural ref → RelocalizationModule (load_map / score_map / current_map).
    _relocalization: RelocalizationSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._rooms: dict[str, dict[str, Any]] = {}

    @rpc
    def start(self) -> None:
        super().start()
        self._rooms = load_room_registry(self.config.registry_file)
        logger.info("RoomManager: %d rooms registered: %s", len(self._rooms), list(self._rooms))
        if self.config.default_room:
            self._load_room(self.config.default_room)

    def _load_room(self, room: str) -> bool:
        cfg = self._rooms.get(room)
        if not cfg or "map" not in cfg:
            return False
        return bool(self._relocalization.load_map(cfg["map"]))

    def _room_for_map(self, map_name: str | None) -> str | None:
        if map_name is None:
            return None
        for room, cfg in self._rooms.items():
            if cfg.get("map") == map_name:
                return room
        return None

    @skill
    def set_room(self, room: str) -> SkillResult:
        """Tell the robot which room it is in and load that room's saved map.

        Use when the user says where the robot is (e.g. "you're in the kitchen").
        Loads the room's pre-built map so the robot relocalizes into its saved
        coordinate frame. `room` must be one of the registered rooms.
        """
        if room not in self._rooms:
            return SkillResult.fail(
                "UNKNOWN_ROOM",
                f"No map for '{room}'. Known rooms: {', '.join(self._rooms) or '(none)'}",
            )
        if not self._load_room(room):
            return SkillResult.fail("LOAD_FAILED", f"Failed to load the map for '{room}'")
        return SkillResult.ok(f"Loaded the map for '{room}'; relocalizing into it.")

    @skill
    def guess_room(self) -> SkillResult:
        """Figure out which room the robot is in from what its lidar sees.

        Use when the user asks the robot where it is / to find/recognize its room
        without telling it. Scan-matches the current lidar against every known
        room map and loads the best match. The robot should have been powered on
        for a few seconds (and ideally looked around) so it has enough scan data.
        """
        if not self._rooms:
            return SkillResult.fail("NO_ROOMS", "No rooms are registered")

        scores: dict[str, float] = {}
        for room, cfg in self._rooms.items():
            map_name = cfg.get("map")
            if not map_name:
                continue
            scores[room] = self._relocalization.score_map(map_name)

        if not scores:
            return SkillResult.fail("NO_MAPS", "No room maps to match against")

        best_room = max(scores, key=lambda r: scores[r])
        best_score = scores[best_room]
        ranked = ", ".join(f"{r}={scores[r]:.2f}" for r in sorted(scores, key=lambda r: -scores[r]))
        if best_score < self.config.fitness_threshold:
            return SkillResult.fail(
                "NO_MATCH",
                f"Couldn't confidently recognize the room (best {best_room}={best_score:.2f}, "
                f"need ≥{self.config.fitness_threshold}). Scores: {ranked}",
            )
        if not self._load_room(best_room):
            return SkillResult.fail("LOAD_FAILED", f"Failed to load the map for '{best_room}'")
        return SkillResult.ok(
            f"This looks like the {best_room} (fit {best_score:.2f}); loaded its map. "
            f"Scores: {ranked}",
        )

    @skill
    def where_am_i(self) -> SkillResult:
        """Report which room map is currently loaded, if any.

        Use when the user asks which room the robot thinks it's in right now.
        """
        room = self._room_for_map(self._relocalization.current_map())
        if room is None:
            return SkillResult.ok("No room map is loaded yet.")
        return SkillResult.ok(f"Currently localized in the {room}.")

    @skill
    def list_rooms(self) -> SkillResult:
        """List the rooms the robot has saved maps for.

        Use when the user asks which rooms/maps are available.
        """
        if not self._rooms:
            return SkillResult.ok("No rooms are registered yet.")
        return SkillResult.ok("Known rooms: " + ", ".join(self._rooms))
