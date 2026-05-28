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

"""Save the map built during a live session to disk so it can be reused later.

Skills:
  - save_map:  snapshot the map the robot has built so far and write it to a
    reusable ``.pc2.lcm`` premap file.
  - list_maps: list the maps already saved on disk.

This is the live-session counterpart to ``dimos export-premap`` (which builds a
premap offline from a recording): it captures the ``global_map`` point cloud
that ``VoxelGridMapper`` already publishes (the accumulated world-frame map that
also feeds the costmap and explorer) and encodes it with the same LCM wire
format that :class:`~dimos.mapping.relocalization.module.RelocalizationModule`
loads. So a map saved here is byte-compatible with an export-premap output.

To reuse a saved map, register it in ``data/rooms.yaml`` and load it with the
RoomManager skills (``set_room`` / ``guess_room``), or pass it directly via
``relocalizationmodule.map_file=<name>`` at launch.

NOTE: this is a live snapshot (no pose-graph loop-closure correction). For the
highest-quality map over large loops, use the offline ``dimos export-premap``
pipeline instead.
"""

from pathlib import Path
import re

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data_dir
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Same suffix RelocalizationModule expects (see relocalization/module.py MAP_SUFFIX).
_MAP_SUFFIX = ".pc2.lcm"
# Map names become filenames and a relocalization config value, so keep them to
# a safe, shell/path-friendly alphabet.
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
# Below this the relocalizer can't lock on (see RelocalizationModule.MIN_LOCAL_POINTS).
_MIN_USEFUL_POINTS = 50_000


class MapSaverSkillConfig(ModuleConfig):
    # Where to write/look for maps. Empty -> the shared dimos data dir, which is
    # also where `dimos export-premap` writes and where `map_file=<name>` /
    # rooms.yaml entries resolve names from.
    map_dir: str = ""


class MapSaverSkill(Module):
    config: MapSaverSkillConfig
    # Auto-wired to VoxelGridMapper.global_map (the accumulated world-frame map).
    global_map: In[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self._latest: PointCloud2 | None = None
        self.global_map.subscribe(self._on_map)

    def _on_map(self, cloud: PointCloud2) -> None:
        self._latest = cloud

    def _maps_dir(self) -> Path:
        return Path(self.config.map_dir) if self.config.map_dir else get_data_dir()

    @skill
    def save_map(self, name: str) -> SkillResult:
        """Save the map built so far to disk under `name` so it can be reused later.

        Use when the user asks to save / store / keep the map (or the area
        scanned) so it can be loaded again instead of re-exploring. `name` is a
        short label (letters, digits, '_' or '-'), e.g. "office" or
        "ground_floor". Register the saved map in rooms.yaml (or pass it via
        relocalizationmodule.map_file) to relocalize into it on a later run.
        """
        if not _NAME_RE.match(name):
            return SkillResult.fail(
                "BAD_NAME",
                "Map name must be letters/digits/_/- only (e.g. 'office'), no spaces or slashes.",
            )

        cloud = getattr(self, "_latest", None)
        if cloud is None:
            return SkillResult.fail(
                "NO_MAP",
                "No map has been built yet — let the robot map/explore the area first.",
            )

        n_points = len(cloud)
        out = self._maps_dir() / f"{name}{_MAP_SUFFIX}"
        try:
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_bytes(cloud.lcm_encode())
        except Exception as e:
            logger.exception("save_map failed")
            return SkillResult.fail("WRITE_FAILED", f"Could not write map: {e}")

        logger.info("save_map wrote %d points to %s", n_points, out)
        msg = f"Saved map '{name}' ({n_points} points) to {out}."
        if n_points < _MIN_USEFUL_POINTS:
            msg += (
                f" Warning: only {n_points} points — that may be too sparse to "
                "relocalize against; map more of the area before relying on it."
            )
        return SkillResult.ok(msg)

    @skill
    def list_maps(self) -> SkillResult:
        """List the maps that have been saved to disk."""
        d = self._maps_dir()
        if not d.exists():
            return SkillResult.ok("No saved maps yet.")
        names = sorted(p.name[: -len(_MAP_SUFFIX)] for p in d.glob(f"*{_MAP_SUFFIX}"))
        if not names:
            return SkillResult.ok("No saved maps yet.")
        return SkillResult.ok("Saved maps: " + ", ".join(names))
