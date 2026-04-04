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

"""Agent skill: text query spatial memory to navigate with position target."""

from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.robot.drone.specs import DroneGoToPositionSpec, SpatialMemoryQuerySpec


def _world_pose_to_local_ned(pos_x: float, pos_y: float, pos_z: float) -> tuple[float, float, float]:
    """Map world pose (same as mavlink integrated pose / TF) to MAV_FRAME_LOCAL_NED.

    Matches dimos/robot/drone/mavlink_connection.py: internal x=North, y=-East, z=up.
    """
    ned_x = pos_x
    ned_y = -pos_y
    ned_z = -pos_z
    return ned_x, ned_y, ned_z


def _metadata_dict(raw: Any) -> dict[str, Any] | None:
    if raw is None:
        return None
    if isinstance(raw, list) and raw:
        raw = raw[0]
    if isinstance(raw, dict):
        return raw
    return None


def _extract_xyz(metadata: dict[str, Any]) -> tuple[float, float, float] | None:
    if "pos_x" in metadata and "pos_y" in metadata and "pos_z" in metadata:
        return (
            float(metadata["pos_x"]),
            float(metadata["pos_y"]),
            float(metadata["pos_z"]),
        )
    return None


class DroneSpatialNavSkill(Module):
    """Bridges SpatialMemory text search and drone position commands."""

    _memory: SpatialMemoryQuerySpec
    _drone: DroneGoToPositionSpec

    @rpc
    def start(self) -> None:
        super().start()

    @skill
    def navigate_to_where_i_saw(
        self, description: str, match_index: int = 0, limit: int = 5
    ) -> str:
        """Fly toward a place remembered from vision using a text description.

        Queries spatial memory (CLIP) for frames matching the description, takes the
        stored robot pose for the chosen match, converts it to local NED, and sends a
        position target to the drone.

        Args:
            description: What to look for in stored views (e.g. "red building", "runway").
            match_index: Which result to use (0 = best match, 1 = second, etc.).
            limit: How many spatial-memory candidates to retrieve.

        Returns:
            Result message for the agent.
        """
        results = self._memory.query_by_text(description.strip(), limit=limit)
        if not results:
            stats = self._memory.get_stats()
            frame_count = stats.get("frame_count", 0)
            stored = stats.get("stored_frame_count", 0)
            return (
                f"No spatial memory matches for: {description!r}. "
                f"Memory stats: frame_count={frame_count}, stored_frame_count={stored}. "
                "If stored_frame_count is 0, move the drone so camera+TF samples are recorded."
            )

        if match_index < 0 or match_index >= len(results):
            return (
                f"match_index {match_index} out of range; query returned {len(results)} result(s) (0..{len(results) - 1})."
            )

        entry = results[match_index]
        meta_raw = entry.get("metadata", {})
        meta = _metadata_dict(meta_raw)
        if meta is None:
            return "Spatial memory entry has no usable metadata."

        xyz = _extract_xyz(meta)
        if xyz is None:
            return (
                "Could not read pos_x/pos_y/pos_z from spatial memory metadata for this match."
            )

        ned = _world_pose_to_local_ned(*xyz)
        ok = self._drone.go_to_position(ned[0], ned[1], ned[2], 0.0, 0.0, 0.0)
        if ok:
            return (
                f"Sent position target toward spatial memory match {match_index} for {description!r}: "
                f"NED ({ned[0]:.2f}, {ned[1]:.2f}, {ned[2]:.2f}) m."
            )
        return "Failed to send position target (no connection or FC rejected)."


__all__ = ["DroneSpatialNavSkill"]
