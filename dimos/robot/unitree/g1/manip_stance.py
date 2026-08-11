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

"""Where the G1 stands to pour into a pot, and whether it is there yet.

``artifacts/right_arm_pour_reach.json`` holds the offline answer: which
base-frame pot offsets the right arm can pour into, gridded (see
``tool_pour_reach_map``). Everything here is a lookup on that grid plus
planar geometry -- no solving at runtime.

Two questions get asked. Before walking: "given the pot's world XY, where
should I stand?" (:func:`select_stance`). While walking: "is the pot inside
the reachable region yet?" (:meth:`PourReachMap.contains`), which is the
last-mile servo's stopping test and is deliberately a region, not a point.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy import ndimage

from dimos.utils.data import LfsPath

DEFAULT_MAP_PATH = Path(__file__).resolve().parent / "artifacts" / "right_arm_pour_reach.json"

# Pour height above the floor. The pot's own height is irrelevant: the tool
# pours from a height the arm is comfortable at and the water falls.
POUR_Z = 0.90
# Tipping about the tool's roll axis toward the robot's left; the pitch-axis
# tip stops solving past 60 deg, this one solves at 90.
TIP_RADIANS = -np.pi / 2

_G1_URDF = LfsPath("g1_urdf/g1.urdf")
# Grasp centre of the right hand in the wrist frame, from the reachability
# registry (dimos.manipulation.reachability.robots).
_GRASP_CENTER_IN_WRIST = np.array([0.12, 0.05, 0.0])
_palm_to_tcp_cache: list[np.ndarray] = []


def palm_to_tcp_offset() -> np.ndarray:
    """Grasp centre relative to the palm frame, which is what we plan to.

    The capability map records the grasp centre on ``right_wrist_yaw_link``
    while the planning group tips at ``right_hand_palm_link``; the two are
    9 cm apart, so a stance chosen without this correction is 9 cm wrong.
    """
    if not _palm_to_tcp_cache:
        # Imported here, not at the top: yourdfpy drags in trimesh, and
        # everything else in this module is arithmetic on a loaded grid.
        import yourdfpy  # type: ignore[import-untyped]

        urdf = yourdfpy.URDF.load(str(_G1_URDF), load_meshes=False)
        urdf.update_cfg(np.zeros(len(urdf.actuated_joint_names)))
        wrist_to_palm = np.asarray(
            urdf.get_transform("right_hand_palm_link", "right_wrist_yaw_link")
        )
        # The two frames share an orientation, so the correction is a
        # translation in that shared frame.
        _palm_to_tcp_cache.append(_GRASP_CENTER_IN_WRIST - wrist_to_palm[:3, 3])
    return _palm_to_tcp_cache[0]


def palm_to_capability_tcp(
    position: np.ndarray, rotation: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a palm pose into the TCP pose the capability map indexes."""
    return position + rotation @ palm_to_tcp_offset(), rotation


def wrap_angle(angle: float) -> float:
    """Angle folded into [-pi, pi)."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def pot_in_base_frame(
    pot_xy: tuple[float, float], base_xy: tuple[float, float], base_yaw: float
) -> tuple[float, float]:
    """The pot's XY as the robot sees it, standing at ``base_xy``/``base_yaw``."""
    dx = pot_xy[0] - base_xy[0]
    dy = pot_xy[1] - base_xy[1]
    cos_yaw, sin_yaw = np.cos(base_yaw), np.sin(base_yaw)
    return float(cos_yaw * dx + sin_yaw * dy), float(-sin_yaw * dx + cos_yaw * dy)


@dataclass(frozen=True)
class Stance:
    """Where to stand, and where the pot lands in the base frame once there."""

    x: float
    y: float
    yaw: float
    offset: tuple[float, float]
    margin_cells: int

    @property
    def xy(self) -> tuple[float, float]:
        return self.x, self.y


class PourReachMap:
    """The gridded answer to "can the right arm pour into a pot there?"."""

    def __init__(self, data: dict[str, Any]) -> None:
        self.pour_z = float(data["pour_z"])
        self.tip_radians = float(data["tip_radians"])
        self.cell = float(data["cell"])
        self.x0 = float(data["x0"])
        self.y0 = float(data["y0"])
        upright = np.asarray(data["ik_upright"], dtype=bool)
        tipped = np.asarray(data["ik_tipped"], dtype=bool)
        if upright.shape != tipped.shape or upright.ndim != 2 or not upright.size:
            raise ValueError(f"reach map grids are not a matching 2D pair: {upright.shape}")
        # A map sampled for a different pour would send the robot to a stance
        # that cannot hold the pose the demo actually commands, and it would
        # only show up as an unexplained planning failure after the walk.
        if not np.isclose(self.pour_z, POUR_Z) or not np.isclose(self.tip_radians, TIP_RADIANS):
            raise ValueError(
                f"reach map is stale: sampled for pour z={self.pour_z} tip={self.tip_radians} rad, "
                f"but the demo pours at z={POUR_Z} tip={TIP_RADIANS} rad. "
                "Regenerate it with dimos.robot.unitree.g1.tool_pour_reach_map."
            )
        # Both pour poses have to solve from a stance, so the region the demo
        # can use is the intersection, not either one alone.
        self.reachable = upright & tipped
        self._distance = _interior_distance(self.reachable)

    @classmethod
    def load(cls, path: Path | None = None) -> PourReachMap:
        target = path or DEFAULT_MAP_PATH
        return cls(json.loads(target.read_text()))

    def _indices(self, offset: tuple[float, float]) -> tuple[int, int]:
        ix = round((offset[0] - self.x0) / self.cell)
        iy = round((offset[1] - self.y0) / self.cell)
        return iy, ix

    def margin(self, offset: tuple[float, float]) -> int:
        """How many cells the offset sits inside the reachable region.

        0 means outside it, 1 means on its edge. The servo aims for more
        than 1 so that the robot's own stopping error stays inside.
        """
        iy, ix = self._indices(offset)
        if not (0 <= iy < self.reachable.shape[0] and 0 <= ix < self.reachable.shape[1]):
            return 0
        return int(self._distance[iy, ix])

    def contains(self, offset: tuple[float, float], margin_cells: int = 2) -> bool:
        """Is the pot reachable from here, with room to spare?"""
        return self.margin(offset) >= margin_cells

    def best_offset(
        self, margin_cells: int = 2, max_bearing: float = np.deg2rad(35.0)
    ) -> tuple[float, float]:
        """The offset to aim the robot at.

        Deep inside the region, because the aim point has to absorb however
        far the robot overshoots when it stops. The region's deepest cells
        sit ~47 deg off the robot's nose (a right arm reaches right), which
        is a stance that pours well but reads as if the robot is ignoring
        the pot, so the search is capped to a cone that still faces it.
        """
        candidates = np.argwhere(self._distance >= margin_cells)
        if not len(candidates):
            raise ValueError(f"reach map has no cell {margin_cells} cells inside the region")
        best, best_key = None, None
        for iy, ix in candidates:
            offset = (self.x0 + ix * self.cell, self.y0 + iy * self.cell)
            bearing = abs(np.arctan2(offset[1], offset[0]))
            if bearing > max_bearing:
                continue
            key = (-int(self._distance[iy, ix]), bearing, float(np.hypot(*offset)))
            if best_key is None or key < best_key:
                best, best_key = offset, key
        if best is None:
            raise ValueError(
                f"reach map has no cell {margin_cells} cells inside the region within "
                f"{np.rad2deg(max_bearing):.0f} deg of straight ahead"
            )
        return best


def _interior_distance(reachable: np.ndarray) -> np.ndarray:
    """Cells to the nearest unreachable cell (Chebyshev), 0 where unreachable."""
    distance = np.zeros(reachable.shape, dtype=int)
    eroded = reachable.copy()
    step = 0
    while eroded.any():
        step += 1
        distance[eroded] = step
        eroded = ndimage.binary_erosion(eroded)
    return distance


def select_stance(
    pot_xy: tuple[float, float],
    approach_yaw: float,
    reach_map: PourReachMap | None = None,
    margin_cells: int = 3,
) -> Stance:
    """Where to stand to pour into a pot at ``pot_xy``.

    ``approach_yaw`` is the heading the robot ends up facing -- normally the
    direction it already sees the pot from, so the last-mile walk is a nudge
    rather than a lap around the table. The stance places the pot at the
    map's best offset in the base frame, which is what makes it reachable.
    """
    reach = reach_map or PourReachMap.load()
    offset = reach.best_offset(margin_cells)
    cos_yaw, sin_yaw = np.cos(approach_yaw), np.sin(approach_yaw)
    return Stance(
        x=float(pot_xy[0] - (cos_yaw * offset[0] - sin_yaw * offset[1])),
        y=float(pot_xy[1] - (sin_yaw * offset[0] + cos_yaw * offset[1])),
        yaw=wrap_angle(approach_yaw),
        offset=offset,
        margin_cells=reach.margin(offset),
    )


def tool_yaw_for(offset: tuple[float, float], base_yaw: float) -> float:
    """World yaw to command the tool with: pointing at the pot, as the map
    was sampled."""
    return wrap_angle(base_yaw + float(np.arctan2(offset[1], offset[0])))
