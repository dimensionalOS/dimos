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


"""The body: what the planner plans for and the follower drives.

Pure geometry, gait plant and cost numbers; no dependency on worlds or modules.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

import numpy as np
from numpy.typing import NDArray
from pydantic import TypeAdapter

from dimos.navigation.trajectory_follower.fancy.controller import ControllerConfig


@dataclass(frozen=True)
class Embodiment:
    """One robot's measured and fitted numbers; a new robot is a new one of these.

    Nothing measured has a default: a body states every number (`embodiment/go2.py::GO2`)
    or is `replace(GO2, ...)` of one that does.
    """

    # Moving-body envelope, measured, in the yaw-aligned base frame; the fallback where
    # `envelope` has no row.
    length: float
    width: float
    comfort: float  # obstacles-we-care-about radius, tunable
    precision: float  # tracking accuracy; clearance below it is fiction
    # The governor curve, a wire contract between planner and follower (control/profile.py),
    # so it is the body's and not either module's config.
    max_speed: float
    min_speed: float
    speed_clearance: float
    max_yaw_rate: float
    # The gait plant, measured: ground speed ~= walk_gain * cmd - walk_slip above the stall
    # band; below walk_slip_ramp the inverse fades to identity so a stop stays a stop.
    command_slew: tuple[float, float, float]
    gait_band: tuple[float, float]
    walk_gain: float
    walk_slip: float
    walk_slip_ramp: float
    # Cost preferences: forward = 1, strafe/reverse scale it, yaw_w prices rotation per rad.
    strafe: float
    reverse: float
    yaw_w: float
    # Vertical geometry, measured from the surface the feet stand on.
    steppable: float
    height: float
    base_height: float
    # Fitted, where everything above is measured.
    control: ControllerConfig = field(hash=False)
    center_off: float = 0.0  # body center relative to the pose point
    # Per-|drift| rows (deg, length, width, off_x, off_y) at the lattice's own drift angles;
    # off_y is stored for positive drift and mirrored by sign at lookup. Empty = union everywhere.
    envelope: tuple[tuple[float, float, float, float, float], ...] = ()
    # Extra swept width per rad-per-metre of curvature, so it survives a lattice pitch change.
    arc_inflate: float = 0.0

    @property
    def half_diag(self) -> float:
        return math.hypot(self.length, self.width) / 2.0

    def envelope_at(self, drift: float) -> tuple[float, float, float, float]:
        """(length, width, off_x, off_y) for a body-frame drift angle in rad."""
        if not self.envelope:
            return self.length, self.width, self.center_off, 0.0
        rel = math.remainder(drift, 2.0 * math.pi)
        deg = math.degrees(abs(rel))
        row = min(self.envelope, key=lambda r: abs(r[0] - deg))
        return row[1], row[2], row[3], row[4] if rel >= 0.0 else -row[4]

    def to_json(self, indent: int | None = None) -> str:
        """The body as every rust crate reads it."""
        return TypeAdapter(Embodiment).dump_json(self, indent=indent).decode()

    def box(self, drift: float | None) -> tuple[float, float, float, float]:
        """The swept box a heading needs; ``None`` asks for the all-gait union."""
        if drift is None:
            return self.length, self.width, self.center_off, 0.0
        return self.envelope_at(drift)

    def dilated(self, by: float = 0.0, precision: float | None = None) -> Embodiment:
        """This body with every box grown by `by` per side (negative shrinks) and an optional
        clearance floor."""
        pad = 2.0 * by
        rows = tuple((a, ln + pad, w + pad, ox, oy) for a, ln, w, ox, oy in self.envelope)
        return replace(
            self,
            length=self.length + pad,
            width=self.width + pad,
            envelope=rows,
            precision=self.precision if precision is None else precision,
        )

    def stand_box(self) -> tuple[float, float, float, float]:
        """The standing body: the largest box nested in every envelope row, so a replan from
        this planner's own route cannot refuse."""
        if not self.envelope:
            return self.length, self.width, self.center_off, 0.0
        lo = max(r[3] - r[1] / 2.0 for r in self.envelope)
        hi = min(r[3] + r[1] / 2.0 for r in self.envelope)
        # Mirroring folds a row's y interval onto |off_y| .. w/2 - |off_y|.
        half_w = min(r[2] / 2.0 - abs(r[4]) for r in self.envelope)
        return hi - lo, 2.0 * half_w, (lo + hi) / 2.0, 0.0

    def offsets(self, step: float = 0.05, drift: float | None = None) -> NDArray[np.float64]:
        """Footprint sample points; ``drift`` None asks for the all-gait union."""
        return box_offsets(self.box(drift), step)


def box_offsets(box: tuple[float, float, float, float], step: float = 0.05) -> NDArray[np.float64]:
    """Footprint sample points of one swept box `(length, width, off_x, off_y)`."""
    length, width, off_x, off_y = box
    hl, hw = length / 2.0, width / 2.0
    return np.array(
        [
            (x + off_x, y + off_y)
            for x in np.arange(-hl, hl + step / 2.0, step)
            for y in np.arange(-hw, hw + step / 2.0, step)
        ]
    )
