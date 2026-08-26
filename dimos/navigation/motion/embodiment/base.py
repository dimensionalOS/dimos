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

Pure geometry, gait plant and cost numbers, no dependency on worlds or on any
module. The deployed adapters are configured with one to configure a live robot.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from dimos.navigation.motion.control.controller import ControllerConfig


@dataclass(frozen=True)
class Embodiment:
    """One robot's measured and fitted numbers; a new robot is a new one of these.

    Nothing measured has a default: a body states every number
    (`embodiment/go2.py::GO2`) or is `replace(GO2, ...)` of one that does.
    comfort = obstacles-we-care-about radius (preference, tunable);
    precision = local control tracking accuracy (hard floor -- clearance
    below it is fiction, planning it is planning a contact).
    """

    tag: str
    # Moving-body envelope, measured: the union of all robot geometry over a
    # command sweep, in the yaw-aligned base frame. It is the fallback wherever
    # a body has no per-heading `envelope`, so it has to stay conservative.
    length: float
    width: float
    comfort: float
    precision: float
    # The governor curve: the speed the planner prices a metre of clearance at
    # and the follower reads back out of the path's stamps (control/profile.py).
    # The ramp's floor is `precision`. It is a wire contract between the two
    # modules, so it is the body's and not either module's config.
    max_speed: float
    min_speed: float
    speed_clearance: float
    max_yaw_rate: float
    # The gait plant, measured: how the walking policy answers a command.
    # ground speed ~= walk_gain * cmd - walk_slip above the stall band; the laws
    # invert it so a request is the speed the governor chose. Below
    # walk_slip_ramp the inverse fades to identity: a stop stays a stop.
    command_slew: tuple[float, float, float]
    gait_band: tuple[float, float]
    walk_gain: float
    walk_slip: float
    walk_slip_ramp: float
    # gait preferences for the planner's cost function.
    # forward = 1; strafe/reverse scale it; yaw_w prices rotation per rad.
    strafe: float
    reverse: float
    yaw_w: float
    # Vertical geometry, all measured from the surface the feet stand on.
    # The base rides a known height above the ground (motion/obstacles.py)
    steppable: float
    height: float
    base_height: float
    # The follower tuning searched on this body -- fitted, where everything
    # above is measured. Nested so the line between the two stays visible.
    control: ControllerConfig
    center_off: float = 0.0  # body center relative to the pose point
    # Motion-conditioned envelope, one row per |drift| angle in degrees:
    # (deg, length, width, off_x, off_y). 0 = nose-first, 90 = strafe,
    # 180 = reverse. Rows sit at the lattice's own drift angles, so
    # nearest-row lookup is exact for every edge the SE(2) search generates.
    # off_y is stored for POSITIVE drift and mirrored by sign at lookup: the
    # swept box lags the drift laterally, and a row that covers +theta covers
    # -theta only when it is mirrored with it. EMPTY = the union applies at
    # every heading.
    envelope: tuple[tuple[float, float, float, float, float], ...] = ()
    # Extra swept WIDTH per rad-per-metre of curvature (edge dyaw / edge
    # length). Curvature, not per-edge yaw, so the number survives a lattice
    # pitch change unmeasured.
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

    def box(self, drift: float | None) -> tuple[float, float, float, float]:
        """The swept box a heading needs; ``None`` asks for the all-gait union."""
        if drift is None:
            return self.length, self.width, self.center_off, 0.0
        return self.envelope_at(drift)

    def dilated(self, by: float = 0.0, precision: float | None = None) -> Embodiment:
        """This body with every box grown by `by` PER SIDE, and an optional
        clearance floor.

        Negative shrinks it. The table's own numbers are measured -- the union
        and the per-heading rows are where the legs actually swing -- so this is
        the one place a deployment says "plan me tighter than measured" and owns
        the consequence. `precision` is the hard fits/does-not-fit margin the
        search tests against, so a gap has to be `width + 2 * precision` wide
        before a route through it exists at all.
        """
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
        """The STANDING body: the largest box nested in every envelope row.

        Standing is not the union of the swept walking boxes — it is the static
        body, and every gait's sweep contains it. The rows are intersected in
        BOTH drift signs, exactly as `envelope_at` mirrors them, so the result
        is nested in whatever shape an edge may actually have been cleared by:
        a pose whose row clears the margin clears this too, which is what makes
        replanning from a route this planner emitted unable to refuse. No
        measured envelope means no rows to intersect and the union is all there
        is — nothing changes for those bodies.
        """
        if not self.envelope:
            return self.length, self.width, self.center_off, 0.0
        lo = max(r[3] - r[1] / 2.0 for r in self.envelope)
        hi = min(r[3] + r[1] / 2.0 for r in self.envelope)
        # Mirroring folds a row's y interval onto |off_y| .. w/2 - |off_y|.
        half_w = min(r[2] / 2.0 - abs(r[4]) for r in self.envelope)
        return hi - lo, 2.0 * half_w, (lo + hi) / 2.0, 0.0

    def offsets(self, step: float = 0.05, drift: float | None = None) -> np.ndarray:
        """Footprint sample points, dense enough that thin slats can't slip.

        ``drift`` None asks for the all-gait union; a body-frame drift angle
        in rad asks for the swept box that heading actually needs.
        """
        return box_offsets(self.box(drift), step)


def box_offsets(box: tuple[float, float, float, float], step: float = 0.05) -> np.ndarray:
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
