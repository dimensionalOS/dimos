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


"""The body under test: what the planner plans for and the judge measures.

Shared domain, not benchmark -- the deployed adapter reads EMBODIMENTS to
configure a live robot, and both referees condition their scoring on it. Pure
geometry and gait-cost numbers, no dependency on worlds or on the sim.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Embodiment:
    """The robot under test — conditions the gold oracle, the generator's
    difficulty rules, and the judge. A net trained on varied embodiments
    deploys on a new robot by being handed a new one of these.

    comfort = obstacles-we-care-about radius (preference, tunable);
    precision = local control tracking accuracy (hard floor — clearance
    below it is fiction, planning it is planning a contact).
    """

    # Moving-body envelope measured in the fitted MuJoCo sim (union of all
    # robot geometry over the full 95-cell command protocol of
    # simulation/envelope.py, yaw-aligned base frame): the swinging legs, not
    # the 0.31 m trunk, set the width. Measured 0.883 x 0.593, centre +0.002.
    # Re-baselined from 0.852 x 0.495 by planner/revision.md: that number came
    # from a smaller command sweep and was NOT conservative for fast strafe or
    # for slow tight arcs. The union's jobs -- judge veto, half_diag, body
    # carve, and the fallback for embodiments with no measured `envelope` --
    # are exactly where honest-conservative is the only acceptable property.
    tag: str = "go2"
    length: float = 0.883
    width: float = 0.593
    center_off: float = 0.002  # body center relative to the pose point
    comfort: float = 0.4
    precision: float = 0.05
    # gait preferences for the planner's cost function.
    # forward = 1; strafe/reverse scale it; yaw_w prices rotation per rad.
    strafe: float = 1.8
    reverse: float = 1.5
    yaw_w: float = 0.25
    # Vertical geometry, all measured from the surface the feet stand on.
    # The base rides a known height above the ground (motion/obstacles.py)
    steppable: float = 0.20  # legs negotiate obstacles below this - at a cost (TODO)
    height: float = 0.45  # above this the body passes underneath; not an obstacle
    base_height: float = 0.29  # base origin above support; frame plumbing, not semantics

    # Motion-conditioned envelope, one row per |drift| angle in degrees:
    # (deg, length, width, off_x, off_y). 0 = nose-first, 90 = strafe,
    # 180 = reverse. Rows sit at the lattice's own drift angles, so
    # nearest-row lookup is exact for every edge the SE(2) search generates.
    # off_y is stored for POSITIVE drift and mirrored by sign at lookup: the
    # swept box lags the drift laterally, and a row that covers +theta covers
    # -theta only when it is mirrored with it. EMPTY = the union
    # length/width/center_off applies at every heading (the fallback for any
    # unmeasured embodiment). See planner/revision.md.
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

    def stand_box(self) -> tuple[float, float, float, float]:
        """The STANDING body: the largest box nested in every envelope row.

        Standing is not the union of the swept walking boxes — it is the static
        body, and every gait's sweep contains it. The rows are intersected in
        BOTH drift signs, exactly as `envelope_at` mirrors them, so the result
        is nested in whatever shape an edge may actually have been cleared by:
        a pose whose row clears the margin clears this too, which is what makes
        replanning from a route this planner emitted unable to refuse. No
        measured envelope means no rows to intersect and the union is all there
        is — nothing changes for those bodies. See planner/revision.md.
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


# Baked by `python -m dimos.navigation.motion.simulation.envelope --bake` over
# the governed slow band (stand + 0.35 + 0.50 m/s); see
# planner/envelope_results.md for the surface these fold out of.
GO2_ENVELOPE: tuple[tuple[float, float, float, float, float], ...] = (
    (0.0, 0.819, 0.416, -0.023, 0.000),
    (26.6, 0.802, 0.436, -0.032, -0.008),
    (45.0, 0.788, 0.472, -0.035, -0.018),
    (63.4, 0.781, 0.500, -0.039, -0.016),
    (90.0, 0.781, 0.507, -0.039, -0.009),
    (116.6, 0.781, 0.497, -0.039, 0.000),
    (135.0, 0.781, 0.463, -0.039, -0.001),
    (153.4, 0.781, 0.422, -0.039, -0.003),
    (180.0, 0.781, 0.416, -0.039, 0.000),
)

# Measured 0.0334 m of extra width per rad/m, residuals <= 12 mm.
GO2_ARC_INFLATE = 0.0334

GO2 = Embodiment(envelope=GO2_ENVELOPE, arc_inflate=GO2_ARC_INFLATE)

EMBODIMENTS = {
    "go2": GO2,
    # payload adds 8 cm in front: longer body, centre 4 cm further forward.
    # No measured envelope of its own: it falls back to the union everywhere.
    "go2-payload": Embodiment(tag="go2-payload", length=0.963, center_off=0.042, comfort=0.5),
    "slim": Embodiment(tag="slim", length=2.0, width=0.24, comfort=0.3),
    # cannot crab, and has no legs to step over anything with
    "diffdrive": Embodiment(tag="diffdrive", strafe=50.0, reverse=3.0, steppable=0.0),
}
