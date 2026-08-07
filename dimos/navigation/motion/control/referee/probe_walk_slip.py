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

"""Open-loop calibration of the walking policy's velocity deficit.

Provenance for ``walk_gain``/``walk_slip`` (ControllerConfig; the law that
reads them is ``control/laws/blind.py``). Diagnostic only: not part of the
battery, not part of any gate, and nothing in the scored path imports it.

The follower's twist is a request to a learned gait, and the gait does not
deliver it. This holds a constant body-frame command on a flat, empty world and
differences the executed body pose to recover what the request is actually
worth in metres per second. Run it whenever the policy blob changes -- the
constant in the law is a property of that blob, not of the worlds.

    python -m dimos.navigation.motion.control.referee.probe_walk_slip          # magnitude sweep
    python -m dimos.navigation.motion.control.referee.probe_walk_slip --dirs   # heading + yaw sweep
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.controller import ControllerConfig
from dimos.navigation.motion.control.referee.episode import EpisodeConfig, run_episode
from dimos.navigation.motion.planner.referee.scenarios import GO2, Scenario
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.utils.data import get_data

SETTLE_S = 3.0  # policy warmup + command slew ramp; excluded from the average
HOLD_S = 12.0


class _Const:
    """A follower that ignores the plan and holds one twist."""

    def __init__(self, vx: float, vy: float = 0.0, wz: float = 0.0) -> None:
        self.config = ControllerConfig()
        self._v = (vx, vy, wz)

    def reset(self) -> None:
        pass

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist:
        vx, vy, wz = self._v
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))


def _probe(policy: FreePolicy, vx: float, vy: float = 0.0, wz: float = 0.0) -> tuple[float, float]:
    """Return (achieved ground speed, tilt p99) for one held command."""
    sc = Scenario(
        name="flat",
        boxes=[],
        goal=(50.0, 0.0),  # far enough that the episode never terminates early
        start=(0.0, 0.0, 0.0),
        expect="clear",
        emb=GO2,
        note="walk-slip probe",
    )
    cfg = EpisodeConfig(replan_hz=0.0, planner="target", annotate_clearance=False, timeout=HOLD_S)
    res = run_episode(sc, _Const(vx, vy, wz), policy, cfg)
    keep = res.t >= SETTLE_S
    xy, t = res.pos[keep, :2], res.t[keep]
    # path length, not displacement: with a yaw rate the heading rotates
    dist = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    return dist / (t[-1] - t[0]), float(np.percentile(res.tilt[keep], 99))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dirs", action="store_true", help="sweep heading and yaw rate too")
    args = ap.parse_args()

    policy = FreePolicy.load(get_data("ml-trajectory-research/freewalk_mcf.bin"))

    if not args.dirs:
        print(f"{'cmd':>6} {'got':>7} {'ratio':>6} {'deficit':>8} {'tilt99':>7}")
        for cmd in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.65, 0.80, 1.00):
            got, tilt = _probe(policy, cmd)
            print(f"{cmd:6.2f} {got:7.3f} {got / cmd:6.2f} {cmd - got:8.3f} {tilt:7.3f}")
        return

    print(f"{'head':>5} {'wz':>5} {'cmd':>6} {'got':>7} {'ratio':>6} {'deficit':>8} {'tilt99':>7}")
    for wz in (0.0, 0.5):
        for deg in (0, 45, 90):
            a = math.radians(deg)
            for cmd in (0.25, 0.35, 0.50, 0.65):
                got, tilt = _probe(policy, cmd * math.cos(a), cmd * math.sin(a), wz)
                print(
                    f"{deg:5d} {wz:5.1f} {cmd:6.2f} {got:7.3f} "
                    f"{got / cmd:6.2f} {cmd - got:8.3f} {tilt:7.3f}"
                )


if __name__ == "__main__":
    main()
