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

"""Regenerate the G1 right-arm pour reach map committed next to this file.

The map answers one question the pour demo asks constantly: with the robot
standing here, can the right hand hold the pour poses over a pot at this
base-frame XY? It is a grid, sampled offline, so the runtime only ever does
a lookup (see ``manip_stance``).

When a capability map is supplied, two sources agree on a cell before it
counts as reachable:

* the arm capability map (``dimos.manipulation.reachability``) prefilters
  the standoff band. It quotients pelvis yaw, so it answers "reachable from
  this distance after turning in place" -- a superset of what is reachable
  at a fixed heading, which makes it a sound (never over-eager) filter;
* the real ``PinkIK`` solve, collision-checked against the same planning
  model the ManipulationModule uses, at the actual heading. This is what
  the quotient cannot answer, and it is what catches the arm's
  left/right asymmetry.

Both pour poses are checked: the upright hold over the pot and the tipped
pour. A cell is green only if both solve.

The capability map is only a performance prefilter. Omitting it probes every
cell with the runtime IK model, which is slower but exact and avoids allocating
one large capability grid per sampling worker::

    .venv/bin/python -m dimos.robot.unitree.g1.tool_pour_reach_map \
        --plot /tmp/reach.png

To use the optional prefilter, build it first and pass it to this tool::

    .venv/bin/python -m dimos.manipulation.reachability.construct \\
        --robot g1-right --samples 5000000 --workers 10 \\
        --out /tmp/g1-right_capability.npz

    .venv/bin/python -m dimos.robot.unitree.g1.tool_pour_reach_map \\
        --capability-map /tmp/g1-right_capability.npz --plot /tmp/reach.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.manipulation.reachability.capability_map import CapabilityMap
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.manip_config import make_g1_model_config
from dimos.robot.unitree.g1.manip_stance import (
    DEFAULT_MAP_PATH,
    DEFAULT_SPOUT_OFFSET_IN_PALM,
    POUR_Z,
    palm_position_for_spout,
    palm_to_capability_tcp,
    tip_radians_for_spout,
)

# The WBC's standing stance in sim, measured off /g1/joints once the policy
# has settled, in make_g1_model_config().joint_names order. Legs and waist
# are locked at these values while the arm solves: the waist pitch alone
# moves the hand ~1.5 cm, and the legs decide half the self-collisions.
_STANDING_POSTURE = (
    -0.3975, 0.0507, 0.0136, 0.7447, -0.3269, -0.0329,
    -0.3992, -0.0505, -0.0139, 0.7529, -0.3335, 0.0268,
    0.0032, -0.0118, 0.0523,
    0.0185, -0.0015, 0.0005, 0.0465, 0.0002, 0.0199, 0.0001,
    0.0535, 0.0053, -0.0015, 0.1348, 0.0075, 0.0836, -0.0070,
)  # fmt: skip

_GROUP_ID = "g1/right_arm"
# Every attempt reseeds off the same posture, so one attempt is the
# reproducible verdict: a committed grid should not shift between runs.
_MAX_ATTEMPTS = 1


def _rotation(tool_yaw: float, roll: float) -> np.ndarray:
    """Tool rotation for ``move_to_pose(roll=roll, pitch=0, yaw=tool_yaw)``."""
    cy, sy = np.cos(tool_yaw), np.sin(tool_yaw)
    cr, sr = np.cos(roll), np.sin(roll)
    yaw_matrix = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    roll_matrix = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return yaw_matrix @ roll_matrix


def _quaternion(rotation: np.ndarray) -> Quaternion:
    return Quaternion.from_rotation_matrix(rotation)


def capability_scores(
    cap: CapabilityMap,
    offsets: np.ndarray,
    pour_z: float,
    tip: float,
    spout_offset_in_palm: tuple[float, float, float],
) -> np.ndarray:
    """Heading-free capability score for the tipped pour at each pot offset."""
    positions = np.empty((len(offsets), 3))
    rotations = np.empty((len(offsets), 3, 3))
    for i, (qx, qy) in enumerate(offsets):
        rotation = _rotation(float(np.arctan2(qy, qx)), tip)
        palm = palm_position_for_spout(
            np.array([qx, qy, pour_z]),
            rotation,
            spout_offset_in_palm,
        )
        positions[i], rotations[i] = palm_to_capability_tcp(palm, rotation)
    return np.asarray(cap.scores(positions, rotations))


class _Solver:
    """The ManipulationModule's pose-target solve, offline and base-framed."""

    def __init__(self) -> None:
        self.config = make_g1_model_config()
        self.world = RoboPlanWorld()
        self.robot_id = self.world.add_robot(self.config)
        self.world.finalize()
        self.posture = JointState(
            {
                "name": [f"g1/{name}" for name in self.config.joint_names],
                "position": list(_STANDING_POSTURE),
            }
        )
        self.world.sync_from_joint_state(self.robot_id, self.posture)
        self.group = self.world._planning_groups.get(_GROUP_ID)
        self.ik = PinkIK(
            PinkKinematicsConfig(
                position_tolerance=0.01, orientation_cost=0.3, orientation_tolerance=0.35
            )
        )

    def _seed(self) -> JointState:
        live = dict(zip(self.posture.name, self.posture.position, strict=True))
        names, values = [], []
        for local in self.group.local_joint_names:
            names.append(f"g1/{local}")
            values.append(self.config.ik_posture.get(local, live[f"g1/{local}"]))
        return JointState({"name": names, "position": values})

    def solves(self, position: np.ndarray, rotation: np.ndarray) -> bool:
        """True when the palm can hold this base-frame pose, collision-free.

        The planning base sits at the default nominal pelvis pose, so a
        base-frame target is just a world target to the solver.
        """
        target = PoseStamped(
            frame_id="world",
            position=Vector3(*(float(v) for v in position)),
            orientation=_quaternion(rotation),
        )
        result = self.ik.solve_pose_targets(
            world=self.world,
            pose_targets={self.group: target},
            seed=self._seed(),
            check_collision=True,
            max_attempts=_MAX_ATTEMPTS,
        )
        return bool(result.is_success())


def build(
    capability_map: Path | None = None,
    pour_z: float = POUR_Z,
    tip: float | None = None,
    spout_offset_in_palm: tuple[float, float, float] = DEFAULT_SPOUT_OFFSET_IN_PALM,
    cell: float = 0.025,
    x_range: tuple[float, float] = (0.05, 0.85),
    y_range: tuple[float, float] = (-0.65, 0.45),
    min_capability: int = 1,
) -> dict[str, object]:
    """Grid the base-frame pot offsets and score each one."""
    effective_tip = tip_radians_for_spout(spout_offset_in_palm) if tip is None else tip
    cap = CapabilityMap.load(capability_map) if capability_map is not None else None
    xs = np.round(np.arange(x_range[0], x_range[1] + 1e-9, cell), 6)
    ys = np.round(np.arange(y_range[0], y_range[1] + 1e-9, cell), 6)
    offsets = np.array([(x, y) for y in ys for x in xs])

    if cap is None:
        scores = np.full((len(ys), len(xs)), min_capability, dtype=np.uint8)
    else:
        scores = capability_scores(
            cap,
            offsets,
            pour_z,
            effective_tip,
            spout_offset_in_palm,
        ).reshape(len(ys), len(xs))
    solver = _Solver()
    over = np.zeros(scores.shape, dtype=np.uint8)
    tipped = np.zeros(scores.shape, dtype=np.uint8)

    started = time.time()
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            # The capability map quotients heading, so a zero there means no
            # heading reaches this offset: skipping it cannot hide a green cell.
            if scores[iy, ix] < min_capability:
                continue
            tool_yaw = float(np.arctan2(y, x))
            tipped_rotation = _rotation(tool_yaw, effective_tip)
            # Match runtime: pre-position the palm at its final tipped-pour
            # position, then roll in place so the spout sweeps onto the pot.
            palm = palm_position_for_spout(
                np.array([x, y, pour_z]),
                tipped_rotation,
                spout_offset_in_palm,
            )
            over[iy, ix] = solver.solves(palm, _rotation(tool_yaw, 0.0))
            tipped[iy, ix] = solver.solves(palm, tipped_rotation)
    elapsed = time.time() - started

    green = int(np.count_nonzero(over & tipped))
    print(
        f"{green} green of {int(np.count_nonzero(scores >= min_capability))} probed "
        f"({offsets.shape[0]} cells) in {elapsed:.0f}s"
    )
    return {
        "generated_by": "dimos/robot/unitree/g1/tool_pour_reach_map.py",
        "frame": "base: pelvis ground projection, +x along pelvis heading",
        "capability_map": (
            {"robot": cap.robot, "model_id": cap.model_id} if cap is not None else None
        ),
        "pour_z": pour_z,
        "tip_radians": round(float(effective_tip), 6),
        "spout_offset_in_palm": list(spout_offset_in_palm),
        "cell": cell,
        "x0": float(xs[0]),
        "y0": float(ys[0]),
        "capability_score": [[int(v) for v in row] for row in scores],
        "ik_upright": [[int(v) for v in row] for row in over],
        "ik_tipped": [[int(v) for v in row] for row in tipped],
    }


def plot(data: dict[str, object], path: Path) -> None:
    """Render the grid: probed band, upright-only cells, and the green core."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scores = np.array(data["capability_score"])
    over = np.array(data["ik_upright"])
    tipped = np.array(data["ik_tipped"])
    cell, x0, y0 = data["cell"], data["x0"], data["y0"]
    picture = np.zeros(scores.shape)
    picture[scores > 0] = 1.0
    picture[over > 0] = 2.0
    picture[(over > 0) & (tipped > 0)] = 3.0
    extent = (
        x0 - cell / 2,
        x0 + cell * (scores.shape[1] - 0.5),
        y0 - cell / 2,
        y0 + cell * (scores.shape[0] - 0.5),
    )

    figure, axes = plt.subplots(figsize=(7.0, 6.0))
    axes.imshow(
        picture,
        origin="lower",
        extent=extent,
        cmap=matplotlib.colors.ListedColormap(["#1b1d23", "#4b3f2a", "#7a6a1f", "#3fa34d"]),
        vmin=0,
        vmax=3,
    )
    axes.plot(0.0, 0.0, marker="o", color="white")
    axes.annotate("pelvis", (0.0, 0.0), color="white", xytext=(4, 4), textcoords="offset points")
    axes.set_xlabel("pot x in base frame (m)")
    axes.set_ylabel("pot y in base frame (m)")
    axes.set_title(
        f"G1 right-arm pour reach, pot XY in base frame (pour z={data['pour_z']} m)\n"
        "green: upright + tipped both solve · olive: upright only · brown: capability band"
    )
    axes.grid(color="#ffffff22", linewidth=0.5)
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    print(path)


def cli_main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--capability-map",
        type=Path,
        default=None,
        help="Optional heading-free prefilter; omit to run exact IK over every grid cell",
    )
    parser.add_argument("--out", type=Path, default=DEFAULT_MAP_PATH)
    parser.add_argument("--pour-z", type=float, default=POUR_Z)
    parser.add_argument("--cell", type=float, default=0.025)
    parser.add_argument(
        "--spout-offset",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_SPOUT_OFFSET_IN_PALM,
        help="Palm-frame offset from right_hand_palm_link to the water exit (metres)",
    )
    parser.add_argument("--plot", type=Path, default=None)
    args = parser.parse_args()

    data = build(
        args.capability_map,
        pour_z=args.pour_z,
        spout_offset_in_palm=tuple(float(value) for value in args.spout_offset),
        cell=args.cell,
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, indent=2) + "\n")
    print(args.out)
    if args.plot is not None:
        plot(data, args.plot)


if __name__ == "__main__":
    cli_main()
