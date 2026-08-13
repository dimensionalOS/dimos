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

"""Motion-conditioned swept envelope of the go2, measured in the fitted sim.

One steady command per cell; every robot geom's oriented bounding box is
projected into the yaw-aligned base frame at the full sim rate and the extents
accumulated over whole gait cycles. MAX, never a quantile: a collision is a
hard constraint. Feeds the per-heading `envelope` rows of the planner spec
(`planner/revision.md`); the union over every cell is what `Embodiment`
carries today.

    python -m dimos.navigation.motion.simulation.envelope [--json]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from typing import Any

import mujoco
import numpy as np

from dimos.navigation.motion.simulation import evaluate as ev
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.navigation.motion.simulation.walk import walk
from dimos.utils.data import get_data

POLICY_BLOB = "ml-trajectory-research/freewalk_mcf.bin"

# The lattice's own drift bins, left and right measured separately (folding to
# |angle| is the spec's assumption, not ours to make before checking).
DRIFTS: tuple[float, ...] = (
    0.0,
    26.6,
    -26.6,
    45.0,
    -45.0,
    63.4,
    -63.4,
    90.0,
    -90.0,
    116.6,
    -116.6,
    135.0,
    -135.0,
    153.4,
    -153.4,
    180.0,
)
SPEEDS: tuple[float, ...] = (0.2, 0.35, 0.5, 0.75, 0.95)

# Arc sweep: the deployed ceiling is the controller's max_yaw_rate.
MAX_YAW_RATE = 1.4
YAW_RATES: tuple[float, ...] = (0.35, 0.7, MAX_YAW_RATE)
ARC_SPEEDS: tuple[float, ...] = (0.35, 0.75)

SETTLE = 2.0  # gait converges before the window opens
WINDOW = 4.0  # >= 4 cycles of the ~1.8 Hz trot

# Speeds the runtime rows are baked over (planner/revision.md, "Speed:
# eliminated via the governor"). Stand is in the band because a follower can
# stop anywhere. 0.2 is NOT: this policy cannot execute it, so that cell
# measures a body marching in place against a command it refuses, and the sim
# undertracks the field pair badly enough at 0.35 that 0.5 is the honest top
# of the governed band. See planner/envelope_results.md.
GOVERNED: tuple[float, ...] = (0.0, 0.35, 0.5)

# Validity gates. The drift angle and the yaw are what say the body did the
# commanded motion; the speed ratio is reported per cell (the policy
# undertracks everywhere, see planner/envelope_results.md) and only a body
# that did not translate at all is called unreachable.
DIR_TOL = 15.0  # deg between commanded and achieved drift
YAW_TOL = 0.15  # rad/s of unbidden turn
YAW_GAIN_TOL = 0.5  # fraction of a commanded turn that must show up
STALL = 0.05  # m/s under which the body is marching in place

CORNERS = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)


def yaw_of(quat_wxyz: np.ndarray) -> float:
    """Body yaw from a wxyz quaternion."""
    w, x, y, z = quat_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def corner_offsets(aabb: np.ndarray) -> np.ndarray:
    """The 8 corners of each geom's local bounding box, ``(n, 6) -> (n, 8, 3)``."""
    out: np.ndarray = aabb[:, None, :3] + CORNERS[None] * aabb[:, None, 3:]
    return out


def to_base(points: np.ndarray, origin: np.ndarray, yaw: float) -> np.ndarray:
    """World points into the yaw-aligned base frame, ``(..., 3) -> (n, 2)``."""
    rel = (points - origin).reshape(-1, 3)[:, :2]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c], axis=1)


@dataclass
class Extent:
    """Swept outline in the yaw-aligned base frame, grown point by point."""

    lo: np.ndarray = field(default_factory=lambda: np.full(2, np.inf))
    hi: np.ndarray = field(default_factory=lambda: np.full(2, -np.inf))

    def add(self, xy: np.ndarray) -> None:
        self.lo = np.minimum(self.lo, xy.min(axis=0))
        self.hi = np.maximum(self.hi, xy.max(axis=0))

    def __or__(self, other: Extent) -> Extent:
        return Extent(np.minimum(self.lo, other.lo), np.maximum(self.hi, other.hi))

    def mirrored(self) -> Extent:
        """The same outline reflected across the body's own x axis."""
        return Extent(np.array([self.lo[0], -self.hi[1]]), np.array([self.hi[0], -self.lo[1]]))

    @property
    def empty(self) -> bool:
        return not bool(np.all(np.isfinite(self.lo)))

    @property
    def length(self) -> float:
        return float(self.hi[0] - self.lo[0])

    @property
    def width(self) -> float:
        return float(self.hi[1] - self.lo[1])

    @property
    def center_off(self) -> float:
        """Outline centre along body x, relative to the base frame origin."""
        return float((self.hi[0] + self.lo[0]) / 2.0)

    @property
    def lateral_off(self) -> float:
        """Outline centre along body y: how far the swept box lags the drift."""
        return float((self.hi[1] + self.lo[1]) / 2.0)


class Outline:
    """walk() probe: accumulates the swept outline once ``since`` has passed."""

    def __init__(self, since: float = SETTLE) -> None:
        self.since = since
        self.extent = Extent()
        self._ids: np.ndarray | None = None
        self._local: np.ndarray | None = None

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if data.time < self.since:
            return
        if self._ids is None:
            self._ids = np.flatnonzero(model.geom_bodyid != 0)
            self._local = corner_offsets(model.geom_aabb[self._ids])
        assert self._local is not None
        mats = data.geom_xmat[self._ids].reshape(-1, 3, 3)
        world = data.geom_xpos[self._ids][:, None, :] + np.einsum("kij,kcj->kci", mats, self._local)
        self.extent.add(to_base(world, data.qpos[0:3], yaw_of(data.qpos[3:7])))


@dataclass(frozen=True)
class Cell:
    """One steady command and the outline it swept."""

    drift: float  # commanded body-frame drift angle, deg
    speed: float  # commanded translation speed, m/s
    yaw_rate: float  # commanded wz, rad/s
    extent: Extent
    achieved: tuple[float, float, float]  # body vx, vy, mean wz
    reason: str  # "" when the cell is a measurement

    @property
    def ok(self) -> bool:
        return not self.reason

    @property
    def gain(self) -> float:
        """Achieved over commanded translation speed; nan for a pure spin."""
        if self.speed <= 0.0:
            return float("nan")
        return math.hypot(*self.achieved[:2]) / self.speed

    def row(self) -> dict[str, Any]:
        e = self.extent
        return {
            "drift": self.drift,
            "speed": self.speed,
            "yaw_rate": self.yaw_rate,
            "length": round(e.length, 4),
            "width": round(e.width, 4),
            "center_off": round(e.center_off, 4),
            "lo": [round(v, 4) for v in e.lo],
            "hi": [round(v, 4) for v in e.hi],
            "achieved": [round(v, 4) for v in self.achieved],
            "gain": round(self.gain, 3),
            "reason": self.reason,
        }


def command(drift: float, speed: float, yaw_rate: float = 0.0) -> np.ndarray:
    """Body-frame twist for a drift angle in degrees."""
    a = math.radians(drift)
    return np.array([speed * math.cos(a), speed * math.sin(a), yaw_rate])


def achieved_twist(
    t: np.ndarray, pos: np.ndarray, quat: np.ndarray, since: float
) -> tuple[float, float, float]:
    """Mean body-frame (vx, vy, wz) over the measurement window."""
    keep = t >= since
    t, pos, quat = t[keep], pos[keep], quat[keep]
    yaw = np.unwrap(np.array([yaw_of(q) for q in quat]))
    v = np.diff(pos[:, :2], axis=0) / np.diff(t)[:, None]
    c, s = np.cos(yaw[:-1]), np.sin(yaw[:-1])
    return (
        float((v[:, 0] * c + v[:, 1] * s).mean()),
        float((-v[:, 0] * s + v[:, 1] * c).mean()),
        float((yaw[-1] - yaw[0]) / (t[-1] - t[0])),
    )


def verdict(drift: float, speed: float, yaw_rate: float, got: tuple[float, float, float]) -> str:
    """Why a cell is not a measurement of the commanded motion, or ""."""
    vx, vy, wz = got
    if speed > 0.0:
        if math.hypot(vx, vy) < STALL:
            return "stalled"
        err = abs(math.degrees(math.atan2(vy, vx)) - drift)
        if min(err, 360.0 - err) > DIR_TOL:
            return "drifted"
    if yaw_rate == 0.0:
        return "turned" if abs(wz) > YAW_TOL else ""
    return "" if abs(wz) >= YAW_GAIN_TOL * abs(yaw_rate) else "no-turn"


def measure(
    policy: FreePolicy,
    drift: float,
    speed: float,
    yaw_rate: float = 0.0,
    *,
    settle: float = SETTLE,
    window: float = WINDOW,
    preset: ev.Preset | None = None,
) -> Cell:
    """Roll one steady command out and return its swept outline."""
    preset = preset or ev.FITTED
    probe = Outline(since=settle)
    with ev._physics(preset.physics):
        track = walk(
            policy,
            command=command(drift, speed, yaw_rate),
            seconds=settle + window,
            command_delay=preset.command_delay,
            actuator_tau=preset.actuator_tau,
            probe=probe,
        )
    got = achieved_twist(track.t, track.pos, track.quat, settle)
    return Cell(drift, speed, yaw_rate, probe.extent, got, verdict(drift, speed, yaw_rate, got))


def sweep(policy: FreePolicy, **kw: Any) -> list[Cell]:
    """Every cell of the protocol: steady translation, arcs, spin, stand."""
    cells = [measure(policy, 0.0, 0.0, **kw)]
    cells += [measure(policy, d, v, **kw) for d in DRIFTS for v in SPEEDS]
    cells += [
        measure(policy, 0.0, v, w * sign, **kw)
        for v in ARC_SPEEDS
        for w in YAW_RATES
        for sign in (1.0, -1.0)
    ]
    cells += [measure(policy, 0.0, 0.0, MAX_YAW_RATE * s, **kw) for s in (1.0, -1.0)]
    return cells


def union(cells: list[Cell]) -> Extent:
    """Outline over every measured cell -- what Embodiment carries today."""
    out = Extent()
    for c in cells:
        out = out | c.extent
    return out


def fold(
    cells: list[Cell], speeds: tuple[float, ...] = GOVERNED
) -> list[tuple[float, float, float, float, float]]:
    """`envelope` rows: per |drift|, the swept box that heading needs.

    Rows are stored for POSITIVE drift and mirrored by sign at lookup, so a row
    has to cover both what +|a| swept and the MIRROR of what -|a| swept: a box
    that does not contain its own mirror image is not one the two signs can
    share. That keeps the lateral lag (`off_y`) instead of unioning it away,
    which is what the ± fold used to cost. A stand cell in the band joins every
    row -- a follower can stop on any edge -- and joins it mirrored too, having
    no drift sign of its own.
    """
    fam: dict[float, Extent] = {}
    stand = Extent()
    for c in cells:
        if c.yaw_rate != 0.0 or c.speed not in speeds:
            continue
        if c.speed == 0.0:
            stand = stand | c.extent
            continue
        fam[c.drift] = fam.get(c.drift, Extent()) | c.extent
    stand = stand | stand.mirrored()
    rows = []
    for a in sorted({abs(d) for d in fam}):
        plus = fam.get(a, Extent()) | stand
        minus = (fam[-a] if -a in fam else fam[a]) | stand
        e = plus | minus.mirrored()
        rows.append(
            (
                a,
                round(e.length, 3),
                round(e.width, 3),
                round(e.center_off, 3),
                round(e.lateral_off, 3),
            )
        )
    return rows


def arc_inflate(cells: list[Cell]) -> float:
    """Extra swept width per rad-per-metre of curvature.

    The gait's turning splay collapses on yaw per metre, not on yaw rate, so
    this number survives a lattice pitch change unmeasured: the search
    multiplies it by the edge's own yaw change over the edge's own length.
    """
    straight = {c.speed: c.extent.width for c in cells if c.yaw_rate == 0.0 and c.drift == 0.0}
    wide: dict[tuple[float, float], float] = {}
    for c in cells:
        if c.yaw_rate == 0.0 or c.speed <= 0.0:
            continue
        k = (c.speed, abs(c.yaw_rate))
        wide[k] = max(wide.get(k, 0.0), c.extent.width)
    num = den = 0.0
    for (v, w), width in wide.items():
        x = w / v
        num += x * (width - straight[v])
        den += x * x
    return num / den if den else 0.0


def table(cells: list[Cell], band: tuple[float, ...] = GOVERNED) -> str:
    """The (drift, speed, yaw_rate) surface, one row per cell."""
    lines = [
        f"{'drift':>7} {'v':>5} {'wz':>6} {'len':>6} {'wid':>6} {'off':>7} "
        f"{'gain':>5} {'vx':>6} {'vy':>6} {'wz_a':>6}  note",
    ]
    for c in cells:
        e = c.extent
        lines.append(
            f"{c.drift:7.1f} {c.speed:5.2f} {c.yaw_rate:6.2f} {e.length:6.3f} "
            f"{e.width:6.3f} {e.center_off:+7.3f} {c.gain:5.2f} "
            f"{c.achieved[0]:+6.2f} {c.achieved[1]:+6.2f} {c.achieved[2]:+6.2f}  {c.reason}"
        )
    u = union(cells)
    lines.append(
        f"\nunion over {len(cells)} cells: {u.length:.3f} x {u.width:.3f}, "
        f"centre {u.center_off:+.3f}"
    )
    lines.append(f"\nenvelope rows baked over v in {band}:")
    for row in fold(cells, band):
        lines.append("    ({:.1f}, {:.3f}, {:.3f}, {:+.3f}, {:+.3f}),".format(*row))
    lines.append(f"arc_inflate = {arc_inflate(cells):.4f}  # extra width per rad/m of curvature")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(prog="motion.simulation.envelope")
    ap.add_argument("--policy", default=None, help="FREE .bin (default: the fitted freewalk net)")
    ap.add_argument("--preset", default=None, help="physics preset (default: fitted)")
    ap.add_argument("--settle", type=float, default=SETTLE, help="seconds before the window opens")
    ap.add_argument("--window", type=float, default=WINDOW, help="seconds of swept outline")
    ap.add_argument(
        "--bake",
        default=",".join(f"{v:g}" for v in GOVERNED),
        help="speeds the runtime rows are baked over",
    )
    ap.add_argument("--json", action="store_true", help="machine output instead of the table")
    args = ap.parse_args()

    band = tuple(float(v) for v in args.bake.split(","))
    policy = FreePolicy.load(args.policy or get_data(POLICY_BLOB))
    cells = sweep(
        policy,
        settle=args.settle,
        window=args.window,
        preset=ev.load_preset(args.preset),
    )
    if args.json:
        u = union(cells)
        print(
            json.dumps(
                {
                    "cells": [c.row() for c in cells],
                    "union": {
                        "length": round(u.length, 4),
                        "width": round(u.width, 4),
                        "center_off": round(u.center_off, 4),
                    },
                    "envelope": fold(cells, band),
                    "arc_inflate": round(arc_inflate(cells), 4),
                },
                indent=2,
            )
        )
    else:
        print(table(cells, band))


if __name__ == "__main__":
    main()
