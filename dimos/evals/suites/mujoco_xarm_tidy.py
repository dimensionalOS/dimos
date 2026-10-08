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

"""xArm7 table cases: raw move-to-center, duty tidy pair, and upright-a-fallen-cup.

Reuses ``xarm7/scene.xml`` objects (cup, apple, orange). Prompts name the cup; they
do not give meter values. Graders score distance from the table-top center, and
the fallen-cup case also requires an upright orientation.

    dimos evals run dimos.evals.suites.mujoco_xarm_tidy --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
import tempfile
from typing import TYPE_CHECKING, Any

import mujoco

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.data import LfsPath

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall

PERCEPTION_MODULES = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)

# table_top box is centered at (0.45, 0) with half-size (0.15, 0.20).
TABLE_CENTER = (0.45, 0.0)
# Full credit at the center; 0 at and beyond this radius (messy offset distance).
_SCORE_BAND_M = 0.10
# Already-tidy control: stay close to the start pose.
_STAY_BAND_M = 0.03
# Messy start: 0.10 m in +x from the center — clear of apple and orange.
MESSY_CUP = (TABLE_CENTER[0] + _SCORE_BAND_M, TABLE_CENTER[1])

# Upright cup center height in stock scene; fallen lies on its side on the tabletop.
CUP_UPRIGHT_Z = 0.19
CUP_RADIUS = 0.035
# table top surface ≈ 0.13; fallen center ≈ surface + radius.
CUP_FALLEN_Z = 0.13 + CUP_RADIUS
# MuJoCo quat w x y z: 90° about +y lays the cylinder on its side.
CUP_FALLEN_QUAT = (0.70710678, 0.0, 0.70710678, 0.0)

# Scene fruit centers / radii (for clearance checks in tests).
APPLE_XY = (0.40, 0.08)
# Stock orange sits on the cup's tidy spot; eval scenes move it toward -y.
ORANGE_EVAL_XY = (0.45, -0.15)
APPLE_RADIUS = 0.04
ORANGE_RADIUS = 0.045

_TABLE_X = (0.30, 0.60)
_TABLE_Y = (-0.20, 0.20)

_RAW = "Move the cup to the middle of the table."

_DUTY = (
    "You are a cleaning arm that is periodically woken by a cron job to tidy the table. "
    "The table is expected to have a cup in the middle of the table among the other items. "
    "Feel free to act as you see fit. Go."
)

_DUTY_UPRIGHT = (
    "You are a cleaning arm that is periodically woken by a cron job to tidy the table. "
    "The cup should stand upright in the middle of the table among the other items — "
    "not lying on its side. Feel free to act as you see fit. Go."
)


def _cup_on_table(x: float, y: float) -> bool:
    return (
        _TABLE_X[0] + CUP_RADIUS <= x <= _TABLE_X[1] - CUP_RADIUS
        and _TABLE_Y[0] + CUP_RADIUS <= y <= _TABLE_Y[1] - CUP_RADIUS
    )


def at_xy(
    end: tuple[float, float],
    target: tuple[float, float],
    *,
    band: float = _SCORE_BAND_M,
    start_z: float | None = None,
    end_z: float | None = None,
    z_band: float = 0.02,
) -> float:
    """1.0 at ``target``, linear to 0 at ``band``, and 0 off the table."""
    if start_z is not None and end_z is not None and abs(end_z - start_z) > z_band:
        return 0.0
    if not _cup_on_table(end[0], end[1]):
        return 0.0
    return ramp(math.hypot(end[0] - target[0], end[1] - target[1]), band=band)


def stayed_xy(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    band: float = _STAY_BAND_M,
    start_z: float | None = None,
    end_z: float | None = None,
    z_band: float = 0.02,
) -> float:
    """1.0 when ``end`` stays near ``start`` and on the table (already-tidy control)."""
    return at_xy(end, start, band=band, start_z=start_z, end_z=end_z, z_band=z_band)


def uprightness(rotation: Quaternion) -> float:
    """1.0 when the cup axis is vertical, 0 when it is on its side (or inverted is OK)."""
    axis = rotation.rotate_vector(Vector3(0.0, 0.0, 1.0))
    # abs: upside-down still counts as standing for this tabletop task.
    alignment = abs(axis.z)
    # Full credit above ~18° from vertical; none once past ~60° from vertical.
    return ramp(max(0.0, 1.0 - alignment), band=0.5)


def at_xy_upright(
    end: tuple[float, float],
    target: tuple[float, float],
    rotation: Quaternion,
    *,
    end_z: float,
    upright_z: float = CUP_UPRIGHT_Z,
    band: float = _SCORE_BAND_M,
    z_band: float = 0.03,
) -> float:
    """Near ``target`` on the table, standing upright near the usual height."""
    if abs(end_z - upright_z) > z_band:
        return 0.0
    xy = at_xy(end, target, band=band)
    return xy * uprightness(rotation)


def near_table_center(
    body: str, center: tuple[float, float] = TABLE_CENTER, *, band: float = _SCORE_BAND_M
) -> Callable[[Outcome], float]:
    """Credit for finishing ``body`` near the middle of the table."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation
                end = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return at_xy(
            (end.x, end.y),
            center,
            band=band,
            start_z=start.z,
            end_z=end.z,
        )

    return grade


def near_center_upright(
    body: str, center: tuple[float, float] = TABLE_CENTER, *, band: float = _SCORE_BAND_M
) -> Callable[[Outcome], float]:
    """Credit for finishing upright near the middle (height may change from a fall)."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                end = last_body_transform(store, body)
            except LookupError:
                return 0.0
        t = end.translation
        return at_xy_upright((t.x, t.y), center, end.rotation, end_z=t.z, band=band)

    return grade


def stayed_put(body: str, *, band: float = _STAY_BAND_M) -> Callable[[Outcome], float]:
    """Credit for leaving ``body`` where the episode started."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation
                end = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return stayed_xy(
            (start.x, start.y),
            (end.x, end.y),
            band=band,
            start_z=start.z,
            end_z=end.z,
        )

    return grade


def _write_eval_scene(
    dest_dir: Path,
    cup_xy: tuple[float, float],
    *,
    cup_z: float = CUP_UPRIGHT_Z,
    cup_quat: tuple[float, float, float, float] | None = None,
    orange_xy: tuple[float, float] = ORANGE_EVAL_XY,
) -> Path:
    """Write a per-launch scene under ``dest_dir`` via MjSpec (not the LFS tree)."""
    stock = Path(str(LfsPath("xarm7/scene.xml")))
    spec = mujoco.MjSpec.from_file(str(stock))
    meshdir = spec.meshdir or "."
    spec.meshdir = str((stock.parent / meshdir).resolve())
    cup = spec.body("cup")
    cup.pos = [*cup_xy, cup_z]
    if cup_quat is not None:
        cup.quat = list(cup_quat)
    orange = spec.body("orange")
    orange.pos = [*orange_xy, float(orange.pos[2])]
    path = dest_dir / "scene.xml"
    path.write_text(spec.to_xml())
    return path


class _CupSceneEnv(MujocoEnvironment):
    """Launch with cup (and optional tip-over) plus orange rewritten on the stock table."""

    def __init__(
        self,
        cup_xy: tuple[float, float],
        *,
        cup_z: float = CUP_UPRIGHT_Z,
        cup_quat: tuple[float, float, float, float] | None = None,
        **kwargs: Any,
    ) -> None:
        self._cup_xy = cup_xy
        self._cup_z = cup_z
        self._cup_quat = cup_quat
        super().__init__(**kwargs)

    def configure_launch(self, proc: DimosCliCall) -> None:
        workdir = Path(
            self._resources.enter_context(tempfile.TemporaryDirectory(prefix="xarm_tidy_"))
        )
        self.config.scene = _write_eval_scene(
            workdir,
            self._cup_xy,
            cup_z=self._cup_z,
            cup_quat=self._cup_quat,
        )
        super().configure_launch(proc)


def _env(
    cup_xy: tuple[float, float],
    *,
    cup_z: float = CUP_UPRIGHT_Z,
    cup_quat: tuple[float, float, float, float] | None = None,
) -> MujocoEnvironment:
    return _CupSceneEnv(
        cup_xy,
        cup_z=cup_z,
        cup_quat=cup_quat,
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath("xarm7/scene.xml"),
        tracked_bodies=("cup",),
    )


# Raw manipulation: explicit command; upright cup starts off-center.
move_cup_to_center = EvalCase(
    id="xarm_move_cup_to_center",
    inputs=_RAW,
    environment=_env(MESSY_CUP),
    grade=near_table_center("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "manipulation", "raw"}),
)

# Scene interpretability: open duty — fix when messy, leave alone when tidy.
tidy_cup_messy = EvalCase(
    id="xarm_tidy_cup_messy",
    inputs=_DUTY,
    environment=_env(MESSY_CUP),
    grade=near_table_center("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "manipulation", "interpretability"}),
)

tidy_cup_already_tidy = EvalCase(
    id="xarm_tidy_cup_already_tidy",
    inputs=_DUTY,
    environment=_env(TABLE_CENTER),
    grade=stayed_put("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "manipulation", "interpretability", "control"}),
)

# Fallen cup: stand it upright in the middle (duty states the upright expectation).
tidy_cup_fallen = EvalCase(
    id="xarm_tidy_cup_fallen",
    inputs=_DUTY_UPRIGHT,
    environment=_env(MESSY_CUP, cup_z=CUP_FALLEN_Z, cup_quat=CUP_FALLEN_QUAT),
    grade=near_center_upright("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "manipulation", "interpretability", "upright"}),
)

SUITE: Suite = [
    move_cup_to_center,
    tidy_cup_messy,
    tidy_cup_already_tidy,
    tidy_cup_fallen,
]
