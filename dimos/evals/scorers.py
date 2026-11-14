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

"""Scoring helpers: plain functions over typed values, graded credit in one line.

Scores are floats in ``[0, 1]``. Msg types support arithmetic, so physical
scorers stay one-liners::

    lambda s: ramp((GOAL - s.streams.odom.last().data.position).length(), band=0.5)

LLM-based scoring wraps ``openevals`` — a function library (nothing to
subclass): factories return evaluators called with
``inputs/outputs/reference_outputs`` returning ``{"key", "score", "comment"}``.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import itertools
import math
from typing import TYPE_CHECKING, TypeVar

import numpy as np

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.types import Outcome, recording

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Transform import Transform

T = TypeVar("T")


def exact(expected: T, got: T) -> float:
    return float(expected == got)


def rank_order(expected: Sequence[str], got: Sequence[str]) -> float:
    """Fraction of correctly ordered pairs in a complete, unique-label ranking."""
    if len(expected) < 2 or len(set(expected)) != len(expected):
        raise ValueError("Expected ranking must contain at least two unique labels")
    if len(got) != len(expected) or set(got) != set(expected):
        return 0.0
    positions = {label: i for i, label in enumerate(got)}
    correct = sum(
        positions[left] < positions[right]
        for i, left in enumerate(expected)
        for right in expected[i + 1 :]
    )
    return correct / (len(expected) * (len(expected) - 1) / 2)


def numeric(expected: float, got: float, *, tolerance: float, band: float) -> float:
    """Compare numbers: full credit within tolerance, linear to zero at band.

    Parse model text separately, e.g. with ``first_number``. Non-finite
    observations receive zero; invalid scoring parameters raise ValueError.
    """
    if not all(math.isfinite(v) for v in (expected, tolerance, band)) or not 0 <= tolerance < band:
        raise ValueError("Require finite reference and 0 <= tolerance < band")
    if not math.isfinite(got):
        return 0.0
    error = abs(got - expected)
    # Allow only a few floating-point ULPs, capped relative to the score band.
    rounding = min(4 * max(math.ulp(got), math.ulp(expected)), (band - tolerance) * 1e-12)
    if error <= tolerance or abs(error - tolerance) <= rounding:
        return 1.0
    if error >= band or abs(error - band) <= rounding:
        return 0.0
    return max(0.0, min(1.0, (band - error) / (band - tolerance)))


# -- parsers (model text -> typed answer) -----------------------------------------


def ranking(text: str) -> tuple[str, ...]:
    """Parse single-letter labels, contiguous or separated by commas/whitespace.

    Vocabulary, completeness, and uniqueness are checked by ``rank_order``.
    """
    return tuple(c for c in text.strip().upper() if c != "," and not c.isspace())


def first_number(text: str) -> float:
    """The number a reply answers with: the only number on its last line ("...\\n\\n4",
    "≈ 3.1 m²", "Answer: 4"), else the first number anywhere ("about 12.5 meters")."""
    import re

    plain = re.sub(r"(?<=\d),(?=\d{3}\b)", "", text)  # 20,834 -> 20834
    on_last_line = re.findall(_NUMBER, _last_line(plain))
    if len(on_last_line) == 1:
        return float(on_last_line[0])
    match = re.search(_NUMBER, plain)
    if match is None:
        raise ValueError(f"no number in reply: {text[:80]!r}")
    return float(match.group())


def yes_no(text: str) -> str:
    """Normalize a reply to "yes"/"no": the only yes/no on its last line, else its opening word."""
    import re

    on_last_line = re.findall(r"\b(yes|no)\b", _last_line(text).lower())
    if len(on_last_line) == 1:
        return str(on_last_line[0])
    t = text.strip().lower().lstrip("*_`#\"' ")
    if t.startswith(("yes", "no")):
        return "yes" if t.startswith("yes") else "no"
    raise ValueError(f"not a yes/no reply: {text[:80]!r}")


def choice(options: Sequence[str], *, case_sensitive: bool = False) -> Callable[[str], str]:
    """Parser for a multiple-choice reply: the last option the model names, so
    that reasoning before the answer does not decide it. Longest option first,
    so "northeast" wins over "north". ``case_sensitive`` for lettered options
    ("A", "B"), where the article "a" must not count.

    Boundaries are alphanumeric lookarounds rather than ``\\b``, so options that
    end in punctuation (e.g. ``under 25%``) still match when quoted or alone.
    """
    import re

    words = "|".join(re.escape(o) for o in sorted(options, key=len, reverse=True))
    # (?<![A-Za-z0-9_]) / (?![A-Za-z0-9_]) ≈ word edges, but also allow a match
    # to end on '%', which \\b does not ('%' is non-word, so \\b after it needs
    # a following word char).
    pattern = re.compile(
        rf"(?<![A-Za-z0-9_])({words})(?![A-Za-z0-9_])",
        0 if case_sensitive else re.IGNORECASE,
    )

    def parse(text: str) -> str:
        # "north-west" must read as northwest, not as west.
        found = pattern.findall(re.sub(r"(?<=[A-Za-z])-(?=[A-Za-z])", "", text))
        if not found:
            raise ValueError(f"no option from {list(options)} in reply: {text[:80]!r}")
        return str(found[-1]) if case_sensitive else str(found[-1]).lower()

    return parse


_NUMBER = r"-?\d+(?:\.\d+)?"


def _last_line(text: str) -> str:
    lines = [line.strip("*_` .!\t") for line in text.splitlines() if line.strip()]
    return lines[-1] if lines else ""


def within(band: float) -> Callable[[float, float], float]:
    """1.0 at exact, linear to 0.0 at ``band`` away."""
    return lambda expected, got: max(0.0, 1.0 - abs(got - expected) / band)


def ramp(distance: float, band: float) -> float:
    """Distance (meters) -> [0, 1] credit inside ``band``."""
    return max(0.0, 1.0 - distance / band)


def lifted(body: str, *, by_m: float) -> Callable[[Outcome], float]:
    """How far the body ended above where it started, full credit at ``by_m``."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation.z
                end = last_body_transform(store, body).translation.z
            except LookupError:
                return 0.0
        return min(max((end - start) / by_m, 0.0), 1.0)

    return grade


def stacked_on(
    top: str, base: str, *, rise_m: tuple[float, float], band_m: float
) -> Callable[[Outcome], float]:
    """``top`` ended resting on ``base``: its centre ``rise_m`` above the base's, 1.0 centred and
    0.0 at ``band_m`` off. A body held higher than the resting height scores 0.0."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                t = last_body_transform(store, top).translation
                b = last_body_transform(store, base).translation
            except LookupError:
                return 0.0
        if not rise_m[0] <= t.z - b.z <= rise_m[1]:
            return 0.0
        return ramp(math.hypot(t.x - b.x, t.y - b.y), band=band_m)

    return grade


def inside(
    body: str, container: str, *, half_extents_m: tuple[float, float], rise_m: tuple[float, float]
) -> Callable[[Outcome], float]:
    """1.0 if ``body`` ended in ``container``: its centre over the floor, ``half_extents_m``
    (x, y) in the container's frame, and ``rise_m`` above the container's origin; else 0.0."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                t = last_body_transform(store, body).translation
                c = last_body_transform(store, container)
            except LookupError:
                return 0.0
        rotation = c.rotation.to_rotation_matrix()
        x, y, z = rotation.T @ np.array(
            [t.x - c.translation.x, t.y - c.translation.y, t.z - c.translation.z]
        )
        on_floor = abs(x) <= half_extents_m[0] and abs(y) <= half_extents_m[1]
        return float(on_floor and rise_m[0] <= z <= rise_m[1])

    return grade


def opened_door(outcome: Outcome) -> float:
    with recording(outcome) as store:
        try:
            frame = last_body_transform(store, "Door_frame").rotation.to_rotation_matrix()
            panel = last_body_transform(store, "Door_door").rotation.to_rotation_matrix()
        except LookupError:
            return 0.0
    relative = frame.T @ panel
    return float(math.atan2(relative[1, 0], relative[0, 0]) >= 0.3)


def placed_can(outcome: Outcome) -> float:
    with recording(outcome) as store:
        try:
            can = last_body_transform(store, "Can_main")
            marker = last_body_transform(store, "VisualCan_main")
        except LookupError:
            return 0.0
    delta = (can.translation - marker.translation).to_numpy()
    return float(
        np.all(np.abs(delta) <= [0.05, 0.075, 0.005])
        and can.rotation.to_rotation_matrix()[2, 2] > 0.98
    )


def seated_nut(outcome: Outcome) -> float:
    with recording(outcome) as store:
        try:
            nut = last_body_transform(store, "SquareNut_main")
            peg = last_body_transform(store, "peg1")
        except LookupError:
            return 0.0
    delta = (nut.translation - peg.translation).to_numpy()
    # The seated nut's centre is 2 cm below the exported peg body's origin.
    return float(
        np.linalg.norm(delta[:2]) < 0.007
        and abs(delta[2] + 0.02) < 0.004
        and abs(nut.rotation.to_rotation_matrix()[2, 2]) > 0.98
    )


def hung_tool(outcome: Outcome) -> float:
    """Geometric assembly/hanging proxy from body poses; does not check contact or release."""
    with recording(outcome) as store:
        try:
            stand = last_body_transform(store, "stand_root")
            frame = last_body_transform(store, "frame_root")
            hole = last_body_transform(store, "tool_hole1_root")
        except LookupError:
            return 0.0
    stand_r = stand.rotation.to_rotation_matrix()
    frame_r = frame.rotation.to_rotation_matrix()
    # Local offsets from the exported frame tip, stand slot and horizontal hook.
    tip_world = frame.translation.to_numpy() + frame_r @ np.array([0.04375, 0, -0.13445])
    tip_in_stand = stand_r.T @ (tip_world - stand.translation.to_numpy())
    hole_in_frame = frame_r.T @ (hole.translation - frame.translation).to_numpy()
    return float(
        stand_r[2, 2] > 0.98
        and np.dot(stand_r[:, 2], frame_r[:, 2]) > 0.98
        and np.linalg.norm(tip_in_stand - np.array([0, 0.045, -0.07])) < 0.01
        and -0.043 < hole_in_frame[0] < 0.04375
        and math.hypot(hole_in_frame[1], hole_in_frame[2] - 0.08625) < 0.007
        and abs(np.dot(hole.rotation.to_rotation_matrix()[:, 2], frame_r[:, 0])) > 0.95
    )


# ABC Sim (data/abc_*): success rules follow abc_sim/task_eval.
Box = tuple[tuple[float, float, float], tuple[float, float, float]]


def _inside(item: Transform, container: Transform, box: Box) -> bool:
    """``item``'s origin lies in ``box`` (centre, half sizes) in the container's frame."""
    local = (
        container.rotation.to_rotation_matrix().T
        @ (item.translation - container.translation).to_numpy()
    )
    return bool(np.all(np.abs(local - box[0]) <= box[1]))


def count_inside(
    items: Sequence[str], container: str, box: Box, *, want: int
) -> Callable[[Outcome], float]:
    """Exactly ``want`` of ``items`` ended inside ``container``."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                box_pose = last_body_transform(store, container)
                poses = [last_body_transform(store, item) for item in items]
            except LookupError:
                return 0.0
        return float(sum(_inside(pose, box_pose, box) for pose in poses) == want)

    return grade


def placed_beside(
    mover: str,
    reference: str,
    axis: tuple[float, float],
    *,
    along_m: tuple[float, float] = (0.06, 0.30),
    perp_max_m: float = 0.08,
    placed_z_m: float = 0.03,
) -> Callable[[Outcome], float]:
    """``mover`` rests ``along_m`` from ``reference`` along world ``axis``, within ``perp_max_m``
    of it, and no higher than ``placed_z_m`` above where it started."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, mover).translation
                end = last_body_transform(store, mover).translation
                ref = last_body_transform(store, reference).translation
            except LookupError:
                return 0.0
        rel = np.array([end.x - ref.x, end.y - ref.y])
        along = float(rel @ axis)
        perp = abs(float(rel[0] * axis[1] - rel[1] * axis[0]))
        return float(
            along_m[0] <= along <= along_m[1]
            and perp <= perp_max_m
            and end.z - start.z <= placed_z_m
        )

    return grade


def _last_poses(outcome: Outcome, bodies: Sequence[str]) -> dict[str, Transform] | None:
    """Each body's newest recorded pose, or None when any is missing."""
    with recording(outcome) as store:
        try:
            return {body: last_body_transform(store, body) for body in bodies}
        except LookupError:
            return None


def _local(item: Transform, frame: Transform | None) -> np.ndarray:
    """``item``'s origin in ``frame`` (the world when None)."""
    position: np.ndarray = item.translation.to_numpy()
    if frame is None:
        return position
    rotation: np.ndarray = frame.rotation.to_rotation_matrix()
    local: np.ndarray = rotation.T @ (position - frame.translation.to_numpy())
    return local


def _axis_up(pose: Transform, axis: int) -> float:
    """World-z component of the body's local ``axis`` (0 x, 1 y, 2 z)."""
    return float(pose.rotation.to_rotation_matrix()[2, axis])


def all_of(*grades: Callable[[Outcome], float]) -> Callable[[Outcome], float]:
    """1.0 only when every grade is: the lowest of them."""

    def grade(outcome: Outcome) -> float:
        return min(g(outcome) for g in grades)

    return grade


def in_box(
    items: Sequence[str],
    container: str | None,
    box: Box,
    *,
    at_least: int | None = None,
    outside: Sequence[str] = (),
) -> Callable[[Outcome], float]:
    """At least ``at_least`` of ``items`` (default: all) end with their origin in ``box``
    (centre, half sizes) in ``container``'s frame, or the world's when None, and none of
    ``outside`` do."""
    need = len(items) if at_least is None else at_least

    def grade(outcome: Outcome) -> float:
        poses = _last_poses(outcome, [*items, *outside, *([container] if container else [])])
        if poses is None:
            return 0.0
        frame = poses[container] if container else None

        def inside(body: str) -> bool:
            return bool(np.all(np.abs(_local(poses[body], frame) - box[0]) <= box[1]))

        return float(
            sum(inside(item) for item in items) >= need
            and not any(inside(other) for other in outside)
        )

    return grade


def in_cylinder(
    items: Sequence[str],
    container: str | None,
    centre: tuple[float, float],
    radius_m: float,
    height_m: tuple[float, float],
    *,
    at_least: int | None = None,
) -> Callable[[Outcome], float]:
    """At least ``at_least`` of ``items`` (default: all) end within ``radius_m`` of the
    vertical axis through ``centre`` and inside ``height_m``, in ``container``'s frame
    (the world's when None); the axis is the frame's local z."""
    need = len(items) if at_least is None else at_least

    def grade(outcome: Outcome) -> float:
        poses = _last_poses(outcome, [*items, *([container] if container else [])])
        if poses is None:
            return 0.0
        frame = poses[container] if container else None
        count = 0
        for item in items:
            x, y, z = _local(poses[item], frame)
            radial = math.hypot(x - centre[0], y - centre[1])
            count += radial <= radius_m and height_m[0] <= z <= height_m[1]
        return float(count >= need)

    return grade


def upright(
    bodies: Sequence[str], *, axis: int = 2, min_up: float = 0.95, max_lift_m: float | None = None
) -> Callable[[Outcome], float]:
    """Every body ends with its local ``axis`` within ``acos(min_up)`` of world up and, with
    ``max_lift_m``, no higher than that above where it started (so not still held)."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = {body: first_body_transform(store, body) for body in bodies}
                end = {body: last_body_transform(store, body) for body in bodies}
            except LookupError:
                return 0.0
        for body in bodies:
            if _axis_up(end[body], axis) < min_up:
                return 0.0
            lift = end[body].translation.z - start[body].translation.z
            if max_lift_m is not None and lift > max_lift_m:
                return 0.0
        return 1.0

    return grade


def standing(
    bodies: Sequence[str], *, axis: int = 2, max_up: float = 0.55
) -> Callable[[Outcome], float]:
    """Every body ends on edge: its local ``axis`` (a plate's normal) is mostly horizontal."""

    def grade(outcome: Outcome) -> float:
        poses = _last_poses(outcome, bodies)
        if poses is None:
            return 0.0
        return float(all(abs(_axis_up(poses[b], axis)) <= max_up for b in bodies))

    return grade


def spelled(
    letters: Sequence[str],
    *,
    rest_z: float,
    gap_m: tuple[float, float] = (0.015, 0.060),
    row_dx_max_m: float = 0.03,
    z_tolerance_m: float = 0.02,
    face_up_min: float = 0.85,
) -> Callable[[Outcome], float]:
    """The letter blocks, in order, form one row that reads left to right from the operator's
    side (decreasing world y), each at rest on the table with a letter face (local x) up
    (abc_sim/task_eval/spell.py)."""

    def grade(outcome: Outcome) -> float:
        poses = _last_poses(outcome, letters)
        if poses is None:
            return 0.0
        points = [poses[letter].translation for letter in letters]
        for a, b in itertools.pairwise(points):
            dy, dx = b.y - a.y, b.x - a.x
            if not (-gap_m[1] <= dy <= -gap_m[0] and abs(dx) <= row_dx_max_m):
                return 0.0
        return float(
            all(
                abs(poses[letter].translation.z - rest_z) <= z_tolerance_m
                and abs(_axis_up(poses[letter], 0)) >= face_up_min
                for letter in letters
            )
        )

    return grade


def set_up(
    board: str,
    goals: Mapping[str, Sequence[tuple[float, float, float]]],
    pieces: Mapping[str, Sequence[str]],
    *,
    planar_m: float = 0.028,
    height_m: float = 0.05,
    min_up: float = 0.85,
) -> Callable[[Outcome], float]:
    """Every goal square (``goals[kind]``, board-frame centres) holds a standing piece of that
    kind (any of ``pieces[kind]``: identical pieces are interchangeable), within ``planar_m``
    across the board and ``height_m`` above it (abc_sim/task_eval/chess.py). The board's
    local y is its normal."""

    def grade(outcome: Outcome) -> float:
        bodies = [board, *(p for kind in goals for p in pieces[kind])]
        poses = _last_poses(outcome, bodies)
        if poses is None:
            return 0.0
        frame = poses[board]
        for kind, squares in goals.items():
            free = [p for p in pieces[kind] if _axis_up(poses[p], 2) >= min_up]
            for square in squares:
                match = next(
                    (
                        p
                        for p in free
                        if math.hypot(*(_local(poses[p], frame) - square)[[0, 2]]) <= planar_m
                        and abs((_local(poses[p], frame) - square)[1]) <= height_m
                    ),
                    None,
                )
                if match is None:
                    return 0.0
                free.remove(match)
        return 1.0

    return grade


def crossed_table(
    body: str,
    *,
    start_sign: float,
    table_z: float,
    x_bounds: tuple[float, float],
    y_bounds: tuple[float, float],
    min_y_m: float = 0.10,
    max_height_m: float = 0.10,
) -> Callable[[Outcome], float]:
    """``body`` ends at rest on the far half of the table: at least ``min_y_m`` past the
    centre line from the side ``start_sign`` (+1 left, -1 right) it started on, within the
    table top and below ``max_height_m`` above it (abc_sim/task_eval/inhand_transfer.py)."""

    def grade(outcome: Outcome) -> float:
        poses = _last_poses(outcome, [body])
        if poses is None:
            return 0.0
        p = poses[body].translation
        return float(
            p.y * start_sign <= -min_y_m
            and p.z <= table_z + max_height_m
            and x_bounds[0] <= p.x <= x_bounds[1]
            and y_bounds[0] <= p.y <= y_bounds[1]
        )

    return grade


def held_on_tray(
    tray: str,
    ball: str,
    *,
    half_extents_m: tuple[float, float],
    ball_radius_m: float,
    lift_m: float = 0.05,
    hold_s: float = 10.0,
    dropped_z_m: float = 0.008,
) -> Callable[[Outcome], float]:
    """Stricter than abc_sim/task_eval/ball_tray.py, which passes a robot that never moves:
    for the last ``hold_s`` of the run the tray stays at least ``lift_m`` above where it
    started, and the ball stays on it (inside the tray's footprint, less its radius, and not
    below its surface)."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            if "body_poses" not in store.streams:
                return 0.0
            frames: list[tuple[float, dict[str, Transform]]] = []
            for record in store.streams.body_poses.order_by("ts"):
                found = {t.child_frame_id: t for t in record.data.transforms}
                if tray in found and ball in found:
                    frames.append((record.ts, found))
        if not frames or frames[-1][0] - frames[0][0] < hold_s:
            return 0.0
        start_z = frames[0][1][tray].translation.z
        end = frames[-1][0]
        window = [poses for ts, poses in frames if ts >= end - hold_s]
        for poses in window:
            local = _local(poses[ball], poses[tray])
            if poses[tray].translation.z - start_z < lift_m:
                return 0.0
            if abs(local[0]) > half_extents_m[0] - ball_radius_m:
                return 0.0
            if abs(local[1]) > half_extents_m[1] - ball_radius_m:
                return 0.0
            if local[2] < ball_radius_m - dropped_z_m:
                return 0.0
        return 1.0

    return grade


def tower(
    blocks: Sequence[str],
    *,
    table_z: float,
    levels: int = 3,
    min_rise_m: float = 0.02,
    max_offset_m: float = 0.05,
) -> Callable[[Outcome], float]:
    """At least ``levels`` blocks end stacked: each one at least ``min_rise_m`` higher than,
    and within ``max_offset_m`` across of, the one below, the lowest on the table. Upstream
    ABC has no evaluator for its wood-block tower; this is ours."""

    def grade(outcome: Outcome) -> float:
        poses = _last_poses(outcome, blocks)
        if poses is None:
            return 0.0
        points = sorted((poses[b].translation.to_numpy() for b in blocks), key=lambda p: p[2])
        best = 1
        for i, base in enumerate(points):
            if base[2] > table_z + 0.06:
                break
            height, top = 1, base
            for p in points[i + 1 :]:
                if p[2] - top[2] >= min_rise_m and math.hypot(*(p[:2] - top[:2])) <= max_offset_m:
                    height, top = height + 1, p
            best = max(best, height)
        return float(best >= levels)

    return grade


def human(task: str) -> Callable[[Outcome], float]:
    """The operator watched the run and says whether ``task`` succeeded: 1.0 for yes."""

    def grade(outcome: Outcome) -> float:
        answer = input(
            f"\nTask: {task}\nAgent's final answer: {outcome.trajectory.final_answer!r}\n"
            "Did the robot succeed by the end of the agent's run? [y/N] "
        )
        return 1.0 if answer.strip().lower().startswith("y") else 0.0

    return grade


def judge(rubric: str, *, model: str = "openai:gpt-5.6-luna") -> Callable[[str, str], float]:
    """LLM-as-judge with partial credit via openevals ``continuous=True``.

    ``rubric`` may reference ``{inputs}``, ``{outputs}``, ``{reference_outputs}``.
    """
    from openevals.llm import create_llm_as_judge

    evaluator = create_llm_as_judge(prompt=rubric, model=model, continuous=True)

    def _score(expected: str, got: str) -> float:
        result = evaluator(inputs="", outputs=got, reference_outputs=expected)
        if isinstance(result, list):
            result = result[0]
        return float(result["score"])

    return _score


# -- reducers over a series (e.g. a score per recorded pose) --------------------------


def final(scores: Sequence[float]) -> float:
    return scores[-1]


def floor(scores: Sequence[float]) -> float:
    """Worst moment wins — "never left the zone"."""
    return min(scores)


def mean(scores: Sequence[float]) -> float:
    return sum(scores) / len(scores)
