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

from collections.abc import Callable, Sequence
from datetime import datetime
import math
from typing import TypeVar

import numpy as np

from dimos.evals.environments.lib.recorded_poses import (
    body_pose_stream,
    first_body_transform,
    last_body_transform,
)
from dimos.evals.types import Outcome, recording

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
    ("A", "B"), where the article "a" must not count."""
    import re

    words = "|".join(re.escape(o) for o in sorted(options, key=len, reverse=True))
    pattern = re.compile(rf"\b({words})\b", 0 if case_sensitive else re.I)

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


# Staged partial credit for the robosuite cases. These read the whole tf history of the tracked
# bodies, not only the final pose, so a failed run still earns the stages it reached. A case
# scores 1.0 only when its binary check above passes, so pass rates stay comparable.

GRIPPER = "xarm_gripper_base_link"
FINGERS = ("left_finger", "right_finger")
GRIPPER_BODIES = (GRIPPER, *FINGERS)
_TCP_M = 0.172  # along the gripper base's z axis (XARM7_GRIPPER_NOTES)
_SETTLE_S = 2.0  # spawned objects may still be dropping onto the table when recording starts


class _Track:
    """One body's world poses over the run, in time order."""

    def __init__(self, ts: list[float], pos: list[np.ndarray], rot: list[np.ndarray]) -> None:
        self.ts = np.asarray(ts)
        self.pos = np.asarray(pos)
        self.rot = np.asarray(rot)
        settled = int(np.searchsorted(self.ts, self.ts[0] + _SETTLE_S, side="right")) - 1
        self.rest = self.pos[max(settled, 0)]

    @property
    def final(self) -> np.ndarray:
        return self.pos[-1]

    def rise(self) -> float:
        """Highest the body got above its resting height."""
        return float(self.pos[:, 2].max() - self.rest[2])

    def moved(self) -> float:
        """Farthest the body got from its resting position."""
        return float(np.linalg.norm(self.pos - self.rest, axis=1).max())

    def at(self, ts: np.ndarray) -> np.ndarray:
        """Index of this track's newest sample at or before each of ``ts``."""
        return np.clip(np.searchsorted(self.ts, ts, side="right") - 1, 0, len(self.ts) - 1)


def _answered_at(outcome: Outcome) -> float | None:
    """When the agent gave its final answer (epoch seconds); None if it ran out of steps or time."""
    steps = outcome.trajectory.steps
    if not steps or steps[-1].source != "agent" or steps[-1].tool_calls:
        return None
    return datetime.fromisoformat(steps[-1].timestamp.replace("Z", "+00:00")).timestamp()


def _tracks(outcome: Outcome, bodies: Sequence[str]) -> dict[str, _Track]:
    """World tracks of ``bodies`` that the sim published; missing bodies are left out.

    A run that ends with an answer is graded as it stood at that answer: the sim keeps stepping
    for several seconds while the environment shuts down, and a held part can drop then.
    """
    raw: dict[str, tuple[list[float], list[np.ndarray], list[np.ndarray]]] = {
        b: ([], [], []) for b in bodies
    }
    until = _answered_at(outcome)
    with recording(outcome) as store:
        try:
            stream = body_pose_stream(store)
        except LookupError:
            return {}
        for record in stream.order_by("ts"):
            if until is not None and record.ts > until:
                break
            for transform in record.data.transforms:
                # Robot links are also on tf relative to their parents; only world poses count.
                if transform.child_frame_id in raw and transform.frame_id == "world":
                    ts, pos, rot = raw[transform.child_frame_id]
                    ts.append(record.ts)
                    pos.append(transform.translation.to_numpy())
                    rot.append(transform.rotation.to_rotation_matrix())
    return {b: _Track(*v) for b, v in raw.items() if v[0]}


def _tcp(tracks: dict[str, _Track]) -> np.ndarray | None:
    """The gripper's TCP over time, on the gripper base's timestamps."""
    base = tracks.get(GRIPPER)
    if base is None:
        return None
    return base.pos + base.rot[:, :, 2] * _TCP_M


def _closed(tracks: dict[str, _Track]) -> np.ndarray | None:
    """Per gripper sample: jaws at least 2 cm narrower than their widest in the run."""
    base, left, right = (tracks.get(b) for b in GRIPPER_BODIES)
    if base is None or left is None or right is None:
        return None
    gap = np.linalg.norm(left.pos[left.at(base.ts)] - right.pos[right.at(base.ts)], axis=1)
    return gap < gap.max() - 0.02


def _came_within(tracks: dict[str, _Track], body: str, radius_m: float) -> bool:
    """The TCP came within ``radius_m`` of ``body``'s origin at some point."""
    tcp, target = _tcp(tracks), tracks.get(body)
    if tcp is None or target is None:
        return False
    nearest = target.pos[target.at(tracks[GRIPPER].ts)]
    return bool((np.linalg.norm(tcp - nearest, axis=1) < radius_m).any())


def lift_partial(body: str, *, by_m: float, half_size_m: float) -> Callable[[Outcome], float]:
    """0.2 for touching ``body``, then 0.8 x final height gained / ``by_m``."""

    def grade(outcome: Outcome) -> float:
        tracks = _tracks(outcome, (body, *GRIPPER_BODIES))
        cube = tracks.get(body)
        if cube is None:
            return 0.0
        touched = cube.moved() > 0.01 or _came_within(tracks, body, half_size_m + 0.015)
        height = min(max((cube.final[2] - cube.rest[2]) / by_m, 0.0), 1.0)
        return float(0.2 * touched + 0.8 * height)

    return grade


def stack_partial(
    top: str, base: str, *, rise_m: tuple[float, float], band_m: float
) -> Callable[[Outcome], float]:
    """0.2 once ``top`` is lifted 2 cm, then 0.8 x the ``stacked_on`` placement score."""
    placed = stacked_on(top, base, rise_m=rise_m, band_m=band_m)

    def grade(outcome: Outcome) -> float:
        track = _tracks(outcome, (top,)).get(top)
        if track is None:
            return 0.0
        return float(0.2 * (track.rise() >= 0.02) + 0.8 * placed(outcome))

    return grade


# Door_handle geom in the Door_latch body frame: centre and half-sizes.
_HANDLE_CENTRE = np.array([0.075, -0.10, 0.0])
_HANDLE_HALF = np.array([0.075, 0.015, 0.02])


def door_partial(outcome: Outcome) -> float:
    """0.2 for holding the handle (or opening the door at all), then 0.8 x angle / 0.3 rad."""
    tracks = _tracks(outcome, ("Door_frame", "Door_door", "Door_latch", *GRIPPER_BODIES))
    frame, panel = tracks.get("Door_frame"), tracks.get("Door_door")
    if frame is None or panel is None:
        return 0.0
    relative = frame.rot[-1].T @ panel.rot[-1]
    angle = math.atan2(relative[1, 0], relative[0, 0])
    held = angle >= 0.05  # the door is latched, so it only swings once the handle is turned
    tcp, closed, latch = _tcp(tracks), _closed(tracks), tracks.get("Door_latch")
    if not held and tcp is not None and closed is not None and latch is not None:
        i = latch.at(tracks[GRIPPER].ts)
        local = np.einsum("nji,nj->ni", latch.rot[i], tcp - latch.pos[i]) - _HANDLE_CENTRE
        outside = np.linalg.norm(np.maximum(np.abs(local) - _HANDLE_HALF, 0.0), axis=1)
        held = bool(((outside < 0.015) & closed).any())
    return float(0.2 * held + 0.8 * min(max(angle, 0.0) / 0.3, 1.0))


def can_partial(outcome: Outcome) -> float:
    """0.2 lifted, 0.4 resting in the target compartment, 0.4 for position near the marker."""
    tracks = _tracks(outcome, ("Can_main", "VisualCan_main"))
    can, marker = tracks.get("Can_main"), tracks.get("VisualCan_main")
    if can is None or marker is None:
        return 0.0
    score = 0.2 * (can.rise() >= 0.03)
    dx, dy, dz = np.abs(can.final - marker.final)
    # The marker sits at the centre of its bin quarter (0.2 x 0.25 m) at the can's resting height.
    if not (dx < 0.1 and dy < 0.125 and dz < 0.02):
        return float(score)
    score += 0.4
    if placed_can(outcome):
        return float(score + 0.4)
    # Ramp from the pass box (5 x 7.5 cm) out to the compartment walls; half credit lying down.
    off = math.hypot(max(dx - 0.05, 0.0), max(dy - 0.075, 0.0))
    upright = can.rot[-1][2, 2] > 0.98
    return float(score + 0.4 * ramp(off, band=0.05) * (1.0 if upright else 0.5))


def nut_partial(outcome: Outcome) -> float:
    """0.4 picked up, 0.2 released resting near the peg, 0.4 x how far down the peg it went."""
    tracks = _tracks(outcome, ("SquareNut_main", "peg1"))
    nut, peg = tracks.get("SquareNut_main"), tracks.get("peg1")
    if nut is None or peg is None or nut.rise() < 0.02:
        return 0.0
    delta = nut.final - peg.final
    flat = math.hypot(delta[0], delta[1])
    centred = flat < 0.007 and abs(nut.rot[-1][2, 2]) > 0.98
    # The exported peg box is 0.2 m tall about its origin; a seated nut sits 2 cm below the origin.
    depth = (0.1 - delta[2]) / (0.1 + 0.02) if centred else 0.0
    resting = nut.final[2] <= nut.rest[2] + 0.01 or depth > 0.0
    score = 0.4 + 0.2 * (resting and flat < 0.05)
    if seated_nut(outcome):
        return float(score + 0.4)
    return float(score + 0.4 * min(max(depth, 0.0), 1.0))


def hang_partial(outcome: Outcome) -> float:
    """0.1 frame lifted, 0.3 frame in the stand, 0.1 wrench lifted, 0.2 wrench reached the hook,
    0.3 wrench hung (the full ``hung_tool`` check)."""
    tracks = _tracks(outcome, ("stand_root", "frame_root", "tool_hole1_root"))
    stand, frame, hole = (tracks.get(b) for b in ("stand_root", "frame_root", "tool_hole1_root"))
    if stand is None or frame is None or hole is None:
        return 0.0
    score = 0.1 * (frame.rise() >= 0.03) + 0.1 * (hole.rise() >= 0.03)
    # Frame in stand: same geometry as hung_tool, on the final poses.
    stand_r, frame_r = stand.rot[-1], frame.rot[-1]
    tip = frame.final + frame_r @ np.array([0.04375, 0, -0.13445])
    off = float(np.linalg.norm(stand_r.T @ (tip - stand.final) - np.array([0, 0.045, -0.07])))
    aligned = stand_r[2, 2] > 0.98 and np.dot(stand_r[:, 2], frame_r[:, 2]) > 0.98
    if aligned and off < 0.01:
        score += 0.3
    else:
        score += 0.3 * ramp(max(off - 0.01, 0.0), band=0.05) * (1.0 if aligned else 0.5)
    # Wrench at the hook at some point: hole within 3 cm of the hook segment in the frame.
    i = frame.at(hole.ts)
    local = np.einsum("nji,nj->ni", frame.rot[i], hole.pos - frame.pos[i])
    along = np.clip(local[:, 0], -0.043, 0.04375)
    near = np.sqrt((local[:, 0] - along) ** 2 + local[:, 1] ** 2 + (local[:, 2] - 0.08625) ** 2)
    score += 0.2 * bool((near < 0.03).any())
    return float(score + 0.3 * hung_tool(outcome))


# Bottle cap (binbottle): the cap is welded to a carrier that turns on a thread; after
# CAP_TURN_RAD the weld is released and the cap is a free body.
CAP_TURN_RAD = math.radians(200)


def cap_partial(outcome: Outcome) -> float:
    """0.1 touched the cap, 0.5 x how far it was unscrewed, 0.4 taken 2 cm off the neck.
    Where the cap ends up after that is not graded."""
    tracks = _tracks(outcome, ("cap_main", "cap_carrier", *GRIPPER_BODIES))
    cap, carrier = tracks.get("cap_main"), tracks.get("cap_carrier")
    if cap is None or carrier is None:
        return 0.0
    # The carrier only turns about the bottle's vertical axis (the cap itself tumbles once free).
    yaw = np.unwrap(np.arctan2(carrier.rot[:, 1, 0], carrier.rot[:, 0, 0]))
    turned = float((yaw - yaw[0]).max())
    touched = turned > 0.05 or cap.moved() > 0.002 or _came_within(tracks, "cap_main", 0.04)
    off = cap.moved() >= 0.02
    turn = min(max(turned / CAP_TURN_RAD, 0.0), 1.0)
    score = 0.1 * touched + 0.5 * turn + 0.4 * off
    return round(float(score), 6)


# Letter blocks (dimosblocks): spell DIMOS. The word may lie at any angle on the table; the letters'
# common up direction defines the reader, and the word must read left to right for that reader.
WORD = "DIMOS"
_LETTER_UP_LOCAL = 1  # the top-face letter's up direction is the block's local +y
_HALF_TURN_SAME = frozenset("IS")  # look the same turned 180 deg, so either way up reads
_ANY_TURN_SAME = frozenset("O")  # reads the same at any angle on the table


def word_partial(outcome: Outcome) -> float:
    """0.7 x neighbouring letter pairs in reading order, 0.3 x the share of letters upright for
    the same reader: top face up and turned to the reader (I and S either way up, O at any angle,
    but flat). Spacing and alignment are not graded. Nothing counts until a block was moved (the
    random start may already have a pair in order)."""
    names = [f"block_{c}_main" for c in WORD]
    tracks = _tracks(outcome, names)
    if len(tracks) < len(names):
        return 0.0
    blocks = [tracks[n] for n in names]
    if not any(b.moved() > 0.01 for b in blocks):
        return 0.0
    pos = np.array([b.final[:2] for b in blocks])
    ups = np.array([b.rot[-1][:2, _LETTER_UP_LOCAL] for b in blocks])  # letter up, on the table
    ups /= np.maximum(np.linalg.norm(ups, axis=1, keepdims=True), 1e-9)
    # Read the word as a person would: try each letter's up (and its reverse for I and S) as the
    # reader's up and keep the best-scoring reading. The reader's up is averaged over the letters
    # within 20 deg of it (I and S turned to match), so one stray letter does not tilt the word. O
    # has no up of its own, so it neither sets the reader's up nor needs to match it.
    cos20 = math.cos(math.radians(20))
    flips = np.array([c in _HALF_TURN_SAME for c in WORD])
    free = np.array([c in _ANY_TURN_SAME for c in WORD])
    flat = np.array([b.rot[-1][2, 2] > cos20 for b in blocks])  # lettered top faces up

    def read(candidate: np.ndarray) -> float:
        dots = ups @ candidate
        matches = (np.where(flips, np.abs(dots), dots) > cos20) & ~free
        signs = np.where(flips & (dots < 0), -1.0, 1.0)
        inliers = (ups * signs[:, None])[matches]
        up = inliers.sum(axis=0) / np.linalg.norm(inliers.sum(axis=0))
        right = np.array([up[1], -up[0]])  # the reader's left-to-right: up turned 90 deg clockwise
        along = pos @ right
        pairs = sum(along[i] < along[i + 1] for i in range(len(WORD) - 1))
        # Letters upright for this reader (I and S either way up, O at any angle), each credited.
        dots = ups @ up
        upright = flat & ((np.where(flips, np.abs(dots), dots) > cos20) | free)
        return 0.7 * pairs / (len(WORD) - 1) + 0.3 * float(upright.mean())

    return float(max(read(u) for u in (*ups[~free], *(-ups[flips]))))


# Pour (data/robosuite_2/pour): a cup of balls that stand in for water, poured into a bowl with
# slanted walls. Inside sizes, in each body's frame (origin at the centre of its base).
BALLS = tuple(f"ball{i}" for i in range(100))
_CUP_R, _CUP_H = 0.033, 0.10
_BOWL_R_BOTTOM, _BOWL_R_TOP, _BOWL_H = 0.05, 0.078, 0.055


def _final_positions(outcome: Outcome, bodies: Sequence[str]) -> dict[str, np.ndarray]:
    """World positions of ``bodies`` in the newest pose message at or before the answer.

    Reads backwards and stops at the first match, so tracking many bodies (the balls) costs one
    message, not the whole history ``_tracks`` reads.
    """
    until = _answered_at(outcome)
    wanted = set(bodies)
    with recording(outcome) as store:
        try:
            stream = body_pose_stream(store)
        except LookupError:
            return {}
        for record in stream.order_by("ts", desc=True):
            if until is not None and record.ts > until:
                continue
            found = {
                t.child_frame_id: t.translation.to_numpy()
                for t in record.data.transforms
                if t.child_frame_id in wanted and t.frame_id == "world"
            }
            if found:
                return found
    return {}


def _poured(balls: np.ndarray, bowl: _Track, cup: _Track) -> int:
    """Balls inside the bowl's walls at its final pose; any still inside the cup do not count."""

    def local(track: _Track) -> np.ndarray:
        return np.asarray((balls - track.final) @ track.rot[-1])  # row-wise R^T (p - origin)

    b = local(bowl)
    wall = _BOWL_R_BOTTOM + (_BOWL_R_TOP - _BOWL_R_BOTTOM) * np.clip(b[:, 2] / _BOWL_H, 0.0, 1.0)
    in_bowl = (b[:, 2] > 0.0) & (b[:, 2] < _BOWL_H + 0.01) & (np.hypot(b[:, 0], b[:, 1]) < wall)
    c = local(cup)
    in_cup = (c[:, 2] > 0.0) & (c[:, 2] < _CUP_H) & (np.hypot(c[:, 0], c[:, 1]) < _CUP_R)
    return int((in_bowl & ~in_cup).sum())


def pour_partial(outcome: Outcome) -> float:
    """0.1 touched the cup, 0.2 lifted it 3 cm, 0.7 x the share of the balls in the bowl at the
    end. Balls still in the cup do not count, even with the cup set down inside the bowl."""
    tracks = _tracks(outcome, ("cup_main", "bowl_main", *GRIPPER_BODIES))
    cup, bowl = tracks.get("cup_main"), tracks.get("bowl_main")
    if cup is None or bowl is None:
        return 0.0
    touched = cup.moved() > 0.005 or _came_within(tracks, "cup_main", 0.08)
    lifted = cup.rise() >= 0.03
    balls = _final_positions(outcome, BALLS)
    poured = _poured(np.array(list(balls.values())), bowl, cup) if balls else 0
    return round(float(0.1 * touched + 0.2 * lifted + 0.7 * poured / len(BALLS)), 6)
