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
"""Drive questions and the answers -> joystick decoding (pure functions)."""

from __future__ import annotations

from dataclasses import dataclass

from dimos.agents.typesafe.client import Answers, ChoiceAnswer, Question, Text, choice, noul

AXES = (("x", "forward", "backward"), ("y", "left", "right"), ("yaw", "turn_left", "turn_right"))
_CONTEXT = (
    "In `objects` the target is first. The steering bearing is the target's `bearing` while "
    "`way_to_target.state` is clear. While it is blocked it is the `bearing` of one entry of "
    "`way_to_target.open_sides`: the one whose `side` is `way_to_target.going_around.side` "
    "(the side you began with: keep it while it is listed); without `going_around`, the one "
    "with the smaller `detour_deg`, an entry with `been_there` only when no other is listed. "
    "Turn toward the steering bearing, drive when it is in front; never strafe toward it. "
    "While blocked, standing still (`robot.recent.pattern` still) is never an answer: turn."
)
_CONTEXT_Y = (
    "The steering bearing is the target's `bearing` while `way_to_target.state` is clear, "
    "else that of an entry of `way_to_target.open_sides`."
)


def _opt(what: str, not_for: str, examples: list[object]) -> Text:
    return {"what": what, "not_for": not_for, "examples": examples}


def _q(question: str) -> Text:
    return {"question": question, "context": _CONTEXT}


def questions(labels: tuple[str, ...]) -> dict[str, Question]:
    qs: dict[str, Question] = {
        "drive.x": choice(
            _q(
                "Should the robot move straight ahead, reverse, or neither, to get closer to the target named in `goal`?"
            ),
            {
                "forward": _opt(
                    "the steering bearing is ahead (a listed open side has room along it whatever `free_space.ahead` reads; for the target `way_to_target.room` must not be blocked), or it is ahead_left or ahead_right and `free_space.ahead.state` is not blocked",
                    "steering bearing left, right or behind: turn first; target ahead with `way_to_target.room` blocked",
                    [
                        "bookshelf ahead, mid, way clear, room clear",
                        "way blocked by wall, doorway ahead, ahead tight",
                    ],
                ),
                "none": _opt(
                    "the steering bearing is left, right or behind (turn first), the target is touching, or the way is clear and `way_to_target.room` is blocked",
                    "steering bearing ahead with room along it",
                    ["person left, near", "way blocked by wall, doorway left"],
                ),
                "backward": _opt(
                    "the robot is touching an obstacle ahead and must back away, or `robot.recent.pattern` is stuck with the target not near",
                    "any case where turning or stopping would do",
                    ["ahead blocked at 0.3 m"],
                ),
            },
        ),
        "drive.y": choice(
            {
                "question": "Should the robot strafe left, right, or neither, to sidestep an obstacle right in front of it?",
                "context": _CONTEXT_Y,
            },
            {
                "left": _opt(
                    "a sidestep: way clear, target ahead and not touching, `way_to_target.room` blocked, left not blocked, and `way_to_target.narrowed_on` is right, or without `narrowed_on` the target is mid or far and `robot.last_drive.y` is already left or left has more room than right",
                    "steering bearing off to a side: turn, do not strafe; room ahead; left blocked",
                    ["lamp ahead, mid, room blocked, left clear 2.1, right tight 0.8"],
                ),
                "none": _opt(
                    "every other case: steering bearing off to a side or behind (turn instead), room along the line, way blocked, or target near without `narrowed_on`",
                    "way clear, target mid or far and ahead, room blocked, space on a side",
                    ["bookshelf ahead, mid", "way blocked by wall, open side left"],
                ),
                "right": _opt(
                    "a sidestep: way clear, target ahead and not touching, `way_to_target.room` blocked, right not blocked, and `way_to_target.narrowed_on` is left, or without `narrowed_on` the target is mid or far and `robot.last_drive.y` is already right or right has more room than left",
                    "steering bearing off to a side: turn, do not strafe; room ahead; right blocked",
                    ["lamp ahead, far, room blocked, right clear 3.0, left blocked"],
                ),
            },
        ),
        "drive.yaw": choice(
            _q(
                "Should the robot rotate in place counter-clockwise, clockwise, or not at all, so the steering bearing is ahead?"
            ),
            {
                "turn_left": _opt(
                    "the steering bearing is left, ahead_left, or behind_left",
                    "steering bearing ahead or on the right",
                    ["person left, far, way clear", "way blocked by wall, open side ahead_left"],
                ),
                "none": _opt(
                    "the steering bearing is ahead",
                    "steering bearing off to a side or behind",
                    ["bookshelf ahead, mid, way clear", "way blocked by bench, open side ahead"],
                ),
                "turn_right": _opt(
                    "the steering bearing is right, ahead_right, or behind_right",
                    "steering bearing ahead or on the left",
                    [
                        "lamp behind_right",
                        "way blocked by wall, open side right, going_around right",
                    ],
                ),
            },
        ),
        "stop": noul(
            {
                "question": "Should the robot stop moving right now?",
                "context": "Read `goal` and `objects` (the target is first).",
            },
            {
                "true": "the target named in `goal` has `distance` touching, or `goal` asks to stop, or the target is not in `objects`",
                "false": "the target is in `objects` with `distance` near, mid or far; an obstacle ahead or a blocked way is a reason to turn toward an open side, not to stop; being near is not a reason to stop",
            },
        ),
        "task": choice(
            {
                "question": "Is the task in `goal` complete, or should the robot keep going?",
                "context": "Read `task`, `goal`, `objects` and `way_to_target`. `distance` is measured to the object's nearest edge.",
            },
            {
                "finished": _opt(
                    "the target named in `goal` is in `objects` with `distance` touching, or near while `robot.motion` is stopped or `robot.recent.pattern` is stuck, and `way_to_target.blocked_by` is not wall (way clear with `room` blocked, or stuck, means it is as close as it gets): the robot is at the object and the task is done",
                    "the target is mid or far, or near while still driving, or near with a wall on the line to it, or not in `objects`",
                    ["bookshelf ahead, touching", "bench ahead, near, robot stopped, way clear"],
                ),
                "continue": _opt(
                    "the target's `distance` is mid or far, or near while the robot is still driving, or `way_to_target.blocked_by` is wall (the target is on the other side of it), or the target is not yet in `objects`",
                    "the robot is touching the target, or stopped near it with no wall between",
                    ["bookshelf ahead_left, mid", "lamp right, near, way blocked by wall"],
                ),
            },
        ),
    }
    if labels:
        qs["target"] = choice(
            "Which entry of `objects` is the thing `goal` asks to go to? Match by `label`.",
            {**dict.fromkeys(labels), "none": "`goal` names nothing that is in `objects`"},
        )
    return qs


@dataclass(frozen=True)
class Drive:
    x: float
    y: float
    yaw: float
    stop: bool
    confidence: float
    labels: tuple[str, str, str]
    target: str | None
    finished: bool = False

    @property
    def is_zero(self) -> bool:
        return self.stop or not (self.x or self.y or self.yaw)


def _choice(answers: Answers, key: str) -> ChoiceAnswer | None:
    a = answers.get(key)
    return a if a is not None and a["type"] == "choice" else None


def _noul(answers: Answers, key: str, threshold: float) -> bool:
    a = answers.get(key)
    return a is not None and a["type"] == "noul" and a["noul"] >= threshold


def decode(answers: Answers, *, stop_threshold: float) -> Drive:
    """Picks are taken as picked: confidence is reported, never a gate (it only measures
    how far the other options trailed)."""
    task = _choice(answers, "task")
    finished = task is not None and task["choice"] == "finished"
    stop = finished or _noul(answers, "stop", stop_threshold)
    vals: list[float] = []
    labels: list[str] = []
    confs: list[float] = []
    for axis, pos, _neg in AXES:
        a = _choice(answers, f"drive.{axis}")
        label, conf = (a["choice"], a["confidence"]) if a else ("none", 0.0)
        vals.append(0.0 if stop or label == "none" else 1.0 if label == pos else -1.0)
        labels.append(label)
        confs.append(conf)
    t = _choice(answers, "target")
    target = t["choice"] if t and t["choice"] != "none" else None
    return Drive(
        vals[0],
        vals[1],
        vals[2],
        stop,
        min(confs),
        (labels[0], labels[1], labels[2]),
        target,
        finished,
    )
