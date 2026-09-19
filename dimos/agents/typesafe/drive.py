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
_CONTEXT = "Read `goal`, `robot`, `objects` (each has `bearing` and `distance`) and `room.sectors` (each has `state`)."


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
                    "the target is ahead / ahead_left / ahead_right and `room.sectors.ahead.state` is not blocked",
                    "target beside or behind; ahead blocked",
                    ["chair ahead, mid, ahead clear"],
                ),
                "none": _opt(
                    "the target is beside or behind, is touching, or ahead is blocked",
                    "target ahead with a clear path",
                    ["person left, near"],
                ),
                "backward": _opt(
                    "the robot is touching an obstacle ahead and must back away",
                    "any case where turning or stopping would do",
                    ["ahead blocked at 0.3 m"],
                ),
            },
        ),
        "drive.y": choice(
            _q(
                "Should the robot strafe left, right, or neither, to line up with the target named in `goal` or sidestep an obstacle?"
            ),
            {
                "left": _opt(
                    "the target is ahead_left or left and that side is not blocked",
                    "target ahead or right",
                    ["door ahead_left, near"],
                ),
                "none": _opt(
                    "the target is straight ahead, behind, or strafing would hit an obstacle",
                    "target clearly off to one side",
                    ["chair ahead, mid"],
                ),
                "right": _opt(
                    "the target is ahead_right or right and that side is not blocked",
                    "target ahead or left",
                    ["table ahead_right, near"],
                ),
            },
        ),
        "drive.yaw": choice(
            _q(
                "Should the robot rotate in place counter-clockwise, clockwise, or not at all, so the target named in `goal` is ahead?"
            ),
            {
                "turn_left": _opt(
                    "the target bearing is left, ahead_left, or behind_left",
                    "target ahead or on the right",
                    ["person left, far"],
                ),
                "none": _opt(
                    "the target is ahead", "target off to a side or behind", ["bed ahead, mid"]
                ),
                "turn_right": _opt(
                    "the target bearing is right, ahead_right, behind_right, or behind",
                    "target ahead or on the left",
                    ["chair behind"],
                ),
            },
        ),
        "stop": noul(
            _q("Should the robot stop moving right now?"),
            {
                "true": "the target named in `goal` has `distance` touching, or `goal` asks to stop, or the target is not in `objects`, or `room.sectors.ahead.state` is blocked while moving forward",
                "false": "the target is in `objects` with `distance` near, mid or far, and there is a clear direction to move; being near is not a reason to stop",
            },
        ),
        "task": choice(
            {
                "question": "Is the task in `goal` complete, or should the robot keep going?",
                "context": "Read `task`, `goal` and `objects`. `distance` is measured to the object's nearest edge.",
            },
            {
                "finished": _opt(
                    "the target named in `goal` is in `objects` with `distance` touching, or near while `robot.motion` is stopped: the robot is at the object and the task is done; the coordinates in `goal` are not a place to stand",
                    "the target is mid or far, or near while still driving, or not in `objects`",
                    ["chair ahead, touching", "table ahead, near, robot stopped"],
                ),
                "continue": _opt(
                    "the target's `distance` is mid or far, or near while the robot is still driving, or the target is not yet in `objects`",
                    "the robot is touching the target, or stopped near it",
                    ["chair ahead_left, mid"],
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
