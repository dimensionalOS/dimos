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
from dimos.agents.typesafe.drive import decode, questions
from dimos.agents.typesafe.types import Answers, ChoiceAnswer


def choice(label: str, *options: str, p: float = 1.0) -> ChoiceAnswer:
    """The chosen option carries probability `p`; the rest share the remainder."""
    rest = (1.0 - p) / max(1, len(options) - 1)
    return {
        "type": "choice",
        "choice": label,
        "confidence": p,
        "probabilities": {o: p if o == label else rest for o in options},
    }


def answers(
    x: str = "none",
    y: str = "none",
    yaw: str = "none",
    p: float = 1.0,
    stop: float = 0.0,
    task: str = "continue",
) -> Answers:
    return {
        "task": choice(task, "finished", "continue", p=p),
        "target": choice("chair", "chair", "none"),
        "drive.x": choice(x, "forward", "none", "backward", p=p),
        "drive.y": choice(y, "left", "none", "right", p=p),
        "drive.yaw": choice(yaw, "turn_left", "none", "turn_right", p=p),
        "stop": {"type": "noul", "noul": stop},
    }


def _decode(a: Answers):  # type: ignore[no-untyped-def]
    return decode(a, stop_threshold=0.7)


def test_questions_shape() -> None:
    assert set(questions(())) == {"drive.x", "drive.y", "drive.yaw", "stop", "task"}
    q = questions(("chair", "person"))
    assert q["target"]["type"] == "choice" and set(q["target"]["criteria"]) == {
        "chair",
        "person",
        "none",
    }
    assert set(q["drive.x"]["criteria"]) == {"forward", "none", "backward"}


def test_axes_compose() -> None:
    d = _decode(answers(x="forward", y="left"))
    assert (d.x, d.y, d.yaw, d.stop, d.labels, d.target) == (
        1.0,
        1.0,
        0.0,
        False,
        ("forward", "left", "none"),
        "chair",
    )


def test_low_probability_pick_still_counts() -> None:
    d = _decode(answers(x="forward", p=0.3))
    assert (d.x, d.labels[0], d.confidence) == (1.0, "forward", 0.3)


def test_stop_overrides_axes() -> None:
    d = _decode(answers(x="forward", yaw="turn_right", stop=0.9))
    assert d.stop and d.is_zero


def test_finished_stops_and_flags() -> None:
    d = _decode(answers(x="forward", task="finished"))
    assert d.finished and d.stop and d.is_zero
    assert _decode(answers(x="forward", task="finished", p=0.1)).finished  # the pick counts
    assert not _decode(answers(x="forward")).finished


def test_target_none_dropped() -> None:
    a = answers(x="forward")
    a["target"] = choice("none", "chair", "none")
    assert _decode(a).target is None


def test_undeclared_choice_reads_as_none() -> None:
    a = answers()
    a["drive.x"] = choice("sideways", "sideways", "forward", "none", "backward")
    d = _decode(a)
    assert (d.x, d.labels[0]) == (0.0, "none")


def test_min_probability_gates_an_axis_pick() -> None:
    d = decode(answers(x="forward", p=0.4), stop_threshold=0.7, min_probability=0.5)
    assert (d.x, d.labels[0]) == (0.0, "none")
    assert _decode(answers(x="forward", p=0.4)).x == 1.0  # off by default: the pick counts
