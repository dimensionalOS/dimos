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
from dimos.agents.typesafe.client import Answer, Answers
from dimos.agents.typesafe.drive import decode, questions


def _choice(label: str, *options: str, confidence: float = 0.9) -> Answer:
    return {
        "type": "choice",
        "choice": label,
        "confidence": confidence,
        "probabilities": {o: float(o == label) for o in options},
    }


def _answers(
    x: str = "none",
    y: str = "none",
    yaw: str = "none",
    conf: float = 0.9,
    stop: float = 0.0,
    task: str = "continue",
) -> Answers:
    return {
        "drive.x": _choice(x, "forward", "none", "backward", confidence=conf),
        "drive.y": _choice(y, "left", "none", "right", confidence=conf),
        "drive.yaw": _choice(yaw, "turn_left", "none", "turn_right", confidence=conf),
        "stop": {"type": "noul", "noul": stop},
        "task": _choice(task, "finished", "continue", confidence=conf),
    }


def _decode(answers: Answers):  # type: ignore[no-untyped-def]
    return decode(answers, stop_threshold=0.7)


def test_questions_one_choice_per_axis_plus_stop_and_target() -> None:
    assert set(questions(())) == {"drive.x", "drive.y", "drive.yaw", "stop", "task"}
    q = questions(("chair", "person"))
    assert q["target"]["type"] == "choice" and set(q["target"]["criteria"]) == {
        "chair",
        "person",
        "none",
    }


def test_forward_and_strafe_compose() -> None:
    d = _decode(_answers(x="forward", y="left"))
    assert (d.x, d.y, d.yaw, d.stop, d.labels) == (
        1.0,
        1.0,
        0.0,
        False,
        ("forward", "left", "none"),
    )


def test_low_confidence_pick_still_counts() -> None:
    d = _decode(_answers(x="forward", conf=0.3))
    assert (d.x, d.labels[0], d.confidence) == (1.0, "forward", 0.3)


def test_stop_overrides_axes() -> None:
    d = _decode(_answers(x="forward", yaw="turn_right", stop=0.9))
    assert d.stop and d.is_zero


def test_finished_stops_and_flags() -> None:
    d = _decode(_answers(x="forward", task="finished"))
    assert d.finished and d.stop and d.is_zero
    assert _decode(_answers(x="forward", task="finished", conf=0.1)).finished  # the pick counts
    assert not _decode(_answers(x="forward")).finished


def test_target_decoded_and_none_dropped() -> None:
    a = _answers(x="forward")
    a["target"] = _choice("chair", "chair", "none")
    assert _decode(a).target == "chair"
    a["target"] = _choice("none", "chair", "none")
    assert _decode(a).target is None
