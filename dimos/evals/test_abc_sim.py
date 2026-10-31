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

from pathlib import Path

from dimos.evals.scorers import count_inside, placed_beside
from dimos.evals.suites.abc_sim import COUNT_BOX, SUITE
from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

Pose = tuple[float, float, float]


def _outcome(tmp_path: Path, name: str, frames: list[dict[str, Pose]]) -> Outcome:
    """A recording with one body_poses message per frame of ``{body: xyz}``."""
    db = tmp_path / f"{name}.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("body_poses", TFMessage)
        for i, frame in enumerate(frames):
            stream.append(
                TFMessage(
                    *(
                        Transform(
                            translation=Vector3(*xyz),
                            rotation=Quaternion(0, 0, 0, 1),
                            frame_id="world",
                            child_frame_id=body,
                            ts=float(i),
                        )
                        for body, xyz in frame.items()
                    )
                ),
                ts=float(i),
            )
    # These graders read only the recording.
    return Outcome(trajectory=None, artifacts={"recording": db})  # type: ignore[arg-type]


def test_count_inside_needs_exactly_the_wanted_number(tmp_path: Path) -> None:
    grade = count_inside(("a", "b", "c"), "box", COUNT_BOX, want=2)
    box = (0.70, 0.0, 0.75)
    inside, outside = (0.70, 0.05, 0.80), (0.45, -0.3, 0.776)
    on_lid = (0.70, 0.0, 0.96)  # above the lid underside does not count

    def score(name: str, a: Pose, b: Pose, c: Pose) -> float:
        return grade(_outcome(tmp_path, name, [{"box": box, "a": a, "b": b, "c": c}]))

    assert score("two", inside, inside, outside) == 1.0
    assert score("one", inside, outside, on_lid) == 0.0
    assert score("three", inside, inside, inside) == 0.0


def test_placed_beside_is_directional_and_placed(tmp_path: Path) -> None:
    grade = placed_beside("m", "r", (0.0, 1.0))
    ref, start = (0.44, -0.24, 0.775), (0.64, -0.24, 0.775)

    def score(name: str, end: Pose) -> float:
        return grade(_outcome(tmp_path, name, [{"m": start, "r": ref}, {"m": end, "r": ref}]))

    assert score("left", (0.46, -0.12, 0.775)) == 1.0
    assert score("right", (0.46, -0.36, 0.775)) == 0.0, "wrong side"
    assert score("held", (0.46, -0.12, 0.90)) == 0.0, "still carried"
    assert score("touching", (0.44, -0.21, 0.775)) == 0.0, "too close"


def test_suite_builds() -> None:
    assert {case.id for case in SUITE} == {"abc_count_two", "abc_grab_clutter", "abc_put_relative"}
