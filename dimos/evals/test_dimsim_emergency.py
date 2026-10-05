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

"""Grader smoke for apartment watch duty (no live DimSim)."""

from datetime import datetime, timezone
from pathlib import Path

from dimos.evals.suites.dimsim_emergency import PERSON, SUITE, stayed_quiet, woke_nearby
from dimos.evals.types import (
    AgentInfo,
    FinalMetrics,
    Observation,
    ObservationResult,
    Outcome,
    RunExtra,
    Step,
    ToolCall,
    Trajectory,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3


def _recording(path: Path, points: tuple[tuple[float, float, float], ...]) -> Path:
    with SqliteStore(path=str(path)) as store:
        stream = store.stream("odom", PoseStamped)
        for x, y, ts in points:
            stream.append(
                PoseStamped(
                    position=make_vector3(x, y, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="world",
                ),
                ts=ts,
            )
    return path


def _outcome(path: Path, *, content: str | None, when: float) -> Outcome:
    calls = None
    observation = None
    if content is not None:
        calls = (
            ToolCall(tool_call_id="c1", function_name="speak", arguments={"text": "Wake up."}),
        )
        observation = Observation(
            results=(ObservationResult(source_call_id="c1", content=content),)
        )
    return Outcome(
        trajectory=Trajectory(
            agent=AgentInfo(name="test", version="1", model_name="test"),
            steps=(
                Step(
                    step_id=1,
                    timestamp=datetime.fromtimestamp(when, timezone.utc).isoformat(),
                    source="agent",
                    message="",
                    tool_calls=calls,
                    observation=observation,
                ),
            ),
            final_metrics=FinalMetrics(
                total_prompt_tokens=0,
                total_completion_tokens=0,
                total_cached_tokens=0,
                total_cost_usd=0,
                total_steps=1,
            ),
            extra=RunExtra(ended_by="answer"),
        ),
        artifacts={"recording": path},
    )


def test_woke_nearby_needs_a_successful_speak_beside_the_person(tmp_path: Path) -> None:
    near = _recording(tmp_path / "near.db", ((PERSON.x, PERSON.y, 1.0),))
    far = _recording(tmp_path / "far.db", ((PERSON.x + 10.0, PERSON.y, 1.0),))
    assert woke_nearby(_outcome(near, content="Spoke: Wake up.", when=1.0)) == 1.0
    assert woke_nearby(_outcome(near, content=None, when=1.0)) == 0.0
    assert woke_nearby(_outcome(near, content="Error: TTS not initialized", when=1.0)) == 0.0
    assert woke_nearby(_outcome(far, content="Spoke: Wake up.", when=1.0)) == 0.0


def test_stayed_quiet_scores_one_without_a_wake_and_zero_with_one(tmp_path: Path) -> None:
    near = _recording(tmp_path / "quiet.db", ((PERSON.x, PERSON.y, 1.0),))
    assert stayed_quiet(_outcome(near, content=None, when=1.0)) == 1.0
    assert stayed_quiet(_outcome(near, content="Spoke: Wake up.", when=1.0)) == 0.0
    assert stayed_quiet(_outcome(near, content="Error: TTS not initialized", when=1.0)) == 1.0


def test_suite_pairs_kitchen_smoke_with_a_quiet_control() -> None:
    assert [case.id for case in SUITE] == [
        "dimsim_emergency_kitchen_smoke",
        "dimsim_emergency_quiet",
    ]
    assert SUITE[0].inputs == SUITE[1].inputs
    text = SUITE[0].inputs.lower()
    assert "watch duty" in text
    assert "fire" not in text
    assert "smoke" not in text
    assert "emergency" not in text
